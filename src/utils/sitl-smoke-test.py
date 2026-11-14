#!/usr/bin/env python3
"""
SITL smoke test: build output in, pass/fail out.

    python3 src/utils/sitl-smoke-test.py obj/main/rotorflight_SITL.elf

Runs the SITL executable in a fresh temporary directory and checks, in order:

  1. first start: no eeprom.bin yet, defaults are written, the process exits 0
  2. MSP on TCP 5761: FC variant RTFL, board SITL
  3. CLI: an ARM switch on AUX1 is set and saved (the save ends the process)
  4. with a level FDM feed on UDP 9003 and RC frames over MSP RX:
     the accelerometer calibrates, all arming blockers clear, the switch arms,
     full throttle drives motor 1 on the UDP 9002 output, the switch disarms
  5. CRSF on UART2 (TCP 5762), configured through the CLI: RC frames clear RXLOSS,
     the firmware's CRSF telemetry arrives with valid CRCs, the CRSF switch arms
     and disarms
  6. with FDM packets sent in bursts (a stalling sender), the firmware clock
     still follows real time
  7. lockstep (SITL_LOCKSTEP=1): each 250 us FDM step gets one output packet,
     and the firmware clock advances exactly with the simulated time; on 200 us
     steps the gyro loop still runs at 4 kHz, not once per step

Uses only the Python standard library. Ports 5761, 5762, 9002 and 9003 must be free.
SITL_LOOP_SLEEP_US is passed through to the SITL (see target/SITL/target.c).
"""

import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_RTC = 247

ARMING_FLAG_NAMES = [
    "NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "GOVERNOR", "RPM_SIGNAL",
    "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI", "CMS", "BST",
    "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD", "DSHOT_BBANG",
    "NO_ACC_CAL", "MOTOR_PROTO", "OVERRIDE", "ARMSWITCH",
]

# Default channel map AETRC123: roll, pitch, throttle, yaw, collective, aux1, aux2, aux3
def rc_frame(throttle, aux1):
    return [1500, 1500, throttle, 1500, 1500, aux1, 1000, 1000]


class Failure(Exception):
    pass


def log(msg):
    print(msg, flush=True)


class Sitl:
    """One SITL process in the run directory, its output in a log file."""

    def __init__(self, elf, rundir, name):
        self.log_path = os.path.join(rundir, name + ".log")
        self.log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen([elf], cwd=rundir, stdout=self.log_file,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)

    def wait(self, timeout):
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            if self.wait(5) is None:
                self.proc.kill()
                self.proc.wait()
        self.log_file.close()

    def log_contains(self, text):
        with open(self.log_path, errors="replace") as f:
            return text in f.read()


def connect_uart1(sitl, timeout=15):
    """Connect once the firmware itself has accepted UART1, not merely a TCP port."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if sitl.proc.poll() is not None:
            raise Failure("SITL exited before serving UART1 (see %s)" % sitl.log_path)
        if sitl.log_contains("bind port 5761"):
            try:
                return socket.create_connection(("127.0.0.1", 5761), timeout=3)
            except OSError:
                pass
        time.sleep(0.2)
    raise Failure("UART1 (TCP 5761) not served within %d s" % timeout)


class Msp:
    def __init__(self, sock):
        self.sock = sock

    def request(self, cmd, payload=b""):
        frame = b"$M<" + bytes([len(payload), cmd]) + payload
        checksum = 0
        for b in frame[3:]:
            checksum ^= b
        self.sock.sendall(frame + bytes([checksum]))
        buf = b""
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(512)
            except socket.timeout:
                continue
            if not chunk:
                raise Failure("MSP connection closed")
            buf += chunk
            i = buf.find(b"$M")
            if i >= 0 and len(buf) >= i + 5:
                size = buf[i + 3]
                if len(buf) >= i + 6 + size:
                    if buf[i + 2:i + 3] == b"!":
                        raise Failure("MSP command %d rejected" % cmd)
                    return buf[i + 5:i + 5 + size]
        raise Failure("no MSP reply to command %d within 3 s" % cmd)

    def status(self):
        st = self.request(MSP_STATUS)
        modes = struct.unpack_from("<I", st, 6)[0]
        flags = struct.unpack_from("<I", st, 17)[0]
        names = [ARMING_FLAG_NAMES[i] if i < len(ARMING_FLAG_NAMES) else str(i)
                 for i in range(32) if flags >> i & 1]
        return bool(modes & 1), names

    def rc(self, channels):
        self.request(MSP_SET_RAW_RC, struct.pack("<%dH" % len(channels), *channels))


def cli(sock, commands):
    """Enter the CLI and send commands; returns the text received."""
    out = b""

    def drain(seconds):
        nonlocal out
        end = time.time() + seconds
        sock.settimeout(0.2)
        while time.time() < end:
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            out += data

    sock.sendall(b"#\r\n")
    drain(1.0)
    for c in commands:
        sock.sendall(c.encode() + b"\r\n")
        drain(1.5)
    return out.decode(errors="replace")


class FdmFeed(threading.Thread):
    """Level, motionless FDM packets (136-byte fdm_packet) on UDP 9003 at a steady rate.

    With burst > 1, packets go out `burst` at a time, back to back, as from a sender
    that stalls and catches up; the average rate and the timestamps stay the same.
    """

    def __init__(self, rate=200.0, burst=1):
        super().__init__(daemon=True)
        self.rate = rate
        self.burst = burst
        self.running = True

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0 = time.monotonic()
        n = 0
        while self.running:
            for _ in range(self.burst):
                ts = n / self.rate
                # timestamp, gyro rpy, accel xyz (NED body, 1 g up), quat wxyz, velocity, position
                pkt = struct.pack("<17d", ts, 0, 0, 0, 0, 0, -9.80665, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                s.sendto(pkt, ("127.0.0.1", 9003))
                n += 1
            delay = t0 + n / self.rate - time.monotonic()
            if delay > 0:
                time.sleep(delay)


class OutputReader(threading.Thread):
    """Latest output packet from UDP 9002: motor_speed[4], then servo[8] (us)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 9002))
        self.sock.settimeout(0.1)
        self.latest = None
        self.count = 0
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            self.latest = data
            self.count += 1
        self.sock.close()       # frees UDP 9002 for a later stage

    def motors(self):
        data = self.latest
        return struct.unpack_from("<4f", data) if data and len(data) >= 16 else None


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crsf_rc_frame(channels_us):
    """Up to 16 channels in microseconds -> CRSF RC_CHANNELS_PACKED frame."""
    bits = 0
    for i, us in enumerate(channels_us + [1500] * (16 - len(channels_us))):
        ticks = max(172, min(1811, round((us - 1500) * 8 / 5 + 992)))
        bits |= ticks << (11 * i)
    body = bytes([0x16]) + bits.to_bytes(22, "little")
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8_dvb_s2(body)])


class CrsfRadio(threading.Thread):
    """A radio on the CRSF UART: RC frames at 50 Hz out, telemetry frames in."""

    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock = sock
        self.sock.settimeout(0.005)
        self.channels = rc_frame(1000, 1000)
        self.frames = 0
        self.bad_crc = 0
        self.running = True

    def run(self):
        buf = b""
        next_tx = time.monotonic()
        while self.running:
            try:
                if time.monotonic() >= next_tx:
                    self.sock.sendall(crsf_rc_frame(self.channels))
                    next_tx += 0.02
                data = self.sock.recv(4096)
                if not data:
                    return
                buf += data
            except socket.timeout:
                pass
            except OSError:
                return
            while len(buf) >= 2:
                size = buf[1]
                if size < 2 or size > 62:
                    buf = buf[1:]
                    continue
                if len(buf) < size + 2:
                    break
                frame, buf = buf[:size + 2], buf[size + 2:]
                if crc8_dvb_s2(frame[2:-1]) == frame[-1]:
                    self.frames += 1
                else:
                    self.bad_crc += 1


def hold(msp, throttle, aux1, seconds):
    end = time.time() + seconds
    while time.time() < end:
        msp.rc(rc_frame(throttle, aux1))
        time.sleep(0.02)
    return msp.status()


def run(elf, rundir):
    # 1. first start writes the defaults and exits
    sitl = Sitl(elf, rundir, "1-first-start")
    rc = sitl.wait(30)
    sitl.stop()
    eeprom = os.path.join(rundir, "eeprom.bin")
    if rc != 0 or not os.path.isfile(eeprom):
        raise Failure("first start: exit %s, eeprom.bin %s" % (rc, "written" if os.path.isfile(eeprom) else "missing"))
    log("PASS first start: exit 0, eeprom.bin %d bytes" % os.path.getsize(eeprom))

    # 2. identity, 3. ARM switch on AUX1 via the CLI
    sitl = Sitl(elf, rundir, "2-config")
    try:
        sock = connect_uart1(sitl)
        msp = Msp(sock)
        api = msp.request(MSP_API_VERSION)
        variant = msp.request(MSP_FC_VARIANT)[:4]
        board = msp.request(MSP_BOARD_INFO)[:4]
        if variant != b"RTFL" or board != b"SITL":
            raise Failure("identity: variant %r board %r" % (variant, board))
        log("PASS MSP identity: %s, API %d.%d, board %s" % (variant.decode(), api[1], api[2], board.decode()))

        out = cli(sock, ["aux 0 0 0 1700 2100 0 0", "save"])
        sock.close()
        rc = sitl.wait(15)
        if rc != 0:
            raise Failure("CLI save did not end the process with exit 0 (exit %s)\n%s" % (rc, out))
        log("PASS CLI: ARM on AUX1 saved, process exited 0")
    finally:
        sitl.stop()

    # 4. arm and disarm with a level FDM feed
    outputs = OutputReader()
    outputs.start()
    sitl = Sitl(elf, rundir, "3-arm")
    feed = FdmFeed()
    try:
        feed.start()
        sock = connect_uart1(sitl)
        msp = Msp(sock)
        msp.request(MSP_ACC_CALIBRATION)

        deadline = time.time() + 30   # the baro calibrates for 12.5 s, as on a board
        armed, flags = hold(msp, 1000, 1000, 0.5)
        seen = None
        while flags and time.time() < deadline:
            if flags != seen:
                log("     waiting on %s" % flags)
                seen = flags
            armed, flags = hold(msp, 1000, 1000, 0.2)
        if flags:
            raise Failure("arming blockers did not clear within 30 s: %s" % flags)
        log("PASS arming blockers clear (acc calibrated, RX up)")

        armed, flags = hold(msp, 1000, 1900, 1.0)
        if not armed:
            raise Failure("switch on, not armed: %s" % flags)
        log("PASS armed")

        armed, flags = hold(msp, 2000, 1900, 1.5)
        motors = outputs.motors()
        if not armed or not motors or motors[0] < 0.9:
            raise Failure("full throttle: armed=%s motors=%s" % (armed, motors))
        log("PASS full throttle: motor 1 = %.3f, %d output packets of %d bytes"
            % (motors[0], outputs.count, len(outputs.latest)))

        armed, flags = hold(msp, 1000, 1000, 1.0)
        motors = outputs.motors()
        if armed or not motors or motors[0] != 0:
            raise Failure("switch off: armed=%s motors=%s" % (armed, motors))
        log("PASS disarmed, motor 1 = 0")
        sock.close()
    finally:
        feed.running = False
        outputs.running = False
        outputs.join(2)
        sitl.stop()

    # 5. CRSF receiver and telemetry on UART2
    sitl = Sitl(elf, rundir, "4-crsf-config")
    try:
        sock = connect_uart1(sitl)
        out = cli(sock, ["feature -RX_MSP", "feature RX_SERIAL", "feature TELEMETRY",
                         "serial 1 64 115200 57600 0 115200", "set serialrx_provider = CRSF", "save"])
        sock.close()
        if sitl.wait(15) != 0:
            raise Failure("CRSF configuration was not saved\n%s" % out)
    finally:
        sitl.stop()

    feed = FdmFeed()
    sitl = Sitl(elf, rundir, "5-crsf")
    radio = None
    try:
        feed.start()
        msp = Msp(connect_uart1(sitl))
        deadline = time.time() + 10
        while radio is None:
            try:
                radio = CrsfRadio(socket.create_connection(("127.0.0.1", 5762), timeout=1))
            except OSError:
                if time.time() > deadline:
                    raise Failure("CRSF UART2 (TCP 5762) not served")
                time.sleep(0.2)
        radio.start()

        deadline = time.time() + 30   # the baro calibrates for 12.5 s, as on a board
        armed, flags = msp.status()
        seen = None
        while flags and time.time() < deadline:
            if flags != seen:
                log("     waiting on %s" % flags)
                seen = flags
            time.sleep(0.2)
            armed, flags = msp.status()
        if flags:
            raise Failure("CRSF: arming blockers did not clear within 30 s: %s" % flags)
        if not radio.frames or radio.bad_crc:
            raise Failure("CRSF telemetry: %d frames, %d with a bad CRC" % (radio.frames, radio.bad_crc))
        log("PASS CRSF: RX up, %d telemetry frames, no CRC errors" % radio.frames)

        radio.channels = rc_frame(1000, 1900)
        time.sleep(1.0)
        armed, flags = msp.status()
        if not armed:
            raise Failure("CRSF switch on, not armed: %s" % flags)
        radio.channels = rc_frame(1000, 1000)
        time.sleep(1.0)
        armed, flags = msp.status()
        if armed:
            raise Failure("CRSF switch off, still armed")
        log("PASS CRSF: armed and disarmed with the CRSF switch")
    finally:
        feed.running = False
        if radio:
            radio.running = False
        sitl.stop()

    # 6. the firmware clock follows real time under a bursty FDM feed
    feed = FdmFeed(burst=4)
    sitl = Sitl(elf, rundir, "6-clock")
    try:
        feed.start()
        sock = connect_uart1(sitl)
        t0 = time.time()
        time.sleep(10)
        out = cli(sock, ["status"])
        real = time.time() - t0
        sock.close()
        uptime = [int(w) for line in out.splitlines() if "System Uptime" in line
                  for w in line.replace(",", " ").split() if w.isdigit()]
        if not uptime or abs(uptime[0] - real) > 4:
            raise Failure("clock under a bursty feed: firmware uptime %s s after %.0f s" % (uptime, real))
        log("PASS clock: uptime %d s after %.0f s with packets sent 4 at a time" % (uptime[0], real))
    finally:
        feed.running = False
        sitl.stop()

    # 7. lockstep: one FDM step, one output packet, firmware time = sim time
    lockstep(elf, rundir)
    lockstep(elf, rundir, dt=200e-6)


GYRO_PERIOD_US = 250    # the fake gyro's 4 kHz


def lockstep(elf, rundir, sim_seconds=6.0, dt=250e-6):
    """SITL_LOCKSTEP=1: step the FDM by dt, wait for the output packet, repeat.
    MSP_RTC, read at two sim times while stepping, must advance by the sim time.
    MSP_STATUS, read throughout, gives the gyro task's latest interval: on steps
    shorter than the gyro period it is one or two steps, averaging the period."""
    outputs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    outputs.bind(("127.0.0.1", 9002))
    outputs.settimeout(0.05)
    fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    env_before = os.environ.get("SITL_LOCKSTEP")
    os.environ["SITL_LOCKSTEP"] = "1"
    sitl = Sitl(elf, rundir, "7-lockstep")
    try:
        sock = connect_uart1(sitl)
        sock.setblocking(False)
        steps = int(sim_seconds / dt)
        ask_at = {int(1.0 / dt), int(5.0 / dt)}
        timeouts, rx, rtc, asked = 0, b"", [], None
        gyro_us, status_pending = [], False
        t0 = time.time()
        for n in range(steps):
            ts = n * dt
            if n in ask_at:
                sock.send(b"$M<" + bytes([0, MSP_RTC, MSP_RTC]))
                asked = ts
            elif not status_pending and n % 50 == 0 and n * dt >= 1.0:
                sock.send(b"$M<" + bytes([0, MSP_STATUS, MSP_STATUS]))
                status_pending = True
            fdm.sendto(struct.pack("<17d", ts, 0, 0, 0, 0, 0, -9.80665, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                       ("127.0.0.1", 9003))
            try:
                outputs.recvfrom(256)
            except socket.timeout:
                timeouts += 1
            try:
                rx += sock.recv(512)
            except (BlockingIOError, socket.timeout):
                pass
            while True:
                i = rx.find(b"$M>")
                if i < 0 or len(rx) < i + 6 + rx[i + 3]:
                    break
                payload = rx[i + 5:i + 5 + rx[i + 3]]
                if rx[i + 4] == MSP_RTC and len(payload) >= 9:
                    _, _, _, h, mi, sec, ms = struct.unpack("<HBBBBBH", payload[:9])
                    rtc.append((asked, ts, ((h * 60 + mi) * 60 + sec) * 1000 + ms))
                elif rx[i + 4] == MSP_STATUS and len(payload) >= 4:
                    gyro_us.append(struct.unpack_from("<H", payload, 2)[0])
                    status_pending = False
                rx = rx[i + 6 + rx[i + 3]:]
        real = time.time() - t0
        if timeouts > steps // 100:
            raise Failure("lockstep: %d of %d steps got no output packet" % (timeouts, steps))
        if len(rtc) != 2:
            raise Failure("lockstep: MSP_RTC replies %s" % rtc)
        # Each reply was made somewhere between the step that sent the request and the
        # step that received it. TCP is polled on real time, so running faster than real
        # time a reply lags by tens of steps, by hundreds on Windows. The firmware's
        # advance must lie within those bounds, +-2 ms for the RTC's ms resolution.
        (ask1, got1, fw1), (ask2, got2, fw2) = rtc
        dfw = fw2 - fw1
        lo, hi = (ask2 - got1) * 1000 - 2, (got2 - ask1) * 1000 + 2
        dsim = (ask2 - ask1) * 1000
        if not lo <= dfw <= hi or max(got1 - ask1, got2 - ask2) > 1.0:
            raise Failure("lockstep: firmware advanced %d ms, sim %.1f..%.1f ms" % (dfw, lo + 2, hi - 2))
        # The gyro runs on the first step at or after each 250 us boundary, so its
        # intervals are whole steps averaging the gyro period: with 200 us steps three
        # of 200 and one of 400 (a step-sampled mean of 240), never 200 throughout.
        if len(gyro_us) < 10:
            raise Failure("lockstep: only %d MSP_STATUS replies" % len(gyro_us))
        step_us = dt * 1e6
        gyro_mean = sum(gyro_us) / len(gyro_us)
        expect = GYRO_PERIOD_US if step_us >= GYRO_PERIOD_US else 0.96 * GYRO_PERIOD_US
        if abs(gyro_mean - expect) > 0.06 * GYRO_PERIOD_US:
            raise Failure("lockstep: gyro interval %.0f us on %.0f us steps (expected ~%.0f): %s"
                          % (gyro_mean, step_us, expect, sorted(set(gyro_us))))
        log("PASS lockstep: %d steps of %.0f us at %.1fx real time, %d without reply; firmware %d ms "
            "over %.1f ms sim; gyro interval %.0f us"
            % (steps, step_us, sim_seconds / real, timeouts, dfw, dsim, gyro_mean))
    finally:
        if env_before is None:
            os.environ.pop("SITL_LOCKSTEP", None)
        else:
            os.environ["SITL_LOCKSTEP"] = env_before
        sitl.stop()


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: %s <rotorflight_SITL.elf>" % sys.argv[0])
    elf = os.path.abspath(sys.argv[1])
    rundir = tempfile.mkdtemp(prefix="sitl-smoke-")
    try:
        run(elf, rundir)
    except Failure as e:
        log("FAIL %s" % e)
        for name in sorted(os.listdir(rundir)):
            if name.endswith(".log"):
                with open(os.path.join(rundir, name), errors="replace") as f:
                    lines = [l for l in f.read().splitlines() if "FLASH_ProgramWord" not in l]
                log("--- %s (last 30 lines)\n%s" % (name, "\n".join(lines[-30:])))
        sys.exit(1)
    finally:
        shutil.rmtree(rundir, ignore_errors=True)
    log("SITL smoke test passed")


if __name__ == "__main__":
    main()
