#!/usr/bin/env python3
"""End-to-end check of the ESC emulators against the SITL.

    python3 src/utils/sitl/esc/esc_check.py obj/main/rotorflight_SITL.elf [protocol ...]

For each protocol: configure the ESC sensor on UART3 (TCP 5763) over the CLI, start the
SITL with a level FDM feed, run `esc_sim.py` on the ESC UART, then over MSP

  1. wait for the ESC's parameter block (MSP_ESC_PARAMETERS): the firmware read it
  2. check its telemetry arrived (MSP_MOTOR_TELEMETRY: voltage, current)
  3. write the block back (MSP_SET_ESC_PARAMETERS) with one setting changed and wait for
     the emulator to report the write: the programming path the Configurator's ESC tab uses

Exit status 0 when every protocol passes.
"""

import importlib.util
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "smoke", os.path.join(HERE, "..", "..", "sitl-smoke-test.py"))
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)

MSP_MOTOR_TELEMETRY = 139
MSP_ESC_PARAMETERS = 217
MSP_SET_ESC_PARAMETERS = 218

# esc_sim protocol -> (esc_sensor_protocol, half duplex)
PROTOCOLS = {
    "openyge": ("OPENYGE", True),
    "hobbywingv5": ("HOBBYWINGV5", True),
    "scorpion": ("SCORPION", True),
    "flyrotor": ("FLYROTOR", True),
    "xdfly": ("XDFLY", True),
    "omphobby": ("OMPHOBBY", True),
    "ztw": ("ZTW", True),
}


def msp_try(msp, cmd, payload=b""):
    try:
        return msp.request(cmd, payload)
    except Exception:                       # an MSP error reply: not (yet) available
        return None


def check(elf, protocol):
    fw_name, half = PROTOCOLS[protocol]
    rundir = tempfile.mkdtemp(prefix="sitl-esc-%s-" % protocol)
    s = smoke.Sitl(elf, rundir, "1-first"); s.wait(30); s.stop()
    s = smoke.Sitl(elf, rundir, "2-config")
    sock = smoke.connect_uart1(s)
    smoke.cli(sock, ["feature ESC_SENSOR", "serial 2 1024 115200 57600 0 115200",
                     "set esc_sensor_protocol = %s" % fw_name,
                     "set esc_sensor_halfduplex = %s" % ("ON" if half else "OFF"),
                     "save"])
    sock.close(); s.wait(15); s.stop()

    feed = smoke.FdmFeed(); feed.start()
    s = smoke.Sitl(elf, rundir, "3-esc")
    out = open(os.path.join(rundir, "esc_sim.log"), "w")
    esc = None
    try:
        msp = smoke.Msp(smoke.connect_uart1(s))
        deadline = time.time() + 15
        while True:                         # the ESC UART listens once the SITL is up
            try:
                socket.create_connection(("127.0.0.1", 5763), timeout=1).close()
                break
            except OSError:
                if time.time() > deadline:
                    return False, "ESC UART 5763 not served"
                time.sleep(0.2)
        time.sleep(0.5)
        esc = subprocess.Popen([sys.executable, os.path.join(HERE, "esc_sim.py"), "--protocol",
                                protocol, "--port", "5763", "--seconds", "90"],
                               stdout=out, stderr=subprocess.STDOUT)
        params, deadline = None, time.time() + 45
        while time.time() < deadline and not params:
            params = msp_try(msp, MSP_ESC_PARAMETERS)
            time.sleep(0.5)
        if not params:
            return False, "no parameter block within 45 s"
        # telemetry: some ESCs stream only once the parameter dialogue has gone quiet
        volt = curr = 0
        deadline = time.time() + 15
        while time.time() < deadline and not volt:
            tel = msp.request(MSP_MOTOR_TELEMETRY)
            _rpm, _err, volt, curr = struct.unpack_from("<IHHH", tel, 1)
            if not volt:
                time.sleep(0.5)
        # change one byte near the end of the block (settings, not the header/version
        # fields at its start) and write it back
        new = bytearray(params)
        new[-2] ^= 0x01
        # header byte 1: version in the low 6 bits, command in the top 2. A block read
        # back may carry capability bits there (Scorpion: 0x80, "can reset"); written
        # back unchanged that is a command, not a save. Clear them to save.
        new[1] &= 0x3F
        msp_try(msp, MSP_SET_ESC_PARAMETERS, bytes(new))
        wrote, deadline = False, time.time() + 30
        while time.time() < deadline and not wrote:
            time.sleep(0.5)
            out.flush()
            with open(out.name) as f:
                wrote = "WRITE #" in f.read()
        detail = "%d-byte block, telemetry %.2f V %.2f A, write %s" % (
            len(params), volt / 1000, curr / 1000, "reached the ESC" if wrote else "NOT seen")
        return wrote and volt > 0, detail
    finally:
        feed.running = False
        if esc:
            esc.terminate()
        s.stop()
        out.close()


def main():
    elf = os.path.abspath(sys.argv[1])
    protocols = sys.argv[2:] or list(PROTOCOLS)
    failed = 0
    for p in protocols:
        ok, detail = check(elf, p)
        print("%s %-12s %s" % ("PASS" if ok else "FAIL", p, detail), flush=True)
        failed += not ok
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
