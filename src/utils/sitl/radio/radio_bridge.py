#!/usr/bin/env python3
"""Radio in the loop: the EdgeTX simulator on one end of a CRSF link, the SITL on the other.

    python3 src/utils/sitl/radio/radio_bridge.py --bindir <companion lib dir> --sd <sd card> \
            [--radio tx16smk3] [--port 5762] [--seconds 0]

The radio is EdgeTX itself, loaded from Companion's simulator library (`edgetx_sim.py`),
with its own SD card - radio settings, a model, Lua scripts. Its mixer outputs go to the
SITL as CRSF RC frames at 50 Hz; the SITL's CRSF telemetry frames go back into the radio's
internal module, where EdgeTX and its Lua scripts receive them as from a real receiver.
So a Lua suite on the SD card talks to the simulated flight controller over the same
CRSF/MSP path it uses with a real one.

The SITL side is a CRSF receiver on a TCP UART:

    feature -RX_MSP
    feature RX_SERIAL
    feature TELEMETRY
    serial 1 64 115200 57600 0 115200        # UART2 = TCP 5762
    set serialrx_provider = CRSF

`map` must match the model's channel order (EdgeTX default: Rud, Ele, Thr, Ail ->
`map RETA1C23`).

Linux: the simulator library is in Companion's AppImage (`--appimage-extract`,
`usr/lib/companion212`); run with `LD_LIBRARY_PATH=<squashfs-root>/usr/lib`. Under WSL1,
strip `.note.ABI-tag` from that `libQt6Core.so.6` (it asks for a newer kernel than WSL1
reports). Windows: Companion's `bin` directory; the binding makes sure the process uses a
C++ runtime (`msvcp140.dll`) of 14.40 or newer, which the library needs.
"""

import argparse
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from edgetx_sim import Sim, settle      # noqa: E402

CRSF_SYNC = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def edgetx_to_us(value):
    """EdgeTX channel units (+-1024 = +-100 %) -> microseconds (1000..2000 at +-100 %)."""
    return 1500 + value * 500 / 1024


def crsf_rc_frame(channels_us):
    """16 channels in microseconds -> CRSF RC_CHANNELS_PACKED (11 bits each)."""
    bits, nbits, payload = 0, 0, bytearray()
    for us in (list(channels_us) + [1500] * 16)[:16]:
        ticks = int(round((us - 1500) * 8 / 5 + 992))
        bits |= max(0, min(2047, ticks)) << nbits
        nbits += 11
        while nbits >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            nbits -= 8
    body = bytes([CRSF_FRAMETYPE_RC_CHANNELS_PACKED]) + bytes(payload)
    return bytes([CRSF_SYNC, len(body) + 1]) + body + bytes([crc8_dvb_s2(body)])


class RadioBridge(threading.Thread):
    """Channels out, telemetry in, between a running `Sim` and a SITL CRSF UART."""

    def __init__(self, sim, host="127.0.0.1", port=5762, rate_hz=50.0):
        super().__init__(daemon=True)
        self.sim = sim
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(0.002)
        self.period = 1.0 / rate_hz
        self.running = True
        self.rc_frames = 0
        self.telemetry_frames = 0
        self.bad_crc = 0
        self.last_channels = []

    def run(self):
        buf = b""
        next_tx = time.monotonic()
        while self.running:
            now = time.monotonic()
            if now >= next_tx:
                self.last_channels = [edgetx_to_us(v) for v in self.sim.channels.read()[:16]]
                try:
                    self.sock.sendall(crsf_rc_frame(self.last_channels))
                except OSError:
                    return
                self.rc_frames += 1
                next_tx = max(now, next_tx + self.period)
            try:
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
                if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                    self.bad_crc += 1
                    continue
                self.telemetry_frames += 1
                self.sim.send_internal_telemetry(frame)

    def stop(self):
        self.running = False
        self.join(2)
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bindir", required=True, help="directory holding libedgetx-<radio>-simulator")
    ap.add_argument("--sd", required=True, help="the radio's SD card (a directory)")
    ap.add_argument("--radio", default="tx16smk3")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5762, help="the SITL's CRSF UART (5760 + n)")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until interrupted")
    args = ap.parse_args()

    sim = Sim(args.radio, bindir=args.bindir, sd_path=args.sd)
    sim.set_sd_path(args.sd)
    sim.start()
    if not sim.wait_running():
        sys.exit("the radio did not start")
    settle(1.0)
    bridge = RadioBridge(sim, args.host, args.port)
    bridge.start()
    print("radio %s on %s:%d" % (args.radio, args.host, args.port), flush=True)
    t0 = time.time()
    try:
        while not args.seconds or time.time() - t0 < args.seconds:
            time.sleep(1.0)
            print("rc %d out, telemetry %d in (%d bad CRC)"
                  % (bridge.rc_frames, bridge.telemetry_frames, bridge.bad_crc), flush=True)
    except KeyboardInterrupt:
        pass
    bridge.stop()
    sim.stop()


if __name__ == "__main__":
    main()
