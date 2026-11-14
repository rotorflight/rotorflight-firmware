#!/usr/bin/env python3
"""End-to-end check of the radio in the loop: EdgeTX simulator <-> CRSF <-> SITL.

    LD_LIBRARY_PATH=<squashfs-root>/usr/lib \
    python3 src/utils/sitl/radio/radio_check.py obj/main/rotorflight_SITL.elf \
            --bindir <squashfs-root>/usr/lib/companion212 --sd-template <suite>/simulator

The SD card is a copy of `--sd-template` (the EdgeTX Lua suite's `simulator/` folder: a
TX16S MK3 radio and a CRSF model with the four sticks on CH1-4) with one mix added: switch
SA on CH5, the arm switch. The SITL takes CRSF on UART2 with `map RETA1C23`, ARM on AUX1.

Checks, all driven from the simulated radio:
  1. the link: RXLOSS clears once the radio's RC frames arrive
  2. a stick: the aileron stick moves the firmware's roll input
  3. arming: throttle stick low, SA up -> armed; SA down -> disarmed
  4. telemetry: the firmware's CRSF frames reach the radio with valid CRCs
"""

import argparse
import importlib.util
import os
import re
import shutil
import struct
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
spec = importlib.util.spec_from_file_location("smoke", os.path.join(HERE, "..", "..", "sitl-smoke-test.py"))
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)

from edgetx_sim import Sim, settle          # noqa: E402
from radio_bridge import RadioBridge        # noqa: E402

MSP_RC = 105
# EdgeTX analog order on these radios: Rud, Ele, Thr, Ail
STICK_RUD, STICK_ELE, STICK_THR, STICK_AIL = 0, 1, 2, 3
SWITCH_SA = 0

ARM_MIX = """ -
   destCh: 4
   srcRaw: "SA"
   carryTrim: 0
   mixWarn: 0
   mltpx: ADD
   delayPrec: 0
   speedPrec: 0
   flightModes: 000000000
   weight: 100
   offset: 0
   swtch: "NONE"
   delayUp: 0
   delayDown: 0
   speedUp: 0
   speedDown: 0
   name: "ARM"
"""


def make_sd(template):
    sd = tempfile.mkdtemp(prefix="radio-sd-")
    shutil.copytree(template, sd, dirs_exist_ok=True)
    model = os.path.join(sd, "MODELS", "model1.yml")
    text = open(model, encoding="utf-8").read()
    head, sep, rest = text.partition("expoData:")
    if not sep:
        raise smoke.Failure("model1.yml has no expoData section to insert the mix before")
    open(model, "w", encoding="utf-8").write(head + ARM_MIX + sep + rest)
    # radio.yml carries a checksum over its exact bytes, so a checkout with other line
    # endings than the file was saved with (git on Linux: LF) reads as corrupt and the
    # radio comes up on an alert with its mixer stopped. manuallyEdited: 1 tells
    # EdgeTX not to check it.
    radio = os.path.join(sd, "RADIO", "radio.yml")
    with open(radio, encoding="utf-8", newline="") as f:
        text = f.read()
    text, n = re.subn(r"(?m)^manuallyEdited:[ \t]*\d+", "manuallyEdited: 1", text)
    if not n:
        raise smoke.Failure("radio.yml has no manuallyEdited field")
    with open(radio, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    return sd


def wait(cond, seconds, step=0.1):
    end = time.time() + seconds
    while time.time() < end:
        value = cond()
        if value:
            return value
        time.sleep(step)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("elf")
    ap.add_argument("--bindir", required=True)
    ap.add_argument("--sd-template", required=True)
    ap.add_argument("--radio", default="tx16smk3")
    a = ap.parse_args()
    elf = os.path.abspath(a.elf)

    rundir = tempfile.mkdtemp(prefix="sitl-radio-")
    s = smoke.Sitl(elf, rundir, "1-first"); s.wait(30); s.stop()
    s = smoke.Sitl(elf, rundir, "2-config")
    try:
        sock = smoke.connect_uart1(s)
        smoke.cli(sock, ["feature -RX_MSP", "feature RX_SERIAL", "feature TELEMETRY",
                         "serial 1 64 115200 57600 0 115200", "set serialrx_provider = CRSF",
                         "map RETA1C23", "aux 0 0 0 1700 2100 0 0", "save"])
        sock.close(); s.wait(15)
    finally:
        s.stop()        # a timeout above must not leave this SITL holding the ports

    sim = Sim(a.radio, bindir=a.bindir)
    sim.set_sd_path(make_sd(a.sd_template))
    sim.start()
    if not sim.wait_running():
        raise smoke.Failure("the EdgeTX simulator did not start")
    settle(1.0)
    sim.analog(STICK_THR, -1024)
    sim.switch(SWITCH_SA, -1)
    settle(0.3)

    feed = smoke.FdmFeed(); feed.start()
    s = smoke.Sitl(elf, rundir, "3-radio")
    bridge = None
    try:
        msp = smoke.Msp(smoke.connect_uart1(s))
        bridge = RadioBridge(sim, port=smoke.uart_port(2))
        bridge.start()
        msp.request(smoke.MSP_ACC_CALIBRATION)

        flags = wait(lambda: (lambda st: None if st[1] else ["clear"])(msp.status()), 30)
        if not flags:
            raise smoke.Failure("arming blockers did not clear: %s" % msp.status()[1])
        smoke.log("PASS link: the radio's CRSF frames clear RXLOSS (%d RC frames)" % bridge.rc_frames)

        def roll():
            return struct.unpack_from("<H", msp.request(MSP_RC), 0)[0]
        sim.analog(STICK_AIL, 1024); settle(0.5); right = roll()
        sim.analog(STICK_AIL, -1024); settle(0.5); left = roll()
        sim.analog(STICK_AIL, 0); settle(0.5)
        if not (right > 1900 and left < 1100):
            raise smoke.Failure("aileron stick: roll input %d at full right, %d at full left" % (right, left))
        smoke.log("PASS stick: aileron full right/left -> roll input %d / %d us" % (right, left))

        sim.switch(SWITCH_SA, 1)
        if not wait(lambda: msp.status()[0], 5):
            raise smoke.Failure("SA up did not arm: %s" % msp.status()[1])
        sim.switch(SWITCH_SA, -1)
        if wait(lambda: not msp.status()[0], 5) is None:
            raise smoke.Failure("SA down did not disarm")
        smoke.log("PASS arming: SA up armed, SA down disarmed")

        before = bridge.telemetry_frames
        time.sleep(3)
        got = bridge.telemetry_frames - before
        if got < 5 or bridge.bad_crc:
            raise smoke.Failure("telemetry: %d frames in 3 s, %d bad CRC" % (got, bridge.bad_crc))
        if not bridge.link_stats_frames:
            raise smoke.Failure("telemetry: no link statistics sent to the radio")
        smoke.log("PASS telemetry: %d CRSF frames in 3 s into the radio, no CRC errors; "
                  "%d link statistics frames" % (got, bridge.link_stats_frames))
    finally:
        feed.running = False
        if bridge:
            bridge.stop()
        sim.stop()
        s.stop()


if __name__ == "__main__":
    try:
        main()
    except smoke.Failure as e:
        smoke.log("FAIL %s" % e)
        sys.exit(1)
