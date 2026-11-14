#!/usr/bin/env python3
"""End-to-end check of the ESC emulators against the SITL.

    python3 src/utils/sitl/esc/esc_check.py obj/main/rotorflight_SITL.elf [protocol ...]

For each protocol: configure the ESC sensor on UART3 (TCP 5763) over the CLI, start the
SITL with a level FDM feed, run `esc_sim.py` on the ESC UART, then over MSP

  1. wait for the ESC's parameter block (MSP_ESC_PARAMETERS): the firmware read it
  2. check its telemetry arrived (MSP_MOTOR_TELEMETRY: voltage, current)
  3. write the block back (MSP_SET_ESC_PARAMETERS) with the emulator's witness setting
     changed and wait for the emulator to report holding the new value: the programming
     path the Configurator's ESC tab uses

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

sys.path.insert(0, HERE)
import esc_sim                                                       # noqa: E402
from esc_wire import field_offset                                    # noqa: E402

MSP_MOTOR_TELEMETRY = 139
MSP_ESC_PARAMETERS = 217
MSP_SET_ESC_PARAMETERS = 218
MSP_PARAM_HEADER_LENGTH = 2          # signature, version/command; then the ESC's payload

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


def emulator(protocol):
    module_name, class_name, kwargs = esc_sim.PROTOCOLS[protocol]
    return getattr(importlib.import_module(module_name), class_name)(**kwargs)


def check(elf, protocol):
    fw_name, half = PROTOCOLS[protocol]
    esc_port = smoke.uart_port(3)           # UART3; moves with SITL_INSTANCE
    rundir = tempfile.mkdtemp(prefix="sitl-esc-%s-" % protocol)
    # Every SITL, socket and helper is cleaned up on any exit, Ctrl+C included: a
    # SITL left running holds the ports the next protocol's check needs.
    s = sock = feed = esc = out = None
    try:
        s = smoke.Sitl(elf, rundir, "1-first"); s.wait(30); s.stop()
        s = smoke.Sitl(elf, rundir, "2-config")
        sock = smoke.connect_uart1(s)
        smoke.cli(sock, ["feature ESC_SENSOR", "serial 2 1024 115200 57600 0 115200",
                         "set esc_sensor_protocol = %s" % fw_name,
                         "set esc_sensor_halfduplex = %s" % ("ON" if half else "OFF"),
                         "save"])
        sock.close(); sock = None
        s.wait(15); s.stop()

        feed = smoke.FdmFeed(); feed.start()
        s = smoke.Sitl(elf, rundir, "3-esc")
        out = open(os.path.join(rundir, "esc_sim.log"), "w")
        sock = smoke.connect_uart1(s)
        msp = smoke.Msp(sock)
        deadline = time.time() + 15
        while True:                         # the ESC UART listens once the SITL is up
            try:
                socket.create_connection(("127.0.0.1", esc_port), timeout=1).close()
                break
            except OSError:
                if time.time() > deadline:
                    return False, "ESC UART (TCP %d) not served" % esc_port
                time.sleep(0.2)
        time.sleep(0.5)
        esc = subprocess.Popen([sys.executable, os.path.join(HERE, "esc_sim.py"), "--protocol",
                                protocol, "--port", str(esc_port), "--seconds", "90"],
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
        while time.time() < deadline and not (volt and curr):
            tel = msp.request(MSP_MOTOR_TELEMETRY)
            _rpm, _err, volt, curr = struct.unpack_from("<IHHH", tel, 1)
            if not (volt and curr):
                time.sleep(0.5)
        # change the emulator's witness -- a real, writable setting of this protocol --
        # and write the block back
        esc_model = emulator(protocol)
        witness = esc_model.witness
        offset, size = field_offset(esc_model.spec, witness)
        offset += MSP_PARAM_HEADER_LENGTH
        new = bytearray(params)
        want = int.from_bytes(new[offset:offset + size], "little") ^ 0x01
        new[offset:offset + size] = want.to_bytes(size, "little")
        # header byte 1: version in the low 6 bits, command in the top 2. A block read
        # back may carry capability bits there (Scorpion: 0x80, "can reset"); written
        # back unchanged that is a command, not a save. Clear them to save.
        new[1] &= 0x3F
        msp_try(msp, MSP_SET_ESC_PARAMETERS, bytes(new))
        # esc_sim prints the witness as the ESC holds it after every write
        mark = "; %s = %d" % (witness, want)
        wrote, deadline = False, time.time() + 30
        while time.time() < deadline and not wrote:
            time.sleep(0.5)
            out.flush()
            with open(out.name) as f:
                wrote = any(line.rstrip().endswith(mark)
                            for line in f if "WRITE #" in line)
        detail = "%d-byte block, telemetry %.2f V %.2f A, %s %s" % (
            len(params), volt / 1000, curr / 1000, witness,
            "-> %d reached the ESC" % want if wrote else "-> %d NOT seen" % want)
        return wrote and volt > 0 and curr > 0, detail
    finally:
        if feed:
            feed.running = False
        if esc:
            esc.terminate()
        if sock:
            sock.close()
        if s:
            s.stop()            # also safe on an already stopped SITL
        if out:
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
