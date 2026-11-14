#!/usr/bin/env python3
"""Check sitl-run.py: the SITL comes back after every reboot, and not after a crash.

    python3 src/utils/sitl/sitl-run-check.py obj/main/rotorflight_SITL.elf

Runs the reboots a Configurator causes - a CLI `save`, a CLI `exit`, `MSP_REBOOT` - and
reconnects to UART1 after each; a setting saved before the first one must still be there.
Then it kills the SITL, which is not a reset, and expects sitl-run.py to stop.
"""

import importlib.util
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("smoke", os.path.join(HERE, "..", "sitl-smoke-test.py"))
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)


def connect(timeout=20):
    end = time.time() + timeout
    while time.time() < end:
        try:
            s = socket.create_connection(("127.0.0.1", 5761), timeout=2)
            smoke.Msp(s).request(smoke.MSP_API_VERSION)
            return s
        except Exception:
            time.sleep(0.2)
    raise smoke.Failure("no MSP on UART1 within %d s" % timeout)


def wait_closed(s):
    """Until the SITL drops the connection on its way out."""
    s.settimeout(10)
    try:
        while s.recv(256):
            pass
    except socket.timeout:
        s.close()
        raise smoke.Failure("the SITL kept the connection open for 10 s instead of rebooting")
    except OSError:
        pass                        # reset by the closing process: closed
    s.close()


def sitl_pid(logpath):
    """The pid of the SITL sitl-run.py started last, or None."""
    pids = re.findall(r"SITL pid (\d+)", open(logpath).read())
    return int(pids[-1]) if pids else None


def kill(pid):
    if os.name == "nt":
        subprocess.run(["taskkill", "/F", "/PID", str(pid)], capture_output=True)
    else:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main():
    elf = os.path.abspath(sys.argv[1])
    rundir = tempfile.mkdtemp(prefix="sitl-run-")
    logpath = os.path.join(rundir, "sitl-run.log")
    log = open(logpath, "w")
    sup = subprocess.Popen([sys.executable, os.path.join(HERE, "sitl-run.py"), elf, "--dir", rundir],
                           stdout=log, stderr=subprocess.STDOUT)
    try:
        s = connect()
        smoke.cli(s, ["set name = sitl-run-check", "save"])
        wait_closed(s)
        s = connect()
        out = smoke.cli(s, ["get name", "exit"])
        wait_closed(s)
        if "sitl-run-check" not in out:
            raise smoke.Failure("the setting saved before the reboot is gone")
        smoke.log("PASS back after CLI save, setting kept; back after CLI exit")
        s = connect()
        s.sendall(b"$M<" + bytes([0, 68, 68]))          # MSP_REBOOT, firmware
        wait_closed(s)
        connect().close()
        smoke.log("PASS back after MSP_REBOOT")
        if sup.poll() is not None:
            raise smoke.Failure("sitl-run.py ended early")

        kill(sitl_pid(logpath))
        code = sup.wait(15)
        if code == 0:
            raise smoke.Failure("sitl-run.py treated a killed SITL as a reset")
        smoke.log("PASS a killed SITL is not restarted (sitl-run.py exit %d)" % code)
    finally:
        if sup.poll() is None:
            # sitl-run.py's SITL is a process of its own: stop it first, or it
            # keeps the ports and the next run cannot bind them
            pid = sitl_pid(logpath)
            sup.kill()
            sup.wait(5)
            if pid is not None:
                kill(pid)
        log.close()


if __name__ == "__main__":
    try:
        main()
    except smoke.Failure as e:
        smoke.log("FAIL %s" % e)
        sys.exit(1)
