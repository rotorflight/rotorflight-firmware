#!/usr/bin/env python3
"""Run the SITL and bring it back after every reboot, like a board coming back on power.

    python3 src/utils/sitl/sitl-run.py obj/main/rotorflight_SITL.elf [--dir RUNDIR] [--max 50]

On this target a reboot ends the process: a CLI `save`, `MSP_REBOOT` or the Configurator's
"Save and Reboot" all reach `systemResetHard()`, which prints `[system]Reset!` and exits. A
real board would come back on the configuration it just stored, so this starts the SITL
again in the same directory, on the same `eeprom.bin`, and the Configurator (or any MSP
client) reconnects to `tcp://127.0.0.1:5761`/`5762` as it would after a board reboot.

Two rules keep it honest:

- **Only a reset is restarted.** The firmware has to have said `[system]Reset!` before it
  ended. A process that ends without it crashed, and a crash is reported and stops the
  run: a supervisor that restarts crashes turns a failure into a slow loop.
- **`--max` bounds the restarts**, so a configuration that resets on every boot cannot spin.

The first start in an empty directory writes the defaults to `eeprom.bin` and resets; that
restart is counted like any other. The SITL's output is passed through, with a line marking
each restart. Ctrl+C stops the SITL and this.
"""

import argparse
import os
import subprocess
import sys
import threading
import time

RESET_MARK = "[system]Reset"


def run_once(elf, rundir, env):
    """Run the SITL to its end; returns (exit code, whether it announced a reset)."""
    proc = subprocess.Popen([elf], cwd=rundir, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    print("[sitl-run] SITL pid %d in %s" % (proc.pid, rundir), flush=True)
    seen = threading.Event()

    def pump():
        for raw in proc.stdout:
            line = raw.decode(errors="replace")
            if RESET_MARK in line:
                seen.set()
            sys.stdout.write(line)
            sys.stdout.flush()

    reader = threading.Thread(target=pump, daemon=True)
    reader.start()
    try:
        code = proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        raise
    reader.join(2)
    return code, seen.is_set()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("elf", help="the SITL executable (obj/main/rotorflight_SITL.elf)")
    ap.add_argument("--dir", default=".", help="run directory: eeprom.bin and the blackbox live here")
    ap.add_argument("--max", type=int, default=50, help="restarts before giving up (default 50)")
    args = ap.parse_args()

    elf = os.path.abspath(args.elf)
    rundir = os.path.abspath(args.dir)
    os.makedirs(rundir, exist_ok=True)
    env = dict(os.environ)
    restarts = 0
    try:
        while True:
            code, reset = run_once(elf, rundir, env)
            if not reset:
                print("[sitl-run] the SITL ended (exit %s) without announcing a reset: "
                      "not restarted" % code, flush=True)
                return code or 1
            if restarts >= args.max:
                print("[sitl-run] reset again after %d restarts (--max): not restarted"
                      % restarts, flush=True)
                return 2
            restarts += 1
            # the old process has closed its listeners; give the OS a moment to free the ports
            time.sleep(0.3)
            print("[sitl-run] ===== reset: restart %d of at most %d, same eeprom.bin ====="
                  % (restarts, args.max), flush=True)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
