#!/usr/bin/env python3
"""Run SITL checks side by side, each SITL on its own set of ports.

    python3 src/utils/sitl/run-parallel.py --workflow .github/workflows/sitl.yml --job sitl
    python3 src/utils/sitl/run-parallel.py --commands checks.txt --jobs 4 --repeat 10
    python3 src/utils/sitl/run-parallel.py -- "python3 sim/heli/heli_bridge.py obj/main/rotorflight_SITL.elf --check"

Every check starts its SITL(s) on fixed ports, so on one host they run one at a time.
Here each worker gets its own `SITL_INSTANCE` (1..N): the SITL moves all its default ports
by 10 * N (TCP UARTs, FDM in, outputs out), and so do the scripts that talk to it
(`sitl-smoke-test.py` and the scripts built on it, `heli_bridge.py` in rotorflight-sim).
Instance 0 - the usual ports - is left free for a Configurator or a manual run. Each run
also gets its own TMPDIR, kept with its output under `--logs`.

**How many at once.** In lockstep a SITL's main loop spins on a core and the bridge or
check driving it takes most of another, so the default is half the CPUs. Lockstep checks
do not care about host load; checks that run the firmware on real time (the smoke test's
clock and CRSF stages) do, and are better left out or run with fewer jobs.

`--workflow FILE --job NAME` takes the job's single-line `run: python...` steps in order;
multi-line steps (installs, builds) are skipped. Exit status 0 when every run passed.
"""

import argparse
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor


def workflow_commands(path, job):
    """Single-line `run: python...` steps of one job, by indentation, without a YAML library."""
    commands, in_job, job_indent = [], False, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            stripped = line.rstrip("\n")
            m = re.match(r"^(\s+)([A-Za-z0-9_-]+):\s*$", stripped)
            if m and (job_indent is None or len(m.group(1)) == job_indent):
                if m.group(2) == job:
                    in_job, job_indent = True, len(m.group(1))
                    continue
                if in_job:
                    break                   # the next job at the same level
            if in_job:
                # `run:` under a named step, or `- run:` opening an unnamed one
                m = re.match(r"^\s+(?:-\s+)?run:\s*(python3?\s.+)$", stripped)
                if m:
                    commands.append(m.group(1).strip())
    if not commands:
        sys.exit("no single-line python steps in job %r of %s" % (job, path))
    return commands


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", nargs="*", help="commands to run, one per argument")
    ap.add_argument("--commands", help="file with one command per line (# comments)")
    ap.add_argument("--workflow", help="take the commands from this workflow file ...")
    ap.add_argument("--job", help="... and this job in it")
    ap.add_argument("--jobs", "-j", type=int, default=max(1, (os.cpu_count() or 2) // 2),
                    help="runs at once (default: half the CPUs)")
    ap.add_argument("--repeat", type=int, default=1, help="run the whole list this many times")
    ap.add_argument("--logs", default="sitl-parallel-logs",
                    help="directory for each run's output and TMPDIR")
    a = ap.parse_args()

    commands = list(a.command)
    if a.commands:
        with open(a.commands, encoding="utf-8") as f:
            commands += [ln.strip() for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
    if a.workflow or a.job:
        if not (a.workflow and a.job):
            ap.error("--workflow and --job go together")
        commands += workflow_commands(a.workflow, a.job)
    if not commands:
        ap.error("no commands")
    if not 1 <= a.jobs <= 99:
        ap.error("--jobs takes 1..99 (SITL_INSTANCE 1..99)")
    if a.repeat < 1:
        ap.error("--repeat takes 1 or more")     # 0 would run nothing and report success

    runs = [(i, cmd) for i, cmd in enumerate(commands * a.repeat, 1)]
    os.makedirs(a.logs, exist_ok=True)
    free = list(range(1, a.jobs + 1))       # SITL_INSTANCE numbers, one per worker
    lock = threading.Lock()
    results = {}
    t0 = time.time()

    def run(item):
        n, cmd = item
        with lock:
            instance = free.pop(0)
        try:
            tmp = os.path.abspath(os.path.join(a.logs, "run-%03d" % n))
            os.makedirs(tmp, exist_ok=True)
            env = dict(os.environ, SITL_INSTANCE=str(instance), TMPDIR=tmp, TEMP=tmp, TMP=tmp)
            start = time.time()
            with open(os.path.join(tmp, "output.txt"), "w", encoding="utf-8") as out:
                # POSIX: split as a shell would. Windows: hand the string to CreateProcess,
                # which applies the Windows quoting rules itself - shlex's non-POSIX mode
                # would keep the quote characters inside the arguments.
                argv = cmd if os.name == "nt" else shlex.split(cmd)
                rc = subprocess.call(argv, env=env, stdout=out, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
            took = time.time() - start
        finally:
            with lock:
                free.append(instance)
        with lock:
            results[n] = (rc, took, cmd)
            print("%s %3d/%d  %5.1f s  [instance %d]  %s"
                  % ("PASS" if rc == 0 else "FAIL", n, len(runs), took, instance, cmd), flush=True)
        return rc

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        list(pool.map(run, runs))

    failed = sorted(n for n, (rc, _, _) in results.items() if rc != 0)
    print("\n%d of %d passed in %.0f s with %d at once; output and TMPDIR of each under %s/"
          % (len(runs) - len(failed), len(runs), time.time() - t0, a.jobs, a.logs))
    for n in failed:
        print("  FAIL run-%03d: %s" % (n, results[n][2]))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
