#!/usr/bin/env python3
"""Local test dashboard: run the CI's firmware builds, unit tests and SITL checks on this
machine and follow their progress in the browser.

    python3 src/utils/local-ci/local_ci.py                 # open the dashboard, pick steps, Run
    python3 src/utils/local-ci/local_ci.py --run           # start every step right away
    python3 src/utils/local-ci/local_ci.py --run --no-ui --only unit,sitl   # terminal only

The steps are read from the CI files, so they follow CI without edits here:

  build  one `make <target>` per target of TARGETS in .github/workflows/pr.yml
         (`unified` expanded to UNIFIED_TARGETS from make/targets_list.mk)
  unit   `make -k test` (googletest under src/test), one row per test program
  sitl   `make TARGET=SITL`, then every `python` step of the `sitl` job in
         .github/workflows/sitl.yml (`sitl-windows` on Windows): smoke test, reboot check,
         scenarios, ESC check and the rotorflight-sim heli checks at the pinned SIM_REF;
         the EdgeTX radio check when it is configured

Steps run one at a time (the SITL checks share fixed ports). A failing step does not stop
the run, except that the SITL checks are skipped when the SITL build fails. Logs go to
obj/local-ci/<run>/, the heli runs' SITL folders to obj/local-ci/<run>/heli-runs.

Local settings (shells, toolchain paths, EdgeTX) live in make/local-ci.json (git-ignored),
see src/utils/local-ci/README.md. Uses only the Python standard library.
"""

import argparse
import json
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
WINDOWS = os.name == "nt"
SIM_REPO = "https://github.com/rotorflight/rotorflight-sim.git"
ELF = "obj/main/rotorflight_SITL.elf"
MAX_LOG_LINES = 50000
SITL_PORTS = [("tcp", 5761), ("tcp", 5762), ("udp", 9002), ("udp", 9003)]
PORT_WAIT = 600     # seconds to wait for another SITL to free the ports


# --- CI files -------------------------------------------------------------------------------

def read(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return f.read()


def workflow_env(text, name):
    m = re.search(r"^\s+%s:\s*(.+?)\s*$" % re.escape(name), text, re.M)
    return m.group(1) if m else None


def workflow_steps(text, job):
    """Steps of one job: [{name, run, uses, timeout}]. Only the YAML that the workflows use."""
    steps, in_job, step, block = [], False, None, None
    for line in text.splitlines():
        if re.match(r"^  [\w-]+:\s*$", line):
            in_job = line.strip() == job + ":"
            step = block = None
            continue
        if not in_job:
            continue
        if block is not None:
            if line.startswith(" " * 8) or not line.strip():
                block.append(line[8:])
                continue
            step["run"] = "\n".join(block).strip()
            block = None
        m = re.match(r"^    - name:\s*(.+?)\s*$", line)
        if m:
            step = {"name": m.group(1), "run": None, "uses": None, "timeout": None}
            steps.append(step)
            continue
        if step is None:
            continue
        m = re.match(r"^      (run|uses|timeout-minutes):\s*(.*?)\s*(#.*)?$", line)
        if m:
            key, value = m.group(1), m.group(2)
            if key == "run" and value == "|":
                block = []
            elif key == "run":
                step["run"] = value
            elif key == "uses":
                step["uses"] = value
            else:
                step["timeout"] = int(value) * 60
    if block is not None and step is not None:
        step["run"] = "\n".join(block).strip()
    return steps


def unified_targets():
    m = re.search(r"^UNIFIED_TARGETS\s*:=\s*\\\n((?:[ \t]+\S+[ \t]*\\?\n)+)", read("make/targets_list.mk"), re.M)
    return m.group(1).replace("\\", " ").split() if m else []


def build_targets():
    targets = []
    for t in (workflow_env(read(".github/workflows/pr.yml"), "TARGETS") or "").split():
        targets += unified_targets() if t == "unified" else [t]
    return targets


# --- configuration --------------------------------------------------------------------------

def windows_bash():
    """The bash next to the `make` on the PATH: an MSYS2 or Git for Windows program passes no
    environment to one of the other runtime, so make would not see ARM_SDK_DIR and the like.
    Not a login shell (that would drop the Windows PATH). `bash` alone may be WSL's."""
    candidates = []
    for tool in ("make", "git"):
        path = shutil.which(tool)
        if path:
            d = os.path.dirname(path)
            candidates += [os.path.join(d, "bash.exe"), os.path.join(os.path.dirname(d), "bin", "bash.exe")]
    candidates += ["C:/msys64/usr/bin/bash.exe", "C:/Program Files/Git/bin/bash.exe"]
    return next((c for c in candidates if os.path.exists(c)), None)


def default_config():
    if WINDOWS:
        build_shell = [windows_bash() or "bash", "-c"]
        unit_shell = ["wsl", "-e", "bash", "-lc"]   # -e: no outer shell expanding $VARS
    else:
        build_shell = unit_shell = ["bash", "-c"]
    return {
        "jobs": os.cpu_count() or 4,
        "build": {"shell": build_shell, "setup": "", "env": {}, "make_args": ""},
        "unit": {"shell": unit_shell, "setup": "", "env": {}, "make_args": "", "goal": "test"},
        "sitl": {"env": {}},
        "sim": None,
        "radio": None,
    }


def load_config(path):
    cfg = default_config()
    if path and os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            user = json.load(f)
        for key, value in user.items():
            if isinstance(value, dict) and isinstance(cfg.get(key), dict):
                cfg[key].update(value)
            else:
                cfg[key] = value
    return cfg


# --- steps ----------------------------------------------------------------------------------

class Step:
    def __init__(self, sid, group, name, kind, **kw):
        self.id, self.group, self.name, self.kind = sid, group, name, kind
        self.cmd = kw.get("cmd")            # shell: command string; exec: argv
        self.shell = kw.get("shell")        # config section for shell steps
        self.env = kw.get("env", {})
        self.func = kw.get("func")
        self.timeout = kw.get("timeout")
        self.needs = kw.get("needs")
        self.parser = kw.get("parser")
        self.disabled = kw.get("disabled")  # reason the step cannot run here
        self.ports = kw.get("ports")        # runs a SITL: wait for its fixed ports
        self.reset()

    def reset(self):
        self.status = "idle"
        self.started = self.ended = None
        self.rc = None
        self.items = {}                     # sub-results, e.g. unit test programs
        self.notes = []
        self.lines = []
        self.dropped = 0
        self.logfile = None

    def log(self, line):
        self.lines.append(line)
        if len(self.lines) > MAX_LOG_LINES:
            cut = len(self.lines) - MAX_LOG_LINES
            del self.lines[:cut]
            self.dropped += cut
        if self.logfile:
            self.logfile.write(line + "\n")
            self.logfile.flush()

    def summary(self):
        now = time.time()
        dur = None
        if self.started:
            dur = (self.ended or now) - self.started
        done = sum(1 for s in self.items.values() if s in ("pass", "fail"))
        return {
            "id": self.id, "group": self.group, "name": self.name, "status": self.status,
            "disabled": self.disabled, "duration": dur, "rc": self.rc, "notes": self.notes[-50:],
            "items": [{"name": k, "status": v} for k, v in self.items.items()],
            "progress": [done, len(self.items)] if self.items else None,
            "log_len": self.dropped + len(self.lines),
        }


class UnitParser:
    """Turns `make -k test_help test` output into one item per test program."""
    HELP = re.compile(r"^\s+test_(\S+)\s*$")
    PASS = re.compile(r"running test_(\S+): PASS")
    FAIL = re.compile(r"\*\*\* \[[^\]]*?\btest_([\w.+-]+)\] Error")
    BUILD_FAIL = re.compile(r"\*\*\* \[[^\]]*?obj/test/([^/\]]+)/[^\]]*\] Error")
    COMPILE = re.compile(r"^(?:compiling|linking) \S*?(?:unit/|obj/test/[^/]+/)([\w.+-]+?_unittest)\b")
    GTEST_FAIL = re.compile(r"^\[  FAILED  \] (\S+\.\S+)")

    def __init__(self):
        self.in_help = True

    def __call__(self, step, line):
        if self.in_help:
            m = self.HELP.match(line)
            if m:
                step.items.setdefault(m.group(1), "queued")
                return
            if line.startswith(("compiling", "linking", "running")):
                self.in_help = False
        m = self.PASS.search(line)
        if m:
            step.items[m.group(1)] = "pass"
            return
        m = self.FAIL.search(line) or self.BUILD_FAIL.search(line)
        if m:
            step.items[m.group(1)] = "fail"
            return
        m = self.COMPILE.search(line)
        if m and step.items.get(m.group(1)) in (None, "queued"):
            step.items[m.group(1)] = "running"
            return
        m = self.GTEST_FAIL.match(line)
        if m and m.group(1) not in step.notes:
            step.notes.append(m.group(1))


def python_argv(run, sim_dir):
    argv = shlex.split(run)
    if argv and argv[0] in ("python", "python3"):
        argv[0] = sys.executable
    return [os.path.join(sim_dir, a[4:]) if a.startswith("sim/") else a for a in argv]


def build_steps(cfg, sim_override):
    steps = []
    jflag = "-j%d" % int(cfg["jobs"])
    b, u = cfg["build"], cfg["unit"]

    for t in build_targets():
        steps.append(Step("build:" + t, "build", "Build " + t, "shell", shell=b,
                          cmd="make %s %s %s" % (jflag, t, b.get("make_args", ""))))

    goal = u.get("goal", "test")
    steps.append(Step("unit:" + goal, "unit", "Unit tests (make %s)" % goal, "shell", shell=u,
                      cmd="cd src/test && make -s help && make -k %s %s %s" % (jflag, goal, u.get("make_args", "")),
                      parser=UnitParser()))

    wf = read(".github/workflows/sitl.yml")
    sim_ref = workflow_env(wf, "SIM_REF")
    job = "sitl-windows" if WINDOWS else "sitl"
    steps.append(Step("sitl:build", "sitl", "Build SITL", "shell", shell=b,
                      cmd="make %s TARGET=SITL %s" % (jflag, b.get("make_args", ""))))
    sim_dir = os.path.abspath(sim_override or cfg.get("sim") or os.path.join(ROOT, "obj", "local-ci", "sim"))
    sitl_env = dict(cfg["sitl"].get("env", {}))
    checkout_added = False
    for s in workflow_steps(wf, job):
        run = s["run"] or ""
        if s["uses"] and s["name"].startswith("Check out the flight models"):
            pinned = not (sim_override or cfg.get("sim"))
            steps.append(Step("sitl:sim", "sitl", "Flight models (rotorflight-sim %s)" % (sim_ref or "?")[:10],
                              "func", func=lambda step, r, d=sim_dir, ref=sim_ref, p=pinned: r.sim_checkout(step, d, ref, p),
                              needs="sitl:build"))
            checkout_added = True
            continue
        if not re.match(r"^python3? ", run):
            continue
        env = dict(sitl_env)
        needs = "sitl:build"
        if "sim/" in run:
            env["KEEP"] = "1"
            env["TMPDIR"] = "{heli_runs}"
            needs = "sitl:sim" if checkout_added else needs
        sid = "sitl:" + re.sub(r"[^a-z0-9]+", "-", s["name"].lower()).strip("-")
        steps.append(Step(sid, "sitl", s["name"], "exec", cmd=python_argv(run, sim_dir),
                          env=env, timeout=s["timeout"], needs=needs, ports=True))

    radio = cfg.get("radio") or {}
    argv = [sys.executable, "src/utils/sitl/radio/radio_check.py", ELF,
            "--bindir", radio.get("bindir") or "", "--sd-template", radio.get("sd_template") or ""]
    steps.append(Step("sitl:radio", "sitl", "Radio check (EdgeTX simulator)", "exec", cmd=argv,
                      env={**sitl_env, **radio.get("env", {})}, timeout=300, needs="sitl:build", ports=True,
                      disabled=None if radio.get("bindir") and radio.get("sd_template")
                      else "set radio.bindir and radio.sd_template in make/local-ci.json"))
    return steps, {"sim_ref": sim_ref, "sim_dir": sim_dir, "sitl_job": job}


# --- runner ---------------------------------------------------------------------------------

class Runner:
    def __init__(self, steps, info):
        self.steps = steps
        self.by_id = {s.id: s for s in steps}
        self.info = info
        self.lock = threading.RLock()
        self.thread = None
        self.proc = None
        self.stop_requested = False
        self.run_dir = None
        self.started = self.ended = None
        self.listeners = []

    # state

    def state(self):
        with self.lock:
            return {
                "running": self.running(), "run_dir": self.run_dir,
                "started": self.started, "ended": self.ended, "now": time.time(),
                "info": self.info, "steps": [s.summary() for s in self.steps],
            }

    def running(self):
        return bool(self.thread and self.thread.is_alive())

    def log_lines(self, sid, start):
        with self.lock:
            s = self.by_id.get(sid)
            if not s:
                return None
            start = max(start, s.dropped)
            return {"start": start, "lines": s.lines[start - s.dropped:start - s.dropped + 5000],
                    "total": s.dropped + len(s.lines)}

    # control

    def start(self, ids):
        with self.lock:
            if self.running():
                return False
            ids = [s.id for s in self.steps if s.id in set(ids) and not s.disabled]
            if not ids:
                return False
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.run_dir = os.path.join(ROOT, "obj", "local-ci", stamp)
            os.makedirs(os.path.join(self.run_dir, "heli-runs"), exist_ok=True)
            for s in self.steps:
                if s.id in ids:
                    s.reset()
                    s.status = "queued"
            self.stop_requested = False
            self.started, self.ended = time.time(), None
            self.thread = threading.Thread(target=self._run, args=(ids,), daemon=True)
            self.thread.start()
            return True

    def stop(self):
        with self.lock:
            self.stop_requested = True
            if self.proc:
                kill_tree(self.proc)

    def _run(self, ids):
        for sid in ids:
            s = self.by_id[sid]
            with self.lock:
                if self.stop_requested:
                    s.status = "stopped"
                    continue
                dep = self.by_id.get(s.needs) if s.needs in ids else None
                if dep and dep.status in ("fail", "skipped", "stopped"):
                    s.status = "skipped"
                    s.notes.append("skipped: %s %s" % (dep.name, dep.status))
                    self._emit(s)
                    continue
                s.status = "running"
                s.started = time.time()
                s.logfile = open(os.path.join(self.run_dir, re.sub(r"[^\w.-]", "_", sid) + ".log"),
                                 "w", encoding="utf-8")
            self._emit(s)
            try:
                rc = s.func(s, self) if s.kind == "func" else self.execute(s)
            except Exception as e:      # a broken step must not end the run
                s.log("local-ci: %s: %s" % (type(e).__name__, e))
                rc = 1
            with self.lock:
                s.rc = rc
                s.ended = time.time()
                if self.stop_requested:
                    s.status = "stopped"
                elif rc == "timeout":
                    s.status = "fail"
                    s.notes.append("timed out after %d s" % s.timeout)
                else:
                    s.status = "pass" if rc == 0 else "fail"
                for k, v in s.items.items():
                    if v in ("queued", "running"):
                        s.items[k] = "fail" if v == "running" and s.status == "fail" else "not run"
                s.logfile.close()
                s.logfile = None
            self._emit(s)
        with self.lock:
            self.ended = time.time()
        self._write_summary()
        self._emit(None)

    def _emit(self, step):
        for cb in self.listeners:
            cb(step)

    def _write_summary(self):
        with open(os.path.join(self.run_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(self.state(), f, indent=1)

    # processes

    def execute(self, s):
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        heli_runs = os.path.join(self.run_dir, "heli-runs")
        if s.kind == "shell":
            sh = s.shell
            env.update({k: str(v) for k, v in sh.get("env", {}).items()})
            if "-l" in sh["shell"][-1] and "msys" in sh["shell"][0].lower():
                env["CHERE_INVOKING"] = "1"     # MSYS2 login shell: stay in the repository
            setup = sh.get("setup") or ""
            argv = list(sh["shell"]) + [(setup.rstrip("; \n") + "; " if setup else "") + s.cmd]
            s.log("$ " + s.cmd)
        else:
            if s.ports and not self.wait_for_ports(s):
                return 1
            env.update({k: str(v).replace("{heli_runs}", heli_runs) for k, v in s.env.items()})
            argv = s.cmd
            s.log("$ " + " ".join(shlex.quote(a) for a in argv))
        return self.run_argv(s, argv, env)

    def wait_for_ports(self, s):
        """A SITL started elsewhere (another checkout, a stray process) holds the fixed ports
        and makes the checks fail in odd ways: wait for it to end."""
        deadline, waiting = time.time() + PORT_WAIT, None
        while not self.stop_requested:
            busy = [str(p) for kind, p in SITL_PORTS if port_busy(kind, p)]
            if not busy:
                if waiting:
                    s.log("SITL ports free again")
                    s.notes.remove(waiting)
                return True
            if time.time() > deadline:
                s.notes.append("SITL ports %s still in use after %d s" % (", ".join(busy), PORT_WAIT))
                return False
            if not waiting:
                waiting = "waiting: SITL ports %s in use (another SITL running?)" % ", ".join(busy)
                s.notes.append(waiting)
                s.log(waiting)
            time.sleep(1)
        return False

    def run_argv(self, s, argv, env=None):
        kw = {}
        if WINDOWS:
            kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kw["start_new_session"] = True
        with self.lock:
            if self.stop_requested:
                return 1
            self.proc = subprocess.Popen(argv, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **kw)
        proc = self.proc
        timed_out = []
        timer = None
        if s.timeout:
            timer = threading.Timer(s.timeout, lambda: (timed_out.append(1), kill_tree(proc)))
            timer.daemon = True
            timer.start()
        for raw in proc.stdout:
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            for part in line.split("\r"):
                with self.lock:
                    s.log(part)
                    if s.parser:
                        s.parser(s, part)
        rc = proc.wait()
        if timer:
            timer.cancel()
        with self.lock:
            self.proc = None
        return "timeout" if timed_out else rc

    def sim_checkout(self, s, sim_dir, ref, pinned):
        git = lambda *a: self.run_argv(s, ["git", "-C", sim_dir] + list(a))
        if not pinned:
            s.log("using %s as configured" % sim_dir)
            if not os.path.isdir(sim_dir):
                s.log("not found")
                return 1
            head = subprocess.run(["git", "-C", sim_dir, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
            s.log("HEAD %s, CI pins %s" % (head, ref))
            if ref and head != ref:
                s.notes.append("rotorflight-sim at %s, CI pins %s" % (head[:10], ref[:10]))
            return 0
        if not os.path.isdir(os.path.join(sim_dir, ".git")):
            os.makedirs(os.path.dirname(sim_dir), exist_ok=True)
            rc = self.run_argv(s, ["git", "clone", "--quiet", SIM_REPO, sim_dir])
            if rc:
                return rc
        if subprocess.run(["git", "-C", sim_dir, "cat-file", "-e", ref + "^{commit}"],
                          capture_output=True).returncode != 0:
            rc = git("fetch", "--quiet", "origin")
            if rc:
                return rc
        rc = git("checkout", "--quiet", "--detach", ref)
        if rc == 0:
            s.log("rotorflight-sim at %s" % ref)
        return rc


def port_busy(kind, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM if kind == "tcp" else socket.SOCK_DGRAM)
    try:
        if WINDOWS:     # do not share a port another socket is bound to
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:           # but do allow TCP ports in TIME_WAIT
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", port))
        return False
    except OSError:
        return True
    finally:
        sock.close()


def kill_tree(proc):
    if proc.poll() is not None:
        return
    try:
        if WINDOWS:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        else:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
    except (OSError, ProcessLookupError):
        pass


# --- web UI ---------------------------------------------------------------------------------

def make_handler(runner):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    self.send(200, f.read(), "text/html; charset=utf-8")
            elif url.path == "/api/state":
                self.send(200, runner.state())
            elif url.path == "/api/log":
                q = parse_qs(url.query)
                res = runner.log_lines(q.get("id", [""])[0], int(q.get("from", ["0"])[0]))
                self.send(200 if res else 404, res or {"error": "no such step"})
            else:
                self.send(404, {"error": "not found"})

        def do_POST(self):
            # only same-origin requests from the page (no CORS); refuse form posts from elsewhere
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                return self.send(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"error": "json only"})
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/api/run":
                ok = runner.start(body.get("ids", []))
                self.send(200 if ok else 409, {"ok": ok})
            elif self.path == "/api/stop":
                runner.stop()
                self.send(200, {"ok": True})
            else:
                self.send(404, {"error": "not found"})

    return Handler


# --- terminal -------------------------------------------------------------------------------

def terminal_listener(runner):
    def on_step(s):
        if s is None:
            return
        if s.status == "running":
            print("[ .. ] %s" % s.name, flush=True)
        elif s.status in ("pass", "fail", "skipped", "stopped"):
            dur = " (%.0f s)" % (s.ended - s.started) if s.started and s.ended else ""
            extra = ""
            if s.items:
                n = sum(1 for v in s.items.values() if v == "pass")
                extra = " %d/%d passed" % (n, len(s.items))
            print("[%s] %s%s%s" % (s.status.upper()[:4].ljust(4), s.name, extra, dur), flush=True)
            for n in s.notes:
                print("       " + n, flush=True)
            if s.status == "fail":
                for line in s.lines[-30:]:
                    print("     | " + line, flush=True)
    return on_step


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", default=os.path.join(ROOT, "make", "local-ci.json"),
                    help="local settings (default make/local-ci.json)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--only", help="comma-separated groups or step ids to select: build,unit,sitl,sitl:build,...")
    ap.add_argument("--run", action="store_true", help="start the selected steps right away")
    ap.add_argument("--no-ui", action="store_true", help="no web server; with --run, print progress and exit")
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser")
    ap.add_argument("--sim", help="use this rotorflight-sim checkout instead of one pinned to SIM_REF")
    ap.add_argument("--list", action="store_true", help="list the steps and exit")
    args = ap.parse_args()

    cfg = load_config(args.config)
    steps, info = build_steps(cfg, args.sim)
    info["platform"] = sys.platform
    info["config"] = args.config if os.path.exists(args.config) else None
    runner = Runner(steps, info)

    selected = [s.id for s in steps if not s.disabled]
    if args.only:
        want = [w.strip() for w in args.only.split(",") if w.strip()]
        selected = [s.id for s in steps if not s.disabled and (s.group in want or s.id in want)]
    info["selected"] = selected

    if args.list:
        for s in steps:
            mark = "x" if s.id in selected else " "
            print("[%s] %-14s %s%s" % (mark, s.id, s.name, "  (%s)" % s.disabled if s.disabled else ""))
        return 0

    if args.no_ui:
        if not args.run:
            ap.error("--no-ui needs --run")
        runner.listeners.append(terminal_listener(runner))
        runner.start(selected)
        try:
            while runner.running():
                time.sleep(0.2)
        except KeyboardInterrupt:
            runner.stop()
            runner.thread.join()
        print("logs: " + runner.run_dir)
        failed = [s for s in steps if s.status == "fail"]
        return 1 if failed or runner.stop_requested else 0

    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(runner))
    url = "http://127.0.0.1:%d/" % args.port
    print("Local test dashboard: " + url + "   (Ctrl+C to quit)", flush=True)
    runner.listeners.append(terminal_listener(runner))
    if args.run:
        runner.start(selected)
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        runner.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
