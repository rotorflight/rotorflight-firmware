# Local test dashboard

Runs the CI's firmware builds, unit tests and SITL checks on your machine and shows their
progress in a small web page.

```sh
python3 src/utils/local-ci/local_ci.py            # opens http://127.0.0.1:8765/
python3 src/utils/local-ci/local_ci.py --run      # and starts every step right away
python3 src/utils/local-ci/local_ci.py --list     # list the steps
python3 src/utils/local-ci/local_ci.py --run --no-ui --only unit,sitl   # terminal only, exit code 1 on failure
```

In the page, tick steps or whole groups and press **Run selected**. Click a step to see its
log; the log follows the running step until you pick one. **Rerun failed** runs only the
steps that failed. **Stop** kills the running step and its child processes.

## Steps

The steps are read from the CI files, so they follow CI without changes here:

| Group | Steps | From |
| --- | --- | --- |
| Firmware builds | `make <target>` for each target | `TARGETS` in `.github/workflows/pr.yml`, `unified` expanded from `make/targets_list.mk` |
| Unit tests | `make -k test`, one chip per test program | `src/test/Makefile` |
| SITL | `make TARGET=SITL`, then each `python` step of the `sitl` job (`sitl-windows` on Windows) | `.github/workflows/sitl.yml` |

- Steps run one after another: the SITL checks use fixed ports.
- A failing step does not stop the run. The SITL checks are skipped when the SITL build fails
  in the same run; otherwise they use the existing `obj/main/rotorflight_SITL.elf`.
- The heli checks use a rotorflight-sim clone in `obj/local-ci/sim` at the commit CI pins
  (`SIM_REF`). `--sim <dir>` (or `"sim"` in the config) uses your own checkout instead; the page
  notes when it is not at `SIM_REF`.
- `timeout-minutes` from the workflow apply.
- Logs: `obj/local-ci/<date-time>/<step>.log` and `summary.json`; the heli runs' SITL folders
  (`config.log`, `fly.log`, `eeprom.bin`) are kept in `heli-runs/` there.

## Local settings

Optional, in `make/local-ci.json` (git-ignored) or `--config <file>`. Every key is optional:

```json
{
  "jobs": 8,
  "build": {
    "shell": ["bash", "-c"],
    "env": { "ARM_SDK_DIR": "/path/to/gcc-arm-none-eabi-9-2020-q2-update" },
    "make_args": ""
  },
  "unit": {
    "shell": ["bash", "-c"],
    "setup": "export PATH=/usr/lib/llvm-14/bin:$PATH",
    "make_args": "CC=clang CXX=clang++",
    "goal": "test"
  },
  "sitl": { "env": { "SITL_LOOP_SLEEP_US": "0" } },
  "sim": "../rotorflight-sim",
  "radio": {
    "bindir": "/path/to/companion/lib",
    "sd_template": "/path/to/rotorflight-lua-edgetx-suite/simulator",
    "env": { "LD_LIBRARY_PATH": "/path/to/squashfs-root/usr/lib" }
  }
}
```

- `shell` is the command line that runs a step's command string; `setup` is put in front of it
  (in the same shell), `env` is added to the environment. `"goal": "test-all"` also runs the
  per-target expanded unit tests.
- The radio check (EdgeTX simulator) runs only when `radio.bindir` and `radio.sd_template` are
  set; see the `sitl-radio` jobs in `sitl.yml` for what they point to.

### Windows

- Builds run in the bash next to the GNU make on the `PATH` (MSYS2 or Git for Windows), not a
  login shell. Mixing the two (Git's bash running MSYS2's make) loses the environment.
- The SITL needs MinGW (`make mingw_sdk_install`). To share one install between clones and
  worktrees, set `ARM_SDK_DIR` and `MINGW_SDK_DIR` in `build.env`.
- The unit tests do not build natively on Windows. By default they run in WSL
  (`["wsl", "-e", "bash", "-lc"]`, in the same folder; without `-e` WSL's outer shell
  expands `$VARS` in the command first); put the clang setup in `unit.setup`, for
  example for a clang-14 unpacked into `~/clang-local`:

  ```json
  "unit": {
    "setup": "R=~/clang-local/root; export PATH=$R/usr/lib/llvm-14/bin:$PATH LD_LIBRARY_PATH=$R/usr/lib/x86_64-linux-gnu:$R/usr/lib/llvm-14/lib LIBRARY_PATH=$R/usr/lib/x86_64-linux-gnu CPATH=$R/usr/include",
    "make_args": "CC=clang CXX=clang++"
  }
  ```
