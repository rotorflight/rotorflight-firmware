# Reproduce software validation

The workspace layout is `/work/{firmware,sitl,sitl-baseline,baseline,viewer,viewer-2.3.0,evidence,fixtures}` inside Docker.
The Dockerfile supports Linux arm64 and amd64 and installs GNU Arm 9.3.1, Clang 18 and Node 22.17.0. The validation
report records the resolved image ID, source revisions, artifact hashes and actual commands.

From the directory above a clean checkout of the PR candidate:

```sh
docker build -t rotorflight-bench -f firmware/tools/validation/Dockerfile firmware/tools/validation
mkdir -p evidence/builds
docker run --rm -v "$PWD:/work" rotorflight-bench bash tools/validation/software.sh
```

`software.sh` runs the enabled host suite, the three journal variants under ASan/UBSan, the static audit and its
negative tests, the four hardware builds, and the optional G474/F722 profiling builds. It exits at the first failure.
The sanitizer exceptions are documented in `src/test/unit/sanitizer.ignore`; they preserve the stock PG registry ABI.
Existing disabled tests are not silently converted into passing tests.

For simulator tests, copy the clean firmware checkout to `sitl`, excluding `obj` and `.git`, then apply
`firmware/tools/validation/sitl.patch` with `git apply --directory=sitl` from its parent directory.
The patch contains only simulator support plus a guarded fake-gyro-rate option. It is not applied to hardware builds.
Use the viewer working snapshot recorded in the report for `viewer`; use released Rotorflight Blackbox 2.3.0 for
`viewer-2.3.0`. The analyzer requires both and fails if either is unavailable.

```sh
docker run --rm -v "$PWD:/work" rotorflight-bench bash tools/validation/simulator.sh
```

All simulator builds use a monotonic real-time clock. Standard scenarios use a 1 kHz fake gyro and PID divisor 1
to reduce Docker scheduling overhead. The divisor matrix uses 1/2/3/4/8 and sets the fake gyro to 800 times the
divisor, keeping the PID rate at the supported 800 Hz minimum. It asserts the actual `param_loop` divisor because
firmware validation clamps unsupported loop rates. The LOAD arming guard remains unchanged. These cases validate
subtask ordering and logging, not MCU timing; host scheduling can still prevent a scenario from arming.
Independent MSP timestamps share the firmware clock; journal points are checked against surrounding frame-time intervals because host stalls invalidate exact time interpolation.

`analyze.cjs` checks stored-value ground truth, operation sources and times, cached versus pending PID behavior,
profile/arm slow fields, frame continuity, header hashes, journal CRCs/sequences, and released-viewer frame equality.
`semantics.cjs` limits 4.7 runtime interpretation to the scenario keys reviewed against master `a88bec8f8`;
other keys remain effect-unknown. This is a test adapter, not complete 4.7 desktop analysis support.
`protocol_faults.cjs` corrupts captured records and checks truncated headers and per-log legacy header isolation.
`abrupt_check.cjs` checks safe valid prefixes after forced termination. A write after the last complete frame can leave
no evidence; absence of a trailing-gap flag in that case is not a claim that the write was recorded.

The simulator matrix includes FULL/CHANGES/OFF, gains and profile switches, profile copy/reset, governor and filter
settings, servo commands, receiver adjustments, header-time changes, GRACE rearming and save, between-log changes,
PID denominators 1/2/3/4/8, abrupt stops at 0/2/4/8 ms, SWITCH stop/restart, slow serial fallback, CLI entry and EEPROM
readback after restart. Bounded ring overflow/resynchronization, full storage, nested operations, unhooked writes and
ISR rejection are exercised by the host journal tests; Docker does not emulate a particular flash chip or radio ISR.

Build the stock baseline from the same upstream master commit used by the PR. Record both revisions in the report.
Create `sitl-baseline` from that clean baseline and apply the same simulator-support patch. `simulator.sh` builds both copies with the same options. `OFF` and baseline headers must match apart from firmware revision, build date and log date.
Keep third-party flight-log fixtures out of version control. `legacy_check.cjs` verifies complete decoded frames
against release 2.3.0; the fixture URL and SHA-256 belong in the validation report.

## Hardware handoff

Use the target-matched candidate HEX and the validation manifest. G474 and F722 profiling builds include debug symbols, DWT counters, a refused-flash-byte counter and the existing stack-check task.
DWT indices are HeaderTick, WriteHeader, Running, AfterFrame, OpBegin, OpEnd and OperationSpan. The last includes the writer/loaders.
Read them through your usual debugger with `profiles.gdb`; reset counters between measured scenarios while disarmed.
Measure the actual board clock, and compare OFF/CHANGES/FULL with the same stock configuration. Include storage flushes
and task late counts. Linker headroom and host timing cannot establish stack headroom or hardware timing.

These scripts do not flash hardware or publish a release. Hardware bench and flight results must identify
the actual tested revision; results from an older release do not validate a rebase onto master.
