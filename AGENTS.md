# AGENTS Guide: Rotorflight Firmware

## Project Identity

Rotorflight is flight controller firmware for RC helicopters, based on Betaflight 4.3. It does not
support multirotors or airplanes.

Reference: [README.md](README.md)

## Guidance for Agents

### Keep Stable Interfaces Stable

- Do not renumber BOX permanent IDs unless an explicit migration plan exists.
- Preserve MSP and CLI backward compatibility where possible. MSP changes must stay in step with
  [rotorflight-configurator](https://github.com/rotorflight/rotorflight-configurator) and the Lua scripts.

### Mixer, Governor and PID Work Is Safety-Critical

- Treat changes in [src/main/flight/mixer.c](src/main/flight/mixer.c),
  [src/main/flight/governor.c](src/main/flight/governor.c), [src/main/flight/pid.c](src/main/flight/pid.c) and
  [src/main/flight/servos.c](src/main/flight/servos.c) as high risk.
- Validate arming interactions and override behavior in [src/main/fc/core.c](src/main/fc/core.c).

### Document Behavior, Not Only Intent

- If a setting is renamed, keep aliases where practical and record it in [Changes.md](Changes.md).
- User-facing documentation lives on <https://www.rotorflight.org/>. Note in the PR when a behaviour change needs
  a matching docs update.

## Build and Test

```
make arm_sdk_install          # once: installs the ARM toolchain under tools/
make TARGET=STM32F7X2         # one unified target, or `make unified` for all
make test                     # unit tests in src/test/unit
```

`make help` lists the options. Extra defines go in `OPTIONS="USE_SOMETHING"`. CI runs the GitHub Actions
workflows in `.github/workflows`.

`src/main/build/version.h` requires `FC_VERSION_STRING` to stay under 30 bytes, so a long `FC_VER_SUFFIX` fails
the build with `fc_version_string_too_long`.

Unit tests exist for PID, setpoint and maths. The governor, mixer, servos, rescue and leveling have none, so
changes there need extra care and, where possible, a new test.

## Issues and Pull Requests

Maintainers and users read these, so keep them short and precise. No filler, no restating the diff.

**Issues**

- Title: the symptom in one line.
- Body: firmware version and target, steps to reproduce, expected vs actual behaviour. Attach CLI `diff all` or a
  Blackbox log when relevant.

**Pull requests**

- Title: imperative, under ~70 characters (e.g. "Fix governor spool-up overshoot").
- Body, a few lines each:
  - **What**: the change and why it is needed. Link the issue (`Fixes #123`).
  - **Compatibility**: MSP, CLI or setting changes, and whether the Configurator or Lua scripts need updating.
  - **Testing**: what was built and run (unit tests, SITL, bench, flight).
- One topic per PR. Leave out file-by-file change lists and generated summaries.
