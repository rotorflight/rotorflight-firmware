# AGENTS Guide: Rotorflight Firmware

## Project Identity

Rotorflight is flight controller firmware for RC helicopters, based on Betaflight 4.3. It does not
support multirotors or airplanes.

Reference: [README.md](README.md)

## Project Components

Rotorflight is split across several repositories under <https://github.com/rotorflight>:

| Repository | Purpose |
| --- | --- |
| [rotorflight-firmware](https://github.com/rotorflight/rotorflight-firmware) | Flight controller firmware (this repository) |
| [rotorflight-targets](https://github.com/rotorflight/rotorflight-targets) | Board configurations (custom defaults) applied on top of the unified targets |
| [rotorflight-configurator](https://github.com/rotorflight/rotorflight-configurator) | Desktop/web app for flashing and configuring the FC over MSP |
| [rotorflight-blackbox](https://github.com/rotorflight/rotorflight-blackbox) | Blackbox Explorer for analysing flight logs |
| [rotorflight-lua-scripts](https://github.com/rotorflight/rotorflight-lua-scripts) | Transmitter Lua scripts for EdgeTX |
| [rotorflight-lua-edgetx-suite](https://github.com/rotorflight/rotorflight-lua-edgetx-suite) | Lua suite for EdgeTX |
| [rotorflight-lua-ethos](https://github.com/rotorflight/rotorflight-lua-ethos) | Transmitter Lua scripts for FrSky Ethos |
| [rotorflight-lua-ethos-suite](https://github.com/rotorflight/rotorflight-lua-ethos-suite) | Lua suite for FrSky Ethos |
| [rotorflight-presets](https://github.com/rotorflight/rotorflight-presets) | Parameter presets loaded by the Configurator |
| [rotorflight-artifacts](https://github.com/rotorflight/rotorflight-artifacts) | Firmware builds mirrored for the web configurator |
| [rotorflight-docs](https://github.com/rotorflight/rotorflight-docs) | Documentation website (<https://www.rotorflight.org/>) |
| [rotorflight-ref-design](https://github.com/rotorflight/rotorflight-ref-design) | Flight controller reference hardware designs |
| [rotorflight](https://github.com/rotorflight/rotorflight) | Wiki material, media, changelogs and example files |

The Configurator and all Lua scripts talk to the firmware over MSP, so MSP changes here affect all of them.

## Guidance for Agents

### Keep Stable Interfaces Stable

- Do not renumber BOX permanent IDs unless an explicit migration plan exists.
- Preserve MSP and CLI backward compatibility where possible. MSP changes must stay in step with
  [rotorflight-configurator](https://github.com/rotorflight/rotorflight-configurator) and the Lua scripts.

### Never Bump the MSP API or Firmware Version

Agents must not change these. The maintainers bump them manually when a new version is released, which keeps
version changes to a minimum:

- MSP API version: `API_VERSION_MAJOR` / `API_VERSION_MINOR` in
  [src/main/msp/msp_protocol.h](src/main/msp/msp_protocol.h).
- Firmware version: `FC_VERSION_MAJOR` / `FC_VERSION_MINOR` / `FC_VERSION_PATCH_LEVEL` in
  [src/main/build/version.h](src/main/build/version.h).

MSP changes made between releases belong to the upcoming API version. List them in the PR under **Compatibility**
so they are covered by the release bump and the Configurator can pick them up.

Parameter groups are the exception: bump the PG version (last argument of `PG_REGISTER*`, e.g.
`PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3)`) in the same change whenever the
parameter group changes: fields added, removed, reordered or resized, array lengths, default values or value ranges
changed. This makes stored settings from older builds reset to the current defaults instead of being misread or
keeping stale defaults. The version is 4 bits (0–15).

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

The ARM toolchain does not need to be installed by hand: `make arm_sdk_install` downloads the version the
Makefile expects into `tools/`.

Rotorflight builds **unified targets**, one firmware per MCU family: `STM32F405`, `STM32F411`, `STM32F7X2`,
`STM32F745`, `STM32G47X`, `STM32H743` (see [make/targets_list.mk](make/targets_list.mk)). Board-specific pin
mappings and defaults are not in this repository; they live in
[rotorflight-targets](https://github.com/rotorflight/rotorflight-targets) and are applied as a config on top of
the unified firmware.

Prefer `STM32F7X2` for test builds.

```
make arm_sdk_install          # once: installs the ARM toolchain under tools/
make TARGET=STM32F7X2         # preferred test target; `make unified` builds all
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
