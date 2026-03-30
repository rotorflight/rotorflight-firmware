# CLAUDE.md

This file provides guidance to AI coding agents working with this repository.

## Project Overview

Rotorflight is a safety-critical embedded C firmware for single-rotor RC helicopters, forked from Betaflight 4.3.
It targets STM32F4/F7/H7 MCUs (and others). Code quality and correctness are paramount — this firmware controls moving rotors.

## Build Commands

Full build reference is in [`build-guide.md`](rules/build-guide.md).

```bash
make                     # Build default target (STM32F7X2)
make TARGET=STM32F745    # Build a specific target
make ci                  # Build all CI targets
make clean               # Delete build artefacts
make arm_sdk_install     # Install ARM toolchain
make dfu_flash           # Flash via DFU
```

Key `make` variables: `TARGET`, `DEBUG` (empty/INFO/GDB), `V` (0/1 for verbosity).

Build output goes to `obj/`.

## Testing

```bash
# Run all unit tests
make test

# Run a single test
make test_maths_unittest
```

Tests live in `src/test/unit/` and use Google Test. Test files are `*_unittest.cc` with `*_stubs.c` for dependencies.

## Static Analysis

```bash
make cppcheck
```

The project compiles with `-Werror` — all warnings are errors. Zero tolerance.

## Architecture

Full architecture reference (data flow, subsystems, entry points, source layout) is in [`architecture.md`](rules/architecture.md).
Domain vocabulary (swashplate, governor, CCPM, eRPM, etc.) is in [`glossary.md`](rules/glossary.md).

## C Coding Standards

Full coding standards are in [`coding-standards.md`](rules/coding-standards.md).
Common mistakes and gotchas are in [`common-pitfalls.md`](rules/common-pitfalls.md).

Key points:
- 4 spaces, never tabs. Max 120 characters per line.
- Allman braces for function definitions, K&R for control structures.
- Variables: `snake_case`; functions: `camelCase`; macros/constants: `UPPERCASE_WITH_UNDERSCORES`; types: `_t` suffix.
- `float` only — never `double`. Float literals must have `f` suffix: `3.14159f`.
- Static allocation only — no `malloc`/`free`. Stack variables ≤ 128 bytes.
- No recursion. ISRs must not block, allocate, or log.
- New files need the Rotorflight GPL3 copyright header.

## Change Documentation

Any change to CLI commands, MSP protocol, or default values **must** be documented in `Changes.md`.

## Where to Start

| Task | Reference |
|---|---|
| Add / change a CLI command | `architecture.md` § CLI → `coding-standards.md` → `Changes.md` |
| Add / change MSP messages | `architecture.md` § MSP Protocol → `Changes.md` |
| Change PID / flight control | `architecture.md` § Critical Path, `glossary.md` |
| Add a new target or board | `build-guide.md` |
| Add a new RX protocol | `architecture.md` § RX Subsystem |
| Add a new ESC protocol | `architecture.md` § ESC Protocols |
| Review or prepare a PR | `code-review.md`, `common-pitfalls.md`, `pr-review.md` |

## PR Guidelines

Full review checklist is in [`code-review.md`](rules/code-review.md).

- Each PR must have a clear, limited scope. Do not mix unrelated changes.
- Describe whether the change is flight-critical (affects control loop, sensor pipelines, failsafes, arming, power management, or timing).
- State testing performed (bench only / hover test / full flight, including heli type).
- Do not make drive-by formatting or renaming changes unless strictly necessary for the PR.
