# Rotorflight Build Guide

Reference for building the firmware and using the Makefile.

---

## Toolchain Setup

```bash
make arm_sdk_install   # Download and install the ARM GCC toolchain into the repo
make arm_sdk_clean     # Remove the installed toolchain
```

The toolchain is installed locally under the repo; no system-wide installation is required.

---

## Core Build Commands

```bash
make                         # Build the default target (STM32F7X2)
make TARGET=STM32F745        # Build a specific target
make TARGET=STM32F745 V=1    # Build with verbose compiler output
make ci                      # Build all CI targets (all BASE_TARGETS minus excluded)
make clean                   # Delete all build artefacts in obj/
make clean_all               # Clean every target
make TARGET=STM32F745_clean  # Clean one specific target
```

Build output goes to `obj/`. Binaries land in `obj/<TARGET>.hex` / `obj/<TARGET>.bin`.

### Valid TARGET values

```
APM32F405  APM32F407  AT32F435G  AT32F435M
STM32F405  STM32F411  STM32F446  STM32F745  STM32F7X2
STM32G47X  STM32H563  STM32H723  STM32H725  STM32H730
STM32H735  STM32H743  STM32H750
RP2350A    RP2350B    SITL
```

Run `make targets` to get the current list (useful for scripting).

---

## Key Make Variables

| Variable | Default | Purpose |
|---|---|---|
| `TARGET` | *(none — falls back to `STM32F7X2`)* | Target MCU board |
| `CONFIG` | *(none)* | Board config overlay (enables `USE_CONFIG`) |
| `DEBUG` | *(empty)* | `INFO` = debug symbols + optimisations; `GDB` = minimal optimisations |
| `EXST` | `no` | `yes` = build for External Storage Bootloader |
| `RAM_BASED` | `no` | `yes` = image loaded into RAM |
| `FLASH_SIZE` | *(auto)* | Override flash size in KB |
| `V` | `0` | Verbosity: `0` = quiet, `1` = full compiler commands |
| `OPTIONS` | *(empty)* | Extra compile-time feature flags |
| `EXTRA_FLAGS` | *(empty)* | Arbitrary extra CFLAGS |
| `PARALLEL_JOBS` | `4` | `-j` value used for parallel sub-makes |
| `USE_PRINTF` | *(empty)* | `NULL` / `ITM` / `SERIAL` — printf debug channel |
| `USE_ITM_DEBUG` | `no` | Enable ITM real-time debugging |
| `FLASH_CONFIG_ERASE` | `no` | Generate image that erases EEPROM config on boot |

---

## Flashing

```bash
make TARGET=STM32F745_flash   # Build and flash via the default method
make dfu_flash                # Flash .bin via DFU (USB)
make tty_flash                # Flash .hex via serial port (SERIAL_DEVICE auto-detected)
make st-flash                 # Flash .bin via ST-Link / OpenOCD
make unbrick                  # Emergency unbrick procedure
```

`SERIAL_DEVICE` defaults to the first `/dev/ttyACM*` or `/dev/ttyUSB*` found.

---

## Testing

```bash
make test                     # Run the full unit-test suite
make test-representative      # Run a representative subset (faster; one expansion per test)
make test-all                 # Run all tests including all per-target expansions
make junittest                # Run tests and emit JUnit XML in obj/test/
make test_clean               # Remove test build artefacts
make test_help                # List available individual tests
make test_versions            # Print compiler versions used for tests
make test_<name>              # Run a single test, e.g. make test_maths_unittest
```

Tests live in `src/test/unit/`. They use Google Test. Each test file is
`*_unittest.cc`; stubs for firmware dependencies live in `*_stubs.c`.

---

## Static Analysis

```bash
make cppcheck
```

The firmware compiles with `-Werror` — every warning is a build error.

---

## Release / Distribution

```bash
make TARGET=STM32F745_zip    # Build target and create a distributable .zip
make TARGET=STM32F745_rev    # Build and include git revision in the filename
make version                 # Print firmware version string
make all_rev                 # Build all CI targets with revision in filenames
```

---

## Configs (Board Overlays)

Some boards ship with a `CONFIG` overlay that customises a base target:

```bash
make configs                 # Populate/refresh available config targets
make CONFIG=<config>         # Build a config-based target
make <config>_clean          # Clean a config-based target
```

---

## Local Developer Overrides

Create `mk/local.mk` (gitignored) to override variables without touching the
main Makefile. For example:

```makefile
PARALLEL_JOBS := 8
DEFAULT_TARGET := STM32H743
```

---

## Useful Introspection Targets

```bash
make help               # Print the full help message
make targets            # Machine-readable list of all valid targets
make target-mcu         # Print MCU type for the current TARGET
make targets-by-mcu     # Build all targets matching a given MCU_TYPE
make targets-ci-print   # Print the list of CI targets (space-separated)
```
