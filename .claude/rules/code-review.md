# Code Review Checklist

Use this checklist when preparing or reviewing a PR. For flight-critical changes, every item must be explicitly considered.

---

## Code Style

- [ ] 4-space indentation, no tabs
- [ ] Lines ≤ 120 characters
- [ ] Allman braces on function definitions; K&R on control structures
- [ ] No trailing whitespace; no runs of 3+ blank lines
- [ ] `#pragma once` present in all new header files
- [ ] New files have the Rotorflight GPL3 copyright header
- [ ] File section order: includes → defines → types → static variables → static functions → public functions

---

## Naming and Types

- [ ] Variables use `snake_case`; module-level globals and functions use `camelCase`
- [ ] Macros and constants use `UPPERCASE_WITH_UNDERSCORES`
- [ ] Types use `_t` suffix (structs) and `_e` suffix (enums)
- [ ] No magic numbers — named constants used, with units in the name where appropriate
- [ ] `float` used for all floating point; no `double`; all float literals have `f` suffix
- [ ] Fixed-width integer types (`uint8_t`, `int32_t`) used where size matters
- [ ] No unintended signed/unsigned mixing
- [ ] Pointer parameters that don't modify their target are declared `const T *`

---

## Memory Safety

- [ ] No dynamic memory allocation (`malloc`, `free`, VLAs)
- [ ] No stack allocations larger than 128 bytes
- [ ] No recursion
- [ ] All variables initialised before use
- [ ] `FAST_DATA_ZERO_INIT` used for data accessed in the gyro/PID loop
- [ ] `INIT_CODE` used for functions that only run at startup
- [ ] No use of `goto` except branching to an error-recovery label at the end of the function

---

## Numerical Safety

- [ ] No integer overflow in arithmetic — check intermediate results for promotion and wrap-around
- [ ] Bitwise operations are only performed on unsigned types
- [ ] Shifts are not performed on signed types or by amounts ≥ the type width
- [ ] Time comparisons use `cmp32()` to handle 32-bit microsecond wrap-around
- [ ] Floating point comparisons don't use `==` or `!=` directly; tolerance or `isfinite()` used where appropriate
- [ ] Division by zero is not possible — denominators are guarded or clamped
- [ ] Accumulator or integrator values are bounded to prevent wind-up

---

## Real-time and Interrupt Safety

- [ ] No blocking operations, logging, or allocation in ISRs or the gyro/PID loop
- [ ] Data shared between ISR and non-ISR code is declared `volatile`
- [ ] Accesses to multi-byte shared variables are protected by critical sections or are atomic
- [ ] Any change to task scheduling, loop rates, or critical sections has been reviewed for CPU load and jitter
- [ ] No new polling loops added to timing-sensitive paths without justification
- [ ] ISR handlers are short — no complex logic, no loops with variable bounds

---

## Hardware and Platform

- [ ] No endianness assumptions without explicit byte-swapping
- [ ] `__attribute__((packed))` used only where unavoidable and alignment consequences are understood
- [ ] Bit fields are not used (undefined layout across compilers)
- [ ] Hardware resource usage (timers, DMA channels, pins) does not conflict with existing assignments in the target
- [ ] DMA buffers are correctly sized and aligned; cache coherency considered on H7 targets
- [ ] Peripheral initialisation order dependencies are respected (e.g., bus before device)
- [ ] Feature is correctly gated with `#ifdef USE_<FEATURE>` and verified to compile with and without it

---

## Flight Safety

- [ ] All external inputs (CLI, MSP, RC, sensors) are validated and clamped to safe ranges before use
- [ ] Failsafe and arming logic is not inadvertently bypassed or weakened
- [ ] Control gains, limits, and default values are within safe operating ranges
- [ ] Error conditions are surfaced to the user where appropriate (OSD, MSP, blackbox)
- [ ] No change silently alters behaviour for existing configurations (or it is explicitly documented)
- [ ] `STATIC_ASSERT` used to guard any struct size or array length assumptions that the code depends on

---

## Documentation and API Changes

- [ ] CLI command changes (add, remove, rename, default change) documented in `Changes.md`
- [ ] MSP protocol changes documented in `Changes.md`
- [ ] Default configuration value changes documented in `Changes.md`
- [ ] Complex or non-obvious logic has explanatory comments (the *why*, not the *what*)
- [ ] Public API headers are minimal — no internal globals or implementation details exposed

---

## PR Hygiene

- [ ] PR has a clear, single-scope title and description
- [ ] No unrelated changes bundled in (formatting, renaming, drive-by fixes)
- [ ] PR description states whether the change is flight-critical
- [ ] Testing performed is described (bench only / hover / full flight, heli type if relevant)
- [ ] `make ci` passes — builds all CI targets successfully, confirming no target-specific compilation failures
- [ ] Relevant unit tests pass (`make test`); new logic has test coverage where feasible
