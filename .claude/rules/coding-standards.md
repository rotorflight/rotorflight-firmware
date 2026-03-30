# Rotorflight Coding Standards

These standards apply to new files and Rotorflight-specific code.
In legacy Cleanflight/Betaflight files, follow the existing style.

---

## Copyright Headers

New Rotorflight files:
```c
/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
```

---

## Header Guards

Use `#pragma once` at the top of every header file, immediately after the copyright header:
```c
/*
 * ... copyright ...
 */

#pragma once
```

Do not use the traditional `#ifndef`/`#define`/`#endif` guard pattern in new files.

---

## File Structure Order

```c
// 1. Copyright header
// 2. C standard library includes
// 3. "platform.h"
// 4. "build/..."
// 5. "common/..."
// 6. "config/..."
// 7. "drivers/..."
// 8. "fc/..."
// 9. "flight/..."
// 10. "io/..."
// 11. "pg/..."
// 12. "local-header.h"  (same directory, last)

// Defines and constants
// Type definitions
// Static variables
// Static helper functions
// Public API functions
```

---

## Indentation and Formatting

- **4 spaces**, never tabs
- Maximum **120 characters** per line
- No trailing whitespace
- Blank lines: 1 between blocks, max 2 between sections, never 3+

### Brace style

**Allman** for function definitions:
```c
void myFunction(int x, data_t *data)
{
    ...
}
```

**K&R** for control structures:
```c
if (condition) {
    ...
}
else {
    ...
}

for (i = 0; i < 10; i++) {
    ...
}
```

The case entries indented:
```c
switch (var) {
    case 0:
        ...
        break;
    default:
        break;
}
```

Single-statement bodies on the same line are acceptable for very short forms:
```c
if (pos > max)
    return max;
```

---

## Naming Conventions

- **Local variables** — `snake_case`: `float gyro_rate;`, `int motor_count;`
- **Module-level static globals** — `camelCase`: `motorOutput`, `servoInput`, `pidUpdateCounter`
- **Functions** — `camelCase`: `pidApplySetpoint()`, `getCurrentPidProfileIndex()`

Common verb patterns:
- `get<Entity>()` / `set<Entity>()` — accessor/mutator
- `has<Entity>()` / `is<State>()` / `are<Entities><State>()` — boolean query
- `<entity>Init()` / `<entity>Shutdown()` / `<entity>Update()` — lifecycle
- `reset<Entity>()` / `validateAndFix<Entity>()` — management

### Macros and constants — `UPPER_CASE_WITH_UNDERSCORES`
```c
#define MAX_SUPPORTED_MOTORS    8
#define MOTOR_OVERRIDE_OFF     -1
#define MOTOR_OVERRIDE_MIN      0
#define MOTOR_OVERRIDE_MAX   1000
```

Group related constants together and align values vertically:
```c
#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_MIN     -700
#define DEFAULT_SERVO_MAX      700
```

Prefix conventions: `DEFAULT_` for reset values, `USE_` for feature flags.

### Types — `_t` suffix (structs), `_e` suffix (enums)
```c
typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef enum {
    TERM_P,
    TERM_I,
    TERM_D,
    TERM_F,
} term_e;
```

Struct tag gets `_s` suffix; typedef drops `_s` and adds `_t`.
Enum typedef gets `_e` suffix. Anonymous enums (for integer constants) omit the typedef:
```c
enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_ITEM_COUNT
};
```

---

## Const Correctness

Pointer parameters that do not modify the pointed-to data must be declared `const`:
```c
void pidApplyProfile(const pidProfile_t *pidProfile);
float calculateRate(const rcControlsConfig_t *config, float input);
```

This lets the compiler catch unintended mutations and communicates intent clearly to the reader.

---

## Data Types

- **No `double`** — use `float` for all floating point
- Float literals **must** have `f` suffix: `3.14159f`, `0.7660444431f`
- Integer constants are allowed as float literals: `3.14159f * 2`
- Use `int` / `unsigned int` when size is irrelevant; use `uint8_t`, `int32_t`, etc. when size matters
- Use `bool` for boolean values

---

## Memory and Performance Attributes

`FAST_DATA_ZERO_INIT` places data in fast RAM, zero-initialized at boot.
Use it for all data that is accessed inside the time critical path.
```c
static FAST_DATA_ZERO_INIT float    motorOutput[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT float    servoInput[MAX_SUPPORTED_SERVOS];
FAST_DATA_ZERO_INIT pidAxisData_t   pidData[XYZ_AXIS_COUNT];
```

`INIT_CODE` marks functions that run rarely and do not need to be optimised for speed.
```c
void INIT_CODE motorInit(void)
{
    motorDevInit();
}

void INIT_CODE validateAndFixServoConfig(void)
{
    ...
}
```

Rules:
- **Always** use static allocation — no `malloc`/`free`/heap
- Stack variables must not exceed **128 bytes**

---

## Use of `goto`

`goto` is permitted **only** for branching to an error recovery path at the end of the current function. This avoids deeply nested cleanup logic while keeping control flow easy to follow:
```c
bool sensorInit(void)
{
    if (!busInit())
        goto error;

    if (!deviceProbe())
        goto error;

    return true;

error:
    busRelease();
    return false;
}
```

Any other use of `goto` (jumping forward past cleanup, jumping between functions, jumping into a block) is not allowed.

---

## Real-time and Concurrency

- Code in ISRs or tight loops **must not** block, log, or allocate
- Data shared between ISR and non-ISR code requires `volatile` and appropriate synchronisation
- Any change to scheduling or critical sections must be reviewed for CPU load and jitter impact
- **No recursion** — recursive calls make stack depth unbounded, which is incompatible with static memory allocation and real-time guarantees

---

## Time Types

```c
timeUs_t currentTimeUs;     // absolute microsecond timestamp (from micros())
timeDelta_t timeout;        // microsecond duration/delta
```

Use `cmp32()` for time comparisons to handle 32-bit wraparound:
```c
if (cmp32(currentTimeUs, motorOverrideTimeout) > 0)
    resetMotorOverride();
```

---

## Feature Guards

Gate optional features with `#ifdef USE_<FEATURE>`:
```c
#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
```

Whole files can be guarded at the top, after `#include "platform.h"`:
```c
#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"
// ... rest of file ...

#endif  // USE_SERVOS
```

Avoid scattering many small `#ifdef` blocks through a single function; prefer wrapping the
whole function if it only makes sense with the feature enabled.

---

## Getter / Setter Patterns

Stored internally as `float` in normalised units; getters convert to integer API units:
```c
// Stored: float in [0..1]  →  returned: int in [0..1000]
int getMotorOutput(uint8_t motor)
{
    return lrintf(motorOutput[motor] * 1000);
}

// Stored: float in [-2..2]  →  returned: int in [-2000..2000]
int getServoOutput(uint8_t servo)
{
    return lrintf(servoOutput[servo]);
}
```

Use `NAN` as a sentinel for "not active" / "no override"; test with `isfinite()`:
```c
bool hasMotorOverride(uint8_t motor)
{
    return isfinite(motorOverride[motor]);
}

int getMotorOverride(uint8_t motor)
{
    return isfinite(motorOverride[motor]) ? lrintf(motorOverride[motor] * 1000) : MOTOR_OVERRIDE_OFF;
}
```

Setters validate input and return the set value or an error sentinel:
```c
int setMotorOverride(uint8_t motor, int value, timeDelta_t timeout)
{
    if (!ARMING_FLAG(ARMED) && motor < MAX_SUPPORTED_MOTORS &&
        value >= MOTOR_OVERRIDE_MIN && value <= MOTOR_OVERRIDE_MAX) {
        motorOverride[motor] = value / 1000.0f;
        motorOverrideTimeout = timeout ? (micros() + timeout) | BIT(0) : 0;
        return value;
    }
    else {
        motorOverride[motor] = NAN;
    }
    return MOTOR_OVERRIDE_OFF;
}
```

---

## Parameter Groups (Persistent Configuration)

Config structs that need to be stored in EEPROM live in `src/main/pg/` and use PG macros.
Any constants or enum values used in the PG structures are defined in the the PG header files.

Header (`pg/servo.h`):
```c
typedef struct {
    int16_t  mid;    // center (mid) point
    int16_t  min;    // lower limit in us from midpoint
    int16_t  max;    // upper limit in us from midpoint
    uint16_t rate;   // servo update rate Hz
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);
PG_DECLARE(servoConfig_t, servoConfig);
```

Implementation uses `RESET_CONFIG` for default initialisation:
```c
void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = { 0 },
    );
}
```

---

## Comments

- Single-line: `//`
- Multi-line: `/* ... */`
- Inline field comments: aligned with spaces, describe units or meaning:
  ```c
  int16_t  mid;    // center (mid) point
  uint16_t rate;   // servo update rate Hz
  ```
- Code comments explain *why* or *what the math means*, not *what the code says*:
  ```c
  // 1.0 == 50° without correction
  float height = constrainf(pos * 0.7660444431f, -1.0f, 1.0f);

  // Scale 50° in rad => 1.0
  float rotation = asin_approx(height) * 1.14591559026f;
  ```

---

## Change Documentation

Any change in the external APIs **must** be documented in `Changes.md`:
- CLI commands (added, removed, renamed, changed defaults)
- MSP protocol messages
- Default configuration values

---

## Static Assertions

Use `STATIC_ASSERT` to catch incorrect assumptions at compile time — struct size constraints, array length expectations, config range checks:
```c
STATIC_ASSERT(sizeof(servoParam_t) == 16, servoParam_t_size_mismatch);
STATIC_ASSERT(PID_PROFILE_COUNT <= 6, too_many_pid_profiles);
```

Prefer `STATIC_ASSERT` over runtime checks for anything that can be known at compile time.

---

## Function Length

Functions should fit within 1–2 screenfuls of text. If a function is longer, it is likely doing too much and should be split into smaller static helpers. This also applies to the review principle of keeping PRs narrowly scoped — a function that cannot be read in one view is harder to reason about for correctness and safety.

---

## Compiler Warnings

The project compiles with `-Werror`. All warnings are errors. Key flags:
- `-Werror=unused-variable` — never declare unused variables
- `-Werror=implicit-function-declaration` — all functions must be declared before use
- `-Werror=return-type` — all non-void functions must return a value
- `-Werror=uninitialized` — variables must be initialised before use
- `-Werror=format` — printf format strings must match argument types
