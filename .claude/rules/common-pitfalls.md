# Common Pitfalls

Mistakes that AI agents and new contributors make repeatedly. Read this before writing or
reviewing code. Each section explains the rule, shows the correct pattern from the codebase,
and explains what goes wrong if you get it wrong.

---

## 1. Float Only — Never Double

All floating-point arithmetic uses `float` (32-bit). The firmware has no double-precision
support on most targets and `-Werror` will catch some cases, but not all.

### Math functions must use the `f` suffix variants

```c
// WRONG — calls double-precision libm, silently promotes and truncates
float x = sin(angle);
float y = sqrt(val);
float z = fabs(err);
int n = lrint(val);

// RIGHT — single-precision throughout
float x = sin_approx(angle);   // or sinf(angle)
float y = sqrtf(val);
float z = fabsf(err);
int n = lrintf(val);
```

The codebase provides approximation wrappers in `common/maths.h` — prefer these in
flight-critical code:
```c
sin_approx()  cos_approx()  tan_approx()  asin_approx()
acos_approx() atan2_approx() exp_approx()  log_approx()
```

These map to either fast polynomial approximations (`USE_FAST_MATH`) or to the `f`-suffix
libm functions (`USE_STANDARD_MATH`). Never call the unwrapped `sin()`, `cos()`, etc.

### Float literals must have the `f` suffix

```c
// WRONG — 3.14159 is a double literal; implicit narrowing conversion
float angle = 3.14159;
float scale = val * 0.001;

// RIGHT
float angle = 3.14159f;
float scale = val * 0.001f;
```

Many standard constants are already defined in `common/maths.h`:
```c
#define M_PIf           3.14159265358979323846f
#define M_2PIf          6.28318530717958647693f
#define M_RADf          0.01745329251994329577f
```

---

## 2. FAST_DATA_ZERO_INIT and INIT_CODE Placement

On STM32F7/H7 targets, `FAST_DATA_ZERO_INIT` places data in DTCM/ITCM RAM (fast,
single-cycle access). `INIT_CODE` marks functions that are rarely run and can be
optimised for size instead of speed.

### When to use FAST_DATA_ZERO_INIT

Any module-level static variable that is read or written in the gyro/PID/mixer loop:
```c
// flight/pid.c
static FAST_DATA_ZERO_INIT pidData_t pid;

// flight/motors.c
static FAST_DATA_ZERO_INIT float motorOutput[MAX_SUPPORTED_MOTORS];

// flight/mixer.c
static FAST_DATA_ZERO_INIT mixerData_t mixer;
```

**Do not use it for**: configuration data (PG structs), init-only data, CLI/MSP buffers,
telemetry state, or anything not on the critical path. Fast RAM is scarce.

### When to use INIT_CODE

Functions that only run during `init()` or profile changes:
```c
void INIT_CODE mixerInit(void) { ... }
void INIT_CODE pidLoadProfile(const pidProfile_t *pidProfile) { ... }
void INIT_CODE validateAndFixMixerConfig(void) { ... }
```

**Do not use it for**: functions called regularly, e.g. in the PID loop.
`INIT_CODE` applies `-Os` optimisation which may be slower in hot paths.

---

## 3. Timer Comparisons — Always Use cmp32()

32-bit microsecond timers wrap around at ~71 minutes (`timeUs_t`).
Naive subtraction produces wrong results at wraparound.

```c
// WRONG — breaks at 32-bit wraparound
if (currentTimeUs - lastTimeUs > timeout) { ... }

// RIGHT — handles wraparound correctly
if (cmp32(currentTimeUs, lastTimeUs) > timeout) { ... }
```

Real usage from `flight/governor.c`:
```c
return cmp32(millis(), gov.stateEntryTime);
```

---

## 4. MSP sbuf — Check Remaining Bytes Before Reading

The `sbuf_t` stream buffer has **no bounds checking**. Reading past the end corrupts data
silently. Always check `sbufBytesRemaining()` before conditional reads.

### Write order must match Configurator read order

MSP is a positional binary protocol. The Configurator reads fields in exact order. If you
add a field, it must go at the **end** of the existing payload. Inserting a field in the
middle breaks all existing Configurator versions.

---

## 5. Lookup Tables Must Stay in Sync with Enums

CLI settings use lookup tables (string arrays) indexed by enum values. The array index
**must** match the enum value exactly — there is no runtime bounds check.

```c
// cli/settings.c
// sync with accelerationSensor_e          <-- this comment is critical
const char * const lookupTableAccHardware[] = {
    "AUTO", "NONE", "ADXL345", "MPU6050", ...
};
```

If someone adds a value to the enum but forgets the lookup table (or vice versa), the
wrong string maps to the wrong enum value. Every lookup table has a `// sync with` comment
identifying its corresponding enum — check both sides when making changes.

---

## 6. NAN Sentinel — Never Compare with ==

The codebase uses `NAN` as a sentinel for "not set" / "not known".
IEEE 754 defines `NAN != NAN`, so direct comparison always fails.

```c
// WRONG — always evaluates to false
if (value == NAN) { ... }
if (value != NAN) { ... }

// RIGHT
if (!isfinite(value)) { ... }   // true for NAN and INF
if (isnan(value)) { ... }       // true only for NAN
```

Real patterns from the codebase:
```c
bool hasMotorOverride(uint8_t motor)
{
    return isfinite(motorOverride[motor]);
}
```

---

## 7. Feature Guards — Missing #ifdef Causes Linker Errors

Optional features are gated with `#ifdef USE_<FEATURE>`. If you call a function that only
exists inside a feature guard, the code **compiles** but fails at **link time** with an
unhelpful "undefined reference" error.

### File-level guard pattern

When an entire file only makes sense with a feature:
```c
#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"
// ... entire file ...

#endif  // USE_SERVOS
```

### Inline guard pattern

```c
#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
```

### Common mistake

Adding a call to a feature-gated function without wrapping the call site:
```c
// This compiles on targets where USE_DSHOT is defined but fails on others
dshotSetPidLoopTime(pidLooptime);   // WRONG if not inside #ifdef USE_DSHOT
```

After adding any `#ifdef`-gated code, verify it builds both ways.

---

## 8. PG Registration and Versioning

Parameter Groups store persistent config in EEPROM. Each PG has a numeric ID and a
version number (0–15, stored in the top 4 bits of the PGN).

```c
PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);
//                                                       ^^ ID          ^^ version
```

### When to bump the version

Bump when the struct layout changes in a way that makes old EEPROM data invalid:
- Adding, removing, or reordering fields
- Changing field types or sizes

### What happens if you don't bump

The firmware loads old EEPROM data into the new struct layout. Fields are misaligned.
Values are silently corrupt. On a flight controller, this can mean incorrect PID gains,
wrong servo limits, or broken failsafe settings.

### What happens on version mismatch

The PG is reset to compiled-in defaults. This is safe but means the user loses their
tuning. Document version bumps clearly so users know to re-enter settings.

### IDs are in pg/pg_ids.h

New PGs need a new ID added to `pg/pg_ids.h`. IDs must be unique and must never be reused.

---

## 9. volatile — Only for ISR-Shared Data

`volatile` tells the compiler not to cache a variable in registers. It is required when
data is written by an ISR and read by main code (or vice versa), but harmful when overused
because it defeats optimisation.

### Correct uses

```c
// drivers/serial_uart_impl.h — ISR writes to buffer, main loop reads
volatile uint8_t *txBuffer;
volatile uint8_t *rxBuffer;

// drivers/adc_impl.h — ADC DMA ISR writes, main loop reads
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

// drivers/light_ws2811strip.h — DMA completion ISR sets flag
extern volatile bool ws2811LedDataTransferInProgress;
```

### Common mistake

Marking gyro or PID data as `volatile`. These are **not** ISR-shared — the scheduler
ensures they are only accessed from one context at a time. Adding `volatile` to hot-path
data prevents the compiler from keeping values in registers and measurably increases PID
loop time.

---

## 10. constrain() vs constrainf()

Two separate clamping functions exist — one for integers, one for floats. Using the wrong
one causes silent implicit conversions.

```c
// common/maths.h
static inline int constrain(int value, int low, int high);
static inline float constrainf(float value, float low, float high);
```

```c
// WRONG — float truncated to int, clamped, then promoted back
float rate = constrain(gyroRate, -500.0f, 500.0f);

// RIGHT
float rate = constrainf(gyroRate, -500.0f, 500.0f);

// Integer values use constrain()
int channel = constrain(input, 0, 15);
```

---

## 11. Copyright Headers — Match the File's Origin

The codebase has two copyright headers. Which one to use depends on the file's lineage:

| Header | When to use |
|---|---|
| **Rotorflight** | New files, or files substantially rewritten for Rotorflight |
| **Cleanflight and Betaflight** | Existing upstream files with minor modifications |

Do not change an existing file's copyright header unless you are substantially rewriting
it. The header indicates code lineage and legal attribution.

---

## 12. Disabled Tests (.cc.txt Convention)

Tests are disabled by renaming from `.cc` to `.cc.txt`. The build system ignores `.txt`
files. Currently 27 of 51 test files are disabled.

- Do **not** delete disabled tests — they contain valid test logic
- Do **not** blindly re-enable them — they typically need stub updates
- If you modify code covered by a disabled test, check whether the test can be fixed
- New code should have an enabled test where feasible

---

## 13. Static Allocation — No malloc, No VLAs

The firmware uses only static allocation. No heap. No `malloc`/`free`/`calloc`/`realloc`.
No variable-length arrays (VLAs).

```c
// WRONG
float *buffer = malloc(n * sizeof(float));
float values[channelCount];   // VLA — stack size unknown at compile time

// RIGHT
static float buffer[MAX_CHANNELS];
float values[MAX_SUPPORTED_RC_CHANNEL_COUNT];
```

Stack variables must not exceed 128 bytes. Large data goes in module-level statics.

---

## 14. No Recursion

Recursive calls make stack depth unbounded, which is incompatible with the fixed-size
stack on embedded targets. Use iterative approaches instead. This includes indirect
recursion (A calls B calls A).
