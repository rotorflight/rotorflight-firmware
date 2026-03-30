# Rotorflight Architecture Reference

Definitive architecture guide for AI agents making changes. Read this before exploring
the codebase for any non-trivial task.

See also: [`coding-standards.md`](coding-standards.md) for C style rules and [`code-review.md`](code-review.md) before submitting a PR.

---

## System Overview

```
main() → init() → scheduler loop (runs forever)
```

- Entry point: `src/main/build/main.c` → `init()` in `src/main/fc/init.c`
- All work after boot happens inside the scheduler; there is no main loop beyond it
- The scheduler is purely cooperative/priority-based — no RTOS, no threads

---

## Source Tree (with roles)

```
src/
├── main/
│   ├── blackbox/   # Flight data logger (writes to SD card or flash)
│   ├── build/      # Build configuration headers (version, debug, MCU IDs)
│   ├── cli/        # Text CLI (activated by '#' on serial port)
│   ├── cms/        # On-device configuration menu (OSD-driven)
│   ├── common/     # Shared utilities: maths, filters, printf, circular buffers
│   ├── config/     # EEPROM load/save, factory reset, feature bit flags
│   ├── drivers/    # Hardware abstraction: SPI, I2C, UART, timers, DMA, GPIO
│   ├── fc/         # Flight controller core: init, arming, RC processing, tasks
│   ├── flight/     # Control algorithms: PID, mixer, governor, IMU, failsafe
│   ├── io/         # Higher-level I/O: serial mux, beeper, dashboard, GPS, LED
│   ├── msc/        # USB Mass Storage Class (SD card access over USB)
│   ├── msp/        # MSP protocol — Configurator communication
│   ├── osd/        # On-screen display rendering
│   ├── pg/         # Parameter groups — all persistent config lives here
│   ├── rx/         # RC receiver protocols: CRSF, SBUS, DSM, FPort, ELRS, etc.
│   ├── scheduler/  # Real-time cooperative task scheduler
│   ├── sensors/    # Sensor fusion: gyro pipeline, accel, compass, baro, battery
│   ├── startup/    # MCU startup / reset handlers
│   ├── target/     # Per-board definitions (target.h, pin maps, peripheral assigns)
│   ├── telemetry/  # Outbound telemetry: CRSF, S.Port, HoTT, GHST, etc.
│   └── vcp*/       # USB Virtual COM Port drivers (vcp/, vcpf4/, vcp_hal/)
├── link/           # Linker scripts (.ld files per MCU family)
└── test/unit/      # Google Test unit tests (*_unittest.cc + *_stubs.c)
```

---

## The Critical Path: Gyro → PID → Outputs

This loop runs at ~8 kHz (gyro sample rate). Everything else is secondary.

```
TASK_GYRO        reads gyro ADC, applies decimation
     ↓
TASK_FILTER      LPF1 → LPF2 → notch filters → RPM notch → dynamic notch
     ↓           (result: gyro.gyroADCf[])
TASK_PID         pidController()
     ├── reads: gyroADCf[], setpoint[], attitude
     ├── computes: P + I + D + F + B + O per axis
     ├── runs governor (headspeed control → throttle)
     └── calls mixerUpdate()
          ├── maps PID outputs + RC inputs → servo positions
          ├── maps governor output → motor throttle
          └── writes to servoOutput[] / motorOutput[]
               ↓
          TASK_SERVO / motor drivers → PWM / DShot hardware
```

Key files:
- `sensors/gyro.c` — gyro pipeline, filter chain
- `flight/pid.c` — PID controller (`pidController()`)
- `flight/governor.c` — headspeed governor
- `flight/mixer.c` — mixer matrix
- `flight/servos.c` — servo output
- `flight/motors.c` — motor output

---

## Scheduler and Task Rates

**File**: `scheduler/scheduler.c`, task definitions in `fc/tasks.c`

Tasks are defined as `task_attribute_t` entries with a desired period and priority.
The scheduler ages dynamic priority to prevent starvation.

| Task | Rate | Priority | Purpose |
|---|---|---|---|
| TASK_GYRO | ~8 kHz | REALTIME | Gyro sampling |
| TASK_FILTER | ~8 kHz | REALTIME | Gyro filtering chain |
| TASK_PID | ~8 kHz | REALTIME | PID + governor + mixer |
| TASK_ACCEL | 1 kHz | HIGH | Accelerometer read |
| TASK_ATTITUDE | 500 Hz | HIGH | IMU attitude estimate |
| TASK_DISPATCH | 1 kHz | HIGH | Deferred function queue |
| TASK_RX | ~33 Hz (event-driven) | HIGH | RC input processing |
| TASK_SERIAL | 100 Hz | MEDIUM | Serial port I/O |
| TASK_MAIN | 1 kHz | MEDIUM | SD card / flash |
| TASK_TELEMETRY | 250 Hz | MEDIUM | Outbound telemetry |
| TASK_OSD | display rate | MEDIUM | OSD rendering |
| TASK_BATTERY_* | 50 Hz | LOW | Voltage/current monitoring |
| TASK_COMPASS | 10 Hz | LOW | Magnetometer |
| TASK_BARO | 20 Hz | LOW | Barometer |

**Rule**: Any new code that runs at gyro/PID rate must not block, allocate, or log.
It must use `FAST_DATA_ZERO_INIT` for its working data.

---

## RC Input Processing

```
Receiver hardware → rx/ protocol driver → rxFrameComplete()
     ↓
TASK_RX  →  rxUpdateCheck()  →  processRxChannels()
     ↓
rcRaw[] (raw 1000–2000 µs)  →  rcCommand[] (normalized -500..+500)
     ↓
fc/rc.c: updateRcCommands()
     ↓
setpoint.c: setpointUpdate()  →  setpoint[] consumed by pidController()
```

Channel assignments in `rx/rx.h`:
- `ROLL=0`, `PITCH=1`, `YAW=2`, `COLLECTIVE=3`, `THROTTLE=4`, aux channels 5+

**TASK_RX** is event-driven — it runs when the receiver signals a new frame, not on a
fixed timer. Rate depends on the link protocol (CRSF: up to 500 Hz, SBUS: 100 Hz, etc.).

---

## Arming State Machine

**File**: `fc/core.c`

Key functions:
- `updateArmingStatus()` — checks all arming conditions, sets/clears `ARMING_FLAG(ARMED)`
- `tryArm()` — attempts to arm; runs all safety checks
- `disarm(reason)` — disarms with a logged reason

`ARMING_FLAG(ARMED)` is the authoritative armed state. Motor and servo outputs are gated
on this flag. Do not bypass it.

Arming is blocked by:
- RC not connected / signal lost
- Gyro calibration not complete
- Throttle not at minimum
- Any active `armingDisableFlags` bit

---

## PID Controller Detail

**File**: `flight/pid.c`, declarations in `flight/pid.h`

```c
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);
```

Axes: `FD_ROLL=0`, `FD_PITCH=1`, `FD_YAW=2`

Terms computed per axis:
- **P**: `Kp × error`
- **I**: integrated error with i-term relax and wind-up limits
- **D**: `Kd × filtered(d/dt error)` via biquad D-term filter
- **F**: `Kf × setpoint_rate` (feed-forward, open loop)
- **B**: boost term for yaw snap
- **O**: static offset

Output `pidData[axis].Sum` feeds the mixer.

Helicopter-specific additions:
- Collective-to-yaw feed-forward (`collectiveFF`) — preempts torque change from collective
- Cross-coupling compensation between main and tail
- Inertia precompensation

---

## Governor

**File**: `flight/governor.c`, declarations in `flight/governor.h`

Closed-loop headspeed control. Runs inside `TASK_PID`.

- Input: RPM from sensor or bidirectional DShot eRPM (converted via gear ratio)
- Target: configured headspeed (rpm) from governor profile
- Output: throttle command to mixer (`MIXER_IN_RC_COMMAND_THROTTLE`)
- ESC telemetry RPM is **not** suitable for the governor (too slow); requires dedicated RPM
  sensor or bidirectional DShot

---

## Mixer

**File**: `flight/mixer.c`, declarations in `flight/mixer.h`

The mixer maps named inputs to outputs via a weight matrix.

**Inputs** (`mixerInput_e`):
- `MIXER_IN_RC_COMMAND_ROLL/PITCH/YAW/COLLECTIVE/THROTTLE` (0–4): raw RC
- `MIXER_IN_STABILIZED_ROLL/PITCH/YAW` (5–7): PID output

**Output indexing**:
```c
#define MIXER_SERVO_OFFSET   1
#define MIXER_MOTOR_OFFSET   (1 + MAX_SUPPORTED_SERVOS)
// output[0]             = (unused/sync)
// output[1..8]          = servos (swashplate + tail servo if present)
// output[9..10]         = motors (main + tail)
```

Saturation tracking feeds back into I-term limiting in the PID.

---

## Parameter Group (PG) System

**Files**: `pg/pg.h`, IDs in `pg/pg_ids.h`, per-subsystem structs in `pg/*.h`

All persistent config is stored as PG structs. They are automatically serialised to EEPROM
on `mspFcProcessCommand(MSP_EEPROM_WRITE)` or `cli: save`.

### Declaring a PG

**Header** (`pg/my_module.h`):
```c
#pragma once
#include "pg/pg.h"

typedef struct {
    uint16_t myParam;
    uint8_t  myFlag;
} myConfig_t;

PG_DECLARE(myConfig_t, myConfig);
```

**Implementation** (`my_module.c`):
```c
PG_REGISTER_WITH_RESET_TEMPLATE(myConfig_t, myConfig, PG_MY_CONFIG, 0);

const pgRegistry_t myConfig_Registry PG_REGISTER_ATTRIBUTES = { ... };

// Default values:
static const myConfig_t myConfigDefaults = {
    .myParam = 100,
    .myFlag  = 1,
};
```

Add `PG_MY_CONFIG` to `pg/pg_ids.h`.

**Access** (anywhere):
```c
#include "pg/my_module.h"
uint16_t v = myConfig()->myParam;
```

**Mutable access** (during init or CLI set):
```c
myConfigMutable()->myParam = newValue;
```

### Constants and enum values used in PG structs go in the PG header, not in the
implementation file.

---

## MSP Protocol

**Files**: `msp/msp.c`, `msp/msp_protocol.h`, `msp/msp_serial.c`

MSP is the binary protocol used by Rotorflight Configurator over USB/UART.

### Request dispatch chain (in `mspFcProcessCommand()`):
1. `mspCommonProcessOutCommand()` — shared Betaflight/Rotorflight queries
2. `mspProcessOutCommand()` — Rotorflight-specific read handlers
3. `mspFcProcessOutCommandWithArg()` — handlers needing extra argument
4. `mspCommonProcessInCommand()` — write handlers

### Adding a new MSP handler

1. Add command ID to `msp/msp_protocol.h`:
   ```c
   #define MSP_MY_COMMAND  250
   ```

2. Add handler in `msp/msp.c`:
   ```c
   case MSP_MY_COMMAND:
       sbufWriteU16(dst, myValue);
       return MSP_RESULT_ACK;
   ```
   For a setter, add to the `default:` / write-command section and use `sbufReadU16(src)`.

3. Document in `Changes.md`.

Payload is read/written via `sbuf_t *src` / `sbuf_t *dst` using `sbufRead*` / `sbufWrite*`
helpers. Never read past the end of `src` — check `sbufBytesRemaining(src)` first.

---

## CLI

**Files**: `cli/cli.c`, `cli/settings.c`

CLI activates when `#` is received on a serial port. All commands are lower-case strings.

### Adding a new CLI command

1. Define handler in `cli/cli.c`:
   ```c
   static void cliMyCommand(const char *cmdline)
   {
       // parse cmdline; use cliPrintf() for output
   }
   ```

2. Add entry to the `cmdTable[]` array (alphabetical order):
   ```c
   { "mycommand", "description", cliMyCommand },
   ```

3. Document in `Changes.md`.

**Existing built-ins to be aware of**: `set`, `get`, `dump`, `defaults`, `save`, `status`,
`tasks`, `version`, `mixer`, `servo`, `motor`, `feature`, `map`.

`cli/settings.c` auto-generates `set`/`get` for all PG fields that are declared in the
`settingTable[]`. Adding a PG field to `settingTable[]` makes it available as `set <name>`.

---

## RX (Receiver) Subsystem

**Files**: `rx/rx.c`, protocol-specific files in `rx/`

### Adding a new serial RX protocol

1. Create `rx/myprotocol.c` and `rx/myprotocol.h`
2. Implement frame parser; call `rxRxFrameReceived()` when a frame is complete
3. Add to `SerialRXType` enum in `rx/rx.h`
4. Add init case to `rxInit()` in `rx/rx.c`
5. Add PG config entry if the protocol needs parameters
6. Gate with `#ifdef USE_SERIALRX_MYPROTOCOL`

---

## Telemetry

**Files**: `telemetry/telemetry.c`, per-protocol files in `telemetry/`

Telemetry runs in `TASK_TELEMETRY`. Each protocol (CRSF, S.Port, HoTT, GHST, etc.) has
its own handler file.

### Adding a new telemetry sensor value

For CRSF (most common in new designs):
- `telemetry/crsf.c` — add new sensor frame type and populate from flight data

For FrSky S.Port:
- `telemetry/smartport.c` — add sensor ID and value function

For all protocols, sensor data is pulled from `sensors/`, `flight/`, `fc/` via their
public accessor functions — do not read hardware directly from telemetry code.

---

## Sensor Drivers (drivers/accgyro/)

**Pattern**: Each sensor IC has a `*_init()` function that populates a `gyroDev_t` struct
with function pointers (`read`, `intStatus`, etc.). The generic `gyro.c` calls through
these pointers — it never calls the IC driver directly after init.

To add a new IMU:
1. Create `drivers/accgyro/accgyro_myimu.c`
2. Implement `myImuGyroDetect()` → fills `gyroDev_t`
3. Add detect call to `gyroInit()` in `sensors/gyro.c`
4. Gate with `USE_GYRO_MYIMU`

Same pattern applies to barometers (`drivers/barometer/`) and compasses
(`drivers/compass/`).

---

## ESC Protocols / Motor Output

**Files**: `drivers/motor.h`, `drivers/pwm_output.c`, `drivers/dshot.c`

Motor protocol is selected at runtime from `motorConfig()->motorProtocol`.

To add a new ESC protocol:
1. Add enum value to `motorProtocol_e` in `drivers/motor.h`
2. Implement frame encoding
3. Add init case in `motorDevInit()` in `drivers/motor.c`
4. Gate with `USE_MY_ESC_PROTOCOL`
5. If it carries RPM feedback, integrate with `sensors/rpm_filter.c`

---

## Flight Modes

**Files**: `fc/rc_modes.h` (mode definitions), `fc/rc_modes.c` (activation logic),
`flight/leveling.c`, `flight/rescue.c`, `flight/trainer.c` (mode implementations)

### Adding a new flight mode

1. Add to `boxId_e` enum in `fc/rc_modes.h`
2. Add `boxEntry` in the `boxes[]` table in `fc/rc_modes.c`
3. Implement mode logic in `flight/`; check activation with `IS_RC_MODE_ACTIVE(BOXMYMODE)`
4. Call from appropriate task (usually `TASK_PID` via `fc/core.c`)

---

## Feature Flags

Two levels of feature gating:

**Compile-time** (`USE_*` defines in `target/target.h` or Makefile):
```c
#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
```
These control whether code is compiled in at all.

**Runtime** (`FEATURE_*` bits in `config/feature.h`):
```c
if (featureIsEnabled(FEATURE_TELEMETRY)) { ... }
```
These can be toggled by the user. A feature must be both compiled in (`USE_*`) and enabled
at runtime (`FEATURE_*`) to be active.

---

## Startup / Init Sequence

`fc/init.c:init()` runs once at boot in this order:
1. Clock and memory init (platform)
2. Scheduler init
3. Config load from EEPROM (or defaults if corrupt/new)
4. Sensor detection and driver init
5. Filter chain setup
6. Mixer and output init
7. RC receiver init
8. Telemetry / MSP / CLI init
9. Task registration → hand off to `scheduler()`

Functions that only run during init should be marked `INIT_CODE` to allow the linker to
discard them after boot (saving RAM on constrained targets).

---

## Change Documentation Rule

Any change that affects the external surface **must** be documented in `Changes.md`:
- CLI command added, removed, renamed, or default changed
- MSP command added or changed
- Arming/failsafe behaviour changed

This is mandatory, not optional.
