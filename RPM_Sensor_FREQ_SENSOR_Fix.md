# RPM Sensor (FREQ_SENSOR) Enable Fix — CH32H417

## Summary

|                |                                                                                                                                                                         |
| -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Date**       | 2026-09-11                                                                                                                                                              |
| **Symptom**    | The "RPM sensor" toggle in Rotorflight Configurator → Motors tab reverts to **disabled** after Save & Reboot                                                            |
| **Root cause** | `USE_FREQ_SENSOR` was not compiled in for the `CH32H417` target, so the firmware silently force-disabled the `FREQ_SENSOR` feature on every boot and every EEPROM write |
| **Fix**        | Define `USE_FREQ_SENSOR` for the target and port the freq sensor driver's STM32-only timer code to the CH32H41x SPL                                                     |
| **Status**     | Fixed — full build `make TARGET=CH32H417` passes                                                                                                                        |

---

## 1. The Problem

When enabling **RPM sensor** from the Rotorflight Configurator (Motors tab) and clicking
Save, the option reverts to _disabled_ after the Save & Reboot cycle. All other settings
persist correctly.

### 1.1 What the "RPM sensor" switch actually is

Checking the Rotorflight Configurator source
(`rotorflight-configurator/src/tabs/motors/RPM.svelte`):

```js
<Field id="rpm-sensor" label="motorsRPMSensor">
  <Switch
    id="rpm-sensor"
    bind:checked={FC.FEATURE_CONFIG.features.FREQ_SENSOR}
  />
</Field>
```

The switch is bound to the **`FREQ_SENSOR` feature bit (BIT 28)** — _not_ to
`useDshotTelemetry` (that is the separate "DShot bidir" switch shown in the same
section when the ESC protocol is DShot).

Feature map (`src/main/config/feature.h`):

| Feature               | Bit |
| --------------------- | --- |
| `FEATURE_FREQ_SENSOR` | 28  |
| `FEATURE_DYN_NOTCH`   | 29  |
| `FEATURE_RPM_FILTER`  | 30  |

### 1.2 Why it always reverted

`src/main/config/config.c` runs `validateAndFixConfig()` **at every boot and at every
EEPROM write**. Inside it:

```c
#ifndef USE_FREQ_SENSOR
    featureDisableImmediate(FEATURE_FREQ_SENSOR);
#endif
```

`featureDisableImmediate()` clears the feature from _both_ the stored configuration
and the runtime mask. Since `USE_FREQ_SENSOR` was never defined for this target
(it was only defined for `STM32_UNIFIED` targets), the sequence was:

```
Enable toggle → MSP_SET_FEATURE_CONFIG → bit 28 stored
       ↓
Save → MSP_EEPROM_WRITE → writeEEPROM → validateAndFixConfig()
       ↓
featureDisableImmediate(FEATURE_FREQ_SENSOR)  ← bit silently stripped
       ↓
Reboot → feature gone → toggle reads "disabled"
```

**Important:** this was a _target-configuration gap_, not the config-streamer /
flash-write corruption bug fixed earlier (`config_streamer.c` fast page-engine
rewrite). The evidence was that every _other_ setting persisted fine across
Save & Reboot — the EEPROM write path itself was healthy.

---

## 2. The Solution

Enable the freq sensor for the CH32H417 target and make the driver compile on the
CH32H41x (WCH RISC-V) SPL, which names and sizes several timer registers differently
from STM32.

### 2.1 Enable the feature — `src/main/target/CH32H417/target.h`

```c
// External magnetic/optical RPM sensor (timer input capture).
// The configurator Motors tab "RPM sensor" switch maps to the FREQ_SENSOR
// feature (bit 28). Without USE_FREQ_SENSOR, validateAndFixConfig() calls
// featureDisableImmediate(FEATURE_FREQ_SENSOR) at every boot and every
// EEPROM write, so the toggle always reverts to disabled after Save & Reboot.
#define USE_FREQ_SENSOR
```

With this defined, `featureDisableImmediate(FEATURE_FREQ_SENSOR)` no longer runs,
the bit survives boot **and** save, and `init.c` calls `freqInit()` at startup.

### 2.2 Port the driver — `src/main/drivers/freq.c`

`freq.c` was written for STM32 (StdPeriph / HAL). Three incompatibilities with the
CH32H41x SPL had to be fixed:

**(a) Missing `FREQ_PRESCALER_MAX`** — the auto-adapter limits were only defined for
STM32 families. Without a CH32H4 branch the file would not compile:

```c
#if defined(STM32F411xE)
#define FREQ_PRESCALER_MAX    0x0080
#elif defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
#define FREQ_PRESCALER_MAX    0x0100
#elif defined(STM32H7)
#define FREQ_PRESCALER_MAX    0x0200
#elif defined(CH32H4)                      // added
#define FREQ_PRESCALER_MAX    0x0200       // added
#endif
```

**(b) Timer register names** — the CH32H41x SPL calls the same registers differently:

| Purpose                   | STM32 SPL                 | CH32H41x SPL                                      |
| ------------------------- | ------------------------- | ------------------------------------------------- |
| Auto-reload register      | `tim->ARR` (32-bit)       | `tim->ATRLR` (16-bit; `ATRLR_32` only on TIM9–12) |
| Event-generation register | `tim->EGR` / `TIM_EGR_UG` | `tim->SWEVGR` / `TIM_UG`                          |

```c
static void freqSetBaseClock(freqInputPort_t *input, uint32_t prescaler)
{
    TIM_TypeDef *tim = input->timerHardware->tim;

    input->prescaler = prescaler;

#ifdef CH32H4
    // CH32H41x SPL: ATRLR is 16-bit (32-bit counters only on TIM9-12),
    // and the event generation register is named SWEVGR with bit TIM_UG
    tim->PSC = prescaler - 1;
    tim->ATRLR = 0xffff;
    tim->SWEVGR = TIM_UG;
#else
    tim->PSC = prescaler - 1;
    tim->ARR = 0xffffffff;
    tim->EGR = TIM_EGR_UG;
#endif
}
```

These constants were verified against `lib/main/CH32H41x/Peripheral/inc/ch32h417.h`
and `ch32h417_tim.h` (`TIM_UG`, `TIM_ICInit`, `TIM_ICPolarity_Rising/Falling`,
`TIM_ICSelection_DirectTI`, `TIM_ICPSC_DIV1` all exist in the WCH SPL).

**(c) 32-bit timer detection** — on STM32, TIM2/TIM5 are 32-bit counters, and the
driver takes a 32-bit capture path for them. On CH32H41x, **TIM2/TIM5 are 16-bit**
(32-bit accessors `CNT_32`/`ATRLR_32` exist only on TIM9–12, which here are output
timers: TIM11 = LED strip, TIM12 = main motor). Taking the 32-bit path would
silently truncate the period:

```c
input->timerHardware = timer;
input->enabled = true;
#ifdef CH32H4
    // CH32H41x: TIM2/TIM5 counters are 16-bit. The 32-bit accessors
    // (CNT_32/ATRLR_32) exist only on TIM9-12, which are not suitable
    // for a freq input here (TIM11/TIM12 are motor/LED outputs).
    input->timer32 = false;
#else
    input->timer32 = (timer->tim == TIM2 || timer->tim == TIM5);
#endif
```

---

## 3. How the Root Cause Was Found (method)

1. **Located the actual MSP target of the switch.** Searched the Rotorflight
   Configurator repo rather than guessing — the Motors tab "RPM sensor" maps to
   `FEATURE_CONFIG.features.FREQ_SENSOR` (feature bit 28), which immediately
   ruled out the `useDshotTelemetry` / DShot-bidir path as the culprit.
2. **Traced where the feature bit could be cleared.** Grepped firmware for
   `FEATURE_FREQ_SENSOR` and found the unconditional
   `featureDisableImmediate()` in `config.c` guarded by `#ifndef USE_FREQ_SENSOR`.
3. **Checked whether the target compiled the feature.** `USE_FREQ_SENSOR` was
   only `#define`d in `target/STM32_UNIFIED/target.h` — absent for CH32H417.
   This exactly matches the "reverts after save _and_ reboot" symptom, because
   the validation runs both at boot and during EEPROM write.
4. **Ruled out config-storage corruption** (the earlier `config_streamer.c`
   flash bug) — other settings persisted, so the EEPROM write path was healthy.
5. **Assessed portability of the driver.** Verified the CH32 port already builds
   the generic timer core (`drivers/timer.c`, `drivers/timer_common.c`,
   `drivers/timer_ch32h41x.c` with `timerClock()`) and that `timerChConfigIC()`,
   `timerChConfigCallbacks()`, `timerChCCR()` exist — input-capture support was
   already proven on this port by `serial_escserial.c`. Only the register-name
   and prescaler-limit differences needed patching.
6. **Validated with the WCH toolchain** (`riscv-wch-elf-gcc -fsyntax-only` with the
   target's define set), then a full `make TARGET=CH32H417` — clean build.

---

## 4. What Works Now vs. What Is Still Missing

### Works now (after reflashing this build)

- The **RPM sensor toggle persists** across Save & Reboot.
- **RPM from bidirectional DShot telemetry** works unchanged: select a DShot ESC
  protocol in the Motors tab and enable the "DShot bidir" switch (same section).
  `rpmSourceInit()` (`src/main/flight/motors.c`) picks the source automatically
  (priority: freq sensor → dshot telemetry → ESC sensor), feeding both the
  motors tab RPM display and the **RPM filter**.
- Note: `config.c` silently clears `useDshotTelemetry` if the motor protocol is
  not DShot — the RPM sensor and DShot bidir both require a DShot protocol.

### Still missing for a _magnetic_ RPM sensor

The freq sensor measures pulses on a **timer input-capture channel**. It discovers
its pin from timer entries flagged `TIM_USE_FREQ`:

```c
// src/main/pg/freq.c
freqConfig->ioTag[index] = timerioTagGetByUsage(TIM_USE_FREQ, index);
```

`target/CH32H417/target.c` currently defines 6 timer channels, none of them
`TIM_USE_FREQ`:

```c
DEF_TIM(TIM8,  CH1, PE3, TIM_USE_SERVO, 0, 0),  // S1
DEF_TIM(TIM8,  CH2, PE4, TIM_USE_SERVO, 0, 1),  // S2
DEF_TIM(TIM8,  CH3, PE5, TIM_USE_SERVO, 0, 2),  // S3
DEF_TIM(TIM12, CH4, PE6, TIM_USE_MOTOR, 0, 6),  // Main motor (ESC)
DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MOTOR, 0, 4), // Tail motor (PWM/DShot)
DEF_TIM(TIM11, CH1, PD3, TIM_USE_LED, 0, 5),    // LED strip (WS2812)
```

So until a pad is assigned, the feature stays configured but the sensor reads 0
RPM (`isFreqSensorPortInitialized()` is false). To activate it:

1. Pick a routed, unused pad on the board and add, e.g.:
   `DEF_TIM(TIMx, CHy, PIN, TIM_USE_FREQ, 0, 0),` to `timerHardware[]`
   and bump `USABLE_TIMER_CHANNEL_COUNT` in `target.h`.
2. Wire the magnetic/optical RPM sensor output to that pad (open collector /
   pulse output works with `freq_input_pull = ON`).
3. Rebuild + flash — the ioTag is picked up automatically from the timer table
   (`freq_input_edge` defaults to falling edge).

> TODO: ask which board pad is routed for the RPM sensor input, then add the
> `DEF_TIM(..., TIM_USE_FREQ, ...)` entry.

---

## 5. Files Changed

| File                                | Change                                                                                                                                                      |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `src/main/target/CH32H417/target.h` | Added `#define USE_FREQ_SENSOR` with explanation comment                                                                                                    |
| `src/main/drivers/freq.c`           | Added `CH32H4` branch for `FREQ_PRESCALER_MAX`; `freqSetBaseClock()` uses `ATRLR`/`SWEVGR`/`TIM_UG` on CH32; forced 16-bit mode (`timer32 = false`) on CH32 |

## 6. Verification

- `riscv-wch-elf-gcc -fsyntax-only` on `src/main/drivers/freq.c` with
  `-DUSE_FREQ_SENSOR -DCH32H415 -DCH32H41x -DCore_V5F -DRISC_V -DUSE_CHBSP_DRIVER
-DCH32H417` — exit 0, no errors.
- Full firmware build: `make TARGET=CH32H417` — success.
