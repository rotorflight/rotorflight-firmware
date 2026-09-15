# CH32H417 — Broken `micros()` Causing CPU Load Spikes (4290%) and "LOAD" Arming Block

**Date:** 2026-09-10
**Affected file:** `src/main/drivers/system.c`
**Target:** CH32H417 (V5F core, RISC-V, MounRiver riscv-wch-elf-gcc)

---

## Symptoms

- Configurator / MSP reported absurd CPU load values — e.g. **4290%** (and similar multi-thousand-percent spikes).
- The **"LOAD"** arming-disabled flag appeared randomly in the configurator, intermittently preventing the helicopter from arming.
- Occasionally unexplained RX-related arming/glitch behaviour (same root cause).

---

## The Problem

The CH32H4 port of `micros()` / `microsISR()` in `src/main/drivers/system.c` combined the
SysTick millisecond counter with the free-running RISC-V `mcycle` CSR:

```c
uint32_t microsISR(void)
{
#if defined(CH32H4)
    uint32_t ms = sysTickUptime;
    uint32_t cycle_cnt;
    asm volatile("csrr %0, mcycle" : "=r"(cycle_cnt));
    return (ms * 1000) + (cycle_cnt / usTicks) % 1000;   // <-- BROKEN
#else
    ...
#endif
}
```

### Why it is wrong

`mcycle` is a **free-running counter that is never reset** at millisecond (or any other)
boundaries. The expression `(cycle_cnt / usTicks) % 1000` is therefore _not_ the
sub-millisecond fraction of the current millisecond — it is a pseudo-random sawtooth
derived from the **absolute** time since boot.

The result: `micros()` returned values that **jumped forward and backward by up to
±1000 µs**, many times per second. It was non-monotonic — the most damaging possible
property for the firmware's primary time source.

### Why that broke the CPU load and arming

1. **CPU load measurement** — `schedulerExecuteTask()` (in `src/main/scheduler/scheduler.c`)
   measures every task's execution time as:

   ```c
   const timeUs_t before = micros();
   selectedTask->attribute->taskFunc(...);
   const timeUs_t after  = micros();
   taskExecutionTimeUs = after - before;    // negative glitch → wraps to ~4.29e9 µs
   taskTotalExecutionTime += taskExecutionTimeUs;
   ```

   When `micros()` glitches _downward_ between the two reads, `after - before` becomes
   negative, which wraps to approximately $2^{32} \approx 4.29\times10^9$ µs as a
   `timeUs_t` (uint32). That single wrapped sample inflates
   `taskTotalExecutionTime` by ~4290 seconds, and

   ```c
   averageCPULoad = 1000 * taskTotalExecutionTime / deltaTime;   // (per-mille)
   ```

   explodes to thousands of percent. This is exactly the observed "4290%" style value —
   it is the wrapped 32-bit delta, not a real measurement.

2. **"LOAD" flag / arming failure** — `src/main/fc/core.c` blocks arming when the load
   looks impossible:

   ```c
   if (getMaxRealTimeLoad() > 750 || getAverageCPULoad() > 750 || getAverageSystemLoad() > 750) {
       setArmingDisabled(ARMING_DISABLED_LOAD);
   }
   ```

   (`averageCPULoad` is in per-mille, so 750 = 75%.) The glitched load spikes tripped
   this check at random times, so the "LOAD" flag appeared intermittently and the FC
   refused to arm.

3. **Side effects** — `microsISR()` is also used by RX drivers (`sbus.c`, `crsf.c`,
   `fport.c`, `spektrum.c`, …) for frame-gap / failsafe timing. The same ±1000 µs
   glitches corrupted those checks too, contributing to random RX-status and
   arming-related behaviour.

### Why correlating `mcycle` with SysTick can't be made reliable on this chip

Even "fixing" the original formula would not work, because the two counters cannot be
correlated without a race:

- `mcycle` runs at the core clock and is **never reset**; SysTick1 is an auto-reload
  timer whose ISR (`SysTick1_Handler`, priority `0xF0`) updates `sysTickUptime` only
  after interrupt latency.
- `ATOMIC_BLOCK(NVIC_PRIO_MAX)` on CH32H4 lowers the PFIC threshold register
  (`ITHRESDR = 0x10`, see `src/main/build/atomic.h`), which masks interrupts with
  priority ≤ the threshold — but SysTick1 sits at priority `0xF0`, **above** the
  threshold, so `sysTickUptime` can still advance inside an "atomic" section. The
  STM32-style read-retry loop (`ms != sysTickUptime`) therefore cannot detect the
  race condition reliably on this platform.

---

## The Fix

Replace the CH32H4 `micros()` with a **monotonic microsecond accumulator driven by
wrap-safe `mcycle` deltas** — independent of SysTick entirely. This mirrors the
behaviour of the DWT-CYCCNT-based `micros()` on STM32, including the ~70-minute wrap.

### Changes in `src/main/drivers/system.c`

**1. New accumulator state:**

```c
static uint32_t mcycleLast = 0;  // last mcycle snapshot (cycles)
static uint32_t microsAccum = 0; // accumulated microseconds (rollover in 70 minutes)
```

**2. `cycleCounterInit()`** — initialize the accumulator right after `mcycle` is reset:

```c
__set_MCOUNT_INHIBIT(0x5);  // disable mcycle
__set_MCYCLE(0);            // clear mcycle
__set_MCOUNT_INHIBIT(0x0);  // enable mcycle

mcycleLast = 0;
microsAccum = 0;
```

**3. `micros()`** — atomic, ISR-safe, drift-free delta accumulation:

```c
uint32_t micros(void)
{
#if defined(CH32H4)
    uint32_t mstatus;
    uint32_t now, delta, dUs;

    // Atomic snapshot: clear MIE for the few cycles of read+update so a nested ISR
    // cannot re-enter and corrupt the accumulator state. Restored only if it was set,
    // so the function is also safe when called from inside an ISR.
    asm volatile("csrr %0, mstatus" : "=r"(mstatus));
    asm volatile("csrci mstatus, 8" ::: "memory");

    asm volatile("csrr %0, mcycle" : "=r"(now));

    // Wrap-safe delta (32-bit unsigned subtraction handles mcycle rollover)
    delta = now - mcycleLast;
    if (usTicks) {
        dUs = delta / usTicks;
        // Carry the sub-microsecond remainder so no drift accumulates
        mcycleLast += dUs * usTicks;
        microsAccum += dUs;
    }

    if (mstatus & 8) {
        asm volatile("csrs mstatus, 8" ::: "memory");
    }

    return microsAccum;
#else
    ... (STM32 path unchanged)
#endif
}
```

Key properties:

| Property                  | How it is achieved                                                                                                                                                                                |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Monotonic**             | µs only ever accumulate from positive cycle deltas — glitches are impossible by construction.                                                                                                     |
| **Wrap-safe**             | 32-bit unsigned `now - mcycleLast` handles `mcycle` rollover correctly. Requires `micros()` to be called at least once per wrap period (~15 s at 288 MHz) — the scheduler always guarantees this. |
| **ISR-safe**              | Global interrupts (`mstatus.MIE`) disabled for only a few cycles; restored _only if they were set before_, so it is correct when called from within an ISR.                                       |
| **Drift-free**            | The sub-microsecond cycle remainder is carried in `mcycleLast` instead of being discarded by integer division.                                                                                    |
| **No SysTick dependency** | No correlation race with `sysTickUptime`; SysTick IRQ latency is irrelevant.                                                                                                                      |

**4. `microsISR()`** — simply delegates, since `micros()` is now ISR-safe:

```c
uint32_t microsISR(void)
{
#if defined(CH32H4)
    return micros();
#else
    ... (STM32 path unchanged)
#endif
}
```

**5.** Added `#include "drivers/time.h"` (needed because `microsISR()` now calls `micros()`,
which is defined later in the same file).

### What was deliberately NOT changed

- `getCycleCounter()` — used for scheduler cycle math and DShot DMA profiling; raw
  32-bit `mcycle` deltas are already wrap-safe.
- `millis()` / `sysTickUptime` / `SysTick1_Handler` — the 1 ms tick was and is correct.
- All scheduler load formulas in `scheduler.c` — they were correct; only their input
  (`micros()`) was broken.
- The entire STM32 (`#else`) code paths.

---

## Verification

- Syntax check of `system.c` with the MounRiver `riscv-wch-elf-gcc` — clean, no errors/warnings.
- Full build `make TARGET=CH32H417` — succeeds:

  ```
  FLASH1: 492132 B / 848 KB (56.67%)
  SRAM:   246080 B / 359 KB (66.94%)
  Creating HEX ./obj/rotorflight_4.6.0_CH32H417.hex
  ```

---

## Expected behaviour after flashing

- CPU load in the configurator/MSP shows a realistic small percentage (single digits to
  ~30% depending on configuration) instead of thousands of percent.
- The "LOAD" arming-disabled flag stays clear and arming is no longer intermittently blocked
  by load spikes.
- RX timing (frame gaps, failsafe detection) no longer sees phantom glitches.

## Notes / limitations

- `micros()` must be called at least once per `mcycle` wrap period (~15 s at 288 MHz)
  or elapsed time is silently lost. The scheduler loop guarantees this under all
  normal and failsafe operation; it is not a concern in practice.
- On multi-core builds (V3F core), each core would need its own accumulator state;
  the current firmware runs the flight code on the V5F core only.
