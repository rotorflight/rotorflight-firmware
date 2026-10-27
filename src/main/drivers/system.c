/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/persistent.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "flight/servos.h"
#include "flight/motors.h"

#include "system.h"

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
// See "RM CoreSight Architecture Specification"
// B2.3.10  "LSR and LAR, Software Lock Status Register and Software Lock Access Register"
// "E1.2.11  LAR, Lock Access Register"

#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55

#endif

// cycles per microsecond
static uint32_t usTicks = 0;
#if defined(CH32H4)
// micros() on CH32H4 is a monotonic microsecond accumulator driven by mcycle deltas.
// It does not depend on SysTick (which cannot be correlated with mcycle without a race),
// it is ISR-safe (global interrupts are disabled for the few cycles of the snapshot) and
// it wraps every ~70 minutes, just like the DWT-CYCCNT based micros() on STM32.
static uint32_t mcycleLast = 0;  // last mcycle snapshot (cycles)
static uint32_t microsAccum = 0; // accumulated microseconds (rollover in 70 minutes)
#endif
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;
static uint32_t cpuClockFrequency = 0;

void cycleCounterInit(void)
{
#if defined(CH32H4)
    // RISC-V: use mcycle CSR for cycle counting
    // SystemAndCoreClockUpdate reads the RCC registers
    SystemAndCoreClockUpdate();
    cpuClockFrequency = SystemCoreClock;
    usTicks = cpuClockFrequency / 1000000;

    // Reset mcycle before use to avoid unpredictable wrap-around
    __set_MCOUNT_INHIBIT(0x5); // disable mcycle
    __set_MCYCLE(0);           // clear mcycle
    __set_MCOUNT_INHIBIT(0x0); // enable mcycle

    // Initialize the micros() accumulator state (mcycle now starts at 0)
    mcycleLast = 0;
    microsAccum = 0;
#elif defined(USE_HAL_DRIVER)
    cpuClockFrequency = HAL_RCC_GetSysClockFreq();
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    cpuClockFrequency = clocks.SYSCLK_Frequency;
#endif

#if !defined(CH32H4)
    usTicks = cpuClockFrequency / 1000000;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(DWT_LAR_UNLOCK_VALUE)
#if defined(STM32H7)
    ITM->LAR = DWT_LAR_UNLOCK_VALUE;
#elif defined(STM32F7)
    DWT->LAR = DWT_LAR_UNLOCK_VALUE;
#elif defined(STM32F4)
    // Note: DWT_Type does not contain LAR member.
#define DWT_LAR
    __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
    *(DWTLAR) = DWT_LAR_UNLOCK_VALUE;
#endif
#endif

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif // !CH32H4
}

// SysTick

static volatile int sysTickPending = 0;
void SysTick_Handler(void)
{
#if defined(CH32H4) || defined(CH32H41x)
    sysTickUptime++;
    // SysTick1 status is reported in SysTick0->ISR bit 1 on CH32H41x.
    SysTick0->ISR &= ~(1 << 1);
#else
    ATOMIC_BLOCK(NVIC_PRIO_MAX)
    {
        sysTickUptime++;
        sysTickValStamp = SysTick->VAL;
        sysTickPending = 0;
        (void)(SysTick->CTRL);
    }
#endif
#ifdef USE_HAL_DRIVER
    // used by the HAL for some timekeeping and timeouts, should always be 1ms
    HAL_IncTick();
#endif
}

#if defined(CH32H4) || defined(CH32H41x)
/* CH32H417 V5F uses SysTick1 configured via SysTick_Config() with SysTick1_IRQn.
 * SysTick1_Handler must be a strong definition to override the weak default in the startup file.
 * Without this, the IRQ hits the default infinite-loop handler and the firmware hangs. */
void __attribute__((interrupt("WCH-Interrupt-fast"))) SysTick1_Handler(void)
{
    SysTick_Handler();
}

/* SysTick0 handler kept for completeness in case SysTick0 is used elsewhere */
void __attribute__((interrupt("WCH-Interrupt-fast"))) SysTick0_Handler(void)
{
    SysTick_Handler();
}
#endif

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t microsISR(void)
{
#if defined(CH32H4)
    // micros() is ISR-safe on CH32H4 (it disables global interrupts only for a few
    // cycles and restores them conditionally), so it is safe to call from ISRs too.
    return micros();
#else
    register uint32_t ms, pending, cycle_cnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX)
    {
        cycle_cnt = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            sysTickPending = 1;
            cycle_cnt = SysTick->VAL;
        }

        ms = sysTickUptime;
        pending = sysTickPending;
    }

    return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
#endif
}

uint32_t micros(void)
{
#if defined(CH32H4)
    uint32_t mstatus;
    uint32_t now, delta, dUs;

    // Take an atomic snapshot: disable global interrupts for the few cycles of the
    // read+update so a nested ISR cannot re-enter and corrupt the accumulator state.
    // MIE (mstatus bit 3) is cleared, then restored only if it was set before - this
    // keeps the function safe when called from within an ISR (MIE already cleared).
    asm volatile("csrr %0, mstatus" : "=r"(mstatus));
    asm volatile("csrci mstatus, 8" ::: "memory");

    asm volatile("csrr %0, mcycle" : "=r"(now));

    // Wrap-safe delta (mcycle is 32-bit here; correct as long as micros() is called
    // at least once per mcycle wrap period (~15s at 288MHz), which the scheduler
    // always guarantees)
    delta = now - mcycleLast;
    if (usTicks)
    {
        dUs = delta / usTicks;
        // carry the sub-microsecond remainder so no drift accumulates
        mcycleLast += dUs * usTicks;
        microsAccum += dUs;
    }

    if (mstatus & 8)
    {
        asm volatile("csrs mstatus, 8" ::: "memory");
    }

    return microsAccum;
#else
    register uint32_t ms, cycle_cnt;

    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI()))
    {
        return microsISR();
    }

    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
#endif
}
uint32_t getCycleCounter(void)
{
#if defined(CH32H4)
    uint32_t cnt;
    asm volatile("csrr %0, mcycle" : "=r"(cnt));
    return cnt;
#else
    return DWT->CYCCNT;
#endif
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / usTicks;
}

// Note that this conversion is signed as this is used for periods rather than absolute timestamps
int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)usTicks;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us)
        ;
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;)
    {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

static void indicate(uint8_t count, uint16_t duration)
{
    if (count)
    {
        LED1_ON;
        LED0_OFF;

        while (count--)
        {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);

            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(duration);
        }
    }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--)
    {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        delay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        delay(1000);
    }
}

void failureMode(failureMode_e mode)
{
    indicateFailure(mode, 10);

#ifdef DEBUG_BUILD
    systemResetHard();
#else
    systemReset(RESET_BOOTLOADER_REQUEST_ROM);
#endif
}

void initialiseMemorySections(void)
{
#ifdef USE_ITCM_RAM
    /* Load functions into ITCM RAM */
    extern uint8_t tcm_code_start;
    extern uint8_t tcm_code_end;
    extern uint8_t tcm_code;
    memcpy(&tcm_code_start, &tcm_code, (size_t)(&tcm_code_end - &tcm_code_start));
#endif

#ifdef USE_CCM_CODE
    /* Load functions into RAM */
    extern uint8_t ccm_code_start;
    extern uint8_t ccm_code_end;
    extern uint8_t ccm_code;
    memcpy(&ccm_code_start, &ccm_code, (size_t)(&ccm_code_end - &ccm_code_start));
#endif

#ifdef USE_FAST_DATA
    /* Load FAST_DATA variable initializers into DTCM RAM */
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, (size_t)(&_efastram_data - &_sfastram_data));
#endif
}

#ifdef STM32H7
void initialiseD2MemorySections(void)
{
    /* Load DMA_DATA variable intializers into D2 RAM */
    extern uint8_t _sdmaram_bss;
    extern uint8_t _edmaram_bss;
    extern uint8_t _sdmaram_data;
    extern uint8_t _edmaram_data;
    extern uint8_t _sdmaram_idata;
    bzero(&_sdmaram_bss, (size_t)(&_edmaram_bss - &_sdmaram_bss));
    memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t)(&_edmaram_data - &_sdmaram_data));
}
#endif

static void unusedPinInit(IO_t io)
{
    if (IOGetOwner(io) == OWNER_FREE)
    {
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

void unusedPinsInit(void)
{
    IOTraversePins(unusedPinInit);
}

void systemReset(int reason)
{
#ifdef USE_PERSISTENT_OBJECTS
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, reason);
#else
    UNUSED(reason);
#endif

    motorStop();
    motorShutdown();
    servoShutdown();

    systemResetHard();
}

#if defined(CH32H4)
void systemResetHard(void)
{
    __disable_irq();
    NVIC_SystemReset();
}
#endif
