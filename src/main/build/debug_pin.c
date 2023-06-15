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

#include "platform.h"

#ifdef USE_DEBUG_PIN

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "debug_pin.h"

typedef struct dbgPinState_s {
    GPIO_TypeDef *gpio;
    uint32_t setBSRR;
    uint32_t resetBSRR;
} dbgPinState_t;

dbgPinState_t dbgPinStates[DEBUG_PIN_COUNT] = { 0 };

extern dbgPin_t dbgPins[DEBUG_PIN_COUNT];

#if 0

// Add something like this to target.c
static dbgPin_t dbgPins[DEBUG_PIN_COUNT] = {
    { .tag = IO_TAG(PA2) },
};

// Add something like this to target.h
#define USE_DEBUG_PIN
#define DEBUG_PIN_COUNT 1

#endif

#endif /* USE_DEBUG_PIN */

void dbgPinInit(void)
{
#ifdef USE_DEBUG_PIN
    for (unsigned i = 0; i < ARRAYLEN(dbgPins); i++) {
        dbgPin_t *dbgPin = &dbgPins[i];
        dbgPinState_t *dbgPinState = &dbgPinStates[i];
        IO_t io = IOGetByTag(dbgPin->tag);
        if (io) {
            IOInit(io, OWNER_SYSTEM, 0);
            IOConfigGPIO(io, IOCFG_OUT_PP);
            dbgPinState->gpio = IO_GPIO(io);

            int pinSrc = IO_GPIO_PinSource(io);
            dbgPinState->setBSRR = (1 << pinSrc);
            dbgPinState->resetBSRR = (1 << (pinSrc + 16));
        }
    }
#endif
}

void dbgPinHi(int index)
{
#ifdef USE_DEBUG_PIN
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPinState_t *dbgPinState = &dbgPinStates[index];
    if (dbgPinState->gpio) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        dbgPinState->gpio->BSRR
#else
        dbgPinState->gpio->BSRRL
#endif
             = dbgPinState->setBSRR;
    }
#else
    UNUSED(index);
#endif
}

void dbgPinLo(int index)
{
#ifdef USE_DEBUG_PIN
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPinState_t *dbgPinState = &dbgPinStates[index];

    if (dbgPinState->gpio) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        dbgPinState->gpio->BSRR
#else
        dbgPinState->gpio->BSRRL
#endif
            = dbgPinState->resetBSRR;
    }
#else
    UNUSED(index);
#endif
}

void dbgPinSet(int index, int value)
{
#ifdef USE_DEBUG_PIN
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPinState_t *dbgPinState = &dbgPinStates[index];

    if (dbgPinState->gpio) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        dbgPinState->gpio->BSRR
#else
        dbgPinState->gpio->BSRRL
#endif
           = (value) ?
                dbgPinState->setBSRR:
                dbgPinState->resetBSRR;
    }
#else
    UNUSED(index);
    UNUSED(value);
#endif
}

