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

#include "drivers/io.h"
#include "io_impl.h"
#include "drivers/time.h"

#include "light_led.h"

static IO_t leds[STATUS_LED_NUMBER];
static uint8_t ledInversion = 0;

// Debug LED on PC4 (LED0_PIN). Self-contained so it works before ledInit().
static IO_t debugLed = IO_NONE;
static bool debugLedReady = false;

void debugLedInit(void)
{
    if (debugLedReady) {
        return;
    }
    debugLed = IOGetByTag(IO_TAG(PC4));
    IOInit(debugLed, OWNER_LED, RESOURCE_INDEX(0));
    IOConfigGPIO(debugLed, IOCFG_OUT_PP);
    IOWrite(debugLed, true);   // LED0 is active-low on this board -> off
    debugLedReady = true;
}

// Blink `pattern` times quickly (e.g. 3 = three short blinks), then a long pause.
// Call this at a checkpoint; if the code hangs after it, the LED shows the last
// checkpoint reached.
void debugLedBlink(int pattern)
{
    debugLedInit();
    for (int i = 0; i < pattern; i++) {
        IOWrite(debugLed, false);   // on (active low)
        delay(500);
        IOWrite(debugLed, true);    // off
        delay(500);
    }
    delay(1000);                     // long pause between patterns
}

// Same as debugLedBlink but loops forever. Use at the point where you suspect
// the code is stuck; the LED will keep repeating the pattern.
void debugLedBlinkForever(int pattern)
{
    debugLedInit();
    while (true) {
        for (int i = 0; i < pattern; i++) {
            IOWrite(debugLed, false);
            delay(100);
            IOWrite(debugLed, true);
            delay(100);
        }
        delay(500);
    }
}

void ledInit(const statusLedConfig_t *statusLedConfig)
{
    ledInversion = statusLedConfig->inversion;
    for (int i = 0; i < STATUS_LED_NUMBER; i++) {
        if (statusLedConfig->ioTags[i]) {
            leds[i] = IOGetByTag(statusLedConfig->ioTags[i]);
            IOInit(leds[i], OWNER_LED, RESOURCE_INDEX(i));
            IOConfigGPIO(leds[i], IOCFG_OUT_PP);
        } else {
            leds[i] = IO_NONE;
        }
    }

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
}

void ledToggle(int led)
{
    IOToggle(leds[led]);
}

void ledSet(int led, bool on)
{
    const bool inverted = (1 << (led)) & ledInversion;
    IOWrite(leds[led], on ? inverted : !inverted);
}
