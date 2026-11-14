/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Stubs for what io/ledstrip.c reads at run time (flight state, battery,
 * RSSI, GPS) and the WS2811 driver it drives. The configuration tests
 * don't render the strip, so these are all no-ops.
 */

#pragma once

extern "C" {
#include "drivers/io_types.h"
#include "drivers/light_ws2811strip.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "io/beeper.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "scheduler/scheduler.h"
#include "sensors/battery.h"

uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t stateFlags;
float rcCommand[MAX_SUPPORTED_RC_CHANNEL_COUNT];
gpsSolutionData_t gpsSol;

timeMs_t millis(void) { return 0; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool failsafeIsActive(void) { return false; }
bool isArmingDisabled(void) { return false; }
bool isBeeperOn(void) { return false; }
bool sensors(uint32_t) { return false; }
float getThrottle(void) { return 0; }
uint8_t getBatteryChargeLevel(void) { return 0; }
batteryState_e getBatteryState(void) { return BATTERY_OK; }
uint8_t getRssiPercent(void) { return 0; }
bool rxIsReceivingSignal(void) { return false; }
void schedulerIgnoreTaskExecTime(void) {}

void getLedHsv(uint16_t, hsvColor_t *) {}
void setLedHsv(uint16_t, const hsvColor_t *) {}
void setLedsWithInvertedLedFormat(uint32_t) {}
void setStripColor(const hsvColor_t *) {}
void setUsedLedCount(unsigned) {}
bool isWS2811LedStripReady(void) { return false; }
void ws2811LedStripEnable(void) {}
void ws2811LedStripInit(ioTag_t) {}
void ws2811UpdateStrip(ledStripFormatRGB_e, uint8_t) {}
}
