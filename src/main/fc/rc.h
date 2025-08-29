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

#pragma once

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/rc_modes.h"

#include "flight/setpoint.h"

#define PRIMARY_CHANNEL_COUNT 5

enum {
    ROLL = 0,
    PITCH,
    YAW,
    COLLECTIVE,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
};

// Legacy RC command -500..500
extern float rcCommand[];

void initRcProcessing(void);
void updateRcCommands(void);

float getRcDeflection(int axis);

float getThrottle(void);

static inline float getThrottleCommand(void) { return getThrottle() * 1000; }
static inline uint8_t getThrottlePercent(void) { return lrintf(getThrottle() * 100); }

bool isThrottleOff(void);

uint getCurrentRxRefreshRate(void);
float getAverageRxRefreshRate(void);

void updateRcRefreshRate(timeUs_t currentTimeUs);
