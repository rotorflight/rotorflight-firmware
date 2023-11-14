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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"
#include "fc/core.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc_rates.h"

#include "flight/pid.h"
#include "flight/setpoint.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "rc.h"


#define RX_REFRESH_RATE_MIN_US          950
#define RX_REFRESH_RATE_MAX_US        65000

#define RX_REFRESH_RATE_AVERAGING       100

#define RX_RANGE_COUNT                    4


FAST_DATA_ZERO_INIT float rcCommand[MAX_SUPPORTED_RC_CHANNEL_COUNT];           // -500..+500 for RPYCT, more for AUX

static FAST_DATA_ZERO_INIT float rcDeflection[5];                              // -1..1 for RPYC, 0..1 for THROTTLE
static FAST_DATA_ZERO_INIT float rcDeadband[4];

static FAST_DATA_ZERO_INIT uint16_t currentRxRefreshRate;
static FAST_DATA_ZERO_INIT float    averageRxRefreshRate;
static FAST_DATA_ZERO_INIT uint16_t averagingLength;
static FAST_DATA_ZERO_INIT timeUs_t lastRxTimeUs;

static FAST_DATA_ZERO_INIT uint32_t changeCount;
static FAST_DATA_ZERO_INIT uint16_t repeatCount;
static FAST_DATA_ZERO_INIT uint32_t repeatRange[RX_RANGE_COUNT];
static FAST_DATA_ZERO_INIT uint16_t lastRcValue;
static FAST_DATA_ZERO_INIT uint8_t  currentMult;


void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    rcDeflection[YAW] = 0;
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

float getThrottle(void)
{
    return rcDeflection[THROTTLE];
}

bool isArmingThrottle(void)
{
    // Allow slight deadband
    return (rcInput[THROTTLE] < rcControlsConfig()->rc_arm_throttle + 5);
}

throttleStatus_e getThrottleStatus(void)
{
    return (rcInput[THROTTLE] < rcControlsConfig()->rc_min_throttle) ? THROTTLE_LOW : THROTTLE_HIGH;
}

uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

float getAverageRxRefreshRate(void)
{
    return averageRxRefreshRate;
}

float getAverageRxUpdateRate(void)
{
    return averageRxRefreshRate * currentMult;
}


static void updateRcChange(void)
{
    const uint16_t rcValue = lrintf(rcCommand[FD_ROLL]);

    if (rcValue == lastRcValue) {
        repeatCount++;
    }
    else {
        if (repeatCount < RX_RANGE_COUNT) {
            repeatRange[repeatCount]++;
            changeCount++;
        }
        repeatCount = 0;
        lastRcValue = rcValue;
    }

    uint32_t rangeLimit = constrain(changeCount / RX_RANGE_COUNT, 10, 1000);
    for (int i=0; i<RX_RANGE_COUNT; i++) {
        if (repeatRange[i] > rangeLimit) {
            currentMult = i + 1;
            break;
        }
    }

    DEBUG(RX_TIMING, 7, currentMult);
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs = 0;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);
    timeDelta_t localDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);

    DEBUG(RX_TIMING, 4, frameDeltaUs);
    DEBUG(RX_TIMING, 5, localDeltaUs);
    DEBUG(RX_TIMING, 6, frameAgeUs);

    if (frameDeltaUs == 0 || localDeltaUs <= frameAgeUs) {
        frameDeltaUs = localDeltaUs;
    }

    currentRxRefreshRate = frameDeltaUs;
    lastRxTimeUs = currentTimeUs;

    float currentRateUs = frameDeltaUs;

    if (averagingLength >= RX_REFRESH_RATE_AVERAGING) {
        if (rxIsReceivingSignal() && currentRxRefreshRate > RX_REFRESH_RATE_MIN_US && currentRxRefreshRate < RX_REFRESH_RATE_MAX_US) {
            currentRateUs = constrainf(currentRateUs, 0.75f * averageRxRefreshRate, 1.25f * averageRxRefreshRate);
        } else {
            currentRateUs = averageRxRefreshRate;
        }
    }
    else {
        averagingLength++;
    }

    averageRxRefreshRate += (currentRateUs - averageRxRefreshRate) / averagingLength;

    updateRcChange();

    DEBUG(RX_TIMING, 0, averageRxRefreshRate);
    DEBUG(RX_TIMING, 1, averageRxRefreshRate * currentMult);
    DEBUG(RX_TIMING, 2, currentRxRefreshRate);
}


void updateRcCommands(void)
{
    float data;
    float range;

    setpointUpdateTiming(averageRxRefreshRate * currentMult);

    // rcInput => rcDeflection => rcCommand
    for (int axis = 0; axis < 4; axis++) {
        // Center point
        data = rcInput[axis] - rcControlsConfig()->rc_center;

        // RC yaw rate and gyro yaw rate have opposite signs
        if (axis == FD_YAW)
            data = -data;

        // Apply deadband
        data = fapplyDeadband(data, rcDeadband[axis]);
        range = rcControlsConfig()->rc_deflection - rcDeadband[axis];

        // Deflection range is -1..1
        rcDeflection[axis] = limitf(data / range, 1.0f);

        // rcCommand range is -500..500
        rcCommand[axis] = rcDeflection[axis] * 500;

        DEBUG(RC_COMMAND, axis, rcCommand[axis]);
    }

    // Throttle deflection range is 0..1
    data = scaleRangef(rcInput[THROTTLE], rcControlsConfig()->rc_min_throttle, rcControlsConfig()->rc_max_throttle, 0, 1);
    rcDeflection[THROTTLE] = constrainf(data, 0, 1);

    // Throttle command range is -500..500 for compatibility with other channels
    rcCommand[THROTTLE] = rcDeflection[THROTTLE] * 1000 - 500;

    DEBUG(RC_COMMAND, THROTTLE, lrintf(rcDeflection[THROTTLE] * 1000));

    // AUX channels
    for (int axis = CONTROL_CHANNEL_COUNT; axis < MAX_SUPPORTED_RC_CHANNEL_COUNT; axis++) {
        rcCommand[axis] = rcInput[axis] - rcControlsConfig()->rc_center;
    }
}

INIT_CODE void initRcProcessing(void)
{
    rcDeadband[0] = rcControlsConfig()->rc_deadband;
    rcDeadband[1] = rcControlsConfig()->rc_deadband;
    rcDeadband[2] = rcControlsConfig()->rc_yaw_deadband;
    rcDeadband[3] = 0;

    currentMult = 1;
}

