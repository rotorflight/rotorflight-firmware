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

#include "pg/rx.h"

#include "rx/rx.h"

#include "rc.h"


FAST_DATA_ZERO_INIT float rcCommand[5];                  // -500..+500 for RPYC and 0..1000 for THROTTLE
FAST_DATA_ZERO_INIT float rcDeflection[5];               // -1..1 for RPYC, 0..1 for THROTTLE

static FAST_DATA_ZERO_INIT float rawSetpoint[4];
static FAST_DATA_ZERO_INIT float smoothSetpoint[4];

static FAST_DATA_ZERO_INIT float rcDeadband[4];

static FAST_DATA_ZERO_INIT timeUs_t lastRxTimeUs;
static FAST_DATA_ZERO_INIT uint16_t currentRxRefreshRate;


void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    rcDeflection[YAW] = 0;
    rawSetpoint[YAW] =  0;
    smoothSetpoint[YAW] = 0;
}

float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getRcSetpoint(int axis)
{
    return smoothSetpoint[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}


uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);
    timeDelta_t localDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);

    if (frameDeltaUs == 0 || localDeltaUs <= frameAgeUs) {
        frameDeltaUs = localDeltaUs;
    }

    currentRxRefreshRate = frameDeltaUs;
    lastRxTimeUs = currentTimeUs;

    DEBUG(RX_TIMING, 0, frameDeltaUs);
    DEBUG(RX_TIMING, 1, localDeltaUs);
    DEBUG(RX_TIMING, 2, frameAgeUs);
}


static inline float deadband(float x, float deadband)
{
    if (x > deadband)
        return x - deadband;
    else if (x < -deadband)
        return x + deadband;
    else
        return 0;
}

void updateRcCommands(void)
{
    float data;

    // rcData => rcCommand => rcDeflection => rawSetpoint
    for (int axis = 0; axis < 4; axis++) {
        data = rcData[axis] - rxConfig()->midrc;
        data = constrainf(data, -500, 500);
        data = deadband(data, rcDeadband[axis]);

        // RC yaw rate and gyro yaw rate have opposite sign
        if (axis == FD_YAW)
            data = -data;

        rcCommand[axis] = data;
        rcDeflection[axis] = data / (500 - rcDeadband[axis]);

        data = applyRatesCurve(axis, rcDeflection[axis]);
        rawSetpoint[axis] = data;

        DEBUG(RC_COMMAND, axis, rcCommand[axis]);
        DEBUG(RC_COMMAND, axis+4, rcDeflection[axis] * 1000);

        DEBUG(RC_SETPOINT, axis, data);
    }

    // RF TODO FIXME
    data = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;
    rcCommand[THROTTLE] = data;
    rcDeflection[THROTTLE] = data / PWM_RANGE;
}

void processRcCommand(void)
{
    // rawSetpoint => smoothSetpoint
    for (int axis = 0; axis < 4; axis++) {
        smoothSetpoint[axis] = rawSetpoint[axis];
    }
}

INIT_CODE void initRcProcessing(void)
{
    rcDeadband[0] = rcControlsConfig()->deadband;
    rcDeadband[1] = rcControlsConfig()->deadband;
    rcDeadband[2] = rcControlsConfig()->yaw_deadband;
    rcDeadband[3] = 0;
}
