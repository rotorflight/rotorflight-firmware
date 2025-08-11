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


#define RX_REFRESH_RATE_MIN_US          490
#define RX_REFRESH_RATE_MAX_US        40000

#define RX_REFRESH_RATE_AVERAGING        64

#define RX_RANGE_COUNT                    4


// Legacy rcCommand for direct acceess
FAST_DATA_ZERO_INIT float rcCommand[MAX_SUPPORTED_RC_CHANNEL_COUNT];           // -500..+500 for RPYCT, more for AUX

// Internal data
typedef struct {

    float       deflection[4];          // RPYC
    float       deadband[4];
    float       center[4];
    float       range[4];

    float       throttle;
    float       offThrottle;
    float       minThrottle;
    float       maxThrottle;

    uint        currentRxRefreshMult;
    uint        currentRxRefreshRate;
    float       averageRxRefreshRate;

    uint        lastRcValue;
    timeUs_t    lastRxTimeUs;

    uint        changeCount;
    uint        repeatCount;
    uint        repeatRange[RX_RANGE_COUNT];


} rcData_t;

static FAST_DATA_ZERO_INIT rcData_t rc;


float getRcDeflection(int axis)
{
    return rc.deflection[axis];
}

float getThrottle(void)
{
    return rc.throttle;
}

bool isThrottleOff(void)
{
    return (rcInput[THROTTLE] < rc.offThrottle);
}

uint getCurrentRxRefreshRate(void)
{
    return rc.currentRxRefreshRate;
}

float getAverageRxRefreshRate(void)
{
    return rc.averageRxRefreshRate;
}


static void updateRcChange(void)
{
    const uint rcValue = lrintf(rcCommand[FD_ROLL]);

    if (rcValue == rc.lastRcValue) {
        rc.repeatCount++;
    }
    else {
        if (rc.repeatCount < RX_RANGE_COUNT) {
            rc.repeatRange[rc.repeatCount]++;
            rc.changeCount++;
        }
        rc.repeatCount = 0;
        rc.lastRcValue = rcValue;
    }

    const uint rangeLimit = constrain(rc.changeCount / RX_RANGE_COUNT, 10, 1000);
    for (int i=0; i<RX_RANGE_COUNT; i++) {
        if (rc.repeatRange[i] > rangeLimit) {
            rc.currentRxRefreshMult = i + 1;
            break;
        }
    }

    DEBUG(RX_TIMING, 7, rc.currentRxRefreshMult);
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs = 0;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);
    timeDelta_t localDeltaUs = cmpTimeUs(currentTimeUs, rc.lastRxTimeUs);

    rc.lastRxTimeUs = currentTimeUs;

    DEBUG(RX_TIMING, 4, frameDeltaUs);
    DEBUG(RX_TIMING, 5, localDeltaUs);
    DEBUG(RX_TIMING, 6, frameAgeUs);

    if (frameDeltaUs == 0 || localDeltaUs <= frameAgeUs) {
        frameDeltaUs = localDeltaUs;
    }

    rc.currentRxRefreshRate = frameDeltaUs;

    if (rxIsReceivingSignal()) {
        if (rc.currentRxRefreshRate > RX_REFRESH_RATE_MIN_US && rc.currentRxRefreshRate < RX_REFRESH_RATE_MAX_US) {
            rc.averageRxRefreshRate += (frameDeltaUs - rc.averageRxRefreshRate) / RX_REFRESH_RATE_AVERAGING;
            updateRcChange();
        }
    }

    DEBUG(RX_TIMING, 0, rc.averageRxRefreshRate);
    DEBUG(RX_TIMING, 1, rc.averageRxRefreshRate * rc.currentRxRefreshMult);
    DEBUG(RX_TIMING, 2, rc.currentRxRefreshRate);
}


void updateRcCommands(void)
{
    setpointUpdateTiming(rc.averageRxRefreshRate * rc.currentRxRefreshMult);

    // rcInput => rc.deflection => rcCommand
    for (int axis = 0; axis < 4; axis++) {
        // Center point
        float data = rcInput[axis] - rc.center[axis];

        // Apply deadband
        data = fapplyDeadband(data, rc.deadband[axis]);

        // Deflection range is -1..1
        rc.deflection[axis] = constrainf(data / rc.range[axis], -1, 1);

        // Legacy rcCommand range is -500..500
        rcCommand[axis] = rc.deflection[axis] * 500;

        DEBUG(RC_COMMAND, axis, rcCommand[axis]);
    }

    // Throttle range is 0..1
    rc.throttle = transition(rcInput[THROTTLE], rc.minThrottle, rc.maxThrottle, 0, 1);

    // Legacy throttle command range is -500..500 for compatibility with other channels
    rcCommand[THROTTLE] = rc.throttle * 1000 - 500;

    DEBUG(RC_COMMAND, THROTTLE, lrintf(rc.throttle * 1000));

    // AUX channels
    for (int axis = CONTROL_CHANNEL_COUNT; axis < MAX_SUPPORTED_RC_CHANNEL_COUNT; axis++) {
        rcCommand[axis] = rcInput[axis] - rcControlsConfig()->rc_center;
    }
}

INIT_CODE void initRcProcessing(void)
{
    rc.deadband[0] = rcControlsConfig()->rc_deadband;
    rc.deadband[1] = rcControlsConfig()->rc_deadband;
    rc.deadband[2] = rcControlsConfig()->rc_yaw_deadband;
    rc.deadband[3] = 0;

    for (int axis = 0; axis < 4; axis++) {
        rc.center[axis] = rcControlsConfig()->rc_center;
        rc.range[axis] = rcControlsConfig()->rc_deflection - rc.deadband[axis];
    }

    if (rcControlsConfig()->rc_min_throttle)
        rc.minThrottle = constrain(rcControlsConfig()->rc_min_throttle, RX_PWM_PULSE_MIN, RX_PWM_PULSE_MAX);
    else
        rc.minThrottle = rcControlsConfig()->rc_center - rcControlsConfig()->rc_deflection * 0.9f;

    if (rcControlsConfig()->rc_max_throttle)
        rc.maxThrottle = constrain(rcControlsConfig()->rc_max_throttle, RX_PWM_PULSE_MIN, RX_PWM_PULSE_MAX);
    else
        rc.maxThrottle = rcControlsConfig()->rc_center + rcControlsConfig()->rc_deflection * 0.9f;

    rc.maxThrottle = fmaxf(rc.maxThrottle, rc.minThrottle + 10);

    rc.offThrottle = rc.minThrottle - 1;

    // Start with 100Hz rate
    rc.averageRxRefreshRate = 10000;
    rc.currentRxRefreshMult = 1;
}

