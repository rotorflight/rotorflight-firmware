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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "setpoint.h"


#define SP_SMOOTHING_FILTER_MIN_HZ             1
#define SP_SMOOTHING_FILTER_MAX_HZ          1000

#define SP_MAX_UP_CUTOFF                   20.0f
#define SP_MAX_DN_CUTOFF                    0.5f

typedef struct
{
    float deflection[4];
    float setpoint[4];
    float limited[4];
    float maximum[4];

    float accelLimit[4];
    float ringLimit;

    float movementThreshold[4];

    float maxGainUp;
    float maxGainDown;

    filter_t filter[4];

    float smoothingFactor;

    uint16_t responseCutoff[4];
    uint16_t activeCutoff[4];

} setpointData_t;

static FAST_DATA_ZERO_INIT setpointData_t sp;


float getSetpoint(int axis)
{
    return sp.setpoint[axis];
}

float getDeflection(int axis)
{
    return sp.deflection[axis];
}


static float setpointAutoSmoothingCutoff(float frameTimeUs)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        cutoff = sp.smoothingFactor / frameTimeUs;
    }

    return constrainf(cutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
}

void setpointUpdateTiming(float frameTimeUs)
{
    float maxCutoff = setpointAutoSmoothingCutoff(frameTimeUs);

    for (int i = 0; i < 4; i++) {
        float cutoff = MIN(sp.responseCutoff[i], maxCutoff);
        if (sp.activeCutoff[i] != cutoff) {
            filterUpdate(&sp.filter[i], cutoff, pidGetPidFrequency());
            sp.activeCutoff[i] = cutoff;
            DEBUG_AXIS(SETPOINT, i, 6, cutoff);
        }
    }

    DEBUG(SETPOINT, 7, frameTimeUs);
}

INIT_CODE void setpointInitProfile(void)
{
    sp.ringLimit = 1.0f / (1.4142135623f - currentControlRateProfile->cyclic_ring * 0.004142135623f);

    for (int i = 0; i < 4; i++) {
        sp.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
        sp.responseCutoff[i] = constrain(
            2500 / (10 * currentControlRateProfile->response_time[i] + 1),
            SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
    }
}

INIT_CODE void setpointInit(void)
{
    sp.smoothingFactor = 15e6f / (10 + rcControlsConfig()->rc_smoothness);

    sp.maxGainUp = pt1FilterGain(SP_MAX_UP_CUTOFF, pidGetPidFrequency());
    sp.maxGainDown = pt1FilterGain(SP_MAX_DN_CUTOFF, pidGetPidFrequency());

    setpointInitProfile();

    for (int i = 0; i < 4; i++) {
        sp.movementThreshold[i] = sq(rcControlsConfig()->rc_threshold[i] / 1000.0f);
        sp.activeCutoff[i] = sp.responseCutoff[i];
        lowpassFilterInit(&sp.filter[i], LPF_PT3, sp.activeCutoff[i], pidGetPidFrequency(), 0);
    }
}

void setpointUpdate(void)
{
    for (int axis = 0; axis < 4; axis++) {
        float deflection, delta;

        deflection = sp.deflection[axis] = getRcDeflection(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, deflection * 1000);

        delta = sq(deflection)- sp.maximum[axis];
        sp.maximum[axis] += delta * ((delta > 0) ? sp.maxGainUp : sp.maxGainDown);

        DEBUG_AXIS(SETPOINT, axis, 5, sp.maximum[axis]);
    }

    DEBUG(AIRBORNE, 0, sqrtf(sp.maximum[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, sqrtf(sp.maximum[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, sqrtf(sp.maximum[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, sqrtf(sp.maximum[FD_COLL]) * 1000);
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, isSpooledUp());
    DEBUG(AIRBORNE, 6, isHandsOn());
    DEBUG(AIRBORNE, 7, isAirborne());

    const float R = sp.deflection[FD_ROLL]  * sp.ringLimit;
    const float P = sp.deflection[FD_PITCH] * sp.ringLimit;
    const float C = sqrtf(sq(R) + sq(P));

    if (C > 1.0f) {
        sp.deflection[FD_ROLL]  /= C;
        sp.deflection[FD_PITCH] /= C;
    }

    DEBUG_AXIS(SETPOINT, FD_ROLL, 1, sp.deflection[FD_ROLL] * 1000);
    DEBUG_AXIS(SETPOINT, FD_PITCH, 1, sp.deflection[FD_PITCH] * 1000);

    for (int axis = 0; axis < 4; axis++) {
        float SP = sp.deflection[axis];

        // rcCommand[YAW] CW direction is positive, while gyro[YAW] is negative
        if (axis == FD_YAW)
            SP = -SP;

        SP = sp.limited[axis] = slewLimit(sp.limited[axis], SP, sp.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 2, SP * 1000);

        SP = sp.deflection[axis] = filterApply(&sp.filter[axis], SP);
        DEBUG_AXIS(SETPOINT, axis, 3, SP * 1000);

        SP = sp.setpoint[axis] = applyRatesCurve(axis, SP);
        DEBUG_AXIS(SETPOINT, axis, 4, SP);
    }
}

bool isHandsOn(void)
{
    return (
        sp.maximum[FD_ROLL] > sp.movementThreshold[FD_ROLL] ||
        sp.maximum[FD_PITCH] > sp.movementThreshold[FD_PITCH] ||
        sp.maximum[FD_YAW] > sp.movementThreshold[FD_YAW] ||
        sp.maximum[FD_COLL] > sp.movementThreshold[FD_COLL]
    );
}

bool isAirborne(void)
{
    return (
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            isHandsOn() ||
            //getAltitude() > 2.0f ||
            getCosTiltAngle() < 0.9f ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}
