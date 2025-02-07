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


#define SP_SMOOTHING_FILTER_MIN_HZ            10
#define SP_SMOOTHING_FILTER_MAX_HZ          1000

#define SP_MAX_UP_CUTOFF                   20.0f
#define SP_MAX_DN_CUTOFF                    0.5f

typedef struct
{
    float setpoint[4];
    float deflection[4];

    float ringLimit;

    float limited[4];
    float responseAccel[4];
    float responseFactor[4];

    float smoothingFactor;
    uint16_t smoothingCutoff;
    filter_t smoothingFilter[4];

    float maximum[4];
    float maxGainUp;
    float maxGainDown;
    float movementThreshold[4];

} setpointData_t;

static FAST_DATA_ZERO_INIT setpointData_t sp;

typedef enum {
    GROUND,
    SPOOLUP,
    AIRBORNE,
} airborneState_e;
airborneState_e airborneState = GROUND;

float getSetpoint(int axis)
{
    return sp.setpoint[axis];
}

float getDeflection(int axis)
{
    return sp.deflection[axis];
}

static float setpointResponseAccel(int axis, float value)
{
    sp.limited[axis] += limitf((value - sp.limited[axis]) * sp.responseFactor[axis], sp.responseAccel[axis]);

    return sp.limited[axis];
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
    const uint16_t cutoff = setpointAutoSmoothingCutoff(frameTimeUs);

    if (sp.smoothingCutoff != cutoff) {
        for (int i = 0; i < 4; i++) {
            filterUpdate(&sp.smoothingFilter[i], cutoff, pidGetPidFrequency());
            DEBUG_AXIS(SETPOINT, i, 6, cutoff);
        }
        sp.smoothingCutoff = cutoff;
    }

    DEBUG(SETPOINT, 7, frameTimeUs);
}

INIT_CODE void setpointInitProfile(void)
{
    sp.ringLimit = 1.0f / (1.4142135623f - currentControlRateProfile->cyclic_ring * 0.004142135623f);

    for (int i = 0; i < 4; i++) {
        if (currentControlRateProfile->response_time[i]) {
            const float cutoff = 500.0f / currentControlRateProfile->response_time[i];
            sp.responseFactor[i] = pt1FilterGain(cutoff, pidGetPidFrequency());
        }
        else {
            sp.responseFactor[i] = 1;
        }

        if (currentControlRateProfile->accel_limit[i]) {
            sp.responseAccel[i] = 1000.0f / currentControlRateProfile->accel_limit[i] * pidGetDT();
        }
        else {
            sp.responseAccel[i] = 1;
        }

        sp.movementThreshold[i] = sq(rcControlsConfig()->rc_threshold[i] / 1000.0f);
    }
}

INIT_CODE void setpointInit(void)
{
    sp.smoothingFactor = 25e6f / constrain(rcControlsConfig()->rc_smoothness, 1, 250);
    sp.smoothingCutoff = SP_SMOOTHING_FILTER_MAX_HZ;

    sp.maxGainUp = pt1FilterGain(SP_MAX_UP_CUTOFF, pidGetPidFrequency());
    sp.maxGainDown = pt1FilterGain(SP_MAX_DN_CUTOFF, pidGetPidFrequency());

    for (int i = 0; i < 4; i++) {
        lowpassFilterInit(&sp.smoothingFilter[i], LPF_PT3, SP_SMOOTHING_FILTER_MAX_HZ, pidGetPidFrequency(), LPF_UPDATE);
    }

    setpointInitProfile();

    airborneState = GROUND;
}

float angleOfVectors(float a[3], float b[3])
{
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    float lenA = sqrtf(sq(a[0]) + sq(a[1]) + sq(a[2]));
    float lenB = sqrtf(sq(b[0]) + sq(b[1]) + sq(b[2]));

    return acosf(dot / (lenA * lenB));
}

void getYawVector(float v[3], quaternion *q)
{
    /*
     * (0, 0, 1) rotated by the quaternion:
     * x = 2 * (x*z + w*y)
     * y = 2 * (y*z - w*x)
     * z = 1 - 2 * (x*x + y*y)
     */
    v[0] = 2 * (q->x * q->z + q->w * q->y);
    v[1] = 2 * (q->y * q->z - q->w * q->x);
    v[2] = 1 - 2 * (sq(q->x) + sq(q->y));
}

void setpointUpdate(void)
{
    float deflection[4];

    for (int axis = 0; axis < 4; axis++) {
        deflection[axis] = getRcDeflection(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, deflection[axis] * 1000);

        float delta = sq(deflection[axis])- sp.maximum[axis];
        sp.maximum[axis] += delta * ((delta > 0) ? sp.maxGainUp : sp.maxGainDown);

        DEBUG_AXIS(SETPOINT, axis, 5, sp.maximum[axis]);
    }

    DEBUG(AIRBORNE, 0, sqrtf(sp.maximum[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, sqrtf(sp.maximum[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, sqrtf(sp.maximum[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, sqrtf(sp.maximum[FD_COLL]) * 1000);

    const float R = deflection[FD_ROLL]  * sp.ringLimit;
    const float P = deflection[FD_PITCH] * sp.ringLimit;
    const float C = sqrtf(sq(R) + sq(P));

    if (C > 1.0f) {
        deflection[FD_ROLL]  /= C;
        deflection[FD_PITCH] /= C;
    }

    for (int axis = 0; axis < 4; axis++) {
        float SP = deflection[axis];

        // rcCommand[YAW] CW direction is positive, while gyro[YAW] is negative
        if (axis == FD_YAW)
            SP = -SP;

        SP = filterApply(&sp.smoothingFilter[axis], SP);
        DEBUG_AXIS(SETPOINT, axis, 2, SP * 1000);

        SP = sp.deflection[axis] = setpointResponseAccel(axis, SP);
        DEBUG_AXIS(SETPOINT, axis, 3, SP * 1000);

        SP = sp.setpoint[axis] = applyRatesCurve(axis, SP);
        DEBUG_AXIS(SETPOINT, axis, 4, SP);
    }

    static float groundYawVector[3] = {0, 0, 1};
    switch (airborneState) {
    case GROUND:
        if (ARMING_FLAG(ARMED) && getGovernorOutput() > 0.1f) {
            // Transit to SPOOLUP and record ground yaw vector.
            airborneState = SPOOLUP;
            quaternion q;
            getQuaternion(&q);
            getYawVector(groundYawVector, &q);
        }
        break;
    case SPOOLUP: {
        // Compute tilt angle from the ground yaw vector.
        // We effectively ignore the yaw component, since the heli may rotate in
        // yaw when spooling up.
        float yawVector[3];
        quaternion q;
        getQuaternion(&q);
        getYawVector(yawVector, &q);

        float tiltAngle =
            angleOfVectors(groundYawVector, yawVector) / M_RADf; // in degree

        if (!isnormal(tiltAngle)) {
            // If something is broken (how?), we assume it's airborne.
            tiltAngle = 10;
        }

        DEBUG(AIRBORNE, 4, tiltAngle * 1000);

        if (ARMING_FLAG(ARMED) && isSpooledUp() && fabsf(tiltAngle) > 5) {
            airborneState = AIRBORNE;
        }

        if (!ARMING_FLAG(ARMED) || getGovernorOutput() < 0.1f) {
            airborneState = GROUND;
        }
        break;
    }
    case AIRBORNE:
        if (!ARMING_FLAG(ARMED) || (!isSpooledUp() && !isHandsOn())) {
            airborneState = GROUND;
        }
        break;
    }

    DEBUG(AIRBORNE, 5, airborneState);
    DEBUG(AIRBORNE, 6, isHandsOn());
    DEBUG(AIRBORNE, 7, isAirborne());
}

bool isHandsOn(void)
{
    return (
        sp.maximum[FD_ROLL] > sp.movementThreshold[FD_ROLL] ||
        sp.maximum[FD_PITCH] > sp.movementThreshold[FD_PITCH] ||
        sp.maximum[FD_YAW] > sp.movementThreshold[FD_YAW]
        // FD_COLL is not considered since the axis can stop at any position.
    );
}

bool isAirborne(void)
{
    return airborneState == AIRBORNE;
}
