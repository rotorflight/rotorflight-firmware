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

#define SP_BOOST_SCALE                   0.1e-4f

#define DYNAMIC_DEADBAND_SCALE             5e-4f
#define DYNAMIC_DEADBAND_LIMIT             0.40f
#define DYNAMIC_CEILING_LIMIT              0.40f


typedef struct
{
    float setpoint[4];
    float deflection[4];

    bool  polarCoord;
    float ringLimit[2];

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

    float boostGain[4];
    difFilter_t boostFilter[4];

    float yawDynamicCeiling;
    float yawDynamicCeilingGain;
    float yawDynamicDeadband;
    float yawDynamicDeadbandGain;

    pt1Filter_t yawDynamicSepointLPF;
    difFilter_t yawDynamicSepointDiff;

} setpointData_t;

static FAST_DATA_ZERO_INIT setpointData_t sp;


//// Adjustment Functions

int get_ADJUSTMENT_PITCH_SP_BOOST_GAIN(void)
{
    return currentControlRateProfile->setpoint_boost_gain[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_SP_BOOST_GAIN(int value)
{
    currentControlRateProfile->setpoint_boost_gain[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_SP_BOOST_GAIN(void)
{
    return currentControlRateProfile->setpoint_boost_gain[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_SP_BOOST_GAIN(int value)
{
    currentControlRateProfile->setpoint_boost_gain[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_SP_BOOST_GAIN(void)
{
    return currentControlRateProfile->setpoint_boost_gain[FD_YAW];
}

void set_ADJUSTMENT_YAW_SP_BOOST_GAIN(int value)
{
    currentControlRateProfile->setpoint_boost_gain[FD_YAW] = value;
}

int get_ADJUSTMENT_COLL_SP_BOOST_GAIN(void)
{
    return currentControlRateProfile->setpoint_boost_gain[FD_COLL];
}

void set_ADJUSTMENT_COLL_SP_BOOST_GAIN(int value)
{
    currentControlRateProfile->setpoint_boost_gain[FD_COLL] = value;
}

int get_ADJUSTMENT_YAW_DYN_CEILING_GAIN(void)
{
    return currentControlRateProfile->yaw_dynamic_ceiling_gain;
}

void set_ADJUSTMENT_YAW_DYN_CEILING_GAIN(int value)
{
    currentControlRateProfile->yaw_dynamic_ceiling_gain = value;
    sp.yawDynamicCeilingGain = value * DYNAMIC_DEADBAND_SCALE;
}

int get_ADJUSTMENT_YAW_DYN_DEADBAND_GAIN(void)
{
    return currentControlRateProfile->yaw_dynamic_deadband_gain;
}

void set_ADJUSTMENT_YAW_DYN_DEADBAND_GAIN(int value)
{
    currentControlRateProfile->yaw_dynamic_deadband_gain = value;
    sp.yawDynamicDeadbandGain = value * DYNAMIC_DEADBAND_SCALE;
}

int get_ADJUSTMENT_YAW_DYN_DEADBAND_FILTER(void)
{
    return currentControlRateProfile->yaw_dynamic_deadband_filter;
}

void set_ADJUSTMENT_YAW_DYN_DEADBAND_FILTER(int value)
{
    currentControlRateProfile->yaw_dynamic_deadband_filter = value;
    pt1FilterUpdate(&sp.yawDynamicSepointLPF, value / 10.0f, pidGetPidFrequency());
}


//// Access Functions

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
        }
        sp.smoothingCutoff = cutoff;
    }
}

INIT_CODE void setpointInitProfile(void)
{
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

        sp.boostGain[i] = currentControlRateProfile->setpoint_boost_gain[i] * SP_BOOST_SCALE;
        difFilterUpdate(&sp.boostFilter[i], currentControlRateProfile->setpoint_boost_cutoff[i], pidGetPidFrequency());
    }

    sp.yawDynamicCeilingGain = currentControlRateProfile->yaw_dynamic_ceiling_gain * DYNAMIC_DEADBAND_SCALE;
    sp.yawDynamicDeadbandGain = currentControlRateProfile->yaw_dynamic_deadband_gain * DYNAMIC_DEADBAND_SCALE;

    difFilterUpdate(&sp.yawDynamicSepointDiff, currentControlRateProfile->yaw_dynamic_deadband_cutoff, pidGetPidFrequency());
    pt1FilterUpdate(&sp.yawDynamicSepointLPF, currentControlRateProfile->yaw_dynamic_deadband_filter / 10.0f, pidGetPidFrequency());

    sp.polarCoord = currentControlRateProfile->cyclic_polar;

    if (currentControlRateProfile->cyclic_ring) {
        const float cyclicLimit = currentControlRateProfile->cyclic_ring / 100.0f;
        sp.ringLimit[FD_ROLL]  = applyRatesCurve(FD_ROLL, 1.0f)  * cyclicLimit;
        sp.ringLimit[FD_PITCH] = applyRatesCurve(FD_PITCH, 1.0f) * cyclicLimit;
    }
    else {
        sp.ringLimit[FD_ROLL]  = 2000;
        sp.ringLimit[FD_PITCH] = 2000;
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
        difFilterInit(&sp.boostFilter[i], currentControlRateProfile->setpoint_boost_cutoff[i], pidGetPidFrequency());
    }
    difFilterInit(&sp.yawDynamicSepointDiff, currentControlRateProfile->yaw_dynamic_deadband_cutoff, pidGetPidFrequency());
    pt1FilterInit(&sp.yawDynamicSepointLPF, currentControlRateProfile->yaw_dynamic_deadband_filter / 10.0f, pidGetPidFrequency());

    setpointInitProfile();
}

static float applyYawDynamicRange(float setpoint)
{
    const float spdiff = difFilterApply(&sp.yawDynamicSepointDiff, setpoint);
    const float factor = pt1FilterApply(&sp.yawDynamicSepointLPF, fabsf(spdiff));

    sp.yawDynamicCeiling = constrainf(factor  * sp.yawDynamicCeilingGain, 0, DYNAMIC_CEILING_LIMIT);
    sp.yawDynamicDeadband = constrainf(factor  * sp.yawDynamicDeadbandGain, 0, DYNAMIC_DEADBAND_LIMIT);

    const float point = fapplyDeadband(setpoint, sp.yawDynamicDeadband);
    const float range = 1.0f - sp.yawDynamicDeadband - sp.yawDynamicCeiling;

    setpoint = limitf(point / range, 1.0f);

    return setpoint;
}

static void airborneDebug(void)
{
    DEBUG(AIRBORNE, 0, sqrtf(sp.maximum[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, sqrtf(sp.maximum[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, sqrtf(sp.maximum[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, sqrtf(sp.maximum[FD_COLL]) * 1000);
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, isSpooledUp());
    DEBUG(AIRBORNE, 6, isHandsOn());
    DEBUG(AIRBORNE, 7, isAirborne());
}

void setpointUpdate(void)
{
    float SP[4];

    for (int axis = 0; axis < 4; axis++) {
        SP[axis] = getRcDeflection(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, SP[axis] * 1000);

        float delta = sq(SP[axis])- sp.maximum[axis];
        sp.maximum[axis] += delta * ((delta > 0) ? sp.maxGainUp : sp.maxGainDown);
    }

    // rcCommand[YAW] CW direction is positive, while gyro[YAW] is negative
    SP[FD_YAW] = -SP[FD_YAW];

    for (int axis = 0; axis < 4; axis++) {
        SP[axis] = filterApply(&sp.smoothingFilter[axis], SP[axis]);
        DEBUG_AXIS(SETPOINT, axis, 1, SP[axis] * 1000);
    }

    SP[FD_YAW] = applyYawDynamicRange(SP[FD_YAW]);
    DEBUG_AXIS(SETPOINT, FD_YAW, 2, SP[FD_YAW] * 1000);

    for (int axis = 0; axis < 4; axis++) {
        SP[axis] = sp.deflection[axis] = setpointResponseAccel(axis, SP[axis]);
        DEBUG_AXIS(SETPOINT, axis, 3, SP[axis] * 1000);

        SP[axis] += difFilterApply(&sp.boostFilter[axis], SP[axis]) * sp.boostGain[axis];
        DEBUG_AXIS(SETPOINT, axis, 4, SP[axis]);
    }

    if (sp.polarCoord) {
        const float POLAR = sqrtf(sq(SP[FD_ROLL]) + sq(SP[FD_PITCH]));
        const float RATES = fminf(applyRatesCurve(FD_PITCH, POLAR), sp.ringLimit[FD_PITCH]);
        const float RATIO = (POLAR > 0.001f) ? RATES / POLAR : 0;

        SP[FD_ROLL] = SP[FD_ROLL] * RATIO;
        SP[FD_PITCH] = SP[FD_PITCH] * RATIO;

        DEBUG(POLAR_SP, 2, RATES);
        DEBUG(POLAR_SP, 3, RATIO);

        DEBUG_AXIS(SETPOINT, FD_ROLL, 5, SP[FD_ROLL]);
        DEBUG_AXIS(SETPOINT, FD_PITCH, 5, SP[FD_PITCH]);
    }
    else {
        SP[FD_ROLL] = applyRatesCurve(FD_ROLL, SP[FD_ROLL]);
        SP[FD_PITCH] = applyRatesCurve(FD_PITCH, SP[FD_PITCH]);

        DEBUG_AXIS(SETPOINT, FD_ROLL, 5, SP[FD_ROLL]);
        DEBUG_AXIS(SETPOINT, FD_PITCH, 5, SP[FD_PITCH]);

        const float R = SP[FD_ROLL]  / sp.ringLimit[FD_ROLL];
        const float P = SP[FD_PITCH] / sp.ringLimit[FD_PITCH];
        const float C = sqrtf(sq(R) + sq(P));

        if (C > 1.0f) {
            SP[FD_ROLL]  /= C;
            SP[FD_PITCH] /= C;
        }
    }

    DEBUG(POLAR_SP, 0, SP[FD_ROLL]);
    DEBUG(POLAR_SP, 1, SP[FD_PITCH]);

    SP[FD_YAW] = applyRatesCurve(FD_YAW, SP[FD_YAW]);
    DEBUG_AXIS(SETPOINT, FD_YAW, 5, SP[FD_YAW]);

    SP[FD_COLL] = applyRatesCurve(FD_COLL, SP[FD_COLL]);
    DEBUG_AXIS(SETPOINT, FD_COLL, 5, SP[FD_COLL]);

    for (int axis = 0; axis < 4; axis++) {
        sp.setpoint[axis] = SP[axis];
        DEBUG_AXIS(SETPOINT, axis, 7, SP[axis] * 1000);
    }

    airborneDebug();
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
