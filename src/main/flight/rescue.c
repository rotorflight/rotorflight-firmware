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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/position.h"
#include "flight/rescue.h"
#include "flight/pid.h"
#include "flight/imu.h"


#define RESCUE_MAX_DECIANGLE  450

typedef struct {

    /* Config parameters */

    rescueMode_e    mode;
    bool            flip;

    rescueState_e   state;
    timeMs_t        stateEntryTime;

    timeDelta_t     pullUpTime;
    timeDelta_t     climbTime;
    timeDelta_t     flipTime;
    timeDelta_t     exitTime;

    float           levelGain;
    float           flipGain;

    /* Climb only */

    float           pullUpCollective;
    float           climbCollective;
    float           hoverCollective;

    /* Altitude hold */

    float           hoverAltitude;

    float           alt_Kp;
    float           alt_Ki;
    float           alt_Kd;

    float           alt_Iterm;

    /* Setpoint limits */

    float           maxRate;
    float           maxAccel;
    float           maxColl;

    /* Setpoint output */

    float           setpoint[4];
    float           prevSetpoint[4];

} rescueState_t;

static FAST_DATA_ZERO_INIT rescueState_t rescue;


//// Internal functions

static inline void rescueChangeState(uint8_t newState)
{
    rescue.state = newState;
    rescue.stateEntryTime = millis();

    if (newState == RESCUE_STATE_CLIMB)
        rescue.alt_Iterm = rescue.hoverCollective;
}

static inline timeDelta_t rescueStateTime(void)
{
    return cmp32(millis(), rescue.stateEntryTime);
}

static inline bool rescueActive(void)
{
    return FLIGHT_MODE(RESCUE_MODE);
}

static inline float rescueSetpoint(uint8_t axis, float setpoint)
{
    if (rescue.state == RESCUE_STATE_OFF) {
        rescue.prevSetpoint[axis] = rescue.setpoint[axis] = setpoint;
    }
    else if (rescue.state == RESCUE_STATE_EXIT) {
        float alpha = (float)rescueStateTime() / (float)(rescue.exitTime + 1);
        setpoint = alpha * setpoint + (1.0f - alpha) * rescue.setpoint[axis];
    }
    else {
        setpoint = rescue.setpoint[axis];
    }

    return setpoint;
}

static inline bool rescueIsInverted(void)
{
    return getCosTiltAngle() < 0;
}

static inline bool rescueIsLeveled(void)
{
    return fabsf(getCosTiltAngle()) > 0.866f; // less than 30deg error from level
}

static void rescueApplyLimits(void)
{
    // Limits for RPY
    for (int i=0; i<3; i++) {
        rescue.setpoint[i] = limitf(rescue.setpoint[i], rescue.maxRate);
        rescue.setpoint[i] = slewLimit(rescue.prevSetpoint[i], rescue.setpoint[i], rescue.maxAccel);
    }

    // Previous values
    for (int i=0; i<4; i++) {
        rescue.prevSetpoint[i] = rescue.setpoint[i];
    }
}

static inline int wrap180(int angle)
{
    while (angle > 1800)
        angle -= 3600;
    while (angle < -1800)
        angle += 3600;
    return angle;
}

static void rescueApplyLeveling(bool allow_inverted)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int rollAngle = wrap180(attitude.values.roll - trim->values.roll);
    const int pitchAngle = wrap180(attitude.values.pitch - trim->values.pitch);

    int rollError, pitchError;

    if (allow_inverted && (rollAngle > 900 || rollAngle < -900)) {
        pitchError = pitchAngle;
        rollError = -rollAngle + 1800;
    }
    else {
        pitchError = -pitchAngle;
        rollError = -rollAngle;
    }

    pitchError = wrap180(pitchError);
    rollError = wrap180(rollError);

    // Avoid "gimbal lock"
    if (attitude.values.pitch > 800 || attitude.values.pitch < -800) {
        rollError = 0;
    }

    rescue.setpoint[FD_PITCH] = pitchError * rescue.flipGain;
    rescue.setpoint[FD_ROLL] = rollError * rescue.flipGain;
    rescue.setpoint[FD_YAW] = 0;
}

static void rescueApplyStabilisation(bool allow_inverted)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int rollAngle = wrap180(attitude.values.roll - trim->values.roll);
    const int pitchAngle = wrap180(attitude.values.pitch - trim->values.pitch);

    int rollError = getRcDeflection(FD_ROLL) * RESCUE_MAX_DECIANGLE;
    int pitchError = getRcDeflection(FD_PITCH) * RESCUE_MAX_DECIANGLE;

    if (allow_inverted && (rollAngle > 900 || rollAngle < -900)) {
        pitchError += pitchAngle;
        rollError -= rollAngle - 1800;
    }
    else {
        pitchError -= pitchAngle;
        rollError -= rollAngle;
    }

    pitchError = wrap180(pitchError);
    rollError = wrap180(rollError);

    // Avoid "gimbal lock"
    if (attitude.values.pitch > 800 || attitude.values.pitch < -800) {
        rollError = 0;
    }

    rescue.setpoint[FD_PITCH] = pitchError * rescue.levelGain;
    rescue.setpoint[FD_ROLL] = rollError * rescue.levelGain;
    rescue.setpoint[FD_YAW] = getSetpoint(FD_YAW);
}

static float rescueApplyAltitudePID(float altitude)
{
    const float tilt = getCosTiltAngle();

    const float error = altitude - getAltitude();
    const float sqerr = copysignf(sqrtf(fabsf(error)), error);

    float Pterm = error * rescue.alt_Kp;
    float Iterm = error * rescue.alt_Ki * tilt * tilt + rescue.alt_Iterm;
    float Dterm = getVario() * rescue.alt_Kd;

    Iterm = constrainf(Iterm, 0, rescue.maxColl);

    rescue.alt_Iterm = Iterm;

    float pidSum = fminf(Pterm + Iterm + Dterm, rescue.maxColl);

    DEBUG(RESCUE_ALTHOLD, 0, error * 100);
    DEBUG(RESCUE_ALTHOLD, 1, sqerr * 100);
    DEBUG(RESCUE_ALTHOLD, 2, Pterm);
    DEBUG(RESCUE_ALTHOLD, 3, Iterm);
    DEBUG(RESCUE_ALTHOLD, 4, Dterm);
    DEBUG(RESCUE_ALTHOLD, 5, pidSum);

    return pidSum;
}

static void rescueApplyClimbCollective(void)
{
    const float tilt = getCosTiltAngle();
    const float tlt2 = tilt * fabsf(tilt);

    if (rescue.mode == RESCUE_MODE_CLIMB) {
        rescue.setpoint[FD_COLL] = rescue.climbCollective * tlt2;
    }
    else if (rescue.mode == RESCUE_MODE_ALT_HOLD) {
        rescue.setpoint[FD_COLL] = rescueApplyAltitudePID(rescue.hoverAltitude) * tlt2;
    }
}

static void rescueApplyHoverCollective(void)
{
    const float tilt = getCosTiltAngle();
    const float tlt2 = tilt * fabsf(tilt);

    if (rescue.mode == RESCUE_MODE_CLIMB) {
        rescue.setpoint[FD_COLL] = rescue.hoverCollective * tlt2 + 0.25f * getSetpoint(FD_COLL);
    }
    else if (rescue.mode == RESCUE_MODE_ALT_HOLD) {
        rescue.setpoint[FD_COLL] = rescueApplyAltitudePID(rescue.hoverAltitude) * tlt2;
    }
}

static void rescueApplyCollective(float collective)
{
    const float tilt = getCosTiltAngle();
    const float tlt2 = tilt * fabsf(tilt);

    rescue.setpoint[FD_COLL] = collective * tlt2;
}

static void rescuePullUp(void)
{
    rescueApplyLeveling(true);
    rescueApplyCollective(rescue.pullUpCollective);
    rescueApplyLimits();
}

static inline bool rescuePullUpDone(void)
{
    return (rescueStateTime() > rescue.pullUpTime);
}

static void rescueFlipOver(void)
{
    rescueApplyLeveling(false);
    rescueApplyCollective(rescue.pullUpCollective);
    rescueApplyLimits();
}

static inline bool rescueFlipDone(void)
{
    return (getCosTiltAngle() > 0.95f);
}

static inline bool rescueFlipTimeout(void)
{
    return (rescueStateTime() > rescue.flipTime);
}

static void rescueClimb(void)
{
    rescueApplyStabilisation(true);
    rescueApplyClimbCollective();
    rescueApplyLimits();
}

static inline bool rescueClimbDone(void)
{
    if (rescue.mode == RESCUE_MODE_CLIMB) {
        return (rescueStateTime() > rescue.climbTime);
    }
    else if (rescue.mode == RESCUE_MODE_ALT_HOLD) {
        return (rescueStateTime() > rescue.climbTime || fabsf(rescue.hoverAltitude - getAltitude()) < 0.5f);
    }

    return true;
}

static void rescueHover(void)
{
    rescueApplyStabilisation(true);
    rescueApplyHoverCollective();
    rescueApplyLimits();
}

static inline bool rescueSlowExitDone(void)
{
    return (rescueStateTime() > rescue.exitTime);
}


static void rescueUpdateState(void)
{
#ifdef RESCUE_REQUIRES_ARMING
    if (!ARMING_FLAG(ARMED)) {
        rescueChangeState(RESCUE_STATE_OFF);
    }
    else
#endif
    {
        switch (rescue.state)
        {
            case RESCUE_STATE_OFF:
                if (rescueActive()) {
                    rescueChangeState(RESCUE_STATE_PULLUP);
                    rescuePullUp();
                }
                break;

            case RESCUE_STATE_PULLUP:
                rescuePullUp();
                if (!rescueActive())
                    rescueChangeState(RESCUE_STATE_EXIT);
                else if (rescuePullUpDone()) {
                    if (rescueIsLeveled()) {
                        if (rescue.flip && rescueIsInverted())
                            rescueChangeState(RESCUE_STATE_FLIP);
                        else
                            rescueChangeState(RESCUE_STATE_CLIMB);
                    }
                    else {
                        rescueChangeState(RESCUE_STATE_EXIT);
                    }
                }
                break;

            case RESCUE_STATE_FLIP:
                rescueFlipOver();
                if (rescueFlipDone()) {
                    if (!rescueActive())
                        rescueChangeState(RESCUE_STATE_EXIT);
                    else
                        rescueChangeState(RESCUE_STATE_CLIMB);
                }
                else if (rescueFlipTimeout()) {
                    if (rescueIsLeveled())
                        rescueChangeState(RESCUE_STATE_CLIMB);
                    else
                        rescueChangeState(RESCUE_STATE_EXIT);
                }
                break;

            case RESCUE_STATE_CLIMB:
                rescueClimb();
                if (!rescueActive())
                    rescueChangeState(RESCUE_STATE_EXIT);
                else if (rescueClimbDone())
                    rescueChangeState(RESCUE_STATE_HOVER);
                break;

            case RESCUE_STATE_HOVER:
                rescueHover();
                if (!rescueActive())
                    rescueChangeState(RESCUE_STATE_EXIT);
                break;

            case RESCUE_STATE_EXIT:
                if (rescueActive())
                    rescueChangeState(RESCUE_STATE_PULLUP);
                else if (rescueSlowExitDone())
                    rescueChangeState(RESCUE_STATE_OFF);
                break;
        }
    }
}


//// Interface functions

int getRescueState(void)
{
    return rescue.state;
}

void rescueUpdate(void)
{
    if (rescue.mode) {
        rescueUpdateState();

        DEBUG(RESCUE, 0, attitude.values.roll);
        DEBUG(RESCUE, 1, attitude.values.pitch);
        DEBUG(RESCUE, 2, attitude.values.yaw);
        DEBUG(RESCUE, 3, getCosTiltAngle() * 1000);

        DEBUG(RESCUE, 4, rescue.setpoint[0]);
        DEBUG(RESCUE, 5, rescue.setpoint[1]);
        DEBUG(RESCUE, 6, rescue.setpoint[2]);
        DEBUG(RESCUE, 7, rescue.setpoint[3]);
    }
}

float rescueApply(uint8_t axis, float setpoint)
{
    if (rescue.mode)
        setpoint = rescueSetpoint(axis, setpoint);

    return setpoint;
}

void INIT_CODE rescueInitProfile(const pidProfile_t *pidProfile)
{
    rescue.mode = pidProfile->rescue.mode;
    rescue.flip = pidProfile->rescue.flip_mode;

    rescue.levelGain = pidProfile->rescue.level_gain / 250.0f;
    rescue.flipGain = pidProfile->rescue.flip_gain / 250.0f;

    rescue.maxRate = pidProfile->rescue.max_setpoint_rate;
    rescue.maxAccel = pidProfile->rescue.max_setpoint_accel * pidGetDT() * 10.0f;
    rescue.maxColl = pidProfile->rescue.max_collective;

    rescue.pullUpTime = pidProfile->rescue.pull_up_time * 100;
    rescue.climbTime = pidProfile->rescue.climb_time * 100;
    rescue.flipTime = pidProfile->rescue.flip_time * 100;
    rescue.exitTime = pidProfile->rescue.exit_time * 100;

    rescue.pullUpCollective = pidProfile->rescue.pull_up_collective;
    rescue.climbCollective = pidProfile->rescue.climb_collective;
    rescue.hoverCollective = pidProfile->rescue.hover_collective;

    rescue.hoverAltitude = pidProfile->rescue.hover_altitude / 100.0f;

    rescue.alt_Kp = pidProfile->rescue.alt_p_gain;
    rescue.alt_Ki = pidProfile->rescue.alt_i_gain * pidGetDT() / 10.0f;
    rescue.alt_Kd = pidProfile->rescue.alt_d_gain * -1.0f;
}
