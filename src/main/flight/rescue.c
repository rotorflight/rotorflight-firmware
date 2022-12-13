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

enum {
    RSTATE_OFF = 0,
    RSTATE_PULLUP,
    RSTATE_FLIP,
    RSTATE_CLIMB,
    RSTATE_HOVER,
    RSTATE_EXIT,
};

typedef struct {

    /* Config parameters */

    uint8_t         mode;
    uint8_t         flip;

    uint8_t         state;
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

    float           alt_Ka;
    float           alt_Kp;
    float           alt_Ki;

    float           alt_Iterm;

    /* Setpoint limits */

    float           maxRate;
    float           maxAccel;
    float           maxColl;
    float           maxClimb;

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

    if (newState == RSTATE_CLIMB)
        rescue.alt_Iterm = rescue.prevSetpoint[FD_COLL];
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
    if (rescue.state == RSTATE_OFF) {
        rescue.prevSetpoint[axis] = rescue.setpoint[axis] = setpoint;
    }
    else if (rescue.state == RSTATE_EXIT) {
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
        rescue.setpoint[i] = constrainf(rescue.setpoint[i], -rescue.maxRate, rescue.maxRate);
        rescue.setpoint[i] = slewLimit(rescue.prevSetpoint[i], rescue.setpoint[i], rescue.maxAccel);
        rescue.prevSetpoint[i] = rescue.setpoint[i];
    }

    // Collective limit
    rescue.setpoint[FD_COLL] = constrainf(rescue.setpoint[FD_COLL], -rescue.maxColl, rescue.maxColl);
    rescue.prevSetpoint[FD_COLL] = rescue.setpoint[FD_COLL];
}

static void rescueApplyLeveling(bool allow_inverted)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    float rollError = -(attitude.values.roll - trim->values.roll);
    float pitchError = -(attitude.values.pitch - trim->values.pitch);

    if (allow_inverted) {
        if (attitude.values.roll > 900) {
            pitchError = -pitchError;
            rollError += 1800;
        }
        else if (attitude.values.roll < -900) {
            pitchError = -pitchError;
            rollError -= 1800;
        }
    }

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

    float rollError = getRcDeflection(FD_ROLL) * RESCUE_MAX_DECIANGLE  -
        (attitude.values.roll - trim->values.roll);
    float pitchError = getRcDeflection(FD_PITCH) * RESCUE_MAX_DECIANGLE -
        (attitude.values.pitch - trim->values.pitch);

    if (allow_inverted) {
        if (attitude.values.roll > 900) {
            pitchError = -pitchError;
            rollError += 1800;
        }
        else if (attitude.values.roll < -900) {
            pitchError = -pitchError;
            rollError -= 1800;
        }
    }

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

    float alterr = altitude - getAltitude();
    alterr = copysignf(sqrtf(fabsf(alterr)), alterr);

    float climb = alterr * rescue.alt_Ka;
    climb = constrainf(climb, -rescue.maxClimb, rescue.maxClimb);

    float error = climb - getVario();
    float Pterm = error * rescue.alt_Kp;
    float Iterm = error * rescue.alt_Ki * tilt * tilt + rescue.alt_Iterm;

    Iterm = constrainf(Iterm, -rescue.maxColl, rescue.maxColl);

    float pidSum = Pterm + Iterm;

    pidSum = constrainf(pidSum, -rescue.maxColl, rescue.maxColl);

    DEBUG(ALT_HOLD, 0, climb * 100);
    DEBUG(ALT_HOLD, 1, Pterm);
    DEBUG(ALT_HOLD, 2, Iterm);
    DEBUG(ALT_HOLD, 3, pidSum);

    rescue.alt_Iterm = Iterm;

    return pidSum;
}

static void rescueApplyClimbCollective(void)
{
    const float tilt = getCosTiltAngle();
    float collective = 0;

    if (rescue.mode == RESCUE_MODE_CLIMB) {
        collective = rescue.climbCollective;
    }
    else if (rescue.mode == RESCUE_MODE_ALT_HOLD) {
        collective = rescueApplyAltitudePID(rescue.hoverAltitude);
    }

    rescue.setpoint[FD_COLL] = collective * copysignf(tilt*tilt, tilt);
}

static void rescueApplyHoverCollective(void)
{
    const float tilt = getCosTiltAngle();
    float collective = 0;

    if (rescue.mode == RESCUE_MODE_CLIMB) {
        collective = rescue.hoverCollective + 0.5f * getSetpoint(FD_COLL);
    }
    else if (rescue.mode == RESCUE_MODE_ALT_HOLD) {
        collective = rescueApplyAltitudePID(rescue.hoverAltitude);
    }

    rescue.setpoint[FD_COLL] = collective * copysignf(tilt*tilt, tilt);
}

static void rescueApplyCollective(float collective)
{
    const float tilt = getCosTiltAngle();

    rescue.setpoint[FD_COLL] = collective * copysignf(tilt*tilt, tilt);
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
    return (rescueStateTime() > rescue.climbTime);
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
    // Handle DISARM separately
    if (!ARMING_FLAG(ARMED)) {
        rescueChangeState(RSTATE_OFF);
    }
    else {
        switch (rescue.state)
        {
            case RSTATE_OFF:
                if (rescueActive()) {
                    rescueChangeState(RSTATE_PULLUP);
                    rescuePullUp();
                }
                break;

            case RSTATE_PULLUP:
                rescuePullUp();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescuePullUpDone()) {
                    if (rescueIsLeveled()) {
                        if (rescue.flip && rescueIsInverted())
                            rescueChangeState(RSTATE_FLIP);
                        else
                            rescueChangeState(RSTATE_CLIMB);
                    }
                    else {
                        rescueChangeState(RSTATE_EXIT);
                    }
                }
                break;

            case RSTATE_FLIP:
                rescueFlipOver();
                if (rescueFlipDone()) {
                    if (!rescueActive())
                        rescueChangeState(RSTATE_EXIT);
                    else
                        rescueChangeState(RSTATE_CLIMB);
                }
                else if (rescueFlipTimeout()) {
                    if (rescueIsLeveled())
                        rescueChangeState(RSTATE_CLIMB);
                    else
                        rescueChangeState(RSTATE_EXIT);
                }
                break;

            case RSTATE_CLIMB:
                rescueClimb();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescueClimbDone())
                    rescueChangeState(RSTATE_HOVER);
                break;

            case RSTATE_HOVER:
                rescueHover();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                break;

            case RSTATE_EXIT:
                if (rescueActive())
                    rescueChangeState(RSTATE_PULLUP);
                else if (rescueSlowExitDone())
                    rescueChangeState(RSTATE_OFF);
                break;
        }
    }
}


//// Interface functions

uint8_t getRescueState(void)
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
    rescue.maxClimb = pidProfile->rescue.max_climb_rate / 100.0f;

    rescue.pullUpTime = pidProfile->rescue.pull_up_time * 100;
    rescue.climbTime = pidProfile->rescue.climb_time * 100;
    rescue.flipTime = pidProfile->rescue.flip_time * 100;
    rescue.exitTime = pidProfile->rescue.exit_time * 100;

    rescue.pullUpCollective = pidProfile->rescue.pull_up_collective;
    rescue.climbCollective = pidProfile->rescue.climb_collective;
    rescue.hoverCollective = pidProfile->rescue.hover_collective;

    rescue.hoverAltitude = pidProfile->rescue.hover_altitude / 100.0f;

    rescue.alt_Ka = pidProfile->rescue.alt_a_gain;
    rescue.alt_Kp = pidProfile->rescue.alt_p_gain;
    rescue.alt_Ki = pidProfile->rescue.alt_i_gain * pidGetDT();
}
