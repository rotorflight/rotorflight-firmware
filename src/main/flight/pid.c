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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/trainer.h"
#include "flight/leveling.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

#if defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 3);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .levelAngleLimit = 55,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .itermLimit = 400,
        .iterm_rotation = false,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .profileName = { 0 },
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}


const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (pidRuntime.itermRotation) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

#if defined(USE_ACC)
        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
            currentPidSetpoint = pidLevelApply(axis, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            currentPidSetpoint = acroTrainerApply(axis, currentPidSetpoint);
        }
#endif

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate;

        // -----calculate I component
        float Ki = pidRuntime.pidCoefficient[axis].Ki;
        pidData[axis].I = constrainf(previousIterm + Ki * itermErrorRate * pidRuntime.dT, -pidRuntime.itermLimit, pidRuntime.itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0; // FIXME: Calculate setpoint delta here
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate D component
        if (pidRuntime.pidCoefficient[axis].Kd > 0) {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float gyroRateDterm = gyro.gyroDtermADCf[axis];
            const float delta = (pidRuntime.previousGyroRateDterm[axis] - gyroRateDterm) * pidRuntime.pidFrequency;
            pidRuntime.previousGyroRateDterm[axis] = gyroRateDterm;
            pidData[axis].D = pidRuntime.pidCoefficient[axis].Kd * delta;

        } else {
            pidData[axis].D = 0;
        }

        // -----calculate feedforward component
        float feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // halve feedforward in Level mode since stick sensitivity is weaker by about half
            feedforwardGain *= FLIGHT_MODE(ANGLE_MODE) ? 0.5f : 1.0f;
            // transition now calculated in feedforward.c when new RC data arrives
            float feedForward = feedforwardGain * pidSetpointDelta * pidRuntime.pidFrequency;
            pidData[axis].F = feedForward;
        } else {
            pidData[axis].F = 0;
        }

        // calculating the PID sum
        pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    }
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT()
{
    return pidRuntime.dT;
}

float pidGetPidFrequency()
{
    return pidRuntime.pidFrequency;
}
