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
#include "common/maths.h"

#include "config/config_reset.h"

#include "drivers/dshot_command.h"
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
#include "flight/setpoint.h"
#include "flight/leveling.h"
#include "flight/trainer.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90 },
            [PID_PITCH] = { 46, 90, 38, 95 },
            [PID_YAW] =   { 45, 90, 0, 90 },
            [PID_LEVEL] = { 50, 50, 75, 0 },
        },
        .levelAngleLimit = 55,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .itermLimit = 400,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .profileName = { 0 },
        .ff_interpolate_sp = FF_INTERPOLATE_AVG2,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .yaw_cyclic_ff_gain = 0,
        .yaw_collective_ff_gain = 300,
        .yaw_collective_ff_impulse_gain = 300,
        .yaw_collective_ff_impulse_freq = 100,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float tailCyclicFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveImpulseFFGain;

static FAST_RAM_ZERO_INIT float collectiveDeflectionLPF;
static FAST_RAM_ZERO_INIT float collectiveDeflectionHPF;
static FAST_RAM_ZERO_INIT float collectiveImpulseFilterGain;

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static uint8_t itermRelaxCutoff;
#endif

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acGain;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acCutoff;
static FAST_RAM_ZERO_INIT pt1Filter_t acLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif


void pidInitFilters(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);
    
    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        return;
    }

#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif
#if defined(USE_ABSOLUTE_CONTROL)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&acLpf[i], pt1FilterGain(acCutoff, dT));
        }
    }
#endif
}


typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float itermLimit;
static FAST_RAM_ZERO_INIT bool itermRotation;

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}


#ifdef USE_INTERPOLATED_SP
static FAST_RAM_ZERO_INIT bool spInterpolation;
#endif


void pidInitConfig(const pidProfile_t *pidProfile)
{
    // Roll axis
    pidCoefficient[FD_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[FD_ROLL].P;
    pidCoefficient[FD_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[FD_ROLL].I;
    pidCoefficient[FD_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[FD_ROLL].D;
    pidCoefficient[FD_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[FD_ROLL].F;

    // Pitch axis
    pidCoefficient[FD_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[FD_PITCH].P;
    pidCoefficient[FD_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[FD_PITCH].I;
    pidCoefficient[FD_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[FD_PITCH].D;
    pidCoefficient[FD_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[FD_PITCH].F;

    // Yaw axis
    pidCoefficient[FD_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[FD_YAW].P;
    pidCoefficient[FD_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[FD_YAW].I;
    pidCoefficient[FD_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[FD_YAW].D;
    pidCoefficient[FD_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[FD_YAW].F;

    itermLimit = pidProfile->itermLimit;
    itermRotation = pidProfile->iterm_rotation;

#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif

#if defined(USE_ABSOLUTE_CONTROL)
    acGain = (float)pidProfile->abs_control_gain;
    acLimit = (float)pidProfile->abs_control_limit;
    acErrorLimit = (float)pidProfile->abs_control_error_limit;
    acCutoff = (float)pidProfile->abs_control_cutoff;
    float rollCorrection  = -acGain * ROLL_P_TERM_SCALE  / ROLL_I_TERM_SCALE  * pidCoefficient[FD_ROLL].Kp;
    float pitchCorrection = -acGain * PITCH_P_TERM_SCALE / PITCH_I_TERM_SCALE * pidCoefficient[FD_PITCH].Kp;
    float yawCorrection   = -acGain * YAW_P_TERM_SCALE   / YAW_I_TERM_SCALE   * pidCoefficient[FD_YAW].Kp;
    pidCoefficient[FD_ROLL].Ki  = MAX(0.0f, pidCoefficient[FD_ROLL].Ki  + rollCorrection);
    pidCoefficient[FD_PITCH].Ki = MAX(0.0f, pidCoefficient[FD_PITCH].Ki + pitchCorrection);
    pidCoefficient[FD_YAW].Ki   = MAX(0.0f, pidCoefficient[FD_YAW].Ki   + yawCorrection);
#endif

#ifdef USE_INTERPOLATED_SP
    spInterpolation = (pidProfile->ff_interpolate_sp != FF_INTERPOLATE_OFF);
    interpolatedSpInit(pidProfile);
#endif

    // Collective impulse high-pass filter
    collectiveImpulseFilterGain = pt1FilterGain(pidProfile->yaw_collective_ff_impulse_freq / 100.0f, dT);

    // Tail feedforward gains
    tailCyclicFFGain = pidProfile->yaw_cyclic_ff_gain;
    tailCollectiveFFGain = pidProfile->yaw_collective_ff_gain;
    tailCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
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
    if (itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (itermRotation) {
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


#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&acLpf[axis], *currentPidSetpoint);
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        // Check to ensure we are spooled up at a reasonable level
        if (isSpooledUp()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * dT, -acErrorLimit, acErrorLimit);
            const float acCorrection = constrainf(axisError[axis] * acGain, -acLimit, acLimit);
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }

        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif


// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
#if !defined(USE_ACC)
    UNUSED(currentTimeUs);
#endif

    static float previousGyroRateDterm[XYZ_AXIS_COUNT];

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        levelMode = LEVEL_MODE_RP;
    } else {
        levelMode = LEVEL_MODE_OFF;
    }

    // Keep track of when we entered a self-level mode so that we can
    // add a guard time before crash recovery can activate.
    // Also reset the guard time whenever GPS Rescue is activated.
    if (levelMode) {
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }
    gpsRescuePreviousState = gpsRescueIsActive;
#endif

    rotateItermAndAxisError();

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
#if defined(USE_ACC)
        switch (levelMode) {
        case LEVEL_MODE_OFF:

            break;
        case LEVEL_MODE_R:
            if (axis == FD_PITCH) {
                break;
            }

            FALLTHROUGH;
        case LEVEL_MODE_RP:
            if (axis == FD_YAW) {
                break;
            }
            currentPidSetpoint = pidLevelApply(axis, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        currentPidSetpoint = acroTrainerApply(axis, currentPidSetpoint);
#endif // USE_ACRO_TRAINER

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y
        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;
#ifdef USE_ABSOLUTE_CONTROL
        float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate;

        // -----calculate I component
        float Ki = pidCoefficient[axis].Ki;
        pidData[axis].I = constrainf(previousIterm + Ki * dT * itermErrorRate, -itermLimit, itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_INTERPOLATED_SP
        if (spInterpolation) {
            pidSetpointDelta = interpolatedSpApply(axis);
        } else
#endif
        {
            pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
        }

        previousPidSetpoint[axis] = currentPidSetpoint;

#ifdef USE_RC_SMOOTHING_FILTER
        pidSetpointDelta = rcSmoothingApplyDerivativeFilter(axis, pidSetpointDelta);
#endif

        // -----calculate D component
        if (pidCoefficient[axis].Kd > 0) {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            float gyroRateDterm = gyro.gyroDtermADCf[axis];
            const float delta =
                - (gyroRateDterm - previousGyroRateDterm[axis]) * pidFrequency;

            pidData[axis].D = pidCoefficient[axis].Kd * delta;
            previousGyroRateDterm[axis] = gyroRateDterm;
        } else {
            pidData[axis].D = 0;
        }

        // -----calculate feedforward component
#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in FF
        pidSetpointDelta += setpointCorrection - oldSetpointCorrection[axis];
        oldSetpointCorrection[axis] = setpointCorrection;
#endif

        // Direct stick feedforward on all axis
        pidData[axis].F = pidCoefficient[axis].Kf * currentPidSetpoint;

        // Calculate tail feedforward precompensation and add it to the yaw pidSum
        if (axis == FD_YAW) {

            // Get absolute value of collective stick throw
            float collectiveDeflection = getCollectiveDeflection();

            // Collective pitch impulse feed-forward for the main motor
            collectiveDeflectionLPF += (collectiveDeflection - collectiveDeflectionLPF) * collectiveImpulseFilterGain;
            collectiveDeflectionHPF = collectiveDeflection - collectiveDeflectionLPF;

            // Feedforward collective components
            float tailCollectiveFF = collectiveDeflection * tailCollectiveFFGain;
            float tailCollectiveImpulseFF = collectiveDeflectionHPF * tailCollectiveImpulseFFGain;

            // Feedforward cyclic component
            float tailCyclicFF = getCyclicDeflection() * tailCyclicFFGain;

            // Calculate total tail feedforward
            float tailTotalFF = tailCollectiveFF + tailCollectiveImpulseFF + tailCyclicFF;

            // Main rotor direction changes the feedforward sign
            if (motorConfig()->mainRotorDir == DIR_CW)
                pidData[FD_YAW].F -= tailTotalFF;
            else
                pidData[FD_YAW].F += tailTotalFF;
        }

        // calculating the PID sum
        pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
    }

    // Disable PID control if gyro overflow detected
    // This may look very inefficient, but it is done on purpose to always show real CPU usage as in flight
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
    return previousPidSetpoint[axis];
}

float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}

