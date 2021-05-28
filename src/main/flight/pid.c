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

#include "config/feature.h"
#include "config/config.h"
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

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/setpoint.h"
#include "flight/gps_rescue.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"


PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = { 0, },
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90 },
            [PID_PITCH] = { 46, 90, 38, 95 },
            [PID_YAW] =   { 45, 90, 0, 90 },
        },
        .angle_level_strength = 50,
        .angle_level_limit = 55,
        .horizon_level_strength = 50,
        .horizon_transition = 75,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .iterm_limit = 400,
        .iterm_decay = 20,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .iterm_relax_cutoff = 10,
        .acro_trainer_gain = 75,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .abs_control = false,
        .abs_control_gain = 10,
        .abs_control_limit = 120,
        .abs_control_error_limit = 45,
        .abs_control_cutoff = 6,
        .ff_interpolate_sp = FF_INTERPOLATE_AVG2,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .yaw_center_offset = 0,
        .yaw_cyclic_ff_gain = 0,
        .yaw_collective_ff_gain = 300,
        .yaw_collective_ff_impulse_gain = 300,
        .yaw_collective_ff_impulse_freq = 100,
        .rate_normalization = RATE_NORM_ABSOLUTE,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}


FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;
static FAST_RAM_ZERO_INIT uint32_t pidLooptime;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousDtermGyroRate[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float tailCenterOffset;
static FAST_RAM_ZERO_INIT float tailCyclicFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveImpulseFFGain;

static FAST_RAM_ZERO_INIT float collectiveDeflectionLPF;
static FAST_RAM_ZERO_INIT float collectiveDeflectionHPF;
static FAST_RAM_ZERO_INIT float collectiveImpulseFilterGain;

#ifdef USE_ITERM_RELAX
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
#endif

#ifdef USE_ABSOLUTE_CONTROL
static FAST_RAM_ZERO_INIT bool  absoluteControl;
static FAST_RAM_ZERO_INIT pt1Filter_t acFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acGain;
#endif

static FAST_RAM_ZERO_INIT uint8_t rateNormalization;

#ifdef USE_INTERPOLATED_SP
static FAST_RAM_ZERO_INIT bool spInterpolation;
#endif

static FAST_RAM_ZERO_INIT float itermLimit;

#ifdef USE_ITERM_DECAY
static FAST_RAM_ZERO_INIT float itermDecay;
#endif

#ifdef USE_ITERM_ROTATION
static FAST_RAM_ZERO_INIT bool itermRotation;
#endif


float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}

uint32_t pidGetLooptime(void)
{
    return pidLooptime;
}

static void pidSetLooptime(uint32_t looptime)
{
    pidLooptime = looptime;
    dT = pidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

float pidGetSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(pidProfile->iterm_relax_cutoff, dT));
        }
#ifdef USE_ABSOLUTE_CONTROL
        if (absoluteControl) {
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pt1FilterInit(&acFilter[i], pt1FilterGain(pidProfile->abs_control_cutoff, dT));
            }
        }
#endif
    }
#endif
}

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

    itermLimit = pidProfile->iterm_limit;

#ifdef USE_ITERM_ROTATION
    itermRotation = pidProfile->iterm_rotation;
#endif
#ifdef USE_ITERM_DECAY
    itermDecay = dT * 10.0f / pidProfile->iterm_decay;
#endif
#ifdef USE_ITERM_RELAX
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
#endif

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif

#ifdef USE_ABSOLUTE_CONTROL
    absoluteControl = pidProfile->abs_control && pidProfile->abs_control_gain > 0;
    acGain = pidProfile->abs_control_gain / 10.0f;
    acLimit = pidProfile->abs_control_limit;
    acErrorLimit = pidProfile->abs_control_error_limit;

    float RC = acGain * pidCoefficient[FD_ROLL].Kp * ROLL_P_TERM_SCALE / ROLL_I_TERM_SCALE;
    float PC = acGain * pidCoefficient[FD_PITCH].Kp * PITCH_P_TERM_SCALE / PITCH_I_TERM_SCALE;
    float YC = acGain * pidCoefficient[FD_YAW].Kp * YAW_P_TERM_SCALE / YAW_I_TERM_SCALE;

    pidCoefficient[FD_ROLL].Ki  = MAX(0, pidCoefficient[FD_ROLL].Ki  - RC);
    pidCoefficient[FD_PITCH].Ki = MAX(0, pidCoefficient[FD_PITCH].Ki - PC);
    pidCoefficient[FD_YAW].Ki   = MAX(0, pidCoefficient[FD_YAW].Ki   - YC);
#endif

#ifdef USE_INTERPOLATED_SP
    spInterpolation = (pidProfile->ff_interpolate_sp != FF_INTERPOLATE_OFF);
    interpolatedSpInit(pidProfile);
#endif

    // Rate normalization for pitch & roll
    rateNormalization = pidProfile->rate_normalization;

    // Collective impulse high-pass filter
    collectiveImpulseFilterGain = dT / (dT + (1 / (2 * M_PIf * pidProfile->yaw_collective_ff_impulse_freq / 100.0f)));

    // Tail feedforward gains
    tailCenterOffset = pidProfile->yaw_center_offset / 1000.0f;
    tailCyclicFFGain = pidProfile->yaw_cyclic_ff_gain;
    tailCollectiveFFGain = pidProfile->yaw_collective_ff_gain;
    tailCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);

    pidInitConfig(pidProfile);
    pidInitFilters(pidProfile);
}

static void pidReset(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 0;
        pidData[axis].Sum = 0;
    }
}

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0;
#ifdef USE_ABSOLUTE_CONTROL
        acError[axis] = 0;
#endif
    }
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

/*
 * 2D Rotation matrix
 *
 *        | cos(r)   -sin r |
 *    R = |                 |
 *        | sin(r)    cos r |
 *
 *
 *                3     5     7     9
 *               x     x     x     x
 * sin(x) = x - --- + --- - --- + --- - ...
 *               3!    5!    7!    9!
 *
 *                2     4     6     8
 *               x     x     x     x
 * cos(x) = 1 - --- + --- - --- + --- - ...
 *               2!    4!    6!    8!
 *
 *
 * For very small values of x, sin(x) ~= x and cos(x) ~= 1.
 *
 * In the use case below, using an additional term gives nearly 24bits of
 * resolution, which is close to what can be stored in a float anyway.
 *
 */

#ifdef __ZERO_ORDER_APPROX__
static inline void rotateVector(float *x, float *y, float r)
{
    float a,b;

    a = x[0] + y[0] * r;
    b = y[0] - x[0] * r;

    x[0] = a;
    y[0] = b;
}
#else

#define SIN2(R) ((R)-(R)*(R)*(R)/6)
#define COS2(R) (1.0f-(R)*(R)/2)

static inline void rotateVector(float *x, float *y, float r)
{
    float a,b,s,c;

    s = SIN2(r);
    c = COS2(r);

    a = x[0]*c + y[0]*s;
    b = y[0]*c - x[0]*s;

    x[0] = a;
    y[0] = b;
}
#endif

#ifdef USE_ITERM_ROTATION
static inline void rotateIterm(void)
{
    if (itermRotation) {
        rotateVector(&pidData[X].I, &pidData[Y].I, gyro.gyroADCf[Z]*dT*RAD);
    }
}
#endif

#ifdef USE_ABSOLUTE_CONTROL
static inline void rotateAxisError(void)
{
    if (itermRelax && absoluteControl) {
        rotateVector(&acError[X], &acError[Y], gyro.gyroADCf[Z]*dT*RAD);
    }
}

static FAST_CODE float applyAbsoluteControl(const int axis,
	const float gyroRate, const float currentPidSetpoint)
{
    const float acLpf = pt1FilterApply(&acFilter[axis], currentPidSetpoint);
    const float acHpf = fabsf(currentPidSetpoint - acLpf);
    const float acGmax = acLpf + 2 * acHpf;
    const float acGmin = acLpf - 2 * acHpf;
    const float acError1 = acGmax - gyroRate;
    const float acError2 = acGmin - gyroRate;

    float acErrorRate = 0, acCorrection = 0;

    if (gyroRate > acGmax) {
        acErrorRate = acError1; // < 0
    }
    else if (gyroRate < acGmin) {
        acErrorRate = acError2; // > 0
    }
    else {
        if (acError[axis] < 0)
            acErrorRate = acError1; // > 0
        else
            acErrorRate = acError2; // < 0
    }

    if (fabsf(acErrorRate * dT) > fabsf(acError[axis]))
        acErrorRate = -acError[axis] * pidFrequency;

    if (pidAxisSaturated(axis) || !isSpooledUp())
        acErrorRate = 0;

    acError[axis] = constrainf(acError[axis] + acErrorRate * dT, -acErrorLimit, acErrorLimit);

    acCorrection = constrainf(acError[axis] * acGain, -acLimit, acLimit);

    DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(acError[axis] * 10));
    DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
        DEBUG32_SET(DEBUG_ITERM_RELAX, 7, acCorrection * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 0, gyroRate * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 1, currentPidSetpoint * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 2, acLpf * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 3, acHpf * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 4, acError1 * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 5, acError2 * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 6, acErrorRate * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 7, acCorrection * 1000);
    }

    return acCorrection;
}
#endif

#ifdef USE_ITERM_RELAX
static FAST_CODE float applyItermRelax(const int axis, const float iterm,
    float itermErrorRate, const float gyroRate, const float currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
    const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);

    // Always active on ROLL & PITCH; active also on YAW if _RPY
    if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC)
    {
        const float itermRelaxFactor = MAX(0, 1.0f - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);

        if ((itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC) &&
            (((iterm > 0) && (itermErrorRate < 0)) || ((iterm < 0) && (itermErrorRate > 0)))) {
            // Iterm decreasing, no change
        }
        else {
            if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            }
        }

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
            DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
            DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(itermErrorRate));
            DEBUG32_SET(DEBUG_ITERM_RELAX, 0, currentPidSetpoint * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 1, gyroRate * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 2, setpointLpf * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 3, setpointHpf * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 4, itermRelaxFactor * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 5, itermErrorRate * 1000);
        }
    }

    return itermErrorRate;
}
#endif

static inline float getPIDNormalizationGain(uint8_t axis, float hsRatio)
{
    if (axis == FD_YAW && mixerMotorizedTail())
        return 1.0f;
    else
        return 1.0f / (hsRatio*hsRatio);
}

static inline float getYawPrecompGain(float hsRatio)
{
    if (mixerMotorizedTail())
        return (hsRatio*hsRatio);
    else
        return 1.0f;
}

static inline float getNormalizedRate(uint8_t axis, float hsRatio)
{
    float ratio;

    // YAW is always absolute rate
    if (axis == FD_YAW) {
        ratio = 1.0f;
    }
    // PITCH and ROLL can be relaxed
    else {
        if (rateNormalization == RATE_NORM_NATURAL)
            ratio = hsRatio * hsRatio;
        else if (rateNormalization == RATE_NORM_LINEAR)
            ratio = hsRatio;
        else
            ratio = 1.0f;
    }

    return ratio * getSetpointRate(axis);
}

static FAST_CODE void applyTailPrecompensation(const pidProfile_t *pidProfile, float hsRatio)
{
    UNUSED(pidProfile);

    // Yaw precompensation gain and direction
    float Kc = getYawPrecompGain(hsRatio) * mixerRotationSign();

    // Get stick throws
    float cyclicDeflection = getCyclicDeflection();
    float collectiveDeflection = getCollectiveDeflection();

    // Collective pitch impulse filter
    collectiveDeflectionLPF += (collectiveDeflection - collectiveDeflectionLPF) * collectiveImpulseFilterGain;
    collectiveDeflectionHPF = collectiveDeflection - collectiveDeflectionLPF;

    // Collective components
    float tailCollectiveFF = fabsf(collectiveDeflection) * tailCollectiveFFGain;
    float tailCollectiveImpulseFF = fabsf(collectiveDeflectionHPF) * tailCollectiveImpulseFFGain;

    // Cyclic component
    float tailCyclicFF = fabsf(cyclicDeflection) * tailCyclicFFGain;

    // Calculate total precompensation
    float tailPrecomp = (tailCollectiveFF + tailCollectiveImpulseFF + tailCyclicFF + tailCenterOffset) * Kc;

    // Add to YAW PID sum
    pidData[FD_YAW].Sum += tailPrecomp;
}

FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

#ifdef USE_ITERM_ROTATION
    rotateIterm();
#endif
#ifdef USE_ABSOLUTE_CONTROL
    rotateAxisError();
#endif

    // -----headspeed ratio with limits
    float hsRatio = constrainf(getHeadSpeedRatio(), 0.7f, 1.2f);

    // -----foreach (ROLL, PITCH, YAW)
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
    {
        // -----normalised Setpoint rate
        float currentPidSetpoint = getNormalizedRate(axis, hsRatio);

        // -----PID normalisation gain
        float Kn = getPIDNormalizationGain(axis, hsRatio);

#ifdef USE_ACC
        // -----apply leveling
        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
            currentPidSetpoint = pidLevelApply(axis, currentPidSetpoint);
        }
#ifdef USE_ACRO_TRAINER
        else {
            // -----apply trainer
            currentPidSetpoint = acroTrainerApply(axis, currentPidSetpoint);
        }
#endif
#endif

        // -----calculate gyro rate
        float gyroRate = gyro.gyroADCf[axis];

        // -----calculate error rate for I-term
        float itermErrorRate = currentPidSetpoint - gyroRate;

#ifdef USE_ITERM_DECAY
        if (!isSpooledUp()) {
            pidData[axis].I -= pidData[axis].I * itermDecay;
#ifdef USE_ABSOLUTE_CONTROL
            acError[axis] -= acError[axis] * itermDecay;
#endif
        }
#endif
#ifdef USE_ITERM_RELAX
        if (itermRelax) {
            itermErrorRate = applyItermRelax(axis, pidData[axis].I, itermErrorRate, gyroRate, currentPidSetpoint);
#ifdef USE_ABSOLUTE_CONTROL
            if (absoluteControl) {
                float delta = applyAbsoluteControl(axis, gyroRate, currentPidSetpoint);
                itermErrorRate += delta;
                currentPidSetpoint += delta;
            }
#endif
        }
#endif
        // -----axis saturated
        if (pidAxisSaturated(axis))
            itermErrorRate = 0;

        // -----calculate I component
        float itermDelta = pidCoefficient[axis].Ki * dT * itermErrorRate;
        pidData[axis].I = constrainf(pidData[axis].I + itermDelta, -itermLimit, itermLimit);

        // -----calculate error rate after currentPidSetpoint modifications
        float errorRate = currentPidSetpoint - gyroRate;

        // -----calculate P component
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate;

#ifdef __NOT_USED__
        // -----calculate setpoint delta
        float pidSetpointDelta = 0;
#ifdef USE_INTERPOLATED_SP
        if (spInterpolation)
            pidSetpointDelta = interpolatedSpApply(axis);
        else
#endif
            pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];

        previousPidSetpoint[axis] = currentPidSetpoint;

#ifdef USE_RC_SMOOTHING_FILTER
        pidSetpointDelta = rcSmoothingApplyDerivativeFilter(axis, pidSetpointDelta);
#endif
#endif

        // -----calculate gyro D component
        if (pidCoefficient[axis].Kd > 0) {
            const float dtermGyroRate = gyro.gyroDtermADCf[axis];
            const float delta = (previousDtermGyroRate[axis] - dtermGyroRate) * pidFrequency;
            pidData[axis].D = pidCoefficient[axis].Kd * delta;
            previousDtermGyroRate[axis] = dtermGyroRate;
        } else {
            pidData[axis].D = 0;
        }

        // -----calculate feedforward component
        pidData[axis].F = pidCoefficient[axis].Kf * currentPidSetpoint;

        // -----calculate the PID sum
        pidData[axis].Sum = (pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F) * Kn;

    }

    // Calculate cyclic/collective precompensation for tail
    applyTailPrecompensation(pidProfile, hsRatio);

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}

