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
            [PID_ROLL] =  { 10, 50,  0, 50 },
            [PID_PITCH] = { 10, 50,  0, 50 },
            [PID_YAW] =   { 50, 50,  0,  0 },
        },
        .debug_axis = FD_ROLL,
        .angle_level_strength = 50,
        .angle_level_limit = 55,
        .horizon_level_strength = 50,
        .horizon_transition = 75,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .iterm_limit = { 400, 400, 400 },
        .iterm_decay = 25,
        .iterm_rotation = true,
        .iterm_relax = ITERM_RELAX_RPY,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .iterm_relax_cutoff = { 10, 10, 10 },
        .acro_trainer_gain = 75,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .abs_control = false,
        .abs_control_gain = 10,
        .abs_control_limit = 120,
        .abs_control_error_limit = 45,
        .abs_control_cutoff = 6,
        .ff_interpolate_sp = 0,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .yaw_center_offset = 0,
        .yaw_cw_stop_gain = 100,
        .yaw_ccw_stop_gain = 100,
        .yaw_cyclic_ff_gain = 50,
        .yaw_collective_ff_gain = 100,
        .yaw_collective_ff_impulse_gain = 20,
        .yaw_collective_ff_impulse_freq = 100,
        .cyclic_normalization = NORM_ABSOLUTE,
        .collective_normalization = NORM_NATURAL,
        .normalization_min_ratio = 50,
        .rescue_collective = 0,
        .rescue_boost = 0,
        .rescue_delay = 35,
        .gov_headspeed = 1000,
        .gov_gain = 50,
        .gov_p_gain = 40,
        .gov_i_gain = 50,
        .gov_d_gain = 0,
        .gov_f_gain = 15,
        .gov_tta_gain = 0,
        .gov_tta_limit = 0,
        .gov_cyclic_ff_weight = 40,
        .gov_collective_ff_weight = 100,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}



static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;
static FAST_RAM_ZERO_INIT uint32_t pidLooptime;

static FAST_RAM_ZERO_INIT uint8_t pidDebugAxis;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT pt1Filter_t errorFilter[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float tailCWStopGain;
static FAST_RAM_ZERO_INIT float tailCCWStopGain;

static FAST_RAM_ZERO_INIT float tailCenterOffset;

static FAST_RAM_ZERO_INIT float tailCyclicFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveImpulseFFGain;

static FAST_RAM_ZERO_INIT float collectiveDeflectionLPF;
static FAST_RAM_ZERO_INIT float collectiveDeflectionHPF;
static FAST_RAM_ZERO_INIT float collectiveImpulseFilterGain;

static FAST_RAM_ZERO_INIT float collectiveCommand;

#ifdef USE_ITERM_RELAX
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static FAST_RAM_ZERO_INIT pt1Filter_t itermRelaxLpf[XYZ_AXIS_COUNT];
#endif

#ifdef USE_ABSOLUTE_CONTROL
static FAST_RAM_ZERO_INIT bool  absoluteControl;
static FAST_RAM_ZERO_INIT pt1Filter_t acFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acGain;
#endif

static FAST_RAM_ZERO_INIT float itermLimit[XYZ_AXIS_COUNT];

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

bool pidAxisDebug(int axis)
{
    return (axis == pidDebugAxis);
}

float getPidSum(int axis)
{
    return pidData[axis].Sum / PIDSUM_SCALING;
}

const pidAxisData_t * getPidData(int axis)
{
    return &pidData[axis];
}

float pidGetStabilizedCollective(void)
{
    return collectiveCommand;
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        if (pidProfile->error_filter_hz[i]) {
            pt1FilterInit(&errorFilter[i], pt1FilterGain(pidProfile->error_filter_hz[i], dT));
        }
    }

#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            uint8_t freq = constrain(pidProfile->iterm_relax_cutoff[i], 1, 50);
            pt1FilterInit(&itermRelaxLpf[i], pt1FilterGain(freq, dT));
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

void pidInitProfile(const pidProfile_t *pidProfile)
{
    pidDebugAxis = pidProfile->debug_axis;

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

    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        itermLimit[i] = constrain(pidProfile->iterm_limit[i], 0, 1000);

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
    interpolatedSpInit(pidProfile);
#endif

    // Collective impulse high-pass filter
    collectiveImpulseFilterGain = pt1FilterGain(pidProfile->yaw_collective_ff_impulse_freq / 100.0f, dT);

    // Tail/yaw parameters
    tailCWStopGain = pidProfile->yaw_cw_stop_gain / 100.0f;
    tailCCWStopGain = pidProfile->yaw_ccw_stop_gain / 100.0f;
    tailCenterOffset = pidProfile->yaw_center_offset / 1000.0f;
    tailCyclicFFGain = pidProfile->yaw_cyclic_ff_gain;
    tailCollectiveFFGain = pidProfile->yaw_collective_ff_gain;
    tailCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain;

    // Governor profile
    governorInitProfile(pidProfile);

    // PID filters
    pidInitFilters(pidProfile);
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);

    pidInitProfile(pidProfile);
}

static void pidReset(void)
{
    memset(pidData, 0, sizeof(pidData));
}

void pidResetError(int axis)
{
    pidData[axis].I = 0;
#ifdef USE_ABSOLUTE_CONTROL
    acError[axis] = 0;
#endif
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
        rotateVector(&pidData[X].Ierror, &pidData[Y].Ierror, gyro.gyroADCf[Z]*dT*RAD);
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

    if (pidAxisDebug(axis)) {
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
    const float setpointLpf = pt1FilterApply(&itermRelaxLpf[axis], currentPidSetpoint);
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

        if (pidAxisDebug(axis)) {
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


static FAST_CODE void pidApplyYawPrecomp(void)
{
    // Yaw precompensation direction
    float rotSign = mixerRotationSign();

    // Get stick throws (from previous cycle)
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
    float tailPrecomp = (tailCollectiveFF + tailCollectiveImpulseFF + tailCyclicFF + tailCenterOffset) * rotSign;

    // Add to YAW feedforward
    pidData[FD_YAW].F   += tailPrecomp;
    pidData[FD_YAW].Sum += tailPrecomp;

    DEBUG_SET(DEBUG_YAW_PRECOMP, 0, lrintf(tailCyclicFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 1, lrintf(tailCollectiveFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 2, lrintf(tailCollectiveImpulseFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 3, lrintf(tailPrecomp));

    DEBUG32_SET(DEBUG_YAW_PRECOMP, 0, cyclicDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 1, tailCyclicFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 2, collectiveDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 3, tailCollectiveFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 4, collectiveDeflectionHPF * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 5, tailCollectiveImpulseFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 6, tailPrecomp * 10);
}

static FAST_CODE void pidApplyCollective(void)
{
    if (FLIGHT_MODE(RESCUE_MODE))
        collectiveCommand = pidRescueCollective();
    else
        collectiveCommand = rcCommand[COLLECTIVE] * MIXER_RC_SCALING;
}

static FAST_CODE void pidApplyAxis(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float pidSetpoint = getSetpointRate(axis);

#ifdef USE_ACC
    // Apply leveling
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        pidSetpoint = pidLevelApply(axis, pidSetpoint);
    }
#ifdef USE_ACRO_TRAINER
    else {
        // Apply trainer
        pidSetpoint = acroTrainerApply(axis, pidSetpoint);
    }
#endif
#endif

    // Get gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate for I-term
    float itermErrorRate = pidSetpoint - gyroRate;

    // Calculate rate and delta for D-term
    float dtermErrorRate = gyro.gyroDtermADCf[axis];
    float dtermDelta = (pidData[axis].Derror - dtermErrorRate) * pidFrequency;

    // Apply I-term decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        pidData[axis].Ierror -= pidData[axis].Ierror * itermDecay;
#ifdef USE_ABSOLUTE_CONTROL
        acError[axis] -= acError[axis] * itermDecay;
#endif
    }
#endif

    // Apply I-term relax & Absolute Control
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        itermErrorRate = applyItermRelax(axis, pidData[axis].Ierror, itermErrorRate, gyroRate, pidSetpoint);
#ifdef USE_ABSOLUTE_CONTROL
        if (absoluteControl) {
            float delta = applyAbsoluteControl(axis, gyroRate, pidSetpoint);
            itermErrorRate += delta;
            pidSetpoint += delta;
        }
#endif
    }
#endif

    // Save data
    pidData[axis].Setpoint = pidSetpoint;
    pidData[axis].GyroRate = gyroRate;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermErrorRate = 0;
        dtermDelta = 0;
    }

    // Calculate I-term error change after modifications
    float itermDelta = itermErrorRate * dT;

    // Calculate I-component
    pidData[axis].Ierror = constrainf(pidData[axis].Ierror + itermDelta, -itermLimit[axis], itermLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * pidData[axis].Ierror;

    // Calculate P-term error rate after modifications
    float ptermErrorRate = pidSetpoint - gyroRate;

    // Extra error filtering
    if (pidProfile->error_filter_hz[axis])
        ptermErrorRate = pt1FilterApply(&errorFilter[axis], ptermErrorRate);

    // Extra stop gain
    float Ks;
    if (axis == FD_YAW)
        Ks = (ptermErrorRate > 0) ? tailCWStopGain : tailCCWStopGain;
    else
        Ks = 1.0f;

    // Calculate P-component
    pidData[axis].Perror = ptermErrorRate;
    pidData[axis].P = Ks * pidCoefficient[axis].Kp * ptermErrorRate;

    // Calculate D-component
    pidData[axis].Derror = dtermErrorRate;
    pidData[axis].D = Ks * pidCoefficient[axis].Kd * dtermDelta;

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[axis].Kf * pidSetpoint;

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
}


FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Rotate error around yaw axis
#ifdef USE_ITERM_ROTATION
    rotateIterm();
#endif
#ifdef USE_ABSOLUTE_CONTROL
    rotateAxisError();
#endif

    // Update rescue mode state
    pidRescueUpdate();

    // Apply PID for each axis
    pidApplyAxis(pidProfile, FD_ROLL);
    pidApplyAxis(pidProfile, FD_PITCH);
    pidApplyAxis(pidProfile, FD_YAW);

    // Calculate cyclic/collective precompensation for tail
    pidApplyYawPrecomp();

    // Calculate stabilized collective
    pidApplyCollective();

    // Reset error if in passthrough mode
    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        pidResetError(FD_ROLL);
        pidResetError(FD_PITCH);
    }

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}

