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

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_rates.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rescue.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/governor.h"
#include "flight/rpm_filter.h"

#include "pid.h"

static FAST_DATA_ZERO_INIT pid_t pid;


float pidGetDT()
{
    return pid.dT;
}

float pidGetPidFrequency()
{
    return pid.freq;
}

float pidGetSetpoint(int axis)
{
    return pid.data[axis].setPoint;
}

float pidGetOutput(int axis)
{
    return pid.data[axis].pidSum;
}

float pidGetCollective(void)
{
    return pid.collective;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void INIT_CODE pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));
}

void INIT_CODE pidResetAxisError(int axis)
{
    pid.data[axis].I = 0;
    pid.data[axis].axisError = 0;
}

void INIT_CODE pidResetAxisErrors(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pid.data[axis].I = 0;
        pid.data[axis].axisError = 0;
    }
}


static void INIT_CODE pidSetLooptime(uint32_t pidLooptime)
{
    pid.dT = pidLooptime * 1e-6f;
    pid.freq = 1.0f / pid.dT;

#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);
    pidInitProfile(pidProfile);
}

void INIT_CODE pidInitProfile(const pidProfile_t *pidProfile)
{
    pid.pidMode = pidProfile->pid_mode;

    // Roll axis
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;

    // Yaw alt. axis
    pid.coef[PID_WAY].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_WAY].P;
    pid.coef[PID_WAY].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_WAY].I;
    pid.coef[PID_WAY].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_WAY].D;
    pid.coef[PID_WAY].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_WAY].F;

    // Accumulated error limit
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = constrain(pidProfile->error_limit[i], 0, 360);

    // Error decay speed when not flying
    pid.errorDecay = 1.0f - ((pidProfile->error_decay) ? (10 * pid.dT / pidProfile->error_decay) : 0);

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&pid.gyrorFilter[i], pt1FilterGain(constrain(pidProfile->gyro_cutoff[i], 1, 250), pid.dT));
        pt1FilterInit(&pid.errorFilter[i], pt1FilterGain(constrain(pidProfile->error_cutoff[i], 1, 250), pid.dT));
        pt1FilterInit(&pid.dtermFilter[i], pt1FilterGain(constrain(pidProfile->dterm_cutoff[i], 1, 250), pid.dT));
        pt1FilterInit(&pid.ftermFilter[i], pt1FilterGain(constrain(pidProfile->fterm_cutoff[i], 1, 250), pid.dT));
    }

    // Error relax
    pid.itermRelaxType = pidProfile->iterm_relax_type;
    if (pid.itermRelaxType) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            uint8_t freq = constrain(pidProfile->iterm_relax_cutoff[i], 1, 100);
            pt1FilterInit(&pid.relaxFilter[i], pt1FilterGain(freq, pid.dT));
            pid.itermRelaxLevel[i] = constrain(pidProfile->iterm_relax_level[i], 10, 250);
        }
    }

    // Collective impulse high-pass filter
    pid.precomp.collectiveImpulseFilterGain = pt1FilterGain(pidProfile->yaw_collective_ff_impulse_freq / 100.0f, pid.dT);

    // Tail/yaw PID parameters
    pid.yawCWStopGain = pidProfile->yaw_cw_stop_gain / 100.0f;
    pid.yawCCWStopGain = pidProfile->yaw_ccw_stop_gain / 100.0f;

    // Tail/yaw precomp
    pid.precomp.yawCyclicFFGain = pidProfile->yaw_cyclic_ff_gain / 500.0f;
    pid.precomp.yawCollectiveFFGain = pidProfile->yaw_collective_ff_gain / 500.0f;
    pid.precomp.yawCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain / 500.0f;

    // Pitch precomp
    pid.precomp.pitchCollectiveFFGain = pidProfile->pitch_collective_ff_gain / 500.0f;
    pid.precomp.pitchCollectiveImpulseFFGain = pidProfile->pitch_collective_ff_impulse_gain / 500.0f;

    // Initialise sub-profiles
    governorInitProfile(pidProfile);
#ifdef USE_ACC
    levelingInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif
    rescueInitProfile(pidProfile);
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
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
 * In this use case, using two or three terms gives nearly 24bits of
 * resolution, which is what can be stored in a float.
 */

static inline void rotateAxisError(void)
{
    if (pid.errorRotation) {
        const float x = pid.data[PID_ROLL].axisError;
        const float y = pid.data[PID_PITCH].axisError;
        const float r = gyro.gyroADCf[Z] * RAD * pid.dT;

        const float a = r * r / 2;
        const float c = 1 - a + (a * a / 6);
        const float s = r * (1 - a / 3);

        pid.data[PID_ROLL].axisError  = x * c + y * s;
        pid.data[PID_PITCH].axisError = y * c - x * s;
    }
}


static float applyItermRelax(int axis, float itermError, float gyroRate, float setpoint)
{
    if ((pid.itermRelaxType == ITERM_RELAX_RPY) ||
        (pid.itermRelaxType == ITERM_RELAX_RP && axis == PID_ROLL) ||
        (pid.itermRelaxType == ITERM_RELAX_RP && axis == PID_PITCH))
    {
        const float setpointLpf = pt1FilterApply(&pid.relaxFilter[axis], setpoint);
        const float setpointHpf = setpoint - setpointLpf;

        const float itermRelaxFactor = MAX(0, 1.0f - fabsf(setpointHpf) / pid.itermRelaxLevel[axis]);

        itermError *= itermRelaxFactor;

        DEBUG_AXIS(ITERM_RELAX, axis, 0, setpoint * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 1, gyroRate * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 2, setpointLpf * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 3, setpointHpf * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 4, itermRelaxFactor * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 5, itermError * 1000);
    }

    return itermError;
}


static inline float pidApplySetpoint(const pidProfile_t *pidProfile, uint8_t axis)
{
    UNUSED(pidProfile);

    // Rate setpoint
    float setpoint = getSetpoint(axis);

#ifdef USE_ACC
    // Apply leveling modes
    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        setpoint = angleModeApply(axis, setpoint);
    }
    else if (FLIGHT_MODE(HORIZON_MODE)) {
        setpoint = horizonModeApply(axis, setpoint);
    }
#ifdef USE_ACRO_TRAINER
    else if (FLIGHT_MODE(TRAINER_MODE)) {
        setpoint = acroTrainerApply(axis, setpoint);
    }
#endif
    // Apply rescue
    setpoint = rescueApply(axis, setpoint);
#endif

    // Save setpoint
    pid.data[axis].setPoint = setpoint;

    return setpoint;
}

static inline void pidApplyCollective(void)
{
    float collective = getSetpoint(FD_COLL);

    // Apply rescue (override)
    collective = rescueApply(FD_COLL, collective);

    pid.collective = collective / 1000;
}

static void pidApplyPrecomp(void)
{
    // Yaw precompensation direction
    const int rotSign = mixerRotationSign();

    // Get stick throws (from previous cycle)
    const float cyclicDeflection = getCyclicDeflection();
    const float collectiveDeflection = getCollectiveDeflection();

    // Collective pitch impulse filter - TODO replace with proper filter
    pid.precomp.collectiveDeflectionLPF += (collectiveDeflection - pid.precomp.collectiveDeflectionLPF) * pid.precomp.collectiveImpulseFilterGain;
    const float collectiveDeflectionHPF = collectiveDeflection - pid.precomp.collectiveDeflectionLPF;

    // Pitch precomp
    const float pitchCollectiveFF = collectiveDeflection * pid.precomp.pitchCollectiveFFGain;
    const float pitchCollectiveImpulseFF = collectiveDeflectionHPF * pid.precomp.pitchCollectiveImpulseFFGain;

    // Total pitch precomp
    const float pitchPrecomp = pitchCollectiveFF + pitchCollectiveImpulseFF;

    // Add to PITCH feedforward
    pid.data[FD_PITCH].F += pitchPrecomp;
    pid.data[FD_PITCH].pidSum += pitchPrecomp;

    DEBUG(PITCH_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(PITCH_PRECOMP, 1, collectiveDeflectionHPF * 1000);
    DEBUG(PITCH_PRECOMP, 2, pitchCollectiveFF * 10);
    DEBUG(PITCH_PRECOMP, 3, pitchCollectiveImpulseFF * 10);
    DEBUG(PITCH_PRECOMP, 4, pitchPrecomp * 10);

    // Collective components
    const float yawCollectiveFF = fabsf(collectiveDeflection) * pid.precomp.yawCollectiveFFGain;
    const float yawCollectiveImpulseFF = fabsf(collectiveDeflectionHPF) * pid.precomp.yawCollectiveImpulseFFGain;

    // Cyclic component
    float yawCyclicFF = fabsf(cyclicDeflection) * pid.precomp.yawCyclicFFGain;

    // Calculate total precompensation
    float yawPrecomp = (yawCollectiveFF + yawCollectiveImpulseFF + yawCyclicFF) * rotSign;

    // Add to YAW feedforward
    pid.data[FD_YAW].F += yawPrecomp;
    pid.data[FD_YAW].pidSum += yawPrecomp;

    DEBUG(YAW_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(YAW_PRECOMP, 1, collectiveDeflectionHPF * 1000);
    DEBUG(YAW_PRECOMP, 2, yawCollectiveFF * 10);
    DEBUG(YAW_PRECOMP, 3, yawCollectiveImpulseFF * 10);
    DEBUG(YAW_PRECOMP, 4, cyclicDeflection * 1000);
    DEBUG(YAW_PRECOMP, 5, yawCyclicFF * 10);
    DEBUG(YAW_PRECOMP, 6, yawPrecomp * 10);
}

/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 0 - PASSTHROUGH
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyMode0(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

  //// Unused term
    pid.data[axis].P = 0;
    pid.data[axis].I = 0;
    pid.data[axis].D = 0;

  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;

  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 1 - SAME AS RF1
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro dterm ADC => Kd => D-term
 **   gyro ADC => Relax => Ki => I-term
 **
 **   -- Using gyro-only D-term
 **   -- Yaw stop gain on P and D
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode1(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term
    float pTerm = errorRate;

    // Limit P bandwidth
    if (pidProfile->error_cutoff[axis]) {
        pTerm = pt1FilterApply(&pid.errorFilter[axis], pTerm);
    }

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm;


  //// D-term

    // Calculate derivative
    float dError = -gyro.gyroDtermADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        dTerm = 0;
    }

    // Calculate D-term
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static void pidApplyYawMode1(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term
    float pTerm = errorRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        pTerm = pt1FilterApply(&pid.errorFilter[axis], pTerm);
    }

    // Select stop gain
    float stopGain = (pTerm > 0) ? pid.yawCWStopGain : pid.yawCCWStopGain;

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm * stopGain;


  //// D-term

    // Calculate D-term from filtered gyro signal
    float dError = -gyro.gyroDtermADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // No D if axis saturated
    if (pidAxisSaturated(axis)) {
        dTerm = 0;
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm * stopGain;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 2
 **
 **   gyroFilter => errorFilter => Kp => P-term
 **   gyroFilter => errorFilter => dtermFilter => Kd => D-term
 **   gyroFilter => errorFilter => Ki => I-term
 **
 **   -- Yaw Stop gain on P only
 **   -- D-term selectable D(error) or D(gyro)
 **   -- Gyro D-term filters not used
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode2(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];
    if (pidProfile->gyro_cutoff[axis]) {
        gyroRate = pt1FilterApply(&pid.gyrorFilter[axis], gyroRate);
    }

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pidProfile->dterm_mode ? errorRate : -gyro.gyroADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // Filter D-term
    if (pidProfile->dterm_cutoff[axis]) {
        dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm);
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = saturation ? 0 : errorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static void pidApplyYawMode2(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];
    if (pidProfile->gyro_cutoff[axis]) {
        gyroRate = pt1FilterApply(&pid.gyrorFilter[axis], gyroRate);
    }

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = transition(errorRate, -10, 10, pid.yawCCWStopGain, pid.yawCWStopGain);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate * stopGain;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pidProfile->dterm_mode ? errorRate : -gyro.gyroADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // Filter D-term
    if (pidProfile->dterm_cutoff[axis]) {
       dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm);
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = saturation ? 0 : errorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 9 - PID TEST MODE
 **
 **   gyroFilter => errorFilter => Kp => P-term
 **   gyroFilter => errorFilter => dtermFilter => Kd => D-term
 **   gyroFilter => errorFilter => Ki => I-term
 **
 **   -- Separate PID gains for yaw CW & CCW
 **   -- D-term selectable D(error) or D(gyro)
 **   -- Yaw D-gain selectable on P-term or D-term sign
 **   -- Gyro D-term filters not used
 **   -- Yaw stop gains not used
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode9(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];
    if (pidProfile->gyro_cutoff[axis]) {
        gyroRate = pt1FilterApply(&pid.gyrorFilter[axis], gyroRate);
    }

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pidProfile->dterm_mode ? errorRate : -gyro.gyroADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // Filter D-term
    if (pidProfile->dterm_cutoff[axis]) {
        dTerm = pt1FilterApply(&pid.dtermFilter[axis],  dTerm);
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = saturation ? 0 : errorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], fTerm);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static void pidApplyYawMode9(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];
    if (pidProfile->gyro_cutoff[axis]) {
        gyroRate = pt1FilterApply(&pid.gyrorFilter[axis], gyroRate);
    }

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Select P-gain
    const float Kp = transition(errorRate, -10, 10, pid.coef[PID_WAY].Kp, pid.coef[PID_YAW].Kp);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pidProfile->dterm_mode ? errorRate : -gyro.gyroADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // Select D-gain
    float Kd = 0;
    switch (pidProfile->yaw_d_select) {
        case 2:
            Kd = transition(dTerm, -100, 100, pid.coef[PID_YAW].Kd, pid.coef[PID_WAY].Kd);
            break;
        case 1:
            Kd = transition(gyro.gyroADCf[axis], -10, 10, pid.coef[PID_YAW].Kd, pid.coef[PID_WAY].Kd);
            break;
        default:
            Kd = transition(errorRate, -10, 10, pid.coef[PID_WAY].Kd, pid.coef[PID_YAW].Kd);
            break;
    }

    // Filter D-term
    if (pidProfile->dterm_cutoff[axis]) {
       dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm);
    }

    // Calculate D-term
    pid.data[axis].D = Kd * dTerm;


  //// I-term

    // Apply error relax
    errorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = saturation ? 0 : errorRate * pid.dT;

    // Select I-gain
    const float Ki = transition(errorRate, -10, 10, pid.coef[PID_WAY].Ki, pid.coef[PID_YAW].Ki);

    // Calculate I-component
    if (pid.data[axis].axisError + itermDelta > pid.errorLimit[axis]) {
        pid.data[axis].axisError = pid.errorLimit[axis];
    }
    else if (pid.data[axis].axisError + itermDelta < -pid.errorLimit[axis]) {
        pid.data[axis].axisError = -pid.errorLimit[axis];
    }
    else {
        pid.data[axis].axisError += itermDelta;
        pid.data[axis].I += Ki * itermDelta;
    }

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].I *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], fTerm);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Calculate stabilized collective
    pidApplyCollective();

    // Apply PID for each axis
    switch (pid.pidMode) {
        case 9:
            pidApplyCyclicMode9(pidProfile, PID_ROLL);
            pidApplyCyclicMode9(pidProfile, PID_PITCH);
            pidApplyYawMode9(pidProfile);
            break;
        case 2:
            pidApplyCyclicMode2(pidProfile, PID_ROLL);
            pidApplyCyclicMode2(pidProfile, PID_PITCH);
            pidApplyYawMode2(pidProfile);
            break;
        case 1:
            pidApplyCyclicMode1(pidProfile, PID_ROLL);
            pidApplyCyclicMode1(pidProfile, PID_PITCH);
            pidApplyYawMode1(pidProfile);
            break;
        default:
            pidApplyMode0(pidProfile, PID_ROLL);
            pidApplyMode0(pidProfile, PID_PITCH);
            pidApplyMode0(pidProfile, PID_YAW);
            break;
    }

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp();

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
