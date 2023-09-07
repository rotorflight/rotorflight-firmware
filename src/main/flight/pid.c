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
    // PID algorithm
    pid.pidMode = pidProfile->pid_mode;

    // Roll axis
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * pidProfile->pid[PID_ROLL].B;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * pidProfile->pid[PID_PITCH].B;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * pidProfile->pid[PID_YAW].B;

    // Accumulated error limit
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = pidProfile->error_limit[i];

    // Exponential error decay rates
    pid.errorDecayRateGround = (pidProfile->error_decay_time_ground) ? (10 * pid.dT / pidProfile->error_decay_time_ground) : 0;
    pid.errorDecayRateCyclic = (pidProfile->error_decay_time_cyclic) ? (10 * pid.dT / pidProfile->error_decay_time_cyclic) : 0;
    pid.errorDecayRateYaw    = (pidProfile->error_decay_time_yaw)    ? (10 * pid.dT / pidProfile->error_decay_time_yaw) : 0;

    // Max decay speeds in degs/s (linear decay)
    pid.errorDecayLimitCyclic = (pidProfile->error_decay_limit_cyclic) ? (pid.dT * pidProfile->error_decay_limit_cyclic) : 1e6;
    pid.errorDecayLimitYaw    = (pidProfile->error_decay_limit_yaw)    ? (pid.dT * pidProfile->error_decay_limit_yaw) : 1e6;

    // Error Rotation enable
    pid.errorRotation = pidProfile->error_rotation;

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        lowpassFilterInit(&pid.gyrorFilter[i], pidProfile->gyro_filter_type, pidProfile->gyro_cutoff[i], pid.freq, 0);
        lowpassFilterInit(&pid.errorFilter[i], LPF_ORDER1, pidProfile->error_cutoff[i], pid.freq, 0);
        difFilterInit(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterInit(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
    }

    // Error relax
    pid.itermRelaxType = pidProfile->iterm_relax_type;
    if (pid.itermRelaxType) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            uint8_t freq = constrain(pidProfile->iterm_relax_cutoff[i], 1, 100);
            pt1FilterInit(&pid.relaxFilter[i], freq, pid.freq);
            pid.itermRelaxLevel[i] = constrain(pidProfile->iterm_relax_level[i], 10, 250);
        }
    }

    // D-term calculation
    pid.dtermMode = pidProfile->dterm_mode;
    pid.dtermModeYaw = pidProfile->dterm_mode_yaw;

    // Tail/yaw PID parameters
    pid.yawCWStopGain = pidProfile->yaw_cw_stop_gain / 100.0f;
    pid.yawCCWStopGain = pidProfile->yaw_ccw_stop_gain / 100.0f;

    // Collective dynamic filter
    pt1FilterInit(&pid.precomp.collFilter, 100.0f / constrainf(pidProfile->yaw_collective_dynamic_decay, 1, 250), pid.freq);

    // Tail/yaw precomp
    pid.precomp.yawCyclicFFGain = pidProfile->yaw_cyclic_ff_gain / 100.0f;
    pid.precomp.yawCollectiveFFGain = pidProfile->yaw_collective_ff_gain / 100.0f;
    pid.precomp.yawCollectiveDynamicGain = pidProfile->yaw_collective_dynamic_gain / 100.0f;

    // Pitch precomp
    pid.precomp.pitchCollectiveFFGain = pidProfile->pitch_collective_ff_gain / 500.0f;

    // Cross-coupling compensation
    pid.cyclicCrossCouplingGain[FD_PITCH] = pidProfile->cyclic_cross_coupling_gain * mixerRotationSign() * -CROSS_COUPLING_SCALE;
    pid.cyclicCrossCouplingGain[FD_ROLL]  = pid.cyclicCrossCouplingGain[FD_PITCH] * pidProfile->cyclic_cross_coupling_ratio / -100.0f;

    // Cross-coupling derivative filters
    difFilterInit(&pid.crossCouplingFilter[FD_PITCH], pidProfile->cyclic_cross_coupling_cutoff, pid.freq);
    difFilterInit(&pid.crossCouplingFilter[FD_ROLL], pidProfile->cyclic_cross_coupling_cutoff, pid.freq);

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
 *        | cos(r)   -sin(r) |
 *    R = |                  |
 *        | sin(r)    cos(r) |
 *
 *
 *               x³    x⁵    x⁷    x⁹
 * sin(x) = x - ――― + ――― - ――― + ――― - …
 *               3!    5!    7!    9!
 *
 *
 *               x²    x⁴    x⁶    x⁸
 * cos(x) = 1 - ――― + ――― - ――― + ――― - …
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

        const float t = r * r / 2;
        const float C = t * (1 - t / 6);
        const float S = r * (1 - t / 3);

        pid.data[PID_ROLL].axisError  -= x * C - y * S;
        pid.data[PID_PITCH].axisError -= y * C + x * S;
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


static float pidApplySetpoint(uint8_t axis)
{
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

static float pidApplyGyroRate(uint8_t axis)
{
    // Get gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Bandwidth limiter
    gyroRate = filterApply(&pid.gyrorFilter[axis], gyroRate);

    // Save current rate
    pid.data[axis].gyroRate = gyroRate;

    return gyroRate;
}

static void pidApplyCollective(void)
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

    // Get actual control deflections (from previous cycle)
    const float cyclicDeflection = getCyclicDeflection();
    const float collectiveDeflection = getCollectiveDeflection();

    // Collective High Pass Filter (this is possible with PT1)
    const float collectiveLF = pt1FilterApply(&pid.precomp.collFilter, collectiveDeflection);
    const float collectiveHF = collectiveDeflection - collectiveLF;

  //// Collective-to-Yaw Precomp

    // Collective components
    const float yawCollectiveFF = fabsf(collectiveDeflection) * pid.precomp.yawCollectiveFFGain;
    const float yawCollectiveHF = fabsf(collectiveHF) * pid.precomp.yawCollectiveDynamicGain;

    // Cyclic component
    float yawCyclicFF = fabsf(cyclicDeflection) * pid.precomp.yawCyclicFFGain;

    // Calculate total precompensation
    float yawPrecomp = (yawCollectiveFF + yawCollectiveHF + yawCyclicFF) * rotSign;

    // Add to YAW feedforward
    pid.data[FD_YAW].F += yawPrecomp;
    pid.data[FD_YAW].pidSum += yawPrecomp;

    DEBUG(YAW_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(YAW_PRECOMP, 1, collectiveLF * 1000);
    DEBUG(YAW_PRECOMP, 2, collectiveHF * 1000);
    DEBUG(YAW_PRECOMP, 3, cyclicDeflection * 1000);
    DEBUG(YAW_PRECOMP, 4, yawCollectiveFF * 1000);
    DEBUG(YAW_PRECOMP, 5, yawCollectiveHF * 1000);
    DEBUG(YAW_PRECOMP, 6, yawCyclicFF * 1000);
    DEBUG(YAW_PRECOMP, 7, yawPrecomp * 1000);


  //// Collective-to-Pitch precomp

    // Collective component
    const float pitchPrecomp = collectiveDeflection * pid.precomp.pitchCollectiveFFGain;

    // Add to PITCH feedforward
    pid.data[FD_PITCH].F += pitchPrecomp;
    pid.data[FD_PITCH].pidSum += pitchPrecomp;

    DEBUG(PITCH_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(PITCH_PRECOMP, 1, pitchPrecomp * 1000);
}

static void pidApplyCyclicCrossCoupling(void)
{
    // Setpoint derivative filter
    const float pitchDeriv = difFilterApply(&pid.crossCouplingFilter[FD_PITCH], pid.data[FD_PITCH].setPoint);
    const float rollDeriv  = difFilterApply(&pid.crossCouplingFilter[FD_ROLL], pid.data[FD_ROLL].setPoint);
    const float pitchComp  = rollDeriv * pid.cyclicCrossCouplingGain[FD_ROLL];
    const float rollComp   = pitchDeriv * pid.cyclicCrossCouplingGain[FD_PITCH];

    // Add de-coupling terms
    pid.data[FD_ROLL].pidSum += rollComp;
    pid.data[FD_PITCH].pidSum += pitchComp;

    DEBUG(CROSS_COUPLING, 0, rollDeriv);
    DEBUG(CROSS_COUPLING, 1, pitchDeriv);
    DEBUG(CROSS_COUPLING, 2, rollComp * 1000);
    DEBUG(CROSS_COUPLING, 3, pitchComp * 1000);
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 0 - PASSTHROUGH
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyMode0(uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(axis);

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
 ** MODE 1 - NEARLY THE SAME AS RF1
 **
 **   gyroFilter => errorFilter => Kp => P-term
 **   gyroFilter => difFilter => Kd => D-term
 **   gyroFilter => Relax => Ki => I-term
 **
 **   -- Using gyro-only D-term
 **   -- Yaw stop gain on P only
 **   -- Error filter on P-term only
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode1(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term with extra filtering
    float pTerm = filterApply(&pid.errorFilter[axis], errorRate);

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis))
        dTerm = 0;

    // Calculate D-term
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = itermErrorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isAirborne())
        pid.data[axis].axisError -= pid.data[axis].axisError * pid.errorDecayRateGround;


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static void pidApplyYawMode1(void)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term
    float pTerm = filterApply(&pid.errorFilter[axis], errorRate);

    // Select stop gain
    float stopGain = (pTerm > 0) ? pid.yawCWStopGain : pid.yawCCWStopGain;

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm * stopGain;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // No D if axis saturated
    if (pidAxisSaturated(axis))
        dTerm = 0;

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm * stopGain;


  //// I-term

    // Apply error relax
    float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // I-term change
    float itermDelta = itermErrorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp())
        pid.data[axis].axisError -= pid.data[axis].axisError * pid.errorDecayRateGround;


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 2
 **
 **   gyroFilter => Kp => P-term
 **   gyroFilter => difFilter => Kd => D-term
 **   gyroFilter => Relax => Ki => I-term
 **
 **   -- Yaw Stop gain on P only
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode2(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    const float dError = pid.dtermMode ? errorRate : -gyroRate;
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], dError);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    pid.data[axis].axisError -= isAirborne() ?
      limitf(pid.data[axis].axisError * pid.errorDecayRateCyclic, pid.errorDecayLimitCyclic):
      pid.data[axis].axisError * pid.errorDecayRateGround;


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;

  //// Feedforward Boost (FF Derivative)

    // Calculate B-term with bandwidth limit
    const float bTerm = difFilterApply(&pid.btermFilter[axis], setpoint);

    // Calculate B-component
    pid.data[axis].B = pid.coef[axis].Kb * bTerm;


  //// PID Sum

    // Calculate sum of all terms
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D +
                            pid.data[axis].F + pid.data[axis].B;
}


static void pidApplyYawMode2(void)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;

    // Select stop gain
    const float stopGain = transition(errorRate, -10, 10, pid.yawCCWStopGain, pid.yawCWStopGain);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate * stopGain;


  //// D-term

    // Calculate D-term with bandwidth limit
    const float dError = pid.dtermModeYaw ? errorRate : -gyroRate;
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], dError);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    pid.data[axis].axisError -= isSpooledUp() ?
      limitf(pid.data[axis].axisError * pid.errorDecayRateYaw, pid.errorDecayLimitYaw):
      pid.data[axis].axisError * pid.errorDecayRateGround;


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// Feedforward Boost (FF Derivative)

    // Calculate B-term with bandwidth limit
    const float bTerm = difFilterApply(&pid.btermFilter[axis], setpoint);

    // Calculate B-component
    pid.data[axis].B = pid.coef[axis].Kb * bTerm;


  //// PID Sum

    // Calculate sum of all terms
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D +
                            pid.data[axis].F + pid.data[axis].B;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    // pidProfile can't be used in runtime - pidInitProfile must be called first
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Calculate stabilized collective
    pidApplyCollective();

    // Apply PID for each axis
    switch (pid.pidMode) {
        case 2:
            pidApplyCyclicMode2(PID_ROLL);
            pidApplyCyclicMode2(PID_PITCH);
            pidApplyCyclicCrossCoupling();
            pidApplyYawMode2();
            break;
        case 1:
            pidApplyCyclicMode1(PID_ROLL);
            pidApplyCyclicMode1(PID_PITCH);
            pidApplyYawMode1();
            break;
        default:
            pidApplyMode0(PID_ROLL);
            pidApplyMode0(PID_PITCH);
            pidApplyMode0(PID_YAW);
            break;
    }

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp();

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
