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
    pid.data[axis].O = 0;
    pid.data[axis].axisOffset = 0;
}

void INIT_CODE pidResetAxisErrors(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pid.data[axis].I = 0;
        pid.data[axis].axisError = 0;
        pid.data[axis].O = 0;
        pid.data[axis].axisOffset = 0;
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
    pid.coef[PID_ROLL].Ko = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].O;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * pidProfile->pid[PID_PITCH].B;
    pid.coef[PID_PITCH].Ko = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].O;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * pidProfile->pid[PID_YAW].B;

    // Bleed conversion for pitch
    if (pidProfile->pid[PID_PITCH].O > 0 && pidProfile->pid[PID_PITCH].I > 0)
      pid.coef[PID_PITCH].Kc = pid.coef[PID_PITCH].Ko / pid.coef[PID_PITCH].Ki;
    else
      pid.coef[PID_PITCH].Kc = 0;

    // Bleed conversion for roll
    if (pidProfile->pid[PID_ROLL].O > 0 && pidProfile->pid[PID_ROLL].I > 0)
      pid.coef[PID_ROLL].Kc = pid.coef[PID_ROLL].Ko / pid.coef[PID_ROLL].Ki;
    else
      pid.coef[PID_ROLL].Kc = 0;

    // Accumulated error limit
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = pidProfile->error_limit[i];
    for (int i = 0; i < XY_AXIS_COUNT; i++)
        pid.offsetLimit[i] = pidProfile->offset_limit[i];

    // Exponential error decay rates
    pid.errorDecayRateGround = (pidProfile->error_decay_time_ground) ? (10.0f / pidProfile->error_decay_time_ground) : 0;
    pid.errorDecayRateCyclic = (pidProfile->error_decay_time_cyclic) ? (10.0f / pidProfile->error_decay_time_cyclic) : 0;
    pid.errorDecayRateYaw    = (pidProfile->error_decay_time_yaw)    ? (10.0f / pidProfile->error_decay_time_yaw) : 0;

    // Max decay speeds in degs/s (linear decay)
    pid.errorDecayLimitCyclic = (pidProfile->error_decay_limit_cyclic) ? pidProfile->error_decay_limit_cyclic : 3600;
    pid.errorDecayLimitYaw    = (pidProfile->error_decay_limit_yaw)    ? pidProfile->error_decay_limit_yaw : 3600;

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

    // Collective/cyclic deflection lowpass filters
    lowpassFilterInit(&pid.precomp.collDeflectionFilter, pidProfile->yaw_precomp_filter_type, pidProfile->yaw_precomp_cutoff, pid.freq, 0);
    lowpassFilterInit(&pid.precomp.pitchDeflectionFilter, pidProfile->yaw_precomp_filter_type, pidProfile->yaw_precomp_cutoff, pid.freq, 0);
    lowpassFilterInit(&pid.precomp.rollDeflectionFilter, pidProfile->yaw_precomp_filter_type, pidProfile->yaw_precomp_cutoff, pid.freq, 0);

    // Collective dynamic filter
    pt1FilterInit(&pid.precomp.collDynamicFilter, 100.0f / constrainf(pidProfile->yaw_collective_dynamic_decay, 1, 250), pid.freq);

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
        const float r = gyro.gyroADCf[Z] * RAD * pid.dT;

        const float t = r * r / 2;
        const float C = t * (1 - t / 6);
        const float S = r * (1 - t / 3);

        const float x = pid.data[PID_ROLL].axisError;
        const float y = pid.data[PID_PITCH].axisError;

        pid.data[PID_ROLL].axisError  -= x * C - y * S;
        pid.data[PID_PITCH].axisError -= y * C + x * S;

        const float fx = pid.data[PID_ROLL].axisOffset;
        const float fy = pid.data[PID_PITCH].axisOffset;

        pid.data[PID_ROLL].axisOffset  -= fx * C - fy * S;
        pid.data[PID_PITCH].axisOffset -= fy * C + fx * S;
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
    // Yaw precompensation direction and ratio
    const float masterGain = mixerRotationSign() * getSpoolUpRatio();

    // Get actual control deflections (from previous cycle)
    const float collectiveDeflection = filterApply(&pid.precomp.collDeflectionFilter, mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE));
    const float pitchDeflection = filterApply(&pid.precomp.pitchDeflectionFilter, mixerGetInput(MIXER_IN_STABILIZED_PITCH));
    const float rollDeflection = filterApply(&pid.precomp.rollDeflectionFilter, mixerGetInput(MIXER_IN_STABILIZED_ROLL));

    // Calculate cyclic deflection from the filtered controls
    const float cyclicDeflection = sqrtf(sq(pitchDeflection) + sq(rollDeflection));

    // Collective High Pass Filter (this is possible with PT1)
    const float collectiveLF = pt1FilterApply(&pid.precomp.collDynamicFilter, collectiveDeflection);
    const float collectiveHF = collectiveDeflection - collectiveLF;


  //// Collective-to-Yaw Precomp

    // Collective components
    const float yawCollectiveFF = fabsf(collectiveDeflection) * pid.precomp.yawCollectiveFFGain;
    const float yawCollectiveHF = fabsf(collectiveHF) * pid.precomp.yawCollectiveDynamicGain;

    // Cyclic component
    float yawCyclicFF = fabsf(cyclicDeflection) * pid.precomp.yawCyclicFFGain;

    // Calculate total precompensation
    float yawPrecomp = (yawCollectiveFF + yawCollectiveHF + yawCyclicFF) * masterGain;

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

static float pidTableLookup(float x, const uint8_t * table, int points)
{
    /* Number of bins */
    const int bins = points - 1;

    /* Map x in range 0..1 to piecewise linear table of size count */
    const int index = constrain(x * bins, 0, bins - 1);

    const int a = table[index + 0];
    const int b = table[index + 1];

    const float dy = b - a;
    const float dx = x * bins - index;
    const float y = a + dx * dy;

    return fmaxf(y, 0);
}

static void pidApplyOffsetBleed(const pidProfile_t * pidProfile)
{
    // Actual collective
    const float collective = getCollectiveDeflection();

    // Offset vector
    const float Bx = pid.data[PID_PITCH].axisOffset;
    const float By = pid.data[PID_ROLL ].axisOffset;

    // Cyclic vector
    const float Ax = pid.data[PID_PITCH].setPoint;
    const float Ay = pid.data[PID_ROLL].setPoint;

    // Cyclic norm²
    const float A2 = Ax * Ax + Ay * Ay;

    // Curve lookup input
    const float Cx = sqrtf(A2) / 300.0f;

    // Projection dot-product (>1 for stability)
    const float Dp = (A2 > 1) ? (Ax * Bx + Ay * By) / A2 : 0;

    // Projection components
    const float Px = Ax * Dp;
    const float Py = Ay * Dp;

    // Bleed variables
    float bleedRate = pidTableLookup(Cx, pidProfile->offset_bleed_rate_curve, LOOKUP_CURVE_POINTS) * 0.04f;
    float bleedLimit = pidTableLookup(Cx, pidProfile->offset_bleed_limit_curve, LOOKUP_CURVE_POINTS);

    // Offset bleed amount
    float bleedP = limitf(Px * bleedRate, bleedLimit) * pid.dT;
    float bleedR = limitf(Py * bleedRate, bleedLimit) * pid.dT;

    // Bleed from axisOffset to axisError
    pid.data[PID_PITCH].axisOffset -= bleedP;
    pid.data[PID_ROLL].axisOffset  -= bleedR;
    pid.data[PID_PITCH].axisError  += bleedP * pid.coef[PID_PITCH].Kc * collective;
    pid.data[PID_ROLL].axisError   += bleedR * pid.coef[PID_ROLL].Kc * collective;

    DEBUG(HS_BLEED, 0, pid.data[PID_PITCH].axisOffset * 10);
    DEBUG(HS_BLEED, 1, pid.data[PID_ROLL].axisOffset * 10);
    DEBUG(HS_BLEED, 2, pid.data[PID_PITCH].axisError * 10);
    DEBUG(HS_BLEED, 3, pid.data[PID_ROLL].axisError * 10);
    DEBUG(HS_BLEED, 4, bleedRate * 1000);
    DEBUG(HS_BLEED, 5, bleedLimit * 1000);
    DEBUG(HS_BLEED, 6, bleedP * 1e6);
    DEBUG(HS_BLEED, 7, bleedR * 1e6);
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
 **   setPoint => Kf => F-term
 **   setPoint => difFilter => Kb => B-term
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

    // Select D-term on error or gyro
    const float dError = pid.dtermMode ? errorRate : -gyroRate;

    // Calculate D-term with bandwidth limit
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
    pid.data[axis].axisError -= pid.dT * (isAirborne() ?
      limitf(pid.data[axis].axisError * pid.errorDecayRateCyclic, pid.errorDecayLimitCyclic) :
      pid.data[axis].axisError * pid.errorDecayRateGround);


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

    // Select D-term on error or gyro
    const float dError = pid.dtermModeYaw ? errorRate : -gyroRate;

    // Calculate D-term with bandwidth limit
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
    pid.data[axis].axisError -= pid.dT * (isSpooledUp() ?
      limitf(pid.data[axis].axisError * pid.errorDecayRateYaw, pid.errorDecayLimitYaw) :
      pid.data[axis].axisError * pid.errorDecayRateGround);


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


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 3 - Test mode for New Features
 **
 **   - High Speed Offset
 **
 **   gyroFilter => Kp => P-term
 **   gyroFilter => difFilter => Kd => D-term
 **   gyroFilter => Relax => Ki => I-term
 **
 **   setPoint => Kf => F-term
 **   setPoint => difFilter => Kb => B-term
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyCyclicMode3(uint8_t axis, const pidProfile_t * pidProfile)
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


  //// D-term (gyro only)

    // Select D-term on error or gyro
    const float dError = pid.dtermMode ? errorRate : -gyroRate;

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], dError);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Get actual collective from the mixer
    const float collective = getCollectiveDeflection();

    // Expand to 0..15° range
    const float curve = fabsf(collective) * 0.8f;

    // Apply error decay
    float errorDecayRate, errorDecayLimit;

    if (isAirborne() || pid.errorDecayRateGround == 0) {
      errorDecayRate  = pid.errorDecayRateCyclic * pidTableLookup(curve, pidProfile->error_decay_rate_curve, LOOKUP_CURVE_POINTS) * 0.08f;
      errorDecayLimit = pid.errorDecayLimitCyclic * pidTableLookup(curve, pidProfile->error_decay_limit_curve, LOOKUP_CURVE_POINTS) * 0.08f;
    }
    else {
      errorDecayRate  = pid.errorDecayRateGround;
      errorDecayLimit = 3600;
    }

    const float errorDecay = limitf(pid.data[axis].axisError * errorDecayRate, errorDecayLimit);

    pid.data[axis].axisError -= errorDecay * pid.dT;

    DEBUG_AXIS(ERROR_DECAY, axis, 0, errorDecayRate * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 1, errorDecayLimit);
    DEBUG_AXIS(ERROR_DECAY, axis, 2, errorDecay * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 3, pid.data[axis].axisError * 10);


  //// Offset term

    // Offset saturation
    const bool offSaturation = (pidAxisSaturated(axis) && pid.data[axis].axisOffset * itermErrorRate * collective > 0);

    // Offset change modulated by collective
    const float offMod = copysignf(pidTableLookup(curve, pidProfile->offset_charge_curve, LOOKUP_CURVE_POINTS), collective) / 100.0f;
    const float offDelta = offSaturation ? 0 : itermErrorRate * pid.dT * offMod;

    // Calculate Offset component
    pid.data[axis].axisOffset = limitf(pid.data[axis].axisOffset + offDelta, pid.offsetLimit[axis]);
    pid.data[axis].O = pid.coef[axis].Ko * pid.data[axis].axisOffset * collective;

    DEBUG_AXIS(HS_OFFSET, axis, 0, errorRate * 10);
    DEBUG_AXIS(HS_OFFSET, axis, 1, itermErrorRate * 10);
    DEBUG_AXIS(HS_OFFSET, axis, 2, offMod * 1000);
    DEBUG_AXIS(HS_OFFSET, axis, 3, offDelta * 1000000);
    DEBUG_AXIS(HS_OFFSET, axis, 4, pid.data[axis].axisError * 10);
    DEBUG_AXIS(HS_OFFSET, axis, 5, pid.data[axis].axisOffset * 10);
    DEBUG_AXIS(HS_OFFSET, axis, 6, pid.data[axis].O * 1000);
    DEBUG_AXIS(HS_OFFSET, axis, 7, pid.data[axis].I * 1000);

    // Apply offset decay
    float offsetDecayRate, offsetDecayLimit;

    if (isAirborne() || pid.errorDecayRateGround == 0) {
      offsetDecayRate  = pidTableLookup(curve, pidProfile->offset_decay_rate_curve, LOOKUP_CURVE_POINTS) * 0.04f;
      offsetDecayLimit = pidTableLookup(curve, pidProfile->offset_decay_limit_curve, LOOKUP_CURVE_POINTS);
    }
    else {
      offsetDecayRate  = pid.errorDecayRateGround;
      offsetDecayLimit = 3600;
    }

    const float offsetDecay = limitf(pid.data[axis].axisOffset * offsetDecayRate, offsetDecayLimit);

    pid.data[axis].axisOffset -= offsetDecay * pid.dT;

    DEBUG_AXIS(ERROR_DECAY, axis, 4, offsetDecayRate * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 5, offsetDecayLimit);
    DEBUG_AXIS(ERROR_DECAY, axis, 6, offsetDecay * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 7, pid.data[axis].axisOffset * 10);


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
                            pid.data[axis].F + pid.data[axis].B + pid.data[axis].O;
}


static void pidApplyYawMode3(void)
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

    // Select D-term on error or gyro
    const float dError = pid.dtermModeYaw ? errorRate : -gyroRate;

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], dError);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply error decay
    float decayRate, decayLimit, errorDecay;

    if (isSpooledUp() || !pid.errorDecayRateGround) {
      decayRate = pid.errorDecayRateYaw;
      decayLimit = pid.errorDecayLimitYaw;
    }
    else {
      decayRate = pid.errorDecayRateGround;
      decayLimit = 3600;
    }

    errorDecay = limitf(pid.data[axis].axisError * decayRate, decayLimit);

    pid.data[axis].axisError -= pid.dT * limitf(pid.data[axis].axisError *
      (isSpooledUp() ? pid.errorDecayRateYaw : pid.errorDecayRateGround),
      pid.errorDecayLimitYaw);

    DEBUG_AXIS(ERROR_DECAY, axis, 0, decayRate * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 1, decayLimit);
    DEBUG_AXIS(ERROR_DECAY, axis, 2, errorDecay * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 3, pid.data[axis].axisError * 10);


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

    // Apply PID for each axis
    switch (pid.pidMode) {
        case 3:
            pidApplyCyclicMode3(PID_ROLL, pidProfile);
            pidApplyCyclicMode3(PID_PITCH, pidProfile);
            pidApplyOffsetBleed(pidProfile);
            pidApplyCyclicCrossCoupling();
            pidApplyYawMode3();
            break;
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

    // Calculate stabilized collective
    pidApplyCollective();

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp();

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
