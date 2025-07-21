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

static FAST_DATA_ZERO_INIT pidData_t pid;

static const uint8_t error_decay_rate_curve[PID_LOOKUP_CURVE_POINTS]   = { 12,13,14,15,17,20,23,28,36,49,78,187,250,250,250,250 };
static const uint8_t error_decay_limit_curve[PID_LOOKUP_CURVE_POINTS]  = { 12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12 };

static const uint8_t offset_decay_rate_curve[PID_LOOKUP_CURVE_POINTS]  = { 250,250,250,250,250,30,5,0,0,0,0,0,0,0,0,0 };
static const uint8_t offset_decay_limit_curve[PID_LOOKUP_CURVE_POINTS] = { 12,12,10,8,6,4,2,2,2,2,2,2,2,2,2,2 };

static const uint8_t offset_bleed_rate_curve[PID_LOOKUP_CURVE_POINTS]  = { 0,0,0,0,0,0,2,4,30,250,250,250,250,250,250,250 };
static const uint8_t offset_bleed_limit_curve[PID_LOOKUP_CURVE_POINTS] = { 0,0,0,0,0,0,15,40,100,150,200,250,250,250,250,250 };

static const uint8_t offset_charge_curve[PID_LOOKUP_CURVE_POINTS]      = { 0,100,100,100,100,100,95,90,82,76,72,68,65,62,60,58 };
static const uint8_t offset_flood_curve[PID_LOOKUP_CURVE_POINTS]       = { 0,0,0,20,50,100,180,220,220,220,220,220,220,220,220,220 };


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

static void INIT_CODE pidInitFilters(const pidProfile_t *pidProfile)
{
    // PID Derivative Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        difFilterInit(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterInit(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
    }

    // RPM change filter
    lowpassFilterInit(&pid.precomp.headspeedFilter, LPF_PT2, 20, pid.freq, 0);
    difFilterInit(&pid.precomp.yawInertiaFilter, pidProfile->yaw_inertia_precomp_cutoff / 10.0f, pid.freq);

    // Cross-coupling filters
    firstOrderHPFInit(&pid.crossCouplingFilter[FD_PITCH], pidProfile->cyclic_cross_coupling_cutoff / 10.0f, pid.freq);
    firstOrderHPFInit(&pid.crossCouplingFilter[FD_ROLL], pidProfile->cyclic_cross_coupling_cutoff / 10.0f, pid.freq);
}

void INIT_CODE pidLoadProfile(const pidProfile_t *pidProfile)
{
    // PID not initialised yet
    if (pid.dT == 0)
      return;

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

    // Adjust for PID Mode4
    if (pidProfile->pid_mode == 4) {
      pid.coef[PID_PITCH].Kb *= 10;
      pid.coef[PID_ROLL].Kd /= 5;
      pid.coef[PID_ROLL].Kb /= 5;
    }

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

    // If in rate-mode (I-gain == 0), decay remaining I-term
    if (pidProfile->pid_mode == 4 && pidProfile->pid[PID_YAW].I == 0) {
      pid.errorDecayRateYaw = 10.0f / 50;
    }

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        lowpassFilterInit(&pid.gyrorFilter[i], pidProfile->gyro_filter_type, pidProfile->gyro_cutoff[i], pid.freq, 0);
        difFilterUpdate(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterUpdate(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
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

    // Tail/yaw PID parameters
    pid.yawCWStopGain = pidProfile->yaw_cw_stop_gain / 100.0f;
    pid.yawCCWStopGain = pidProfile->yaw_ccw_stop_gain / 100.0f;

    // Collective/cyclic deflection lowpass filters
    lowpassFilterInit(&pid.precomp.yawPrecompFilter,
      pidProfile->yaw_precomp_filter_type,
      (pidProfile->pid_mode == 4) ? pidProfile->yaw_precomp_cutoff / 10.0f : pidProfile->yaw_precomp_cutoff,
      pid.freq, 0);

    // RPM change filter
    difFilterUpdate(&pid.precomp.yawInertiaFilter, pidProfile->yaw_inertia_precomp_cutoff / 10.0f, pid.freq);

    // Tail/yaw precomp
    pid.precomp.yawCollectiveFFGain = pidProfile->yaw_collective_ff_gain / 100.0f;
    pid.precomp.yawCyclicFFGain = pidProfile->yaw_cyclic_ff_gain / 100.0f;
    pid.precomp.yawInertiaGain = pidProfile->yaw_inertia_precomp_gain / 200.0f;

    // Pitch precomp
    pid.precomp.pitchCollectiveFFGain = pidProfile->pitch_collective_ff_gain / 500.0f;

    // Cross-coupling compensation
    pid.cyclicCrossCouplingGain[FD_PITCH] = pidProfile->cyclic_cross_coupling_gain * mixerRotationSign() * -CROSS_COUPLING_SCALE;
    pid.cyclicCrossCouplingGain[FD_ROLL]  = pid.cyclicCrossCouplingGain[FD_PITCH] * pidProfile->cyclic_cross_coupling_ratio / -100.0f;

    // Cross-coupling filters
    firstOrderHPFUpdate(&pid.crossCouplingFilter[FD_PITCH], pidProfile->cyclic_cross_coupling_cutoff / 10.0f, pid.freq);
    firstOrderHPFUpdate(&pid.crossCouplingFilter[FD_ROLL], pidProfile->cyclic_cross_coupling_cutoff / 10.0f, pid.freq);

    // Offset flood
    pid.offsetFloodRelaxLevel = 1.0f / constrain(pidProfile->offset_flood_relax_level, 10, 250);
    const uint8_t offset_flood_relax_freq = constrain(pidProfile->offset_flood_relax_cutoff, 1, 100);
    pt1FilterInit(&pid.offsetFloodRelaxFilter, offset_flood_relax_freq, pid.freq);

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

void INIT_CODE pidChangeProfile(const pidProfile_t *pidProfile)
{
    pidLoadProfile(pidProfile);
    pidResetAxisErrors();
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidReset();
    pidSetLooptime(gyro.targetLooptime);
    pidInitFilters(pidProfile);
    pidChangeProfile(pidProfile);
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

static inline float dragCoef(float x)
{
  /**
   * Alternatives
   *   - 7th order approx: (x^7 + 3x^2 + x) / 5
   *   - 5th order approx: (x^5 + 3x^2 + x) / 5
   *   - 2nd order approx: x^2
   *   - 1st order approx: x
   *
   * The 7th order is closest to simulation results, but would cause excessive
   * yaw deflection at high collective angles (>14deg).
   *
   * The second order approx is accurate up to 12deg, and doesn't cause issues
   * with excessive yaw or saturation.
   */
  return x * x;
}

static void pidApplyPrecomp(void)
{
    // Yaw precompensation direction and ratio
    const float masterGain = mixerRotationSign() * getSpoolUpRatio();

    // Get actual control deflections (from previous cycle)
    const float collectiveDeflection = getCollectiveDeflection();
    const float cyclicDeflection = getCyclicDeflection();


  //// Main rotor intertia precomp

    // Normalised effective rotor speed
    //const float rotorSpeed = getHeadSpeedf() / 3000;
    const float rotorSpeed = (getHeadSpeedf() + mixerRotationSign() * pidGetSetpoint(FD_YAW) / 6) / 3000;

    // Rotorspeed derivative
    const float speedFiltered = filterApply(&pid.precomp.headspeedFilter, rotorSpeed);
    const float speedChange = difFilterApply(&pid.precomp.yawInertiaFilter, speedFiltered);

    // Momentum change precomp
    const float torquePrecomp = speedChange * pid.precomp.yawInertiaGain;


  //// Collective-to-Yaw Precomp

    // Equivalent Average main rotor deflection
    const float mainDeflection =
      fabsf(collectiveDeflection) * pid.precomp.yawCollectiveFFGain +
      fabsf(cyclicDeflection) * pid.precomp.yawCyclicFFGain;

    // Drag estimate
    const float mainDrag = dragCoef(mainDeflection);

    // Apply filter
    const float mainPrecomp = filterApply(&pid.precomp.yawPrecompFilter, mainDrag);

    // Total precomp with direction
    const float totalPrecomp = (mainPrecomp + torquePrecomp) * masterGain;

    // Add to YAW feedforward
    pid.data[FD_YAW].F += totalPrecomp;
    pid.data[FD_YAW].pidSum += totalPrecomp;

    DEBUG(YAW_PRECOMP, 0, totalPrecomp * 1000);
    DEBUG(YAW_PRECOMP, 1, mainPrecomp * 1000);
    DEBUG(YAW_PRECOMP, 2, mainDeflection * 1000);
    DEBUG(YAW_PRECOMP, 3, collectiveDeflection * 1000);
    DEBUG(YAW_PRECOMP, 4, cyclicDeflection * 1000);
    DEBUG(YAW_PRECOMP, 6, speedChange * 1000);
    DEBUG(YAW_PRECOMP, 7, torquePrecomp * 1000);


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
    // Cross-coupling filters
    const float pitchDeriv = firstOrderFilterApply(&pid.crossCouplingFilter[FD_PITCH], pid.data[FD_PITCH].setPoint);
    const float rollDeriv  = firstOrderFilterApply(&pid.crossCouplingFilter[FD_ROLL], pid.data[FD_ROLL].setPoint);
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
 ** MODE 3 - Current default PID mode
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyOffsetBleedMode3(void)
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
    float bleedRate = pidTableLookup(Cx, offset_bleed_rate_curve, PID_LOOKUP_CURVE_POINTS) * 0.04f;
    float bleedLimit = pidTableLookup(Cx, offset_bleed_limit_curve, PID_LOOKUP_CURVE_POINTS);

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

static void pidApplyOffsetFloodMode3(void)
{
    // Calculate `offsetFloodRelaxFactor`
    const float collective = getCollectiveDeflection();
    const float collectiveLpf = pt1FilterApply(&pid.offsetFloodRelaxFilter, collective);
    const float collectiveHpf = collective - collectiveLpf;
    const float offsetFloodRelaxFactor = fmaxf(0, 1.0f - fabsf(collectiveHpf) * pid.offsetFloodRelaxLevel);

    // Prepare curve lookup. Curve points are stored in 0..15° range.
    const float curve = fabsf(collective) * 0.8f;

    // Apply on ROLL and PITCH
    for (uint8_t axis = PID_ROLL; axis <= PID_PITCH; axis++)
    {
        // The algorithm only makes sense if Kc <> 0
        if (pid.coef[axis].Kc == 0)
            continue;

        const float axisError = pid.data[axis].axisError;
        const float axisOffset = pid.data[axis].axisOffset;

        // 0. calculate bleed rate
        float bleedRate = pidTableLookup(curve, offset_flood_curve, PID_LOOKUP_CURVE_POINTS) * 0.08f;
        bleedRate = copysignf(bleedRate, axisError) * offsetFloodRelaxFactor;

        // 1. offsetDelta = value to be added to axisOffset
        float offsetDelta = bleedRate * pid.dT;

        // 1. determin sign of offsetDelta
        // offsetDelta is positive if bleedRate>0 && collective>0 || bleedRate<0
        // && collective<0
        offsetDelta = copysignf(offsetDelta, bleedRate * collective);

        // 1. Check offsetLimit
        offsetDelta = limitf(axisOffset + offsetDelta, pid.offsetLimit[axis]) - axisOffset;

        // 2. calculate equivalent output delta and errorDelta
        // errorDelta = value to be substract from axisError.
        float errorDelta = offsetDelta * collective * pid.coef[axis].Kc;

        // 2. Check axisError limit
        // Note: axisError and errorDelta have same sign
        // collective == 0 -> errorDelta == 0 and we will not enter (safe)
        if (fabsf(axisError) - fabsf(errorDelta) < 0) {
            // We need to re-calculate outputDelta and offsetDelta:
            errorDelta = axisError;
            offsetDelta = errorDelta / (collective * pid.coef[axis].Kc);
        }

        // 3. Update axisError and axisOffset
        pid.data[axis].axisError -= errorDelta;
        pid.data[axis].axisOffset += offsetDelta;
    }
}


static void pidApplyCyclicMode3(uint8_t axis)
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

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

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

    // Convert 0..15° => 0..1
    const float curve = fabsf(collective) * 0.8f;

    // Apply error decay
    float errorDecayRate, errorDecayLimit;

    if (isAirborne() || pid.errorDecayRateGround == 0) {
      errorDecayRate  = pid.errorDecayRateCyclic * pidTableLookup(curve, error_decay_rate_curve, PID_LOOKUP_CURVE_POINTS) * 0.08f;
      errorDecayLimit = pid.errorDecayLimitCyclic * pidTableLookup(curve, error_decay_limit_curve, PID_LOOKUP_CURVE_POINTS) * 0.08f;
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
    const float offMod = copysignf(pidTableLookup(curve, offset_charge_curve, PID_LOOKUP_CURVE_POINTS), collective) / 100.0f;
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
      offsetDecayRate  = pidTableLookup(curve, offset_decay_rate_curve, PID_LOOKUP_CURVE_POINTS) * 0.04f;
      offsetDecayLimit = pidTableLookup(curve, offset_decay_limit_curve, PID_LOOKUP_CURVE_POINTS);
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

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

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
    float decayRate, decayLimit;

    if (isSpooledUp()) {
      decayRate = pid.errorDecayRateYaw;
      decayLimit = pid.errorDecayLimitYaw;
    }
    else {
      decayRate = pid.errorDecayRateGround;
      decayLimit = 3600;
    }

    const float errorDecay = limitf(pid.data[axis].axisError * decayRate, decayLimit);

    pid.data[axis].axisError -= errorDecay * pid.dT;

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


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 4 - Test mode for new features
 **
 **   - axisError is now scaled with I-gain
 **   - I-gain for Pitch and Roll must be equal
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyOffsetBleedMode4(void)
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
    float bleedRate = pidTableLookup(Cx, offset_bleed_rate_curve, PID_LOOKUP_CURVE_POINTS) * 0.04f;
    float bleedLimit = pidTableLookup(Cx, offset_bleed_limit_curve, PID_LOOKUP_CURVE_POINTS);

    // Offset bleed amount
    float bleedP = limitf(Px * bleedRate, bleedLimit) * pid.dT;
    float bleedR = limitf(Py * bleedRate, bleedLimit) * pid.dT;

    // Bleed from axisOffset to axisError
    pid.data[PID_PITCH].axisOffset -= bleedP;
    pid.data[PID_ROLL].axisOffset  -= bleedR;
    pid.data[PID_PITCH].axisError  += bleedP * collective;
    pid.data[PID_ROLL].axisError   += bleedR * collective;

    DEBUG(HS_BLEED, 0, pid.data[PID_PITCH].axisOffset * 1000);
    DEBUG(HS_BLEED, 1, pid.data[PID_ROLL].axisOffset * 1000);
    DEBUG(HS_BLEED, 2, pid.data[PID_PITCH].axisError * 1000);
    DEBUG(HS_BLEED, 3, pid.data[PID_ROLL].axisError * 1000);
    DEBUG(HS_BLEED, 4, bleedRate * 1000);
    DEBUG(HS_BLEED, 5, bleedLimit * 1000);
    DEBUG(HS_BLEED, 6, bleedP * 1e6);
    DEBUG(HS_BLEED, 7, bleedR * 1e6);
}

static void pidApplyOffsetFloodMode4(void)
{
    // Calculate `offsetFloodRelaxFactor`
    const float collective = getCollectiveDeflection();
    const float collectiveLpf = pt1FilterApply(&pid.offsetFloodRelaxFilter, collective);
    const float collectiveHpf = collective - collectiveLpf;
    const float offsetFloodRelaxFactor = fmaxf(0, 1.0f - fabsf(collectiveHpf) * pid.offsetFloodRelaxLevel);

    // Prepare curve lookup. Curve points are stored in 0..15° range.
    const float curve = fabsf(collective) * 0.8f;

    // Apply on ROLL and PITCH
    for (uint8_t axis = PID_ROLL; axis <= PID_PITCH; axis++)
    {
        const float axisError = pid.data[axis].axisError;
        const float axisOffset = pid.data[axis].axisOffset;

        // 0. calculate bleed rate
        float bleedRate = pidTableLookup(curve, offset_flood_curve, PID_LOOKUP_CURVE_POINTS) * 0.08f;
        bleedRate = copysignf(bleedRate, axisError) * offsetFloodRelaxFactor;

        // 1. offsetDelta = value to be added to axisOffset
        float offsetDelta = bleedRate * pid.dT;

        // 1. determin sign of offsetDelta
        // offsetDelta is positive if bleedRate>0 && collective>0 || bleedRate<0
        // && collective<0
        offsetDelta = copysignf(offsetDelta, bleedRate * collective);

        // 1. Check offsetLimit
        offsetDelta = limitf(axisOffset + offsetDelta, pid.offsetLimit[axis] * pid.coef[axis].Ko) - axisOffset;

        // 2. calculate equivalent output delta and errorDelta
        // errorDelta = value to be substract from axisError.
        float errorDelta = offsetDelta * collective;

        // 2. Check axisError limit
        // Note: axisError and errorDelta have same sign
        // collective == 0 -> errorDelta == 0 and we will not enter (safe)
        if (fabsf(axisError) - fabsf(errorDelta) < 0) {
            // We need to re-calculate outputDelta and offsetDelta:
            errorDelta = axisError;
            offsetDelta = axisError / collective;
        }

        // 3. Update axisError and axisOffset
        pid.data[axis].axisError -= errorDelta;
        pid.data[axis].axisOffset += offsetDelta;
    }
}


static void pidApplyCyclicMode4(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;

    // Gains must be equal for both axis
    const float Ki = fminf(pid.coef[PID_ROLL].Ki, pid.coef[PID_PITCH].Ki);
    const float Ko = fminf(pid.coef[PID_ROLL].Ko, pid.coef[PID_PITCH].Ko);
    const float Kf = fminf(pid.coef[PID_ROLL].Kf, pid.coef[PID_PITCH].Kf);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term (gyro only)

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT * Ki;

    // Calculate I-component (axisError and I are the same in Mode 4)
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis] * Ki);
    pid.data[axis].I = pid.data[axis].axisError;

    // Get actual collective from the mixer
    const float collective = getCollectiveDeflection();

    // Convert 0..15° => 0..1
    const float curve = fabsf(collective) * 0.8f;

    // Apply error decay
    float errorDecayRate, errorDecayLimit;

    if (isAirborne() || pid.errorDecayRateGround == 0) {
      errorDecayRate  = pidTableLookup(curve, error_decay_rate_curve, PID_LOOKUP_CURVE_POINTS) *
        pid.errorDecayRateCyclic * 0.08f;
      errorDecayLimit = pidTableLookup(curve, error_decay_limit_curve, PID_LOOKUP_CURVE_POINTS) *
        pid.errorDecayLimitCyclic * 0.08f;
    }
    else {
      errorDecayRate  = pid.errorDecayRateGround;
      errorDecayLimit = 3600;
    }

    const float errorDecay = limitf(pid.data[axis].axisError * errorDecayRate, errorDecayLimit * Ki);

    pid.data[axis].axisError -= errorDecay * pid.dT;

    DEBUG_AXIS(ERROR_DECAY, axis, 0, errorDecayRate * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 1, errorDecayLimit);
    DEBUG_AXIS(ERROR_DECAY, axis, 2, errorDecay * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 3, pid.data[axis].axisError * 1000);


  //// Offset term

    // Offset saturation
    const bool offSaturation = (pidAxisSaturated(axis) && pid.data[axis].axisOffset * itermErrorRate * collective > 0);

    // Offset change modulated by collective
    const float offMod = copysignf(pidTableLookup(curve, offset_charge_curve, PID_LOOKUP_CURVE_POINTS), collective) / 100.0f;
    const float offDelta = offSaturation ? 0 : itermErrorRate * pid.dT * offMod * Ko;

    // Calculate Offset component
    pid.data[axis].axisOffset = limitf(pid.data[axis].axisOffset + offDelta, pid.offsetLimit[axis] * Ko);
    pid.data[axis].O = pid.data[axis].axisOffset * collective;

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
      offsetDecayRate  = pidTableLookup(curve, offset_decay_rate_curve, PID_LOOKUP_CURVE_POINTS) * 0.04f;
      offsetDecayLimit = pidTableLookup(curve, offset_decay_limit_curve, PID_LOOKUP_CURVE_POINTS);
    }
    else {
      offsetDecayRate  = pid.errorDecayRateGround;
      offsetDecayLimit = 3600;
    }

    const float offsetDecay = limitf(pid.data[axis].axisOffset * offsetDecayRate, offsetDecayLimit * Ko);

    pid.data[axis].axisOffset -= offsetDecay * pid.dT;

    DEBUG_AXIS(ERROR_DECAY, axis, 4, offsetDecayRate * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 5, offsetDecayLimit);
    DEBUG_AXIS(ERROR_DECAY, axis, 6, offsetDecay * 100);
    DEBUG_AXIS(ERROR_DECAY, axis, 7, pid.data[axis].axisOffset * 10);


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = Kf * setpoint;


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


static void pidApplyYawMode4(void)
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
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT * pid.coef[axis].Ki * stopGain;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis] * pid.coef[axis].Ki);
    pid.data[axis].I = pid.data[axis].axisError;

    // Apply error decay
    float decayRate, decayLimit;

    if (isSpooledUp()) {
      decayRate = pid.errorDecayRateYaw;
      decayLimit = pid.errorDecayLimitYaw;
    }
    else {
      decayRate = pid.errorDecayRateGround;
      decayLimit = 3600;
    }

    const float errorDecay = limitf(pid.data[axis].axisError * decayRate, decayLimit * pid.coef[axis].Ki);

    pid.data[axis].axisError -= errorDecay * pid.dT;

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
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Apply PID for each axis
    switch (pid.pidMode) {
        case 4:
            pidApplyCyclicMode4(PID_ROLL);
            pidApplyCyclicMode4(PID_PITCH);
            pidApplyOffsetBleedMode4();
            pidApplyOffsetFloodMode4();
            pidApplyCyclicCrossCoupling();
            pidApplyYawMode4();
            break;
        case 3:
            pidApplyCyclicMode3(PID_ROLL);
            pidApplyCyclicMode3(PID_PITCH);
            pidApplyOffsetBleedMode3();
            pidApplyOffsetFloodMode3();
            pidApplyCyclicCrossCoupling();
            pidApplyYawMode3();
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
