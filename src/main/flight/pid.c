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
#include "flight/feedforward.h"

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

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

#if defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

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
        .yaw_lowpass_hz = 100,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 85,
        .levelAngleLimit = 55,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
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
        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
            // NOTE: dynamic lpf is enabled by default so this setting is actually
            // overridden and the static lowpass 1 is disabled. We can't set this
            // value to 0 otherwise Configurator versions 10.4 and earlier will also
            // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 25,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .vbat_sag_compensation = 0,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25
#define D_LPF_FILT_SCALE 22


const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

#ifdef USE_FEEDFORWARD
float pidGetFeedforwardTransitionFactor()
{
    return pidRuntime.feedforwardTransitionFactor;
}

float pidGetFeedforwardSmoothFactor()
{
    return pidRuntime.feedforwardSmoothFactor;
}

float pidGetFeedforwardJitterFactor()
{
    return pidRuntime.feedforwardJitterFactor;
}

float pidGetFeedforwardBoostFactor()
{
    return pidRuntime.feedforwardBoostFactor;
}
#endif

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#if defined(USE_ACC)
// calculate the stick deflection while applying level mode expo
static float getLevelModeRcDeflection(uint8_t axis)
{
    const float stickDeflection = getRcDeflection(axis);
    if (axis < FD_YAW) {
        const float expof = currentControlRateProfile->levelExpo[axis] / 100.0f;
        return power3(stickDeflection) * expof + stickDeflection * (1 - expof);
    } else {
        return stickDeflection;
    }
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(fabsf(getLevelModeRcDeflection(FD_ROLL)), fabsf(getLevelModeRcDeflection(FD_PITCH)));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (pidRuntime.horizonTiltExpertMode) {
        if (pidRuntime.horizonTransition > 0 && pidRuntime.horizonCutoffDegrees > 0) {
                    // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((pidRuntime.horizonCutoffDegrees-currentInclination) / pidRuntime.horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / pidRuntime.horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (pidRuntime.horizonFactorRatio < 1.0f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180 - currentInclination) / 180 * (1.0f - pidRuntime.horizonFactorRatio) + pidRuntime.horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = pidRuntime.horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = pidRuntime.horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    float angle = pidProfile->levelAngleLimit * getLevelModeRcDeflection(axis);
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * pidRuntime.levelGain;
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * pidRuntime.horizonGain * horizonLevelStrength);
    }
    return currentPidSetpoint;
}

#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

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
    if (pidRuntime.itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
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

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingFeedforwardFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == pidRuntime.rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (pidRuntime.feedforwardLpfInitialized) {
        ret = pt3FilterApply(&pidRuntime.feedforwardPt3[axis], pidSetpointDelta);
        if (axis == pidRuntime.rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    UNUSED(itermErrorRate);

    if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&pidRuntime.acLpf[axis], *currentPidSetpoint);
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
            if (fabsf(acErrorRate * pidRuntime.dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidRuntime.pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
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
#if !defined(USE_ACC)
    UNUSED(pidProfile);
#endif
    UNUSED(currentTimeUs);

    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#endif

    // gradually scale back integration when above windup point
    float dynCi = pidRuntime.dT;
    if (pidRuntime.itermWindupPointInv > 1.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }

    // Precalculate gyro deta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
        // -----calculate raw, unfiltered D component

        // Divide rate change by dT to get differential (ie dr/dt).
        // dT is fixed and calculated from the target PID loop time
        // This is done to avoid DTerm spikes that occur with dynamically
        // calculated deltaT whenever another task causes the PID
        // loop execution to be delayed.
        const float delta =
            - (gyroRateDterm[axis] - previousRawGyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
        previousRawGyroRateDterm[axis] = gyroRateDterm[axis];

        // Log the unfiltered D
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_D_LPF, 0, lrintf(delta));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_D_LPF, 1, lrintf(delta));
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_FEEDFORWARD
    bool newRcFrame = false;
    if (getShouldUpdateFeedforward()) {
        newRcFrame = true;
    }
#endif

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

#if defined(USE_ACC)
        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && pidRuntime.acroTrainerActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
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
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate;
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki;
        float axisDynCi;
        {
            Ki = pidRuntime.pidCoefficient[axis].Ki;
            axisDynCi = (axis == FD_YAW) ? dynCi : pidRuntime.dT; // only apply windup protection to yaw
        }

        pidData[axis].I = constrainf(previousIterm + (Ki * axisDynCi) * itermErrorRate, -pidRuntime.itermLimit, pidRuntime.itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_FEEDFORWARD
        pidSetpointDelta = feedforwardApply(axis, newRcFrame, pidRuntime.feedforwardAveraging);
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate D component
        if (pidRuntime.pidCoefficient[axis].Kd > 0) {

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta =
                - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;

            pidData[axis].D = preTpaD;

            // Log the value of D
            preTpaD *= D_LPF_FILT_SCALE;

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_LPF, 2, lrintf(preTpaD));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_LPF, 3, lrintf(preTpaD));
            }
        } else {
            pidData[axis].D = 0;

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_LPF, 2, 0);
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_LPF, 3, 0);
            }
        }

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in feedforward
        pidSetpointDelta += setpointCorrection - pidRuntime.oldSetpointCorrection[axis];
        pidRuntime.oldSetpointCorrection[axis] = setpointCorrection;
#endif

        float feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // halve feedforward in Level mode since stick sensitivity is weaker by about half
            feedforwardGain *= FLIGHT_MODE(ANGLE_MODE) ? 0.5f : 1.0f;
            // transition now calculated in feedforward.c when new RC data arrives 
            float feedForward = feedforwardGain * pidSetpointDelta * pidRuntime.pidFrequency;

#ifdef USE_FEEDFORWARD
            pidData[axis].F = shouldApplyFeedforwardLimits(axis) ?
                applyFeedforwardLimit(axis, feedForward, pidRuntime.pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
#else
            pidData[axis].F = feedForward;
#endif
#ifdef USE_RC_SMOOTHING_FILTER
            pidData[axis].F = applyRcSmoothingFeedforwardFilter(axis, pidData[axis].F);
#endif // USE_RC_SMOOTHING_FILTER
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

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
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
