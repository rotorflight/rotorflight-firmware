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

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"


// Headspeed quality levels
#define GOV_HS_DETECT_DELAY             200
#define GOV_HS_DETECT_RATIO             0.05f

// RPM glitch ratio
#define GOV_HS_GLITCH_DELTA             0.25f
#define GOV_HS_GLITCH_LIMIT             2.0f

// Lost headspeed levels
#define GOV_HS_INVALID_RATIO            0.01f
#define GOV_HS_INVALID_THROTTLE         0.10f

// Nominal battery cell voltage
#define GOV_NOMINAL_CELL_VOLTAGE        3.70f

// Dynamic min throttle limit (%)
#define GOV_DYN_MIN_THROTTLE_LIMIT      0.80f

// Motor constant K filter
#define GOV_RPM_K_CUTOFF                0.05f

// Approx throttle headroom
#define GOV_THROTTLE_HEADROOM           1.25f


//// Internal Data

typedef struct {

    // Governor type
    govMode_e       govMode;

    // Throttle channel type
    govThrottle_e   throttleType;

    // Governor features
    bool            useBypass;
    bool            useSuspend;
    bool            useThreePosThrottle;
    bool            useFcThrottleCurve;
    bool            useTxPrecompCurve;
    bool            useFallbackPrecomp;
    bool            useHsAdjustment;
    bool            usePidSpoolup;
    bool            useVoltageComp;
    bool            useDynMinThrottle;
    bool            useMotorConstant;
    bool            useAutoRotation;
    bool            useTTA;

    // State machine
    govState_e      state;
    timeMs_t        stateEntryTime;

    // State reset requested
    bool            stateResetReq;

    // Headspeed spoolup active
    bool            hsSpoolupActive;

    // Output throttle
    float           throttleOutput;

    // Input throttle
    float           throttleInput;
    bool            throttleInputOff;
    float           throttlePrevInput;

    // Fallback throttle reduction
    float           fallbackRatio;

    // Collective-to-throttle curve points
    float           wotCollective;
    float           idleCollective;

    // Absolute maximum throttle output
    float           maxThrottle;

    // Idle throttle level
    float           idleThrottle;

    // Autorotation throttle level
    float           autoThrottle;

    // Handover level
    float           handoverThrottle;

    // Startup/spoolup throttle levels
    float           minSpoolupThrottle;
    float           maxSpoolupThrottle;

    // Governor active throttle limits
    float           minActiveThrottle;

    // Current headspeed
    float           currentHeadSpeed;

    // Headspeed config
    float           fullHeadSpeed;
    float           targetHeadSpeed;
    float           requestedHeadSpeed;

    // Proportial headspeeds
    float           requestRatio;
    float           fullHeadSpeedRatio;

    // Main gear ratio (motor/head)
    float           mainGearRatio;

    // RPM Signal Flags
    bool            motorRPMGood;

    // RPM filter & detection
    filter_t        motorRPMFilter;
    float           motorRPMGlitchDelta;
    float           motorRPMGlitchLimit;
    uint32_t        motorRPMDetectTime;

    // RPM constant (approx HS at full throttle)
    float           motorRPMK;
    ewma1Filter_t   motorRPMKFilter;

    // Battery voltage
    float           motorVoltage;
    filter_t        motorVoltageFilter;

    // Nominal battery voltage
    float           nominalVoltage;

    // Voltage compensation gain
    float           voltageCompGain;

    // Dynamic min throttle limit
    float           dynMinLevel;  // TDB remove

    // PID terms
    float           P;
    float           I;
    float           C;
    float           D;
    float           F;
    float           pidSum;

    // Differentiator with bandwidth limiter
    difFilter_t     differentiator;

    // PID Gains
    float           K;
    float           Kp;
    float           Ki;
    float           Kd;
    float           Kf;

    // PID Limits
    float           maxP;
    float           minP;
    float           maxI;
    float           minI;
    float           maxD;
    float           minD;
    float           minF;
    float           maxF;

    // Feedforward
    float           yawWeight;
    float           cyclicWeight;
    float           collectiveWeight;
    uint            collectiveCurve;
    filter_t        precompFilter;

    // Tail Torque Assist
    float           ttaAdd;
    float           ttaGain;
    float           ttaLimit;
    filter_t        ttaFilter;

    // Timeouts
    long            throttleHoldTimeout;

    // Throttle change rates
    float           throttleStartupRate;
    float           throttleSpoolupRate;
    float           throttleRecoveryRate;
    float           throttleTrackingRate;
    float           throttleSpooldownRate;

} govData_t;

static FAST_DATA_ZERO_INIT govData_t gov;


//// Prototypes

static void governorInitTTA(const pidProfile_t *pidProfile);


//// Access functions

int getGovernorState(void)
{
    return gov.state;
}

float getGovernorOutput(void)
{
    return gov.throttleOutput;
}

float getTTAIncrease(void)
{
    return gov.ttaAdd;
}

void getGovernorLogData(govLogData_t *data)
{
    if (data) {
        data->targetHS = gov.targetHeadSpeed;
        data->requestHS = gov.requestedHeadSpeed;
        data->pidTerms[0] = gov.P * 1000;
        data->pidTerms[1] = gov.I * 1000;
        data->pidTerms[2] = gov.D * 1000;
        data->pidTerms[3] = gov.F * 1000;
        data->pidSum = gov.pidSum * 1000;
    }
}

float getFullHeadSpeedRatio(void)
{
    if (gov.govMode > GOV_MODE_EXTERNAL) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLUP:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
                return gov.fullHeadSpeedRatio;

            default:
                return 1.0f;
        }
    }

    return 1.0f;
}

float getSpoolUpRatio(void)
{
    if (!ARMING_FLAG(ARMED))
        return 0;

    if (gov.govMode) {
        switch (gov.state)
        {
            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_THROTTLE_IDLE:
            case GOV_STATE_AUTOROTATION:
                return 0;

            case GOV_STATE_ACTIVE:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_DISABLED:
                return 1.0f;

            case GOV_STATE_SPOOLUP:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_RECOVERY:
                return gov.requestRatio;
        }
        return 0;
    }
    else {
        return 1.0f;
    }

    return 0;
}

bool isSpooledUp(void)
{
    if (!ARMING_FLAG(ARMED))
        return false;

    if (gov.govMode) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_DISABLED:
                return true;

            case GOV_STATE_SPOOLUP:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_FALLBACK:
                return (gov.throttleOutput > 0.333f);

            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_THROTTLE_IDLE:
                return false;
        }
        return false;
    }
    else {
        return (gov.throttleOutput > 0.333f);
    }

    return false;
}


//// Internal functions

static void govDebugGovernor(void)
{
    DEBUG(GOVERNOR, 0, gov.requestedHeadSpeed);
    DEBUG(GOVERNOR, 1, gov.targetHeadSpeed);
    DEBUG(GOVERNOR, 2, gov.currentHeadSpeed);
    DEBUG(GOVERNOR, 3, gov.pidSum * 1000);
    DEBUG(GOVERNOR, 4, gov.P * 1000);
    DEBUG(GOVERNOR, 5, gov.I * 1000);
    DEBUG(GOVERNOR, 6, gov.D * 1000);
    DEBUG(GOVERNOR, 7, gov.F * 1000);
}

static inline bool isAutorotation(void)
{
    return IS_RC_MODE_ACTIVE(BOXAUTOROTATION) || gov.useAutoRotation;
}

static inline bool isGovDisabled(void)
{
    return IS_RC_MODE_ACTIVE(BOXGOVBYPASS) || gov.useBypass;
}

static inline bool isGovSuspend(void)
{
    return IS_RC_MODE_ACTIVE(BOXGOVSUSPEND) || gov.useSuspend;
}

static inline bool isForcedFallback(void)
{
    return IS_RC_MODE_ACTIVE(BOXGOVFALLBACK);
}

static inline void govChangeState(govState_e futureState)
{
    if (gov.state != futureState) {
        gov.state = futureState;
        gov.stateResetReq = true;
        gov.stateEntryTime = millis();
    }
}

static inline long govStateTime(void)
{
    return cmp32(millis(),gov.stateEntryTime);
}

static inline float govGetMappedThrottle(void)
{
    return transition(getRcDeflection(COLLECTIVE), gov.idleCollective, gov.wotCollective, gov.idleThrottle, 1.0f);
}

static void govGetInputThrottle(void)
{
    gov.throttleInput = getThrottle();
    gov.throttleInputOff = isThrottleOff();

    switch (gov.throttleType)
    {
        case GOV_THROTTLE_NORMAL:
            if (gov.useHsAdjustment && gov.throttleInput > gov.handoverThrottle) {
                gov.requestedHeadSpeed = fmaxf(gov.throttleInput * gov.fullHeadSpeed, 100);
                gov.throttleInput = 1.0f;
            }
            break;

        case GOV_THROTTLE_OFF_ON:
            if (gov.throttleInput < 0.666f) {
                gov.throttleInput = 0;
                gov.throttleInputOff = true;
            }
            else {
                gov.throttleInput = gov.useFcThrottleCurve ? govGetMappedThrottle() : 1.0f;
            }
            break;

        case GOV_THROTTLE_OFF_IDLE_ON:
            if (gov.throttleInput < 0.333f) {
                gov.throttleInput = 0;
                gov.throttleInputOff = true;
            }
            else if (gov.throttleInput < 0.666f) {
                gov.throttleInput = gov.idleThrottle;
                gov.throttleInputOff = false;
            }
            else {
                gov.throttleInput = gov.useFcThrottleCurve ? govGetMappedThrottle() : 1.0f;
                gov.throttleInputOff = false;
            }
            break;

        case GOV_THROTTLE_OFF_IDLE_AUTO_ON:
            if (!gov.throttleInputOff) {
                if (gov.throttleInput < 0.333f) {
                    gov.throttleInput = 0;
                    gov.throttleInputOff = false;
                    gov.useAutoRotation = false;
                }
                else if (gov.throttleInput < 0.666f) {
                    gov.throttleInput = 0;
                    gov.throttleInputOff = false;
                    gov.useAutoRotation = true;
                }
                else {
                    gov.throttleInput = gov.useFcThrottleCurve ? govGetMappedThrottle() : 1.0f;
                    gov.throttleInputOff = false;
                    gov.useAutoRotation = false;
                }
            }
            break;
    }
}

static inline float precompCurve(float angle, uint8_t curve)
{
    return pow_approx(fabsf(angle), curve / 10.0f);
}

static float govCalcFeedforward(void)
{
    float totalFF = 0;

    // Throttle curve in the Tx
    if (gov.useTxPrecompCurve) {
        // Use throttle input directly as feedforward
        totalFF = gov.throttleInput;
    }
    else {
        // Collective deflection
        const float collectiveFF = gov.collectiveWeight * precompCurve(getCollectiveDeflectionAbs(), gov.collectiveCurve);

        // Cyclic deflection
        const float cyclicFF = gov.cyclicWeight * getCyclicDeflection();

        // Yaw deflection
        const float yawFF = gov.yawWeight * getYawDeflectionAbs();

        // Total feedforward
        totalFF = gov.K * gov.Kf * (collectiveFF + cyclicFF + yawFF);
    }

    // Filtered value
    totalFF = filterApply(&gov.precompFilter, totalFF);

    return totalFF;
}

static float govCalcVoltCompGain(void)
{
    float gain = 1;

    if (gov.useVoltageComp) {
        const float Vnom = getBatteryCellCount() * GOV_NOMINAL_CELL_VOLTAGE;
        const float Vbat = getBatteryVoltageMeter()->sample * 0.001f;

        if (Vnom > 2.0f && Vbat > 2.0f) {
            gov.motorVoltage = filterApply(&gov.motorVoltageFilter, Vbat);
            gov.nominalVoltage = Vnom;
            if (gov.motorVoltage > 1.0f)
                gain = constrainf(gov.nominalVoltage / gov.motorVoltage, 0.80f, 1.2f);
        }

        DEBUG(GOV_MOTOR, 5, gain * 1000);
    }

    return gain;
}

static float govCalcTTA(void)
{
    float ttaAdd = 0;

    if (gov.useTTA) {
        float YAW = mixerGetInput(MIXER_IN_STABILIZED_YAW);
        float TTA = filterApply(&gov.ttaFilter, YAW) * getSpoolUpRatio() * gov.ttaGain;
        float headroom = 0;

        if (gov.govMode == GOV_MODE_EXTERNAL)
            headroom = gov.ttaLimit;
        else // GOV_MODE_ELECTRIC
            headroom = 2 * fmaxf(1.0f + gov.ttaLimit - gov.fullHeadSpeedRatio, 0);

        ttaAdd = constrainf(TTA, 0, headroom);

        DEBUG(TTA, 0, YAW * 1000);
        DEBUG(TTA, 1, TTA * 1000);
        DEBUG(TTA, 2, headroom * 1000);
        DEBUG(TTA, 3, ttaAdd * 1000);
    }

    return ttaAdd;
}

static float govCalcDynMinThrottle(float minThrottle)
{
    if (gov.useDynMinThrottle) {
        if (gov.motorRPMK > gov.fullHeadSpeed) {
            const float throttleEst = gov.targetHeadSpeed / gov.motorRPMK;
            minThrottle = fmaxf(minThrottle, throttleEst * gov.dynMinLevel); // GOV_DYN_MIN_THROTTLE_LIMIT

            DEBUG(GOV_MOTOR, 2, throttleEst);
            DEBUG(GOV_MOTOR, 3, minThrottle);
        }
    }

    return minThrottle;
}

static void govMotorConstantUpdate(void)
{
    if (gov.useMotorConstant) {
        if (gov.throttleOutput > 0.25f && gov.fullHeadSpeedRatio > 0.25f && gov.currentHeadSpeed > 100) {
            const float RPMK = gov.currentHeadSpeed / gov.throttleOutput;
            gov.motorRPMK = ewma1FilterApply(&gov.motorRPMKFilter, RPMK);

            DEBUG(GOV_MOTOR, 0, RPMK);
            DEBUG(GOV_MOTOR, 1, gov.motorRPMK);
        }
    }
}

static void govDataUpdate(void)
{
    // Calculate effective throttle
    govGetInputThrottle();

    // Assume motor[0]
    const float motorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    const float filteredRPM = filterApply(&gov.motorRPMFilter, motorRPM);

    // Calculate headspeed from filtered motor speed
    gov.currentHeadSpeed = filteredRPM * gov.mainGearRatio;

    // Calculate current HS vs FullHS ratio
    gov.fullHeadSpeedRatio = gov.currentHeadSpeed / gov.fullHeadSpeed;

    // Detect stuck motor / startup problem
    const bool rpmError = ((gov.fullHeadSpeedRatio < GOV_HS_INVALID_RATIO || motorRPM < 10) && gov.throttleOutput > GOV_HS_INVALID_THROTTLE);

    // Detect RPM glitches
    const bool rpmGlitch = (fabsf(motorRPM - filteredRPM) > gov.motorRPMGlitchDelta || motorRPM > gov.motorRPMGlitchLimit);

    // Error cases
    const bool noErrors = !rpmError && !rpmGlitch && !isForcedFallback();

    // Evaluate RPM signal quality
    if (noErrors && motorRPM > 0 && gov.fullHeadSpeedRatio > GOV_HS_DETECT_RATIO) {
        if (!gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = millis();
        }
    }
    else {
        if (gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = 0;
        }
    }

    // Headspeed is present if RPM is stable long enough
    gov.motorRPMGood = (gov.motorRPMDetectTime && cmp32(millis(),gov.motorRPMDetectTime) > GOV_HS_DETECT_DELAY);

    // Calculate request ratio (HS or throttle)
    if (gov.govMode == GOV_MODE_EXTERNAL)
        gov.requestRatio = (gov.throttleInput > 0) ? gov.throttleOutput / gov.throttleInput : 0;
    else
        gov.requestRatio = (gov.motorRPMGood) ? gov.currentHeadSpeed / gov.requestedHeadSpeed : 0;

    // Voltage drop compensation
    gov.voltageCompGain = govCalcVoltCompGain();

    // Tail Torque Assist
    gov.ttaAdd = govCalcTTA();

    // Normalized RPM error
    const float newError = (gov.targetHeadSpeed - gov.currentHeadSpeed) / gov.fullHeadSpeed + gov.ttaAdd;
    const float newDiff = difFilterApply(&gov.differentiator, newError);

    // Update PID terms
    gov.P = gov.K * gov.Kp * newError;
    gov.C = gov.K * gov.Ki * newError * pidGetDT();
    gov.D = gov.K * gov.Kd * newDiff;

    // All precomps
    gov.F = govCalcFeedforward();
}

static inline void govThrottleSlewControl(float minThrottle, float maxThrottle, float upRate, float downRate)
{
    if (gov.throttleInputOff) {
        gov.throttleOutput = 0;
        gov.targetHeadSpeed = 0;
    }
    else {
        // Limit input value range
        const float throttle = constrainf(gov.throttleInput, minThrottle, maxThrottle);

        // Limit change rate
        gov.throttleOutput = slewUpDownLimit(gov.throttleOutput, throttle, upRate, downRate);

        // Update headspeed target
        gov.targetHeadSpeed = gov.currentHeadSpeed;
    }
}

static inline void govThrottleBypass(float minThrottle, float maxThrottle)
{
    if (gov.throttleInputOff) {
        gov.throttleOutput = 0;
        gov.targetHeadSpeed = 0;
    }
    else {
        // Limit value range
        gov.throttleOutput = constrainf(gov.throttleInput, minThrottle, maxThrottle);

        // Update headspeed target
        gov.targetHeadSpeed = gov.currentHeadSpeed;
    }
}

static void govSpoolupInit(void)
{
    gov.hsSpoolupActive = false;
}

static void govSpoolupControl(float minThrottle, float maxThrottle, float maxRate)
{
    if (gov.hsSpoolupActive)
    {
        // PID limits
        gov.P = constrainf(gov.P, gov.minP, gov.maxP);
        gov.I = constrainf(gov.I, gov.minI, gov.maxI);
        gov.D = constrainf(gov.D, gov.minD, gov.maxD);

        // Reset needed
        if (gov.stateResetReq) {
            // Update gov.I from current throttle
            gov.I = gov.throttleOutput - (gov.P + gov.D);

            // Do it once when requested
            gov.stateResetReq = false;
        }

        // Governor PID sum
        gov.pidSum = gov.P + gov.I + gov.C + gov.D;

        // Apply gov.C if pidsum not saturated
        if ((gov.pidSum > minThrottle || gov.C > 0) && (gov.pidSum < maxThrottle || gov.C < 0))
            gov.I += gov.C;

        // Limit value range
        const float throttle = constrainf(gov.pidSum, minThrottle, maxThrottle);

        // Update output throttle (don't exceed rate)
        gov.throttleOutput = slewLimit(gov.throttleOutput, throttle, maxRate);

        // Update headspeed target
        gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, maxRate * gov.fullHeadSpeed * GOV_THROTTLE_HEADROOM);
    }
    else
    {
        // Limit value range
        const float throttle = constrainf(gov.throttleInput, minThrottle, maxThrottle);

        // Update output throttle
        gov.throttleOutput = slewLimit(gov.throttleOutput, throttle, maxRate);

        // Update headspeed target
        gov.targetHeadSpeed = gov.currentHeadSpeed;

        // Move to HS rampup, if applicable
        if (gov.usePidSpoolup && gov.throttleInput > gov.handoverThrottle && gov.throttleOutput > gov.handoverThrottle)
        {
            // Now HS control active
            gov.hsSpoolupActive = true;

            // Request reset
            gov.stateResetReq = true;
        }
    }
}

static void govPIDControl(float minThrottle, float maxThrottle, float maxRate)
{
    // PID limits
    gov.P = constrainf(gov.P, gov.minP, gov.maxP);
    gov.I = constrainf(gov.I, gov.minI, gov.maxI);
    gov.D = constrainf(gov.D, gov.minD, gov.maxD);
    gov.F = constrainf(gov.F, gov.minF, gov.maxF);

    // PID suspended
    if (isGovSuspend()) {
        gov.P = gov.C = gov.D = 0;
    }
    // Reset requested
    else if (gov.stateResetReq) {
        // Expected PID output
        const float pidTarget = gov.throttleOutput / gov.voltageCompGain;

        // Adjust I to prevent throttle jumps
        gov.I = pidTarget - (gov.P + gov.D + gov.F);

        // Do it once when requested
        gov.stateResetReq = false;
    }

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    const float throttle = gov.pidSum * gov.voltageCompGain;

    // update minThrottle
    minThrottle = govCalcDynMinThrottle(minThrottle);

    // Apply gov.C if pidsum not saturated
    if ((throttle > minThrottle || gov.C > 0) && (throttle < maxThrottle || gov.C < 0))
        gov.I += gov.C;

    // Limit value range
    gov.throttleOutput = constrainf(throttle, minThrottle, maxThrottle);

    // Estimate HS change speed
    const float hsRate = fmaxf(gov.fullHeadSpeed * GOV_THROTTLE_HEADROOM, gov.motorRPMK) * maxRate;

    // Update headspeed target
    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, hsRate);

    // Update motor data
    govMotorConstantUpdate();
}

static void govFallbackControl(float minThrottle, float maxThrottle, float __unused maxRate)
{
    // Precomp enabled in fallback
    if (gov.useFallbackPrecomp || gov.useTxPrecompCurve)
        gov.F = constrainf(gov.F, gov.minF, gov.maxF);
    else
        gov.F = 0;

    // Fallback "PID sum"
    gov.pidSum = gov.I + gov.F;

    // Generate throttle signal
    const float throttle = gov.pidSum * gov.voltageCompGain * gov.fallbackRatio;

    // Limit value range
    gov.throttleOutput = constrainf(throttle, minThrottle, maxThrottle);
}

static void govRecoveryInit(void)
{
    // Start recovery from the current headspeed
    gov.targetHeadSpeed = gov.currentHeadSpeed;

    // Set initial output throttle value
    if (gov.govMode == GOV_MODE_ELECTRIC) {
        // Headspeed is still reasonably high. Motor constant has been acquired.
        //  => calculate estimated throttle for the target headspeed
        if (gov.motorRPMGood && gov.fullHeadSpeedRatio > 0.25f && gov.motorRPMK > gov.fullHeadSpeed) {
            gov.throttleOutput = fminf(gov.targetHeadSpeed / gov.motorRPMK, gov.requestRatio);
        }
    }
}


/*
 * External throttle control (Ext.Gov or Throttle Curve)
 */

static void govUpdateExternalThrottle(void)
{
    float throttle = 0;

    switch (gov.state) {
        case GOV_STATE_THROTTLE_OFF:
            throttle = 0;
            break;
        case GOV_STATE_THROTTLE_HOLD:
        case GOV_STATE_THROTTLE_IDLE:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleStartupRate);
            break;
        case GOV_STATE_SPOOLUP:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
        case GOV_STATE_FALLBACK:
        case GOV_STATE_AUTOROTATION:
            throttle = slewLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleRecoveryRate);
            break;
        case GOV_STATE_DISABLED:
            throttle = gov.throttleInput;
            break;
        default:
            break;
    }

    gov.throttlePrevInput = throttle;

    if (gov.state == GOV_STATE_ACTIVE && gov.useTTA) {
        throttle += throttle * gov.ttaAdd;
    }

    gov.throttleOutput = gov.throttleInputOff ? 0 : fminf(throttle, gov.maxThrottle);
}

static void govUpdateExternalState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    // Governor bypassed / disabled
    else if (isGovDisabled()) {
        govChangeState(GOV_STATE_DISABLED);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleOutput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_SPOOLUP);
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GOV_STATE_THROTTLE_HOLD:
                if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.throttleHoldTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Follow the throttle, with a limited ramp up rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If throttle < handover, move to IDLE
            //  -- Once throttle level reached, move to ACTIVE
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If throttle < handover, move to AUTO or SPOOLING_UP
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (isAutorotation())
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_HOLD);
                }
                break;

            // Fallback not applicable here.
            case GOV_STATE_FALLBACK:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move to IDLE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_BAILOUT);
                else if (!isAutorotation())
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move back to AUTO
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Disabled: Direct throttle to output
            //  -- If governor enabled, move back to approriate state
            case GOV_STATE_DISABLED:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_SPOOLUP);
                else
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Get throttle
    govUpdateExternalThrottle();
}


/*
 * Electric motor speed control
 */

static inline void govEnterSpoolupState(void)
{
    govChangeState(GOV_STATE_SPOOLUP);
    govSpoolupInit();
}

static inline void govEnterRecoveryState(void)
{
    govChangeState(GOV_STATE_RECOVERY);
    govRecoveryInit();
}

static inline void govEnterBailoutState(void)
{
    govChangeState(GOV_STATE_BAILOUT);
    govRecoveryInit();
}

static void govUpdateGovernedThrottle(void)
{
    switch (gov.state) {
        case GOV_STATE_THROTTLE_OFF:
            gov.targetHeadSpeed = 0;
            gov.throttleOutput = 0;
            break;
        case GOV_STATE_THROTTLE_IDLE:
            govThrottleSlewControl(gov.idleThrottle, gov.handoverThrottle, gov.throttleStartupRate, gov.throttleSpooldownRate);
            break;
        case GOV_STATE_THROTTLE_HOLD:
            govThrottleSlewControl(gov.idleThrottle, gov.handoverThrottle, gov.throttleStartupRate, gov.throttleSpooldownRate);
            break;
        case GOV_STATE_SPOOLUP:
            govSpoolupControl(gov.minSpoolupThrottle, gov.maxSpoolupThrottle, gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
            govPIDControl(gov.minActiveThrottle, gov.maxThrottle, gov.throttleTrackingRate);
            break;
        case GOV_STATE_FALLBACK:
            govFallbackControl(gov.minActiveThrottle, gov.maxThrottle, gov.throttleTrackingRate);
            break;
        case GOV_STATE_AUTOROTATION:
            govThrottleSlewControl(gov.autoThrottle, gov.handoverThrottle, gov.throttleTrackingRate, gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            govThrottleSlewControl(gov.minSpoolupThrottle, gov.maxSpoolupThrottle, gov.throttleRecoveryRate, gov.throttleRecoveryRate);
            break;
        case GOV_STATE_DISABLED:
            govThrottleBypass(gov.idleThrottle, gov.maxThrottle);
            break;
        default:
            break;
    }
}

static void govUpdateGovernedState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    // Governor bypassed / disabled
    else if (isGovDisabled()) {
        govChangeState(GOV_STATE_DISABLED);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover and stable RPM, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterSpoolupState();
                break;

            // Throttle is moved from high to low. If it is a mistake, give a chance to recover
            //  -- When throttle returns high, move to RECOVERY
            //  -- When timer expires, move to OFF/IDLE
            case GOV_STATE_THROTTLE_HOLD:
                if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterRecoveryState();
                else if (govStateTime() > gov.throttleHoldTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Ramp up throttle until target is reached
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle < handover, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If no headspeed signal, move to FALLBACK
            //  -- If throttle < handover, move to AUTOROTATION or LOW
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_FALLBACK);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (isAutorotation())
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_HOLD);
                }
                break;

            // Fallback: No headspeed signal. Use curves.
            //  -- If NO throttle, move to HOLD
            //  -- If throttle below handover, change state
            //  -- If headspeed recovers, move to RECOVERY
            case GOV_STATE_FALLBACK:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (isAutorotation())
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_HOLD);
                }
                else if (gov.motorRPMGood)
                    govEnterRecoveryState();
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If no headspeed detected, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Throttle passthrough with ramp up limit
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If throttle > handover with RPM, move to BAILOUT
            //  -- Map throttle to motor output
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterBailoutState();
                else if (!isAutorotation())
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If no headspeed detected, move back to AUTOROTATION
            //  -- If throttle < handover, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Governor deactivated: Direct throttle to output
            //  -- If governor enabled, move back to approriate state
            case GOV_STATE_DISABLED:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterRecoveryState();
                else
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Update throttle and HS target
    govUpdateGovernedThrottle();
}


//// Loop update function

void governorUpdate(void)
{
    // Governor is active
    if (gov.govMode)
    {
        // Update internal data
        govDataUpdate();

        // Run state machine
        if (gov.govMode == GOV_MODE_EXTERNAL)
            govUpdateExternalState();
        else
            govUpdateGovernedState();

        // Set GOVERNOR debug data
        govDebugGovernor();
    }
    else
    {
        // Straight passthrough
        gov.throttleOutput = gov.throttleInput;
    }
}


//// Adjustment functions

int get_ADJUSTMENT_GOV_GAIN(void)
{
    return currentPidProfile->governor.gain;
}

void set_ADJUSTMENT_GOV_GAIN(int value)
{
    currentPidProfile->governor.gain = value;
    gov.K  = currentPidProfile->governor.gain / 100.0f;
}

int get_ADJUSTMENT_GOV_P_GAIN(void)
{
    return currentPidProfile->governor.p_gain;
}

void set_ADJUSTMENT_GOV_P_GAIN(int value)
{
    currentPidProfile->governor.p_gain = value;
    gov.Kp = currentPidProfile->governor.p_gain / 10.0f;
}

int get_ADJUSTMENT_GOV_I_GAIN(void)
{
    return currentPidProfile->governor.i_gain;
}

void set_ADJUSTMENT_GOV_I_GAIN(int value)
{
    currentPidProfile->governor.i_gain = value;
    gov.Ki = currentPidProfile->governor.i_gain / 10.0f;
}

int get_ADJUSTMENT_GOV_D_GAIN(void)
{
    return currentPidProfile->governor.d_gain;
}

void set_ADJUSTMENT_GOV_D_GAIN(int value)
{
    currentPidProfile->governor.d_gain = value;
    gov.Kd = currentPidProfile->governor.d_gain / 1000.0f;
}

int get_ADJUSTMENT_GOV_F_GAIN(void)
{
    return currentPidProfile->governor.f_gain;
}

void set_ADJUSTMENT_GOV_F_GAIN(int value)
{
    currentPidProfile->governor.f_gain = value;
    gov.Kf = currentPidProfile->governor.f_gain / 100.0f;
}

int get_ADJUSTMENT_GOV_TTA_GAIN(void)
{
    return currentPidProfile->governor.tta_gain;
}

void set_ADJUSTMENT_GOV_TTA_GAIN(int value)
{
    currentPidProfile->governor.tta_gain = value;
    governorInitTTA((const pidProfile_t *)currentPidProfile);
}

int get_ADJUSTMENT_GOV_CYCLIC_FF(void)
{
    return currentPidProfile->governor.cyclic_weight;
}

void set_ADJUSTMENT_GOV_CYCLIC_FF(int value)
{
    currentPidProfile->governor.cyclic_weight = value;
    gov.cyclicWeight = currentPidProfile->governor.cyclic_weight / 100.0f;
}

int get_ADJUSTMENT_GOV_COLLECTIVE_FF(void)
{
    return currentPidProfile->governor.collective_weight;
}

void set_ADJUSTMENT_GOV_COLLECTIVE_FF(int value)
{
    currentPidProfile->governor.collective_weight = value;
    gov.collectiveWeight = currentPidProfile->governor.collective_weight / 100.0f;
}

int get_ADJUSTMENT_GOV_IDLE_THROTTLE(void)
{
    return governorConfig()->gov_idle_throttle;
}

void set_ADJUSTMENT_GOV_IDLE_THROTTLE(int value)
{
    governorConfigMutable()->gov_idle_throttle = value;
    gov.idleThrottle = value / 1000.0f;
    gov.minSpoolupThrottle = gov.idleThrottle;
}

int get_ADJUSTMENT_GOV_AUTO_THROTTLE(void)
{
    return governorConfig()->gov_auto_throttle;
}

void set_ADJUSTMENT_GOV_AUTO_THROTTLE(int value)
{
    governorConfigMutable()->gov_auto_throttle = value;
    gov.autoThrottle = value / 1000.0f;
}


//// Init functions

void INIT_CODE validateAndFixGovernorConfig(void)
{
    pidProfile_t * pidProfile = currentPidProfile;

    while (governorConfig()->gov_wot_collective - governorConfig()->gov_idle_collective < 1) {
        governorConfigMutable()->gov_wot_collective = MIN(governorConfig()->gov_wot_collective + 1, 100);
        governorConfigMutable()->gov_idle_collective = MAX(governorConfig()->gov_idle_collective - 1, -100);
    }

    if (governorConfig()->gov_throttle_type != GOV_THROTTLE_NORMAL) {
        CLEAR_BIT(pidProfile->governor.flags,
            BIT(GOV_FLAG_TX_PRECOMP_CURVE) |
            BIT(GOV_FLAG_HS_ADJUSTMENT));
    }

    if (pidProfile->governor.flags & BIT(GOV_FLAG_BYPASS)) {
        CLEAR_BIT(pidProfile->governor.flags,
            BIT(GOV_FLAG_HS_ADJUSTMENT) |
            BIT(GOV_FLAG_FALLBACK_PRECOMP) |
            BIT(GOV_FLAG_PID_SPOOLUP) |
            BIT(GOV_FLAG_DYN_MIN_THROTTLE) |
            BIT(GOV_FLAG_VOLTAGE_COMP) |
            BIT(GOV_FLAG_AUTOROTATION));
    }
    if (pidProfile->governor.flags & BIT(GOV_FLAG_TX_PRECOMP_CURVE)) {
        CLEAR_BIT(pidProfile->governor.flags,
            BIT(GOV_FLAG_HS_ADJUSTMENT) |
            BIT(GOV_FLAG_FALLBACK_PRECOMP) |
            BIT(GOV_FLAG_PID_SPOOLUP));
    }
}

static void INIT_CODE governorInitTTA(const pidProfile_t *pidProfile)
{
    if (gov.govMode)
    {
        if (pidProfile->governor.tta_gain && !gov.useBypass) {
            gov.ttaGain = mixerRotationSign() * pidProfile->governor.tta_gain / -125.0f;
            gov.ttaLimit = pidProfile->governor.tta_limit / 100.0f;
            gov.useTTA = true;

            if (gov.govMode == GOV_MODE_ELECTRIC)
                gov.ttaGain /= gov.K * gov.Kp;
        }
        else {
            gov.ttaGain = 0;
            gov.ttaLimit = 0;
            gov.useTTA = false;
        }
    }
}

void INIT_CODE governorInitProfile(const pidProfile_t *pidProfile)
{
    if (gov.govMode)
    {
        gov.stateResetReq = true;

        gov.useBypass = (pidProfile->governor.flags & BIT(GOV_FLAG_BYPASS));
        gov.useSuspend = (pidProfile->governor.flags & BIT(GOV_FLAG_SUSPEND));
        gov.useFcThrottleCurve = (pidProfile->governor.flags & BIT(GOV_FLAG_FC_THROTTLE_CURVE)) && (gov.throttleType != GOV_THROTTLE_NORMAL);
        gov.useTxPrecompCurve = (pidProfile->governor.flags & BIT(GOV_FLAG_TX_PRECOMP_CURVE)) && (gov.throttleType == GOV_THROTTLE_NORMAL) && !gov.useBypass;
        gov.useHsAdjustment = (pidProfile->governor.flags & BIT(GOV_FLAG_HS_ADJUSTMENT)) && !gov.useTxPrecompCurve && (gov.throttleType == GOV_THROTTLE_NORMAL) && !gov.useBypass;
        gov.useFallbackPrecomp = (pidProfile->governor.flags & BIT(GOV_FLAG_FALLBACK_PRECOMP)) && !gov.useBypass;
        gov.usePidSpoolup = (pidProfile->governor.flags & BIT(GOV_FLAG_PID_SPOOLUP)) && !gov.useTxPrecompCurve && !gov.useBypass;
        gov.useMotorConstant = (gov.govMode == GOV_MODE_ELECTRIC) && !gov.useBypass;
        gov.useDynMinThrottle = (pidProfile->governor.flags & BIT(GOV_FLAG_DYN_MIN_THROTTLE)) && gov.useMotorConstant && !gov.useBypass;
        gov.useVoltageComp = (pidProfile->governor.flags & BIT(GOV_FLAG_VOLTAGE_COMP)) && (getBatteryVoltageSource() == VOLTAGE_METER_ADC) && gov.useMotorConstant && !gov.useBypass;
        gov.useAutoRotation = (pidProfile->governor.flags & BIT(GOV_FLAG_AUTOROTATION)) && !gov.useBypass;

        gov.K  = pidProfile->governor.gain / 100.0f;
        gov.Kp = pidProfile->governor.p_gain / 10.0f;
        gov.Ki = pidProfile->governor.i_gain / 10.0f;
        gov.Kd = pidProfile->governor.d_gain / 1000.0f;
        gov.Kf = pidProfile->governor.f_gain / 100.0f;

        gov.maxP = pidProfile->governor.p_limit / 100.0f;
        gov.maxI = pidProfile->governor.i_limit / 100.0f;
        gov.maxD = pidProfile->governor.d_limit / 100.0f;
        gov.maxF = pidProfile->governor.f_limit / 100.0f;

        gov.minP = -gov.maxP;
        gov.minI = -gov.maxI;
        gov.minD = -gov.maxD;
        gov.minF = 0;

        gov.maxThrottle = pidProfile->governor.max_throttle / 100.0f;

        gov.minActiveThrottle = pidProfile->governor.min_throttle / 100.0f;

        gov.minSpoolupThrottle = gov.idleThrottle;
        gov.maxSpoolupThrottle = gov.maxThrottle;

        gov.voltageCompGain = 1.0f;

        gov.fallbackRatio = (100 - constrain(pidProfile->governor.fallback_drop, 0, 50)) / 100.0f;

        gov.dynMinLevel = pidProfile->governor.dyn_min_level / 100.0f; // TDB remove

        gov.yawWeight = pidProfile->governor.yaw_weight / 100.0f;
        gov.cyclicWeight = pidProfile->governor.cyclic_weight / 100.0f;
        gov.collectiveWeight = pidProfile->governor.collective_weight / 100.0f;
        gov.collectiveCurve = pidProfile->governor.collective_curve;

        gov.fullHeadSpeed = constrainf(pidProfile->governor.headspeed, 100, 50000);
        gov.fullHeadSpeedRatio = 1.0f;
        gov.requestedHeadSpeed = gov.fullHeadSpeed;

        gov.motorRPMGlitchDelta = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_DELTA;
        gov.motorRPMGlitchLimit = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_LIMIT;

        governorInitTTA(pidProfile);
    }
}

static inline float govCalcRate(uint16_t param, uint16_t min, uint16_t max)
{
    if (param)
        return 10 * pidGetDT() / constrain(param,min,max);
    else
        return 0;
}

void INIT_CODE governorInit(const pidProfile_t *pidProfile)
{
    if (getMotorCount() > 0)
    {
        validateAndFixGovernorConfig();

        gov.state = GOV_STATE_THROTTLE_OFF;

        gov.govMode = governorConfig()->gov_mode;
        gov.throttleType = governorConfig()->gov_throttle_type;

        switch (gov.govMode)
        {
            case GOV_MODE_EXTERNAL:
                break;
            case GOV_MODE_ELECTRIC:
            case GOV_MODE_NITRO:
                if (!isMotorFastRpmSourceActive(0)) {
                    setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                    setArmingDisabled(ARMING_DISABLED_RPM_SIGNAL);
                    gov.govMode = GOV_MODE_NONE;
                }
                break;
            default:
                gov.govMode = GOV_MODE_NONE;
                break;
        }

        if (gov.govMode)
        {
            gov.mainGearRatio = getMainGearRatio();

            gov.throttleStartupRate  = govCalcRate(governorConfig()->gov_startup_time, 1, 600);
            gov.throttleSpoolupRate  = govCalcRate(governorConfig()->gov_spoolup_time, 1, 600);
            gov.throttleTrackingRate = govCalcRate(governorConfig()->gov_tracking_time, 1, 600);
            gov.throttleRecoveryRate = govCalcRate(governorConfig()->gov_recovery_time, 1, 600);
            gov.throttleSpooldownRate  = govCalcRate(governorConfig()->gov_spooldown_time, 1, 600);

            gov.throttleHoldTimeout  = governorConfig()->gov_throttle_hold_timeout * 100;

            gov.idleThrottle = governorConfig()->gov_idle_throttle / 1000.0f;
            gov.autoThrottle = governorConfig()->gov_auto_throttle / 1000.0f;

            gov.handoverThrottle = constrain(governorConfig()->gov_handover_throttle, 1, 100) / 100.0f;

            gov.wotCollective = constrain(governorConfig()->gov_wot_collective, -100, 100) / 100.0f;
            gov.idleCollective = constrain(governorConfig()->gov_idle_collective, -100, 100) / 100.0f;

            difFilterInit(&gov.differentiator, governorConfig()->gov_d_filter / 10.0f, gyro.targetRateHz);

            ewma1FilterInit(&gov.motorRPMKFilter, GOV_RPM_K_CUTOFF, gyro.targetRateHz);

            lowpassFilterInit(&gov.motorVoltageFilter, LPF_PT2, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.motorRPMFilter, LPF_PT2, governorConfig()->gov_rpm_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.ttaFilter, LPF_PT2, governorConfig()->gov_tta_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.precompFilter, LPF_PT2, governorConfig()->gov_ff_filter, gyro.targetRateHz, 0);

            governorInitProfile(pidProfile);
        }
    }
}
