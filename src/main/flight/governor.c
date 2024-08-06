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


// Throttle mapping in IDLE state
#define GOV_THROTTLE_OFF_LIMIT          0.05f

// Minimum throttle output from PI algorithms
#define GOV_MIN_THROTTLE_OUTPUT         0.05f

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


//// Internal Data

typedef struct {

    // Governor mode (GM_ enum)
    govMode_e       mode;

    // State machine
    govState_e      state;
    timeMs_t        stateEntryTime;

    // Output throttle
    float           throttle;

    // Input throttle
    float           throttleInput;
    bool            throttleInputLow;

    // Earlier input throttle
    float           throttlePrev;

    // Maximum allowed throttle
    float           maxThrottle;

    // Throttle handover level
    float           maxIdleThrottle;

    // Current headspeed
    float           actualHeadSpeed;

    // Headspeed config
    float           fullHeadSpeed;
    float           targetHeadSpeed;
    float           requestedHeadSpeed;

    // Proportial headspeeds
    float           fullHeadSpeedRatio;
    float           requestRatio;

    // Main gear ratio (motor/head)
    float           mainGearRatio;

    // RPM Signal Flags
    bool            motorRPMError;
    bool            motorRPMPresent;

    // RPM filter & detection
    float           motorRPM;
    filter_t        motorRPMFilter;
    float           motorRPMGlitchDelta;
    float           motorRPMGlitchLimit;
    uint32_t        motorRPMDetectTime;

    // Battery voltage & current
    float           motorVoltage;
    filter_t        motorVoltageFilter;
    float           motorCurrent;
    filter_t        motorCurrentFilter;

    // Nominal battery voltage
    float           nominalVoltage;

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

    // Feedforward
    float           yawWeight;
    float           cyclicWeight;
    float           collectiveWeight;
    float           FFExponent;
    filter_t        FFFilter;

    // Tail Torque Assist
    float           TTAAdd;
    float           TTAGain;
    float           TTALimit;
    filter_t        TTAFilter;

    // Autorotation
    bool            autoEnabled;
    long            autoMinEntry;
    long            autoTimeout;

    // Timeouts
    long            zeroThrottleTimeout;
    long            lostHeadspeedTimeout;

    // Headspeed change rates
    float           headSpeedSpoolupRate;
    float           headSpeedBailoutRate;
    float           headSpeedRecoveryRate;
    float           headSpeedTrackingRate;

    // Throttle change rates
    float           throttleStartupRate;
    float           throttleSpoolupRate;
    float           throttleBailoutRate;
    float           throttleRecoveryRate;
    float           throttleTrackingRate;

} govData_t;

static FAST_DATA_ZERO_INIT govData_t gov;


//// Handler functions

typedef void  (*govVoidFn)(void);
typedef float (*govFloatFn)(void);

static FAST_DATA_ZERO_INIT govVoidFn   govStateUpdate;

static FAST_DATA_ZERO_INIT govVoidFn   govSpoolupInit;
static FAST_DATA_ZERO_INIT govFloatFn  govSpoolupCalc;

static FAST_DATA_ZERO_INIT govVoidFn   govActiveInit;
static FAST_DATA_ZERO_INIT govFloatFn  govActiveCalc;


//// Prototypes

static void govPIDInit(void);
static void govMode1Init(void);
static void govMode2Init(void);

static float govPIDControl(void);
static float govMode1Control(void);
static float govMode2Control(void);

static void governorUpdateState(void);
static void governorUpdatePassthrough(void);



//// Access functions

int getGovernorState(void)
{
    return gov.state;
}

float getGovernorOutput(void)
{
    return gov.throttle;
}

float getTTAIncrease(void)
{
    return gov.TTAAdd;
}

float getFullHeadSpeedRatio(void)
{
    if (gov.mode > GM_PASSTHROUGH) {
        switch (gov.state)
        {
            case GS_ACTIVE:
            case GS_RECOVERY:
            case GS_SPOOLING_UP:
            case GS_ZERO_THROTTLE:
            case GS_LOST_HEADSPEED:
            case GS_AUTOROTATION:
            case GS_AUTOROTATION_BAILOUT:
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

    if (gov.mode > GM_OFF) {
        switch (gov.state)
        {
            case GS_THROTTLE_OFF:
            case GS_THROTTLE_IDLE:
            case GS_ZERO_THROTTLE:
            case GS_AUTOROTATION:
                return 0;

            case GS_ACTIVE:
            case GS_RECOVERY:
            case GS_LOST_HEADSPEED:
            case GS_AUTOROTATION_BAILOUT:
                return 1.0f;

            case GS_SPOOLING_UP:
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

    if (gov.mode > GM_OFF) {
        switch (gov.state)
        {
            case GS_ACTIVE:
            case GS_AUTOROTATION:
            case GS_AUTOROTATION_BAILOUT:
                return true;

            case GS_RECOVERY:
            case GS_SPOOLING_UP:
            case GS_LOST_HEADSPEED:
                return (gov.throttle > 0.333f);

            case GS_THROTTLE_OFF:
            case GS_THROTTLE_IDLE:
            case GS_ZERO_THROTTLE:
                return false;
        }
        return false;
    }
    else {
        return (gov.throttle > 0.333f);
    }

    return false;
}


//// Internal functions

static inline float idleMap(float throttle)
{
    // Map throttle in IDLE state (N is maxIdleThrottle)
    //     0%..5%    => 0%
    //     5%..N%    => 0%..N%
    //     >N%       => N%
    throttle = (throttle - GOV_THROTTLE_OFF_LIMIT) * gov.maxIdleThrottle /
        (gov.maxIdleThrottle - GOV_THROTTLE_OFF_LIMIT);

    return constrainf(throttle, 0, gov.maxIdleThrottle);
}

static inline float angleDrag(float angle)
{
    return pow_approx(angle, gov.FFExponent);
}

static inline void govChangeState(govState_e futureState)
{
    gov.state = futureState;
    gov.stateEntryTime = millis();
}

static inline long govStateTime(void)
{
    return cmp32(millis(),gov.stateEntryTime);
}

static void govDebugStats(void)
{
    DEBUG(GOVERNOR, 0, gov.requestedHeadSpeed);
    DEBUG(GOVERNOR, 1, gov.targetHeadSpeed);
    DEBUG(GOVERNOR, 2, gov.actualHeadSpeed);
    DEBUG(GOVERNOR, 3, gov.pidSum * 1000);
    DEBUG(GOVERNOR, 4, gov.P * 1000);
    DEBUG(GOVERNOR, 5, gov.I * 1000);
    DEBUG(GOVERNOR, 6, gov.D * 1000);
    DEBUG(GOVERNOR, 7, gov.F * 1000);
}

static void govUpdateInputs(void)
{
    // Update throttle state
    gov.throttleInput = getThrottle();
    gov.throttleInputLow = (getThrottleStatus() == THROTTLE_LOW);

    // Assume motor[0]
    gov.motorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    float filteredRPM = filterApply(&gov.motorRPMFilter, gov.motorRPM);

    // Calculate headspeed from filtered motor speed
    gov.actualHeadSpeed = filteredRPM * gov.mainGearRatio;

    // Calculate HS vs FullHS ratio
    gov.fullHeadSpeedRatio = gov.actualHeadSpeed / gov.fullHeadSpeed;

    // Detect stuck motor / startup problem
    bool rpmError = ((gov.fullHeadSpeedRatio < GOV_HS_INVALID_RATIO || gov.motorRPM < 10) && gov.throttle > GOV_HS_INVALID_THROTTLE);

    // Detect RPM glitches
    bool rpmGlitch = (gov.motorRPM - filteredRPM > gov.motorRPMGlitchDelta || gov.motorRPM > gov.motorRPMGlitchLimit);

    // Error cases
    gov.motorRPMError = rpmError || rpmGlitch;

    // Evaluate RPM signal quality
    if (!gov.motorRPMError && gov.motorRPM > 0 && gov.fullHeadSpeedRatio > GOV_HS_DETECT_RATIO) {
        if (!gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = millis();
        }
    } else {
        if (gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = 0;
        }
    }

    // Headspeed is present if RPM is stable long enough
    gov.motorRPMPresent = (gov.motorRPMDetectTime && cmp32(millis(),gov.motorRPMDetectTime) > GOV_HS_DETECT_DELAY);

    // Battery state - zero when battery unplugged
    gov.nominalVoltage = getBatteryCellCount() * GOV_NOMINAL_CELL_VOLTAGE;

    // Voltage & current filters
    gov.motorVoltage = filterApply(&gov.motorVoltageFilter, getBatteryVoltageSample() * 0.01f);
    gov.motorCurrent = filterApply(&gov.motorCurrentFilter, getBatteryCurrentSample() * 0.01f);
}

static void govUpdateData(void)
{
    // Update headspeed target
    gov.requestedHeadSpeed = gov.throttleInput * gov.fullHeadSpeed;

    // Calculate request ratio (HS or throttle)
    if (gov.throttleInput > 0) {
        if (gov.mode > GM_PASSTHROUGH)
            gov.requestRatio = gov.actualHeadSpeed / gov.requestedHeadSpeed;
        else
            gov.requestRatio = gov.throttle / gov.throttleInput;
    }
    else {
        gov.requestRatio = 0;
    }

    // Calculate feedforward from collective deflection
    float collectiveFF = gov.collectiveWeight * getCollectiveDeflectionAbs();

    // Calculate feedforward from cyclic deflection
    float cyclicFF = gov.cyclicWeight * getCyclicDeflection();

    // Calculate feedforward from yaw deflection
    float yawFF = gov.yawWeight * getYawDeflectionAbs();

    // Angle-of-attack vs. FeedForward curve
    float totalFF = angleDrag(collectiveFF + cyclicFF) + angleDrag(yawFF);

    // Filtered FeedForward
    totalFF = filterApply(&gov.FFFilter, totalFF);

    // Tail Torque Assist
    if (mixerMotorizedTail() && gov.TTAGain != 0) {
        float YAW = mixerGetInput(MIXER_IN_STABILIZED_YAW);
        float TTA = filterApply(&gov.TTAFilter, YAW) * getSpoolUpRatio() * gov.TTAGain;
        float headroom = 0;

        if (gov.mode > GM_PASSTHROUGH)
            headroom = 2 * fmaxf(1.0f + gov.TTALimit - gov.fullHeadSpeedRatio, 0);
        else
            headroom = gov.TTALimit;

        gov.TTAAdd = constrainf(TTA, 0, headroom);

        DEBUG(TTA, 0, YAW * 1000);
        DEBUG(TTA, 1, TTA * 1000);
        DEBUG(TTA, 2, headroom * 1000);
        DEBUG(TTA, 3, gov.TTAAdd * 1000);

        DEBUG(TTA, 4, gov.P * 1000);
        DEBUG(TTA, 5, gov.I * 1000);
        DEBUG(TTA, 6, gov.pidSum * 1000);
        DEBUG(TTA, 7, gov.targetHeadSpeed);
    }
    else {
        gov.TTAAdd = 0;
    }

    // Normalized RPM error
    float newError = (gov.targetHeadSpeed - gov.actualHeadSpeed) / gov.fullHeadSpeed + gov.TTAAdd;

    // Update PIDF terms
    gov.P = gov.K * gov.Kp * newError;
    gov.C = gov.K * gov.Ki * newError * pidGetDT();
    gov.D = gov.K * gov.Kd * difFilterApply(&gov.differentiator, newError);
    gov.F = gov.K * gov.Kf * totalFF;
}


/*
 * Throttle passthrough with rampup limits and extra stats.
 */

static void governorUpdatePassthrough(void)
{
    float throttleInput = fminf(gov.throttleInput, gov.maxThrottle);
    float govPrev = gov.throttlePrev;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GS_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = govPrev = 0;
                if (!gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleStartupRate);
                if (gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttleInput > gov.maxIdleThrottle)
                    govChangeState(GS_SPOOLING_UP);
                break;

            // Follow the throttle, with a limited rampup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < 20%, stay in spoolup
            //  -- Once throttle >20% and not ramping up, move to ACTIVE
            case GS_SPOOLING_UP:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleSpoolupRate);
                if (gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= throttleInput)
                    govChangeState(GS_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle <20%, move to AUTO or SPOOLING_UP
            case GS_ACTIVE:
                govPrev = slewLimit(govPrev, throttleInput, gov.throttleTrackingRate);
                govMain = govPrev + govPrev * gov.TTAAdd;
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (throttleInput < gov.maxIdleThrottle) {
                    if (gov.autoEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GS_AUTOROTATION);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                }
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GS_ZERO_THROTTLE:
                govMain = govPrev = 0;
                if (!gov.throttleInputLow)
                    govChangeState(GS_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            case GS_AUTOROTATION:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleTrackingRate);
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (throttleInput > gov.maxIdleThrottle)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move back to AUTO
            case GS_AUTOROTATION_BAILOUT:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleBailoutRate);
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_AUTOROTATION);
                else if (govMain >= throttleInput)
                    govChangeState(GS_ACTIVE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move to IDLE
            case GS_RECOVERY:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleRecoveryRate);
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= throttleInput)
                    govChangeState(GS_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                break;
        }
    }

    // Update output variables
    gov.throttle = govMain;
    gov.throttlePrev = govPrev;

    // Set debug
    govDebugStats();
}


/*
 * State machine for governed speed control
 */

static inline void govEnterSpoolupState(govState_e state)
{
    govChangeState(state);
    govSpoolupInit();
}

static inline void govEnterActiveState(govState_e state)
{
    govChangeState(state);
    govActiveInit();
}

static void governorUpdateState(void)
{
    float govPrev = gov.throttle;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || (getBatteryCellCount() == 0 && gov.mode == GM_MODE2)) {
        govChangeState(GS_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                gov.targetHeadSpeed = 0;
                if (!gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- Map throttle to motor output
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleStartupRate);
                gov.targetHeadSpeed = gov.actualHeadSpeed;
                if (gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GS_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle <20%, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            //  -- If NO throttle, move to THROTTLE_OFF
            case GS_SPOOLING_UP:
                govMain = govPrev;
                if (gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (gov.actualHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > 0.95f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedSpoolupRate);
                }
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed signal, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to AUTOROTATION or IDLE
            case GS_ACTIVE:
                govMain = govPrev;
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle) {
                    if (gov.autoEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GS_AUTOROTATION);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                } else {
                    govMain = govActiveCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedTrackingRate);
                }
                break;

            // Throttle is off or low. If it is a mistake, give a chance to recover
            //  -- When throttle and *headspeed* returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GS_ZERO_THROTTLE:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleTrackingRate);
                gov.targetHeadSpeed = gov.actualHeadSpeed;
                if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GS_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // No headspeed signal. Ramp down throttle slowly.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GS_LOST_HEADSPEED:
                govMain = slewLimit(govPrev, idleMap(gov.throttleInput), gov.throttleSpoolupRate);
                gov.targetHeadSpeed = gov.actualHeadSpeed;
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (gov.motorRPMPresent) {
                    if (gov.throttleInput > gov.maxIdleThrottle)
                        govEnterSpoolupState(GS_RECOVERY);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                }
                else if (govStateTime() > gov.lostHeadspeedTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // Throttle passthrough with rampup limit
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GS_AUTOROTATION:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleTrackingRate);
                gov.targetHeadSpeed = gov.actualHeadSpeed;
                if (gov.throttleInputLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GS_AUTOROTATION_BAILOUT:
                govMain = govPrev;
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_AUTOROTATION);
                else if (gov.actualHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > gov.maxThrottle * 0.99f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedBailoutRate);
                }
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GS_RECOVERY:
                govMain = govPrev;
                if (gov.throttleInputLow)
                    govChangeState(GS_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (gov.actualHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > gov.maxThrottle * 0.99f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedRecoveryRate);
                }
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                break;
        }
    }

    // Update output variables
    gov.throttle = fminf(govMain, gov.maxThrottle);

    // Set debug
    govDebugStats();
}


/*
 * Standard PID controller
 */

static void govPIDInit(void)
{
    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);

    // Use gov.I to reach the target
    gov.I = gov.throttle - gov.P;

    // Limited range
    gov.I = constrainf(gov.I, 0, 0.95f);
}

static float govPIDControl(void)
{
    float output;

    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);
    gov.I = constrainf(gov.I,      0, 0.95f);
    gov.D = constrainf(gov.D, -0.25f, 0.25f);

    // Governor PID sum
    gov.pidSum = gov.P + gov.I + gov.D + gov.C;

    // Generate throttle signal
    output = gov.pidSum;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxThrottle && gov.C > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, gov.maxThrottle);

    return output;
}


/*
 * Mode1: PIDF controller
 */

static void govMode1Init(void)
{
    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);
    gov.D = constrainf(gov.D, -0.25f, 0.25f);
    gov.F = constrainf(gov.F,      0, 0.50f);

    // Use gov.I to reach the target
    gov.I = gov.throttle - (gov.P + gov.D + gov.F);

    // Limited range
    gov.I = constrainf(gov.I, 0, 0.95f);
}

static float govMode1Control(void)
{
    float output;

    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);
    gov.I = constrainf(gov.I,      0, 0.95f);
    gov.D = constrainf(gov.D, -0.25f, 0.25f);
    gov.F = constrainf(gov.F,      0, 0.50f);

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    output = gov.pidSum;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxThrottle && gov.C > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, gov.maxThrottle);

    return output;
}


/*
 * Mode2: PIDF with Battery voltage compensation
 */

static void govMode2Init(void)
{
    // Normalized battery voltage
    float pidGain = gov.nominalVoltage / gov.motorVoltage;

    // Expected PID output
    float pidTarget = gov.throttle / pidGain;

    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);
    gov.D = constrainf(gov.D, -0.25f, 0.25f);
    gov.F = constrainf(gov.F,      0, 0.50f);

    // Use gov.I to reach the target
    gov.I = pidTarget - (gov.P + gov.D + gov.F);

    // Limited range
    gov.I = constrainf(gov.I, 0, 0.95f);
}

static float govMode2Control(void)
{
    float output;

    // Normalized battery voltage
    float pidGain = gov.nominalVoltage / gov.motorVoltage;

    // PID limits
    gov.P = constrainf(gov.P, -0.25f, 0.25f);
    gov.I = constrainf(gov.I,      0, 0.95f);
    gov.D = constrainf(gov.D, -0.25f, 0.25f);
    gov.F = constrainf(gov.F,      0, 0.50f);

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    output = gov.pidSum * pidGain;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxThrottle && gov.C > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, gov.maxThrottle);

    return output;
}


static inline float govCalcRate(uint16_t param, uint16_t min, uint16_t max)
{
    if (param)
        return 10 * pidGetDT() / constrain(param,min,max);
    else
        return 0;
}


//// Interface functions

void governorUpdate(void)
{
    // Calculate inputs even if governor not active
    govUpdateInputs();

    // Governor is active
    if (gov.mode)
    {
        // Update internal state data
        govUpdateData();

        // Run state machine
        govStateUpdate();
    }
    else
    {
        // Straight passthrough
        if (gov.throttleInputLow)
            gov.throttle = 0;
        else
            gov.throttle = gov.throttleInput;
    }
}

void governorInitProfile(const pidProfile_t *pidProfile)
{
    if (getMotorCount() > 0)
    {
        gov.K  = pidProfile->governor.gain / 100.0f;
        gov.Kp = pidProfile->governor.p_gain / 10.0f;
        gov.Ki = pidProfile->governor.i_gain / 10.0f;
        gov.Kd = pidProfile->governor.d_gain / 1000.0f;
        gov.Kf = pidProfile->governor.f_gain / 100.0f;

        gov.TTAGain   = mixerRotationSign() * pidProfile->governor.tta_gain / -125.0f;
        gov.TTALimit  = pidProfile->governor.tta_limit / 100.0f;

        if (gov.mode >= GM_STANDARD)
            gov.TTAGain /= gov.K * gov.Kp;

        gov.yawWeight = pidProfile->governor.yaw_ff_weight / 100.0f;
        gov.cyclicWeight = pidProfile->governor.cyclic_ff_weight / 100.0f;
        gov.collectiveWeight = pidProfile->governor.collective_ff_weight / 100.0f;

        gov.FFExponent = pidProfile->governor.ff_exponent / 100.0f;

        gov.maxThrottle = pidProfile->governor.max_throttle / 100.0f;

        gov.fullHeadSpeed = constrainf(pidProfile->governor.headspeed, 100, 50000);

        gov.headSpeedSpoolupRate  = gov.throttleSpoolupRate  * gov.fullHeadSpeed;
        gov.headSpeedTrackingRate = gov.throttleTrackingRate * gov.fullHeadSpeed;
        gov.headSpeedRecoveryRate = gov.throttleRecoveryRate * gov.fullHeadSpeed;
        gov.headSpeedBailoutRate  = gov.throttleBailoutRate  * gov.fullHeadSpeed;

        gov.motorRPMGlitchDelta = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_DELTA;
        gov.motorRPMGlitchLimit = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_LIMIT;
    }
}

void governorInit(const pidProfile_t *pidProfile)
{
    // Must have at least one motor
    if (getMotorCount() > 0)
    {
        gov.mode  = governorConfig()->gov_mode;
        gov.state = GS_THROTTLE_OFF;

        // Check RPM input
        if (gov.mode >= GM_STANDARD) {
            if (!isMotorFastRpmSourceActive(0)) {
                setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                gov.mode = GM_OFF;
            }
        }

        // Check Voltage input
        if (gov.mode >= GM_MODE2) {
            if (!isBatteryVoltageConfigured()) {
                setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                gov.mode = GM_OFF;
            }
        }

        // Mode specific handler functions
        switch (gov.mode) {
            case GM_PASSTHROUGH:
                govStateUpdate = governorUpdatePassthrough;
                govSpoolupInit = NULL;
                govSpoolupCalc = NULL;
                govActiveInit  = NULL;
                govActiveCalc  = NULL;
                break;
            case GM_STANDARD:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govPIDInit;
                govActiveCalc  = govPIDControl;
                break;
            case GM_MODE1:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govMode1Init;
                govActiveCalc  = govMode1Control;
                break;
            case GM_MODE2:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govMode2Init;
                govActiveCalc  = govMode2Control;
                break;
            default:
                break;
        }

        gov.mainGearRatio = getMainGearRatio();

        gov.autoEnabled  = (governorConfig()->gov_autorotation_timeout > 0 &&
                           governorConfig()->gov_autorotation_bailout_time > 0);
        gov.autoTimeout  = governorConfig()->gov_autorotation_timeout * 100;
        gov.autoMinEntry = governorConfig()->gov_autorotation_min_entry_time * 100;

        gov.throttleStartupRate  = govCalcRate(governorConfig()->gov_startup_time, 1, 600);
        gov.throttleSpoolupRate  = govCalcRate(governorConfig()->gov_spoolup_time, 1, 600);
        gov.throttleTrackingRate = govCalcRate(governorConfig()->gov_tracking_time, 1, 100);
        gov.throttleRecoveryRate = govCalcRate(governorConfig()->gov_recovery_time, 1, 100);
        gov.throttleBailoutRate  = govCalcRate(governorConfig()->gov_autorotation_bailout_time, 1, 100);

        gov.headSpeedSpoolupRate  = gov.throttleSpoolupRate  * gov.fullHeadSpeed;
        gov.headSpeedTrackingRate = gov.throttleTrackingRate * gov.fullHeadSpeed;
        gov.headSpeedRecoveryRate = gov.throttleRecoveryRate * gov.fullHeadSpeed;
        gov.headSpeedBailoutRate  = gov.throttleBailoutRate  * gov.fullHeadSpeed;

        gov.zeroThrottleTimeout  = governorConfig()->gov_zero_throttle_timeout * 100;
        gov.lostHeadspeedTimeout = governorConfig()->gov_lost_headspeed_timeout * 100;

        gov.maxIdleThrottle = constrain(governorConfig()->gov_handover_throttle, 10, 50) / 100.0f;

        const float diff_cutoff = governorConfig()->gov_rpm_filter ?
            constrainf(governorConfig()->gov_rpm_filter, 1, 50) : 20;

        difFilterInit(&gov.differentiator, diff_cutoff, gyro.targetRateHz);

        lowpassFilterInit(&gov.motorVoltageFilter, LPF_DAMPED, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.motorCurrentFilter, LPF_DAMPED, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.motorRPMFilter, LPF_DAMPED, governorConfig()->gov_rpm_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.TTAFilter, LPF_DAMPED, governorConfig()->gov_tta_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.FFFilter, LPF_DAMPED, governorConfig()->gov_ff_filter, gyro.targetRateHz, 0);

        governorInitProfile(pidProfile);
    }
}
