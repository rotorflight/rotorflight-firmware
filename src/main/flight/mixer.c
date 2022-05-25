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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    static float motorRangeMinIncrease = 0;
    float currentThrottleInputRange = 0;
    
    {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN;
        currentThrottleInputRange = PWM_RANGE;

        motorRangeMax = mixerRuntime.motorOutputHigh;
        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

static void applyMixerAdjustmentLinear(float *motorMix) {
    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? motorMixRange : 1.0f;
    const float motorMixDelta = 0.5f * motorMixRange;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (throttle > 0.5f) {
            if (mixerConfig()->mixer_type == MIXER_LINEAR) {
                motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
            } else {
                motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + ABS(motorMix[i]), motorMix[i] - ABS(motorMix[i]));
            }
        }
        motorMix[i] /= motorMixNormalizationFactor;
    }
}

static void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax) {
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
    } else {
        if (throttle > 0.5f) {
            throttle = constrainf(throttle, -motorMixMin, 1.0f - motorMixMax);
        }
    }
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle
    mixerThrottle = throttle;

    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {

        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    motorMixRange = motorMixMax - motorMixMin;
    if (mixerConfig()->mixer_type > MIXER_LEGACY) {
        applyMixerAdjustmentLinear(motorMix);
    } else {
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax);
    }

    if (ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}
