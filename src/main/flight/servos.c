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

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;
    servoConfig->servo_lowpass_freq = 0;

    for (unsigned servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servoConfig->dev.ioTags[servoIndex] = timerioTagGetByUsage(TIM_USE_SERVO, servoIndex);
    }
}

PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED
        );
    }
}

int16_t servo[MAX_SUPPORTED_SERVOS];

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];


int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return rcData[channelToForwardFrom];
    }

    return servoParams(servoIndex)->middle;
}

int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoParams(servoIndex)->reversedSources & (1 << inputSource)) {
        return -1;
    } else {
        return 1;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers(i)->rate == 0) {
            break;
        }
        currentServoMixer[i] = *customServoMixers(i);
        servoRuleCount++;
    }
}

void servosInit(void)
{
    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    loadCustomServoMixer();
}

// Write and keep track of written servos

static uint32_t servoWritten;

STATIC_ASSERT(sizeof(servoWritten) * 8 >= MAX_SUPPORTED_SERVOS, servoWritten_is_too_small);

static void servoTable(void);
static void filterServos(void);

void writeServos(void)
{
    servoTable();
    filterServos();

    uint8_t servoIndex = 0;

    // Scan servos and write those marked forwarded and not written yet
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
        if ((channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) && !(servoWritten & (1 << i))) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
    }
}

void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];

    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    input[INPUT_STABILIZED_ROLL] = pidData[FD_ROLL].Sum * PID_SERVO_MIXER_SCALING;
    input[INPUT_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * PID_SERVO_MIXER_SCALING;
    input[INPUT_STABILIZED_YAW] = pidData[FD_YAW].Sum * PID_SERVO_MIXER_SCALING;

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig()->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig()->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig()->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig()->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig()->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig()->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig()->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig()->midrc;

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = 0;
    }

    // mix servos according to rules
    for (int i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoParams(target)->max - servoParams(target)->min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }

            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}


static void servoTable(void)
{
    servoMixer();

    // constrain servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max); // limit the values
    }
}

bool isMixerUsingServos(void)
{
    return true;
}

static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

void servosFilterInit(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            biquadFilterInitLPF(&servoFilter[servoIdx], servoConfig()->servo_lowpass_freq, targetPidLooptime);
        }
    }

}
static void filterServos(void)
{
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif // USE_SERVOS
