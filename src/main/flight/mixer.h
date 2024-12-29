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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "pg/mixer.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/motors.h"


/** Mixer constants **/

#define MIXER_OUTPUT_COUNT    (1 + MAX_SUPPORTED_SERVOS + MAX_SUPPORTED_MOTORS)

#define MIXER_SERVO_OFFSET    1
#define MIXER_MOTOR_OFFSET    (MIXER_SERVO_OFFSET + MAX_SUPPORTED_SERVOS)

#define MIXER_RATE_MIN       -10000
#define MIXER_RATE_MAX        10000

#define MIXER_WEIGHT_MIN     -10000
#define MIXER_WEIGHT_MAX      10000

#define MIXER_INPUT_MIN      -2500
#define MIXER_INPUT_MAX       2500

#define MIXER_OVERRIDE_MIN   -2500
#define MIXER_OVERRIDE_MAX    2500
#define MIXER_OVERRIDE_OFF    (MIXER_OVERRIDE_MAX + 1)
#define MIXER_OVERRIDE_PASSTHROUGH  (MIXER_OVERRIDE_MAX + 2)

#define MIXER_SATURATION_TIME 5


/** Interface function **/

void mixerInit(void);
void mixerInitConfig(void);

void validateAndFixMixerConfig(void);

void mixerUpdate(timeUs_t currentTimeUs);

float mixerGetInput(uint8_t index);
float mixerGetInputHistory(uint8_t index, uint16_t delay);

float mixerGetOutput(uint8_t index);

float getCyclicDeflection(void);

bool mixerSaturated(uint8_t index);
void mixerSaturateInput(uint8_t index);
void mixerSaturateOutput(uint8_t index);

int16_t mixerGetOverride(uint8_t index);
int16_t mixerSetOverride(uint8_t index, int16_t value);

bool mixerIsCyclicServo(uint8_t index);


/** Inline functions **/

static inline float mixerGetServoOutput(uint8_t index)
{
    return mixerGetOutput(MIXER_SERVO_OFFSET + index);
}

static inline float mixerGetMotorOutput(uint8_t index)
{
    return mixerGetOutput(MIXER_MOTOR_OFFSET + index);
}

static inline float getYawDeflection(void)
{
    return mixerGetInput(MIXER_IN_STABILIZED_YAW);
}

static inline float getYawDeflectionAbs(void)
{
    return fabsf(mixerGetInput(MIXER_IN_STABILIZED_YAW));
}

static inline float getCollectiveDeflection(void)
{
    return mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE);
}

static inline float getCollectiveDeflectionAbs(void)
{
    return fabsf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE));
}

static inline float mixerGetThrottle(void)
{
    return mixerGetInput(MIXER_IN_RC_COMMAND_THROTTLE);
}

static inline bool pidAxisSaturated(uint8_t index)
{
    return mixerSaturated(MIXER_IN_STABILIZED_ROLL + index);
}

static inline void mixerSaturateServoOutput(uint8_t index)
{
    mixerSaturateOutput(index + MIXER_SERVO_OFFSET);
}

static inline void mixerSaturateMotorOutput(uint8_t index)
{
    mixerSaturateOutput(index + MIXER_MOTOR_OFFSET);
}

static inline int mixerRotationSign()
{
    return (mixerConfig()->main_rotor_dir == DIR_CW) ? -1 : 1;
}

static inline bool mixerMotorizedTail(void)
{
    return (mixerConfig()->tail_rotor_mode != TAIL_MODE_VARIABLE);
}

static inline bool mixerIsTailMode(int mode)
{
    return (mixerConfig()->tail_rotor_mode == mode);
}
