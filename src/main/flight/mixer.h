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

#include "pg/pg.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/governor.h"
#include "flight/rpm_filter.h"


typedef enum {
    DIR_CW,
    DIR_CCW,
} dir_e;

typedef enum {
    TAIL_MODE_VARIABLE,
    TAIL_MODE_MOTORIZED,
} tail_mode_e;

enum {
    MIXER_IN_NONE = 0,
    MIXER_IN_STABILIZED_ROLL,
    MIXER_IN_STABILIZED_PITCH,
    MIXER_IN_STABILIZED_YAW,
    MIXER_IN_STABILIZED_COLLECTIVE,
    MIXER_IN_STABILIZED_THROTTLE,
    MIXER_IN_RC_COMMAND_ROLL,
    MIXER_IN_RC_COMMAND_PITCH,
    MIXER_IN_RC_COMMAND_YAW,
    MIXER_IN_RC_COMMAND_COLLECTIVE,
    MIXER_IN_RC_COMMAND_THROTTLE,
    MIXER_IN_RC_CHANNEL_ROLL,
    MIXER_IN_RC_CHANNEL_PITCH,
    MIXER_IN_RC_CHANNEL_YAW,
    MIXER_IN_RC_CHANNEL_COLLECTIVE,
    MIXER_IN_RC_CHANNEL_THROTTLE,
    MIXER_IN_RC_CHANNEL_AUX1,
    MIXER_IN_RC_CHANNEL_AUX2,
    MIXER_IN_RC_CHANNEL_AUX3,
    MIXER_IN_RC_CHANNEL_9,
    MIXER_IN_RC_CHANNEL_10,
    MIXER_IN_RC_CHANNEL_11,
    MIXER_IN_RC_CHANNEL_12,
    MIXER_IN_RC_CHANNEL_13,
    MIXER_IN_RC_CHANNEL_14,
    MIXER_IN_RC_CHANNEL_15,
    MIXER_IN_RC_CHANNEL_16,
    MIXER_IN_RC_CHANNEL_17,
    MIXER_IN_RC_CHANNEL_18,
    MIXER_IN_COUNT
};

enum {
    MIXER_OP_NUL = 0,
    MIXER_OP_SET,
    MIXER_OP_ADD,
    MIXER_OP_MUL,
    MIXER_OP_COUNT
};


#define MIXER_RULE_COUNT      32

#define MIXER_INPUT_COUNT     MIXER_IN_COUNT
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

#define MIXER_RC_SCALING      (1.0f / 500)
#define MIXER_PID_SCALING     (1.0f / 500)
#define MIXER_THR_SCALING     (1.0f / (PWM_RANGE_MAX - PWM_RANGE_MIN))
#define MIXER_THR_OFFSET      PWM_RANGE_MIN

#define MIXER_SATURATION_TIME 5


typedef struct
{
    uint8_t   mode;

    uint8_t   main_rotor_dir;   // Main rotor direction: CW/CCW

    uint8_t   tail_rotor_mode;  // Tail motor vs. variable pitch tail
    int16_t   tail_motor_idle;  // Idle throttle for tail motor

    uint8_t   swash_ring;       // Swash ring size

} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct
{
    int16_t   rate;             // multiplier
    int16_t   min;              // minimum input value
    int16_t   max;              // maximum input value
} mixerInput_t;

PG_DECLARE_ARRAY(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs);

typedef struct
{
    uint32_t  mode;              // active flight mode bitmap
    uint8_t   oper;              // rule operation
    uint8_t   input;             // input channel
    uint8_t   output;            // output channel
    int16_t   offset;            // addition
    int16_t   weight;            // multiplier (weight and direction)
} mixerRule_t;

PG_DECLARE_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules);


void mixerInit(void);

void mixerUpdate(void);

float mixerGetInput(uint8_t i);
float mixerGetOutput(uint8_t i);
float mixerGetServoOutput(uint8_t i);
float mixerGetMotorOutput(uint8_t i);

bool mixerSaturated(uint8_t index);
void mixerSaturateInput(uint8_t index);
void mixerSaturateOutput(uint8_t index);

int16_t mixerGetOverride(uint8_t i);
int16_t mixerSetOverride(uint8_t i, int16_t value);

float getCyclicDeflection(void);
float getCollectiveDeflection(void);

static inline float getCollectiveDeflectionAbs(void) { return fabsf(getCollectiveDeflection()); }

static inline float mixerGetThrottle(void) { return mixerGetInput(MIXER_IN_RC_COMMAND_THROTTLE); }

static inline bool pidAxisSaturated(uint8_t i) { return mixerSaturated(MIXER_IN_STABILIZED_ROLL + i); }

static inline void mixerSaturateServoOutput(uint8_t index) { mixerSaturateOutput(index + MIXER_SERVO_OFFSET); }
static inline void mixerSaturateMotorOutput(uint8_t index) { mixerSaturateOutput(index + MIXER_MOTOR_OFFSET); }

static inline int mixerRotationSign() { return (mixerConfig()->main_rotor_dir == DIR_CW) ? -1 : 1; }

static inline bool mixerMotorizedTail() { return (mixerConfig()->tail_rotor_mode == TAIL_MODE_MOTORIZED); }

