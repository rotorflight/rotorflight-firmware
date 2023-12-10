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


enum {
    SWASH_TYPE_NONE = 0,
    SWASH_TYPE_THRU,
    SWASH_TYPE_120,
    SWASH_TYPE_135,
    SWASH_TYPE_140,
    SWASH_TYPE_90L,
    SWASH_TYPE_90V,
};

enum {
    TAIL_MODE_VARIABLE,
    TAIL_MODE_MOTORIZED,
    TAIL_MODE_BIDIRECTIONAL,
};

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

enum {
    DIR_CW,
    DIR_CCW,
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

#define MIXER_SATURATION_TIME 5


typedef struct
{
    uint8_t   main_rotor_dir;   // Main rotor direction: CW/CCW

    uint8_t   tail_rotor_mode;  // Tail motor vs. variable pitch tail
    uint8_t   tail_motor_idle;  // Idle throttle for tail motor
    int8_t    tail_center_trim; // Tail center position offset

    uint8_t   swash_type;       // Swashplate type
    uint8_t   swash_ring;       // Swash ring size
    int16_t   swash_phase;      // Swashplate phasing angle
    uint16_t  swash_pitch_limit; // Maximum main rotor blade pitch
    int8_t    swash_trim[3];    // Swashplate leveling trim

    uint8_t   coll_tta_precomp;  // TTA Collective correction

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
    uint8_t   oper;             // rule operation
    uint8_t   input;            // input channel
    uint8_t   output;           // output channel
    int16_t   offset;           // addition
    int16_t   weight;           // multiplier (weight and direction)
} mixerRule_t;

PG_DECLARE_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules);


/** Interface function **/

void mixerInit(void);
void mixerInitConfig(void);

void validateAndFixMixerConfig(void);

void mixerUpdate(void);

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

static inline bool mixerCwMainRotor()
{
    return mixerConfig()->main_rotor_dir == DIR_CW;
}

static inline int mixerRotationSign()
{
    return mixerCwMainRotor() ? -1 : 1;
}

static inline bool mixerMotorizedTail(void)
{
    return (mixerConfig()->tail_rotor_mode != TAIL_MODE_VARIABLE);
}

static inline bool mixerIsTailMode(int mode)
{
    return (mixerConfig()->tail_rotor_mode == mode);
}
