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

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/governor.h"
#include "flight/leveling.h"

#include "rx/rx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"


PG_REGISTER(mixerConfig_t, mixerConfig, PG_GENERIC_MIXER_CONFIG, 0);

PG_REGISTER_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules, PG_GENERIC_MIXER_RULES, 0);

PG_REGISTER_ARRAY(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs, PG_GENERIC_MIXER_INPUTS, 0);


static FAST_RAM_ZERO_INIT mixerRule_t rules[MIXER_RULE_COUNT];

static FAST_RAM_ZERO_INIT float     mixInput[MIXER_INPUT_COUNT];
static FAST_RAM_ZERO_INIT float     mixOutput[MIXER_OUTPUT_COUNT];
static FAST_RAM_ZERO_INIT int16_t   mixOverride[MIXER_INPUT_COUNT];
static FAST_RAM_ZERO_INIT uint32_t  mixOutputMap[MIXER_OUTPUT_COUNT];
static FAST_RAM_ZERO_INIT uint16_t  mixSaturated[MIXER_INPUT_COUNT];

static FAST_RAM_ZERO_INIT float     cyclicTotal;
static FAST_RAM_ZERO_INIT float     cyclicLimit;

static FAST_RAM_ZERO_INIT float     tailMotorIdle;


static inline float mixerOverrideInput(int index, float val)
{
    // Check override only if not armed
    if (!ARMING_FLAG(ARMED)) {
        if (mixOverride[index] >= MIXER_OVERRIDE_MIN && mixOverride[index] <= MIXER_OVERRIDE_MAX)
            val = mixOverride[index] * 0.001f;
    }

    return val;
}

static inline float mixerScaleInput(int index, float val)
{
    const mixerInput_t *in = mixerInputs(index);

    // Override real input with a test value
    val = mixerOverrideInput(index, val);

    // Constrain and scale
    val = constrainf(val, in->min*0.001f, in->max*0.001f) * in->rate * 0.001f;

    return val;
}


void mixerInit(void)
{
    cyclicLimit = PIDSUM_LIMIT * MIXER_PID_SCALING;

    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        const mixerRule_t *rule = mixerRules(i);

        if (rule->oper) {
            rules[i].mode    = rule->mode;
            rules[i].oper    = constrain(rule->oper, 0, MIXER_OP_COUNT - 1);
            rules[i].input   = constrain(rule->input, 0, MIXER_INPUT_COUNT - 1);
            rules[i].output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
            rules[i].offset  = constrain(rule->offset, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            rules[i].weight  = constrain(rule->weight, MIXER_WEIGHT_MIN, MIXER_WEIGHT_MAX);
        }
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixOverride[i] = MIXER_OVERRIDE_OFF;
    }

    tailMotorIdle = mixerConfig()->tail_motor_idle / 1000.0f;
}

static void mixerUpdateInputs(void)
{
    // Flight Dynamics
    mixInput[MIXER_IN_RC_COMMAND_ROLL]        = rcCommand[ROLL]       * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_PITCH]       = rcCommand[PITCH]      * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_YAW]         = rcCommand[YAW]        * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_COLLECTIVE]  = rcCommand[COLLECTIVE] * MIXER_RC_SCALING;

    // Throttle input
    mixInput[MIXER_IN_RC_COMMAND_THROTTLE]    = (rcCommand[THROTTLE] - MIXER_THR_OFFSET) * MIXER_THR_SCALING;

    // AUX channels
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
        mixInput[MIXER_IN_RC_CHANNEL_ROLL + i] = (rcData[i] - rxConfig()->midrc) * MIXER_RC_SCALING;

    // Collective -- TODO: move rescue to separate module
    if (!FLIGHT_MODE(RESCUE_MODE)) {
        mixInput[MIXER_IN_STABILIZED_COLLECTIVE] = mixInput[MIXER_IN_RC_COMMAND_COLLECTIVE];
    } else {
        mixInput[MIXER_IN_STABILIZED_COLLECTIVE] = pidRescueCollective();
    }

    // Tail/Yaw is always stabilised - positive is against main rotor torque
    mixInput[MIXER_IN_STABILIZED_YAW] = mixerRotationSign() * pidData[FD_YAW].Sum *  MIXER_PID_SCALING;

    // Update governor sub-mixer
    governorUpdate();

    // Update throttle from governor
    mixInput[MIXER_IN_STABILIZED_THROTTLE] = getGovernorOutput();

    // PASSTHROUGH mode disables roll/pitch stabilization (flybar mode)
    if (!FLIGHT_MODE(PASSTHRU_MODE)) {
        mixInput[MIXER_IN_STABILIZED_ROLL]  = pidData[FD_ROLL].Sum  * MIXER_PID_SCALING;
        mixInput[MIXER_IN_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * MIXER_PID_SCALING;
    } else {
        mixInput[MIXER_IN_STABILIZED_ROLL]  = rcCommand[ROLL]       * MIXER_RC_SCALING;
        mixInput[MIXER_IN_STABILIZED_PITCH] = rcCommand[PITCH]      * MIXER_RC_SCALING;
    }

    // Motorized tail control
    if (mixerMotorizedTail()) {

        // Thrust linearizaion
        mixInput[MIXER_IN_STABILIZED_YAW] = sqrtf(constrainf(mixInput[MIXER_IN_STABILIZED_YAW], 0, 1));

        // Spoolup follows main rotor
        if (isSpooledUp()) {
            mixInput[MIXER_IN_STABILIZED_YAW] = constrainf(mixInput[MIXER_IN_STABILIZED_YAW], tailMotorIdle, 1);
        } else {
            if (mixInput[MIXER_IN_STABILIZED_THROTTLE] < 0.01f)
                mixInput[MIXER_IN_STABILIZED_YAW] = 0;
            else if (mixInput[MIXER_IN_STABILIZED_THROTTLE] < 0.25f)
                mixInput[MIXER_IN_STABILIZED_YAW] *= mixInput[MIXER_IN_STABILIZED_THROTTLE] / 0.25f;
        }
    }

    // Scale/limit inputs
    for (int i = 1; i < MIXER_INPUT_COUNT; i++)
        mixInput[i] = mixerScaleInput(i, mixInput[i]);

    // TODO: Move into swash sub-mixer
    // Swashplate cyclic deflection
    cyclicTotal = sqrtf(mixInput[MIXER_IN_STABILIZED_ROLL] * mixInput[MIXER_IN_STABILIZED_ROLL] +
                        mixInput[MIXER_IN_STABILIZED_PITCH] * mixInput[MIXER_IN_STABILIZED_PITCH]);

    // Cyclic ring limit reached
    if (cyclicTotal > cyclicLimit) {
        mixerSaturateInput(MIXER_IN_STABILIZED_ROLL);
        mixerSaturateInput(MIXER_IN_STABILIZED_PITCH);
        mixInput[MIXER_IN_STABILIZED_ROLL]  *= cyclicLimit / cyclicTotal;
        mixInput[MIXER_IN_STABILIZED_PITCH] *= cyclicLimit / cyclicTotal;
        cyclicTotal = cyclicLimit;
    }
}

void mixerUpdate(void)
{
    // Reset mixer inputs
    for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
        mixInput[i] = 0;
        if (mixSaturated[i])
            mixSaturated[i]--;
    }
    // Reset mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixOutput[i] = 0;
        mixOutputMap[i] = 0;
    }

    // Fetch input values
    mixerUpdateInputs();

    // Current flight mode bitmap
    uint32_t flightModeMask = ((uint32_t)(~flightModeFlags)) << 16 | flightModeFlags;

    // Calculate mixer outputs
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (rules[i].oper && ((rules[i].mode == 0) || (rules[i].mode & flightModeMask))) {
            uint8_t src = rules[i].input;
            uint8_t dst = rules[i].output;
            float   out = (rules[i].offset + rules[i].weight * mixInput[src]) * 0.001f;

            switch (rules[i].oper)
            {
                case MIXER_OP_SET:
                    mixOutput[dst] = out;
                    mixOutputMap[dst] = BIT(src);
                    break;
                case MIXER_OP_ADD:
                    mixOutput[dst] += out;
                    mixOutputMap[dst] |= BIT(src);
                    break;
                case MIXER_OP_MUL:
                    mixOutput[dst] *= out;
                    mixOutputMap[dst] |= BIT(src);
                    break;
            }
        }
    }
}

bool mixerSaturated(uint8_t index)
{
    return (mixSaturated[index] > 0);
}

void mixerSaturateInput(uint8_t index)
{
    mixSaturated[index] = MIXER_SATURATION_TIME;
}

void mixerSaturateOutput(uint8_t index)
{
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        if (mixOutputMap[index] & BIT(i)) {
            mixerSaturateInput(i);
        }
    }
}

float mixerGetInput(uint8_t i)
{
    return mixInput[i];
}

float mixerGetOutput(uint8_t i)
{
    return mixOutput[i];
}

float mixerGetServoOutput(uint8_t i)
{
    return mixOutput[MIXER_SERVO_OFFSET + i];
}

float mixerGetMotorOutput(uint8_t i)
{
    return mixOutput[MIXER_MOTOR_OFFSET + i];
}

int16_t mixerGetOverride(uint8_t i)
{
    return mixOverride[i];
}

int16_t mixerSetOverride(uint8_t i, int16_t value)
{
    return mixOverride[i] = value;
}

float getCyclicDeflection(void)
{
    return cyclicTotal;
}

float getCollectiveDeflection(void)
{
    return mixInput[MIXER_IN_STABILIZED_COLLECTIVE];
}
