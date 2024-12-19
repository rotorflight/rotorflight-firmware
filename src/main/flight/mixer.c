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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
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
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/governor.h"
#include "flight/wiggle.h"

#include "rx/rx.h"

#include "pg/mixer.h"

#include "sensors/gyro.h"


/** Internal data **/

typedef struct {

    float           input[MIXER_INPUT_COUNT];
    float           output[MIXER_OUTPUT_COUNT];

    bitmap_t        mapping[MIXER_OUTPUT_COUNT];
    int16_t         override[MIXER_INPUT_COUNT];
    uint16_t        saturation[MIXER_INPUT_COUNT];

    float           tailCenterTrim;
    float           tailMotorIdle;
    int8_t          tailMotorDirection;

    float           swashTrim[3];

    float           collTTAGain;
    float           collGeoCorrection;

    float           cyclicLimit;
    float           cyclicTotal;

    float           totalPitchLimit;
    float           cyclicRingLimit;
    float           cyclicSpeedLimit;

    float           cyclicPhaseSin;
    float           cyclicPhaseCos;

    bitmap_t        cyclicMapping;

} mixerData_t;

static FAST_DATA_ZERO_INIT mixerData_t mixer;


#ifdef USE_MIXER_HISTORY

// History lengh is 1024 samples (must be power of 2)
#define MIXER_HISTORY_TIME   (1<<10)
#define MIXER_HISTORY_MASK   (MIXER_HISTORY_TIME-1)

static float mixerInputHistory[4][MIXER_HISTORY_TIME];

static FAST_DATA_ZERO_INIT uint16_t historyIndex;

float mixerGetInputHistory(uint8_t index, uint16_t delay)
{
    return mixerInputHistory[index][(historyIndex - delay) & MIXER_HISTORY_MASK];
}

static inline void mixerUpdateHistory(void)
{
    historyIndex = (historyIndex + 1) & MIXER_HISTORY_MASK;

    mixerInputHistory[FD_ROLL][historyIndex]   = mixer.input[MIXER_IN_STABILIZED_ROLL];
    mixerInputHistory[FD_PITCH][historyIndex]  = mixer.input[MIXER_IN_STABILIZED_PITCH];
    mixerInputHistory[FD_YAW][historyIndex]    = mixer.input[MIXER_IN_STABILIZED_YAW];
    mixerInputHistory[FD_COLL][historyIndex]   = mixer.input[MIXER_IN_STABILIZED_COLLECTIVE];
}

#endif /* USE_MIXER_HISTORY */


/** Interface functions **/

float mixerGetInput(uint8_t index)
{
    return mixer.input[index];
}

float mixerGetOutput(uint8_t index)
{
    return mixer.output[index];
}

float getCyclicDeflection(void)
{
    return mixer.cyclicTotal;
}

bool mixerSaturated(uint8_t index)
{
    return (mixer.saturation[index] > 0);
}

void mixerSaturateInput(uint8_t index)
{
    mixer.saturation[index] = MIXER_SATURATION_TIME;
}

void mixerSaturateOutput(uint8_t index)
{
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.mapping[index] & BIT(i)) {
            mixerSaturateInput(i);
        }
    }
}

int16_t mixerGetOverride(uint8_t index)
{
    return mixer.override[index];
}

int16_t mixerSetOverride(uint8_t index, int16_t value)
{
    return mixer.override[index] = value;
}

bool mixerIsCyclicServo(uint8_t index)
{
    return (mixer.cyclicMapping & BIT(MIXER_SERVO_OFFSET + index));
}


/** Internal functions **/

static inline void mixerApplyInputLimit(int index, float value)
{
    const mixerInput_t *in = mixerInputs(index);

    // Input limits
    const float in_min = in->min / 1000.0f;
    const float in_max = in->max / 1000.0f;

    // Constrain and saturate
    if (value > in_max) {
        mixer.input[index] = in_max;
        mixerSaturateInput(index);
    }
    else if (value < in_min) {
        mixer.input[index] = in_min;
        mixerSaturateInput(index);
    }
    else {
        mixer.input[index] = value;
    }
}

static void mixerSetInput(int index, float value)
{
    // Use override or wiggle only if not armed
    if (!ARMING_FLAG(ARMED)) {
        if (mixer.override[index] >= MIXER_OVERRIDE_MIN && mixer.override[index] <= MIXER_OVERRIDE_MAX) {
            value = mixer.override[index] / 1000.0f;
        }
        else if (wiggleActive()) {
            if (index >= MIXER_IN_STABILIZED_ROLL && index <= MIXER_IN_STABILIZED_COLLECTIVE)
                value = wiggleGetAxis(index - MIXER_IN_STABILIZED_ROLL);
        }
    }

    mixerApplyInputLimit(index, value);
}

static void mixerUpdateCyclic(void)
{
    float SR = mixer.input[MIXER_IN_STABILIZED_ROLL];
    float SP = mixer.input[MIXER_IN_STABILIZED_PITCH];

    // Limit action active
    if (mixer.cyclicRingLimit > 0 || mixer.totalPitchLimit > 0)
    {
        float factor = 1.0f;

        // Apply cyclic ring limit
        if (mixer.cyclicRingLimit > 0 ) {
            // Inidividual limits on SP and SR
            const mixerInput_t *mixR = mixerInputs(MIXER_IN_STABILIZED_ROLL);
            const mixerInput_t *mixP = mixerInputs(MIXER_IN_STABILIZED_PITCH);

            // Assume min<0 and max>0
            const float maxR = MAX(abs((SR < 0) ? mixR->min : mixR->max), 10) / 1000.0f;
            const float maxP = MAX(abs((SP < 0) ? mixP->min : mixP->max), 10) / 1000.0f;

            // Stretch the values to a unit circle limit
            const float SSR = SR / (maxR * mixer.cyclicRingLimit);
            const float SSP = SP / (maxP * mixer.cyclicRingLimit);

            // Stretched cyclic deflection
            const float cyclic = sqrtf(sq(SSR) + sq(SSP));

            // Cyclic limits reached - scale back
            if (cyclic > 1.0f) {
                factor = 1.0f / cyclic;
            }
        }

        // Apply dynamic cyclic limit
        if (mixer.totalPitchLimit > 0) {
            // Total cyclic after ring limit
            const float cyclic = sqrtf(sq(SR) + sq(SP)) * factor;

            // Cyclic limits reached - scale back
            if (cyclic > mixer.cyclicLimit) {
                factor *= mixer.cyclicLimit / cyclic;
            }
        }

        // Apply limit factor
        if (factor < 1.0f) {
            SP *= factor;
            SR *= factor;
            mixerSaturateInput(MIXER_IN_STABILIZED_ROLL);
            mixerSaturateInput(MIXER_IN_STABILIZED_PITCH);
        }
    }

    // Swash phasing
    if (mixer.cyclicPhaseSin != 0)
    {
        const float P = SP;
        const float R = SR;
        SP = P * mixer.cyclicPhaseCos - R * mixer.cyclicPhaseSin;
        SR = P * mixer.cyclicPhaseSin + R * mixer.cyclicPhaseCos;
    }

    // Apply new values
    mixer.input[MIXER_IN_STABILIZED_ROLL]  = SR;
    mixer.input[MIXER_IN_STABILIZED_PITCH] = SP;

    // Total cyclic deflection
    mixer.cyclicTotal = sqrtf(sq(SP) + sq(SR));
}

static void mixerUpdateCollective(void)
{
    if (mixer.collTTAGain > 0) {
        // TTA ratio
        const float ratio = 1.0f + getTTAIncrease() * mixer.collTTAGain;

        // Counteract lift increase
        mixer.input[MIXER_IN_STABILIZED_COLLECTIVE] /= ratio * ratio;
    }

    // Limit cyclic if needed
    if (mixer.totalPitchLimit > 0) {
        mixer.cyclicLimit = fmaxf(mixer.totalPitchLimit - fabsf(mixer.input[MIXER_IN_STABILIZED_COLLECTIVE]), 0);
    }
}

static float mixerCollectiveCorrection(float SC)
{
    if (mixer.collGeoCorrection != 0) {
        if (SC > 0)
            SC += SC * mixer.collGeoCorrection;
        else
            SC -= SC * mixer.collGeoCorrection;
    }

    return SC;
}

static void mixerUpdateMotorizedTail(void)
{
    // Motorized tail control
    if (mixerIsTailMode(TAIL_MODE_MOTORIZED)) {
        // Yaw input value - positive is against torque
        float yaw = mixer.input[MIXER_IN_STABILIZED_YAW] * mixerRotationSign();

        // Add center trim
        yaw += mixer.tailCenterTrim;

        // Square root law
        float throttle = sqrtf(fmaxf(yaw, 0));

        // Apply minimum throttle
        throttle = fmaxf(throttle, mixer.tailMotorIdle);

        // Slow spoolup
        if (!isSpooledUp()) {
            if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.05f)
                throttle = 0;
            else if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.10f)
                throttle *= mixer.input[MIXER_IN_STABILIZED_THROTTLE] / 0.10f;
        }

        // Yaw is now tail motor throttle
        mixer.input[MIXER_IN_STABILIZED_YAW] = throttle;
    }
    // Bidirectional tail motor
    else if (mixerIsTailMode(TAIL_MODE_BIDIRECTIONAL)) {
        // Yaw input value - positive is against torque
        float yaw = mixer.input[MIXER_IN_STABILIZED_YAW] * mixerRotationSign();

        // Add center trim
        yaw += mixer.tailCenterTrim;

        // Use square root law
        float throttle = copysignf(sqrtf(fabsf(yaw)), yaw);

        // Apply minimum throttle
        if (throttle > -mixer.tailMotorIdle && throttle < mixer.tailMotorIdle)
            throttle = mixer.tailMotorDirection * mixer.tailMotorIdle;

        // Slow spoolup
        if (!isSpooledUp()) {
            if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.05f)
                throttle = 0;
            else if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.10f)
                throttle *= mixer.input[MIXER_IN_STABILIZED_THROTTLE] / 0.10f;
        }

        // Direction sign
        mixer.tailMotorDirection = (throttle < 0) ? -1 : 1;

        // Yaw is now tail motor throttle
        mixer.input[MIXER_IN_STABILIZED_YAW] = throttle;
    }
}

#define inputValue(NAME)                (mixer.input[MIXER_IN_STABILIZED_##NAME] * mixerInputs(MIXER_IN_STABILIZED_##NAME)->rate / 1000.0f)
#define setServoOutput(SERVO,VAL)       (mixer.output[MIXER_SERVO_OFFSET + (SERVO)] = (VAL))
#define setMotorOutput(MOTOR,VAL)       (mixer.output[MIXER_MOTOR_OFFSET + (MOTOR)] = (VAL))

static void mixerUpdateSwash(void)
{
    if (mixerConfig()->swash_type)
    {
        float SR = inputValue(ROLL);
        float SP = inputValue(PITCH);
        float SY = inputValue(YAW);
        float SC = inputValue(COLLECTIVE);
        float ST = inputValue(THROTTLE);

        float TC = mixer.tailCenterTrim;

        SC = mixerCollectiveCorrection(SC);

        SR += mixer.swashTrim[0];
        SP += mixer.swashTrim[1];
        SC += mixer.swashTrim[2];

        switch (mixerConfig()->swash_type) {
            case SWASH_TYPE_120:
                setServoOutput(0, 0.5f * SC - SP);
                setServoOutput(1, 0.5f * SC + 0.86602540f * SR + 0.5f * SP);
                setServoOutput(2, 0.5f * SC - 0.86602540f * SR + 0.5f * SP);
                break;

            case SWASH_TYPE_135:
                setServoOutput(0, 0.5f * SC - SP);
                setServoOutput(1, 0.5f * SC + 0.70710678f * SR + 0.70710678f * SP);
                setServoOutput(2, 0.5f * SC - 0.70710678f * SR + 0.70710678f * SP);
                break;

            case SWASH_TYPE_140:
                setServoOutput(0, 0.5f * SC - SP);
                setServoOutput(1, 0.5f * SC + 0.64278760f * SR + 0.76604444f * SP);
                setServoOutput(2, 0.5f * SC - 0.64278760f * SR + 0.76604444f * SP);
                break;

            case SWASH_TYPE_90L:
                setServoOutput(0, SP);
                setServoOutput(1, SR);
                break;

            case SWASH_TYPE_90V:
                setServoOutput(0,  0.70710678f * SR + 0.70710678f * SP);
                setServoOutput(1, -0.70710678f * SR + 0.70710678f * SP);
                break;

            case SWASH_TYPE_THRU:
                setServoOutput(0, SP);
                setServoOutput(1, SR);
                setServoOutput(2, SC);
                break;
        }

        setMotorOutput(0, ST);

        if (mixerMotorizedTail())
            setMotorOutput(1, SY);
        else
            setServoOutput(3, SY + TC);
    }
}

static void mixerUpdateRules(void)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (mixerRules(i)->oper) {
            uint8_t src = mixerRules(i)->input;
            uint8_t dst = mixerRules(i)->output;
            float   val = mixer.input[src] * mixerInputs(src)->rate / 1000.0f;
            float   out = (mixerRules(i)->offset + mixerRules(i)->weight * val) / 1000.0f;

            switch (mixerRules(i)->oper)
            {
                case MIXER_OP_SET:
                    mixer.output[dst] = out;
                    break;
                case MIXER_OP_ADD:
                    mixer.output[dst] += out;
                    break;
                case MIXER_OP_MUL:
                    mixer.output[dst] *= out;
                    break;
            }
        }
    }
}

static void mixerUpdateInputs(void)
{
    // Flight Dynamics
    mixerSetInput(MIXER_IN_RC_COMMAND_ROLL, getRcDeflection(ROLL));
    mixerSetInput(MIXER_IN_RC_COMMAND_PITCH, getRcDeflection(PITCH));
    mixerSetInput(MIXER_IN_RC_COMMAND_YAW, getRcDeflection(YAW));
    mixerSetInput(MIXER_IN_RC_COMMAND_COLLECTIVE, getRcDeflection(COLLECTIVE));

    // Throttle input
    mixerSetInput(MIXER_IN_RC_COMMAND_THROTTLE, getThrottle());

    // RC channels
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
        mixerSetInput(MIXER_IN_RC_CHANNEL_ROLL + i, rcCommand[i] / 500);

    // Stabilised inputs
    mixerSetInput(MIXER_IN_STABILIZED_ROLL, pidGetOutput(PID_ROLL));
    mixerSetInput(MIXER_IN_STABILIZED_PITCH, pidGetOutput(PID_PITCH));
    mixerSetInput(MIXER_IN_STABILIZED_YAW, pidGetOutput(PID_YAW));
    mixerSetInput(MIXER_IN_STABILIZED_COLLECTIVE, pidGetCollective());

    // Calculate collective
    mixerUpdateCollective();

    // Calculate cyclic
    mixerUpdateCyclic();

    // Update governor sub-mixer
    governorUpdate();

    // Update throttle from governor
    mixerSetInput(MIXER_IN_STABILIZED_THROTTLE, getGovernorOutput());

    // Update motorized tail (must be done after governor)
    mixerUpdateMotorizedTail();

#ifdef USE_MIXER_HISTORY
    // Update historical values
    mixerUpdateHistory();
#endif
}

void mixerUpdate(timeUs_t currentTimeUs)
{
    // Reset saturation
    for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.saturation[i])
            mixer.saturation[i]--;
    }

    // Reset mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
    }

    // Update wiggles
    wiggleUpdate(currentTimeUs);

    // Fetch input values
    mixerUpdateInputs();

    // Evaluate hard-coded mixer
    mixerUpdateSwash();

    // Evaluate rule-based mixer
    mixerUpdateRules();
}

void INIT_CODE validateAndFixMixerConfig(void)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++)
    {
        mixerRule_t *rule = mixerRulesMutable(i);

        if (rule->oper) {
            rule->oper    = constrain(rule->oper, 0, MIXER_OP_COUNT - 1);
            rule->input   = constrain(rule->input, 0, MIXER_INPUT_COUNT - 1);
            rule->output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
            rule->offset  = constrain(rule->offset, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            rule->weight  = constrain(rule->weight, MIXER_WEIGHT_MIN, MIXER_WEIGHT_MAX);
        }
        else {
            rule->oper    = 0;
            rule->input   = 0;
            rule->output  = 0;
            rule->offset  = 0;
            rule->weight  = 0;
        }
    }

    if (mixerConfig()->swash_pitch_limit > 0) {
        const uint16_t min_cyclic = 500;  // = 6°
        uint16_t limit = mixerConfig()->swash_pitch_limit;
        limit = MAX(limit, ABS(mixerInputs(MIXER_IN_STABILIZED_COLLECTIVE)->max) + min_cyclic);
        limit = MAX(limit, ABS(mixerInputs(MIXER_IN_STABILIZED_COLLECTIVE)->min) + min_cyclic);
        mixerConfigMutable()->swash_pitch_limit = limit;
    }
}

void INIT_CODE mixerInitConfig(void)
{
    if (mixerConfig()->swash_pitch_limit)
        mixer.totalPitchLimit = mixerConfig()->swash_pitch_limit / 1000.0f;
    else
        mixer.totalPitchLimit = 0;

    if (mixerConfig()->swash_ring)
        mixer.cyclicRingLimit = 1.4142135623f - mixerConfig()->swash_ring * 0.004142135623f;
    else
        mixer.cyclicRingLimit = 0;

    if (mixerConfig()->swash_phase) {
        const float angle = DECIDEGREES_TO_RADIANS(mixerConfig()->swash_phase);
        mixer.cyclicPhaseSin = sin_approx(angle);
        mixer.cyclicPhaseCos = cos_approx(angle);
    }
    else {
        mixer.cyclicPhaseSin = 0;
        mixer.cyclicPhaseCos = 1;
    }

    for (int i = 0; i < 3; i++)
        mixer.swashTrim[i] = mixerConfig()->swash_trim[i] / 1000.0f;

    mixer.collTTAGain = mixerConfig()->swash_tta_precomp / 100.0f;
    mixer.collGeoCorrection = mixerConfig()->swash_geo_correction / 1000.0f;

    mixer.tailMotorIdle = mixerConfig()->tail_motor_idle / 1000.0f;
    mixer.tailCenterTrim = mixerConfig()->tail_center_trim / 1000.0f;
}

static void INIT_CODE setMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] = BIT(in);

    if (in == MIXER_IN_STABILIZED_ROLL || in == MIXER_IN_STABILIZED_PITCH ||
        in == MIXER_IN_RC_COMMAND_ROLL || in == MIXER_IN_RC_COMMAND_PITCH) {
        mixer.cyclicMapping |= BIT(out);
    }
}

static void INIT_CODE addMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] |= BIT(in);

    if (in == MIXER_IN_STABILIZED_ROLL || in == MIXER_IN_STABILIZED_PITCH ||
        in == MIXER_IN_RC_COMMAND_ROLL || in == MIXER_IN_RC_COMMAND_PITCH) {
        mixer.cyclicMapping |= BIT(out);
    }
}

#define addServoMapping(INDEX,SERVO)    addMapping((INDEX), MIXER_SERVO_OFFSET + (SERVO))
#define addMotorMapping(INDEX,MOTOR)    addMapping((INDEX), MIXER_MOTOR_OFFSET + (MOTOR))

void INIT_CODE mixerInit(void)
{
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
        mixer.mapping[i] = 0;
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixer.override[i] = MIXER_OVERRIDE_OFF;
    }

    if (mixerConfig()->swash_type)
    {
        switch (mixerConfig()->swash_type) {
            case SWASH_TYPE_120:
            case SWASH_TYPE_135:
            case SWASH_TYPE_140:
                addServoMapping(MIXER_IN_STABILIZED_COLLECTIVE, 0);
                addServoMapping(MIXER_IN_STABILIZED_COLLECTIVE, 1);
                addServoMapping(MIXER_IN_STABILIZED_COLLECTIVE, 2);
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 0);
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 1);
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 2);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 1);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 2);
                break;

            case SWASH_TYPE_90L:
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 0);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 1);
                break;

            case SWASH_TYPE_90V:
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 0);
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 1);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 0);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 1);
                break;

            case SWASH_TYPE_THRU:
                addServoMapping(MIXER_IN_STABILIZED_PITCH, 0);
                addServoMapping(MIXER_IN_STABILIZED_ROLL, 1);
                addServoMapping(MIXER_IN_STABILIZED_COLLECTIVE, 2);
                break;
        }

        addMotorMapping(MIXER_IN_STABILIZED_THROTTLE, 0);

        if (mixerMotorizedTail())
            addMotorMapping(MIXER_IN_STABILIZED_YAW, 1);
        else
            addServoMapping(MIXER_IN_STABILIZED_YAW, 3);
    }

    for (int i = 0; i < MIXER_RULE_COUNT; i++)
    {
        const mixerRule_t *rule = mixerRules(i);

        switch (rule->oper)
        {
            case MIXER_OP_SET:
                setMapping(rule->input, rule->output);
                break;
            case MIXER_OP_ADD:
            case MIXER_OP_MUL:
                addMapping(rule->input, rule->output);
                break;
        }
    }

    mixerInitConfig();

    wiggleInit();
}
