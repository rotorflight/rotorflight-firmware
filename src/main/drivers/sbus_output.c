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

#include "sbus_output.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "build/build_config.h"
#include "common/maths.h"
#include "common/time.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "io/serial.h"
#include "pg/bus_servo.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "platform.h"
#include "rx/rx.h"

// SBUS payload value range for analog channels
#define SBUS_MIN 192
#define SBUS_MAX 1792

static serialPort_t *sbusOutPort = NULL;

// Storage for speed limiting (similar to servoInput in servos.c)
static FAST_DATA_ZERO_INIT float sbusServoInput[SBUS_OUT_CHANNELS];

// Cached cyclic ratio (calculated once per update cycle)
static FAST_DATA_ZERO_INIT float sbusCyclicRatio = 1.0f;
static FAST_DATA_ZERO_INIT bool sbusCyclicRatioValid = false;

static void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame,
                                    uint16_t *channels)
{
    frame->syncByte = 0x0F;

    // There's no way to make a bit field array, so we have to go tedious.
    frame->chan0 = channels[0];
    frame->chan1 = channels[1];
    frame->chan2 = channels[2];
    frame->chan3 = channels[3];
    frame->chan4 = channels[4];
    frame->chan5 = channels[5];
    frame->chan6 = channels[6];
    frame->chan7 = channels[7];
    frame->chan8 = channels[8];
    frame->chan9 = channels[9];
    frame->chan10 = channels[10];
    frame->chan11 = channels[11];
    frame->chan12 = channels[12];
    frame->chan13 = channels[13];
    frame->chan14 = channels[14];
    frame->chan15 = channels[15];

    frame->flags = channels[16] ? BIT(0) : 0;
    frame->flags |= channels[17] ? BIT(1) : 0;
    // Other flags?

    frame->endByte = 0;
}

float sbusOutGetRX(uint8_t channel)
{
    if (channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)
        return rcChannel[channel];
    return 0;
}

// Helper function similar to limitTravel in servos.c
static inline float sbusLimitTravel(uint8_t channel, float pos, float min, float max)
{
    const uint8_t servoIndex = BUS_SERVO_OFFSET + channel;
    if (pos > max) {
        mixerSaturateServoOutput(servoIndex);
        return max;
    } else if (pos < min) {
        mixerSaturateServoOutput(servoIndex);
        return min;
    }
    return pos;
}

// Helper function similar to limitSpeed in servos.c
static inline float sbusLimitSpeed(float old, float new, float speed)
{
    float rate = 1200 * pidGetDT() / speed;
    float diff = new - old;

    if (diff > rate)
        new = old + rate;
    else if (diff < -rate)
        new = old - rate;

    return new;
}

// Helper function similar to limitRatio in servos.c
static inline float sbusLimitRatio(float old, float new, float ratio)
{
    return old + (new - old) * ratio;
}

// Calculate cyclic ratio for all channels (called once per update cycle)
static void sbusOutCalculateCyclicRatio(void)
{
    float cyclic_ratio = 1.0f;
    
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++)
    {
        const uint8_t servoIndex = BUS_SERVO_OFFSET + ch;
        
        if (servoIndex >= MAX_SUPPORTED_SERVOS)
            continue;

        const servoParam_t *servo = servoParams(servoIndex);

        // Get normalized mixer output (-1.0 to 1.0), or servo override when disarmed
        float input;
        if (!ARMING_FLAG(ARMED) && hasServoOverride(servoIndex))
            input = getServoOverride(servoIndex) / 1000.0f;
        else
            input = mixerGetServoOutput(servoIndex);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        // Apply geometry correction if enabled for this servo
        if (servo->flags & SERVO_FLAG_GEO_CORR)
            input = geometryCorrection(input);
#endif

        // Calculate cyclic ratio for speed limiting (if this is a cyclic servo)
        if (servo->speed && mixerIsCyclicServo(servoIndex)) {
            const float limit = 1200 * pidGetDT() / servo->speed;
            const float speed = fabsf(input - sbusServoInput[ch]);
            if (speed > limit)
                cyclic_ratio = fminf(cyclic_ratio, limit / speed);
        }
    }
    
    sbusCyclicRatio = cyclic_ratio;
    sbusCyclicRatioValid = true;
}

// Process a single SBUS mixer channel with same logic as servoUpdate()
// Returns processed value in microseconds
float sbusOutGetValueMixer(uint8_t channel)
{
    if (channel >= SBUS_OUT_CHANNELS)
        return 0;
    
    const uint8_t servoIndex = BUS_SERVO_OFFSET + channel;
    
    if (servoIndex >= MAX_SUPPORTED_SERVOS)
        return 0;

    const servoParam_t *servo = servoParams(servoIndex);

    // Get normalized mixer output (-1.0 to 1.0), or servo override when disarmed
    float input;
    if (!ARMING_FLAG(ARMED) && hasServoOverride(servoIndex))
        input = getServoOverride(servoIndex) / 1000.0f;
    else
        input = mixerGetServoOutput(servoIndex - BUS_SERVO_OFFSET);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
    // Apply geometry correction if enabled for this servo
    if (servo->flags & SERVO_FLAG_GEO_CORR)
        input = geometryCorrection(input);
#endif

    float pos = input;

    // Apply speed limiting
    if (servo->speed > 0) {
        if (mixerIsCyclicServo(servoIndex)) {
            // Use cached cyclic ratio (must be calculated first via sbusOutCalculateCyclicRatio)
            if (!sbusCyclicRatioValid)
                sbusOutCalculateCyclicRatio();
            pos = sbusLimitRatio(sbusServoInput[channel], pos, sbusCyclicRatio);
        }
        else {
            pos = sbusLimitSpeed(sbusServoInput[channel], pos, servo->speed);
        }
    }

    // Store input for next iteration
    sbusServoInput[channel] = pos;

    // Apply servo reversal
    if (servo->flags & SERVO_FLAG_REVERSED)
        pos = -pos;

    // Apply servo scale (rneg/rpos)
    float scale = (pos > 0) ? servo->rpos : servo->rneg;

    // Apply travel limits with saturation
    pos = sbusLimitTravel(channel, scale * pos, servo->min, servo->max);
    
    // Add midpoint to get final microsecond value
    pos = servo->mid + pos;

    // Clamp to bus servo limits for SBUS output
    return constrainf(pos, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL);
}

// Process all SBUS mixer channels (batch version for sbusOutUpdate)
void sbusOutProcessMixerChannels(float output[SBUS_OUT_CHANNELS])
{
    // Calculate cyclic ratio once for all channels
    sbusOutCalculateCyclicRatio();
    
    // Process each channel
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++) {
        output[ch] = sbusOutGetValueMixer(ch);
    }
    
    // Invalidate cyclic ratio for next update cycle
    sbusCyclicRatioValid = false;
}

// Get channel value based on source type (RX passthrough or processed mixer output)
static float sbusOutGetChannelValue(uint8_t channel, const float *mixerOutputs)
{
    const busServoSourceType_e source_type = busServoConfig()->sourceType[channel];
    switch (source_type) {
    case BUS_SERVO_SOURCE_RX:
        return sbusOutGetRX(channel);
    case BUS_SERVO_SOURCE_MIXER:
        return mixerOutputs[channel];
    }
    return 0;
}

static uint16_t sbusOutConvertToSbus(uint8_t channel, float pwm)
{
    // For digital channels (16-17), convert to 0 or 1
    if (channel >= 16) {
        // Assume threshold at 1500us
        return (pwm >= 1500) ? 1 : 0;
    }

    // For analog channels (0-15), convert microseconds to SBUS range (192-1792)
    // Bus servo range: (1000 -> BUS_SERVO_MIN_SIGNAL) to (2000 -> BUS_SERVO_MAX_SIGNAL) -> SBUS 192-1792
    const float value = scaleRangef(pwm, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL, SBUS_MIN, SBUS_MAX);
    return constrain(nearbyintf(value), SBUS_MIN, SBUS_MAX);
}

void sbusOutUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!sbusOutPort)
        return;

    // Check TX Buff is free
    if (serialTxBytesFree(sbusOutPort) <= sizeof(sbusOutFrame_t))
        return;

    // Process all mixer channels with servoUpdate() logic
    float mixerOutputs[SBUS_OUT_CHANNELS];
    sbusOutProcessMixerChannels(mixerOutputs);

    // Prepare SBUS frame
    sbusOutFrame_t frame;
    uint16_t channels[SBUS_OUT_CHANNELS];
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++) {
        float value = sbusOutGetChannelValue(ch, mixerOutputs);
        channels[ch] = sbusOutConvertToSbus(ch, value);
        
        // Store the output value for getServoOutput() to retrieve
        setBusServoOutput(ch, value);
    }
    sbusOutPrepareSbusFrame(&frame, channels);

    // serial output
    serialWriteBuf(sbusOutPort, (const uint8_t *)&frame, sizeof(frame));
}

bool sbusOutIsEnabled() 
{ 
    return sbusOutPort != NULL;
}

void sbusOutInit()
{
    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_SBUS_OUT);

    if (!portConfig) {
        sbusOutPort = NULL;
        return;
    }

    sbusOutPort = openSerialPort(
        portConfig->identifier, FUNCTION_SBUS_OUT, NULL, NULL, 100000, MODE_TX,
        SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN |
            (sbusOutConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_UNIDIR |
            (sbusOutConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}
