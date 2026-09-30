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

// Speed-limit state for SBUS output (similar to servoInput in servos.c). F.Bus
// output keeps its own, see sbusOutSpeedState_t.
static FAST_DATA_ZERO_INIT sbusOutSpeedState_t sbusOutSpeedState;

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

// Mixer servo output a bus channel follows. Bus channels read the mixer by
// channel number, so S9-S16 mirror S1-S8 and cyclic servos can sit on the bus.
// Input, cyclic detection and saturation must all use this same index.
static inline uint8_t sbusOutMixerIndex(uint8_t channel)
{
    return channel;
}

static inline bool sbusOutIsMixerChannel(uint8_t channel)
{
    return busServoConfig()->sourceType[channel] == BUS_SERVO_SOURCE_MIXER;
}

// Normalized mixer output (-1.0 to 1.0) for a bus channel, or the servo
// override when disarmed
static float sbusOutGetInput(uint8_t channel, const servoParam_t *servo)
{
    const uint8_t servoIndex = BUS_SERVO_OFFSET + channel;
    float input;

    if (!ARMING_FLAG(ARMED) && hasServoOverride(servoIndex))
        input = getServoOverride(servoIndex) / 1000.0f;
    else
        input = mixerGetServoOutput(sbusOutMixerIndex(channel));

#ifdef USE_SERVO_GEOMETRY_CORRECTION
    // Apply geometry correction if enabled for this servo
    if (servo->flags & SERVO_FLAG_GEO_CORR)
        input = geometryCorrection(input);
#else
    UNUSED(servo);
#endif

    return input;
}

// Helper function similar to limitTravel in servos.c. Saturation goes to the
// mixer output the channel follows, so a mirrored servo at its travel limit
// stops I-term windup like the PWM servo does. Channels not sent from the
// mixer don't saturate it.
static inline float sbusLimitTravel(uint8_t channel, float pos, float min, float max)
{
    if (pos > max) {
        if (sbusOutIsMixerChannel(channel))
            mixerSaturateServoOutput(sbusOutMixerIndex(channel));
        return max;
    } else if (pos < min) {
        if (sbusOutIsMixerChannel(channel))
            mixerSaturateServoOutput(sbusOutMixerIndex(channel));
        return min;
    }
    return pos;
}

// Helper function similar to limitSpeed in servos.c. dt is the time since the
// output's previous frame: the limit runs once per output frame, not once per
// PID loop like servoUpdate().
static inline float sbusLimitSpeed(float old, float new, float speed, float dt)
{
    float rate = 1200 * dt / speed;
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

// Calculate cyclic ratio for the output's channels (called once per output frame).
// Only channels this output sends from the mixer count: others don't update
// state->pos, and a stale position would hold the ratio down.
static void sbusOutCalculateCyclicRatio(sbusOutSpeedState_t *state, uint8_t channelCount)
{
    float cyclic_ratio = 1.0f;

    for (int ch = 0; ch < channelCount; ch++)
    {
        const uint8_t servoIndex = BUS_SERVO_OFFSET + ch;

        if (servoIndex >= MAX_SUPPORTED_SERVOS || !sbusOutIsMixerChannel(ch))
            continue;

        const servoParam_t *servo = servoParams(servoIndex);

        // Calculate cyclic ratio for speed limiting (if this is a cyclic servo)
        if (servo->speed && mixerIsCyclicServo(sbusOutMixerIndex(ch))) {
            const float input = sbusOutGetInput(ch, servo);
            const float limit = 1200 * state->dt / servo->speed;
            const float speed = fabsf(input - state->pos[ch]);
            if (speed > limit)
                cyclic_ratio = fminf(cyclic_ratio, limit / speed);
        }
    }

    state->cyclicRatio = cyclic_ratio;
}

void sbusOutBeginFrame(sbusOutSpeedState_t *state, timeUs_t currentTimeUs, float frameRateHz, uint8_t channelCount)
{
    // Time since this output's previous frame. The first frame assumes the
    // configured frame rate; a long gap is capped at 100ms.
    state->dt = state->lastFrameUs ?
        constrainf(cmpTimeUs(currentTimeUs, state->lastFrameUs) * 1e-6f, 0.0f, 0.1f) :
        1.0f / frameRateHz;
    state->lastFrameUs = currentTimeUs;

    sbusOutCalculateCyclicRatio(state, MIN(channelCount, SBUS_OUT_CHANNELS));
}

// Process a single SBUS mixer channel with same logic as servoUpdate()
// Returns processed value in microseconds. state is the calling output's
// speed-limit state; sbusOutBeginFrame() must have run for this frame.
float sbusOutGetValueMixer(uint8_t channel, sbusOutSpeedState_t *state)
{
    if (channel >= SBUS_OUT_CHANNELS)
        return 0;
    
    const uint8_t servoIndex = BUS_SERVO_OFFSET + channel;
    
    if (servoIndex >= MAX_SUPPORTED_SERVOS)
        return 0;

    const servoParam_t *servo = servoParams(servoIndex);

    float pos = sbusOutGetInput(channel, servo);

    // Apply speed limiting
    if (servo->speed > 0) {
        if (mixerIsCyclicServo(sbusOutMixerIndex(channel))) {
            // Cyclic ratio worked out for this frame by sbusOutBeginFrame()
            pos = sbusLimitRatio(state->pos[channel], pos, state->cyclicRatio);
        }
        else {
            pos = sbusLimitSpeed(state->pos[channel], pos, servo->speed, state->dt);
        }
    }

    // Store input for next iteration
    state->pos[channel] = pos;

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
void sbusOutProcessMixerChannels(float output[SBUS_OUT_CHANNELS], sbusOutSpeedState_t *state)
{
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++) {
        output[ch] = sbusOutGetValueMixer(ch, state);
    }
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
    if (!sbusOutPort)
        return;

    // Check TX Buff is free
    if (serialTxBytesFree(sbusOutPort) <= sizeof(sbusOutFrame_t))
        return;

    sbusOutBeginFrame(&sbusOutSpeedState, currentTimeUs, sbusOutConfig()->frameRate, SBUS_OUT_CHANNELS);

    // Process all mixer channels with servoUpdate() logic
    float mixerOutputs[SBUS_OUT_CHANNELS];
    sbusOutProcessMixerChannels(mixerOutputs, &sbusOutSpeedState);

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

bool sbusOutIsEnabled(void)
{ 
    return sbusOutPort != NULL;
}

void sbusOutInit(void)
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
