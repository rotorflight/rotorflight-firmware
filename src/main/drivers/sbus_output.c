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
#include "flight/servos.h"
#include "io/serial.h"
#include "pg/bus_servo.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "platform.h"
#include "rx/rx.h"

STATIC_UNIT_TESTED serialPort_t *sbusOutPort = NULL;

STATIC_UNIT_TESTED void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame,
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

// Apply servo parameters and return value in microseconds
// Maps SBUS channels to servos S9-S26 (BUS servos)
float sbusOutGetValueMixer(uint8_t channel)
{
    // Map SBUS channel to bus servo index (S9 = index 8, S10 = index 9, etc.)
    const uint8_t servoIndex = BUS_SERVO_OFFSET + channel;
    
    if (servoIndex >= MAX_SUPPORTED_SERVOS)
        return 0;

    // Get normalized mixer output (-1.0 to 1.0), or servo override when disarmed
    float pos = 0;
    if (!ARMING_FLAG(ARMED) && hasServoOverride(servoIndex))
        pos = getServoOverride(servoIndex) / 1000.0f;
    else
        pos = mixerGetOutput(MIXER_SERVO_OFFSET + servoIndex - BUS_SERVO_OFFSET);
    const servoParam_t *servo = servoParams(servoIndex);

    // Apply servo reversal
    if (servo->flags & SERVO_FLAG_REVERSED)
        pos = -pos;

    // Apply servo scale (rneg/rpos)
    float scale = (pos > 0) ? servo->rpos : servo->rneg;
    pos = scale * pos;

    // Apply travel limits
    pos = constrainf(pos, servo->min, servo->max);

    // Add midpoint to get final microsecond value
    float result = servo->mid + pos;

    // Clamp to bus servo limits for SBUS output
    result = constrainf(result, servo->mid + BUS_SERVO_MIN, servo->mid + BUS_SERVO_MAX);

    return result;
}

STATIC_UNIT_TESTED float sbusOutGetChannelValue(uint8_t channel)
{
    const busServoSourceType_e source_type = busServoConfig()->sourceType[channel];
    switch (source_type) {
    case BUS_SERVO_SOURCE_RX:
        return sbusOutGetRX(channel);
    case BUS_SERVO_SOURCE_MIXER:
        return sbusOutGetValueMixer(channel);
    }
    return 0;
}

STATIC_UNIT_TESTED uint16_t sbusOutConvertToSbus(uint8_t channel, float pwm)
{
    // For digital channels (16-17), convert to 0 or 1
    if (channel >= 16) {
        // Assume threshold at 1500us
        return (pwm >= 1500) ? 1 : 0;
    }

    // For analog channels (0-15), convert microseconds to SBUS range (192-1792)
    // Bus servo range: (1500 + BUS_SERVO_MIN) to (1500 + BUS_SERVO_MAX) -> SBUS 192-1792
    const float value = scaleRangef(pwm, 1500 + BUS_SERVO_MIN, 1500 + BUS_SERVO_MAX, 192, 1792);
    return constrain(nearbyintf(value), 0, (1 << 11) - 1);
}

void sbusOutUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!sbusOutPort)
        return;

    // Check TX Buff is free
    if (serialTxBytesFree(sbusOutPort) <= sizeof(sbusOutFrame_t))
        return;

    // Start sending.
    sbusOutFrame_t frame;
    uint16_t channels[SBUS_OUT_CHANNELS];
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++) {
        float value = sbusOutGetChannelValue(ch);
        channels[ch] = sbusOutConvertToSbus(ch, value);
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
