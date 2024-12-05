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
#include "flight/mixer.h"
#include "io/serial.h"
#include "pg/sbus_output.h"
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

static float sbusOutGetRX(uint8_t channel)
{
    if (channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)
        return rcChannel[channel];
    return 0;
}

// Note: this returns 1000x normalized value (-1000 - 1000).
static float sbusOutGetValueMixer(uint8_t channel)
{
    if (channel < MIXER_OUTPUT_COUNT)
        return 1000.0f * mixerGetOutput(channel);
    return 0;
}

static float sbusOutGetServo(uint8_t channel)
{
    if (channel < MAX_SUPPORTED_SERVOS)
        return getServoOutput(channel);
    return 0;
}

// Note: this returns 0 - 1000 range (or -1000 - 1000 for bidirectional
// motor)
static float sbusOutGetMotor(uint8_t channel)
{
    if (channel < MAX_SUPPORTED_MOTORS)
        return getMotorOutput(channel);
    return 0;
}

STATIC_UNIT_TESTED float sbusOutGetChannelValue(uint8_t channel)
{
    const sbusOutSourceType_e source_type =
        sbusOutConfig()->sourceType[channel];
    const uint8_t source_index = sbusOutConfig()->sourceIndex[channel];
    switch (source_type) {
    case SBUS_OUT_SOURCE_NONE:
        return 0;
    case SBUS_OUT_SOURCE_RX:
        return sbusOutGetRX(source_index);
    case SBUS_OUT_SOURCE_MIXER:
        return sbusOutGetValueMixer(source_index);
    case SBUS_OUT_SOURCE_SERVO:
        return sbusOutGetServo(source_index);
    case SBUS_OUT_SOURCE_MOTOR:
        return sbusOutGetMotor(source_index);
    }
    return 0;
}

STATIC_UNIT_TESTED uint16_t sbusOutConvertToSbus(uint8_t channel, float pwm)
{
    const int16_t low  = sbusOutConfig()->sourceRangeLow[channel];
    const int16_t high = sbusOutConfig()->sourceRangeHigh[channel];

    // round and bound values
    if (channel >= 16) {
        const float value = scaleRangef(pwm, low, high, 0, 1);
        return constrain(nearbyintf(value), 0, 1);
    }
    const float value = scaleRangef(pwm, low, high, 192, 1792);
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
        SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED | SERIAL_UNIDIR |
            (sbusOutConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}
