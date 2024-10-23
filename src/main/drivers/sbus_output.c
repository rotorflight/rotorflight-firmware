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

#include <stdbool.h>
#include <string.h>

#include "build/build_config.h"
#include "platform.h"

#include "common/maths.h"
#include "common/time.h"

#include "io/serial.h"

STATIC_UNIT_TESTED uint16_t sbusOutChannel[SBUS_OUT_CHANNELS];
STATIC_UNIT_TESTED serialPort_t *sbusOutPort = NULL;
STATIC_UNIT_TESTED timeUs_t sbusOutLastTxTimeUs = 0;

void sbusOutConfig(sbusOutChannel_t *channel, uint8_t index) {
    if (index >= 1 && index <= SBUS_OUT_CHANNELS) {
        *channel = index;
    } else {
        *channel = 0;
    }
}

void sbusOutSetOutput(sbusOutChannel_t *channel, uint16_t value) {
    if (*channel >= 1 && *channel <= SBUS_OUT_CHANNELS) {
        sbusOutChannel[*channel - 1] = value;
    }
}

STATIC_UNIT_TESTED void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame) {
    frame->syncByte = 0x0F;

    // There's no way to make a bit field array, so we have to go tedious.
    frame->chan0 = sbusOutChannel[0];
    frame->chan1 = sbusOutChannel[1];
    frame->chan2 = sbusOutChannel[2];
    frame->chan3 = sbusOutChannel[3];
    frame->chan4 = sbusOutChannel[4];
    frame->chan5 = sbusOutChannel[5];
    frame->chan6 = sbusOutChannel[6];
    frame->chan7 = sbusOutChannel[7];
    frame->chan8 = sbusOutChannel[8];
    frame->chan9 = sbusOutChannel[9];
    frame->chan10 = sbusOutChannel[10];
    frame->chan11 = sbusOutChannel[11];
    frame->chan12 = sbusOutChannel[12];
    frame->chan13 = sbusOutChannel[13];
    frame->chan14 = sbusOutChannel[14];
    frame->chan15 = sbusOutChannel[15];

    frame->flags = sbusOutChannel[16] ? 0x01 : 0x00;
    frame->flags += sbusOutChannel[17] ? 0x02 : 0x00;
    // Other flags?

    frame->endByte = 0;
}

void sbusOutUpdate(timeUs_t currentTimeUs) {
    static const timeUs_t sbusOutTxIntervalUs = 6000;
    if (sbusOutPort && 
        /* Tx Buff is free */ serialTxBytesFree(sbusOutPort) >
                              sizeof(sbusOutFrame_t) &&
        /* Tx interval check */ currentTimeUs - sbusOutLastTxTimeUs >
                                sbusOutTxIntervalUs) {
        sbusOutFrame_t frame;
        sbusOutPrepareSbusFrame(&frame);
        // serial output
        serialWriteBuf(sbusOutPort, (const uint8_t *)&frame, sizeof(frame));
        sbusOutLastTxTimeUs = currentTimeUs;
    }
}

void sbusOutInit() {
    memset(sbusOutChannel, 0, sizeof(sbusOutChannel));

    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_SBUS_OUT);

    if (!portConfig) {
        sbusOutPort = NULL;
        return;
    }

    sbusOutPort = openSerialPort(
        portConfig->identifier, FUNCTION_SBUS_OUT, NULL, NULL, 100000, MODE_TX,
        SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED |
            SERIAL_BIDIR | SERIAL_NOSWAP);
}

uint16_t sbusOutPwmToSbus(sbusOutChannel_t *channel, float pwm) {
    // This conversion doesn't work for narrow band servo channels.
    // Need user feedback on how this should be used.
    if (*channel >= 1 && *channel <= 16) {
        //  For full-scale channels, [192 - 1792] maps to [1000 - 2000]
        return constrainf(scaleRangef(pwm, 1000, 2000, 192, 1792), 0, 2047);
    }
    if (*channel >= 17 && *channel <= 18) {
        return (pwm > 1500);
    }
    return 0;
}
