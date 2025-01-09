/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * XBUS Mode B and RJ01 implementation from Betaflight
 * XBUS Mode A implementation: Nigel Higgs
 *
 * CRC8 code courtesy of JR XBUS specifications <https://www.jrpropo-jp.com/xbus-specification-en>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_XBUS

#include "common/crc.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/xbus.h"

//
// Serial driver for JR's XBus (MODE B) receiver
//

#define XBUS_CHANNEL_COUNT 12
#define XBUS_RJ01_CHANNEL_COUNT 12

// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define XBUS_FRAME_SIZE_A1 27
#define XBUS_FRAME_SIZE_A2 35

#define XBUS_RJ01_FRAME_SIZE 33
#define XBUS_RJ01_MESSAGE_LENGTH 30
#define XBUS_RJ01_OFFSET_BYTES 3

#define XBUS_BAUDRATE 115200
#define XBUS_RJ01_BAUDRATE 250000
#define XBUS_MAX_FRAME_TIME 8000

// NOTE!
// This is actually based on ID+LENGTH (nibble each)
// 0xA - Multiplex ID (also used by JR, no idea why)
// 0x1 - 12 channels
// 0x2 - 16 channels
// However, the JR XG14 that is used for test at the moment
// does only use 0xA1 as its output. This is why the implementation
// is based on these numbers only. Maybe update this in the future?
#define XBUS_START_OF_FRAME_BYTE_A1 (0xA1)      //12 channels
#define XBUS_START_OF_FRAME_BYTE_A2 (0xA2)      //16 channels transfare, but only 12 channels use for

// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
#define XBUS_CONVERT_TO_USEC(V) (800 + ((V * 1400) >> 12))

// XBus Mode A additions
#define XBUS_MODEA_CHANNEL_COUNT 16
// Frame is: Frame(1 byte) + Length(1 byte) + Key and type (2 bytes - not used) + 16*channel(4 bytes) + CRC(1 byte)
#define XBUS_MODEA_FRAME_SIZE 69
#define XBUS_MODEA_OFFSET_BYTES 4
#define XBUS_MODEA_BAUDRATE 250000
#define XBUS_MODEA_MAX_FRAME_TIME 2700 // 2760us round up to 2800 (69/[250k/10bits]=2760)
#define XBUS_MODEA_START_OF_FRAME_BYTE (0xA4)
// Mode A needs a different conversion as the resolution is 16bit, not 12bit
//0x0000      800uSec
//0x1249      900uSec(-60°, -75°, or -90°)
//0x7FFF      1500uSec(0°)
//0xEDB6      2100uSec(+60°, +75°, or +90°)
//0xFFFF      2200uSec
#define XBUS_MODEA_CONVERT_TO_USEC(V) (800 + ((V * 1400) >> 16))
// As Mode A has a difference frame time, lets set it in a global
static uint32_t xBusMaxFrameTime;

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;
static uint8_t xBusFrameLength;
static uint8_t xBusChannelCount;
static uint8_t xBusProvider;

// Use max values for ram areas
static volatile uint8_t xBusFrame[XBUS_MODEA_FRAME_SIZE];  // size 69 for 16 channels in xbus_Mode_A
static uint16_t xBusChannelData[XBUS_MODEA_CHANNEL_COUNT];

// xBus Mode A uses an 8-bit CRC based on Dallas/Maxim
// Using a lookup table to help calculate the CRC is slightly quicker
static uint8_t s_crc_array[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static uint8_t crc_table(uint8_t data, uint8_t crc)
{
    uint16_t index = (data ^ crc) & 0xff;
    crc = s_crc_array[index];
    return crc;
}

uint8_t crc8(uint8_t* buffer, uint8_t length)
{
    uint8_t crc = 0;

    while (length-- > 0)
        crc = crc_table(*buffer++, crc);

    return crc;
}

// Full RJ01 message CRC calculations
static uint8_t xBusRj01CRC8(uint8_t inData, uint8_t seed)
{
    for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        const uint8_t temp = ((seed ^ inData) & 0x01);

        if (temp == 0) {
            seed >>= 1;
        } else {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }

        inData >>= 1;
    }

    return seed;
}

static void xBusUnpackModeAFrame(uint8_t offsetBytes)
{

    //
    // Check we have correct length of message
    //
    if (xBusFrame[1] != XBUS_MODEA_FRAME_SIZE - 3)
    {
        // Unknown package as length is not ok
        return;
    }

    // Calculate the CRC according to JR XBus Specs
    // Using a CRC lookup table is faster than without
    if (crc8((uint8_t*)xBusFrame, xBusFrame[1] + 2) == xBusFrame[xBusFrame[1] + 2])
    {
        // Need to do a check on bytes 2 and 3. 
        // When failsafe, byte 2 goes from >0 to 0 and byte 3 goes from 0 to a value that appears to be dependant on the TX
        // XG14 goes 0x1c 0x00 to 0x00 0x80
        // Elite goes 0x17 0x00 to 0x00 0x80
        if (xBusFrame[2] != 0x00 && xBusFrame[3] != 0x80)
        {
            // Unpack the data, as we have a valid frame
            // As Mode A can address up to 50 channels, we need to loop through the channel
            // data and update the corresponding channel. The reason for this is that the 
            // number of the channel may not always be in the same spot in the packet. Also
            // there are only 16 channels of data sent per packet
            for (int i = 0; i <= xBusChannelCount; i++)
            {
                // Channel packets are constructed as such:
                // Byte 0 - Channel number
                // Byte 1 - CH Function (unused so ignore)
                // Byte 2 - CH Data-High
                // Byte 3 - CH Data-Low
                // Channel number is 0 based, so we need to subtract 1 from the value
                const uint8_t nChannelNumber = xBusFrame[offsetBytes + i * 4] - 1;
                const uint8_t frameAddr = offsetBytes + 2 + i * 4;
                uint16_t value = ((uint16_t)xBusFrame[frameAddr]) << 8;
                value += ((uint16_t)xBusFrame[frameAddr + 1]);

                if (value >= 0xFFFF) //65535
                    value = 0xFFFF;

                // Convert to internal format
                if (nChannelNumber <= XBUS_MODEA_CHANNEL_COUNT)
                {
                    uint16_t val = XBUS_MODEA_CONVERT_TO_USEC(value);

                    xBusChannelData[nChannelNumber] = val;
                }
            }
            xBusFrameReceived = true;
        }
    }
}

static void xBusUnpackModeBFrame(uint8_t offsetBytes)
{
    // Calculate the CRC of the incoming frame
    // Calculate on all bytes except the final two CRC bytes
    const uint16_t inCrc = crc16_ccitt_update(0, (uint8_t*)&xBusFrame[offsetBytes], xBusFrameLength - 2);

    // Get the received CRC
    const uint16_t crc = (((uint16_t)xBusFrame[offsetBytes + xBusFrameLength - 2]) << 8) + ((uint16_t)xBusFrame[offsetBytes + xBusFrameLength - 1]);

    if (crc == inCrc) {
        // Unpack the data, we have a valid frame, only 12 channel unpack also when receive 16 channel
        for (int i = 0; i < xBusChannelCount; i++) {

            const uint8_t frameAddr = offsetBytes + 1 + i * 2;
            uint16_t value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            // Convert to internal format
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

        xBusFrameReceived = true;
    }
}

static void xBusUnpackRJ01Frame(void)
{
    // Calculate the CRC of the incoming frame
    uint8_t outerCrc = 0;
    uint8_t i = 0;

    // When using the Align RJ01 receiver with
    // a MODE B setting in the radio (XG14 tested)
    // the MODE_B -frame is packed within some
    // at the moment unknown bytes before and after:
    // 0xA1 LEN __ 0xA1 12*(High + Low) CRC1 CRC2 + __ __ CRC_OUTER
    // Compared to a standard MODE B frame that only
    // contains the "middle" package.
    // Hence, at the moment, the unknown header and footer
    // of the RJ01 MODEB packages are discarded.
    // However, the LAST byte (CRC_OUTER) is infact an 8-bit
    // CRC for the whole package, using the Dallas-One-Wire CRC
    // method.
    // So, we check both these values as well as the provided length
    // of the outer/full message (LEN)

    //
    // Check we have correct length of message
    //
    if (xBusFrame[1] != XBUS_RJ01_MESSAGE_LENGTH)
    {
        // Unknown package as length is not ok
        return;
    }

    //
    // CRC calculation & check for full message
    //
    for (i = 0; i < xBusFrameLength - 1; i++) {
        outerCrc = xBusRj01CRC8(outerCrc, xBusFrame[i]);
    }

    if (outerCrc != xBusFrame[xBusFrameLength - 1])
    {
        // CRC does not match, skip this frame
        return;
    }

    // Now unpack the "embedded MODE B frame"
    xBusUnpackModeBFrame(XBUS_RJ01_OFFSET_BYTES);
}

// Receive ISR callback
static void xBusDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint32_t now;
    static uint32_t xBusTimeLast, xBusTimeInterval;

    // Check if we shall reset frame position due to time
    now = micros();
    xBusTimeInterval = now - xBusTimeLast;
    xBusTimeLast = now;
    if (xBusTimeInterval > xBusMaxFrameTime) {
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }

    // Check if we shall start a frame?
    if (xBusFramePosition == 0) {
        if (xBusProvider == SERIALRX_XBUS_MODE_B && c == XBUS_START_OF_FRAME_BYTE_A1) {
            xBusDataIncoming = true;
            xBusFrameLength = XBUS_FRAME_SIZE_A1;   //decrease framesize (when receiver change, otherwise board must reboot)
        } else if (xBusProvider == SERIALRX_XBUS_MODE_B_RJ01 && c == XBUS_START_OF_FRAME_BYTE_A2) {//16channel packet
            xBusDataIncoming = true;
            xBusFrameLength = XBUS_FRAME_SIZE_A2;   //increase framesize
        } else if (xBusProvider == SERIALRX_XBUS_MODE_A && c == XBUS_MODEA_START_OF_FRAME_BYTE) { //16channel packet for Mode A
            xBusDataIncoming = true;
            xBusFrameLength = XBUS_MODEA_FRAME_SIZE;   //increase framesize
        }
    }

    // Only do this if we are receiving to a frame
    if (xBusDataIncoming == true) {
        // Store in frame copy
        xBusFrame[xBusFramePosition] = (uint8_t)c;

        // If we are running xBus Mode A, then check the length of the packet and
        // adjust the frame length accordingly.
        // Packet length is the second byte of the array
        if (xBusProvider == SERIALRX_XBUS_MODE_A && xBusFramePosition == 1) {
            xBusFrameLength = (uint8_t)c + 3;   //adjust framesize
        }
        xBusFramePosition++;
    }

    // Done?
    if (xBusFramePosition == xBusFrameLength) {
        switch (xBusProvider) {
        case SERIALRX_XBUS_MODE_B:
            xBusUnpackModeBFrame(0);
            FALLTHROUGH; //!!TODO - check this fall through is correct
        case SERIALRX_XBUS_MODE_B_RJ01:
            xBusUnpackRJ01Frame();
            FALLTHROUGH; //!!TODO - check this fall through is correct
        case SERIALRX_XBUS_MODE_A:
            xBusUnpackModeAFrame(XBUS_MODEA_OFFSET_BYTES);
        }
        xBusDataIncoming = false;
        xBusFramePosition = 0;
    }
}

// Indicate time to read a frame from the data...
static uint8_t xBusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (!xBusFrameReceived) {
        return RX_FRAME_PENDING;
    }

    xBusFrameReceived = false;

    return RX_FRAME_COMPLETE;
}

static float xBusReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    uint16_t data;

    // Deliver the data wanted
    if (chan >= rxRuntimeState->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}

bool xBusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    uint32_t baudRate;

    xBusFrameReceived = false;
    xBusDataIncoming = false;
    xBusFramePosition = 0;

    rxRuntimeState->rcReadRawFn = xBusReadRawRC;
    rxRuntimeState->rcFrameStatusFn = xBusFrameStatus;

    switch (rxRuntimeState->serialrxProvider) {
    case SERIALRX_XBUS_MODE_A:
        rxRuntimeState->channelCount = XBUS_MODEA_CHANNEL_COUNT;
        rxRuntimeState->rxRefreshRate = 14000;
        xBusMaxFrameTime = XBUS_MODEA_MAX_FRAME_TIME;
        baudRate = XBUS_MODEA_BAUDRATE;
        xBusFrameLength = XBUS_MODEA_FRAME_SIZE;
        xBusChannelCount = XBUS_MODEA_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_A;
        break;
    case SERIALRX_XBUS_MODE_B:
        rxRuntimeState->channelCount = XBUS_CHANNEL_COUNT;
        rxRuntimeState->rxRefreshRate = 11000;
        xBusMaxFrameTime = XBUS_MAX_FRAME_TIME;
        baudRate = XBUS_BAUDRATE;
        xBusFrameLength = XBUS_FRAME_SIZE_A1;
        xBusChannelCount = XBUS_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B;
        break;
    case SERIALRX_XBUS_MODE_B_RJ01:
        rxRuntimeState->channelCount = XBUS_RJ01_CHANNEL_COUNT;
        rxRuntimeState->rxRefreshRate = 14000;
        xBusMaxFrameTime = XBUS_MAX_FRAME_TIME;
        baudRate = XBUS_RJ01_BAUDRATE;
        xBusFrameLength = XBUS_RJ01_FRAME_SIZE;
        xBusChannelCount = XBUS_RJ01_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B_RJ01;
        break;
    default:
        return false;
        break;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    serialPort_t *xBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        xBusDataReceive,
        NULL,
        baudRate,
        portShared ? MODE_RXTX : MODE_RX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
            (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = xBusPort;
    }
#endif

    return xBusPort != NULL;
}
#endif
