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

#include "platform.h"

#ifdef USE_SERIALRX_IBUS2

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "build/atomic.h"

#include "common/crc.h"
#include "common/maths.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "pg/serial.h"

#include "rx/ibus2.h"
#include "rx/ibus2_telemetry.h"
#include "rx/rx.h"

#define IBUS2_BAUDRATE 1500000
#define IBUS2_FIRST_FRAME_MIN_LEN 4
#define IBUS2_FIRST_FRAME_PAYLOAD_MAX_LEN 33
#define IBUS2_UNPACK_TAIL_READ_BYTES 4
#define IBUS2_FIRST_FRAME_MAX_LEN (IBUS2_FIRST_FRAME_PAYLOAD_MAX_LEN + 4)
#define IBUS2_PACKED_BUFFER_LEN (IBUS2_FIRST_FRAME_PAYLOAD_MAX_LEN + IBUS2_UNPACK_TAIL_READ_BYTES)
#define IBUS2_COMMAND_FRAME_LEN 21
#define IBUS2_BASE_CHANNEL_COUNT 18
#define IBUS2_CHANNEL_COUNT 32
#define IBUS2_BASE_CHANNELS_12BIT_PAYLOAD_LEN ((IBUS2_BASE_CHANNEL_COUNT * 12U) / 8U)
#define IBUS2_CHANNEL_TYPES_LENGTH 20
#define IBUS2_CHANNEL_RANGE_100 16384
#define IBUS2_CHANNEL_RANGE_150 ((IBUS2_CHANNEL_RANGE_100 * 150) / 100)
#define IBUS2_HEADER_PACKET_TYPE_MASK 0x03
#define IBUS2_HEADER_SUBTYPE_MASK 0x3C
#define IBUS2_HEADER_SYNC_LOST_MASK 0x40
#define IBUS2_HEADER_FAILSAFE_MASK 0x80
#define IBUS2_ADDRESS_RESERVED_MASK 0xC0
#define IBUS2_INTERFRAME_GAP_MIN_US 70
#define IBUS2_DEBUG_PRINT_INTERVAL_US 1000000
#define IBUS2_CHANNEL_TYPE_BITS 5
#define IBUS2_CHANNEL_TYPE_BITS_MASK 0x0F
#define IBUS2_KEEP_FAILSAFE_CHANNEL (-32768)
#define IBUS2_STOP_FAILSAFE_CHANNEL (-32767)
#define IBUS2_CHANNEL_US_MIN 750
#define IBUS2_CHANNEL_US_MAX 2250

static const uint32_t ibus2UnpackChannelFactors[] = {
    0, 0, 0x40000000, 0, 0, 0, 0x028F5C29, 0x0147AE15,
    0x0083126F, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0x00041894, 0, 0,
    0, 0, 0, 0x20000000, 0x10000000, 0x06666667, 0x03333334, 0,
    0x0147AE15, 0x00A3D70B, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0, 0
};

typedef struct {
    uint8_t buf[IBUS2_FIRST_FRAME_MAX_LEN];
    uint8_t idx;
    uint8_t expectedLen;
    bool allowStart;
    timeUs_t lastByteTimeUs;
} ibus2FrameParser_t;

typedef struct {
    uint8_t buf[IBUS2_FIRST_FRAME_MAX_LEN];
    volatile uint8_t len;
    volatile timeUs_t receivedAtUs;
    volatile bool pending;
} ibus2PendingFirstFrame_t;

typedef struct {
    uint8_t buf[IBUS2_COMMAND_FRAME_LEN];
    volatile timeUs_t receivedAtUs;
    volatile bool pending;
} ibus2PendingCommandFrame_t;

typedef struct {
    uint32_t channelData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    volatile bool frameDone;
    volatile bool frameFailsafe;
    volatile bool frameSyncLost;
    volatile timeUs_t lastFrameTimeUs;
    serialPort_t *rxSerialPort;
    uint8_t channelTypes[IBUS2_CHANNEL_TYPES_LENGTH + 1];
    bool haveChannelTypes;
    uint8_t packedChannels[IBUS2_PACKED_BUFFER_LEN];
    uint8_t packedFailsafe[IBUS2_PACKED_BUFFER_LEN];
    bool havePackedChannels;
    bool lastPackedSyncLost;
    bool lastPackedFailsafe;
    timeUs_t lastPackedTimeUs;
    timeUs_t nextDebugPrintUs;
    uint32_t subtype0FrameCount;
    uint32_t subtype1FrameCount;
    uint32_t subtype2FrameCount;
    uint32_t firstFrameCrcErrorCount;
    uint32_t commandFrameCount;
    uint32_t commandFrameCrcErrorCount;
    ibus2FrameParser_t frameParser;
    ibus2PendingFirstFrame_t pendingFirstFrame;
    ibus2PendingCommandFrame_t pendingCommandFrame;
} ibus2State_t;

static ibus2State_t ibus2State = {
    .frameParser = {
        .allowStart = true,
    },
};

static bool ibus2CrcOk(const uint8_t *frame, size_t frameLen)
{
    return frameLen >= 2 && crc8_update(0xFF, frame, frameLen - 1, 0x25) == frame[frameLen - 1];
}

static inline uint16_t readU16Unaligned(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline uint32_t readU32Unaligned(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static void ibus2ResetState(void)
{
    memset(&ibus2State, 0, sizeof(ibus2State));
    ibus2State.frameParser.allowStart = true;
}

static void ibus2DebugPrintHeartbeat(timeUs_t nowUs)
{
    UNUSED(nowUs);
}

static void SES_UnpackChannels(const uint8_t *packedChannels, int16_t *channelsOut, uint8_t channelCount, const uint8_t *channelsType)
{
    const uint8_t *channelIn = packedChannels;
    uint32_t channelInBitNb = 0;
    uint32_t channelsTypeBitNb = 0;

    for (uint8_t channel = 0; channel < channelCount; channel++) {
        const uint32_t channelType = (readU16Unaligned(channelsType) >> channelsTypeBitNb) & ((1U << IBUS2_CHANNEL_TYPE_BITS) - 1U);
        const uint32_t nbBits = channelType & IBUS2_CHANNEL_TYPE_BITS_MASK;
        int32_t channelOutValue = 0;

        channelsTypeBitNb += IBUS2_CHANNEL_TYPE_BITS;
        if (channelsTypeBitNb >= 8) {
            channelsType++;
            channelsTypeBitNb -= 8;
        }

        if (nbBits >= 2) {
            int32_t channelInValue = (int32_t)(readU32Unaligned(channelIn) >> channelInBitNb) & (int32_t)((1U << nbBits) - 1U);
            if (channelInValue == (1L << (nbBits - 1))) {
                channelOutValue = IBUS2_KEEP_FAILSAFE_CHANNEL;
            } else if (channelInValue == ((1L << (nbBits - 1)) + 1) && nbBits >= 6) {
                channelOutValue = IBUS2_STOP_FAILSAFE_CHANNEL;
            } else {
                bool negative = false;

                if (channelInValue & (1L << (nbBits - 1))) {
                    negative = true;
                    channelInValue = (-channelInValue) & ((1L << (nbBits - 1)) - 1L);
                }

                channelOutValue = (int32_t)((channelInValue * (int32_t)ibus2UnpackChannelFactors[channelType] + (1L << 15)) >> 16);
                if (channelOutValue > IBUS2_CHANNEL_RANGE_150) {
                    channelOutValue = IBUS2_CHANNEL_RANGE_150;
                }
                if (negative) {
                    channelOutValue = -channelOutValue;
                }
            }
        }

        channelsOut[channel] = (int16_t)channelOutValue;
        channelInBitNb += nbBits;
        while (channelInBitNb >= 8) {
            channelIn++;
            channelInBitNb -= 8;
        }
    }
}

static uint32_t ibus2ExtractBitsLE(const uint8_t *data, size_t bitOffset, uint8_t bitCount, size_t dataBytes)
{
    uint32_t value = 0;

    for (uint8_t bit = 0; bit < bitCount; bit++) {
        const size_t index = bitOffset + bit;
        const size_t byteIndex = index >> 3;
        if (byteIndex >= dataBytes) {
            break;
        }

        value |= (uint32_t)(((data[byteIndex] >> (index & 7)) & 0x01) << bit);
    }

    return value;
}

static int16_t ibus2SignExtend12(uint16_t value)
{
    return (value & 0x0800U) ? (int16_t)(value | 0xF000U) : (int16_t)value;
}

static uint16_t ibus2ConvertChannelToUs(int16_t channelValue, uint16_t currentValue)
{
    if (channelValue == IBUS2_KEEP_FAILSAFE_CHANNEL || channelValue == IBUS2_STOP_FAILSAFE_CHANNEL) {
        return currentValue;
    }

    int32_t scaledValue = ((int32_t)(channelValue + 49152) * 8000 + (1 << 15)) >> 16;
    if (scaledValue < (IBUS2_CHANNEL_US_MIN * 4)) {
        scaledValue = IBUS2_CHANNEL_US_MIN * 4;
    }
    if (scaledValue > (IBUS2_CHANNEL_US_MAX * 4)) {
        scaledValue = IBUS2_CHANNEL_US_MAX * 4;
    }

    return (uint16_t)(scaledValue / 4);
}

static bool ibus2DecodePacked12Fallback(const uint8_t *payload, size_t payloadLen, timeUs_t nowUs, bool syncLost, bool failsafe)
{
    const size_t channelCount = (payloadLen >= IBUS2_BASE_CHANNELS_12BIT_PAYLOAD_LEN) ? IBUS2_BASE_CHANNEL_COUNT : (payloadLen * 8U) / 12U;

    if (channelCount < 4) {
        return false;
    }

    for (size_t i = 0; i < channelCount && i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        const int16_t signedRaw = ibus2SignExtend12((uint16_t)ibus2ExtractBitsLE(payload, i * 12U, 12, payloadLen));
        const int32_t channelValue = constrain(1500 + (signedRaw / 2), IBUS2_CHANNEL_US_MIN, IBUS2_CHANNEL_US_MAX);

        ibus2State.channelData[i] = channelValue;
    }

    ibus2State.frameDone = true;
    ibus2State.frameFailsafe = failsafe;
    ibus2State.frameSyncLost = syncLost;
    ibus2State.lastFrameTimeUs = nowUs;

    return true;
}

static bool ibus2DecodeStoredChannels(timeUs_t nowUs, bool syncLost, bool failsafe)
{
    int16_t unpackedChannels[IBUS2_CHANNEL_COUNT] = { 0 };

    if (!ibus2State.haveChannelTypes) {
        return false;
    }

    SES_UnpackChannels(ibus2State.packedChannels, unpackedChannels, IBUS2_CHANNEL_COUNT, ibus2State.channelTypes);

    const uint8_t channelCount = MIN(MAX_SUPPORTED_RC_CHANNEL_COUNT, IBUS2_CHANNEL_COUNT);
    for (uint8_t i = 0; i < channelCount; i++) {
        ibus2State.channelData[i] = ibus2ConvertChannelToUs(unpackedChannels[i], (uint16_t)ibus2State.channelData[i]);
    }

    ibus2State.frameDone = true;
    ibus2State.frameFailsafe = failsafe;
    ibus2State.frameSyncLost = syncLost;
    ibus2State.lastFrameTimeUs = nowUs;

    return true;
}

static void ibus2ProcessFirstFrame(const uint8_t *frame, size_t frameLen, timeUs_t nowUs)
{
    if (frameLen < IBUS2_FIRST_FRAME_MIN_LEN || frame[1] != frameLen) {
        return;
    }

    if ((frame[0] & IBUS2_HEADER_PACKET_TYPE_MASK) != 0 || (frame[2] & IBUS2_ADDRESS_RESERVED_MASK) != 0) {
        return;
    }

    if (!ibus2CrcOk(frame, frameLen)) {
        ibus2State.firstFrameCrcErrorCount++;
        return;
    }

    ibus2TelemetryUpdateAddress(frame, frameLen);

    const uint8_t packetSubtype = (frame[0] & IBUS2_HEADER_SUBTYPE_MASK) >> 2;
    const bool syncLost = (frame[0] & IBUS2_HEADER_SYNC_LOST_MASK) != 0;
    const bool failsafe = (frame[0] & IBUS2_HEADER_FAILSAFE_MASK) != 0;
    const uint8_t *payload = &frame[3];
    const uint8_t payloadLen = frame[1] - 4;

    switch (packetSubtype) {
    case 0:
        ibus2State.subtype0FrameCount++;
        memset(ibus2State.packedChannels, 0, sizeof(ibus2State.packedChannels));
        memcpy(ibus2State.packedChannels, payload, payloadLen < sizeof(ibus2State.packedChannels) ? payloadLen : sizeof(ibus2State.packedChannels));
        ibus2State.havePackedChannels = true;
        ibus2State.lastPackedSyncLost = syncLost;
        ibus2State.lastPackedFailsafe = failsafe;
        ibus2State.lastPackedTimeUs = nowUs;
        if (ibus2State.haveChannelTypes) {
            ibus2DecodeStoredChannels(nowUs, syncLost, failsafe);
        } else {
            ibus2DecodePacked12Fallback(payload, payloadLen, nowUs, syncLost, failsafe);
        }
        break;

    case 1:
        ibus2State.subtype1FrameCount++;
        if (payloadLen < IBUS2_CHANNEL_TYPES_LENGTH) {
            break;
        }
        memcpy(ibus2State.channelTypes, payload, IBUS2_CHANNEL_TYPES_LENGTH);
        ibus2State.haveChannelTypes = true;
        if (ibus2State.havePackedChannels) {
            ibus2DecodeStoredChannels(ibus2State.lastPackedTimeUs, ibus2State.lastPackedSyncLost, ibus2State.lastPackedFailsafe);
        }
        break;

    case 2:
        ibus2State.subtype2FrameCount++;
        memset(ibus2State.packedFailsafe, 0, sizeof(ibus2State.packedFailsafe));
        memcpy(ibus2State.packedFailsafe, payload, payloadLen < sizeof(ibus2State.packedFailsafe) ? payloadLen : sizeof(ibus2State.packedFailsafe));
        break;

    default:
        break;
    }
}

static void ibus2ProcessCommandFrame(const uint8_t *frame, size_t frameLen, timeUs_t receivedAtUs)
{
    if (frameLen != IBUS2_COMMAND_FRAME_LEN) {
        return;
    }

    if ((frame[0] & IBUS2_HEADER_PACKET_TYPE_MASK) != 1) {
        return;
    }

    if (!ibus2CrcOk(frame, frameLen)) {
        ibus2State.commandFrameCrcErrorCount++;
        return;
    }

    ibus2State.commandFrameCount++;
    ibus2TelemetryQueueCommand(frame, frameLen, receivedAtUs);
}

static void ibus2QueueFirstFrameFromIsr(const uint8_t *frame, uint8_t frameLen, timeUs_t receivedAtUs)
{
    if (ibus2State.pendingFirstFrame.pending) {
        return;
    }

    memcpy(ibus2State.pendingFirstFrame.buf, frame, frameLen);
    ibus2State.pendingFirstFrame.len = frameLen;
    ibus2State.pendingFirstFrame.receivedAtUs = receivedAtUs;
    ibus2State.pendingFirstFrame.pending = true;
}

static void ibus2QueueCommandFrameFromIsr(const uint8_t *frame, timeUs_t receivedAtUs)
{
    if (ibus2State.pendingCommandFrame.pending) {
        return;
    }

    memcpy(ibus2State.pendingCommandFrame.buf, frame, IBUS2_COMMAND_FRAME_LEN);
    ibus2State.pendingCommandFrame.receivedAtUs = receivedAtUs;
    ibus2State.pendingCommandFrame.pending = true;
}

static bool ibus2PendingFrameProcessingRequired(void)
{
    return ibus2State.pendingFirstFrame.pending || ibus2State.pendingCommandFrame.pending;
}

static void ibus2ProcessPendingFrames(void)
{
    uint8_t commandFrame[IBUS2_COMMAND_FRAME_LEN];
    timeUs_t commandReceivedAtUs = 0;
    bool haveCommandFrame = false;

    uint8_t firstFrame[IBUS2_FIRST_FRAME_MAX_LEN];
    uint8_t firstFrameLen = 0;
    timeUs_t firstFrameReceivedAtUs = 0;
    bool haveFirstFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (ibus2State.pendingCommandFrame.pending) {
            memcpy(commandFrame, ibus2State.pendingCommandFrame.buf, sizeof(commandFrame));
            commandReceivedAtUs = ibus2State.pendingCommandFrame.receivedAtUs;
            ibus2State.pendingCommandFrame.pending = false;
            haveCommandFrame = true;
        }

        if (ibus2State.pendingFirstFrame.pending) {
            firstFrameLen = ibus2State.pendingFirstFrame.len;
            memcpy(firstFrame, ibus2State.pendingFirstFrame.buf, firstFrameLen);
            firstFrameReceivedAtUs = ibus2State.pendingFirstFrame.receivedAtUs;
            ibus2State.pendingFirstFrame.pending = false;
            haveFirstFrame = true;
        }
    }

    if (haveCommandFrame) {
        ibus2ProcessCommandFrame(commandFrame, sizeof(commandFrame), commandReceivedAtUs);
    }

    if (haveFirstFrame) {
        ibus2ProcessFirstFrame(firstFrame, firstFrameLen, firstFrameReceivedAtUs);
    }
}

static void ibus2FinalizePendingFrame(timeUs_t nowUs)
{
    if (ibus2State.frameParser.idx == 0) {
        return;
    }

    ibus2State.frameParser.idx = 0;
    ibus2State.frameParser.expectedLen = 0;
    ibus2State.frameParser.allowStart = true;
    ibus2State.frameParser.lastByteTimeUs = nowUs;
}

static void ibus2ProcessByte(uint8_t byte)
{
    const timeUs_t nowUs = microsISR();

    if (ibus2State.frameParser.lastByteTimeUs &&
        cmpTimeUs(nowUs, ibus2State.frameParser.lastByteTimeUs) > IBUS2_INTERFRAME_GAP_MIN_US) {
        ibus2FinalizePendingFrame(nowUs);
        ibus2State.frameParser.allowStart = true;
    }
    ibus2State.frameParser.lastByteTimeUs = nowUs;

    if (ibus2State.frameParser.idx == 0) {
        if (!ibus2State.frameParser.allowStart) {
            return;
        }
        const uint8_t packetType = byte & IBUS2_HEADER_PACKET_TYPE_MASK;
        if (packetType > 1) {
            ibus2State.frameParser.allowStart = false;
            return;
        }
        ibus2State.frameParser.buf[ibus2State.frameParser.idx++] = byte;
        ibus2State.frameParser.expectedLen = packetType == 1 ? IBUS2_COMMAND_FRAME_LEN : 0;
        return;
    }

    if (ibus2State.frameParser.idx >= sizeof(ibus2State.frameParser.buf)) {
        ibus2FinalizePendingFrame(nowUs);
        return;
    }

    ibus2State.frameParser.buf[ibus2State.frameParser.idx++] = byte;

    if (ibus2State.frameParser.expectedLen == IBUS2_COMMAND_FRAME_LEN) {
        if (ibus2State.frameParser.idx == ibus2State.frameParser.expectedLen) {
            ibus2QueueCommandFrameFromIsr(ibus2State.frameParser.buf, nowUs);
            ibus2State.frameParser.idx = 0;
            ibus2State.frameParser.expectedLen = 0;
            ibus2State.frameParser.allowStart = false;
        }
        return;
    }

    if (ibus2State.frameParser.idx == 2) {
        ibus2State.frameParser.expectedLen = byte;
        if (byte < IBUS2_FIRST_FRAME_MIN_LEN || byte > IBUS2_FIRST_FRAME_MAX_LEN) {
            ibus2FinalizePendingFrame(nowUs);
            ibus2State.frameParser.allowStart = false;
        }
        return;
    }

    if (ibus2State.frameParser.expectedLen && ibus2State.frameParser.idx == ibus2State.frameParser.expectedLen) {
        ibus2QueueFirstFrameFromIsr(ibus2State.frameParser.buf, ibus2State.frameParser.expectedLen, nowUs);
        ibus2State.frameParser.idx = 0;
        ibus2State.frameParser.expectedLen = 0;
        ibus2State.frameParser.allowStart = false;
    }
}

static void ibus2DataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    ibus2ProcessByte((uint8_t)c);
}

static uint8_t ibus2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibus2State.frameDone) {
        if (ibus2PendingFrameProcessingRequired() || ibus2TelemetryPending()) {
            return RX_FRAME_PROCESSING_REQUIRED;
        }
        return frameStatus;
    }

    ibus2State.frameDone = false;

    if (ibus2State.frameFailsafe) {
        frameStatus = RX_FRAME_FAILSAFE;
    } else if (ibus2State.frameSyncLost) {
        frameStatus = RX_FRAME_DROPPED;
    } else {
        frameStatus = RX_FRAME_COMPLETE;
    }

    rxRuntimeState->lastRcFrameTimeUs = ibus2State.lastFrameTimeUs;
    ibus2DebugPrintHeartbeat(ibus2State.lastFrameTimeUs);
    if (ibus2PendingFrameProcessingRequired() || ibus2TelemetryPending()) {
        frameStatus |= RX_FRAME_PROCESSING_REQUIRED;
    }
    return frameStatus;
}

static float ibus2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibus2State.channelData[chan];
}

static bool ibus2ProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    ibus2ProcessPendingFrames();
    return ibus2TelemetryProcess(micros());
}

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = IBUS2_BASE_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000;
    rxRuntimeState->rcReadRawFn = ibus2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibus2FrameStatus;
    rxRuntimeState->rcProcessFrameFn = ibus2ProcessFrame;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    ibus2ResetState();

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        ibus2State.channelData[i] = 1500;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort_t *ibusPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibus2DataReceive,
        NULL,
        IBUS2_BAUDRATE,
        MODE_RXTX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_BIDIR | SERIAL_BIDIR_PP |
            (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
    );

    ibus2State.rxSerialPort = ibusPort;

    if (ibusPort) {
        ibus2TelemetryInit(ibusPort);
    }

    return ibusPort != NULL;
}

serialPort_t *ibus2GetRxSerialPort(void)
{
    return ibus2State.rxSerialPort;
}

#endif
