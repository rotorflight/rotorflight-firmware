#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "pg/serial.h"

#include "rx/ibus2.h"
#include "rx/rx.h"

#define IBUS2_BAUDRATE 1500000
#define IBUS2_FIRST_FRAME_MIN_LEN 4
#define IBUS2_FIRST_FRAME_MAX_LEN 37
#define IBUS2_BASE_CHANNEL_COUNT 18
#define IBUS2_CHANNEL_COUNT 32
#define IBUS2_CHANNEL_TYPES_LENGTH 20
#define IBUS2_CHANNEL_RANGE_100 16384
#define IBUS2_CHANNEL_RANGE_150 ((IBUS2_CHANNEL_RANGE_100 * 150) / 100)
#define IBUS2_REQUIRED_RESOURCE_CHANNEL_TYPES (1U << 0)
#define IBUS2_REQUIRED_RESOURCE_FAILSAFE      (1U << 1)
#define IBUS2_HEADER_PACKET_TYPE_MASK 0x03
#define IBUS2_HEADER_SUBTYPE_MASK 0x3C
#define IBUS2_HEADER_SYNC_LOST_MASK 0x40
#define IBUS2_HEADER_FAILSAFE_MASK 0x80
#define IBUS2_ADDRESS_RESERVED_MASK 0xC0
#define IBUS2_INTERFRAME_GAP_MIN_US 70
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

static uint32_t ibus2ChannelData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool ibus2FrameDone = false;
static bool ibus2FrameFailsafe = false;
static bool ibus2FrameSyncLost = false;
static timeUs_t ibus2LastFrameTimeUs = 0;
static serialPort_t *ibus2RxSerialPort = NULL;
static uint8_t ibus2RequiredResources = IBUS2_REQUIRED_RESOURCE_CHANNEL_TYPES;
static uint8_t ibus2ChannelTypes[IBUS2_CHANNEL_TYPES_LENGTH + 1];
static bool ibus2HaveChannelTypes = false;
static uint8_t ibus2PackedChannels[IBUS2_CHANNEL_COUNT + 3];
static uint8_t ibus2PackedFailsafe[IBUS2_CHANNEL_COUNT + 3];
static bool ibus2HavePackedChannels = false;
static bool ibus2LastPackedSyncLost = false;
static bool ibus2LastPackedFailsafe = false;
static timeUs_t ibus2LastPackedTimeUs = 0;

typedef struct {
    uint8_t buf[IBUS2_FIRST_FRAME_MAX_LEN];
    uint8_t idx;
    uint8_t expectedLen;
    bool allowStart;
    timeUs_t lastByteTimeUs;
} ibus2FrameParser_t;

static ibus2FrameParser_t ibus2FrameParser = {
    .allowStart = true
};

static uint8_t ibus2Crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x25);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool ibus2CrcOk(const uint8_t *frame, size_t frameLen)
{
    return frameLen >= 2 && ibus2Crc8(frame, frameLen - 1) == frame[frameLen - 1];
}

static uint16_t readU16Unaligned(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t readU32Unaligned(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static void ibus2ResetFrameParser(void)
{
    ibus2FrameParser.idx = 0;
    ibus2FrameParser.expectedLen = 0;
    ibus2FrameParser.allowStart = true;
    ibus2FrameParser.lastByteTimeUs = 0;
}

static void ibus2ResetState(void)
{
    ibus2FrameDone = false;
    ibus2FrameFailsafe = false;
    ibus2FrameSyncLost = false;
    ibus2LastFrameTimeUs = 0;
    ibus2RxSerialPort = NULL;
    ibus2RequiredResources = IBUS2_REQUIRED_RESOURCE_CHANNEL_TYPES;
    ibus2HaveChannelTypes = false;
    ibus2HavePackedChannels = false;
    ibus2LastPackedSyncLost = false;
    ibus2LastPackedFailsafe = false;
    ibus2LastPackedTimeUs = 0;

    memset(ibus2ChannelTypes, 0, sizeof(ibus2ChannelTypes));
    memset(ibus2PackedChannels, 0, sizeof(ibus2PackedChannels));
    memset(ibus2PackedFailsafe, 0, sizeof(ibus2PackedFailsafe));
    ibus2ResetFrameParser();
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
    const size_t channelCount = payloadLen >= 27 ? IBUS2_BASE_CHANNEL_COUNT : (payloadLen * 8U) / 12U;

    if (channelCount < 4) {
        return false;
    }

    for (size_t i = 0; i < channelCount && i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        const int16_t signedRaw = ibus2SignExtend12((uint16_t)ibus2ExtractBitsLE(payload, i * 12U, 12, payloadLen));
        int32_t channelValue = 1500 + (signedRaw / 2);

        if (channelValue < IBUS2_CHANNEL_US_MIN) {
            channelValue = IBUS2_CHANNEL_US_MIN;
        }
        if (channelValue > IBUS2_CHANNEL_US_MAX) {
            channelValue = IBUS2_CHANNEL_US_MAX;
        }

        ibus2ChannelData[i] = channelValue;
    }

    ibus2FrameDone = true;
    ibus2FrameFailsafe = failsafe;
    ibus2FrameSyncLost = syncLost;
    ibus2LastFrameTimeUs = nowUs;

    return true;
}

static bool ibus2DecodeStoredChannels(timeUs_t nowUs, bool syncLost, bool failsafe)
{
    int16_t unpackedChannels[IBUS2_CHANNEL_COUNT] = { 0 };

    if (!ibus2HaveChannelTypes) {
        return false;
    }

    SES_UnpackChannels(ibus2PackedChannels, unpackedChannels, IBUS2_CHANNEL_COUNT, ibus2ChannelTypes);

    const uint8_t channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT < IBUS2_CHANNEL_COUNT ? MAX_SUPPORTED_RC_CHANNEL_COUNT : IBUS2_CHANNEL_COUNT;
    for (uint8_t i = 0; i < channelCount; i++) {
        ibus2ChannelData[i] = ibus2ConvertChannelToUs(unpackedChannels[i], (uint16_t)ibus2ChannelData[i]);
    }

    ibus2FrameDone = true;
    ibus2FrameFailsafe = failsafe;
    ibus2FrameSyncLost = syncLost;
    ibus2LastFrameTimeUs = nowUs;

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
        return;
    }

    const uint8_t packetSubtype = (frame[0] & IBUS2_HEADER_SUBTYPE_MASK) >> 2;
    const bool syncLost = (frame[0] & IBUS2_HEADER_SYNC_LOST_MASK) != 0;
    const bool failsafe = (frame[0] & IBUS2_HEADER_FAILSAFE_MASK) != 0;
    const uint8_t *payload = &frame[3];
    const uint8_t payloadLen = frame[1] - 4;

    switch (packetSubtype) {
    case 0:
        memset(ibus2PackedChannels, 0, sizeof(ibus2PackedChannels));
        memcpy(ibus2PackedChannels, payload, payloadLen < sizeof(ibus2PackedChannels) ? payloadLen : sizeof(ibus2PackedChannels));
        ibus2HavePackedChannels = true;
        ibus2LastPackedSyncLost = syncLost;
        ibus2LastPackedFailsafe = failsafe;
        ibus2LastPackedTimeUs = nowUs;
        if (ibus2HaveChannelTypes) {
            ibus2DecodeStoredChannels(nowUs, syncLost, failsafe);
        } else {
            ibus2DecodePacked12Fallback(payload, payloadLen, nowUs, syncLost, failsafe);
        }
        break;

    case 1:
        if (payloadLen < IBUS2_CHANNEL_TYPES_LENGTH) {
            break;
        }
        memcpy(ibus2ChannelTypes, payload, IBUS2_CHANNEL_TYPES_LENGTH);
        ibus2HaveChannelTypes = true;
        ibus2RequiredResources &= (uint8_t)~IBUS2_REQUIRED_RESOURCE_CHANNEL_TYPES;
        if (ibus2HavePackedChannels) {
            ibus2DecodeStoredChannels(ibus2LastPackedTimeUs, ibus2LastPackedSyncLost, ibus2LastPackedFailsafe);
        }
        break;

    case 2:
        memset(ibus2PackedFailsafe, 0, sizeof(ibus2PackedFailsafe));
        memcpy(ibus2PackedFailsafe, payload, payloadLen < sizeof(ibus2PackedFailsafe) ? payloadLen : sizeof(ibus2PackedFailsafe));
        ibus2RequiredResources &= (uint8_t)~IBUS2_REQUIRED_RESOURCE_FAILSAFE;
        break;

    default:
        break;
    }
}

static void ibus2FinalizePendingFrame(timeUs_t nowUs)
{
    if (ibus2FrameParser.idx == 0) {
        return;
    }

    ibus2FrameParser.idx = 0;
    ibus2FrameParser.expectedLen = 0;
    ibus2FrameParser.allowStart = true;
    ibus2FrameParser.lastByteTimeUs = nowUs;
}

static void ibus2ProcessByte(uint8_t byte)
{
    const timeUs_t nowUs = microsISR();

    if (ibus2FrameParser.lastByteTimeUs &&
        cmpTimeUs(nowUs, ibus2FrameParser.lastByteTimeUs) > IBUS2_INTERFRAME_GAP_MIN_US) {
        ibus2FinalizePendingFrame(nowUs);
        ibus2FrameParser.allowStart = true;
    }
    ibus2FrameParser.lastByteTimeUs = nowUs;

    if (ibus2FrameParser.idx == 0) {
        if (!ibus2FrameParser.allowStart) {
            return;
        }
        if ((byte & IBUS2_HEADER_PACKET_TYPE_MASK) != 0) {
            ibus2FrameParser.allowStart = false;
            return;
        }
        ibus2FrameParser.buf[ibus2FrameParser.idx++] = byte;
        return;
    }

    if (ibus2FrameParser.idx >= sizeof(ibus2FrameParser.buf)) {
        ibus2FinalizePendingFrame(nowUs);
        return;
    }

    ibus2FrameParser.buf[ibus2FrameParser.idx++] = byte;

    if (ibus2FrameParser.idx == 2) {
        ibus2FrameParser.expectedLen = byte;
        if (byte < IBUS2_FIRST_FRAME_MIN_LEN || byte > IBUS2_FIRST_FRAME_MAX_LEN) {
            ibus2FinalizePendingFrame(nowUs);
            ibus2FrameParser.allowStart = false;
        }
        return;
    }

    if (ibus2FrameParser.expectedLen && ibus2FrameParser.idx == ibus2FrameParser.expectedLen) {
        ibus2ProcessFirstFrame(ibus2FrameParser.buf, ibus2FrameParser.expectedLen, nowUs);
        ibus2FrameParser.idx = 0;
        ibus2FrameParser.expectedLen = 0;
        ibus2FrameParser.allowStart = false;
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

    if (!ibus2FrameDone) {
        return frameStatus;
    }

    ibus2FrameDone = false;

    if (ibus2FrameFailsafe) {
        frameStatus = RX_FRAME_FAILSAFE;
    } else if (ibus2FrameSyncLost) {
        frameStatus = RX_FRAME_DROPPED;
    } else {
        frameStatus = RX_FRAME_COMPLETE;
    }

    rxRuntimeState->lastRcFrameTimeUs = ibus2LastFrameTimeUs;
    return frameStatus;
}

static float ibus2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibus2ChannelData[chan];
}

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = IBUS2_BASE_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000;
    rxRuntimeState->rcReadRawFn = ibus2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibus2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    ibus2ResetState();

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        ibus2ChannelData[i] = 1500;
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
        rxConfig->halfDuplex ? MODE_RXTX : MODE_RX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
            (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
    );

    ibus2RxSerialPort = ibusPort;

    return ibusPort != NULL;
}

serialPort_t *ibus2GetRxSerialPort(void)
{
    return ibus2RxSerialPort;
}

#endif
