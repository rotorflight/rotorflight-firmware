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

#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_SRXL2

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "rx/srxl2.h"
#include "rx/srxl2_types.h"
#include "io/spektrum_vtx_control.h"

#ifndef SRXL2_DEBUG
#define SRXL2_DEBUG 0
#endif

#if SRXL2_DEBUG
#define DEBUG_PRINTF(...) //Temporary until a better debug printf can be included
#else
#define DEBUG_PRINTF(...)
#endif



#define SRXL2_MAX_CHANNELS             32
#define SRXL2_FRAME_PERIOD_US   11000 // 5500 for DSMR
#define SRXL2_CHANNEL_SHIFT            2
#define SRXL2_CHANNEL_CENTER           0x8000

#define SRXL2_PORT_BAUDRATE_DEFAULT    115200
#define SRXL2_PORT_BAUDRATE_HIGH       400000
#define SRXL2_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define SRXL2_PORT_MODE                MODE_RXTX

#define SRXL2_REPLY_QUIESCENCE         (2 * 10 * 1000000 / SRXL2_PORT_BAUDRATE_DEFAULT) // 2 * (lastIdleTimestamp - lastReceiveTimestamp). Time taken to send 2 bytes

#define SRXL2_ID                       0xA6
#define SRXL2_MAX_PACKET_LENGTH        80
#define SRXL2_DEVICE_ID_BROADCAST      0xFF

#define SRXL2_FRAME_TIMEOUT_US         50000

#define SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US 50000
#define SRXL2_SEND_HANDSHAKE_TIMEOUT_US 50000
#define SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US 200000

#define SPEKTRUM_PULSE_OFFSET          988 // Offset value to convert digital data into RC pulse

typedef union {
        uint8_t raw[SRXL2_MAX_PACKET_LENGTH];
        Srxl2Header header;
} Srxl2Frame;

struct rxBuf {
    volatile unsigned len;
    Srxl2Frame packet;
};

static uint8_t unitId = 0;
static uint8_t baudRate = 0;

static Srxl2State state = Disabled;
static uint32_t timeoutTimestamp = 0;
static uint32_t fullTimeoutTimestamp = 0;
static uint32_t lastValidPacketTimestamp = 0;
static volatile uint32_t lastReceiveTimestamp = 0;
static volatile uint32_t lastIdleTimestamp = 0;

struct rxBuf readBuffer[2];
struct rxBuf* readBufferPtr = &readBuffer[0];
struct rxBuf* processBufferPtr = &readBuffer[1];
static volatile unsigned readBufferIdx = 0;
static volatile bool transmittingTelemetry = false;
static uint8_t writeBuffer[SRXL2_MAX_PACKET_LENGTH];
static unsigned writeBufferIdx = 0;

static serialPort_t *serialPort;

static uint8_t busMasterDeviceId = 0xFF;
static bool telemetryRequested = false;

static uint8_t telemetryFrame[22];

uint8_t globalResult = 0;

static uint32_t srxl2BootEndUs = 0;

char srxl2DebugMessage[64] = "Not initialized";
char srxl2LastError[64] = "None";
char srxl2StateString[32] = "Unknown";

/* handshake protocol
    1. listen for 50ms for serial activity and go to State::Running if found, autobaud may be necessary
    2. if srxl2_unitId = 0:
            send a Handshake with destinationDeviceId = 0 every 50ms for at least 200ms
        else:
            listen for Handshake for at least 200ms
    3.  respond to Handshake as currently implemented in process if rePst received
    4.  respond to broadcast Handshake
*/

// if 50ms with not activity, go to default baudrate and to step 1

bool srxl2ProcessHandshake(const Srxl2Header* header)
{
    const Srxl2HandshakeSubHeader* handshake = (Srxl2HandshakeSubHeader*)(header + 1);
    if (handshake->destinationDeviceId == Broadcast) {
        DEBUG_PRINTF("broadcast handshake from %x\r\n", handshake->sourceDeviceId);
        busMasterDeviceId = handshake->sourceDeviceId;

        if (handshake->baudSupported == 1) {
            serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_HIGH);
            DEBUG_PRINTF("switching to %d baud\r\n", SRXL2_PORT_BAUDRATE_HIGH);
        }

        state = Running;

        return true;
    }


    if (handshake->destinationDeviceId != ((FlightController << 4) | unitId)) {
        return true;
    }

    DEBUG_PRINTF("FC handshake from %x\r\n", handshake->sourceDeviceId);

    Srxl2HandshakeFrame response = {
        .header = *header,
        .payload = {
            handshake->destinationDeviceId,
            handshake->sourceDeviceId,
            /* priority */ 10,
            /* baudSupported*/ baudRate,
            /* info */ 0,
            // U_ID_2
        }
    };

    srxl2RxWriteData(&response, sizeof(response));

    return true;
}

void srxl2ProcessChannelData(const Srxl2ChannelDataHeader* channelData, rxRuntimeState_t *rxRuntimeState) {
    globalResult = RX_FRAME_COMPLETE;

    if (channelData->rssi >= 0) {
        const int rssiPercent = channelData->rssi;
        setRssi(scaleRange(rssiPercent, 0, 100, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL);
    }

    //If receiver is in a connected state, and a packet is missed, the channel mask will be 0.
    if (!channelData->channelMask.u32) {
        globalResult |= RX_FRAME_DROPPED;
        return;
    }

    const uint16_t *frameChannels = (const uint16_t *) (channelData + 1);
    uint32_t channelMask = channelData->channelMask.u32;
    while (channelMask) {
        unsigned idx = __builtin_ctz (channelMask);
        uint32_t mask = 1 << idx;
        rxRuntimeState->channelData[idx] = *frameChannels++;
        channelMask &= ~mask;
    }

     DEBUG_PRINTF("channel data: %d %d %x\r\n", channelData_header->rssi, channelData_header->frameLosses, channelData_header->channelMask.u32);
}

bool srxl2ProcessControlData(const Srxl2Header* header, rxRuntimeState_t *rxRuntimeState)
{
    const Srxl2ControlDataSubHeader* controlData = (Srxl2ControlDataSubHeader*)(header + 1);
    const uint8_t ownId = (FlightController << 4) | unitId;
    if (controlData->replyId == ownId) {
        telemetryRequested = true;
        DEBUG_PRINTF("command: %x replyId: %x ownId: %x\r\n", controlData->command, controlData->replyId, ownId);
    }

    switch (controlData->command) {
    case ChannelData:
        srxl2ProcessChannelData((const Srxl2ChannelDataHeader *) (controlData + 1), rxRuntimeState);
        break;

    case FailsafeChannelData: {
        globalResult |= RX_FRAME_FAILSAFE;
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
        // DEBUG_PRINTF("fs channel data\r\n");
    } break;

    case VTXData: {
#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
        Srxl2VtxData *vtxData = (Srxl2VtxData*)(controlData + 1);
        DEBUG_PRINTF("vtx data\r\n");
        DEBUG_PRINTF("vtx band: %x\r\n", vtxData->band);
        DEBUG_PRINTF("vtx channel: %x\r\n", vtxData->channel);
        DEBUG_PRINTF("vtx pit: %x\r\n", vtxData->pit);
        DEBUG_PRINTF("vtx power: %x\r\n", vtxData->power);
        DEBUG_PRINTF("vtx powerDec: %x\r\n", vtxData->powerDec);
        DEBUG_PRINTF("vtx region: %x\r\n", vtxData->region);
        // Pack data as it was used before srxl2 to use existing functions.
        // Get the VTX control bytes in a frame
        uint32_t vtxControl =   (0xE0 << 24) | (0xE0 << 8) |
                                ((vtxData->band & 0x07) << 21) |
                                ((vtxData->channel & 0x0F) << 16) |
                                ((vtxData->pit & 0x01) << 4) |
                                ((vtxData->region & 0x01) << 3) |
                                ((vtxData->power & 0x07));
        spektrumHandleVtxControl(vtxControl);
#endif
    } break;

    default:
        break;
    }

    return true;
}

bool srxl2ProcessPacket(const Srxl2Header* header, rxRuntimeState_t *rxRuntimeState)
{
    switch (header->packetType) {
    case Handshake:
        return srxl2ProcessHandshake(header);
    case ControlData:
        return srxl2ProcessControlData(header, rxRuntimeState);
    default:
        DEBUG_PRINTF("Other packet type, ID: %x \r\n", header->packetType);
        break;
    }

    return false;
}

// @note assumes packet is fully there
void srxl2Process(rxRuntimeState_t *rxRuntimeState)
{
    if (processBufferPtr->packet.header.id != SRXL2_ID || processBufferPtr->len != processBufferPtr->packet.header.length) {
        DEBUG_PRINTF("invalid header id: %x, or length: %x received vs %x expected \r\n", processBufferPtr->packet.header.id, processBufferPtr->len, processBufferPtr->packet.header.length);
        globalResult = RX_FRAME_DROPPED;
        return;
    }

    const uint16_t calculatedCrc = crc16_ccitt_update(0, processBufferPtr->packet.raw, processBufferPtr->packet.header.length);

    //Invalid if crc non-zero
    if (calculatedCrc) {
        globalResult = RX_FRAME_DROPPED;
        DEBUG_PRINTF("crc mismatch %x\r\n", calculatedCrc);
        return;
    }

    //Packet is valid only after ID and CRC check out
    lastValidPacketTimestamp = micros();

    if (srxl2ProcessPacket(&processBufferPtr->packet.header, rxRuntimeState)) {
        return;
    }

    DEBUG_PRINTF("could not parse packet: %x\r\n", processBufferPtr->packet.header.packetType);
    globalResult = RX_FRAME_DROPPED;
}


static void srxl2DataReceive(uint16_t character, void *data)
{
    UNUSED(data);

    lastReceiveTimestamp = microsISR();

    //If the buffer len is not reset for whatever reason, disable reception
    if (readBufferPtr->len > 0 || readBufferIdx >= SRXL2_MAX_PACKET_LENGTH) {
        readBufferIdx = 0;
        globalResult = RX_FRAME_DROPPED;
    }
    else {
        readBufferPtr->packet.raw[readBufferIdx] = character;
        readBufferIdx++;
    }
}

static void srxl2Idle()
{
    if (transmittingTelemetry) { // Transmitting telemetry triggers idle interrupt as well. We dont want to change buffers then
        transmittingTelemetry = false;
    }
    else if (readBufferIdx == 0) { // Packet was invalid
        readBufferPtr->len = 0;
    }
    else {
        lastIdleTimestamp = microsISR();
        //Swap read and process buffer pointers
        if (processBufferPtr == &readBuffer[0]) {
            processBufferPtr = &readBuffer[1];
            readBufferPtr = &readBuffer[0];
        } else {
            processBufferPtr = &readBuffer[0];
            readBufferPtr = &readBuffer[1];
        }
        processBufferPtr->len = readBufferIdx;
    }

    readBufferIdx = 0;
}

static inline void put_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void put_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }

static void srxl2SendSmart(uint8_t sensorId, uint8_t secondaryId, const uint8_t data[14])
{
    // Frame: [A6][80][22][dest][sensorId][secondaryId][data(14)][CRC(2)]
    uint8_t f[22];   // header + dest + 16-byte payload (sensorId + secondaryId + data[14]) + CRC(2)
    int i = 0;

    f[i++] = SRXL2_ID;               // 0xA6
    f[i++] = TelemetrySensorData;    // 0x80
    f[i++] = 22;                     // total length incl. CRC
    f[i++] = busMasterDeviceId;      // destination = receiver/bus master

    f[i++] = sensorId;               // SMART sensor ID we want to emulate
    f[i++] = secondaryId;            // usually 0x00

    memcpy(&f[i], data, 14);
    i += 14;

    // Append CRC via existing helper (len = payload + 2 CRC bytes)
    srxl2RxWriteData(f, i + 2);

    telemetryRequested = false;
}

#define BE16(x) (uint8_t)((x) >> 8), (uint8_t)((x) & 0xFF)

#if defined(USE_SMART_ESC)
static bool srxl2SendSmartescTelemetry(void)
{
    uint8_t sensorId = 0;
    uint8_t secondaryId = 0;
    uint8_t payload[14];

    /* Prefer the latest telemetry entry when available and not filtered.
     * If latest is filtered (e.g. sensorId == 0x0C), fall back to the
     * most-recent non-filtered entry from the telemetry history so SRXL2
     * keeps sending useful frames at the expected cadence. */
    if (smartescGetLatestTelemetry(&sensorId, &secondaryId, payload, sizeof(payload))) {
        srxl2SendSmart(sensorId, secondaryId, payload);
        return true;
    }

    /* Latest is unavailable (possibly filtered). Search history newest->oldest. */
    unsigned histCount = smartescGetTelemetryHistoryCount();
    if (histCount == 0) {
        return false;
    }

    for (int i = (int)histCount - 1; i >= 0; --i) {
        uint8_t len = 0;
        uint32_t ts = 0;
        uint8_t sid = 0;
        uint8_t secid = 0;
        uint32_t entryCount = 0;
        unsigned copied = smartescCopyTelemetryHistory((unsigned)i, payload, sizeof(payload), &len, &ts, &sid, &secid, &entryCount);
        if (copied == 0) continue;
        if (sid == 0x0C) continue; /* skip filtered sensor page */
        srxl2SendSmart(sid, secid, payload);
        return true;
    }

    return false;
}
#endif

/* No SMART history available. As a fallback, export per-motor ESC telemetry
 * (from `escSensorData_t`) using the SMART payload layout so receivers still
 * get ESC info even when SMART ESC frames are disabled or absent. When no
 * live ESC data exists, feed a fake block for bench testing. */
static bool srxl2SendEscSensorTelemetry(void)
{
#if defined(USE_ESC_SENSOR)
    /* If the ESC sensor subsystem isn't active, don't emit fallback frames. */
    if (!isEscSensorActive()) {
        return false;
    }

    const int motorCount = getMotorCount();
    for (int m = 0; m < motorCount; ++m) {
        const escSensorData_t *esc = getEscSensorData((uint8_t)m);

        /* Treat missing or zeroed telemetry as absent and inject the shared fake struct. */
        const bool escMissing = !esc;
        const bool escZeroed = esc &&
            esc->erpm == 0 && esc->voltage == 0 && esc->current == 0 &&
            esc->temperature == 0 && esc->temperature2 == 0 &&
            esc->bec_voltage == 0 && esc->bec_current == 0;

        uint8_t payloadEsc[14];
        if (escMissing || escZeroed) {
            return false;
        } else {
            memset(payloadEsc, 0xFF, sizeof(payloadEsc));

            uint16_t rawRpm = (esc->erpm == 0) ? 0xFFFF : (uint16_t)(esc->erpm / 10u);
            put_be16(&payloadEsc[0], rawRpm == 0xFFFF ? 0xFFFF : rawRpm);

            uint16_t rawVolts = (esc->voltage == 0) ? 0xFFFF : (uint16_t)(esc->voltage / 10u);
            put_be16(&payloadEsc[2], rawVolts == 0xFFFF ? 0xFFFF : rawVolts);

            put_be16(&payloadEsc[4], (uint16_t)esc->temperature);

            uint16_t rawCurrent = (esc->current == 0) ? 0xFFFF : (uint16_t)(esc->current / 10u);
            put_be16(&payloadEsc[6], rawCurrent == 0xFFFF ? 0xFFFF : rawCurrent);

            put_be16(&payloadEsc[8], (uint16_t)esc->temperature2);

            /* BEC current: 100 mA units (spec) */
            payloadEsc[10] = (esc->bec_current == 0) ? 0xFF : (uint8_t)MIN(254u, (uint32_t)((esc->bec_current + 50u) / 100u));
            payloadEsc[11] = (esc->bec_voltage == 0) ? 0xFF : (uint8_t)(esc->bec_voltage / 50u);

            /* throttle in 0.1% -> SMART units (0.5%); clamp to 100% (200 counts) */
            if (esc->throttle == 0) {
                payloadEsc[12] = 0xFF;
            } else {
                uint32_t t = (uint32_t)(esc->throttle + 4u) / 5u; // round to nearest
                if (t > 200u) t = 200u; // cap at 100%
                payloadEsc[12] = (uint8_t)t;
            }

            /* powerOut: 0.5% units, cap to 127 per spec */
            if (esc->pwm == 0) {
                payloadEsc[13] = 0xFF;
            } else {
                uint32_t p = (uint32_t)(esc->pwm + 4u) / 5u; // 0.1% -> 0.5% (round)
                if (p > 127u) p = 127u;
                payloadEsc[13] = (uint8_t)p;
            }
        }

        srxl2SendSmart(0x20, 0x00, payloadEsc);
        return true;
    }
#endif

    return false;
}

// Extended: rotate ESC + SMART battery pages (0x42: 0x00, 0x10, 0x80)
static void srxl2SendTelemetryData(void)
{
    if (!telemetryRequested) return;

#if defined(USE_SMART_ESC)
    srxl2SendSmartescTelemetry();
    return;
#endif

    /* If SMART ESC frames are disabled or empty, fall back to ESC sensor data */
#if defined(USE_ESC_SENSOR)
    if (srxl2SendEscSensorTelemetry()) {
        return;
    }
#endif
}

static uint8_t srxl2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    globalResult = RX_FRAME_PENDING;

    // len should only be set after an idle interrupt (packet reception complete)
    if (processBufferPtr != NULL && processBufferPtr->len) {
        srxl2Process(rxRuntimeState);
        processBufferPtr->len = 0;
    }

    uint8_t result = globalResult;

    const uint32_t now = micros();

    switch (state) {
    case Disabled: break;

    case ListenForActivity: {
        // activity detected
        if (lastValidPacketTimestamp != 0) {
            // as ListenForActivity is done at default baud-rate, we don't need to change anything
            // @todo if there were non-handshake packets - go to running,
            // if there were - go to either Send Handshake or Listen For Handshake
            state = Running;
        } else if (cmpTimeUs(lastIdleTimestamp, lastReceiveTimestamp) > 0) {
            if (baudRate != 0) {
                uint32_t currentBaud = serialGetBaudRate(serialPort);

                if(currentBaud == SRXL2_PORT_BAUDRATE_DEFAULT)
                    serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_HIGH);
                else
                    serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            }
        } else if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo if there was activity - detect baudrate and ListenForHandshake

            if (unitId == 0) {
                state = SendHandshake;
                timeoutTimestamp = now + SRXL2_SEND_HANDSHAKE_TIMEOUT_US;
                fullTimeoutTimestamp = now + SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            } else {
                state = ListenForHandshake;
                timeoutTimestamp = now + SRXL2_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            }
        }
    } break;

    case SendHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo set another timeout for 50ms tries
            // fill write buffer with handshake frame
            result |= RX_FRAME_PROCESSING_REQUIRED;
        }

        if (cmpTimeUs(now, fullTimeoutTimestamp) >= 0) {
            serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case SendHandshake: switching to %d baud\r\n", SRXL2_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } break;

    case ListenForHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0)  {
            serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case ListenForHandshake: switching to %d baud\r\n", SRXL2_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } break;

    case Running: {
        // frame timed out, reset state
        if (cmpTimeUs(now, lastValidPacketTimestamp) >= SRXL2_FRAME_TIMEOUT_US) {
            serialSetBaudRate(serialPort, SRXL2_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case Running: switching to %d baud: %d %d\r\n", SRXL2_PORT_BAUDRATE_DEFAULT, now, lastValidPacketTimestamp);
            timeoutTimestamp = now + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
            lastValidPacketTimestamp = 0;
        }

        srxl2SendTelemetryData();
    } 
        strcpy(srxl2StateString, "Running");
        break;
    };

    if (writeBufferIdx) {
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }

    if (!(result & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        rxRuntimeState->lastRcFrameTimeUs = lastIdleTimestamp;
    }

    return result;
}

static bool srxl2ProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (writeBufferIdx == 0) {
        return true;
    }

    const uint32_t now = micros();

    if (cmpTimeUs(lastIdleTimestamp, lastReceiveTimestamp) > 0) {
        // time sufficient for at least 2 characters has passed
        if (cmpTimeUs(now, lastReceiveTimestamp) > SRXL2_REPLY_QUIESCENCE) {
            transmittingTelemetry = true;
            serialWriteBuf(serialPort, writeBuffer, writeBufferIdx);
            writeBufferIdx = 0;
        } else {
            DEBUG_PRINTF("not enough time to send 2 characters passed yet, %d us since last receive, %d required\r\n", now - lastReceiveTimestamp, SRXL2_REPLY_QUIESCENCE);
        }
    } else {
        DEBUG_PRINTF("still receiving ...\r\n");
    }

    return true;
}

static float srxl2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channelIdx)
{
    if (channelIdx >= rxRuntimeState->channelCount) {
        return 0;
    }

    return ((float)(rxRuntimeState->channelData[channelIdx] >> SRXL2_CHANNEL_SHIFT) / 16) + SPEKTRUM_PULSE_OFFSET;
}

void srxl2RxWriteData(const void *data, int len)
{
    const uint16_t crc = crc16_ccitt_update(0, (uint8_t*)data, len - 2);
    ((uint8_t*)data)[len-2] = ((uint8_t *) &crc)[1] & 0xFF;
    ((uint8_t*)data)[len-1] = ((uint8_t *) &crc)[0] & 0xFF;

    len = MIN(len, (int)sizeof(writeBuffer));
    memcpy(writeBuffer, data, len);
    writeBufferIdx = len;
}

void validateAndFixSrxl2Config()
{
    // Force half duplex
    rxConfigMutable()->halfDuplex = true;
}

bool srxl2RxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint16_t channelData[SRXL2_MAX_CHANNELS];
    for (size_t i = 0; i < SRXL2_MAX_CHANNELS; ++i) {
        channelData[i] = SRXL2_CHANNEL_CENTER;
    }

    unitId = rxConfig->srxl2_unit_id;
    baudRate = rxConfig->srxl2_baud_fast;

    rxRuntimeState->channelData = channelData;
    rxRuntimeState->channelCount = SRXL2_MAX_CHANNELS;
    rxRuntimeState->rxRefreshRate = SRXL2_FRAME_PERIOD_US;

    rxRuntimeState->rcReadRawFn = srxl2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = srxl2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;
    rxRuntimeState->rcProcessFrameFn = srxl2ProcessFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_RX_SERIAL,
        srxl2DataReceive,
        NULL,
        SRXL2_PORT_BAUDRATE_DEFAULT,
        SRXL2_PORT_MODE,
        SRXL2_PORT_OPTIONS |
            (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
            (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
        );

    if (!serialPort) {
        return false;
    }

    serialPort->idleCallback = srxl2Idle;

    state = ListenForActivity;
    timeoutTimestamp = micros() + SRXL2_LISTEN_FOR_ACTIVITY_TIMEOUT_US;

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    return (bool)serialPort;
}

bool srxl2RxIsActive(void)
{
    return serialPort;
}

bool srxl2TelemetryRequested(void)
{
    return telemetryRequested;
}

void srxl2InitializeFrame(sbuf_t *dst)
{
    dst->ptr = telemetryFrame;
    dst->end = ARRAYEND(telemetryFrame);

    sbufWriteU8(dst, SRXL2_ID);
    sbufWriteU8(dst, TelemetrySensorData);
    sbufWriteU8(dst, ARRAYLEN(telemetryFrame));
    sbufWriteU8(dst, busMasterDeviceId);
}

void srxl2FinalizeFrame(sbuf_t *dst)
{
    UNUSED(dst); // we don't need to switch to reader; we know the buffer & length
    const int frameLen = telemetryFrame[2];   // SRXL2 length (includes CRC)
    srxl2RxWriteData(telemetryFrame, frameLen);
    telemetryRequested = false;
}

void srxl2Bind(void)
{
    const size_t length = sizeof(Srxl2BindInfoFrame);

    Srxl2BindInfoFrame bind = {
        .header = {
            .id = SRXL2_ID,
            .packetType = BindInfo,
            .length = length
        },
        .payload = {
            .request = EnterBindMode,
            .deviceId = busMasterDeviceId,
            .bindType = DMSX_11ms,
            .options = SRXL_BIND_OPT_TELEM_TX_ENABLE | SRXL_BIND_OPT_BIND_TX_ENABLE,
        }
    };

    srxl2RxWriteData(&bind, length);
}

#endif
