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
#include <math.h>
#include "common/printf.h"

#include "platform.h"

#ifdef USE_SRXL2_ESC

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "drivers/srxl2_esc.h"
#include "rx/srxl2_types.h"
#include "rx/rx.h"
#include "../sensors/esc_sensor.h"
#include "config/feature.h"
#include "pg/srxl2_esc.h"
#if defined(USE_MOTOR)
#include "flight/motors.h"
#include "drivers/motor.h"
#include "pg/motor.h"
#endif

#ifndef SRXL2_ESC_DEBUG
#define SRXL2_ESC_DEBUG 0
#endif

#if SRXL2_ESC_DEBUG
#define DEBUG_PRINTF(...) //Temporary until a better debug printf can be included
#else
#define DEBUG_PRINTF(...)
#endif

#define SRXL2_ESC_MAX_CHANNELS             32
#define SRXL2_ESC_FRAME_PERIOD_US          11000 // 5500 for DSMR
#define SRXL2_ESC_CHANNEL_SHIFT            2
#define SRXL2_ESC_CHANNEL_CENTER           0x8000

// #define SRXL2_ESC_PORT_BAUDRATE_DEFAULT    SRXL2_ESC_PORT_BAUDRATE_DEFAULT
// #define SRXL2_ESC_PORT_BAUDRATE_HIGH       SRXL2_ESC_PORT_BAUDRATE_HIGH
#define SRXL2_ESC_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define SRXL2_ESC_PORT_MODE                MODE_RXTX

/* Minimum quiet time between RX and TX so we flip the half-duplex line cleanly.
 * Dropping this to roughly a single byte-time (≈80us at 115200) makes handshake
 * responses fire sooner instead of waiting ~2 byte times (~174us). */
#define SRXL2_ESC_REPLY_QUIESCENCE         80
#define SRXL2_ESC_TX_TIMEOUT_US             20000
#define SRXL2_ESC_TELEMETRY_RX_TIMEOUT_US   10000
#define SRXL2_ESC_TELEMETRY_POST_HOLDOFF_US   300

/* Guard time after *requesting* telemetry before we consider transmitting again.
 * Needs to be long enough to cover ESC reply start latency + line turnaround,
 * otherwise FC TX can collide with the ESC reply on the half-duplex bus. */
#define SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_US  2500

/* Upper bound for the request holdoff. Prevents long stalls (e.g. 10ms) when
 * throttle update rate is configured low. */
#define SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_MAX_US  5000
#define SRXL2_ESC_TELEM_HISTORY             15

#define packet_HEADER                     0xA6
#define SRXL2_ESC_MAX_PACKET_LENGTH        80
#define SRXL2_ESC_DEVICE_ID_BROADCAST      0xFF

#define SRXL2_ESC_FRAME_TIMEOUT_US         50000

#define SRXL2_ESC_LISTEN_FOR_ACTIVITY_TIMEOUT_US 50000
#define SRXL2_ESC_SEND_HANDSHAKE_TIMEOUT_US 50000
#define SRXL2_ESC_LISTEN_FOR_HANDSHAKE_TIMEOUT_US 200000

#define SPEKTRUM_PULSE_OFFSET          988 // Offset value to convert digital data into RC pulse

typedef union {
        uint8_t raw[SRXL2_ESC_MAX_PACKET_LENGTH];
        Srxl2Header header;
} srxl2escFrame;

struct escBufse {
    volatile unsigned len;
    srxl2escFrame packet;
};

static uint8_t unitId = 0;
static uint8_t baudRate = 0;

static Srxl2State state = Disabled;
static uint32_t timeoutTimestamp = 0;
static uint32_t fullTimeoutTimestamp = 0;
static uint32_t lastValidPacketTimestamp = 0;
static volatile uint32_t lastReceiveTimestamp = 0;
static volatile uint32_t lastIdleTimestamp = 0;

static struct escBufse readBufferse[3];
static struct escBufse* readBufferPtr = &readBufferse[0];
static struct escBufse* processBufferPtr = &readBufferse[1];
static volatile unsigned readBufferIdx = 0;
/* Echo-suppression: After TX, skip exactly txBytesToSkip bytes.
 * This is a simple count-based approach - no byte matching needed. */
static volatile unsigned txBytesToSkip = 0;
static uint8_t writeBuffer[SRXL2_ESC_MAX_PACKET_LENGTH];
static unsigned writeBufferIdx = 0;

static serialPort_t *serialPort;
static serialPort_t *srxl2escDriverPort;
static srxl2esc_runtimeState_t srxl2escDriverRuntime;
static bool srxl2escDriverReady;

static uint8_t busMasterDeviceId = 0xFF;
static bool telemetryRequested = false;
static bool srxl2escTelemetryRxPending = false;
static uint32_t srxl2escTelemetryRxDeadlineUs = 0;
static uint32_t srxl2escTelemetryHoldoffEndUs = 0;

static uint8_t telemetryFrame[22];

typedef struct {
    uint32_t timestamp;
    uint32_t count;
    uint8_t sensorId;
    uint8_t secondaryId;
    uint8_t data[14];
} srxl2escTelemetryHistoryEntry_t;

static srxl2escTelemetryHistoryEntry_t srxl2escTelemetryHistory[SRXL2_ESC_TELEM_HISTORY];
static uint8_t srxl2escTelemetryHistoryCount = 0;
static uint8_t srxl2escTelemetryHistoryHead = 0;
static uint8_t srxl2escLatestTelemetrySensorId = 0;
static uint8_t srxl2escLatestTelemetrySecondaryId = 0;

static uint8_t srxl2escLatestTelemetryData[14];
static bool srxl2escLatestTelemetryValid = false;
static srxl2escTelemetrySnapshot_t srxl2escLatestTelemetrySnapshot = {0};
static volatile uint32_t srxl2escLatestTelemetrySeq = 0;

static uint8_t globalResult = 0;

/* Internal runtime state owned by SMART ESC driver (decoupled from RX subsystem) */
static srxl2esc_runtimeState_t srxl2escInternalRs;
static uint16_t srxl2escInternalChannelData[SRXL2_ESC_MAX_CHANNELS];

/* file-static pointer to runtime state so poll/service helpers can operate
 * srxl2escRsGlobal points to our internal runtime by default. Legacy callers
 * may pass a runtime into smartescInit, but the driver uses its internal
 * state to avoid coupling to the global RX subsystem. */
static srxl2esc_runtimeState_t *srxl2escRsGlobal = NULL;

static uint32_t srxl2escBootEndUs = 0;

#if defined(USE_MOTOR)
static uint16_t srxl2esc_lastMotorPwmRate = 0;
#endif

/* Forward declarations */
bool srxl2escIsActive(void);

static volatile unsigned srxl2escLastTxSkipSet = 0;  // Debug: last transmitted frame size tracked for echo suppression


// /* Timing capture: record durations (microseconds) for first few executions */
#define SRXL2_ESC_TIMING_SLOTS 5

static bool srxl2escTxInProgress = false;
static uint32_t srxl2escTxActiveSinceUs = 0;
static uint32_t srxl2escTxExpectedDoneUs = 0;

/* Helper: return a spare buffer pointer that's not the current read or process buffer.
 * Returns NULL if none available (shouldn't happen with 3 buffers unless pointers overlap). */
static struct escBufse* srxl2escGetSpareBuffer(void)
{
    for (int i = 0; i < 3; ++i) {
        struct escBufse *b = &readBufferse[i];
        if (b != readBufferPtr && b != processBufferPtr) return b;
    }
    return NULL;
}

static volatile unsigned srxl2escLastProcessedLen = 0;
static volatile uint8_t srxl2escLastProcessedPacketType = 0;

#define SRXL2_ESC_HANDSHAKE_FRAME_LIMIT 4
#define SRXL2_ESC_HANDSHAKE_MAX_BYTES 24

typedef enum {
    SRXL2_ESC_HANDSHAKE_STAGE_IDLE = 0,
    SRXL2_ESC_HANDSHAKE_STAGE_WAITING_ESC_REPLY,
    SRXL2_ESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK,
    SRXL2_ESC_HANDSHAKE_STAGE_DONE,
} srxl2escHandshakeStage_t;

static srxl2escHandshakeStage_t handshakeStage = SRXL2_ESC_HANDSHAKE_STAGE_IDLE;
static bool srxl2escHandshakeComplete = false;
static bool srxl2escBroadcastConfirmPending = false;
static Srxl2HandshakeFrame srxl2escBroadcastConfirmFrame;
typedef struct __attribute__((packed)) {
    Srxl2Header header;
    Srxl2ControlDataSubHeader control;
    Srxl2ChannelDataHeader channel;
    uint16_t channelValue;
    uint8_t crcHigh;
    uint8_t crcLow;
} srxl2escChannelDataFrame_t;

static uint16_t srxl2escThrottleCommand = SRXL2_ESC_CHANNEL_CENTER;
static uint32_t srxl2escNextThrottleSendUs = 0;
static uint32_t srxl2escThrottleIntervalUs = 1000000 / 250; /* 250 Hz default */
static uint8_t srxl2escThrottleFrameCounter = 0;
static uint8_t srxl2escTelemetryFrameInterval = 6; /* frames per telemetry poll (0=off) */

static uint32_t srxl2escGetActiveBaudRate(void)
{
    const uint32_t baud = serialPort ? serialGetBaudRate(serialPort) : 0;
    return baud ? baud : SRXL2_ESC_PORT_BAUDRATE_DEFAULT;
}

static uint32_t srxl2escComputeMinThrottleIntervalUs(void)
{
    const uint32_t baud = srxl2escGetActiveBaudRate();
    const uint32_t bitsPerByte = 10; // 8N1
    const uint32_t bytes = (uint32_t)sizeof(srxl2escChannelDataFrame_t);
    const uint32_t txTimeUs = (uint32_t)((bytes * bitsPerByte * 1000000ULL + (baud - 1)) / baud);
    return txTimeUs + SRXL2_ESC_REPLY_QUIESCENCE + 200;
}

static uint16_t srxl2escPulseWidthToChannelValue(uint16_t pulseWidthUs)
{
    if (pulseWidthUs <= SPEKTRUM_PULSE_OFFSET) {
        return 0;
    }

    const uint32_t scaled = (uint32_t)(pulseWidthUs - SPEKTRUM_PULSE_OFFSET) * 64U;
    return (uint16_t)MIN(scaled, 0xFFFFu);
}

static uint16_t srxl2escPulseFromNormalized(float normalized)
{
    float pulseUs;
#if defined(USE_MOTOR)
    const motorConfig_t *config = motorConfig();
    const motorControlMode_e mode = config->dev.motorControlMode[0];
    if (mode == MOTOR_CONTROL_BIDIR) {
        const float clamped = constrainf(normalized, -1.0f, 1.0f);
        if (clamped == 0.0f) {
            pulseUs = (float)config->mincommand;
        } else {
            pulseUs = scaleRangef(clamped,
                -1.0f,
                1.0f,
                (float)config->minthrottle,
                (float)config->maxthrottle);
        }
    } else {
        const float clamped = constrainf(normalized, 0.0f, 1.0f);
        if (clamped == 0.0f) {
            pulseUs = (float)config->mincommand;
        } else {
            pulseUs = scaleRangef(clamped,
                0.0f,
                1.0f,
                (float)config->minthrottle,
                (float)config->maxthrottle);
        }
    }
    pulseUs = constrainf(pulseUs, (float)config->mincommand, (float)config->maxthrottle);
#else
    const float clamped = constrainf(normalized, 0.0f, 1.0f);
    pulseUs = 1000.0f + clamped * 1000.0f;
#endif
    return (uint16_t)lrintf(pulseUs);
}

#if defined(USE_MOTOR)
static void srxl2escUpdateThrottleFromMotors(void)
{
    if (getMotorCount() == 0) {
        return;
    }

    const float normalized = (float)getMotorOutput(0) / 1000.0f;
    const uint16_t pulse = srxl2escPulseFromNormalized(normalized);
    srxl2escThrottleCommand = srxl2escPulseWidthToChannelValue(pulse);
}
#endif

static void srxl2escAdvanceHandshakeStage(srxl2escHandshakeStage_t newStage);
static const char *srxl2escHandshakeStageToString(srxl2escHandshakeStage_t stage);
static void srxl2escScheduleBroadcastConfirm(uint8_t fcId);
static bool srxl2escQueueChannelDataFrame(uint16_t channelValue);
static void srxl2escRecordTelemetryFrame(const Srxl2Header *header);
static void srxl2escHandleTelemetryFrame(const Srxl2Header *header);

static void srxl2escScheduleBroadcastConfirm(uint8_t fcId)
{
    srxl2escBroadcastConfirmFrame = (Srxl2HandshakeFrame) {
        .header = {
            .id = packet_HEADER,
            .packetType = Handshake,
            .length = sizeof(Srxl2HandshakeFrame),
        },
        .payload = {
            .sourceDeviceId      = fcId,
            .destinationDeviceId = SRXL2_ESC_DEVICE_ID_BROADCAST,
            .priority            = 10,
            .baudSupported       = baudRate,
            .info                = 0,
            .uniqueId            = 0,
        },
    };
    srxl2escBroadcastConfirmPending = true;
}

static bool srxl2escQueueChannelDataFrame(uint16_t channelValue)
{
    if (busMasterDeviceId == SRXL2_ESC_DEVICE_ID_BROADCAST) {
        return false;
    }
    if (writeBufferIdx != 0 || srxl2escTxInProgress) {
        return false;
    }

    bool requestTelemetry = false;
    const uint32_t now = micros();
    if (srxl2escTelemetryFrameInterval > 0 && !srxl2escTelemetryRxPending) {
        if (++srxl2escThrottleFrameCounter >= srxl2escTelemetryFrameInterval) {
            srxl2escThrottleFrameCounter = 0;
            requestTelemetry = true;
            srxl2escTelemetryRxPending = true;
            srxl2escTelemetryRxDeadlineUs = now + SRXL2_ESC_TELEMETRY_RX_TIMEOUT_US;
            /* Give the ESC a short window to start replying before we transmit again.
             * This avoids collisions without stalling for the full RX timeout. */
            uint32_t holdoffUs = srxl2escThrottleIntervalUs;
            if (holdoffUs < SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_US) {
                holdoffUs = SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_US;
            }
            if (holdoffUs > SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_MAX_US) {
                holdoffUs = SRXL2_ESC_TELEMETRY_REQUEST_HOLDOFF_MAX_US;
            }
            srxl2escTelemetryHoldoffEndUs = now + holdoffUs;
        }
    }
    const uint8_t replyId = requestTelemetry ? busMasterDeviceId : 0x00;

    srxl2escChannelDataFrame_t frame = {
        .header = {
            .id = packet_HEADER,
            .packetType = ControlData,
            .length = sizeof(srxl2escChannelDataFrame_t),
        },
        .control = {
            .command = ChannelData,
            .replyId = replyId,
        },
        .channel = {
            .rssi = 0,
            .frameLosses = 0,
            .channelMask = {.u32 = 0x00000001u},
        },
        .channelValue = channelValue,
        .crcHigh = 0,
        .crcLow = 0,
    };

    srxl2escWriteData(&frame, sizeof(frame));
    return true;
}

static void srxl2escHandleTelemetryFrame(const Srxl2Header *header)
{
    const uint8_t *payload = (const uint8_t *)(header + 1);
    uint8_t sensorId = 0;
    uint8_t secondaryId = 0;
    if (header->length >= 6) { /* header (3) + dest + sensor + secondary */
        sensorId = payload[1];
        secondaryId = payload[2];
    }
    srxl2escTelemetryRxPending = false;
    srxl2escTelemetryRxDeadlineUs = 0;
    const uint32_t rxCompleteUs = lastReceiveTimestamp ? lastReceiveTimestamp : micros();
    srxl2escTelemetryHoldoffEndUs = rxCompleteUs + SRXL2_ESC_TELEMETRY_POST_HOLDOFF_US;
    /* Previously we filtered out sensor type 0x0C completely here which prevented
     * SRXL2 from ever having telemetry to send if those were the only frames.
     * Record all frames and keep the latest entry so other subsystems (ESC_SENSOR,
     * OSD, etc.) can use the data. Transmission filtering (so SRXL2 doesn't
     * forward 0x0C frames) is handled by smartescGetLatestTelemetry(). */
    srxl2escRecordTelemetryFrame(header);
    srxl2escLatestTelemetrySensorId = sensorId;
    srxl2escLatestTelemetrySecondaryId = secondaryId;
    memcpy(srxl2escLatestTelemetryData, payload + 3, sizeof(srxl2escLatestTelemetryData));
    srxl2escLatestTelemetryValid = true;
    srxl2escLatestTelemetrySeq++;
    srxl2escLatestTelemetrySnapshot.sensorId = sensorId;
    srxl2escLatestTelemetrySnapshot.secondaryId = secondaryId;
    memcpy(srxl2escLatestTelemetrySnapshot.data, payload + 3, sizeof(srxl2escLatestTelemetrySnapshot.data));
    srxl2escLatestTelemetrySnapshot.timestampUs = rxCompleteUs;
    srxl2escLatestTelemetrySnapshot.valid = true;
    srxl2escLatestTelemetrySeq++;
}

static void srxl2escRecordTelemetryFrame(const Srxl2Header *header)
{
    if (!header || header->length < 6) {
        return;
    }
    const uint8_t *payload = (const uint8_t *)(header + 1);
    const uint8_t sensorId = payload[1];
    const uint8_t secondaryId = payload[2];
    const uint8_t *data = payload + 3;

    const unsigned base = (srxl2escTelemetryHistoryHead + SRXL2_ESC_TELEM_HISTORY - srxl2escTelemetryHistoryCount) % SRXL2_ESC_TELEM_HISTORY;
    for (unsigned i = 0; i < srxl2escTelemetryHistoryCount; ++i) {
        const unsigned slotIndex = (base + i) % SRXL2_ESC_TELEM_HISTORY;
        srxl2escTelemetryHistoryEntry_t *entry = &srxl2escTelemetryHistory[slotIndex];
        if (entry->sensorId == sensorId && entry->secondaryId == secondaryId && memcmp(entry->data, data, sizeof(entry->data)) == 0) {
            entry->count++;
            entry->timestamp = micros();
            return;
        }
    }

    srxl2escTelemetryHistoryEntry_t *slot = &srxl2escTelemetryHistory[srxl2escTelemetryHistoryHead];
    slot->timestamp = micros();
    slot->count = 1;
    slot->sensorId = sensorId;
    slot->secondaryId = secondaryId;
    memcpy(slot->data, data, sizeof(slot->data));
    srxl2escTelemetryHistoryHead = (srxl2escTelemetryHistoryHead + 1) % SRXL2_ESC_TELEM_HISTORY;
    if (srxl2escTelemetryHistoryCount < SRXL2_ESC_TELEM_HISTORY) {
        srxl2escTelemetryHistoryCount++;
    }
}

static void srxl2escAdvanceHandshakeStage(srxl2escHandshakeStage_t newStage)
{
    if (handshakeStage == newStage) {
        return;
    }

    handshakeStage = newStage;
    srxl2escHandshakeComplete = (newStage == SRXL2_ESC_HANDSHAKE_STAGE_DONE);
}

static const char *srxl2escHandshakeStageToString(srxl2escHandshakeStage_t stage)
{
    switch (stage) {
    case SRXL2_ESC_HANDSHAKE_STAGE_IDLE: return "idle";
    case SRXL2_ESC_HANDSHAKE_STAGE_WAITING_ESC_REPLY: return "wait_ESC";
    case SRXL2_ESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK: return "wait_FC";
    case SRXL2_ESC_HANDSHAKE_STAGE_DONE: return "done";
    default: return "unknown";
    }
}

bool srxl2escIsHandshakeComplete(void) { return srxl2escHandshakeComplete; }
const char *srxl2escGetHandshakeStageString(void) { return srxl2escHandshakeStageToString(handshakeStage); }

/* handshake protocol
    1. listen for 50ms for serial activity and go to State::Running if found, autobaud may be necessary
    2. if srxl2esc_unitId = 0:
            send a Handshake with destinationDeviceId = 0 every 50ms for at least 200ms
        else:
            listen for Handshake for at least 200ms
    3.  respond to Handshake as currently implemented in process if rePst received
    4.  respond to broadcast Handshake
*/

// if 50ms with not activity, go to default baudrate and to step 1

bool srxl2escProcessHandshake(const Srxl2Header* header)
{
    const Srxl2HandshakeSubHeader* handshake = (const Srxl2HandshakeSubHeader*)(header + 1);

    // srxl2escRecordHandshakeFrame((const uint8_t *)header, header->length, false);

    const uint8_t fcId = (FlightController << 4) | unitId;

    const bool isBroadcast = (handshake->destinationDeviceId == 0x00);
    const bool isOwnId = (handshake->destinationDeviceId == fcId);
    const bool treatsAsBroadcast = isBroadcast || isOwnId;

    if (treatsAsBroadcast) {
        busMasterDeviceId = handshake->sourceDeviceId;
        if (handshake->baudSupported == 1) {
            serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_HIGH);
        }

        Srxl2HandshakeFrame response = {
            .header = {
                .id = packet_HEADER,
                .packetType = Handshake,
                .length = sizeof(Srxl2HandshakeFrame),
            },
            .payload = {
                .sourceDeviceId      = (FlightController << 4) | unitId,
                .destinationDeviceId = handshake->sourceDeviceId,
                .priority            = 10,
                .baudSupported       = baudRate,
                .info                = 0,
                .uniqueId            = 0,
            },
        };

        srxl2escWriteData(&response, sizeof(response));
        /* Defer actual transmit to service() to avoid blocking in poll()/frame processing. */
        if (isOwnId) {
            srxl2escScheduleBroadcastConfirm(fcId);
            srxl2escAdvanceHandshakeStage(SRXL2_ESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK);
        }

        state = Running;
        srxl2escBootEndUs = micros() + 2000000;
        return true;
    }

    return true;
}

void srxl2escProcessChannelData(const Srxl2ChannelDataHeader* channelData, srxl2esc_runtimeState_t *runtimeState) {
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
        runtimeState->channelData[idx] = *frameChannels++;
        channelMask &= ~mask;
    }

    // Fix: channelData_header was an invalid identifier; use channelData for debug output
    DEBUG_PRINTF("channel data: %d %d %x\r\n", channelData->rssi, channelData->frameLosses, channelData->channelMask.u32);

    srxl2escAdvanceHandshakeStage(SRXL2_ESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK);
}

bool srxl2escProcessControlData(const Srxl2Header* header, srxl2esc_runtimeState_t *runtimeState)
{
    const Srxl2ControlDataSubHeader* controlData = (const Srxl2ControlDataSubHeader*)(header + 1);
    const uint8_t ownId = (FlightController << 4) | unitId;

    if (controlData->replyId == ownId) {
        telemetryRequested = true;
        DEBUG_PRINTF("command: %x replyId: %x ownId: %x\r\n",
            controlData->command, controlData->replyId, ownId);
    }

    // ---- Normal handling first ------------------------------------------------
    switch (controlData->command) {
    case ChannelData: {
        const Srxl2ChannelDataHeader *cdh = (const Srxl2ChannelDataHeader *)(controlData + 1);
        srxl2escProcessChannelData(cdh, runtimeState);
    } break;

    case FailsafeChannelData: {
        globalResult |= RX_FRAME_FAILSAFE;
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
    } break;

    default:
        break;
    }

    // ---- Compute and log any extra bytes ("tail") ----------------------------
    // header->length includes: 3-byte smartesc header + payload + 2-byte CRC
    const int totalLen   = (int)header->length;
    const int crcBytes   = 2;
    const int hdrBytes   = 3; // id, packetType, length

    // payload starts at 'controlData' (command is first byte in payload)
    int payloadLen = totalLen - hdrBytes - crcBytes;
    if (payloadLen < 0) payloadLen = 0;

    // Bytes consumed by the specific ControlData subtype:
    int consumed = 0;
    switch (controlData->command) {
    case ChannelData: {
        // layout after command/replyId:
        //   rssi(1), frameLosses(1), channelMask(4), then N 16-bit channel values
        const Srxl2ChannelDataHeader *cdh = (const Srxl2ChannelDataHeader *)(controlData + 1);
        const int subhdrLen = 1 /*command*/ + 1 /*replyId*/ + 1 /*rssi*/ + 1 /*frameLosses*/ + 4 /*channelMask*/;
        const unsigned pop = __builtin_popcount(cdh->channelMask.u32);
        consumed = subhdrLen + (int)(2 * pop);
    } break;
    case FailsafeChannelData: {
        // failsafe control block typically ends at frameLosses/channelMask (no channel words)
        consumed = 1 /*command*/ + 1 /*replyId*/ + 1 /*rssi*/ + 1 /*frameLosses*/ + 4 /*channelMask*/;
    } break;
    default:
        consumed = MIN(payloadLen, 2); // at least command+replyId, be conservative
        break;
    }

    if (consumed < 0) consumed = 0;
    if (consumed > payloadLen) consumed = payloadLen;

    return true;
}

bool srxl2escProcessPacket(const Srxl2Header* header, srxl2esc_runtimeState_t *runtimeState)
{
    UNUSED(runtimeState);
    switch (header->packetType) {
    case Handshake:
        return srxl2escProcessHandshake(header);
    case TelemetrySensorData:
        srxl2escHandleTelemetryFrame(header);
        return true;
    default:
        DEBUG_PRINTF("Other packet type, ID: %x \r\n", header->packetType);
        break;
    }

    return false;
}

static void srxl2escTrySendPendingWriteBuffer(void)
{
    if (!serialPort) {
        return;
    }

    if (srxl2escTxInProgress) {
        const uint32_t now = micros();
        /* On some targets/drivers, isSerialTransmitBufferEmpty() can lag well past
         * the actual on-wire completion time, which stalls subsequent frames.
         * Use a conservative time-based completion as a backstop. */
        const bool timeDone = (srxl2escTxExpectedDoneUs != 0) && (cmpTimeUs(now, srxl2escTxExpectedDoneUs) >= 0);
        if (isSerialTransmitBufferEmpty(serialPort) || timeDone || cmpTimeUs(now, srxl2escTxActiveSinceUs) > SRXL2_ESC_TX_TIMEOUT_US) {
            serialEndWrite(serialPort);
            srxl2escTxInProgress = false;
            srxl2escTxExpectedDoneUs = 0;
        }
        return;
    }

    if (writeBufferIdx == 0) {
        return;
    }

    uint32_t now = micros();
    const bool seenIdleSinceLastRx = cmpTimeUs(lastIdleTimestamp, lastReceiveTimestamp) > 0;
    const uint32_t deadline = lastReceiveTimestamp + SRXL2_ESC_REPLY_QUIESCENCE;
    const bool lineQuietEnough = cmpTimeUs(now, deadline) >= 0;
    if (!seenIdleSinceLastRx && !lineQuietEnough) {
        /* still receiving bytes, wait for idle or quiescence */
        return;
    }

    if (!lineQuietEnough) {
        uint32_t spins = 0;
        while (cmpTimeUs(now, deadline) < 0 && spins < 1000) {
            now = micros();
            spins++;
        }
        if (cmpTimeUs(now, deadline) < 0) {
            /* give up if deadline still not met to avoid stalling other work */
            return;
        }
    }

    srxl2escLastTxSkipSet = writeBufferIdx;  // Debug: record frame size
    /* Set up echo suppression: skip exactly writeBufferIdx bytes that will
     * echo back on this half-duplex bus */
    txBytesToSkip = writeBufferIdx;
    readBufferPtr->len = 0;
    readBufferIdx = 0;


    serialBeginWrite(serialPort);
    /* Record TX start timestamp into the first matching event slot (if any) */
    const uint32_t txStartUs = micros();
    serialWriteBuf(serialPort, writeBuffer, writeBufferIdx);
    srxl2escTxInProgress = true;
    srxl2escTxActiveSinceUs = txStartUs;
    /* Predict completion time from baud and bytes (10 bits per byte on 8N1).
     * Add a small margin for ISR/line turnaround. */
    {
        const uint32_t baud = serialGetBaudRate(serialPort);
        const uint32_t bytes = srxl2escLastTxSkipSet;
        if (baud > 0 && bytes > 0) {
            const uint32_t bits = bytes * 10u;
            const uint32_t durUs = (uint32_t)(((uint64_t)bits * 1000000ull + (uint64_t)baud - 1ull) / (uint64_t)baud);
            srxl2escTxExpectedDoneUs = txStartUs + durUs + 300u;
        } else {
            srxl2escTxExpectedDoneUs = txStartUs + 2000u;
        }
    }
    /* Don't touch echo tracking here - the ISR handles it.
     * The ESC reply may already be arriving. */
    
    writeBufferIdx = 0;
}

/* smartescFlushWriteBufferBlocking removed: flushing is now handled from service() */

// @note assumes packet is fully there
void srxl2escProcess(srxl2esc_runtimeState_t *runtimeState)
{
    /* capture instantaneous snapshot of what we are about to parse */
    const unsigned headerLen = processBufferPtr->packet.header.length;
    const uint8_t packetType = processBufferPtr->packet.header.packetType;
    
    if (processBufferPtr->packet.header.id != packet_HEADER || processBufferPtr->len != processBufferPtr->packet.header.length) {
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
    srxl2escLastProcessedLen = headerLen;
    srxl2escLastProcessedPacketType = packetType;

    if (srxl2escProcessPacket(&processBufferPtr->packet.header, runtimeState)) {
        return;
    }

    DEBUG_PRINTF("could not parse packet: %x\r\n", processBufferPtr->packet.header.packetType);
    globalResult = RX_FRAME_DROPPED;
}

void srxl2escDataReceive(uint16_t character, void *data)
{
    UNUSED(data);
    lastReceiveTimestamp = microsISR();

    /* Echo suppression: skip exactly txBytesToSkip bytes after TX */
    if (txBytesToSkip > 0) {
        txBytesToSkip--;
        return;  /* Skip this echo byte entirely */
    }

    /* If we see a new frame header (0xA6) and already have data in buffer,
     * this means a new frame is starting. Check if the previous frame is
     * complete and swap it to the process buffer. If incomplete, discard it. */
    if (character == packet_HEADER && readBufferIdx > 0) {
        /* Check if previous frame looks complete */
        if (readBufferIdx >= 3) {
            const uint8_t expectedLen = readBufferPtr->packet.raw[2];
            if (readBufferIdx == expectedLen) {
                /* Previous frame is complete - try to move it to processing or queue it */
                if (processBufferPtr->len == 0) {
                    /* Process buffer free: hand this buffer to the processor and pick a spare for further reads */
                    lastIdleTimestamp = microsISR();
                    struct escBufse *spare = srxl2escGetSpareBuffer();
                    /* Make current read buffer the process buffer */
                    processBufferPtr = readBufferPtr;
                    /* Select a new read buffer (spare). If no spare found, fall back to the other buffer */
                    if (spare == NULL) {
                        for (int i = 0; i < 3; ++i) {
                            struct escBufse *b = &readBufferse[i];
                            if (b != processBufferPtr) { spare = b; break; }
                        }
                    }
                    readBufferPtr = spare;
                    processBufferPtr->len = readBufferIdx;                    readBufferPtr->len = 0;
                    readBufferIdx = 0;
                } else {
                    /* Process buffer busy: try to queue this completed frame into a spare buffer */
                    struct escBufse *spare = srxl2escGetSpareBuffer();
                    if (spare != NULL) {
                        unsigned copyLen = readBufferIdx;
                        if (copyLen > SRXL2_ESC_MAX_PACKET_LENGTH) copyLen = SRXL2_ESC_MAX_PACKET_LENGTH;
                        for (unsigned i = 0; i < copyLen; ++i) {
                            spare->packet.raw[i] = readBufferPtr->packet.raw[i];
                        }
                        /* ensure snapshot helpers see this as soon as possible */
                        /* spare->len must be set before ISR snapshot; it already is set below */
                        spare->len = copyLen;
                        /* start a fresh frame in the current read buffer */
                        readBufferIdx = 0;
                        readBufferPtr->len = 0;
                    } else {
                        /* No spare available - discard and start fresh */
                        readBufferIdx = 0;
                    }
                }
            } else {
                /* Frame incomplete - discard it and start fresh with this 0xA6 */
                readBufferIdx = 0;
            }
        } else {
            /* Less than 3 bytes - discard and start fresh */
            readBufferIdx = 0;
        }
    }

    //If the buffer index exceeds max length, disable reception
    if (readBufferIdx >= SRXL2_ESC_MAX_PACKET_LENGTH) {
        readBufferIdx = 0;
        globalResult = RX_FRAME_DROPPED;
    }
    else {
        readBufferPtr->packet.raw[readBufferIdx] = character;
        readBufferIdx++;
    }
}

void srxl2escIdle()
{
    if (readBufferIdx == 0) { // No data received
        readBufferPtr->len = 0;
        return;
    }

    // Validate frame completeness before swapping:
    // Need at least 3 bytes to read header (id, packetType, length)
    if (readBufferIdx < 3) {
        // Incomplete header, don't swap yet - wait for more bytes
        return;
    }

    // Check if we have the full frame based on the length field
    const uint8_t expectedLen = readBufferPtr->packet.raw[2];  // length field
    if (expectedLen > 0 && expectedLen <= SRXL2_ESC_MAX_PACKET_LENGTH && readBufferIdx < expectedLen) {
        // Frame incomplete, don't swap yet - wait for remaining bytes
        return;
    }

    // Frame looks complete (or invalid length which we'll catch in processing)
    lastIdleTimestamp = microsISR();

    /* Only swap if process buffer is empty - don't overwrite unprocessed frame! */
    if (processBufferPtr->len > 0) {
        /* Process buffer still has data - try to queue the current read buffer into a spare buffer */
        struct escBufse *spare = srxl2escGetSpareBuffer();
        if (spare != NULL) {
            unsigned copyLen = readBufferIdx;
            if (copyLen > SRXL2_ESC_MAX_PACKET_LENGTH) copyLen = SRXL2_ESC_MAX_PACKET_LENGTH;
            for (unsigned i = 0; i < copyLen; ++i) {
                spare->packet.raw[i] = readBufferPtr->packet.raw[i];
            }
            spare->len = copyLen;
            /* start fresh in current read buffer */
            readBufferIdx = 0;
            readBufferPtr->len = 0;
            return;
        }
        /* No spare available - discard current frame */
        readBufferIdx = 0;
        return;
    }

    /* Swap read and process buffer pointers */
    if (processBufferPtr == &readBufferse[0]) {
        processBufferPtr = &readBufferse[1];
        readBufferPtr = &readBufferse[0];
    } else {
        processBufferPtr = &readBufferse[0];
        readBufferPtr = &readBufferse[1];
    }
    processBufferPtr->len = readBufferIdx;
    
    readBufferPtr->len = 0;
    readBufferIdx = 0;
}

static inline void put_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

#define BE16(x) (uint8_t)((x) >> 8), (uint8_t)((x) & 0xFF)

static uint8_t srxl2escFrameStatus(srxl2esc_runtimeState_t *runtimeState)
{
    UNUSED(rxRuntimeState);

    globalResult = RX_FRAME_PENDING;

    // len should only be set after an idle interrupt (packet reception complete)
    if (processBufferPtr != NULL && processBufferPtr->len) {
        // About to process packet
        srxl2escProcess(runtimeState);
        // Finished processing
        processBufferPtr->len = 0;

        /* After processing, check if any queued buffer (spare) has data
         * and immediately process it as well. Avoid touching the active
         * read buffer (readBufferPtr). */
        bool found;
        do {
            found = false;
            for (int i = 0; i < 3; ++i) {
                struct escBufse *b = &readBufferse[i];
                if (b == readBufferPtr) continue; /* skip active write buffer */
                if (b->len > 0) {
                    processBufferPtr = b;
                    srxl2escProcess(runtimeState);
                    processBufferPtr->len = 0;
                    found = true;
                    break; /* restart scan */
                }
            }
        } while (found);
    } 

    uint8_t result = globalResult;

    const uint32_t now = micros();

    switch (state) {
    case Disabled: 
        break;

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

                if(currentBaud == SRXL2_ESC_PORT_BAUDRATE_DEFAULT)
                    serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_HIGH);
                else
                    serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            }
        } else if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo if there was activity - detect baudrate and ListenForHandshake

            if (unitId == 0) {
                state = SendHandshake;
                timeoutTimestamp = now + SRXL2_ESC_SEND_HANDSHAKE_TIMEOUT_US;
                fullTimeoutTimestamp = now + SRXL2_ESC_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            } else {
                state = ListenForHandshake;
                timeoutTimestamp = now + SRXL2_ESC_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            }
        }
    } 
        // strcpy(srxl2escStateString, "ListenForActivity");
        break;

    case SendHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo set another timeout for 50ms tries
            // fill write buffer with handshake frame
            result |= RX_FRAME_PROCESSING_REQUIRED;
        }

        if (cmpTimeUs(now, fullTimeoutTimestamp) >= 0) {
            serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case SendHandshake: switching to %d baud\r\n", SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + SRXL2_ESC_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } 
        // strcpy(srxl2escStateString, "SendHandshake");
        break;

    case ListenForHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0)  {
            serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case ListenForHandshake: switching to %d baud\r\n", SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + SRXL2_ESC_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } 
        // strcpy(smartescStateString, "ListenForHandshake");
        break;

    case Running: {
        // frame timed out, reset state
        if (cmpTimeUs(now, lastValidPacketTimestamp) >= SRXL2_ESC_FRAME_TIMEOUT_US) {
            serialSetBaudRate(serialPort, SRXL2_ESC_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case Running: switching to %d baud: %d %d\r\n", SRXL2_ESC_PORT_BAUDRATE_DEFAULT, now, lastValidPacketTimestamp);
            timeoutTimestamp = now + SRXL2_ESC_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
            lastValidPacketTimestamp = 0;
        }
    } 
        break;
    };

    if (writeBufferIdx) {
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }

    if (!(result & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        runtimeState->lastRcFrameTimeUs = lastIdleTimestamp;
    }

    return result;
}

static bool srxl2escProcessFrame(const srxl2esc_runtimeState_t *runtimeState)
{
    UNUSED(runtimeState);

    if (srxl2escBroadcastConfirmPending && writeBufferIdx == 0 && !srxl2escTxInProgress) {
        srxl2escWriteData(&srxl2escBroadcastConfirmFrame, sizeof(srxl2escBroadcastConfirmFrame));
        srxl2escBroadcastConfirmPending = false;
        srxl2escAdvanceHandshakeStage(SRXL2_ESC_HANDSHAKE_STAGE_DONE);
    }
    else if (srxl2escHandshakeComplete && !srxl2escTxInProgress && writeBufferIdx == 0) {
        uint32_t now = micros();
        /* Keep telemetry state progressing, but do not stall throttle/channel frames
         * for the whole telemetry RX timeout. */
        if (srxl2escTelemetryRxPending) {
            if (srxl2escTelemetryRxDeadlineUs == 0 || cmpTimeUs(now, srxl2escTelemetryRxDeadlineUs) >= 0) {
                srxl2escTelemetryRxPending = false;

                srxl2escTelemetryRxDeadlineUs = 0;
            }
        }
        bool telemetryHoldoffActive = false;
        if (srxl2escTelemetryHoldoffEndUs != 0) {
            if (cmpTimeUs(now, srxl2escTelemetryHoldoffEndUs) < 0) {
                telemetryHoldoffActive = true;
            } else {
                srxl2escTelemetryHoldoffEndUs = 0;
            }
        }
        srxl2escUpdateThrottleFromMotors();
        if (!telemetryHoldoffActive && (srxl2escNextThrottleSendUs == 0 || cmpTimeUs(now, srxl2escNextThrottleSendUs) >= 0)) {
            if (srxl2escQueueChannelDataFrame(srxl2escThrottleCommand)) {
                srxl2escNextThrottleSendUs = now + srxl2escThrottleIntervalUs;
            }
        }
    }

    srxl2escTrySendPendingWriteBuffer();

    return (writeBufferIdx == 0);
}

static float srxl2escReadRawRC(const srxl2esc_runtimeState_t *runtimeState, uint8_t channelIdx)
{
    if (channelIdx >= runtimeState->channelCount) {
        return 0;
    }

    return ((float)(runtimeState->channelData[channelIdx] >> SRXL2_ESC_CHANNEL_SHIFT) / 16) + SPEKTRUM_PULSE_OFFSET;
}

/* Provide a local frame-time helper so the SMART ESC driver does not
 * depend on the global RX subsystem. This returns the last idle
 * timestamp observed by the driver as an approximation of frame time. */
static timeUs_t srxl2escLocalFrameTimeUs(void) { return lastIdleTimestamp; }

void srxl2escWriteData(const void *data, int len)
{
    const uint16_t crc = crc16_ccitt_update(0, (uint8_t*)data, len - 2);
    ((uint8_t*)data)[len-2] = ((uint8_t *) &crc)[1] & 0xFF;
    ((uint8_t*)data)[len-1] = ((uint8_t *) &crc)[0] & 0xFF;

    len = MIN(len, (int)sizeof(writeBuffer));
    memcpy(writeBuffer, data, len);
    writeBufferIdx = len;
}

void srxl2escSetThrottleCommand(uint16_t value)
{
    srxl2escThrottleCommand = value;
    srxl2escNextThrottleSendUs = micros();
}

void srxl2escSetThrottleRateHz(uint32_t rateHz)
{
    if (rateHz == 0) {
        return;
    }
    /* If motor config is present, only accept rate changes that match the
     * authoritative `dev.motorPwmRate`. This prevents other modules (CLI/MSP)
     * from directly changing SMART ESC refresh; changes must come via motor
     * configuration. */
#if defined(USE_MOTOR)
    if (motorConfig()->dev.motorPwmRate != 0 && motorConfig()->dev.motorPwmRate != rateHz) {
        return;
    }
#endif
    const uint32_t minIntervalUs = srxl2escComputeMinThrottleIntervalUs();
    uint32_t interval = 1000000 / rateHz;
    if (interval < minIntervalUs) {
        interval = minIntervalUs;
    }
    srxl2escThrottleIntervalUs = interval;
    srxl2escNextThrottleSendUs = micros();

    // Persist effective value
    const uint16_t effectiveRate = (uint16_t)(1000000 / srxl2escThrottleIntervalUs);
    srxl2escConfigMutable()->throttle_rate_hz = effectiveRate;
}

uint32_t srxl2escGetThrottleRateHz(void)
{
    if (srxl2escThrottleIntervalUs == 0) {
        return 0;
    }
    return 1000000 / srxl2escThrottleIntervalUs;
}

void srxl2escSetTelemetryIntervalFrames(uint8_t frames)
{
    srxl2escTelemetryFrameInterval = frames;
    srxl2escThrottleFrameCounter = 0;

    // Persist
    srxl2escConfigMutable()->telem_interval_frames = frames;
    
    /* If telemetry polling is disabled, clear any pending wait/holdoff so we don't
     * introduce a one-time timeout stall from a previous request. */
    if (frames == 0) {
        srxl2escTelemetryRxPending = false;
        srxl2escTelemetryRxDeadlineUs = 0;
        srxl2escTelemetryHoldoffEndUs = 0;
    }
}

uint8_t srxl2escGetTelemetryIntervalFrames(void)
{
    return srxl2escTelemetryFrameInterval;
}

unsigned srxl2escGetTelemetryHistoryCount(void)
{
    return srxl2escTelemetryHistoryCount;
}

unsigned srxl2escCopyTelemetryHistory(unsigned idx, uint8_t *dst, unsigned maxLen, uint8_t *outLen, uint32_t *outTimestamp, uint8_t *outSensorId, uint8_t *outSecondaryId, uint32_t *outCount)
{
    if (idx >= srxl2escTelemetryHistoryCount) {
        return 0;
    }
    const unsigned base = (srxl2escTelemetryHistoryHead + SRXL2_ESC_TELEM_HISTORY - srxl2escTelemetryHistoryCount) % SRXL2_ESC_TELEM_HISTORY;
    const unsigned slotIndex = (base + idx) % SRXL2_ESC_TELEM_HISTORY;
    const srxl2escTelemetryHistoryEntry_t *slot = &srxl2escTelemetryHistory[slotIndex];
    const unsigned copyLen = MIN(sizeof(slot->data), maxLen);
    if (dst && copyLen) {
        memcpy(dst, slot->data, copyLen);
    }
    if (outLen) {
        *outLen = sizeof(slot->data);
    }
    if (outTimestamp) {
        *outTimestamp = slot->timestamp;
    }
    if (outSensorId) {
        *outSensorId = slot->sensorId;
    }
    if (outSecondaryId) {
        *outSecondaryId = slot->secondaryId;
    }
    if (outCount) {
        *outCount = slot->count;
    }
    return copyLen;
}

bool srxl2escGetLatestTelemetry(uint8_t *sensorId, uint8_t *secondaryId, uint8_t *dst, unsigned maxLen)
{
    if (!srxl2escLatestTelemetryValid) {
        return false;
    }

    /* Do not expose sensor type 0x0C or 0x42 for transmission — caller (SRXL2)
     * will skip sending these frames. Keep the data recorded for history
     * and for internal injection into escSensorData_t. */
    if (srxl2escLatestTelemetrySensorId == 0x0C || 0x42) {
        return false;
    }

    if (sensorId) {
        *sensorId = srxl2escLatestTelemetrySensorId;
    }
    if (secondaryId) {
        *secondaryId = srxl2escLatestTelemetrySecondaryId;
    }
    if (dst && maxLen) {
        memcpy(dst, srxl2escLatestTelemetryData, MIN(sizeof(srxl2escLatestTelemetryData), maxLen));
    }

    return true;
}

bool srxl2escCopyLatestTelemetry(srxl2escTelemetrySnapshot_t *out, uint32_t *outSeq)
{
    if (!out) {
        return false;
    }

    uint32_t seqStart;
    uint32_t seqEnd;
    do {
        seqStart = srxl2escLatestTelemetrySeq;
        if (seqStart & 1U) {
            continue;
        }
        *out = srxl2escLatestTelemetrySnapshot;
        seqEnd = srxl2escLatestTelemetrySeq;
    } while (seqStart != seqEnd);

    if (outSeq) {
        *outSeq = seqEnd;
    }

    return out->valid;
}

void validateAndFixSrxl2escConfig()
{
    // Force half duplex
    escSensorConfigMutable()->halfDuplex = true;

    // Force SRXL2 protocol
    escSensorConfigMutable()->protocol = ESC_SENSOR_PROTO_SRXL2;

    // Force motor pwm protocol to SRXL2
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_SRXL2;

    // Clamp/repair persisted SRXL2 ESC rate settings
    srxl2esc_pgConfig_t *cfg = srxl2escConfigMutable();
    if (cfg->throttle_rate_hz == 0) {
        cfg->throttle_rate_hz = 250;
    }
    // Guard against flooding (min interval 500us => max 2000Hz)
    if (cfg->throttle_rate_hz > 2000) {
        cfg->throttle_rate_hz = 2000;
    }
}

bool srxl2escInit(const srxl2esc_config_t *escConfig, srxl2esc_runtimeState_t *srxl2escRs)
{
    (void)srxl2escRs; /* legacy parameter ignored; driver uses internal runtime */

    /* Initialize internal runtime state and channel storage */
    for (unsigned i = 0; i < SRXL2_ESC_MAX_CHANNELS; i++)
        srxl2escInternalChannelData[i] = SRXL2_ESC_CHANNEL_CENTER;

    memset(&srxl2escInternalRs, 0, sizeof(srxl2escInternalRs));
    srxl2escInternalRs.channelData      = srxl2escInternalChannelData;
    srxl2escInternalRs.channelCount     = SRXL2_ESC_MAX_CHANNELS;
    srxl2escInternalRs.rxRefreshRate    = SRXL2_ESC_FRAME_PERIOD_US;
    srxl2escInternalRs.rcReadRawFn      = srxl2escReadRawRC;
    srxl2escInternalRs.rcFrameStatusFn  = srxl2escFrameStatus;
    /* Provide a local frame-time helper so the SMART ESC driver does not
     * depend on the global RX subsystem. This returns the last idle
     * timestamp observed by the driver as an approximation of frame time. */
    srxl2escInternalRs.rcFrameTimeUsFn  = srxl2escLocalFrameTimeUs;
    srxl2escInternalRs.rcProcessFrameFn = srxl2escProcessFrame;

    /* Point global pointer to internal runtime state */
    srxl2escRsGlobal = &srxl2escInternalRs;

    // Reuse already opened port (escSensorInit owns it)
    if (!serialPort) {
        return false;
    }

    serialPort->idleCallback = srxl2escIdle; // idempotent

    unitId   = escConfig ? escConfig->srxl2_unit_id : 0;
    baudRate = 0;

    state = ListenForActivity;
    timeoutTimestamp = micros() + SRXL2_ESC_LISTEN_FOR_ACTIVITY_TIMEOUT_US;

    // Initialize SMART ESC throttle rate from motor config when present
#if defined(USE_MOTOR)
    /* Use motor config as authoritative source for throttle refresh. Fall
     * back to persisted SMART ESC value if motor PG is zero/unset. If the
     * SMART ESC driver is active, enforce a maximum of 250Hz. */
    srxl2esc_lastMotorPwmRate = motorConfig()->dev.motorPwmRate;
    if (srxl2esc_lastMotorPwmRate == 0) {
        srxl2esc_lastMotorPwmRate = srxl2escConfig()->throttle_rate_hz;
    }
    if (srxl2escIsActive() && srxl2esc_lastMotorPwmRate > 250) {
        motorConfigMutable()->dev.motorPwmRate = 250;
        srxl2esc_lastMotorPwmRate = 250;
    }
    srxl2escSetThrottleRateHz(srxl2esc_lastMotorPwmRate);
#else
    srxl2escSetThrottleRateHz(srxl2escConfig()->throttle_rate_hz);
#endif
    srxl2escSetTelemetryIntervalFrames(srxl2escConfig()->telem_interval_frames);

    return true;
}

bool srxl2escIsActive(void)
{
    return serialPort;
}

bool srxl2escTelemetryRequested(void)
{
    return telemetryRequested;
}

void srxl2escInitializeFrame(sbuf_t *dst)
{
    dst->ptr = telemetryFrame;
    dst->end = ARRAYEND(telemetryFrame);

    sbufWriteU8(dst, packet_HEADER);
    sbufWriteU8(dst, TelemetrySensorData);
    sbufWriteU8(dst, ARRAYLEN(telemetryFrame));
    sbufWriteU8(dst, busMasterDeviceId);
}

void smartescFinalizeFrame(sbuf_t *dst)
{
    UNUSED(dst); // we don't need to switch to reader; we know the buffer & length
    const int frameLen = telemetryFrame[2];   // srxl2esc length (includes CRC)
    srxl2escWriteData(telemetryFrame, frameLen);
    telemetryRequested = false;
}

void srxl2escAttachPort(serialPort_t *p)
{
    serialPort = p;
    if (serialPort) serialPort->idleCallback = srxl2escIdle;  // set boundary finisher
}

bool srxl2escDriverInit(void)
{
    if (srxl2escDriverReady && srxl2escDriverPort) {
        return true;
    }

    srxl2escDriverReady = false;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SRXL2_ESC);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | SERIAL_BIDIR;
    options |= escSensorConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP;

    srxl2escDriverPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_SRXL2_ESC,
        srxl2escDataReceive,
        NULL,
        SRXL2_ESC_PORT_BAUDRATE_DEFAULT,
        MODE_RXTX,
        options);
    if (!srxl2escDriverPort) {
        return false;
    }

    srxl2escAttachPort(srxl2escDriverPort);

    memset(&srxl2escDriverRuntime, 0, sizeof(srxl2escDriverRuntime));
    
    /* Use a default local config here to avoid referencing escConfig()/rx types.
     * If you want board-specific unit id or pin-swap, set them via a
     * platform-specific setter or call `srxl2escInit` directly with values. */
    srxl2esc_config_t localCfg = { .srxl2_unit_id = 0, .pinSwap = false };
    if (!srxl2escInit(&localCfg, &srxl2escDriverRuntime)) {
        closeSerialPort(srxl2escDriverPort);
        srxl2escDriverPort = NULL;
        return false;
    }

    /* If we're the initiator (unitId == 0), actively send handshakes in a blocking
     * loop for up to 200ms before returning. This ensures we transmit within the
     * ESC's 250ms listening window, before the scheduler starts. */
    if (localCfg.srxl2_unit_id == 0) {
        const uint32_t startUs = micros();
        const uint32_t deadlineUs = startUs + 200000; // 200ms window
        uint32_t nextHandshakeUs = startUs;
        
        while (cmpTimeUs(micros(), deadlineUs) < 0) {
            const uint32_t now = micros();
            
            /* Send handshake every 50ms */
            if (cmpTimeUs(now, nextHandshakeUs) >= 0) {
                Srxl2HandshakeFrame handshake = {
                    .header = {
                        .id = packet_HEADER,
                        .packetType = Handshake,
                        .length = sizeof(Srxl2HandshakeFrame),
                    },
                    .payload = {
                        .sourceDeviceId      = (FlightController << 4) | localCfg.srxl2_unit_id,
                        .destinationDeviceId = 0x40, // ESC device ID
                        .priority            = 10,
                        .baudSupported       = 0,
                        .info                = 0,
                        .uniqueId            = 0x4156,
                    },
                };
                
                srxl2escWriteData(&handshake, sizeof(handshake));
                srxl2escTrySendPendingWriteBuffer();
                nextHandshakeUs = now + 50000; // next attempt in 50ms
            }
            
            /* Process any incoming handshake responses */
            srxl2esc_poll();
            
            /* If we got a valid handshake response, exit early */
            if (state == Running || busMasterDeviceId != 0xFF) {
                break;
            }
            
            /* Small delay to prevent busy-loop CPU saturation */
            delayMicroseconds(100);
        }
    }

    srxl2escDriverReady = true;
    return true;
}

void srxl2escDriverTask(timeUs_t currentTimeUs)
{
    static timeUs_t nextInitAttemptUs = 0;

    if (!srxl2escDriverReady) {
        const timeUs_t now = currentTimeUs ? currentTimeUs : micros();
        if (nextInitAttemptUs == 0 || cmpTimeUs(now, nextInitAttemptUs) >= 0) {
            nextInitAttemptUs = now + 500000; // retry every 500ms
            if (srxl2escDriverInit()) {
                return;
            }
        }
        return;
    }

    srxl2esc_poll();
    srxl2esc_service();
}

bool srxl2escDriverIsReady(void)
{
    return srxl2escDriverReady && srxl2escIsActive();
}

void srxl2esc_poll(void)
{
    if (!srxl2escRsGlobal) {
        return;
    }

    /* update frame status and process any complete frames */
    (void)srxl2escFrameStatus(srxl2escRsGlobal);
}

void srxl2esc_service(void)
{
    if (!srxl2escRsGlobal) {
        return;
    }

    /* handle any pending transmissions */
    (void)srxl2escProcessFrame(srxl2escRsGlobal);

    #if defined(USE_MOTOR)
    /* If motor PWM rate changed externally (CLI/MSP/PG), adopt it as the
     * SMART ESC throttle refresh rate. This makes `dev.motorPwmRate` the
     * authoritative source. When SMART ESC is active, cap the maximum rate
     * to 250Hz. */
    uint16_t current = motorConfig()->dev.motorPwmRate;
    if (srxl2escIsActive() && current > 250) {
        motorConfigMutable()->dev.motorPwmRate = 250;
        current = 250;
    }
    if (current != srxl2esc_lastMotorPwmRate && current != 0) {
        srxl2esc_lastMotorPwmRate = current;
        srxl2escSetThrottleRateHz((uint32_t)current);
    }
#endif
}

#endif // USE_SRXL2_ESC
