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

#ifdef USE_SMART_ESC

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "drivers/smart_esc.h"
#include "rx/srxl2_types.h"
#include "io/spektrum_vtx_control.h"
#include "rx/rx.h"
#include "../sensors/esc_sensor.h"
#include "config/feature.h"
#include "pg/smart_esc.h"
#if defined(USE_MOTOR)
#include "flight/motors.h"
#include "drivers/motor.h"
#include "pg/motor.h"
#endif

#ifndef smartesc_DEBUG
#define smartesc_DEBUG 0
#endif

#if smartesc_DEBUG
#define DEBUG_PRINTF(...) //Temporary until a better debug printf can be included
#else
#define DEBUG_PRINTF(...)
#endif

#define smartesc_MAX_CHANNELS             32
#define smartesc_FRAME_PERIOD_US          11000 // 5500 for DSMR
#define smartesc_CHANNEL_SHIFT            2
#define smartesc_CHANNEL_CENTER           0x8000

#define smartesc_PORT_BAUDRATE_DEFAULT    SMARTESC_PORT_BAUDRATE_DEFAULT
#define smartesc_PORT_BAUDRATE_HIGH       SMARTESC_PORT_BAUDRATE_HIGH
#define smartesc_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define smartesc_PORT_MODE                MODE_RXTX

/* Minimum quiet time between RX and TX so we flip the half-duplex line cleanly.
 * Dropping this to roughly a single byte-time (≈80us at 115200) makes handshake
 * responses fire sooner instead of waiting ~2 byte times (~174us). */
#define smartesc_REPLY_QUIESCENCE         80
#define smartesc_TX_TIMEOUT_US             20000
#define smartesc_TELEMETRY_RX_TIMEOUT_US   10000
#define smartesc_TELEMETRY_POST_HOLDOFF_US   300

/* Guard time after *requesting* telemetry before we consider transmitting again.
 * Needs to be long enough to cover ESC reply start latency + line turnaround,
 * otherwise FC TX can collide with the ESC reply on the half-duplex bus. */
#define smartesc_TELEMETRY_REQUEST_HOLDOFF_US  2500

/* Upper bound for the request holdoff. Prevents long stalls (e.g. 10ms) when
 * throttle update rate is configured low. */
#define smartesc_TELEMETRY_REQUEST_HOLDOFF_MAX_US  5000
#define SMARTESC_TELEM_HISTORY             15

#define packet_HEADER                     0xA6
#define smartesc_MAX_PACKET_LENGTH        80
#define smartesc_DEVICE_ID_BROADCAST      0xFF

#define smartesc_FRAME_TIMEOUT_US         50000

#define smartesc_LISTEN_FOR_ACTIVITY_TIMEOUT_US 50000
#define smartesc_SEND_HANDSHAKE_TIMEOUT_US 50000
#define smartesc_LISTEN_FOR_HANDSHAKE_TIMEOUT_US 200000

#define SPEKTRUM_PULSE_OFFSET          988 // Offset value to convert digital data into RC pulse

typedef union {
        uint8_t raw[smartesc_MAX_PACKET_LENGTH];
        Srxl2Header header;
} smartescFrame;

struct rxBufse {
    volatile unsigned len;
    smartescFrame packet;
};

static uint8_t unitId = 0;
static uint8_t baudRate = 0;

static Srxl2State state = Disabled;
static uint32_t timeoutTimestamp = 0;
static uint32_t fullTimeoutTimestamp = 0;
static uint32_t lastValidPacketTimestamp = 0;
static volatile uint32_t lastReceiveTimestamp = 0;
static volatile uint32_t lastIdleTimestamp = 0;

static struct rxBufse readBufferse[2];
static struct rxBufse* readBufferPtr = &readBufferse[0];
static struct rxBufse* processBufferPtr = &readBufferse[1];
static volatile unsigned readBufferIdx = 0;
/* Echo-suppression: After TX, skip exactly txBytesToSkip bytes.
 * This is a simple count-based approach - no byte matching needed. */
static volatile unsigned txBytesToSkip = 0;
static uint8_t writeBuffer[smartesc_MAX_PACKET_LENGTH];
static unsigned writeBufferIdx = 0;

static serialPort_t *serialPort;
static serialPort_t *smartescDriverPort;
static smartesc_rxRuntimeState_t smartescDriverRuntime;
static bool smartescDriverReady;

static uint8_t busMasterDeviceId = 0xFF;
static bool telemetryRequested = false;
static bool smartescTelemetryRxPending = false;
static uint32_t smartescTelemetryRxDeadlineUs = 0;
static uint32_t smartescTelemetryHoldoffEndUs = 0;

static uint8_t telemetryFrame[22];

typedef struct {
    uint32_t timestamp;
    uint32_t count;
    uint8_t sensorId;
    uint8_t secondaryId;
    uint8_t data[14];
} smartescTelemetryHistoryEntry_t;
static smartescTelemetryHistoryEntry_t smartescTelemetryHistory[SMARTESC_TELEM_HISTORY];
static uint8_t smartescTelemetryHistoryCount = 0;
static uint8_t smartescTelemetryHistoryHead = 0;
static uint8_t smartescLatestTelemetrySensorId = 0;
static uint8_t smartescLatestTelemetrySecondaryId = 0;

static uint8_t smartescLatestTelemetryData[14];
static bool smartescLatestTelemetryValid = false;

static uint8_t globalResult = 0;

/* Internal runtime state owned by SMART ESC driver (decoupled from RX subsystem) */
static smartesc_rxRuntimeState_t smartescInternalRs;
static uint16_t smartescInternalChannelData[smartesc_MAX_CHANNELS];

/* file-static pointer to runtime state so poll/service helpers can operate
 * smartescRsGlobal points to our internal runtime by default. Legacy callers
 * may pass a runtime into smartescInit, but the driver uses its internal
 * state to avoid coupling to the global RX subsystem. */
static smartesc_rxRuntimeState_t *smartescRsGlobal = NULL;

static uint32_t smartescBootEndUs = 0;

#if defined(USE_MOTOR)
static uint16_t smartesc_lastMotorPwmRate = 0;
#endif

/* Forward declarations */
bool smartescIsActive(void);

static volatile unsigned smartescLastTxSkipSet = 0;  // Debug: last transmitted frame size tracked for echo suppression


// /* Timing capture: record durations (microseconds) for first few executions */
#define SMARTESC_TIMING_SLOTS 5

static bool smartescTxInProgress = false;
static uint32_t smartescTxActiveSinceUs = 0;
static uint32_t smartescTxExpectedDoneUs = 0;

/* Helper: return a spare buffer pointer that's not the current read or process buffer.
 * Returns NULL if none available (shouldn't happen with 3 buffers unless pointers overlap). */
static struct rxBufse* smartescGetSpareBuffer(void)
{
    for (int i = 0; i < 3; ++i) {
        struct rxBufse *b = &readBufferse[i];
        if (b != readBufferPtr && b != processBufferPtr) return b;
    }
    return NULL;
}

static volatile unsigned smartescLastProcessedLen = 0;
static volatile uint8_t smartescLastProcessedPacketType = 0;

#define SMARTESC_HANDSHAKE_FRAME_LIMIT 4
#define SMARTESC_HANDSHAKE_MAX_BYTES 24

typedef enum {
    SMARTESC_HANDSHAKE_STAGE_IDLE = 0,
    SMARTESC_HANDSHAKE_STAGE_WAITING_ESC_REPLY,
    SMARTESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK,
    SMARTESC_HANDSHAKE_STAGE_DONE,
} smartescHandshakeStage_t;

static smartescHandshakeStage_t handshakeStage = SMARTESC_HANDSHAKE_STAGE_IDLE;
static bool smartescHandshakeComplete = false;
static bool smartescBroadcastConfirmPending = false;
static Srxl2HandshakeFrame smartescBroadcastConfirmFrame;
typedef struct __attribute__((packed)) {
    Srxl2Header header;
    Srxl2ControlDataSubHeader control;
    Srxl2ChannelDataHeader channel;
    uint16_t channelValue;
    uint8_t crcHigh;
    uint8_t crcLow;
} smartescChannelDataFrame_t;
static uint16_t smartescThrottleCommand = smartesc_CHANNEL_CENTER;
static uint32_t smartescNextThrottleSendUs = 0;
static uint32_t smartescThrottleIntervalUs = 1000000 / 250; /* 250 Hz default */
static uint8_t smartescThrottleFrameCounter = 0;
static uint8_t smartescTelemetryFrameInterval = 6; /* frames per telemetry poll (0=off) */

static uint32_t smartescGetActiveBaudRate(void)
{
    const uint32_t baud = serialPort ? serialGetBaudRate(serialPort) : 0;
    return baud ? baud : SMARTESC_PORT_BAUDRATE_DEFAULT;
}

static uint32_t smartescComputeMinThrottleIntervalUs(void)
{
    const uint32_t baud = smartescGetActiveBaudRate();
    const uint32_t bitsPerByte = 10; // 8N1
    const uint32_t bytes = (uint32_t)sizeof(smartescChannelDataFrame_t);
    const uint32_t txTimeUs = (uint32_t)((bytes * bitsPerByte * 1000000ULL + (baud - 1)) / baud);
    return txTimeUs + smartesc_REPLY_QUIESCENCE + 200;
}

static uint16_t smartescChannelValueToPulseWidth(uint16_t channelValue)
{
    return (uint16_t)(SPEKTRUM_PULSE_OFFSET + (channelValue / 64U));
}

static uint16_t smartescPulseWidthToChannelValue(uint16_t pulseWidthUs)
{
    if (pulseWidthUs <= SPEKTRUM_PULSE_OFFSET) {
        return 0;
    }

    const uint32_t scaled = (uint32_t)(pulseWidthUs - SPEKTRUM_PULSE_OFFSET) * 64U;
    return (uint16_t)MIN(scaled, 0xFFFFu);
}

static uint16_t smartescPulseFromNormalized(float normalized)
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

static float smartescNormalizedFromPulse(uint16_t pulseWidthUs)
{
    float normalized = 0.0f;
#if defined(USE_MOTOR)
    const motorConfig_t *config = motorConfig();
    const motorControlMode_e mode = config->dev.motorControlMode[0];
    const float pulse = (float)pulseWidthUs;
    if (mode == MOTOR_CONTROL_BIDIR) {
        normalized = scaleRangef(pulse,
            (float)config->minthrottle,
            (float)config->maxthrottle,
            -1.0f,
            1.0f);
        normalized = constrainf(normalized, -1.0f, 1.0f);
    } else {
        normalized = scaleRangef(pulse,
            (float)config->minthrottle,
            (float)config->maxthrottle,
            0.0f,
            1.0f);
        normalized = constrainf(normalized, 0.0f, 1.0f);
    }
#else
    normalized = constrainf(((float)pulseWidthUs - 1000.0f) / 1000.0f, 0.0f, 1.0f);
#endif
    return normalized;
}

#if defined(USE_MOTOR)
static void smartescUpdateThrottleFromMotors(void)
{
    if (getMotorCount() == 0) {
        return;
    }

    const float normalized = (float)getMotorOutput(0) / 1000.0f;
    const uint16_t pulse = smartescPulseFromNormalized(normalized);
    smartescThrottleCommand = smartescPulseWidthToChannelValue(pulse);
}
#endif

static void smartescAdvanceHandshakeStage(smartescHandshakeStage_t newStage);
static const char *smartescHandshakeStageToString(smartescHandshakeStage_t stage);
static void smartescScheduleBroadcastConfirm(uint8_t fcId);
static bool smartescQueueChannelDataFrame(uint16_t channelValue);
static void smartescRecordTelemetryFrame(const Srxl2Header *header);
static void smartescHandleTelemetryFrame(const Srxl2Header *header);

static void smartescScheduleBroadcastConfirm(uint8_t fcId)
{
    smartescBroadcastConfirmFrame = (Srxl2HandshakeFrame) {
        .header = {
            .id = packet_HEADER,
            .packetType = Handshake,
            .length = sizeof(Srxl2HandshakeFrame),
        },
        .payload = {
            .sourceDeviceId      = fcId,
            .destinationDeviceId = smartesc_DEVICE_ID_BROADCAST,
            .priority            = 10,
            .baudSupported       = baudRate,
            .info                = 0,
            .uniqueId            = 0,
        },
    };
    smartescBroadcastConfirmPending = true;
}

static bool smartescQueueChannelDataFrame(uint16_t channelValue)
{
    if (busMasterDeviceId == smartesc_DEVICE_ID_BROADCAST) {
        return false;
    }
    if (writeBufferIdx != 0 || smartescTxInProgress) {
        return false;
    }

    bool requestTelemetry = false;
    const uint32_t now = micros();
    if (smartescTelemetryFrameInterval > 0 && !smartescTelemetryRxPending) {
        if (++smartescThrottleFrameCounter >= smartescTelemetryFrameInterval) {
            smartescThrottleFrameCounter = 0;
            requestTelemetry = true;
            smartescTelemetryRxPending = true;
            smartescTelemetryRxDeadlineUs = now + smartesc_TELEMETRY_RX_TIMEOUT_US;
            /* Give the ESC a short window to start replying before we transmit again.
             * This avoids collisions without stalling for the full RX timeout. */
            uint32_t holdoffUs = smartescThrottleIntervalUs;
            if (holdoffUs < smartesc_TELEMETRY_REQUEST_HOLDOFF_US) {
                holdoffUs = smartesc_TELEMETRY_REQUEST_HOLDOFF_US;
            }
            if (holdoffUs > smartesc_TELEMETRY_REQUEST_HOLDOFF_MAX_US) {
                holdoffUs = smartesc_TELEMETRY_REQUEST_HOLDOFF_MAX_US;
            }
            smartescTelemetryHoldoffEndUs = now + holdoffUs;
        }
    }
    const uint8_t replyId = requestTelemetry ? busMasterDeviceId : 0x00;

    smartescChannelDataFrame_t frame = {
        .header = {
            .id = packet_HEADER,
            .packetType = ControlData,
            .length = sizeof(smartescChannelDataFrame_t),
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

    smartescWriteData(&frame, sizeof(frame));
    return true;
}

static void smartescHandleTelemetryFrame(const Srxl2Header *header)
{
    const uint8_t *payload = (const uint8_t *)(header + 1);
    uint8_t sensorId = 0;
    uint8_t secondaryId = 0;
    if (header->length >= 6) { /* header (3) + dest + sensor + secondary */
        sensorId = payload[1];
        secondaryId = payload[2];
    }
    smartescTelemetryRxPending = false;
    smartescTelemetryRxDeadlineUs = 0;
    const uint32_t rxCompleteUs = lastReceiveTimestamp ? lastReceiveTimestamp : micros();
    smartescTelemetryHoldoffEndUs = rxCompleteUs + smartesc_TELEMETRY_POST_HOLDOFF_US;
    /* Previously we filtered out sensor type 0x0C completely here which prevented
     * SRXL2 from ever having telemetry to send if those were the only frames.
     * Record all frames and keep the latest entry so other subsystems (ESC_SENSOR,
     * OSD, etc.) can use the data. Transmission filtering (so SRXL2 doesn't
     * forward 0x0C frames) is handled by smartescGetLatestTelemetry(). */
    smartescRecordTelemetryFrame(header);
    smartescLatestTelemetrySensorId = sensorId;
    smartescLatestTelemetrySecondaryId = secondaryId;
    memcpy(smartescLatestTelemetryData, payload + 3, sizeof(smartescLatestTelemetryData));
    smartescLatestTelemetryValid = true;

    // Attempt to map SMART ESC sensorId to a motor index and inject parsed escSensorData
    // so the rest of the system (telemetry backends) can see full ESC telemetry.
    if (getMotorCount() > 0) {
        int motorIndex = -1;
        const int motorCount = getMotorCount();

        /*
         * Some SMART/SRXL2 ESC implementations use sensor IDs in a block
         * (e.g. 0x20 .. 0x2F) to represent per-ESC live telemetry. Map
         * those into motor indices by subtracting the base (0x20).
         * Fall back to using a small numeric sensorId directly if it is
         * already in the range [0, motorCount).
         */
        if (sensorId < (uint8_t)motorCount) {
            motorIndex = (int)sensorId;
        } else if (sensorId >= 0x20 && sensorId < (uint8_t)(0x20 + motorCount)) {
            motorIndex = (int)(sensorId - 0x20);
        }

        if (motorIndex >= 0) {
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                // Parse SRXL2 ESC payload (big-endian) into escSensorData_t fields.
                // SRXL2 ESC payload layout (data[14], big-endian):
                //  [0..1] RPM (UINT16) electrical RPM units: 10 RPM (value * 10)
                //  [2..3] voltsInput (UINT16) units: 0.01 V (value / 100) -> mV = value * 10
                //  [4..5] tempFET (UINT16) units: 0.1 C
                //  [6..7] currentMotor (UINT16) units: 10 mA -> mA = value * 10
                //  [8..9] tempBEC (UINT16) units: 0.1 C
                //  [10]   currentBEC (UINT8) units: 100 mA -> mA = value * 100
                //  [11]   voltsBEC (UINT8) units: 0.05 V -> mV = value * 50
                //  [12]   throttle (UINT8) units: 0.5% -> 0.1% units = value * 5
                //  [13]   powerOut (UINT8) units: 0.5% (optional)

                escSensorData_t inj;
                memset(&inj, 0, sizeof(inj));
                inj.id = sensorId;
                inj.age = 0;

                const uint8_t *data = payload + 3;

                // RPM (big-endian)
                uint16_t rawRpm = (uint16_t)((data[0] << 8) | data[1]);
                if (rawRpm != 0xFFFF) {
                    // rawRpm units = 10 RPM
                    inj.erpm = (uint32_t)rawRpm * 10u;
                }

                // voltsInput -> mV
                uint16_t rawVolts = (uint16_t)((data[2] << 8) | data[3]);
                if (rawVolts != 0xFFFF) {
                    // rawVolts units = 0.01 V -> mV = rawVolts * 10
                    inj.voltage = (uint32_t)rawVolts * 10u;
                }

                // tempFET
                uint16_t rawTempFET = (uint16_t)((data[4] << 8) | data[5]);
                if (rawTempFET != 0xFFFF) {
                    inj.temperature = (int16_t)rawTempFET; // already 0.1 C units
                }

                // currentMotor -> mA
                uint16_t rawCurrent = (uint16_t)((data[6] << 8) | data[7]);
                if (rawCurrent != 0xFFFF) {
                    // rawCurrent units = 10 mA -> mA = rawCurrent * 10
                    inj.current = (uint32_t)rawCurrent * 10u;
                }

                // tempBEC
                uint16_t rawTempBEC = (uint16_t)((data[8] << 8) | data[9]);
                if (rawTempBEC != 0xFFFF) {
                    inj.temperature2 = (int16_t)rawTempBEC;
                }

                // currentBEC (UINT8) units = 100 mA
                uint8_t rawCurBEC = data[10];
                if (rawCurBEC != 0xFF) {
                    inj.bec_current = (uint32_t)rawCurBEC * 100u;
                }

                // voltsBEC (UINT8) units = 0.05 V -> mV = val * 50
                uint8_t rawVoltsBEC = data[11];
                if (rawVoltsBEC != 0xFF) {
                    inj.bec_voltage = (uint32_t)rawVoltsBEC * 50u;
                }

                // powerOut (UINT8) units = 0.5% and throttle (UINT8) units = 0.5%
                // Tie powerOut to throttle: prefer powerOut when present, else use throttle byte, else fallback
                uint8_t rawPowerOut = data[13];
                uint8_t rawThrottle = data[12];
                if (rawPowerOut != 0xFF) {
                    // Use powerOut as the authoritative throttle proxy
                    inj.throttle = (uint16_t)rawPowerOut * 5u; // 0.1% units
                    inj.pwm = inj.throttle;
                } else if (rawThrottle != 0xFF) {
                    inj.throttle = (uint16_t)rawThrottle * 5u;
                    inj.pwm = inj.throttle;
                } else {
                    // Fallback: populate from current throttle command like before
                    const uint16_t channelVal = smartescThrottleCommand;
                    const uint16_t pulseUs = smartescChannelValueToPulseWidth(channelVal);
                    const float normalized = smartescNormalizedFromPulse(pulseUs);
                    const int throttle01 = (int)lrintf(normalized * 1000.0f); // 0.1% units
                    inj.throttle = (throttle01 < 0) ? (uint16_t)(-throttle01) : (uint16_t)throttle01;
                    inj.pwm = inj.throttle;
                }

                // Inject parsed data
                escSensorInject((uint8_t)motorIndex, &inj);
            }
        }
    }
}

static void smartescRecordTelemetryFrame(const Srxl2Header *header)
{
    if (!header || header->length < 6) {
        return;
    }
    const uint8_t *payload = (const uint8_t *)(header + 1);
    const uint8_t sensorId = payload[1];
    const uint8_t secondaryId = payload[2];
    const uint8_t *data = payload + 3;

    const unsigned base = (smartescTelemetryHistoryHead + SMARTESC_TELEM_HISTORY - smartescTelemetryHistoryCount) % SMARTESC_TELEM_HISTORY;
    for (unsigned i = 0; i < smartescTelemetryHistoryCount; ++i) {
        const unsigned slotIndex = (base + i) % SMARTESC_TELEM_HISTORY;
        smartescTelemetryHistoryEntry_t *entry = &smartescTelemetryHistory[slotIndex];
        if (entry->sensorId == sensorId && entry->secondaryId == secondaryId && memcmp(entry->data, data, sizeof(entry->data)) == 0) {
            entry->count++;
            entry->timestamp = micros();
            return;
        }
    }

    smartescTelemetryHistoryEntry_t *slot = &smartescTelemetryHistory[smartescTelemetryHistoryHead];
    slot->timestamp = micros();
    slot->count = 1;
    slot->sensorId = sensorId;
    slot->secondaryId = secondaryId;
    memcpy(slot->data, data, sizeof(slot->data));
    smartescTelemetryHistoryHead = (smartescTelemetryHistoryHead + 1) % SMARTESC_TELEM_HISTORY;
    if (smartescTelemetryHistoryCount < SMARTESC_TELEM_HISTORY) {
        smartescTelemetryHistoryCount++;
    }
}

static void smartescAdvanceHandshakeStage(smartescHandshakeStage_t newStage)
{
    if (handshakeStage == newStage) {
        return;
    }

    handshakeStage = newStage;
    smartescHandshakeComplete = (newStage == SMARTESC_HANDSHAKE_STAGE_DONE);
}

static const char *smartescHandshakeStageToString(smartescHandshakeStage_t stage)
{
    switch (stage) {
    case SMARTESC_HANDSHAKE_STAGE_IDLE: return "idle";
    case SMARTESC_HANDSHAKE_STAGE_WAITING_ESC_REPLY: return "wait_ESC";
    case SMARTESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK: return "wait_FC";
    case SMARTESC_HANDSHAKE_STAGE_DONE: return "done";
    default: return "unknown";
    }
}

bool smartescIsHandshakeComplete(void) { return smartescHandshakeComplete; }
const char *smartescGetHandshakeStageString(void) { return smartescHandshakeStageToString(handshakeStage); }

/* handshake protocol
    1. listen for 50ms for serial activity and go to State::Running if found, autobaud may be necessary
    2. if smartesc_unitId = 0:
            send a Handshake with destinationDeviceId = 0 every 50ms for at least 200ms
        else:
            listen for Handshake for at least 200ms
    3.  respond to Handshake as currently implemented in process if rePst received
    4.  respond to broadcast Handshake
*/

// if 50ms with not activity, go to default baudrate and to step 1

bool smartescProcessHandshake(const Srxl2Header* header)
{
    const Srxl2HandshakeSubHeader* handshake = (const Srxl2HandshakeSubHeader*)(header + 1);

    // smartescRecordHandshakeFrame((const uint8_t *)header, header->length, false);

    const uint8_t fcId = (FlightController << 4) | unitId;

    const bool isBroadcast = (handshake->destinationDeviceId == 0x00);
    const bool isOwnId = (handshake->destinationDeviceId == fcId);
    const bool treatsAsBroadcast = isBroadcast || isOwnId;

    if (treatsAsBroadcast) {
        busMasterDeviceId = handshake->sourceDeviceId;
        if (handshake->baudSupported == 1) {
            serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_HIGH);
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

        smartescWriteData(&response, sizeof(response));
        /* Defer actual transmit to service() to avoid blocking in poll()/frame processing. */
        if (isOwnId) {
            smartescScheduleBroadcastConfirm(fcId);
            smartescAdvanceHandshakeStage(SMARTESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK);
        }

        state = Running;
        smartescBootEndUs = micros() + 2000000;
        return true;
    }

    return true;
}

void smartescProcessChannelData(const Srxl2ChannelDataHeader* channelData, smartesc_rxRuntimeState_t *rxRuntimeState) {
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

    // Fix: channelData_header was an invalid identifier; use channelData for debug output
    DEBUG_PRINTF("channel data: %d %d %x\r\n", channelData->rssi, channelData->frameLosses, channelData->channelMask.u32);

    smartescAdvanceHandshakeStage(SMARTESC_HANDSHAKE_STAGE_WAITING_FINAL_ACK);
}

bool smartescProcessControlData(const Srxl2Header* header, smartesc_rxRuntimeState_t *rxRuntimeState)
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
        smartescProcessChannelData(cdh, rxRuntimeState);
    } break;

    case FailsafeChannelData: {
        globalResult |= RX_FRAME_FAILSAFE;
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
    } break;

    case VTXData: {
    #if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
        smartescVtxData *vtxData = (smartescVtxData*)(controlData + 1);
        uint32_t vtxControl =   (0xE0u << 24) | (0xE0u << 8) |
                                ((vtxData->band & 0x07u) << 21) |
                                ((vtxData->channel & 0x0Fu) << 16) |
                                ((vtxData->pit & 0x01u) << 4) |
                                ((vtxData->region & 0x01u) << 3) |
                                ((vtxData->power & 0x07u));
        spektrumHandleVtxControl(vtxControl);
    #endif
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
    case VTXData: {
        // We don’t know the exact size here; if you want a tail view after VTX, set consumed accordingly.
        // For now, assume no extra tail beyond the VTX payload we already used.
        consumed = payloadLen; // nothing left to consider a "tail"
    } break;
    default:
        consumed = MIN(payloadLen, 2); // at least command+replyId, be conservative
        break;
    }

    if (consumed < 0) consumed = 0;
    if (consumed > payloadLen) consumed = payloadLen;

    return true;
}

bool smartescProcessPacket(const Srxl2Header* header, smartesc_rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    switch (header->packetType) {
    case Handshake:
        return smartescProcessHandshake(header);
    case TelemetrySensorData:
        smartescHandleTelemetryFrame(header);
        return true;
    default:
        DEBUG_PRINTF("Other packet type, ID: %x \r\n", header->packetType);
        break;
    }

    return false;
}

static void smartescTrySendPendingWriteBuffer(void)
{
    if (!serialPort) {
        return;
    }

    if (smartescTxInProgress) {
        const uint32_t now = micros();
        /* On some targets/drivers, isSerialTransmitBufferEmpty() can lag well past
         * the actual on-wire completion time, which stalls subsequent frames.
         * Use a conservative time-based completion as a backstop. */
        const bool timeDone = (smartescTxExpectedDoneUs != 0) && (cmpTimeUs(now, smartescTxExpectedDoneUs) >= 0);
        if (isSerialTransmitBufferEmpty(serialPort) || timeDone || cmpTimeUs(now, smartescTxActiveSinceUs) > smartesc_TX_TIMEOUT_US) {
            serialEndWrite(serialPort);
            smartescTxInProgress = false;
            smartescTxExpectedDoneUs = 0;
        }
        return;
    }

    if (writeBufferIdx == 0) {
        return;
    }

    uint32_t now = micros();
    const bool seenIdleSinceLastRx = cmpTimeUs(lastIdleTimestamp, lastReceiveTimestamp) > 0;
    const uint32_t deadline = lastReceiveTimestamp + smartesc_REPLY_QUIESCENCE;
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

    smartescLastTxSkipSet = writeBufferIdx;  // Debug: record frame size
    /* Set up echo suppression: skip exactly writeBufferIdx bytes that will
     * echo back on this half-duplex bus */
    txBytesToSkip = writeBufferIdx;
    readBufferPtr->len = 0;
    readBufferIdx = 0;


    serialBeginWrite(serialPort);
    /* Record TX start timestamp into the first matching event slot (if any) */
    const uint32_t txStartUs = micros();
    serialWriteBuf(serialPort, writeBuffer, writeBufferIdx);
    smartescTxInProgress = true;
    smartescTxActiveSinceUs = txStartUs;
    /* Predict completion time from baud and bytes (10 bits per byte on 8N1).
     * Add a small margin for ISR/line turnaround. */
    {
        const uint32_t baud = serialGetBaudRate(serialPort);
        const uint32_t bytes = smartescLastTxSkipSet;
        if (baud > 0 && bytes > 0) {
            const uint32_t bits = bytes * 10u;
            const uint32_t durUs = (uint32_t)(((uint64_t)bits * 1000000ull + (uint64_t)baud - 1ull) / (uint64_t)baud);
            smartescTxExpectedDoneUs = txStartUs + durUs + 300u;
        } else {
            smartescTxExpectedDoneUs = txStartUs + 2000u;
        }
    }
    /* Don't touch echo tracking here - the ISR handles it.
     * The ESC reply may already be arriving. */
    
    writeBufferIdx = 0;
}

/* smartescFlushWriteBufferBlocking removed: flushing is now handled from service() */

// @note assumes packet is fully there
void smartescProcess(smartesc_rxRuntimeState_t *rxRuntimeState)
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
    smartescLastProcessedLen = headerLen;
    smartescLastProcessedPacketType = packetType;

    if (smartescProcessPacket(&processBufferPtr->packet.header, rxRuntimeState)) {
        return;
    }

    DEBUG_PRINTF("could not parse packet: %x\r\n", processBufferPtr->packet.header.packetType);
    globalResult = RX_FRAME_DROPPED;
}

void smartescDataReceive(uint16_t character, void *data)
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
                    struct rxBufse *spare = smartescGetSpareBuffer();
                    /* Make current read buffer the process buffer */
                    processBufferPtr = readBufferPtr;
                    /* Select a new read buffer (spare). If no spare found, fall back to the other buffer */
                    if (spare == NULL) {
                        for (int i = 0; i < 3; ++i) {
                            struct rxBufse *b = &readBufferse[i];
                            if (b != processBufferPtr) { spare = b; break; }
                        }
                    }
                    readBufferPtr = spare;
                    processBufferPtr->len = readBufferIdx;                    readBufferPtr->len = 0;
                    readBufferIdx = 0;
                } else {
                    /* Process buffer busy: try to queue this completed frame into a spare buffer */
                    struct rxBufse *spare = smartescGetSpareBuffer();
                    if (spare != NULL) {
                        unsigned copyLen = readBufferIdx;
                        if (copyLen > smartesc_MAX_PACKET_LENGTH) copyLen = smartesc_MAX_PACKET_LENGTH;
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
    if (readBufferIdx >= smartesc_MAX_PACKET_LENGTH) {
        readBufferIdx = 0;
        globalResult = RX_FRAME_DROPPED;
    }
    else {
        readBufferPtr->packet.raw[readBufferIdx] = character;
        readBufferIdx++;
    }
}

void smartescIdle()
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
    if (expectedLen > 0 && expectedLen <= smartesc_MAX_PACKET_LENGTH && readBufferIdx < expectedLen) {
        // Frame incomplete, don't swap yet - wait for remaining bytes
        return;
    }

    // Frame looks complete (or invalid length which we'll catch in processing)
    lastIdleTimestamp = microsISR();

    /* Only swap if process buffer is empty - don't overwrite unprocessed frame! */
    if (processBufferPtr->len > 0) {
        /* Process buffer still has data - try to queue the current read buffer into a spare buffer */
        struct rxBufse *spare = smartescGetSpareBuffer();
        if (spare != NULL) {
            unsigned copyLen = readBufferIdx;
            if (copyLen > smartesc_MAX_PACKET_LENGTH) copyLen = smartesc_MAX_PACKET_LENGTH;
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

static uint8_t smartescFrameStatus(smartesc_rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    globalResult = RX_FRAME_PENDING;

    // len should only be set after an idle interrupt (packet reception complete)
    if (processBufferPtr != NULL && processBufferPtr->len) {
        // About to process packet
        smartescProcess(rxRuntimeState);
        // Finished processing
        processBufferPtr->len = 0;

        /* After processing, check if any queued buffer (spare) has data
         * and immediately process it as well. Avoid touching the active
         * read buffer (readBufferPtr). */
        bool found;
        do {
            found = false;
            for (int i = 0; i < 3; ++i) {
                struct rxBufse *b = &readBufferse[i];
                if (b == readBufferPtr) continue; /* skip active write buffer */
                if (b->len > 0) {
                    processBufferPtr = b;
                    smartescProcess(rxRuntimeState);
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

                if(currentBaud == smartesc_PORT_BAUDRATE_DEFAULT)
                    serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_HIGH);
                else
                    serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_DEFAULT);
            }
        } else if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo if there was activity - detect baudrate and ListenForHandshake

            if (unitId == 0) {
                state = SendHandshake;
                timeoutTimestamp = now + smartesc_SEND_HANDSHAKE_TIMEOUT_US;
                fullTimeoutTimestamp = now + smartesc_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            } else {
                state = ListenForHandshake;
                timeoutTimestamp = now + smartesc_LISTEN_FOR_HANDSHAKE_TIMEOUT_US;
            }
        }
    } 
        // strcpy(smartescStateString, "ListenForActivity");
        break;

    case SendHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0) {
            // @todo set another timeout for 50ms tries
            // fill write buffer with handshake frame
            result |= RX_FRAME_PROCESSING_REQUIRED;
        }

        if (cmpTimeUs(now, fullTimeoutTimestamp) >= 0) {
            serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case SendHandshake: switching to %d baud\r\n", smartesc_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + smartesc_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } 
        // strcpy(smartescStateString, "SendHandshake");
        break;

    case ListenForHandshake: {
        if (cmpTimeUs(now, timeoutTimestamp) >= 0)  {
            serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case ListenForHandshake: switching to %d baud\r\n", smartesc_PORT_BAUDRATE_DEFAULT);
            timeoutTimestamp = now + smartesc_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            lastReceiveTimestamp = 0;
        }
    } 
        // strcpy(smartescStateString, "ListenForHandshake");
        break;

    case Running: {
        // frame timed out, reset state
        if (cmpTimeUs(now, lastValidPacketTimestamp) >= smartesc_FRAME_TIMEOUT_US) {
            serialSetBaudRate(serialPort, smartesc_PORT_BAUDRATE_DEFAULT);
            DEBUG_PRINTF("case Running: switching to %d baud: %d %d\r\n", smartesc_PORT_BAUDRATE_DEFAULT, now, lastValidPacketTimestamp);
            timeoutTimestamp = now + smartesc_LISTEN_FOR_ACTIVITY_TIMEOUT_US;
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
        rxRuntimeState->lastRcFrameTimeUs = lastIdleTimestamp;
    }

    return result;
}

static bool smartescProcessFrame(const smartesc_rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (smartescBroadcastConfirmPending && writeBufferIdx == 0 && !smartescTxInProgress) {
        smartescWriteData(&smartescBroadcastConfirmFrame, sizeof(smartescBroadcastConfirmFrame));
        smartescBroadcastConfirmPending = false;
        smartescAdvanceHandshakeStage(SMARTESC_HANDSHAKE_STAGE_DONE);
    }
    else if (smartescHandshakeComplete && !smartescTxInProgress && writeBufferIdx == 0) {
        uint32_t now = micros();
        /* Keep telemetry state progressing, but do not stall throttle/channel frames
         * for the whole telemetry RX timeout. */
        if (smartescTelemetryRxPending) {
            if (smartescTelemetryRxDeadlineUs == 0 || cmpTimeUs(now, smartescTelemetryRxDeadlineUs) >= 0) {
                smartescTelemetryRxPending = false;

                smartescTelemetryRxDeadlineUs = 0;
            }
        }
        bool telemetryHoldoffActive = false;
        if (smartescTelemetryHoldoffEndUs != 0) {
            if (cmpTimeUs(now, smartescTelemetryHoldoffEndUs) < 0) {
                telemetryHoldoffActive = true;
            } else {
                smartescTelemetryHoldoffEndUs = 0;
            }
        }
        smartescUpdateThrottleFromMotors();
        if (!telemetryHoldoffActive && (smartescNextThrottleSendUs == 0 || cmpTimeUs(now, smartescNextThrottleSendUs) >= 0)) {
            if (smartescQueueChannelDataFrame(smartescThrottleCommand)) {
                smartescNextThrottleSendUs = now + smartescThrottleIntervalUs;
            }
        }
    }

    smartescTrySendPendingWriteBuffer();

    return (writeBufferIdx == 0);
}

static float smartescReadRawRC(const smartesc_rxRuntimeState_t *rxRuntimeState, uint8_t channelIdx)
{
    if (channelIdx >= rxRuntimeState->channelCount) {
        return 0;
    }

    return ((float)(rxRuntimeState->channelData[channelIdx] >> smartesc_CHANNEL_SHIFT) / 16) + SPEKTRUM_PULSE_OFFSET;
}

/* Provide a local frame-time helper so the SMART ESC driver does not
 * depend on the global RX subsystem. This returns the last idle
 * timestamp observed by the driver as an approximation of frame time. */
static timeUs_t smartescLocalFrameTimeUs(void) { return lastIdleTimestamp; }

void smartescWriteData(const void *data, int len)
{
    const uint16_t crc = crc16_ccitt_update(0, (uint8_t*)data, len - 2);
    ((uint8_t*)data)[len-2] = ((uint8_t *) &crc)[1] & 0xFF;
    ((uint8_t*)data)[len-1] = ((uint8_t *) &crc)[0] & 0xFF;

    len = MIN(len, (int)sizeof(writeBuffer));
    memcpy(writeBuffer, data, len);
    writeBufferIdx = len;
}

void smartescSetThrottleCommand(uint16_t value)
{
    smartescThrottleCommand = value;
    smartescNextThrottleSendUs = micros();
}

void smartescSetThrottleRateHz(uint32_t rateHz)
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
    const uint32_t minIntervalUs = smartescComputeMinThrottleIntervalUs();
    uint32_t interval = 1000000 / rateHz;
    if (interval < minIntervalUs) {
        interval = minIntervalUs;
    }
    smartescThrottleIntervalUs = interval;
    smartescNextThrottleSendUs = micros();

    // Persist effective value
    const uint16_t effectiveRate = (uint16_t)(1000000 / smartescThrottleIntervalUs);
    smartescConfigMutable()->throttle_rate_hz = effectiveRate;

}

uint32_t smartescGetThrottleRateHz(void)
{
    if (smartescThrottleIntervalUs == 0) {
        return 0;
    }
    return 1000000 / smartescThrottleIntervalUs;
}

void smartescSetTelemetryIntervalFrames(uint8_t frames)
{
    smartescTelemetryFrameInterval = frames;
    smartescThrottleFrameCounter = 0;

    // Persist
    smartescConfigMutable()->telem_interval_frames = frames;

    /* If telemetry polling is disabled, clear any pending wait/holdoff so we don't
     * introduce a one-time timeout stall from a previous request. */
    if (frames == 0) {
        smartescTelemetryRxPending = false;
        smartescTelemetryRxDeadlineUs = 0;
        smartescTelemetryHoldoffEndUs = 0;
    }
}

uint8_t smartescGetTelemetryIntervalFrames(void)
{
    return smartescTelemetryFrameInterval;
}

unsigned smartescGetTelemetryHistoryCount(void)
{
    return smartescTelemetryHistoryCount;
}

unsigned smartescCopyTelemetryHistory(unsigned idx, uint8_t *dst, unsigned maxLen, uint8_t *outLen, uint32_t *outTimestamp, uint8_t *outSensorId, uint8_t *outSecondaryId, uint32_t *outCount)
{
    if (idx >= smartescTelemetryHistoryCount) {
        return 0;
    }
    const unsigned base = (smartescTelemetryHistoryHead + SMARTESC_TELEM_HISTORY - smartescTelemetryHistoryCount) % SMARTESC_TELEM_HISTORY;
    const unsigned slotIndex = (base + idx) % SMARTESC_TELEM_HISTORY;
    const smartescTelemetryHistoryEntry_t *slot = &smartescTelemetryHistory[slotIndex];
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

bool smartescGetLatestTelemetry(uint8_t *sensorId, uint8_t *secondaryId, uint8_t *dst, unsigned maxLen)
{
    if (!smartescLatestTelemetryValid) {
        return false;
    }

    /* Do not expose sensor type 0x0C for transmission — caller (SRXL2)
     * will skip sending these frames. Keep the data recorded for history
     * and for internal injection into escSensorData_t. */
    if (smartescLatestTelemetrySensorId == 0x0C) {
        return false;
    }

    if (sensorId) {
        *sensorId = smartescLatestTelemetrySensorId;
    }
    if (secondaryId) {
        *secondaryId = smartescLatestTelemetrySecondaryId;
    }
    if (dst && maxLen) {
        memcpy(dst, smartescLatestTelemetryData, MIN(sizeof(smartescLatestTelemetryData), maxLen));
    }

    return true;
}

void validateAndFixSmartescConfig()
{
    // Force half duplex
    escSensorConfigMutable()->halfDuplex = true;

    // Force SRXL2 protocol
    escSensorConfigMutable()->protocol = ESC_SENSOR_PROTO_SRXL2;

    // Clamp/repair persisted SMART ESC rate settings
    smartescConfig_t *cfg = smartescConfigMutable();
    if (cfg->throttle_rate_hz == 0) {
        cfg->throttle_rate_hz = 250;
    }
    // Guard against flooding (min interval 500us => max 2000Hz)
    if (cfg->throttle_rate_hz > 2000) {
        cfg->throttle_rate_hz = 2000;
    }
}

bool smartescInit(const srxl2_escConfig_t *escConfig, smartesc_rxRuntimeState_t *smartescRs)
{
    (void)smartescRs; /* legacy parameter ignored; driver uses internal runtime */

    /* Initialize internal runtime state and channel storage */
    for (unsigned i = 0; i < smartesc_MAX_CHANNELS; i++)
        smartescInternalChannelData[i] = smartesc_CHANNEL_CENTER;

    memset(&smartescInternalRs, 0, sizeof(smartescInternalRs));
    smartescInternalRs.channelData      = smartescInternalChannelData;
    smartescInternalRs.channelCount     = smartesc_MAX_CHANNELS;
    smartescInternalRs.rxRefreshRate    = smartesc_FRAME_PERIOD_US;
    smartescInternalRs.rcReadRawFn      = smartescReadRawRC;
    smartescInternalRs.rcFrameStatusFn  = smartescFrameStatus;
    /* Provide a local frame-time helper so the SMART ESC driver does not
     * depend on the global RX subsystem. This returns the last idle
     * timestamp observed by the driver as an approximation of frame time. */
    smartescInternalRs.rcFrameTimeUsFn  = smartescLocalFrameTimeUs;
    smartescInternalRs.rcProcessFrameFn = smartescProcessFrame;

    /* Point global pointer to internal runtime state */
    smartescRsGlobal = &smartescInternalRs;

    // Reuse already opened port (escSensorInit owns it)
    if (!serialPort) {
        return false;
    }

    serialPort->idleCallback = smartescIdle; // idempotent

    unitId   = escConfig ? escConfig->srxl2_unit_id : 0;
    baudRate = 0;

    state = ListenForActivity;
    timeoutTimestamp = micros() + smartesc_LISTEN_FOR_ACTIVITY_TIMEOUT_US;

    // Initialize SMART ESC throttle rate from motor config when present
#if defined(USE_MOTOR)
    /* Use motor config as authoritative source for throttle refresh. Fall
     * back to persisted SMART ESC value if motor PG is zero/unset. If the
     * SMART ESC driver is active, enforce a maximum of 250Hz. */
    smartesc_lastMotorPwmRate = motorConfig()->dev.motorPwmRate;
    if (smartesc_lastMotorPwmRate == 0) {
        smartesc_lastMotorPwmRate = smartescConfig()->throttle_rate_hz;
    }
    if (smartescIsActive() && smartesc_lastMotorPwmRate > 250) {
        motorConfigMutable()->dev.motorPwmRate = 250;
        smartesc_lastMotorPwmRate = 250;
    }
    smartescSetThrottleRateHz(smartesc_lastMotorPwmRate);
#else
    smartescSetThrottleRateHz(smartescConfig()->throttle_rate_hz);
#endif
    smartescSetTelemetryIntervalFrames(smartescConfig()->telem_interval_frames);

    return true;
}

bool smartescIsActive(void)
{
    return serialPort;
}

bool smartescTelemetryRequested(void)
{
    return telemetryRequested;
}

void smartescInitializeFrame(sbuf_t *dst)
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
    const int frameLen = telemetryFrame[2];   // smartesc length (includes CRC)
    smartescWriteData(telemetryFrame, frameLen);
    telemetryRequested = false;
}

void smartescAttachPort(serialPort_t *p)
{
    serialPort = p;
    if (serialPort) serialPort->idleCallback = smartescIdle;  // set boundary finisher
}

bool smartescDriverInit(void)
{
    if (smartescDriverReady && smartescDriverPort) {
        return true;
    }

    smartescDriverReady = false;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SMART_ESC);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | SERIAL_BIDIR;
    options |= rxConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP;

    smartescDriverPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_SMART_ESC,
        smartescDataReceive,
        NULL,
        SMARTESC_PORT_BAUDRATE_DEFAULT,
        MODE_RXTX,
        options);
    if (!smartescDriverPort) {
        return false;
    }

    smartescAttachPort(smartescDriverPort);

    memset(&smartescDriverRuntime, 0, sizeof(smartescDriverRuntime));
    /* Use a default local config here to avoid referencing escConfig()/rx types.
     * If you want board-specific unit id or pin-swap, set them via a
     * platform-specific setter or call `smartescInit` directly with values. */
    srxl2_escConfig_t localCfg = { .srxl2_unit_id = 0, .pinSwap = false };
    if (!smartescInit(&localCfg, &smartescDriverRuntime)) {
        closeSerialPort(smartescDriverPort);
        smartescDriverPort = NULL;
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
                
                smartescWriteData(&handshake, sizeof(handshake));
                smartescTrySendPendingWriteBuffer();
                nextHandshakeUs = now + 50000; // next attempt in 50ms
            }
            
            /* Process any incoming handshake responses */
            smartesc_poll();
            
            /* If we got a valid handshake response, exit early */
            if (state == Running || busMasterDeviceId != 0xFF) {
                break;
            }
            
            /* Small delay to prevent busy-loop CPU saturation */
            delayMicroseconds(100);
        }
    }

    smartescDriverReady = true;
    return true;
}

void smartescDriverTask(timeUs_t currentTimeUs)
{
    static timeUs_t nextInitAttemptUs = 0;

    if (!smartescDriverReady) {
        const timeUs_t now = currentTimeUs ? currentTimeUs : micros();
        if (nextInitAttemptUs == 0 || cmpTimeUs(now, nextInitAttemptUs) >= 0) {
            nextInitAttemptUs = now + 500000; // retry every 500ms
            if (smartescDriverInit()) {
                return;
            }
        }
        return;
    }

    smartesc_poll();
    smartesc_service();
}

bool smartescDriverIsReady(void)
{
    return smartescDriverReady && smartescIsActive();
}

void smartesc_poll(void)
{
    if (!smartescRsGlobal) {
        return;
    }

    /* update frame status and process any complete frames */
    (void)smartescFrameStatus(smartescRsGlobal);
}

void smartesc_service(void)
{
    if (!smartescRsGlobal) {
        return;
    }

    /* handle any pending transmissions */
    (void)smartescProcessFrame(smartescRsGlobal);

    #if defined(USE_MOTOR)
    /* If motor PWM rate changed externally (CLI/MSP/PG), adopt it as the
     * SMART ESC throttle refresh rate. This makes `dev.motorPwmRate` the
     * authoritative source. When SMART ESC is active, cap the maximum rate
     * to 250Hz. */
    uint16_t current = motorConfig()->dev.motorPwmRate;
    if (smartescIsActive() && current > 250) {
        motorConfigMutable()->dev.motorPwmRate = 250;
        current = 250;
    }
    if (current != smartesc_lastMotorPwmRate && current != 0) {
        smartesc_lastMotorPwmRate = current;
        smartescSetThrottleRateHz((uint32_t)current);
    }
#endif
}

#endif // USE_SMART_ESC
