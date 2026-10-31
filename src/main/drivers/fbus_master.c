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

 //this uses SBUS out and SPORT/FBUS_in

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "pg/fbus_master.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "pg/bus_servo.h"

#include "common/maths.h"
#include "common/time.h"

#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/sbus_output.h"
#include "drivers/fbus_master.h"
#include "drivers/fbus_sensor.h"
#include "drivers/fbus_xact.h"

#include "flight/mixer.h"
#include "flight/servos.h"

#include "fc/runtime_config.h"
#include "build/build_config.h"
#include "rx/frsky_crc.h"
#include "rx/fbus.h"
#include "io/serial.h"
#define FBUS_MASTER_BUFFER_SIZE 64

enum {
    FBUS_FRAME_ID_NULL = 0x00,
    FBUS_FRAME_ID_DATA = 0x10,
    FBUS_FRAME_ID_WORK = 0x20,
    FBUS_FRAME_ID_IDLE = 0x21,
    FBUS_FRAME_ID_READ = 0x30,
    FBUS_FRAME_ID_WRITE = 0x31,
    FBUS_FRAME_ID_RESPONSE = 0x32,
    FBUS_FRAME_ID_OTA_START = 0xF0,
    FBUS_FRAME_ID_OTA_DATA = 0xF1,
    FBUS_FRAME_ID_OTA_STOP = 0xF2
};

serialPort_t *fbusMasterPort = NULL;

#define FC_COMMON_ID 0x1B
typedef enum {
    FBUS_MASTER_SCAN_PHY_ID = 0,
    FBUS_MASTER_QUERY_PHY_ID,
}fbusMasterTelemetryState_e;

typedef enum {
    FBUS_MASTER_TELEMETRY = 0,
    FBUS_MASTER_OTA,
}fbusMasterPayloadState_e;

uint8_t phsIdList[FBUS_MAX_PHYS_ID] = {0};
uint8_t physIdsfound = 0;
uint8_t physIdCnt = 0;
uint8_t currentPhysId = 0;
uint8_t readIngoreBytes = 0;
uint8_t readBytes = 0;
uint8_t buffer[FBUS_MASTER_BUFFER_SIZE] = {0};

static timeUs_t nextTelemetryPollTimeUs = 0;
static timeUs_t sensorDiscoveryEndTimeUs = 0;

fbusMasterPayloadState_e fbusMasterPayloadState = FBUS_MASTER_TELEMETRY;
fbusMasterTelemetryState_e fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;

uint16_t fbusMasterFrameRateHz(void)
{
    const uint16_t rate = fbusMasterConfig()->frameRate;
    return fbusMasterConfig()->baudFast ? rate : MIN(rate, FBUS_MASTER_FRAME_RATE_MAX_SLOW_HZ);
}

static uint16_t fbusMasterTelemetryRateHz(void)
{
    return constrain(fbusMasterConfig()->telemetryRate, FBUS_MASTER_TELEMETRY_RATE_MIN_HZ, FBUS_MASTER_TELEMETRY_RATE_MAX_HZ);
}

static uint16_t fbusMasterDiscoveryTimeMs(void)
{
    return constrain(fbusMasterConfig()->sensorDiscoveryTimeMs, FBUS_MASTER_DISCOVERY_TIME_MIN_MS, FBUS_MASTER_DISCOVERY_TIME_MAX_MS);
}

static timeDelta_t fbusMasterTelemetryPeriodUs(void)
{
    return 1000000 / fbusMasterTelemetryRateHz();
}

static void fbusMasterStartDiscoveryWindow(timeUs_t currentTimeUs)
{
    sensorDiscoveryEndTimeUs = currentTimeUs + ((timeUs_t)fbusMasterDiscoveryTimeMs() * 1000);
}

// Resets phy-ID scanning state and re-arms the discovery window. Exposed so XACT servo
// programming can force a fresh scan (e.g. after a servo's physical ID has been reprogrammed).
void fbusMasterStartDiscovery(void)
{
    physIdsfound = 0;
    physIdCnt = 0;
    currentPhysId = 0;
    fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;
    fbusMasterStartDiscoveryWindow(micros());
}

static uint8_t fbusMasterTakeNextScanPhysId(void)
{
    if (currentPhysId >= FBUS_MAX_PHYS_ID) {
        currentPhysId = 0;
    }

    const uint8_t phyId = currentPhysId;
    currentPhysId++;
    return phyId;
}

static void smartportMasterPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 1) ^ fbusGetBit(*phyIDByte, 2)) << 5;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 3) ^ fbusGetBit(*phyIDByte, 4)) << 6;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 4)) << 7;
}

static int8_t smartportMasterStripPhyIDCheckBits(uint8_t phyID)
{
    uint8_t smartportPhyID = phyID & 0x1F;
    uint8_t phyIDCheck = smartportPhyID;
    smartportMasterPhyIDFillCheckBits(&phyIDCheck);
    return phyID == phyIDCheck ? smartportPhyID : -1;
}

/*
  Requests from the radio for a device on this line, passed on by the receiver:
  Ethos Device config and the radio's device scripts. Reads and writes (0x30/0x31)
  and work/idle (0x20/0x21) go out in the downlink slot; the device's answer (0x32)
  goes back to the radio under the ID the request was sent to.
 */
#define FBUS_RELAY_QUEUE_SIZE           4
#define FBUS_RELAY_NO_DEVICE            0xFF
#define FBUS_RELAY_ANSWER_TIMEOUT_US    500000

typedef struct {
    uint8_t phyId;      // the device on this line
    uint8_t replyId;    // the ID the radio sent the request to
    uint8_t prim;
    uint16_t appId;
    uint32_t data;
} fbusRelayFrame_t;

// requests, from the receiver task
static fbusRelayFrame_t relayRequests[FBUS_RELAY_QUEUE_SIZE];
static uint8_t relayRequestCount = 0;
// after a read or write, a data poll with its App ID collects the answer: a device may
// answer in the request's own slot, or only when polled (XACT servos)
static fbusRelayFrame_t relayPoll;
static bool relayPollPending = false;
// the request whose answer is awaited; shared with the UART RX callback. The answer
// is matched on device, App ID and field, so an XACT response for another field, or
// a late answer to an earlier request, is not relayed as this one's answer.
static volatile uint8_t relayWaitPhyId = FBUS_RELAY_NO_DEVICE;
static volatile uint8_t relayWaitReplyId = 0;
static volatile uint16_t relayWaitAppId = 0;
static volatile uint8_t relayWaitFieldId = 0;
static timeUs_t relayWaitStartUs = 0;
// answers for the radio, written by the UART RX callback
static fbusRelayFrame_t relayAnswers[FBUS_RELAY_QUEUE_SIZE];
static volatile uint8_t relayAnswerCount = 0;

static void relayQueuePush(fbusRelayFrame_t *queue, volatile uint8_t *count, const fbusRelayFrame_t *frame)
{
    if (*count >= FBUS_RELAY_QUEUE_SIZE) {
        // full: drop the oldest
        memmove(&queue[0], &queue[1], sizeof(queue[0]) * (FBUS_RELAY_QUEUE_SIZE - 1));
        (*count)--;
    }
    queue[*count] = *frame;
    (*count)++;
}

static void relayQueuePop(fbusRelayFrame_t *queue, volatile uint8_t *count, uint8_t index)
{
    for (uint8_t i = index; i + 1 < *count; i++) {
        queue[i] = queue[i + 1];
    }
    (*count)--;
}

bool fbusMasterRelayRequest(uint8_t replyId, uint8_t prim, uint16_t appId, uint32_t data)
{
    if (!fbusMasterPort) {
        return false;
    }

    // sent to the device's own ID, or to another ID (Ethos Device config uses 0x1B)
    // for the device that sends this App ID
    int8_t phyId = -1;
    if (replyId != FC_COMMON_ID && fbusSensorIsObservedOnFbus(replyId)) {
        phyId = replyId;
    } else {
        phyId = fbusSensorFindFbusPhysicalId(appId);
    }
    if (phyId < 0) {
        return false;
    }

    // the radio repeats a request until it is answered: keep one copy
    for (uint8_t i = 0; i < relayRequestCount; i++) {
        const fbusRelayFrame_t *r = &relayRequests[i];
        if (r->phyId == phyId && r->replyId == replyId && r->prim == prim && r->appId == appId && r->data == data) {
            return true;
        }
    }

    const fbusRelayFrame_t request = {
        .phyId = phyId, .replyId = replyId, .prim = prim, .appId = appId, .data = data,
    };
    relayQueuePush(relayRequests, &relayRequestCount, &request);
    return true;
}

bool fbusMasterRelayGetAnswer(uint8_t replyId, uint8_t *prim, uint16_t *appId, uint32_t *data)
{
    bool found = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        for (uint8_t i = 0; i < relayAnswerCount; i++) {
            if (relayAnswers[i].replyId == replyId) {
                *prim = relayAnswers[i].prim;
                *appId = relayAnswers[i].appId;
                *data = relayAnswers[i].data;
                relayQueuePop(relayAnswers, &relayAnswerCount, i);
                found = true;
                break;
            }
        }
    }

    return found;
}

// fills the downlink with the next relayed frame; false if there is none
static bool relayPrepareDownlink(fbusMasterDownlink_t *downlink, timeUs_t currentTimeUs)
{
    // no settings changes in flight, as for XACT programming
    if (ARMING_FLAG(ARMED)) {
        relayRequestCount = 0;
        relayPollPending = false;
        return false;
    }

    if (relayWaitPhyId != FBUS_RELAY_NO_DEVICE &&
        cmpTimeUs(currentTimeUs, relayWaitStartUs) > FBUS_RELAY_ANSWER_TIMEOUT_US) {
        relayWaitPhyId = FBUS_RELAY_NO_DEVICE;
    }

    fbusRelayFrame_t frame;
    if (relayPollPending) {
        frame = relayPoll;
        relayPollPending = false;
    } else if (relayRequestCount > 0 && relayWaitPhyId == FBUS_RELAY_NO_DEVICE) {
        // the next request waits until the pending answer arrives or times out, so
        // the single wait slot is never overwritten
        frame = relayRequests[0];
        relayQueuePop(relayRequests, &relayRequestCount, 0);
        if (frame.prim == FBUS_FRAME_ID_READ || frame.prim == FBUS_FRAME_ID_WRITE) {
            ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                relayWaitReplyId = frame.replyId;
                relayWaitAppId = frame.appId;
                relayWaitFieldId = frame.data & 0xFF;   // field ID is the first data byte
                relayWaitPhyId = frame.phyId;
            }
            relayWaitStartUs = currentTimeUs;
            relayPoll = frame;
            relayPoll.prim = FBUS_FRAME_ID_DATA;
            relayPoll.data = 0;
            relayPollPending = true;
        }
    } else {
        return false;
    }

    downlink->length = FBUS_DOWNLINK_PAYLOAD_SIZE;
    downlink->phyID = frame.phyId;
    downlink->prim = frame.prim;
    downlink->appId = frame.appId;
    downlink->data[0] = frame.data & 0xFF;
    downlink->data[1] = (frame.data >> 8) & 0xFF;
    downlink->data[2] = (frame.data >> 16) & 0xFF;
    downlink->data[3] = (frame.data >> 24) & 0xFF;
    return true;
}

static void fbusMasterPrepareFrame(fbusMasterFrame_t *frame, uint16_t *channels, timeUs_t currentTimeUs)
{
    // Clear the control.c16 structure
    memset(&frame->c16, 0, sizeof(fbusMasterFrame_t));

    frame->c16.length = FBUS_CONTROL16_LENGTH;
    frame->c16.type = FBUS_CONTROL16_TYPE;

    frame->c16.channels.chan0 = channels[0];
    frame->c16.channels.chan1 = channels[1];
    frame->c16.channels.chan2 = channels[2];
    frame->c16.channels.chan3 = channels[3];
    frame->c16.channels.chan4 = channels[4];
    frame->c16.channels.chan5 = channels[5];
    frame->c16.channels.chan6 = channels[6];
    frame->c16.channels.chan7 = channels[7];
    frame->c16.channels.chan8 = channels[8];
    frame->c16.channels.chan9 = channels[9];
    frame->c16.channels.chan10 = channels[10];
    frame->c16.channels.chan11 = channels[11];
    frame->c16.channels.chan12 = channels[12];
    frame->c16.channels.chan13 = channels[13];
    frame->c16.channels.chan14 = channels[14];
    frame->c16.channels.chan15 = channels[15];

    frame->c16.channels.flags = channels[16] ? BIT(0) : 0;
    frame->c16.channels.flags |= channels[17] ? BIT(1) : 0;

    // Set RSSI (example value)
    frame->c16.rssi = 100; //ToDo

    uint8_t crc = frskyCheckSum((uint8_t *)&(frame->c16.type), FBUS_MASTER_CONTROL_FRAME_PAYLOAD_SIZE);
    frame->c16.crc = crc;

    switch (fbusMasterPayloadState) {
        case FBUS_MASTER_TELEMETRY:
            memset(&frame->downlink, 0, sizeof(fbusMasterDownlink_t));
            frame->downlink.length = FBUS_DOWNLINK_PAYLOAD_SIZE;

            // XACT servo programming (read/write) takes priority over telemetry polling and
            // is not subject to the telemetry rate throttle below, so reads/writes complete quickly.
            if (fbusXactIsBusy() && fbusXactProcessQueue(&frame->downlink)) {
                smartportMasterPhyIDFillCheckBits(&frame->downlink.phyID);
                crc = frskyCheckSum((uint8_t *)&frame->downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
                frame->downlink.crc = crc;
                break;
            }

            // Then the radio's requests for devices on the line, also unthrottled
            if (relayPrepareDownlink(&frame->downlink, currentTimeUs)) {
                smartportMasterPhyIDFillCheckBits(&frame->downlink.phyID);
                frame->downlink.crc = frskyCheckSum((uint8_t *)&frame->downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
                break;
            }

            if (cmpTimeUs(currentTimeUs, nextTelemetryPollTimeUs) < 0) {
                frame->downlink.phyID = 0;
                frame->downlink.prim = FBUS_FRAME_ID_NULL;
                crc = frskyCheckSum((uint8_t *)&frame->downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
                frame->downlink.crc = crc;
                break;
            }

            nextTelemetryPollTimeUs = currentTimeUs + fbusMasterTelemetryPeriodUs();

            if (fbusMasterTelemetryState == FBUS_MASTER_SCAN_PHY_ID && cmpTimeUs(currentTimeUs, sensorDiscoveryEndTimeUs) >= 0) {
                fbusMasterTelemetryState = FBUS_MASTER_QUERY_PHY_ID;
                physIdCnt = 0;
            }
            
            switch (fbusMasterTelemetryState) {
                case FBUS_MASTER_SCAN_PHY_ID:
                    frame->downlink.phyID = fbusMasterTakeNextScanPhysId();
                    frame->downlink.prim = FBUS_FRAME_ID_DATA;
                    break;
                case FBUS_MASTER_QUERY_PHY_ID:
                    if (physIdsfound == 0) {
                        fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;
                        currentPhysId = 0;
                        fbusMasterStartDiscoveryWindow(currentTimeUs);
                        frame->downlink.phyID = 0;
                        frame->downlink.prim = FBUS_FRAME_ID_NULL;
                        break;
                    }

                    if (physIdCnt >= physIdsfound) {
                        physIdCnt = 0;
                    }

                    currentPhysId = phsIdList[physIdCnt];
                    frame->downlink.phyID = currentPhysId;
                    frame->downlink.prim = FBUS_FRAME_ID_DATA;
                    physIdCnt++;
                    break;
                
                default:
                    break;
            }
    
            smartportMasterPhyIDFillCheckBits(&frame->downlink.phyID);
            crc = frskyCheckSum((uint8_t *)&frame->downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
            frame->downlink.crc = crc;
    
            break;
    
        case FBUS_MASTER_OTA:
            //ToDo
            break;
        
        default:
            break;
    }

}

static void processDownlinkFrame(uint8_t *data)
{
    fbusMasterDownlink_t downlink;
    memcpy(&downlink, data, sizeof(downlink));
    uint8_t chkSum = frskyCheckSum((uint8_t *)&downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
    if (chkSum == downlink.crc) {
        const int8_t decodedPhyId = smartportMasterStripPhyIDCheckBits(downlink.phyID);
        if (decodedPhyId < 0) {
            return;
        }

        downlink.phyID = decodedPhyId;
        if (fbusMasterTelemetryState == FBUS_MASTER_SCAN_PHY_ID) {
            bool alreadyInList = false;
            for (uint8_t i = 0; i < physIdsfound; i++) {
                if (phsIdList[i] == downlink.phyID) {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList && physIdsfound < ARRAYLEN(phsIdList)) {
                phsIdList[physIdsfound++] = downlink.phyID;
            }
        }
        // Process sensor data for observation tracking and forwarding
        // Only process if it's a data frame (not null/poll frames)
        if (downlink.prim == FBUS_FRAME_ID_DATA && downlink.phyID != FC_COMMON_ID) {
            // Convert 4-byte array to uint32_t (little-endian)
            uint32_t sensorData = downlink.data[0] | (downlink.data[1] << 8) |
                                  (downlink.data[2] << 16) | (downlink.data[3] << 24);
            fbusSensorProcessData(downlink.phyID, downlink.appId, sensorData);
        }
        // Response frame (XACT servo programming and relayed radio requests):
        // appId = App ID (16-bit), data[0] = FIELDID, data[1..2] = DATA1 (little-endian)
        else if (downlink.prim == FBUS_FRAME_ID_RESPONSE) {
            uint16_t appId = downlink.appId;
            uint8_t fieldId = downlink.data[0];
            uint16_t dataValue = (uint16_t)downlink.data[1] | ((uint16_t)downlink.data[2] << 8);

            if (downlink.phyID == relayWaitPhyId && appId == relayWaitAppId && fieldId == relayWaitFieldId) {
                // the answer to a request from the radio: back to the ID it was sent to
                const fbusRelayFrame_t answer = {
                    .phyId = downlink.phyID,
                    .replyId = relayWaitReplyId,
                    .prim = downlink.prim,
                    .appId = appId,
                    .data = downlink.data[0] | (downlink.data[1] << 8) |
                            (downlink.data[2] << 16) | ((uint32_t)downlink.data[3] << 24),
                };
                relayQueuePush(relayAnswers, &relayAnswerCount, &answer);
                relayWaitPhyId = FBUS_RELAY_NO_DEVICE;
            }

            // Also an XACT answer when a parameter read is in progress for this servo
            // and field; a no-op otherwise, so a relayed answer is never lost to XACT
            // nor an XACT answer to the relay.
            fbusXactSetServoParam(downlink.phyID, fieldId, appId, dataValue);
            fbusXactNotifyResponse(downlink.phyID, fieldId);
        }
    }
}

static FAST_CODE void dataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    // don't listen to self
    if (readIngoreBytes > 0) {
        readIngoreBytes--;
        return;
    }

    buffer[readBytes++] = c;

    if (readBytes >= FBUS_MASTER_BUFFER_SIZE) {
        //sync error
        readBytes = 0;
    } else {
        if (readBytes >= 10 && buffer[0] == FBUS_DOWNLINK_PAYLOAD_SIZE) {
            //process frame, reset readBytes
            processDownlinkFrame(buffer);
            readBytes = 0;
        }
    }
}

// Speed-limit state for F.Bus output, separate from SBUS output's so each
// output steps only on its own frames.
static sbusOutSpeedState_t fbusMasterSpeedState;

static float fbusMasterGetChannelValue(uint8_t channel)
{
    const busServoSourceType_e source_type = busServoConfig()->sourceType[channel];
    switch (source_type) {
        case BUS_SERVO_SOURCE_RX:
            return sbusOutGetRX(channel);
        case BUS_SERVO_SOURCE_MIXER:
            // Use the same servo-parameter-aware function
            return sbusOutGetValueMixer(channel, &fbusMasterSpeedState);
    }
    return 0;
}

static uint16_t fbusMasterConvertToSbus(float value)
{
    // For analog channels (0-15), convert microseconds to SBUS range (192-1792)
    // Bus servo range: (1000 -> BUS_SERVO_MIN_SIGNAL) to (2000 -> BUS_SERVO_MAX_SIGNAL) -> SBUS 192-1792
    const float scaledValue = scaleRangef(value, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL, 192, 1792);
    return constrain(nearbyintf(scaledValue), FBUS_MIN, FBUS_MAX);
}

void fbusMasterUpdate(timeUs_t currentTimeUs)
{
    if (!fbusMasterPort)
        return;

    // Keep derived FBUS sensor states (timeouts/GPS mirrors) updated.
    fbusSensorUpdate(currentTimeUs);

    // Check TX Buff is free
    if (serialTxBytesFree(fbusMasterPort) <= sizeof(fbusMasterFrame_t)) {
        return;
    }

    sbusOutBeginFrame(&fbusMasterSpeedState, currentTimeUs, fbusMasterFrameRateHz(), FBUS_MASTER_CHANNELS);

    // Start sending.
    fbusMasterFrame_t frame;
    uint16_t channels[FBUS_MASTER_CHANNELS];
    for (int ch = 0; ch < FBUS_MASTER_CHANNELS; ch++) {
        float value = fbusMasterGetChannelValue(ch);
        channels[ch] = fbusMasterConvertToSbus(value);
        
        // Store the output value for getServoOutput() to retrieve
        setBusServoOutput(ch, value);
    }
    fbusMasterPrepareFrame(&frame, channels, currentTimeUs);

    // serial output
    serialWriteBuf(fbusMasterPort, (const uint8_t *)&frame, sizeof(frame));
    readIngoreBytes = sizeof(frame);
    readBytes = 0;
}

bool fbusMasterIsEnabled(void) 
{ 
    return fbusMasterPort != NULL;
}

void fbusMasterInit(void)
{
    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_FBUS_MASTER);

    if (!portConfig) {
        fbusMasterPort = NULL;
        return;
    }

    nextTelemetryPollTimeUs = 0;
    fbusMasterStartDiscovery();

    // Initialize XACT servo programming module
    fbusXactInit();

    serialReceiveCallbackPtr callback = dataReceive;
    fbusMasterPort = openSerialPort(
        portConfig->identifier, FUNCTION_FBUS_MASTER, callback, NULL,
        fbusMasterConfig()->baudFast ? FBUS_MASTER_BAUD_FAST : FBUS_MASTER_BAUD_SLOW, MODE_RXTX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (fbusMasterConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_BIDIR |
            (fbusMasterConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}
