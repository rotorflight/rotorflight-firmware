/*
 * This file is part of Rotorflight.
 *
 * Rotorflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Rotorflight is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_FBUS

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#ifdef MSP_FIRMWARE_UPDATE__WARNING_DEAD_CODE__
#include "fc/firmware_update.h"
#endif

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#endif

#include "rx/frsky_crc.h"
#include "rx/rx.h"
#include "rx/sbus_channels.h"
#include "rx/fbus.h"

#define FBUS_MIN_TELEMETRY_RESPONSE_DELAY_US 500
#define FBUS_MAX_TELEMETRY_RESPONSE_DELAY_US 3000
#define FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT 200
#define FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT 50
#define FBUS_MAX_TELEMETRY_AGE_MS 500
#define FBUS_FC_COMMON_ID 0x1B
#define FBUS_FC_MSP_ID 0x0D
#define FPORT2_BAUDRATE 115200
#define FBUS_BAUDRATE 460800
#define FBUS_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define FBUS_RX_TIMEOUT 120 // µs
#define FBUS_CONTROL_FRAME_LENGTH_8CH 13
#define FBUS_CONTROL_FRAME_LENGTH_16CH 24
#define FBUS_CONTROL_FRAME_LENGTH_24CH 35
#define FBUS_OTA_DATA_FRAME_LENGTH 32
#define FBUS_DOWNLINK_FRAME_LENGTH 8
#define FBUS_UPLINK_FRAME_LENGTH 8
#define FBUS_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES 2
#define FBUS_OTA_DATA_FRAME_BYTES 32

#define MS2US(ms)   ((ms) * 1000)

enum {
    DEBUG_FBUS_FRAME_INTERVAL = 0,
    DEBUG_FBUS_FRAME_ERRORS,
    DEBUG_FBUS_FRAME_LAST_ERROR,
    DEBUG_FBUS_TELEMETRY_INTERVAL,
    DEBUG_FBUS_MAX_BUFFER_USAGE,
    DEBUG_FBUS_OTA_FRAME_RESPONSE_TIME,
    DEBUG_FBUS_OTA_RECEIVED_BYTES,
};

enum {
    DEBUG_FBUS_NO_ERROR = 0,
    DEBUG_FBUS_ERROR_TIMEOUT,
    DEBUG_FBUS_ERROR_OVERSIZE,
    DEBUG_FBUS_ERROR_SIZE,
    DEBUG_FBUS_ERROR_CHECKSUM,
    DEBUG_FBUS_ERROR_PHYID_CRC,
    DEBUG_FBUS_ERROR_TYPE,
    DEBUG_FBUS_ERROR_TYPE_SIZE,
    DEBUG_FBUS_ERROR_OTA_BAD_ADDRESS,
};

typedef enum {
    CFT_RC = 0xFF,
    CFT_OTA_START = 0xF0,
    CFT_OTA_DATA = 0xF1,
    CFT_OTA_STOP = 0xF2
} fbus_control_frame_type_e;

typedef enum {
    FT_CONTROL,
    FT_DOWNLINK
} frame_type_e;

typedef enum {
    FS_CONTROL_FRAME_START,
    FS_CONTROL_FRAME_TYPE,
    FS_CONTROL_FRAME_DATA,
    FS_DOWNLINK_FRAME_START,
    FS_DOWNLINK_FRAME_DATA
} frame_state_e;

enum {
    FBUS_FRAME_ID_NULL = 0x00,
    FBUS_FRAME_ID_DATA = 0x10,
    FBUS_FRAME_ID_READ = 0x30,
    FBUS_FRAME_ID_WRITE = 0x31,
    FBUS_FRAME_ID_RESPONSE = 0x32,
    FBUS_FRAME_ID_OTA_START = 0xF0,
    FBUS_FRAME_ID_OTA_DATA = 0xF1,
    FBUS_FRAME_ID_OTA_STOP = 0xF2
};

typedef struct {
    union {
        sbusChannels8ch_t channels_8ch;
        sbusChannels_t channels_16ch;
        sbusChannels24ch_t channels_24ch;
    };
    uint8_t rssi;
} rcData_t;

typedef struct {
    uint8_t phyID;
    /*uint8_t phyID : 5;*/
    /*uint8_t phyXOR : 3;*/
    smartPortPayload_t telemetryData;
} fbusDownlinkData_t;

typedef struct {
    uint8_t type;
    union {
        rcData_t rc;
        uint8_t ota[FBUS_OTA_DATA_FRAME_BYTES];
    };
} fbusControlFrame_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    union {
        fbusControlFrame_t control;
        fbusDownlinkData_t downlink;
    };
} fbusFrame_t;

// RX frames ring buffer
#define NUM_RX_BUFFERS 15

typedef struct fbusBuffer_s {
    uint8_t data[sizeof(fbusFrame_t)+1]; // +1 for CRC
    uint8_t length;
} fbusBuffer_t;

typedef struct {
    uint32_t size;
    uint8_t  crc;
} firmwareUpdateHeader_t;

static volatile fbusBuffer_t rxBuffer[NUM_RX_BUFFERS];
static volatile uint8_t rxBufferWriteIndex = 0;
static volatile uint8_t rxBufferReadIndex = 0;

static serialPort_t *fbusPort;

#ifdef USE_TELEMETRY_SMARTPORT
static smartPortPayload_t *mspPayload = NULL;
static bool telemetryEnabled = false;
static volatile timeUs_t lastTelemetryFrameReceivedUs;
static volatile bool clearToSend = false;
static volatile bool sendNullFrame = false;
static uint8_t downlinkPhyID;
static const smartPortPayload_t emptySmartPortFrame = { .frameId = 0, .valueId = 0, .data = 0 };
static smartPortPayload_t *otaResponsePayload = NULL;
static bool otaMode = false;
static bool otaDataNeedsProcessing = false;
static uint16_t otaMinResponseDelay = FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT;
static uint16_t otaMaxResponseTime = FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT;
static uint32_t otaDataAddress;
static uint8_t otaDataBuffer[FBUS_OTA_DATA_FRAME_BYTES];
static timeUs_t otaFrameEndTimestamp = 0;
static bool firmwareUpdateError = false;

#ifdef MSP_FIRMWARE_UPDATE__WARNING_DEAD_CODE__
static uint8_t firmwareUpdateCRC;
static timeUs_t readyToUpdateFirmwareTimestamp = 0;
#endif
#endif

static volatile uint16_t frameErrors = 0;

static void reportFrameError(uint8_t errorReason)
{
    UNUSED(errorReason);
    frameErrors++;
}

static void clearWriteBuffer(void)
{
    rxBuffer[rxBufferWriteIndex].length = 0;
}

static bool nextWriteBuffer(void)
{
    const uint8_t nextWriteIndex = (rxBufferWriteIndex + 1) % NUM_RX_BUFFERS;

    if (nextWriteIndex != rxBufferReadIndex) {
        rxBufferWriteIndex = nextWriteIndex;
        clearWriteBuffer();
        return true;
    } else {
        clearWriteBuffer();
        return false;
    }
}

static uint8_t writeBuffer(uint8_t byte)
{
    volatile uint8_t * const buffer = rxBuffer[rxBufferWriteIndex].data;
    volatile uint8_t * const buflen = &rxBuffer[rxBufferWriteIndex].length;

    buffer[*buflen] = byte;
    *buflen += 1;

    return *buflen;
}

// UART RX ISR
static void fbusDataReceive(uint16_t byte, void *callback_data)
{
    UNUSED(callback_data);

    static volatile frame_state_e state = FS_CONTROL_FRAME_START;
    static volatile timeUs_t lastRxByteTimestamp = 0;
    static unsigned controlFrameSize;

    const timeUs_t currentTimeUs = micros();
    const timeUs_t timeSincePreviousRxByte = lastRxByteTimestamp ? currentTimeUs - lastRxByteTimestamp : 0;

    lastRxByteTimestamp = currentTimeUs;
#if defined(USE_TELEMETRY_SMARTPORT)
    clearToSend = false;
#endif

    if ((state != FS_CONTROL_FRAME_START) && (timeSincePreviousRxByte > FBUS_RX_TIMEOUT)) {
        state = FS_CONTROL_FRAME_START;
    }

    switch (state) {
        case FS_CONTROL_FRAME_START:
            if ((byte == FBUS_CONTROL_FRAME_LENGTH_8CH) ||
                (byte == FBUS_CONTROL_FRAME_LENGTH_16CH) ||
                (byte == FBUS_CONTROL_FRAME_LENGTH_24CH) ||
                (byte == FBUS_OTA_DATA_FRAME_LENGTH)) {
                clearWriteBuffer();
                writeBuffer(FT_CONTROL);
                writeBuffer(byte);
                controlFrameSize = byte + 4;
                state = FS_CONTROL_FRAME_TYPE;
            }
            break;

        case FS_CONTROL_FRAME_TYPE:
            if ((byte == CFT_RC) || ((byte >= CFT_OTA_START) && (byte <= CFT_OTA_STOP))) {
                unsigned controlFrameType = byte;
                writeBuffer(controlFrameType);
                state = FS_CONTROL_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_CONTROL_FRAME_DATA:
            if (writeBuffer(byte) >= controlFrameSize) {
                nextWriteBuffer();
                state = FS_DOWNLINK_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_START:
            if (byte == FBUS_DOWNLINK_FRAME_LENGTH) {
                writeBuffer(FT_DOWNLINK);
                writeBuffer(byte);
                state = FS_DOWNLINK_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_DATA:
            if (writeBuffer(byte) >= (FBUS_DOWNLINK_FRAME_LENGTH + 3)) {
#if defined(USE_TELEMETRY_SMARTPORT)
                fbusFrame_t *frame = (fbusFrame_t *)rxBuffer[rxBufferWriteIndex].data;
                if ((frame->control.type >= FBUS_FRAME_ID_OTA_START) && (frame->control.type <= FBUS_FRAME_ID_OTA_STOP)) {
                    otaFrameEndTimestamp = currentTimeUs;
                }
                lastTelemetryFrameReceivedUs = currentTimeUs;
#endif
                nextWriteBuffer();
                state = FS_CONTROL_FRAME_START;
            }
            break;

        default:
            state = FS_CONTROL_FRAME_START;
            break;
    }
}

#if defined(USE_TELEMETRY_SMARTPORT)
static void writeUplinkFramePhyID(uint8_t phyID, const smartPortPayload_t *payload)
{
    serialWrite(fbusPort, FBUS_UPLINK_FRAME_LENGTH);
    serialWrite(fbusPort, phyID);

    uint16_t checksum = 0;
    frskyCheckSumStep(&checksum, phyID);

    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); ++i, ++data) {
        serialWrite(fbusPort, *data);
        frskyCheckSumStep(&checksum, *data);
    }

    frskyCheckSumFini(&checksum);
    serialWrite(fbusPort, checksum);
}

static void writeUplinkFrame(const smartPortPayload_t *payload)
{
    writeUplinkFramePhyID(FBUS_FC_COMMON_ID, payload);
}
#endif

static uint8_t frameStatus(rxRuntimeState_t *rxRuntimeConfig)
{
#ifdef USE_TELEMETRY_SMARTPORT
    static smartPortPayload_t payloadBuffer;
    static bool hasTelemetryRequest = false;
    static uint32_t otaPrevDataAddress;
    static bool otaJustStarted = false;
    static bool otaGotData = false;
#endif
    static timeUs_t frameReceivedTimestamp = 0;

    uint8_t result = RX_FRAME_PENDING;
    timeUs_t currentTimeUs = micros();

    if (rxBufferReadIndex != rxBufferWriteIndex) {
        volatile uint8_t *buffer = rxBuffer[rxBufferReadIndex].data;
        uint8_t buflen = rxBuffer[rxBufferReadIndex].length;
        fbusFrame_t *frame = (fbusFrame_t *)buffer;

        if (!frskyCheckSumIsGood((uint8_t *)buffer + 2, buflen - 2)) {  // Skip buffer type and len.
            reportFrameError(DEBUG_FBUS_ERROR_CHECKSUM);
        } else {
            switch (frame->type) {
                case FT_CONTROL:
                    switch (frame->control.type) {
                        case CFT_RC:
                            switch (frame->length) {
                                case FBUS_CONTROL_FRAME_LENGTH_8CH:
                                    result = sbusChannelsDecode8ch(rxRuntimeConfig, &frame->control.rc.channels_8ch);
                                    break;
                                case FBUS_CONTROL_FRAME_LENGTH_16CH:
                                    result = sbusChannelsDecode(rxRuntimeConfig, &frame->control.rc.channels_16ch);
                                    break;
                                case FBUS_CONTROL_FRAME_LENGTH_24CH:
                                    result = sbusChannelsDecode24ch(rxRuntimeConfig, &frame->control.rc.channels_24ch);
                                    break;
                                default:
                                    result = RX_FRAME_DROPPED;
                            }
                            setRssi(scaleRange(frame->control.rc.rssi, 0, 100, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL);
                            frameReceivedTimestamp = currentTimeUs;
#if defined(USE_TELEMETRY_SMARTPORT)
                            otaMode = false;
#endif
                            break;

#if defined(USE_TELEMETRY_SMARTPORT)
                        case CFT_OTA_START: {
                            uint8_t otaMinResponseDelayByte = frame->control.ota[0];
                            if ((otaMinResponseDelayByte > 0) && (otaMinResponseDelayByte <= 4)) {
                                otaMinResponseDelay = otaMinResponseDelayByte * 100;
                            } else {
                                otaMinResponseDelay = FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT;
                            }

                            uint8_t otaMaxResponseTimeByte = frame->control.ota[1];
                            if (otaMaxResponseTimeByte > 0) {
                                otaMaxResponseTime = otaMaxResponseTimeByte * 100;
                            } else {
                                otaMaxResponseTime = FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT;
                            }

                            otaMode = true;
                            break;
                        }

                        case CFT_OTA_DATA:
                            if (otaMode) {
                                memcpy(otaDataBuffer, frame->control.ota, sizeof(otaDataBuffer));
                                otaGotData = true;
                            } else {
                                reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            }
                            break;

                        case CFT_OTA_STOP:
                            if (!otaMode) {
                                reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            }
                            break;
#endif
                        default:
                            reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            break;
                    }
                    break;

                case FT_DOWNLINK:
#if defined(USE_TELEMETRY_SMARTPORT)
                    if (!telemetryEnabled) {
                        break;
                    }

                    downlinkPhyID = frame->downlink.phyID;
                    uint8_t frameId = frame->downlink.telemetryData.frameId;

                    switch (frameId) {
                        case FBUS_FRAME_ID_NULL:
                            hasTelemetryRequest = true;
                            sendNullFrame = true;
                            break;

                        case FBUS_FRAME_ID_DATA:
                            hasTelemetryRequest = true;
                            break;

                        case FBUS_FRAME_ID_OTA_START:
                        case FBUS_FRAME_ID_OTA_DATA:
                        case FBUS_FRAME_ID_OTA_STOP:
                            switch (frameId) {
                                case FBUS_FRAME_ID_OTA_START:
                                    if (otaMode) {
                                        otaJustStarted = true;
                                        otaPrevDataAddress = 0;
                                        hasTelemetryRequest = true;
                                        otaDataNeedsProcessing = false;
                                        firmwareUpdateError = false;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;

                                case FBUS_FRAME_ID_OTA_DATA:
                                    if (otaMode) {
                                        otaDataAddress = frame->downlink.telemetryData.data;
                                        if (otaGotData && (otaDataAddress == (otaJustStarted ? 0 : otaPrevDataAddress + FBUS_OTA_DATA_FRAME_BYTES))) { // check that we got a control frame with data and check address
                                            otaPrevDataAddress = otaDataAddress;
                                            otaGotData = false;
                                            otaDataNeedsProcessing = true;
                                        }
                                        hasTelemetryRequest = true;
                                        otaJustStarted = false;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;

                                case FBUS_FRAME_ID_OTA_STOP:
                                    if (otaMode) {
                                        hasTelemetryRequest = true;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;
                            }
                            if (hasTelemetryRequest) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                otaResponsePayload = &payloadBuffer;
                            }
                            break;

                        default:
                            if ((frameId == FBUS_FRAME_ID_READ) && (downlinkPhyID == FBUS_FC_MSP_ID)) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                mspPayload = &payloadBuffer;
                                hasTelemetryRequest = true;
                            } else if (downlinkPhyID != FBUS_FC_COMMON_ID) {
#if defined(USE_SMARTPORT_MASTER__WARNING_DEAD_CODE__)
                                int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);
                                if (smartportPhyID != -1) {
                                    smartportMasterForward(smartportPhyID, &frame->downlink.telemetryData);
                                    hasTelemetryRequest = true;
                                }
#endif
                            }
                            break;

                    }
#endif
                    break;

                default:
                    reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                    break;

            }
        }

        rxBufferReadIndex = (rxBufferReadIndex + 1) % NUM_RX_BUFFERS;
    }

#if defined(USE_TELEMETRY_SMARTPORT)
    if (((mspPayload || hasTelemetryRequest) && cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) >= FBUS_MIN_TELEMETRY_RESPONSE_DELAY_US)
           || (otaResponsePayload && cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) >= otaMinResponseDelay)) {
        hasTelemetryRequest = false;
        clearToSend = true;
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }
#endif

    if (frameReceivedTimestamp && (cmpTimeUs(currentTimeUs, frameReceivedTimestamp) > MS2US(FBUS_MAX_TELEMETRY_AGE_MS))) {
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
        frameReceivedTimestamp = 0;
    }

#ifdef MSP_FIRMWARE_UPDATE__WARNING_DEAD_CODE__
    if (readyToUpdateFirmwareTimestamp && (cmpTimeUs(currentTimeUs, readyToUpdateFirmwareTimestamp) > 2000000)) {
        readyToUpdateFirmwareTimestamp = 0;
        firmwareUpdateExec(firmwareUpdateCRC);
    }
#endif

    return result;
}

static bool processFrame(const rxRuntimeState_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

#if defined(USE_TELEMETRY_SMARTPORT)
    timeUs_t currentTimeUs = micros();

    if (cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) > FBUS_MAX_TELEMETRY_RESPONSE_DELAY_US) {
       clearToSend = false;
    }

    if (clearToSend) {
        if (otaResponsePayload) {
            switch (otaResponsePayload->frameId) {
                case FBUS_FRAME_ID_OTA_DATA: {
                    if (otaDataNeedsProcessing && !firmwareUpdateError) { // We have got fresh data
#ifdef MSP_FIRMWARE_UPDATE__WARNING_DEAD_CODE__
                        static uint32_t firmwareUpdateSize;
                        uint32_t receivedSize = otaDataAddress - FBUS_OTA_DATA_FRAME_BYTES;
                        if (otaDataAddress == 0) {
                            static firmwareUpdateHeader_t *header = (firmwareUpdateHeader_t *)otaDataBuffer;
                            firmwareUpdateSize = header->size;
                            firmwareUpdateCRC = header->crc;
                            firmwareUpdateError = !firmwareUpdatePrepare(firmwareUpdateSize);
                        } else if (receivedSize < firmwareUpdateSize) {
                            uint8_t firmwareDataSize = MIN((uint8_t)FBUS_OTA_DATA_FRAME_BYTES, firmwareUpdateSize - receivedSize);
                            firmwareUpdateError = !firmwareUpdateStore(otaDataBuffer, firmwareDataSize);
                        }
#else
                        firmwareUpdateError = true;
#endif
                    }
                    otaDataNeedsProcessing = false;
                    break;
                }

                case FBUS_FRAME_ID_OTA_STOP:
                    otaMode = false;
#ifdef MSP_FIRMWARE_UPDATE__WARNING_DEAD_CODE__
                    readyToUpdateFirmwareTimestamp = currentTimeUs;
#endif
                    break;
            }

            timeDelta_t otaResponseTime = cmpTimeUs(micros(), otaFrameEndTimestamp);
            if (!firmwareUpdateError && (otaResponseTime <= otaMaxResponseTime)) { // We can answer in time (firmwareUpdateStore can take time because it might need to erase flash)
                writeUplinkFramePhyID(downlinkPhyID, otaResponsePayload);
            }

            otaResponsePayload = NULL;
            clearToSend = false;

        } else if ((downlinkPhyID == FBUS_FC_COMMON_ID) || (downlinkPhyID == FBUS_FC_MSP_ID)) {
            if ((downlinkPhyID == FBUS_FC_MSP_ID) && !mspPayload) {
                // Send null frame
            } else if (!sendNullFrame) {
                processSmartPortTelemetry(mspPayload, &clearToSend, NULL);
                mspPayload = NULL;
            }

        } else {
#if defined(USE_SMARTPORT_MASTER__WARNING_DEAD_CODE__)
            int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);

            if (smartportPhyID != -1) {
                if (sendNullFrame) {
                    if (!smartportMasterPhyIDIsActive(smartportPhyID)) { // send null frame only if the sensor is active
                        clearToSend = false;
                    }
                } else {
                    smartPortPayload_t forwardPayload;
                    if (smartportMasterNextForwardResponse(smartportPhyID, &forwardPayload) || smartportMasterGetSensorPayload(smartportPhyID, &forwardPayload)) {
                        writeUplinkFramePhyID(downlinkPhyID, &forwardPayload);
                    }
                    clearToSend = false; // either we answered or the sensor is not active, do not send null frame
                }
            } else {
                clearToSend = false;
            }
#else
            clearToSend = false;
#endif
        }

        if (clearToSend) {
            writeUplinkFramePhyID(downlinkPhyID, &emptySmartPortFrame);
            clearToSend = false;
        }

        sendNullFrame = false;
    }
#endif

    return true;
}

bool fbusRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, bool isFPORT2)
{
    static uint16_t sbusChannelData[26];
    rxRuntimeState->channelData = sbusChannelData;
    sbusChannelsInit(rxConfig, rxRuntimeState);

    rxRuntimeState->channelCount = 26;
    rxRuntimeState->rcFrameStatusFn = frameStatus;
    rxRuntimeState->rcProcessFrameFn = processFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    uint32_t baudRate = isFPORT2 ? FPORT2_BAUDRATE : FBUS_BAUDRATE;

    uint32_t options = FBUS_PORT_OPTIONS |
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

    fbusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        fbusDataReceive,
        NULL,
        baudRate,
        MODE_RXTX,
        options
    );

    if (fbusPort) {
#if defined(USE_TELEMETRY_SMARTPORT)
        telemetryEnabled = initSmartPortTelemetryExternal(writeUplinkFrame);
#endif

        if (rssiSource == RSSI_SOURCE_NONE) {
            rssiSource = RSSI_SOURCE_RX_PROTOCOL;
        }
    }

    return fbusPort != NULL;
}

#endif
