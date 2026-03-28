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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_FBUS_MASTER)

#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/fbus_sensor.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/sport_input.h"
#include "telemetry/smartport.h"
#include "telemetry/smartport_input.h"

#define SMARTPORT_INPUT_BAUDRATE 57600
#define SMARTPORT_INPUT_FRAME_SIZE (sizeof(smartPortPayload_t) + 1)
#define SMARTPORT_INPUT_POLL_INTERVAL_US 4000
#define SMARTPORT_INPUT_DISCOVERY_TIME_MS 5000
#define SMARTPORT_INPUT_MAX_DISCOVERED_SENSORS FBUS_MAX_PHYS_ID
#define SMARTPORT_INPUT_FC_COMMON_ID 0x1B

static serialPort_t *smartPortInputPort = NULL;
static uint8_t smartPortInputDiscoveredSensors[SMARTPORT_INPUT_MAX_DISCOVERED_SENSORS];
static uint8_t smartPortInputDiscoveredCount = 0;
static uint8_t smartPortInputDiscoveredIndex = 0;
static uint8_t smartPortInputCurrentScanPhysicalId = 0;
static timeUs_t smartPortInputNextPollTimeUs = 0;
static timeUs_t smartPortInputDiscoveryEndTimeUs = 0;
static bool smartPortInputInDiscoveryMode = true;

typedef struct smartPortInputParser_s {
    uint8_t physicalId;
    uint8_t payload[SMARTPORT_INPUT_FRAME_SIZE];
    uint8_t bytesRead;
    bool waitingForStart;
    bool awaitingPhysicalId;
    bool byteStuffing;
} smartPortInputParser_t;

static smartPortInputParser_t smartPortInputParser = {
    .waitingForStart = true,
};

static inline uint8_t smartPortInputGetBit(uint8_t value, uint8_t bit)
{
    return (value >> bit) & 1U;
}

static int8_t smartPortInputStripPhyIDCheckBits(uint8_t phyID)
{
    const uint8_t smartPortPhyID = phyID & 0x1F;
    uint8_t expectedPhyID = smartPortPhyID;

    expectedPhyID |= (smartPortInputGetBit(expectedPhyID, 0) ^ smartPortInputGetBit(expectedPhyID, 1) ^ smartPortInputGetBit(expectedPhyID, 2)) << 5;
    expectedPhyID |= (smartPortInputGetBit(expectedPhyID, 2) ^ smartPortInputGetBit(expectedPhyID, 3) ^ smartPortInputGetBit(expectedPhyID, 4)) << 6;
    expectedPhyID |= (smartPortInputGetBit(expectedPhyID, 0) ^ smartPortInputGetBit(expectedPhyID, 2) ^ smartPortInputGetBit(expectedPhyID, 4)) << 7;

    return phyID == expectedPhyID ? smartPortPhyID : -1;
}

static uint8_t smartPortInputPhyIdWithCheckBits(uint8_t phyID)
{
    uint8_t checkedPhyID = phyID;

    checkedPhyID |= (smartPortInputGetBit(checkedPhyID, 0) ^ smartPortInputGetBit(checkedPhyID, 1) ^ smartPortInputGetBit(checkedPhyID, 2)) << 5;
    checkedPhyID |= (smartPortInputGetBit(checkedPhyID, 2) ^ smartPortInputGetBit(checkedPhyID, 3) ^ smartPortInputGetBit(checkedPhyID, 4)) << 6;
    checkedPhyID |= (smartPortInputGetBit(checkedPhyID, 0) ^ smartPortInputGetBit(checkedPhyID, 2) ^ smartPortInputGetBit(checkedPhyID, 4)) << 7;

    return checkedPhyID;
}

static void smartPortInputStartDiscoveryWindow(timeUs_t currentTimeUs)
{
    smartPortInputDiscoveryEndTimeUs = currentTimeUs + ((timeUs_t)SMARTPORT_INPUT_DISCOVERY_TIME_MS * 1000);
}

static void smartPortInputRecordDiscoveredSensor(uint8_t physicalId)
{
    for (uint8_t i = 0; i < smartPortInputDiscoveredCount; i++) {
        if (smartPortInputDiscoveredSensors[i] == physicalId) {
            return;
        }
    }

    if (smartPortInputDiscoveredCount < ARRAYLEN(smartPortInputDiscoveredSensors)) {
        smartPortInputDiscoveredSensors[smartPortInputDiscoveredCount++] = physicalId;
    }
}

static uint8_t smartPortInputTakeNextScanPhysicalId(void)
{
    if (smartPortInputCurrentScanPhysicalId >= FBUS_MAX_PHYS_ID) {
        smartPortInputCurrentScanPhysicalId = 0;
    }

    if (smartPortInputCurrentScanPhysicalId == SMARTPORT_INPUT_FC_COMMON_ID) {
        smartPortInputCurrentScanPhysicalId++;
        if (smartPortInputCurrentScanPhysicalId >= FBUS_MAX_PHYS_ID) {
            smartPortInputCurrentScanPhysicalId = 0;
        }
    }

    return smartPortInputCurrentScanPhysicalId++;
}

static bool smartPortInputFrameIsValid(const uint8_t *payload)
{
    uint16_t checksum = 0;

    for (unsigned i = 0; i < SMARTPORT_INPUT_FRAME_SIZE; i++) {
        checksum += payload[i];
    }

    checksum = (checksum & 0xFF) + (checksum >> 8);

    return checksum == 0xFF;
}

static void smartPortInputProcessFrame(const smartPortInputParser_t *parser)
{
    if (!smartPortInputFrameIsValid(parser->payload)) {
        return;
    }

    const int8_t decodedPhysicalId = smartPortInputStripPhyIDCheckBits(parser->physicalId);
    if (decodedPhysicalId < 0) {
        return;
    }

    if ((uint8_t)decodedPhysicalId >= FBUS_MAX_PHYS_ID) {
        return;
    }

    smartPortInputRecordDiscoveredSensor((uint8_t)decodedPhysicalId);
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        fbusSensorObservePhysicalIdWithSource((uint8_t)decodedPhysicalId, FBUS_SENSOR_SOURCE_SPORT);
    }

    smartPortPayload_t payload;
    memcpy(&payload, parser->payload, sizeof(payload));

    if (payload.frameId == FSSP_DATA_FRAME) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            fbusSensorProcessDataWithSource((uint8_t)decodedPhysicalId, payload.valueId, payload.data, FBUS_SENSOR_SOURCE_SPORT);
        }
    }
}

static void smartPortInputResetFrame(void)
{
    smartPortInputParser.awaitingPhysicalId = true;
    smartPortInputParser.byteStuffing = false;
    smartPortInputParser.bytesRead = 0;
}

static void smartPortInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint8_t byte = (uint8_t)c;

    if (byte == FSSP_START_STOP) {
        smartPortInputParser.waitingForStart = false;
        smartPortInputResetFrame();
        return;
    }

    if (smartPortInputParser.waitingForStart) {
        return;
    }

    if (byte == FSSP_DLE) {
        smartPortInputParser.byteStuffing = true;
        return;
    }

    if (smartPortInputParser.byteStuffing) {
        byte ^= FSSP_DLE_XOR;
        smartPortInputParser.byteStuffing = false;
    }

    if (smartPortInputParser.awaitingPhysicalId) {
        smartPortInputParser.physicalId = byte;
        smartPortInputParser.awaitingPhysicalId = false;
        return;
    }

    if (smartPortInputParser.bytesRead >= SMARTPORT_INPUT_FRAME_SIZE) {
        smartPortInputParser.waitingForStart = true;
        return;
    }

    smartPortInputParser.payload[smartPortInputParser.bytesRead++] = byte;

    if (smartPortInputParser.bytesRead == SMARTPORT_INPUT_FRAME_SIZE) {
        smartPortInputProcessFrame(&smartPortInputParser);
        smartPortInputParser.waitingForStart = true;
    }
}

static void smartPortInputPollSensor(uint8_t physicalId)
{
    serialWrite(smartPortInputPort, FSSP_START_STOP);
    smartPortSendByte(smartPortInputPhyIdWithCheckBits(physicalId), NULL, smartPortInputPort);
}

void initSmartPortInput(void)
{
    if (smartPortInputPort) {
        return;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SMARTPORT_INPUT);
    if (!portConfig) {
        return;
    }

    memset(smartPortInputDiscoveredSensors, 0, sizeof(smartPortInputDiscoveredSensors));
    smartPortInputDiscoveredCount = 0;
    smartPortInputDiscoveredIndex = 0;
    smartPortInputCurrentScanPhysicalId = 0;
    smartPortInputInDiscoveryMode = true;
    smartPortInputNextPollTimeUs = 0;
    smartPortInputStartDiscoveryWindow(micros());

    portOptions_e portOptions =
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
        (sportInputConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        (sportInputConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP) |
        SERIAL_BIDIR | SERIAL_BIDIR_PP;

    smartPortInputPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_SMARTPORT_INPUT,
        smartPortInputDataReceive,
        NULL,
        SMARTPORT_INPUT_BAUDRATE,
        MODE_RXTX,
        portOptions);

    UNUSED(smartPortInputPort);
}

void handleSmartPortInput(timeUs_t currentTimeUs)
{
    if (!smartPortInputPort) {
        return;
    }

    if (cmpTimeUs(currentTimeUs, smartPortInputNextPollTimeUs) < 0) {
        return;
    }

    if (smartPortInputInDiscoveryMode && cmpTimeUs(currentTimeUs, smartPortInputDiscoveryEndTimeUs) >= 0) {
        if (smartPortInputDiscoveredCount > 0) {
            smartPortInputInDiscoveryMode = false;
            smartPortInputDiscoveredIndex = 0;
        } else {
            smartPortInputStartDiscoveryWindow(currentTimeUs);
        }
    }

    uint8_t physicalId = 0;

    if (smartPortInputInDiscoveryMode) {
        physicalId = smartPortInputTakeNextScanPhysicalId();
    } else {
        if (smartPortInputDiscoveredCount == 0) {
            smartPortInputInDiscoveryMode = true;
            smartPortInputStartDiscoveryWindow(currentTimeUs);
            physicalId = smartPortInputTakeNextScanPhysicalId();
        } else {
            if (smartPortInputDiscoveredIndex >= smartPortInputDiscoveredCount) {
                smartPortInputDiscoveredIndex = 0;
            }
            physicalId = smartPortInputDiscoveredSensors[smartPortInputDiscoveredIndex++];
        }
    }

    smartPortInputPollSensor(physicalId);
    smartPortInputNextPollTimeUs = currentTimeUs + SMARTPORT_INPUT_POLL_INTERVAL_US;
}

#endif
