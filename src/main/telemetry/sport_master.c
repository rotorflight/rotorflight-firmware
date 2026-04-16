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

#if defined(USE_TELEMETRY) && defined(USE_SPORT_MASTER)

#include "common/utils.h"
#include "drivers/fbus_sensor.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/sport_master.h"
#include "telemetry/smartport.h"
#include "telemetry/sport_master.h"

#define SPORT_MASTER_BAUDRATE 57600
#define SPORT_MASTER_FRAME_SIZE (sizeof(smartPortPayload_t) + 1)
#define SPORT_MASTER_POLL_INTERVAL_US 4000
#define SPORT_MASTER_DISCOVERY_TIME_MS 5000
#define SPORT_MASTER_MAX_DISCOVERED_SENSORS FBUS_MAX_PHYS_ID
#define SPORT_MASTER_FC_COMMON_ID 0x1B

static serialPort_t *sportMasterPort = NULL;
static uint8_t sportMasterDiscoveredSensors[SPORT_MASTER_MAX_DISCOVERED_SENSORS];
static uint8_t sportMasterDiscoveredCount = 0;
static uint8_t sportMasterDiscoveredIndex = 0;
static uint8_t sportMasterCurrentScanPhysicalId = 0;
static timeUs_t sportMasterNextPollTimeUs = 0;
static timeUs_t sportMasterDiscoveryEndTimeUs = 0;
static bool sportMasterInDiscoveryMode = true;

typedef struct sportMasterParser_s {
    uint8_t physicalId;
    uint8_t payload[SPORT_MASTER_FRAME_SIZE];
    uint8_t bytesRead;
    bool waitingForStart;
    bool awaitingPhysicalId;
    bool byteStuffing;
} sportMasterParser_t;

static sportMasterParser_t sportMasterParser = {
    .waitingForStart = true,
};

static inline uint8_t sportMasterGetBit(uint8_t value, uint8_t bit)
{
    return (value >> bit) & 1U;
}

static int8_t sportMasterStripPhyIDCheckBits(uint8_t phyID)
{
    const uint8_t smartPortPhyID = phyID & 0x1F;
    uint8_t expectedPhyID = smartPortPhyID;

    expectedPhyID |= (sportMasterGetBit(expectedPhyID, 0) ^ sportMasterGetBit(expectedPhyID, 1) ^ sportMasterGetBit(expectedPhyID, 2)) << 5;
    expectedPhyID |= (sportMasterGetBit(expectedPhyID, 2) ^ sportMasterGetBit(expectedPhyID, 3) ^ sportMasterGetBit(expectedPhyID, 4)) << 6;
    expectedPhyID |= (sportMasterGetBit(expectedPhyID, 0) ^ sportMasterGetBit(expectedPhyID, 2) ^ sportMasterGetBit(expectedPhyID, 4)) << 7;

    return phyID == expectedPhyID ? smartPortPhyID : -1;
}

static uint8_t sportMasterPhyIdWithCheckBits(uint8_t phyID)
{
    uint8_t checkedPhyID = phyID;

    checkedPhyID |= (sportMasterGetBit(checkedPhyID, 0) ^ sportMasterGetBit(checkedPhyID, 1) ^ sportMasterGetBit(checkedPhyID, 2)) << 5;
    checkedPhyID |= (sportMasterGetBit(checkedPhyID, 2) ^ sportMasterGetBit(checkedPhyID, 3) ^ sportMasterGetBit(checkedPhyID, 4)) << 6;
    checkedPhyID |= (sportMasterGetBit(checkedPhyID, 0) ^ sportMasterGetBit(checkedPhyID, 2) ^ sportMasterGetBit(checkedPhyID, 4)) << 7;

    return checkedPhyID;
}

static void sportMasterStartDiscoveryWindow(timeUs_t currentTimeUs)
{
    sportMasterDiscoveryEndTimeUs = currentTimeUs + ((timeUs_t)SPORT_MASTER_DISCOVERY_TIME_MS * 1000);
}

static void sportMasterRecordDiscoveredSensor(uint8_t physicalId)
{
    for (uint8_t i = 0; i < sportMasterDiscoveredCount; i++) {
        if (sportMasterDiscoveredSensors[i] == physicalId) {
            return;
        }
    }

    if (sportMasterDiscoveredCount < ARRAYLEN(sportMasterDiscoveredSensors)) {
        sportMasterDiscoveredSensors[sportMasterDiscoveredCount++] = physicalId;
    }
}

static uint8_t sportMasterTakeNextScanPhysicalId(void)
{
    if (sportMasterCurrentScanPhysicalId >= FBUS_MAX_PHYS_ID) {
        sportMasterCurrentScanPhysicalId = 0;
    }

    if (sportMasterCurrentScanPhysicalId == SPORT_MASTER_FC_COMMON_ID) {
        sportMasterCurrentScanPhysicalId++;
        if (sportMasterCurrentScanPhysicalId >= FBUS_MAX_PHYS_ID) {
            sportMasterCurrentScanPhysicalId = 0;
        }
    }

    return sportMasterCurrentScanPhysicalId++;
}

static bool sportMasterFrameIsValid(const uint8_t *payload)
{
    uint16_t checksum = 0;

    for (unsigned i = 0; i < SPORT_MASTER_FRAME_SIZE; i++) {
        checksum += payload[i];
    }

    checksum = (checksum & 0xFF) + (checksum >> 8);

    return checksum == 0xFF;
}

static void sportMasterProcessFrame(const sportMasterParser_t *parser)
{
    if (!sportMasterFrameIsValid(parser->payload)) {
        return;
    }

    const int8_t decodedPhysicalId = sportMasterStripPhyIDCheckBits(parser->physicalId);
    if (decodedPhysicalId < 0) {
        return;
    }

    if ((uint8_t)decodedPhysicalId >= FBUS_MAX_PHYS_ID) {
        return;
    }

    sportMasterRecordDiscoveredSensor((uint8_t)decodedPhysicalId);

    smartPortPayload_t payload;
    memcpy(&payload, parser->payload, sizeof(payload));

    if (payload.frameId == FSSP_DATA_FRAME) {
        fbusSensorProcessDataWithSource((uint8_t)decodedPhysicalId, payload.valueId, payload.data, FBUS_SENSOR_SOURCE_SPORT);
    } else {
        fbusSensorObservePhysicalIdWithSource((uint8_t)decodedPhysicalId, FBUS_SENSOR_SOURCE_SPORT);
    }
}

static void sportMasterResetFrame(void)
{
    sportMasterParser.awaitingPhysicalId = true;
    sportMasterParser.byteStuffing = false;
    sportMasterParser.bytesRead = 0;
}

static void sportMasterDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint8_t byte = (uint8_t)c;

    if (byte == FSSP_START_STOP) {
        sportMasterParser.waitingForStart = false;
        sportMasterResetFrame();
        return;
    }

    if (sportMasterParser.waitingForStart) {
        return;
    }

    if (byte == FSSP_DLE) {
        sportMasterParser.byteStuffing = true;
        return;
    }

    if (sportMasterParser.byteStuffing) {
        byte ^= FSSP_DLE_XOR;
        sportMasterParser.byteStuffing = false;
    }

    if (sportMasterParser.awaitingPhysicalId) {
        sportMasterParser.physicalId = byte;
        sportMasterParser.awaitingPhysicalId = false;
        return;
    }

    if (sportMasterParser.bytesRead >= SPORT_MASTER_FRAME_SIZE) {
        sportMasterParser.waitingForStart = true;
        return;
    }

    sportMasterParser.payload[sportMasterParser.bytesRead++] = byte;

    if (sportMasterParser.bytesRead == SPORT_MASTER_FRAME_SIZE) {
        sportMasterProcessFrame(&sportMasterParser);
        sportMasterParser.waitingForStart = true;
    }
}

static void sportMasterPollSensor(uint8_t physicalId)
{
    serialWrite(sportMasterPort, FSSP_START_STOP);
    smartPortSendByte(sportMasterPhyIdWithCheckBits(physicalId), NULL, sportMasterPort);
}

void initSportMaster(void)
{
    if (sportMasterPort) {
        return;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SPORT_MASTER);
    if (!portConfig) {
        return;
    }

    memset(sportMasterDiscoveredSensors, 0, sizeof(sportMasterDiscoveredSensors));
    sportMasterDiscoveredCount = 0;
    sportMasterDiscoveredIndex = 0;
    sportMasterCurrentScanPhysicalId = 0;
    sportMasterInDiscoveryMode = true;
    sportMasterNextPollTimeUs = 0;
    sportMasterStartDiscoveryWindow(micros());

    portOptions_e portOptions =
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
        (sportMasterConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        (sportMasterConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP) |
        SERIAL_BIDIR | SERIAL_BIDIR_PP;

    sportMasterPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_SPORT_MASTER,
        sportMasterDataReceive,
        NULL,
        SPORT_MASTER_BAUDRATE,
        MODE_RXTX,
        portOptions);
}

void handleSportMaster(timeUs_t currentTimeUs)
{
    if (!sportMasterPort) {
        return;
    }

    if (cmpTimeUs(currentTimeUs, sportMasterNextPollTimeUs) < 0) {
        return;
    }

    if (sportMasterInDiscoveryMode && cmpTimeUs(currentTimeUs, sportMasterDiscoveryEndTimeUs) >= 0) {
        if (sportMasterDiscoveredCount > 0) {
            sportMasterInDiscoveryMode = false;
            sportMasterDiscoveredIndex = 0;
        } else {
            sportMasterStartDiscoveryWindow(currentTimeUs);
        }
    }

    uint8_t physicalId = 0;

    if (sportMasterInDiscoveryMode) {
        physicalId = sportMasterTakeNextScanPhysicalId();
    } else {
        if (sportMasterDiscoveredCount == 0) {
            sportMasterInDiscoveryMode = true;
            sportMasterStartDiscoveryWindow(currentTimeUs);
            physicalId = sportMasterTakeNextScanPhysicalId();
        } else {
            if (sportMasterDiscoveredIndex >= sportMasterDiscoveredCount) {
                sportMasterDiscoveredIndex = 0;
            }
            physicalId = sportMasterDiscoveredSensors[sportMasterDiscoveredIndex++];
        }
    }

    sportMasterPollSensor(physicalId);
    sportMasterNextPollTimeUs = currentTimeUs + SPORT_MASTER_POLL_INTERVAL_US;
}

#endif
