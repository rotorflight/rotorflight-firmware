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

#if defined(USE_SERIALRX_IBUS) && defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)

#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/motors.h"
#include "flight/position.h"

#include "io/gps.h"

#include "pg/telemetry.h"

#include "rx/ibus2_telemetry.h"

#include "sensors/barometer.h"
#include "sensors/esc_sensor.h"
#include "telemetry/ibus_shared.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#define IBUS2_PACKET_TYPE_RESPONSE             0x02
#define IBUS2_TX_TEST_MODE                     0
#define IBUS2_COMMAND_CODE_SHIFT               2
#define IBUS2_COMMAND_CODE_MASK                0x3F
#define IBUS2_COMMAND_FRAME_LEN                21
#define IBUS2_RESPONSE_FRAME_LEN               21
#define IBUS2_VALUE_BUFFER_LEN                 16
#define IBUS2_HUB_ADDRESS                      0x07
#define IBUS2_BROADCAST_ADDRESS                0x3F
#define IBUS2_DEVICE_ADDRESS_BASE              0x38
#define IBUS2_MAX_DEVICES                      7
#define IBUS2_RESPONSE_DELAY_US                160
#define IBUS_TEMPERATURE_OFFSET                400

typedef enum {
    IBUS2_CMD_RESET = 0,
    IBUS2_CMD_GET_TYPE = 1,
    IBUS2_CMD_GET_VALUE = 2,
    IBUS2_CMD_GET_PARAM = 3,
    IBUS2_CMD_SET_PARAM = 4,
} ibus2Command_e;

typedef struct {
    uint8_t type;
    uint8_t valueLength;
} ibus2TelemetryDevice_t;

static serialPort_t *ibus2TelemetryPort = NULL;
static uint8_t ibus2LastAddress = IBUS2_BROADCAST_ADDRESS;
static ibus2TelemetryDevice_t ibus2Devices[IBUS2_MAX_DEVICES];
static uint8_t ibus2DeviceCount = 0;
static bool ibus2PendingCommand = false;
static bool ibus2TelemetryTransmitting = false;
static uint8_t ibus2PendingCommandCode = 0;
static uint8_t ibus2PendingFrame[IBUS2_COMMAND_FRAME_LEN];
static timeUs_t ibus2PendingCommandReceivedAtUs = 0;
static ibus2TelemetryDebug_t ibus2TelemetryDebug = { 0 };

static const uint8_t ibus2SensorPriority[] = {
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
    IBUS_SENSOR_TYPE_BAT_CURR,
    IBUS_SENSOR_TYPE_FUEL,
    IBUS_SENSOR_TYPE_TEMPERATURE,
    IBUS_SENSOR_TYPE_RPM_FLYSKY,
    IBUS_SENSOR_TYPE_ALT_FLYSKY,
    IBUS_SENSOR_TYPE_VERTICAL_SPEED,
    IBUS_SENSOR_TYPE_GPS_STATUS,
    IBUS_SENSOR_TYPE_GPS_DIST,
    IBUS_SENSOR_TYPE_GROUND_SPEED,
    IBUS_SENSOR_TYPE_GPS_LAT,
    IBUS_SENSOR_TYPE_GPS_LON,
    IBUS_SENSOR_TYPE_GPS_ALT,
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

static uint16_t readU16Unaligned(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void writeU16Unaligned(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)(value >> 8);
}

static uint8_t ibus2GetCommandCode(const uint8_t *frame)
{
    return (frame[0] >> IBUS2_COMMAND_CODE_SHIFT) & IBUS2_COMMAND_CODE_MASK;
}

static const escSensorData_t *ibus2GetCombinedEscData(void)
{
#if defined(USE_ESC_SENSOR)
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData && escData->age < ESC_DATA_INVALID) {
        return escData;
    }
#endif

    return NULL;
}

static bool ibus2SensorTypeSupported(uint8_t sensorType)
{
    switch (sensorType) {
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        return isBatteryVoltageConfigured();

    case IBUS_SENSOR_TYPE_BAT_CURR:
        return ibus2GetCombinedEscData() != NULL || isBatteryCurrentConfigured();

    case IBUS_SENSOR_TYPE_FUEL:
        return ibus2GetCombinedEscData() != NULL || isBatteryCurrentConfigured() || getBatteryCapacity() > 0;

    case IBUS_SENSOR_TYPE_TEMPERATURE:
    case IBUS_SENSOR_TYPE_RPM_FLYSKY:
    case IBUS_SENSOR_TYPE_ARMED:
    case IBUS_SENSOR_TYPE_FLIGHT_MODE:
        return true;

    case IBUS_SENSOR_TYPE_ALT_FLYSKY:
    case IBUS_SENSOR_TYPE_ALT:
    case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
#if defined(USE_BARO)
        return sensors(SENSOR_BARO);
#else
        return false;
#endif

    case IBUS_SENSOR_TYPE_GPS_STATUS:
    case IBUS_SENSOR_TYPE_GPS_DIST:
    case IBUS_SENSOR_TYPE_GROUND_SPEED:
    case IBUS_SENSOR_TYPE_GPS_LAT:
    case IBUS_SENSOR_TYPE_GPS_LON:
    case IBUS_SENSOR_TYPE_GPS_ALT:
#if defined(USE_GPS)
        return sensors(SENSOR_GPS);
#else
        return false;
#endif

    default:
        return false;
    }
}

static uint8_t ibus2SensorValueLength(uint8_t sensorType)
{
    switch (sensorType) {
    case IBUS_SENSOR_TYPE_GPS_LAT:
    case IBUS_SENSOR_TYPE_GPS_LON:
    case IBUS_SENSOR_TYPE_GPS_ALT:
    case IBUS_SENSOR_TYPE_ALT:
        return 4;

    default:
        return 2;
    }
}

static void ibus2RefreshDevices(void)
{
    ibus2DeviceCount = 0;

    for (unsigned i = 0; i < ARRAYLEN(ibus2SensorPriority) && ibus2DeviceCount < IBUS2_MAX_DEVICES; i++) {
        const uint8_t sensorType = ibus2SensorPriority[i];
        if (!ibus2SensorTypeSupported(sensorType)) {
            continue;
        }

        ibus2Devices[ibus2DeviceCount].type = sensorType;
        ibus2Devices[ibus2DeviceCount].valueLength = ibus2SensorValueLength(sensorType);
        ibus2DeviceCount++;
    }

    if (ibus2DeviceCount == 0) {
        ibus2Devices[0].type = IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE;
        ibus2Devices[0].valueLength = 2;
        ibus2DeviceCount = 1;
    }
}

static uint8_t ibus2HubType(void)
{
    return 0xF1 + (uint8_t)(ibus2DeviceCount - 1);
}

static int8_t ibus2GetDeviceIndexForAddress(uint8_t address)
{
    if (address < IBUS2_DEVICE_ADDRESS_BASE) {
        return -1;
    }

    const uint8_t index = address - IBUS2_DEVICE_ADDRESS_BASE;
    return index < ibus2DeviceCount ? (int8_t)index : -1;
}

static uint16_t ibus2GetPackVoltage(void)
{
    return getBatteryVoltage();
}

static uint16_t ibus2GetHubVoltage(void)
{
    const escSensorData_t *escData = ibus2GetCombinedEscData();

    if (escData && escData->bec_voltage > 0) {
        return (uint16_t)constrain(escData->bec_voltage / 10, 0, 0xFFFF);
    }

    return ibus2GetPackVoltage();
}

static uint16_t ibus2GetTemperature(void)
{
    uint16_t temperature = gyroGetTemperature() * 10;
#if defined(USE_BARO)
    if (sensors(SENSOR_BARO)) {
        temperature = (uint16_t)((baro.baroTemperature + 50) / 10);
    }
#endif
    return temperature + IBUS_TEMPERATURE_OFFSET;
}

static uint16_t ibus2GetFuel(void)
{
    const escSensorData_t *escData = ibus2GetCombinedEscData();

    if (getBatteryCapacity() > 0) {
        return (uint16_t)calculateBatteryPercentageRemaining();
    }

    if (escData) {
        return (uint16_t)constrain(escData->consumption, 0, 0xFFFF);
    }

    return (uint16_t)constrain(getBatteryCapacityUsed(), 0, 0xFFFF);
}

static uint16_t ibus2GetCurrent(void)
{
    const escSensorData_t *escData = ibus2GetCombinedEscData();

    if (escData) {
        return (uint16_t)constrain(escData->current / 10, 0, 0xFFFF);
    }

    return getBatteryCurrent();
}

static uint16_t ibus2GetMode(void)
{
    uint16_t flightMode = 1;

    if (FLIGHT_MODE(ANGLE_MODE)) {
        flightMode = 0;
    }
    if (FLIGHT_MODE(HORIZON_MODE)) {
        flightMode = 7;
    }
    if (FLIGHT_MODE(RESCUE_MODE)) {
        flightMode = 4;
    }
    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        flightMode = 9;
    }

    return flightMode;
}

static bool ibus2SetGpsValue(uint8_t sensorType, uint8_t *valueBuffer, uint8_t length)
{
#if defined(USE_GPS)
    if (!sensors(SENSOR_GPS)) {
        return false;
    }

    switch (sensorType) {
    case IBUS_SENSOR_TYPE_GPS_STATUS:
        valueBuffer[0] = !STATE(GPS_FIX) ? 1 : (gpsSol.numSat < 5 ? 2 : 3);
        valueBuffer[1] = gpsSol.numSat;
        return true;

    case IBUS_SENSOR_TYPE_GPS_DIST:
        writeU16Unaligned(valueBuffer, GPS_distanceToHome);
        return true;

    case IBUS_SENSOR_TYPE_GROUND_SPEED:
        writeU16Unaligned(valueBuffer, gpsSol.groundSpeed);
        return true;

    case IBUS_SENSOR_TYPE_GPS_LAT:
        if (length >= 4) {
            valueBuffer[0] = (uint8_t)(gpsSol.llh.lat & 0xFF);
            valueBuffer[1] = (uint8_t)((gpsSol.llh.lat >> 8) & 0xFF);
            valueBuffer[2] = (uint8_t)((gpsSol.llh.lat >> 16) & 0xFF);
            valueBuffer[3] = (uint8_t)((gpsSol.llh.lat >> 24) & 0xFF);
            return true;
        }
        return false;

    case IBUS_SENSOR_TYPE_GPS_LON:
        if (length >= 4) {
            valueBuffer[0] = (uint8_t)(gpsSol.llh.lon & 0xFF);
            valueBuffer[1] = (uint8_t)((gpsSol.llh.lon >> 8) & 0xFF);
            valueBuffer[2] = (uint8_t)((gpsSol.llh.lon >> 16) & 0xFF);
            valueBuffer[3] = (uint8_t)((gpsSol.llh.lon >> 24) & 0xFF);
            return true;
        }
        return false;

    case IBUS_SENSOR_TYPE_GPS_ALT:
        if (length >= 4) {
            const int32_t altCm = gpsSol.llh.altCm;
            valueBuffer[0] = (uint8_t)(altCm & 0xFF);
            valueBuffer[1] = (uint8_t)((altCm >> 8) & 0xFF);
            valueBuffer[2] = (uint8_t)((altCm >> 16) & 0xFF);
            valueBuffer[3] = (uint8_t)((altCm >> 24) & 0xFF);
            return true;
        }
        return false;

    default:
        return false;
    }
#else
    UNUSED(sensorType);
    UNUSED(valueBuffer);
    UNUSED(length);
    return false;
#endif
}

static bool ibus2GetSensorValue(uint8_t sensorType, uint8_t *valueBuffer, uint8_t *valueLength)
{
    memset(valueBuffer, 0, IBUS2_VALUE_BUFFER_LEN);
    *valueLength = ibus2SensorValueLength(sensorType);

    if (ibus2SetGpsValue(sensorType, valueBuffer, *valueLength)) {
        return true;
    }

    switch (sensorType) {
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        writeU16Unaligned(valueBuffer, ibus2GetPackVoltage());
        return true;

    case IBUS_SENSOR_TYPE_BAT_CURR:
        writeU16Unaligned(valueBuffer, ibus2GetCurrent());
        return true;

    case IBUS_SENSOR_TYPE_FUEL:
        writeU16Unaligned(valueBuffer, ibus2GetFuel());
        return true;

    case IBUS_SENSOR_TYPE_TEMPERATURE:
        writeU16Unaligned(valueBuffer, ibus2GetTemperature());
        return true;

    case IBUS_SENSOR_TYPE_RPM_FLYSKY:
        writeU16Unaligned(valueBuffer, (uint16_t)constrain(getHeadSpeed(), 0, UINT16_MAX));
        return true;

    case IBUS_SENSOR_TYPE_ALT_FLYSKY:
        writeU16Unaligned(valueBuffer, (uint16_t)(int16_t)constrain(getEstimatedAltitudeCm() / 100, SHRT_MIN, SHRT_MAX));
        return true;

    case IBUS_SENSOR_TYPE_ARMED:
        writeU16Unaligned(valueBuffer, ARMING_FLAG(ARMED) ? 1U : 0U);
        return true;

    case IBUS_SENSOR_TYPE_FLIGHT_MODE:
        writeU16Unaligned(valueBuffer, ibus2GetMode());
        return true;

    case IBUS_SENSOR_TYPE_ALT:
        if (*valueLength >= 4) {
            const int32_t altitudeCm = getEstimatedAltitudeCm();
            valueBuffer[0] = (uint8_t)(altitudeCm & 0xFF);
            valueBuffer[1] = (uint8_t)((altitudeCm >> 8) & 0xFF);
            valueBuffer[2] = (uint8_t)((altitudeCm >> 16) & 0xFF);
            valueBuffer[3] = (uint8_t)((altitudeCm >> 24) & 0xFF);
            return true;
        }
        return false;

    case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
        writeU16Unaligned(valueBuffer, (uint16_t)(int16_t)constrain(getEstimatedVarioCms(), SHRT_MIN, SHRT_MAX));
        return true;

    default:
        return false;
    }
}

#if IBUS2_TX_TEST_MODE
static void ibus2SendTxTestPattern(void)
{
    static const uint8_t testFrame[IBUS2_RESPONSE_FRAME_LEN] = {
        0x55, 0xAA, 0x33, 0xCC, 0x55, 0xAA, 0x33, 0xCC, 0x55, 0xAA, 0x33,
        0xCC, 0x55, 0xAA, 0x33, 0xCC, 0x55, 0xAA, 0x33, 0xCC, 0x96
    };

    if (!ibus2TelemetryPort) {
        return;
    }

    ibus2TelemetryDebug.sendCount++;
    ibus2TelemetryDebug.lastResponseCode = 0x3F;
    ibus2TelemetryDebug.responseByte0 = testFrame[0];
    ibus2TelemetryDebug.responseByte1 = testFrame[1];
    ibus2TelemetryDebug.responseByte2 = testFrame[2];
    ibus2TelemetryDebug.responseByte3 = testFrame[3];
    ibus2TelemetryDebug.responseByte14 = testFrame[14];
    ibus2TelemetryDebug.responseByte15 = testFrame[15];
    serialWriteBuf(ibus2TelemetryPort, testFrame, sizeof(testFrame));
    ibus2TelemetryTransmitting = true;
}
#endif

static void ibus2SendResponse(uint8_t commandCode, const uint8_t *payload, size_t payloadLen)
{
    if (!ibus2TelemetryPort) {
        return;
    }

    uint8_t frame[IBUS2_RESPONSE_FRAME_LEN] = { 0 };
    frame[0] = (uint8_t)(IBUS2_PACKET_TYPE_RESPONSE | (commandCode << IBUS2_COMMAND_CODE_SHIFT));

    if (payloadLen > (IBUS2_RESPONSE_FRAME_LEN - 2)) {
        payloadLen = IBUS2_RESPONSE_FRAME_LEN - 2;
    }

    memcpy(&frame[1], payload, payloadLen);
    frame[IBUS2_RESPONSE_FRAME_LEN - 1] = ibus2Crc8(frame, IBUS2_RESPONSE_FRAME_LEN - 1);

    ibus2TelemetryDebug.sendCount++;
    ibus2TelemetryDebug.lastResponseCode = commandCode;
    ibus2TelemetryDebug.responseByte0 = frame[0];
    ibus2TelemetryDebug.responseByte1 = frame[1];
    ibus2TelemetryDebug.responseByte2 = frame[2];
    ibus2TelemetryDebug.responseByte3 = frame[3];
    ibus2TelemetryDebug.responseByte14 = frame[14];
    ibus2TelemetryDebug.responseByte15 = frame[15];
    serialWriteBuf(ibus2TelemetryPort, frame, sizeof(frame));
    ibus2TelemetryTransmitting = true;
}

static void ibus2SendGetTypeResponse(uint8_t address)
{
    uint8_t payload[IBUS2_RESPONSE_FRAME_LEN - 2] = { 0 };
    uint8_t deviceType = ibus2HubType();
    uint8_t valueLength = 2;

    if (address != IBUS2_HUB_ADDRESS) {
        const int8_t index = ibus2GetDeviceIndexForAddress(address);
        if (index < 0) {
            return;
        }

        deviceType = ibus2Devices[index].type;
        valueLength = ibus2Devices[index].valueLength;
    }

    if (address == IBUS2_HUB_ADDRESS || ibus2GetDeviceIndexForAddress(address) >= 0) {
        payload[0] = deviceType;
        payload[1] = valueLength;
        payload[2] = 0;
        ibus2TelemetryDebug.requiredResources = 0;
        ibus2SendResponse(IBUS2_CMD_GET_TYPE, payload, sizeof(payload));
    }
}

static void ibus2SendGetValueResponse(uint8_t address)
{
    uint8_t payload[IBUS2_RESPONSE_FRAME_LEN - 2] = { 0 };
    uint8_t valueLength = 0;

    if (address == IBUS2_HUB_ADDRESS) {
        writeU16Unaligned(payload, ibus2GetHubVoltage());
        ibus2SendResponse(IBUS2_CMD_GET_VALUE, payload, sizeof(payload));
        return;
    } else {
        const int8_t index = ibus2GetDeviceIndexForAddress(address);
        if (index < 0) {
            return;
        }

        if (!ibus2GetSensorValue(ibus2Devices[index].type, payload, &valueLength)) {
            return;
        }
    }

    ibus2SendResponse(IBUS2_CMD_GET_VALUE, payload, sizeof(payload));
}

static void ibus2SendGetParamResponse(const uint8_t *frame)
{
    uint8_t payload[IBUS2_RESPONSE_FRAME_LEN - 2] = { 0 };
    const uint16_t paramType = readU16Unaligned(&frame[1]);

    writeU16Unaligned(payload, paramType);
    payload[2] = 0;
    ibus2TelemetryDebug.lastParamType = paramType;

    ibus2SendResponse(IBUS2_CMD_GET_PARAM, payload, sizeof(payload));
}

static void ibus2SendSetParamResponse(const uint8_t *frame)
{
    uint8_t payload[IBUS2_RESPONSE_FRAME_LEN - 2] = { 0 };
    const uint16_t paramType = readU16Unaligned(&frame[1]);

    writeU16Unaligned(payload, paramType);
    ibus2TelemetryDebug.lastParamType = paramType;
    payload[2] = 0;

    ibus2SendResponse(IBUS2_CMD_SET_PARAM, payload, sizeof(payload));
}

static void ibus2TelemetryRespondNow(timeUs_t receivedAtUs)
{
    const uint8_t commandCode = ibus2PendingCommandCode;
    const timeDelta_t sendDelayUs = cmpTimeUs(micros(), receivedAtUs);
    ibus2TelemetryDebug.lastSendDelayUs = (uint16_t)constrain(sendDelayUs, 0, UINT16_MAX);
    if (ibus2TelemetryDebug.lastSendDelayUs > ibus2TelemetryDebug.maxSendDelayUs) {
        ibus2TelemetryDebug.maxSendDelayUs = ibus2TelemetryDebug.lastSendDelayUs;
    }

    ibus2PendingCommand = false;
#if IBUS2_TX_TEST_MODE
    ibus2SendTxTestPattern();
    return;
#endif

    if (ibus2LastAddress == IBUS2_BROADCAST_ADDRESS) {
        ibus2TelemetryDebug.broadcastCount++;
        if (commandCode == IBUS2_CMD_RESET) {
            ibus2TelemetryReset();
        }
        return;
    }

    if (ibus2LastAddress != IBUS2_HUB_ADDRESS && ibus2GetDeviceIndexForAddress(ibus2LastAddress) < 0) {
        ibus2TelemetryDebug.addressDropCount++;
        return;
    }

    switch ((ibus2Command_e)commandCode) {
    case IBUS2_CMD_GET_TYPE:
        ibus2SendGetTypeResponse(ibus2LastAddress);
        break;

    case IBUS2_CMD_GET_VALUE:
        ibus2SendGetValueResponse(ibus2LastAddress);
        break;

    case IBUS2_CMD_GET_PARAM:
        ibus2SendGetParamResponse(ibus2PendingFrame);
        break;

    case IBUS2_CMD_SET_PARAM:
        ibus2SendSetParamResponse(ibus2PendingFrame);
        break;

    case IBUS2_CMD_RESET:
    default:
        break;
    }
}

void ibus2TelemetryInit(serialPort_t *port)
{
    ibus2TelemetryPort = port;
    memset(&ibus2TelemetryDebug, 0, sizeof(ibus2TelemetryDebug));
    ibus2TelemetryReset();
}

void ibus2TelemetryReset(void)
{
    ibus2LastAddress = IBUS2_BROADCAST_ADDRESS;
    ibus2PendingCommand = false;
    ibus2PendingCommandCode = 0;
    ibus2PendingCommandReceivedAtUs = 0;
    ibus2TelemetryTransmitting = false;
    ibus2RefreshDevices();
}

void ibus2TelemetrySetRequiredResources(uint8_t requiredResources)
{
    ibus2TelemetryDebug.requiredResources = requiredResources;
}

void ibus2TelemetryUpdateAddress(const uint8_t *frame, size_t frameLen)
{
    if (frameLen < 3) {
        return;
    }

    ibus2LastAddress = frame[2] & IBUS2_BROADCAST_ADDRESS;
    ibus2TelemetryDebug.lastAddress = ibus2LastAddress;
}

void ibus2TelemetryQueueCommand(const uint8_t *frame, size_t frameLen, timeUs_t receivedAtUs)
{
    if (!ibus2TelemetryPort || frameLen != IBUS2_COMMAND_FRAME_LEN) {
        return;
    }

    ibus2PendingCommand = true;
    ibus2PendingCommandCode = ibus2GetCommandCode(frame);
    ibus2TelemetryDebug.queueCount++;
    ibus2TelemetryDebug.lastAddress = ibus2LastAddress;
    ibus2TelemetryDebug.lastCommandCode = ibus2PendingCommandCode;
    ibus2PendingCommandReceivedAtUs = receivedAtUs;
    memcpy(ibus2PendingFrame, frame, sizeof(ibus2PendingFrame));
}

void ibus2TelemetryGetDebug(ibus2TelemetryDebug_t *debug)
{
    if (!debug) {
        return;
    }

    *debug = ibus2TelemetryDebug;
    debug->pendingCommand = ibus2PendingCommand;
    debug->transmitting = ibus2TelemetryTransmitting;
    debug->lastAddress = ibus2LastAddress;
}

bool ibus2TelemetryPending(void)
{
    return ibus2PendingCommand || ibus2TelemetryTransmitting;
}

bool ibus2TelemetryProcess(timeUs_t nowUs)
{
    if (ibus2TelemetryTransmitting) {
        if (isSerialTransmitBufferEmpty(ibus2TelemetryPort)) {
            ibus2TelemetryTransmitting = false;
            ibus2TelemetryDebug.txCompleteCount++;
            return true;
        }
        return false;
    }

    if (ibus2PendingCommand) {
        if (cmpTimeUs(nowUs, ibus2PendingCommandReceivedAtUs) < IBUS2_RESPONSE_DELAY_US) {
            return false;
        }

        ibus2TelemetryRespondNow(ibus2PendingCommandReceivedAtUs);
        return false;
    }

    return true;
}

#else

void ibus2TelemetryInit(serialPort_t *port)
{
    UNUSED(port);
}

void ibus2TelemetryReset(void)
{
}

void ibus2TelemetrySetRequiredResources(uint8_t requiredResources)
{
    UNUSED(requiredResources);
}

void ibus2TelemetryUpdateAddress(const uint8_t *frame, size_t frameLen)
{
    UNUSED(frame);
    UNUSED(frameLen);
}

void ibus2TelemetryQueueCommand(const uint8_t *frame, size_t frameLen, timeUs_t receivedAtUs)
{
    UNUSED(frame);
    UNUSED(frameLen);
    UNUSED(receivedAtUs);
}

void ibus2TelemetryGetDebug(ibus2TelemetryDebug_t *debug)
{
    UNUSED(debug);
}

bool ibus2TelemetryPending(void)
{
    return false;
}

bool ibus2TelemetryProcess(timeUs_t nowUs)
{
    UNUSED(nowUs);
    return true;
}

#endif
