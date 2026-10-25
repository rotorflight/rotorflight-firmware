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

#ifdef USE_CRSF_SENSORS

#include <math.h>
#include <string.h>

#include "build/atomic.h"
#include "build/debug.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/crsf_sensors.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/crsf_sensors.h"
#include "rx/crsf_protocol.h"

#define CRSF_SENSORS_FRAME_BUFFER_SIZE CRSF_FRAME_SIZE_MAX

typedef struct crsfSensorsFrame_s {
    uint8_t data[CRSF_SENSORS_FRAME_BUFFER_SIZE];
    uint8_t length;
    bool valid;
} crsfSensorsFrame_t;

static serialPort_t *crsfSensorsPort;
static volatile uint8_t rxBuffer[CRSF_SENSORS_FRAME_BUFFER_SIZE];
static volatile uint8_t rxPosition;
static volatile uint8_t rxExpectedLength;
static volatile bool rxFrameReady;
static crsfSensorsFrame_t processFrame;

static crsfSensorsGpsData_t gpsData;
static crsfSensorsBatteryData_t batteryData;
static crsfSensorsBaroData_t baroData;
static crsfSensorsCellsData_t cellsData;
static uint32_t cellsPopulatedMask; // bit i set => cellVoltageMv[i] has been reported
static crsfSensorsRpmData_t rpmData;
static uint32_t rpmPopulatedMask; // bit i set => rpmValues[i] has been reported
static bool useBaroAltitude;
static bool useRpm;

// Link-level rx diagnostics - see crsfSensorsGetDebugStats().
static volatile uint32_t debugRxByteCount;
static volatile uint32_t debugRxSyncCount;
static volatile uint32_t debugRxCrcOkCount;
static volatile uint32_t debugRxCrcFailCount;
static volatile uint8_t debugLastFrameType;
static volatile uint8_t debugLastFrameLength;
static volatile uint8_t debugRawBytes[CRSF_SENSORS_DEBUG_RAW_LEN];
static volatile uint8_t debugRawBytesHead;

static uint16_t be16Read(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t be24Read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static int32_t be24ReadSigned(const uint8_t *p)
{
    uint32_t raw = be24Read(p);
    if (raw & 0x800000U) {
        raw |= 0xFF000000U; // sign-extend 24 -> 32 bits
    }
    return (int32_t)raw;
}

static uint32_t be32Read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static uint8_t crsfSensorsCrc8(const volatile uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int32_t decodeBaroAltitudeCm(uint16_t packed)
{
    if (packed & 0x8000) {
        return (int32_t)(packed & 0x7FFF) * 100;
    }

    return ((int32_t)packed - 10000) * 10;
}

static int16_t decodeVerticalSpeedCmS(int8_t packed)
{
    if (packed == 0) {
        return 0;
    }

    const int sign = packed > 0 ? 1 : -1;
    const float magnitude = (expf(fabsf((float)packed) * 0.026f) - 1.0f) * 100.0f;
    return (int16_t)lrintf(magnitude * sign);
}

static void handleGpsFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    // CRSF allows newer fields to be appended to a frame; reject only a
    // payload too short to hold what we read, not one longer than expected.
    if (payloadLength < CRSF_FRAME_GPS_PAYLOAD_SIZE) {
        return;
    }

    gpsData.latitude = (int32_t)be32Read(&payload[0]);
    gpsData.longitude = (int32_t)be32Read(&payload[4]);

    const uint16_t groundspeedKmh100 = be16Read(&payload[8]);
    gpsData.groundspeedCmS = (uint16_t)((uint32_t)groundspeedKmh100 * 100U / 36U);
    gpsData.headingDeg10 = be16Read(&payload[10]) / 10U;
    gpsData.altitudeCm = ((int32_t)be16Read(&payload[12]) - 1000) * 100;
    gpsData.satellites = payload[14];
    gpsData.valid = true;
    gpsData.lastUpdateUs = currentTimeUs;
}

static void handleBatteryFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    // Same forward-compatibility reasoning as handleGpsFrame() above.
    if (payloadLength < CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE) {
        return;
    }

    const uint16_t rawVoltage = be16Read(&payload[0]);
    const uint16_t rawCurrent = be16Read(&payload[2]);

    batteryData.voltageMv = (uint32_t)rawVoltage * 100U;
    batteryData.currentMa = (uint32_t)rawCurrent * 100U;
    batteryData.capacityMah = be24Read(&payload[4]);
    batteryData.remainingPct = payload[7];
    batteryData.valid = true;
    batteryData.lastUpdateUs = currentTimeUs;
}

static void handleBaroFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength < 3) {
        return;
    }

    baroData.altitudeCm = decodeBaroAltitudeCm(be16Read(&payload[0]));
    baroData.verticalSpeedCmS = decodeVerticalSpeedCmS((int8_t)payload[2]);
    baroData.valid = true;
    baroData.lastUpdateUs = currentTimeUs;
}

// CRSF_FRAMETYPE_CELLS (0x0E), per the TBS CRSF spec ("Voltages" / "Voltage
// Group", https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md):
//   byte 0        Voltage_source_id - a SOURCE identifier, not a cell index.
//                 0-127: cell voltages of a single battery (up to 29S); a
//                 sensor reporting more than one battery sends multiple
//                 0x0E frames with a different source_id per battery (0 for
//                 battery 1, 1 for battery 2, ...). 128-255: a general,
//                 non-cell voltage source (e.g. an ESC's input/BEC/MCU
//                 rails), not a per-cell battery reading.
//   byte 1..      uint16_t big-endian millivolt values for that source,
//                 starting at cell 0 of that battery - NOT offset by
//                 source_id.
// We only support a single battery pack, so only source_id 0 is decoded;
// any other source (a second battery, or a general/non-cell voltage group)
// is a different product/reading and would otherwise corrupt this pack's
// cell array if naively merged in.
static void handleCellsFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength < 3) {
        return;
    }

    const uint8_t sourceId = payload[0];
    if (sourceId != 0) {
        return;
    }

    const uint8_t valueCount = (uint8_t)((payloadLength - 1) / 2);
    for (uint8_t i = 0; i < valueCount; i++) {
        const uint8_t cellIndex = i;
        if (cellIndex >= CRSF_SENSORS_CELLS_MAX) {
            break;
        }
        const uint16_t mv = be16Read(&payload[1 + i * 2]);
        if (mv > CRSF_SENSORS_CELL_MV_MAX) {
            // Physically impossible for a single cell of any common
            // chemistry - almost certainly a bad channel/connection on the
            // sensor itself, not a real reading. Skip it rather than
            // letting one faulty channel wreck the pack total.
            continue;
        }
        cellsData.cellVoltageMv[cellIndex] = mv;
        cellsPopulatedMask |= (1u << cellIndex);
    }

    if (cellsPopulatedMask == 0) {
        return;
    }

    uint8_t cellCount = 0;
    uint32_t totalMv = 0;
    for (uint8_t i = 0; i < CRSF_SENSORS_CELLS_MAX; i++) {
        if (cellsPopulatedMask & (1u << i)) {
            totalMv += cellsData.cellVoltageMv[i];
            cellCount = (uint8_t)(i + 1); // highest populated index + 1
        }
    }

    cellsData.cellCount = cellCount;
    cellsData.totalVoltageMv = totalMv;
    cellsData.valid = true;
    cellsData.lastUpdateUs = currentTimeUs;
}

// CRSF_FRAMETYPE_RPM (0x0C), per the TBS CRSF spec - same shape this
// firmware's own outbound telemetry/crsf.c uses to send it (crsfFrameRPM()):
//   byte 0   rpm_source_id - a SOURCE/device identifier (0 = motor group 1,
//            1 = motor group 2, ...), not a starting index into the values
//            below; our own encoder always sends source_id 0.
//   byte 1.. int24_t big-endian RPM values for that source, one per motor
//            in that group, starting at motor 0 - NOT offset by source_id.
// We only support a single motor group, so only source_id 0 is decoded;
// any other source is a different device and would otherwise corrupt this
// group's RPM array if naively merged in at an offset.
static void handleRpmFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength < 4) {
        return;
    }

    const uint8_t sourceId = payload[0];
    if (sourceId != 0) {
        return;
    }

    const uint8_t valueCount = (uint8_t)((payloadLength - 1) / 3);
    for (uint8_t i = 0; i < valueCount; i++) {
        const uint8_t rpmIndex = i;
        if (rpmIndex >= CRSF_SENSORS_RPM_MAX) {
            break;
        }
        rpmData.rpmValues[rpmIndex] = be24ReadSigned(&payload[1 + i * 3]);
        rpmPopulatedMask |= (1u << rpmIndex);
    }

    if (rpmPopulatedMask == 0) {
        return;
    }

    uint8_t rpmCount = 0;
    for (uint8_t i = 0; i < CRSF_SENSORS_RPM_MAX; i++) {
        if (rpmPopulatedMask & (1u << i)) {
            rpmCount = (uint8_t)(i + 1); // highest populated index + 1
        }
    }

    rpmData.rpmCount = rpmCount;
    rpmData.valid = true;
    rpmData.lastUpdateUs = currentTimeUs;
}

static void processReceivedFrame(const crsfSensorsFrame_t *frame, timeUs_t currentTimeUs)
{
    if (!frame->valid || frame->length < 5) {
        return;
    }

    const uint8_t frameLength = frame->data[1];
    if (frameLength < 2 || (uint8_t)(frameLength + 2) != frame->length) {
        return;
    }

    const uint8_t type = frame->data[2];
    const uint8_t payloadLength = frameLength - CRSF_FRAME_LENGTH_TYPE_CRC;
    const uint8_t *payload = &frame->data[3];

    switch (type) {
    case CRSF_FRAMETYPE_GPS:
        handleGpsFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        handleBatteryFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_ALTITUDE_SENSOR:
        handleBaroFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_CELLS:
        handleCellsFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_RPM:
        handleRpmFrame(payload, payloadLength, currentTimeUs);
        break;
    default:
        break;
    }
}

static void crsfSensorsDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const uint8_t byte = (uint8_t)c;

    debugRxByteCount++;
    debugRawBytes[debugRawBytesHead] = byte;
    debugRawBytesHead = (uint8_t)((debugRawBytesHead + 1) % CRSF_SENSORS_DEBUG_RAW_LEN);

    if (rxPosition == 0) {
        // Any byte is a candidate frame start: the CRSF device-address space
        // is nearly the whole byte range (fixed addresses, an ESC block at
        // 0x90-0x97, and a dynamic NAT range at 0x20-0x7F), so filtering on
        // a fixed address list is both incomplete and unnecessary - the
        // length check below and the CRC8 check at frame-end are what
        // actually validate a frame. A false start here just costs one
        // wasted resync.
        debugRxSyncCount++;
        rxBuffer[rxPosition++] = byte;
        rxExpectedLength = 0;
        return;
    }

    if (rxPosition == 1) {
        if (byte < 2 || byte > (CRSF_FRAME_SIZE_MAX - 2)) {
            rxPosition = 0;
            return;
        }
        rxBuffer[rxPosition++] = byte;
        rxExpectedLength = (uint8_t)(byte + 2);
        return;
    }

    if (rxPosition >= CRSF_SENSORS_FRAME_BUFFER_SIZE || (rxExpectedLength != 0 && rxPosition >= rxExpectedLength)) {
        rxPosition = 0;
        rxExpectedLength = 0;
        return;
    }

    rxBuffer[rxPosition++] = byte;

    if (rxExpectedLength != 0 && rxPosition == rxExpectedLength) {
        const uint8_t crc = crsfSensorsCrc8(&rxBuffer[2], (uint8_t)(rxExpectedLength - 3));
        if (crc == rxBuffer[rxExpectedLength - 1]) {
            debugRxCrcOkCount++;
            debugLastFrameType = rxBuffer[2];
            debugLastFrameLength = rxExpectedLength;
            if (!rxFrameReady) {
                memcpy((void *)processFrame.data, (const void *)rxBuffer, rxExpectedLength);
                processFrame.length = rxExpectedLength;
                processFrame.valid = true;
                rxFrameReady = true;
            }
        } else {
            debugRxCrcFailCount++;
        }
        rxPosition = 0;
        rxExpectedLength = 0;
    }
}

void crsfSensorsGetDebugStats(crsfSensorsDebugStats_t *stats)
{
    if (!stats) {
        return;
    }

    stats->rxByteCount = debugRxByteCount;
    stats->rxSyncCount = debugRxSyncCount;
    stats->rxCrcOkCount = debugRxCrcOkCount;
    stats->rxCrcFailCount = debugRxCrcFailCount;
    stats->lastFrameType = debugLastFrameType;
    stats->lastFrameLength = debugLastFrameLength;

    // Oldest-first snapshot of the raw byte ring buffer. Not atomic with respect
    // to the rx ISR, but good enough for a point-in-time debug dump.
    const uint8_t head = debugRawBytesHead;
    for (uint8_t i = 0; i < CRSF_SENSORS_DEBUG_RAW_LEN; i++) {
        stats->rawBytes[i] = debugRawBytes[(head + i) % CRSF_SENSORS_DEBUG_RAW_LEN];
    }
}

void crsfSensorsInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_CRSF_SENSORS);

    crsfSensorsPort = NULL;
    rxPosition = 0;
    rxExpectedLength = 0;
    rxFrameReady = false;
    processFrame.valid = false;
    memset(&gpsData, 0, sizeof(gpsData));
    memset(&batteryData, 0, sizeof(batteryData));
    memset(&baroData, 0, sizeof(baroData));
    memset(&cellsData, 0, sizeof(cellsData));
    cellsPopulatedMask = 0;
    memset(&rpmData, 0, sizeof(rpmData));
    rpmPopulatedMask = 0;
    useBaroAltitude = crsfSensorsConfig()->useBaroAltitude != 0;
    useRpm = crsfSensorsConfig()->useRpm != 0;

    if (!portConfig) {
        return;
    }

    crsfSensorsPort = openSerialPort(portConfig->identifier,
        FUNCTION_CRSF_SENSORS,
        crsfSensorsDataReceive,
        NULL,
        CRSF_BAUDRATE,
        MODE_RX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED |
            (crsfSensorsConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

void crsfSensorsUpdate(timeUs_t currentTimeUs)
{
    const timeDelta_t timeoutUs = (timeDelta_t)crsfSensorsConfig()->sensorTimeoutMs * 1000;
    useBaroAltitude = crsfSensorsConfig()->useBaroAltitude != 0;
    useRpm = crsfSensorsConfig()->useRpm != 0;

    if (rxFrameReady) {
        // Snapshot the frame and clear both flags atomically with respect to
        // the rx ISR: without this, the ISR can overwrite processFrame with
        // a newly received frame the instant rxFrameReady goes false but
        // before this task finishes decoding it, tearing the read and
        // silently dropping the new frame when processFrame.valid is then
        // cleared. Decode happens from the local copy, outside the section.
        crsfSensorsFrame_t frame;
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            frame = processFrame;
            rxFrameReady = false;
            processFrame.valid = false;
        }
        processReceivedFrame(&frame, currentTimeUs);
    }

    if (gpsData.valid && cmpTimeUs(currentTimeUs, gpsData.lastUpdateUs) > timeoutUs) {
        gpsData.valid = false;
    }
    if (batteryData.valid && cmpTimeUs(currentTimeUs, batteryData.lastUpdateUs) > timeoutUs) {
        batteryData.valid = false;
    }
    if (baroData.valid && cmpTimeUs(currentTimeUs, baroData.lastUpdateUs) > timeoutUs) {
        baroData.valid = false;
    }
    if (cellsData.valid && cmpTimeUs(currentTimeUs, cellsData.lastUpdateUs) > timeoutUs) {
        cellsData.valid = false;
        cellsPopulatedMask = 0; // start fresh rather than merging with a stale reading
    }
    if (rpmData.valid && cmpTimeUs(currentTimeUs, rpmData.lastUpdateUs) > timeoutUs) {
        rpmData.valid = false;
        rpmPopulatedMask = 0; // start fresh rather than merging with a stale reading
    }
}

bool crsfSensorsIsEnabled(void)
{
    return crsfSensorsPort != NULL;
}

bool crsfSensorsGetGpsData(crsfSensorsGpsData_t *data)
{
    if (!gpsData.valid) {
        return false;
    }

    if (data) {
        *data = gpsData;
    }
    return true;
}

bool crsfSensorsHasGpsData(void)
{
    return gpsData.valid;
}

bool crsfSensorsGetBatteryData(crsfSensorsBatteryData_t *data)
{
    if (!batteryData.valid) {
        return false;
    }

    if (data) {
        *data = batteryData;
    }
    return true;
}

bool crsfSensorsHasBatteryData(void)
{
    return batteryData.valid;
}

bool crsfSensorsGetBaroData(crsfSensorsBaroData_t *data)
{
    if (!baroData.valid) {
        return false;
    }

    if (data) {
        *data = baroData;
    }
    return true;
}

bool crsfSensorsHasBaroData(void)
{
    return baroData.valid;
}

bool crsfSensorsGetCellsData(crsfSensorsCellsData_t *data)
{
    if (!cellsData.valid) {
        return false;
    }

    if (data) {
        *data = cellsData;
    }
    return true;
}

bool crsfSensorsHasCellsData(void)
{
    return cellsData.valid;
}

bool crsfSensorsGetRpmData(crsfSensorsRpmData_t *data)
{
    if (!rpmData.valid) {
        return false;
    }

    if (data) {
        *data = rpmData;
    }
    return true;
}

bool crsfSensorsHasRpmData(void)
{
    return rpmData.valid;
}

void crsfSensorsSetBaroUse(bool enabled)
{
    useBaroAltitude = enabled;
}

bool crsfSensorsGetBaroUse(void)
{
    return useBaroAltitude;
}

bool crsfSensorsGetRpmUse(void)
{
    return useRpm;
}

#endif
