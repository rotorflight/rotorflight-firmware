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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_TELEMETRY_SMARTPORT)

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"
#include "flight/imu.h"
#include "msp/msp.h"

#include "fc/rc_adjustments.h"

#include "rx/frsky_crc.h"
#include "rx/rx.h"

#include "io/serial.h"
#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "telemetry/msp_shared.h"
#include "telemetry/smartport.h"
#include "telemetry/telemetry.h"


enum {
    TELEMETRY_STATE_UNINITIALIZED,
    TELEMETRY_STATE_INITIALIZED_SERIAL,
    TELEMETRY_STATE_INITIALIZED_EXTERNAL,
};

static uint8_t telemetryState = TELEMETRY_STATE_UNINITIALIZED;

static serialPort_t *smartPortSerialPort = NULL;
static const serialPortConfig_t *portConfig = NULL;
static portSharing_e smartPortPortSharing = 0;

static smartPortWriteFrameFn *smartPortWriteFrame = NULL;

#if defined(USE_MSP_OVER_TELEMETRY)

#define SMARTPORT_MSP_PAYLOAD_SIZE      6

static bool smartPortMspReplyPending = false;

#endif /* USE_MSP_OVER_TELEMETRY */


/** Sensor definitions **/

#define TLM_SENSOR(NAME, APPID, FAST, SLOW, WF, WS, ENC) \
    { \
        .senid = TELEM_##NAME, \
        .appid = (APPID), \
        .fast_interval = (FAST), \
        .slow_interval = (SLOW), \
        .fast_weight = (WF), \
        .slow_weight = (WS), \
        .bucket = 0, \
        .value = 0, \
        .update = 0, \
        .active = false, \
        .encode = (telemetryEncode_f)smartPortSensorEncode##ENC, \
    }

static void smartPortSensorEncodeINT(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    payload->data = sensor->value;
}

static void smartPortSensorEncodeAttitude(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    const uint32_t roll = telemetrySensorValue(TELEM_ATTITUDE_ROLL) & 0xFFFF;
    const uint32_t pitch = telemetrySensorValue(TELEM_ATTITUDE_PITCH) & 0xFFFF;

    payload->data = roll | pitch << 16;
}

static void smartPortSensorEncodeKnots(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    const uint32_t mknots = gpsSol.groundSpeed * 19438UL / 1000;

    payload->data = mknots;
}

static void smartPortSensorEncodeLat(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    const uint32_t lat = (gpsSol.llh.lat * 6 / 100) & 0x7FFFFFFF;

    payload->data = lat;
}

static void smartPortSensorEncodeLon(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    const uint32_t lon = (gpsSol.llh.lon * 6 / 100) | 0x80000000;

    payload->data = lon;
}

static void smartPortSensorEncodeHeading(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    const uint32_t cdeg = gpsSol.groundCourse * 10;

    payload->data = cdeg;
}

static void smartPortSensorEncodeAdjFunc(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    payload->data = getAdjustmentsRangeFunc();
}

static void smartPortSensorEncodeAdjValue(__unused telemetrySensor_t *sensor, smartPortPayload_t *payload)
{
    payload->data = getAdjustmentsRangeValue();
}

static telemetrySensor_t smartportTelemetrySensors[] =
{
    TLM_SENSOR(HEARTBEAT,               0x5100,  1000,  1000,   0,   0,   INT),

    TLM_SENSOR(BATTERY_VOLTAGE,         0x0210,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BATTERY_CURRENT,         0x0200,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BATTERY_CONSUMPTION,     0x5250,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BATTERY_CHARGE_LEVEL,    0x0600,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(BATTERY_CELL_COUNT,      0x5260,   200,  3000,   1,  10,   INT),
    TLM_SENSOR(BATTERY_CELL_VOLTAGE,    0x0910,   200,  3000,   1,  10,   INT),

    TLM_SENSOR(THROTTLE_CONTROL,        0x5440,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(PITCH_CONTROL,           0x5441,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ROLL_CONTROL,            0x5442,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(YAW_CONTROL,             0x5443,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(COLLECTIVE_CONTROL,      0x5444,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(ESC1_VOLTAGE,            0x0218,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_CURRENT,            0x0208,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_CAPACITY,           0x5258,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_ERPM,               0x0508,   100,  3000,   1,  10,   INT),
    //TLM_SENSOR(ESC1_POWER,              0x????,   100,  3000,   1,  10,   INT),
    //TLM_SENSOR(ESC1_THROTTLE,           0x????,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_TEMP1,              0x0418,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_TEMP2,              0x0419,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_BEC_VOLTAGE,        0x0219,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC1_BEC_CURRENT,        0x0229,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(ESC2_VOLTAGE,            0x021A,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC2_CURRENT,            0x020A,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC2_CAPACITY,           0x525A,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC2_ERPM,               0x050A,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC2_TEMP1,              0x041A,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(MCU_VOLTAGE,             0x0900,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BEC_VOLTAGE,             0x0901,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BUS_VOLTAGE,             0x0902,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(ESC_VOLTAGE,             0x0211,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ESC_CURRENT,             0x0201,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(BEC_CURRENT,             0x0222,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(MCU_TEMP,                0x0400,   200,  3000,   1,   2,   INT),
    TLM_SENSOR(ESC_TEMP,                0x0401,   200,  3000,   1,   2,   INT),
    TLM_SENSOR(BEC_TEMP,                0x0402,   200,  3000,   1,   2,   INT),

    TLM_SENSOR(HEADING,                 0x0840,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ALTITUDE,                0x0100,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(VARIOMETER,              0x0110,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(HEADSPEED,               0x0500,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(TAILSPEED,               0x0501,   100,  3000,   1,  10,   INT),

    TLM_SENSOR(ATTITUDE,                0x0730,   100,  3000,   1,  10,   Attitude),

    TLM_SENSOR(ACCEL_X,                 0x0700,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ACCEL_Y,                 0x0710,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(ACCEL_Z,                 0x0720,   100,  3000,   1,  10,   INT),

    //TLM_SENSOR(GPS_SATS,                0x????,   500,  3000,   1,  10,   INT),
    TLM_SENSOR(GPS_COORD,               0x0800,   100,  3000,   1,  10,   Lat),
    TLM_SENSOR(GPS_COORD,               0x0800,   100,  3000,   1,  10,   Lon),
    TLM_SENSOR(GPS_ALTITUDE,            0x0820,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(GPS_HEADING,             0x0840,   100,  3000,   1,  10,   Heading),
    TLM_SENSOR(GPS_GROUNDSPEED,         0x0830,   100,  3000,   1,  10,   Knots),
    //TLM_SENSOR(GPS_HOME_DISTANCE,       0x0420,   100,  3000,   1,  10,   INT),
    //TLM_SENSOR(GPS_HOME_DIRECTION,      0x????,   100,  3000,   1,  10,   INT),

    //TLM_SENSOR(CPU_LOAD,                0x????,   200,  3000,   1,  10,   INT),
    //TLM_SENSOR(SYS_LOAD,                0x????,   200,  3000,   1,  10,   INT),
    //TLM_SENSOR(RT_LOAD,                 0x????,   200,  3000,   1,  10,   INT),

    TLM_SENSOR(MODEL_ID,                0x5460,   200,  3000,   0,   0,   INT),
    TLM_SENSOR(FLIGHT_MODE,             0x5461,   100,  3000,   1,   1,   INT),
    TLM_SENSOR(ARMING_FLAGS,            0x5462,   100,  3000,   1,   1,   INT),
    TLM_SENSOR(ARMING_DISABLE_FLAGS,    0x5463,   100,  3000,   1,   1,   INT),
    TLM_SENSOR(RESCUE_STATE,            0x5454,   100,  3000,   1,   1,   INT),
    TLM_SENSOR(GOVERNOR_STATE,          0x5465,   100,  3000,   1,   1,   INT),

    TLM_SENSOR(PID_PROFILE,             0x5471,   200,  3000,   1,   1,   INT),
    TLM_SENSOR(RATES_PROFILE,           0x5472,   200,  3000,   1,   1,   INT),

    TLM_SENSOR(ADJFUNC,                 0x5110,   200,  3000,   1,   1,   AdjFunc),
    TLM_SENSOR(ADJFUNC,                 0x5111,   200,  3000,   1,   1,   AdjValue),

    TLM_SENSOR(DEBUG_0,                 0x52F0,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_1,                 0x52F1,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_2,                 0x52F2,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_3,                 0x52F3,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_4,                 0x52F4,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_5,                 0x52F5,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_6,                 0x52F6,   100,  3000,   1,  10,   INT),
    TLM_SENSOR(DEBUG_7,                 0x52F8,   100,  3000,   1,  10,   INT),
};


void smartPortSendByte(uint8_t c, uint16_t *checksum, serialPort_t *port)
{
    // smart port escape sequence
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        serialWrite(port, FSSP_DLE);
        serialWrite(port, c ^ FSSP_DLE_XOR);
    } else {
        serialWrite(port, c);
    }

    if (checksum) {
        frskyCheckSumStep(checksum, c);
    }
}

smartPortPayload_t *smartPortDataReceive(uint8_t c, bool *clearToSend, smartPortReadyToSendFn *readyToSend, bool useChecksum)
{
    static uint8_t rxBuffer[sizeof(smartPortPayload_t)];
    static uint8_t smartPortRxBytes = 0;
    static bool skipUntilStart = true;
    static bool awaitingSensorId = false;
    static bool byteStuffing = false;
    static uint16_t checksum = 0;

    if (c == FSSP_START_STOP) {
        *clearToSend = false;
        smartPortRxBytes = 0;
        awaitingSensorId = true;
        skipUntilStart = false;
        return NULL;
    }

    if (skipUntilStart) {
        return NULL;
    }

    if (awaitingSensorId) {
        awaitingSensorId = false;
        if ((c == FSSP_SENSOR_ID1) && readyToSend()) {
            // our slot is starting, start sending
            *clearToSend = true;
            // no need to decode more
            skipUntilStart = true;
        } else if (c == FSSP_SENSOR_ID2) {
            checksum = 0;
        } else {
            skipUntilStart = true;
        }
    } else {
        if (c == FSSP_DLE) {
            byteStuffing = true;
            return NULL;
        } else if (byteStuffing) {
            c ^= FSSP_DLE_XOR;
            byteStuffing = false;
        }

        if (smartPortRxBytes < sizeof(smartPortPayload_t)) {
            rxBuffer[smartPortRxBytes++] = (uint8_t)c;
            checksum += c;
            if (!useChecksum && (smartPortRxBytes == sizeof(smartPortPayload_t))) {
                skipUntilStart = true;
                return (smartPortPayload_t *)&rxBuffer;
            }
        } else {
            skipUntilStart = true;
            checksum += c;
            checksum = (checksum & 0xFF) + (checksum >> 8);
            if (checksum == 0xFF) {
                return (smartPortPayload_t *)&rxBuffer;
            }
        }
    }

    return NULL;
}

void smartPortWriteFrameSerial(const smartPortPayload_t *payload, serialPort_t *port, uint16_t checksum)
{
    uint8_t *data = (uint8_t *)payload;

    for (size_t i = 0; i < sizeof(smartPortPayload_t); i++) {
        smartPortSendByte(*data++, &checksum, port);
    }
    frskyCheckSumFini(&checksum);
    smartPortSendByte(checksum, NULL, port);
}

static void smartPortWriteFrameInternal(const smartPortPayload_t *payload)
{
    smartPortWriteFrameSerial(payload, smartPortSerialPort, 0);
}

static void INIT_CODE initSmartPortSensors(void)
{
    telemetrySensorConfig.batCurrentScale = 100;
    telemetrySensorConfig.becCurrentScale = 1;
    telemetrySensorConfig.attitudeScale = 1;
    telemetrySensorConfig.accelScale = 1;

    telemetryScheduleInit(smartportTelemetrySensors, ARRAYLEN(smartportTelemetrySensors), false);

    for (size_t i = 0; i < ARRAYLEN(smartportTelemetrySensors); i++) {
        telemetrySensor_t * sensor = &smartportTelemetrySensors[i];
        if (telemetrySensorActive(sensor->senid)) {
            for (size_t j = 0; j < TELEM_SENSOR_SLOT_COUNT; j++) {
                if (telemetryConfig()->telemetry_sensors[j] == sensor->senid) {
                    if (telemetryConfig()->telemetry_interval[j])
                        sensor->fast_interval = telemetryConfig()->telemetry_interval[j];
                    if (sensor->slow_interval > 1000)
                        sensor->slow_interval += rand() % 100;
                    telemetryScheduleAdd(sensor);
                }
            }
        }
    }
}

bool INIT_CODE initSmartPortTelemetry(void)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
        if (portConfig) {
            smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);
            smartPortWriteFrame = smartPortWriteFrameInternal;
            initSmartPortSensors();
            telemetryState = TELEMETRY_STATE_INITIALIZED_SERIAL;
        }

        return true;
    }

    return false;
}

bool INIT_CODE initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        smartPortWriteFrame = smartPortWriteFrameExternal;
        initSmartPortSensors();
        telemetryState = TELEMETRY_STATE_INITIALIZED_EXTERNAL;
        return true;
    }

    return false;
}

static void INIT_CODE configureSmartPortTelemetryPort(void)
{
    if (portConfig) {
        portOptions_e portOptions =
            (telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED) |
            (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
            (telemetryConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

        smartPortSerialPort = openSerialPort(
            portConfig->identifier,
            FUNCTION_TELEMETRY_SMARTPORT,
            NULL,
            NULL,
            57600,
            MODE_RXTX,
            portOptions);
    }
}

static void INIT_CODE freeSmartPortTelemetryPort(void)
{
    if (smartPortSerialPort) {
        closeSerialPort(smartPortSerialPort);
        smartPortSerialPort = NULL;
    }
}

void checkSmartPortTelemetryState(void)
{
    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL) {
        const bool enableSerialTelemetry = telemetryDetermineEnabledState(smartPortPortSharing);

        if (enableSerialTelemetry && !smartPortSerialPort) {
            configureSmartPortTelemetryPort();
        } else if (!enableSerialTelemetry && smartPortSerialPort) {
            freeSmartPortTelemetryPort();
        }
    }
}

#if defined(USE_MSP_OVER_TELEMETRY)
static void smartPortSendMspResponse(uint8_t *data, const uint8_t dataSize)
{
    smartPortPayload_t payload;

    payload.frameId = FSSP_MSPS_FRAME;
    memcpy(&payload.valueId, data, MIN(dataSize,SMARTPORT_MSP_PAYLOAD_SIZE));

    smartPortWriteFrame(&payload);
}

bool smartPortPayloadContainsMSP(const smartPortPayload_t *payload)
{
    return payload->frameId == FSSP_MSPC_FRAME_SMARTPORT || payload->frameId == FSSP_MSPC_FRAME_FPORT;
}
#endif

void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const timeUs_t *requestTimeout)
{
    static uint8_t skipRequests = 0;

    if (requestTimeout) {
        if (cmpTimeUs(micros(), *requestTimeout) > 0) {
            *clearToSend = false;
            return;
        }
    }

    if (skipRequests) {
        skipRequests--;
        *clearToSend = false;
        return;
    }

#if defined(USE_MSP_OVER_TELEMETRY)
    if (payload && smartPortPayloadContainsMSP(payload)) {
        uint8_t *frameStart = (uint8_t *)&payload->valueId;
        smartPortMspReplyPending = handleMspFrame(frameStart, SMARTPORT_MSP_PAYLOAD_SIZE, &skipRequests);

        // Skip a few telemetry requests before sending response
        if (skipRequests) {
            *clearToSend = false;
            return;
        }
    }
#else
    UNUSED(payload);
#endif

    if (*clearToSend) {
#if defined(USE_MSP_OVER_TELEMETRY)
        static timeUs_t nextSensorSendTime = 0;

        if (smartPortMspReplyPending) {
            smartPortMspReplyPending = sendMspReply(SMARTPORT_MSP_PAYLOAD_SIZE, &smartPortSendMspResponse);
            // Don't send sensor data for one second.
            nextSensorSendTime = micros() + 1000000;
            *clearToSend = false;
            return;
        }

        // We have a small chance that `nextSendTime` is indeed 0.
        // This is a benign defect and just slows down MSP reply.
        if (nextSensorSendTime != 0) {
            if (cmpTimeUs(micros(), nextSensorSendTime) <= 0) {
                // Don't send sensor data now. This will improve the reception of the MSP message.
                *clearToSend = false;
                return;
            } else {
                // Blackout window passed. Reset `nextSendTime`.
                nextSensorSendTime = 0;
            }
        }
#endif

        telemetrySensor_t *sensor = telemetryScheduleNext();
        if (sensor) {
            smartPortPayload_t frame;

            frame.frameId = FSSP_DATA_FRAME;
            frame.valueId = sensor->appid;
            sensor->encode(sensor, &frame);

            telemetryScheduleCommit(sensor);
            smartPortWriteFrame(&frame);

            *clearToSend = false;
        }
    }
}

static bool serialReadyToSend(void)
{
    return (serialRxBytesWaiting(smartPortSerialPort) == 0);
}

void handleSmartPortTelemetry(timeUs_t currentTimeUs)
{
    if (telemetryState != TELEMETRY_STATE_UNINITIALIZED) {
        telemetryScheduleUpdate(currentTimeUs);
    }

    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL && smartPortSerialPort) {
        smartPortPayload_t *payload = NULL;
        bool clearToSend = false;

        while (serialRxBytesWaiting(smartPortSerialPort) > 0 && !payload) {
            uint8_t c = serialRead(smartPortSerialPort);
            payload = smartPortDataReceive(c, &clearToSend, serialReadyToSend, true);
        }

        processSmartPortTelemetry(payload, &clearToSend, NULL);
    }
}

#endif
