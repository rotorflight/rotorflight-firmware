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

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "config/feature.h"
#include "config/config.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "common/time.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "flight/mixer.h"

#include "io/serial.h"

#include "esc_sensor.h"



PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .protocol = ESC_SENSOR_PROTO_KISS,
        .halfDuplex = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
        .current_offset = 0,
        .hw4_current_offset = 15,
        .hw4_current_gain = 100,
        .hw4_voltage_gain = 110,
);


enum {
    DEBUG_ESC_1_RPM = 0,
    DEBUG_ESC_1_TEMP,
    DEBUG_ESC_1_VOLTAGE,
    DEBUG_ESC_1_CURRENT,
    DEBUG_ESC_2_RPM,
    DEBUG_ESC_2_TEMP,
    DEBUG_ESC_2_VOLTAGE,
    DEBUG_ESC_2_CURRENT,
};

enum {
    DEBUG_DATA_RPM = 0,
    DEBUG_DATA_PWM,
    DEBUG_DATA_TEMP,
    DEBUG_DATA_VOLTAGE,
    DEBUG_DATA_CURRENT,
    DEBUG_DATA_CAPACITY,
    DEBUG_DATA_EXTRA,
    DEBUG_DATA_AGE,
};

enum {
    DEBUG_FRAME_BYTE_COUNT = 0,
    DEBUG_FRAME_FRAME_COUNT,
    DEBUG_FRAME_SYNC_COUNT,
    DEBUG_FRAME_SYNC_ERRORS,
    DEBUG_FRAME_CRC_ERRORS,
    DEBUG_FRAME_TIMEOUTS,
    DEBUG_FRAME_BUFFER,
};

#define TELEMETRY_BUFFER_SIZE    40

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];
static escSensorData_t escSensorDataCombined;
static bool combinedNeedsUpdate = true;

static timeUs_t dataUpdateUs = 0;

static uint32_t totalByteCount = 0;
static uint32_t totalFrameCount = 0;
static uint32_t totalTimeoutCount = 0;
static uint32_t totalCrcErrorCount = 0;
static uint32_t totalSyncErrorCount = 0;

static uint8_t buffer[TELEMETRY_BUFFER_SIZE] = { 0, };

static uint8_t *bufferPtr = NULL;

static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPos = 0;

static uint8_t  readBytes = 0;
static uint32_t syncCount = 0;


uint8_t getNumberEscBytesRead(void)
{
    return bufferPos;
}

bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint16_t getEscSensorRPM(uint8_t motorNumber)
{
    return escSensorData[motorNumber].rpm;
}

static void combinedDataUpdate(void)
{
    const int motorCount = getMotorCount();

    if (combinedNeedsUpdate && motorCount > 0) {
        escSensorDataCombined.dataAge = 0;
        escSensorDataCombined.temperature = 0;
        escSensorDataCombined.voltage = 0;
        escSensorDataCombined.current = 0;
        escSensorDataCombined.consumption = 0;
        escSensorDataCombined.rpm = 0;

        for (int i = 0; i < motorCount; i++) {
            escSensorDataCombined.dataAge = MAX(escSensorDataCombined.dataAge, escSensorData[i].dataAge);
            escSensorDataCombined.temperature = MAX(escSensorDataCombined.temperature, escSensorData[i].temperature);
            escSensorDataCombined.voltage += escSensorData[i].voltage;
            escSensorDataCombined.current += escSensorData[i].current;
            escSensorDataCombined.consumption += escSensorData[i].consumption;
            escSensorDataCombined.rpm += escSensorData[i].rpm;
        }

        escSensorDataCombined.voltage = escSensorDataCombined.voltage / motorCount;
        escSensorDataCombined.rpm = escSensorDataCombined.rpm / motorCount;

        combinedNeedsUpdate = false;
    }
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
            if (motorNumber < getMotorCount()) {
                return &escSensorData[motorNumber];
            }
            else if (motorNumber == ESC_SENSOR_COMBINED) {
                combinedDataUpdate();
                return &escSensorDataCombined;
            }
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4 ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_OMPHOBBY) {
            if (motorNumber == 0 || motorNumber == ESC_SENSOR_COMBINED)
                return &escSensorData[0];
        }
    }

    return NULL;
}

static FAST_CODE void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    totalByteCount++;

    if (bufferPos < bufferSize) {
        bufferPtr[bufferPos++] = c;
    }
}

bool escSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);

    if (!portConfig) {
        return false;
    }

    if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
        portOptions_e options = SERIAL_NOT_INVERTED  | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, escSensorDataReceive, NULL, 115200, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 19200, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_EVEN | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 115200, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_OMPHOBBY) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 115200, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_COLLECT) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_EVEN | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 115200, MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    return (escSensorPort != NULL);
}


/*
 * KISS ESC TELEMETRY PROTOCOL
 * ---------------------------
 *
 * One packet is ten 8-bit bytes sent with 115200 bps.
 *
 * Byte 0:      Temperature
 * Byte 1,2:    Voltage
 * Byte 3,4:    Current
 * Byte 5,6:    Consumption
 * Byte 7,8:    RPM
 * Byte 9:      CRC8
 *
 */

#define KISS_ESC_BOOTTIME    5000            // 5 seconds
#define KISS_REQ_TIMEOUT     100             // 100 ms (data transfer takes only 900us)
#define ESC_FRAME_SIZE       10

enum {
    ESC_FRAME_PENDING = 0,
    ESC_FRAME_COMPLETE = 1,
    ESC_FRAME_FAILED = 2,
};

enum {
    ESC_TRIGGER_STARTUP = 0,
    ESC_TRIGGER_PENDING = 1,
};

static uint32_t escTriggerTimestamp;
static uint8_t escTriggerState = ESC_TRIGGER_STARTUP;

static uint8_t currentEsc = 0;


static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc ^ crc_seed;

    for (int i = 0 ; i < 8; i++) {
        crc_u = (crc_u & 0x80) ?
            (crc_u << 1) ^ 0x07 :
            (crc_u << 1);
    }

    return crc_u;
}

uint8_t calculateCrc8(const uint8_t *buf, const uint8_t buf_len)
{
    uint8_t crc = 0;

    for (int i = 0; i < buf_len; i++) {
        crc = updateCrc8(buf[i], crc);
    }

    return crc;
}

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    bufferPos = 0;
    bufferPtr = frameBuffer;
    bufferSize = frameLength;
}

static void increaseDataAge(void)
{
    if (escSensorData[currentEsc].dataAge < ESC_DATA_INVALID) {
        escSensorData[currentEsc].dataAge++;
        combinedNeedsUpdate = true;
    }

    DEBUG_AXIS(ESC_SENSOR_DATA, currentEsc, DEBUG_DATA_AGE, escSensorData[currentEsc].dataAge);
}

static void selectNextMotor(void)
{
    if (++currentEsc >= getMotorCount())
        currentEsc = 0;
}

static void setTelemetryReqeust(timeMs_t currentTimeMs)
{
    startEscDataRead(buffer, ESC_FRAME_SIZE);
    getMotorDmaOutput(currentEsc)->protocolControl.requestTelemetry = true;

    escTriggerState = ESC_TRIGGER_PENDING;
    escTriggerTimestamp = currentTimeMs;
}

static uint8_t decodeTelemetryFrame(void)
{
    // First, check the variables that can change in the interrupt
    if (bufferPos < bufferSize)
        return ESC_FRAME_PENDING;

    // Verify CRC8 checksum
    uint16_t chksum = calculateCrc8(buffer, ESC_FRAME_SIZE - 1);
    uint16_t tlmsum = buffer[ESC_FRAME_SIZE - 1];

    if (chksum == tlmsum) {
        uint16_t temp = buffer[0];
        uint16_t volt = buffer[1] << 8 | buffer[2];
        uint16_t curr = buffer[3] << 8 | buffer[4];
        uint16_t capa = buffer[5] << 8 | buffer[6];
        uint16_t erpm = buffer[7] << 8 | buffer[8];

        escSensorData[currentEsc].dataAge = 0;
        escSensorData[currentEsc].temperature = temp;
        escSensorData[currentEsc].voltage = volt;
        escSensorData[currentEsc].current = curr;
        escSensorData[currentEsc].consumption = capa;
        escSensorData[currentEsc].rpm = erpm;

        combinedNeedsUpdate = true;

        totalFrameCount++;

        if (currentEsc == 0) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, curr);
        }
        else if (currentEsc == 1) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_CURRENT, curr);
        }

        if (currentEsc == debugAxis) {
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, erpm);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, volt);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, curr);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capa);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);
        }

        return ESC_FRAME_COMPLETE;
    }

    totalCrcErrorCount++;

    return ESC_FRAME_FAILED;
}

static void kissSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (escTriggerState) {
        case ESC_TRIGGER_STARTUP:
            if (currentTimeMs >= KISS_ESC_BOOTTIME) {
                setTelemetryReqeust(currentTimeMs);
            }
            break;

        case ESC_TRIGGER_PENDING:
            if (currentTimeMs < escTriggerTimestamp + KISS_REQ_TIMEOUT) {
                uint8_t state = decodeTelemetryFrame();
                switch (state) {
                    case ESC_FRAME_COMPLETE:
                        selectNextMotor();
                        setTelemetryReqeust(currentTimeMs);
                        break;
                    case ESC_FRAME_FAILED:
                        increaseDataAge();
                        selectNextMotor();
                        setTelemetryReqeust(currentTimeMs);
                        break;
                    case ESC_FRAME_PENDING:
                        break;
                }
            }
            else {
                increaseDataAge();
                selectNextMotor();
                setTelemetryReqeust(currentTimeMs);

                totalTimeoutCount++;
            }
            break;
    }
}


/*
 * Hobbywing V4 telemetry
 *
 * Credit to:       https://github.com/dgatf/msrc/
 *
 * Byte 0:          Sync 0x9B
 * Byte 1,2,3:      Packet counter
 * Byte 4,5:        Throttle
 * Byte 6,7:        PWM
 * Byte 8,9,10:     RPM
 * Byte 11,12:      Voltage
 * Byte 13,14:      Current
 * Byte 15,16:      Temperature (FETs)
 * Byte 17,18:      Temperature (BEC)
 *
 *
 * Voltage Gain:
 *   3-6S  (LV):    gain = 110
 *   3-8S  (LVv2):  gain = 154
 *   5-12s (HV):    gain = 210
 *
 * Current Gain:
 *   60A:           gain = 60
 *   80A:           gain = 78
 *   100A:          gain = 90
 *   120A:          gain = 100
 *   130A:          gain = 113
 *   150A:          gain = 129
 *   160A:          gain = 137
 *   200A:          gain = 169
 *
 */

static timeUs_t consumptionUpdateUs = 0;

static float totalConsumption = 0.0f;

#define ESCHW4_V_REF            3.3f
#define ESCHW4_DIFFAMP_SHUNT    0.00025f
#define ESCHW4_ADC_RESOLUTION   4096
#define ESCHW4_NTC_BETA         3950.0f
#define ESCHW4_NTC_R1           10000.0f
#define ESCHW4_NTC_R_REF        47000.0f

static float calcTempHW(uint16_t tempRaw)
{
    float voltage = tempRaw * (ESCHW4_V_REF / ESCHW4_ADC_RESOLUTION);
    float ntcR_Rref = (voltage / (ESCHW4_V_REF - voltage)) * (ESCHW4_NTC_R1 / ESCHW4_NTC_R_REF);

    if (ntcR_Rref < 0.001f)
        return 0;

    float temperature = 1.0f / (logf(ntcR_Rref) / ESCHW4_NTC_BETA + 1.0f / 298.15f) - 273.15f;

    if (temperature < 0)
        return 0;

    return temperature;
}

static float calcVoltHW(uint16_t voltRaw)
{
    return voltRaw * (ESCHW4_V_REF / ESCHW4_ADC_RESOLUTION) *
        (escSensorConfig()->hw4_voltage_gain / 10.0f);
}

static float calcCurrHW(uint16_t currentRaw)
{
    if (currentRaw > escSensorConfig()->hw4_current_offset) {
        return (currentRaw - escSensorConfig()->hw4_current_offset) *
            (ESCHW4_V_REF / (ESCHW4_ADC_RESOLUTION * ESCHW4_DIFFAMP_SHUNT * escSensorConfig()->hw4_current_gain / 10.0f));
    }

    return 0;
}

static void frameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}

static void frameTimeoutError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalTimeoutCount++;
}

static bool processHW4TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte == 0x9B) {
            syncCount++;
        }
        else if (dataByte == 0xB9) {
            readBytes = 0;
            syncCount++;
        }
        else {
            frameSyncError();
        }
    }
    else if (readBytes == 12) {
        if (buffer[1] == 0x9B) {
            readBytes = 0;
        }
    }
    else if (readBytes == 19) {
        readBytes = 0;
        return true;
    }

    return false;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW4TelemetryStream(serialRead(escSensorPort))) {
            if (buffer[4] < 4 && buffer[6] < 4 && buffer[8] < 4 &&
                buffer[11] < 0x10 && buffer[13] < 0x10 && buffer[15] < 0x10 && buffer[17] < 0x10) {

                //uint32_t cnt = buffer[1] << 16 | buffer[2] << 8 | buffer[3];
                uint16_t thr = buffer[4] << 8 | buffer[5];
                uint16_t pwm = buffer[6] << 8 | buffer[7];
                uint32_t rpm = buffer[8] << 16 | buffer[9] << 8 | buffer[10];

                float voltage = calcVoltHW(buffer[11] << 8 | buffer[12]);
                float current = calcCurrHW(buffer[13] << 8 | buffer[14]);
                float tempFET = calcTempHW(buffer[15] << 8 | buffer[16]);
                float tempBEC = calcTempHW(buffer[17] << 8 | buffer[18]);

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = lrintf(tempFET);
                escSensorData[0].voltage = lrintf(voltage * 100);
                escSensorData[0].current = lrintf(current * 100);
                escSensorData[0].rpm = rpm / 100;

                // Hobbywing reports the last current reading when the motor stops.
                // That's completely useless, so set it to zero.
                if (rpm < 100 || thr < 50) {
                    escSensorData[0].current = 0.0f;
                }

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, lrintf(current * 100));

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, lrintf(current * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, lrintf(tempBEC * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Calculate consumption using the last valid current reading
    totalConsumption += cmp32(currentTimeUs, consumptionUpdateUs) * escSensorData[0].current * 10.0f;
    consumptionUpdateUs = currentTimeUs;

    // Convert mAus to mAh
    escSensorData[0].consumption = lrintf(totalConsumption / 3600e6f);

    // Log consumption
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, escSensorData[0].consumption);

    // Increment data age counter if no updates in 500ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 500000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Kontronik telemetry
 *
 * See https://www.kontronik.com/fileadmin/kontronik-sobek/Public/Content/Images/Content/Downloads/Software/Kontronik_TelMe_V4.12_1.12_EN.pdf
 *
 * Byte 0-3:        Sync 0x4B 0x4F 0x44 0x4C "KODL"
 * Byte 4-7:        RPM
 * Byte 8-9:        Battery voltage
 * Byte 10-11:      Battery current
 * Byte 12-13:      Motor current average
 * Byte 14-15:      Motor current peak
 * Byte 16-17:      Capacity
 * Byte 18-19:      BEC current
 * Byte 20-21:      BEC Voltage
 * Byte 22-23:      PWM in (us)
 * Byte 24:         Gas in
 * Byte 25:         PWM out
 * Byte 26:         FET temp
 * Byte 27:         BEC temp
 * Byte 28-31:      Error flags
 * Byte 32:         Operational condition
 * Byte 33:         Timing
 * Byte 34-37:      CRC
 *
 */

static uint32_t calculateCRC32(const uint8_t *buf, uint8_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (int i = 0; i < length; i++) {
        crc = crc ^ buf[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }

    return ~crc;
}

static bool processKontronikTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0x4B)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x4F)
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 0x44)
            frameSyncError();
    }
    else if (readBytes == 4) {
        if (dataByte != 0x4C)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 38) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void kontronikSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processKontronikTelemetryStream(serialRead(escSensorPort))) {
            uint32_t crc = buffer[37] << 24 | buffer[36] << 16 | buffer[35] << 8 | buffer[34];

            if (calculateCRC32(buffer, 34) == crc) {
                uint32_t rpm = buffer[7] << 24 | buffer[6] << 16 | buffer[5] << 8 | buffer[4];
                uint16_t pwm = buffer[23] << 8 | buffer[22];
                uint16_t voltage = buffer[9] << 8 | buffer[8];
                uint16_t current = buffer[11] << 8 | buffer[10];
                uint16_t capacity = buffer[17] << 8 | buffer[16];
                uint16_t tempFET = buffer[26];
                uint16_t tempBEC = buffer[27];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = tempFET;
                escSensorData[0].voltage = voltage;
                escSensorData[0].current = current;
                escSensorData[0].rpm = rpm / 100;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tempFET);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tempBEC);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * OMP Hobby telemetry
 *
 * Byte 0:          Start Flag 0xdd
 * Byte 1-2:        Message Type 0x0120
 * Byte 3-4:        Battery voltage (100mV steps)
 * Byte 5-6:        Battery current (100mA steps)
 * Byte 7:          Throttle Percent
 * Byte 8-9:        RPM (10rpm steps)
 * Byte 10:         Temperature
 * Byte 11:         Unused / Zero
 * Byte 12:         PWM Throttle Percent
 * Byte 13:         Unused / Zero
 * Byte 14:         ESC State Code
 * Byte 15-16:      Used Capacity mAh
 * Byte 17-31:      Unused / Zeros
 *
 */

static bool processOMPTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xDD)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void ompSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processOMPTelemetryStream(serialRead(escSensorPort))) {
            // This is a telemetry frame
            if (buffer[1] == 0x01 && buffer[2] == 0x20 && buffer[11] == 0 && buffer[13] == 0) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t pwm = buffer[11];
                uint16_t temp = buffer[10];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[14];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = 0; // capacity; // FIXME bogus value

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Raw Telemetry Data Collector
 */

static void collectSensorProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort) && readBytes < 32) {
        totalByteCount++;
        buffer[readBytes++] = serialRead(escSensorPort);
    }

    if (readBytes > 0) {
        blackboxLogCustomData(buffer, readBytes);
        totalFrameCount++;
        readBytes = 0;
    }
}


void escSensorProcess(timeUs_t currentTimeUs)
{
    if (escSensorPort && motorIsEnabled()) {
        switch (escSensorConfig()->protocol) {
            case ESC_SENSOR_PROTO_KISS:
                kissSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW4:
                hw4SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_KONTRONIK:
                kontronikSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_OMPHOBBY:
                ompSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_COLLECT:
                collectSensorProcess(currentTimeUs);
                break;
        }

        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BYTE_COUNT, totalByteCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_FRAME_COUNT, totalFrameCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_COUNT, syncCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_ERRORS, totalSyncErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_CRC_ERRORS, totalCrcErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_TIMEOUTS, totalTimeoutCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BUFFER, readBytes);
    }
}

#endif
