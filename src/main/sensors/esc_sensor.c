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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "build/debug.h"

#include "common/time.h"

#include "config/feature.h"
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

#include "esc_sensor.h"

#include "config/config.h"

#include "flight/mixer.h"

#include "io/serial.h"


PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .protocol = ESC_SENSOR_PROTO_KISS,
        .halfDuplex = 0,
        .offset = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
        .hw4_current_offset = 15,
        .hw4_current_gain = 100,
        .hw4_voltage_gain = 110,
);


enum {
    DEBUG_ESC_MOTOR_INDEX = 0,
    DEBUG_ESC_NUM_TIMEOUTS = 1,
    DEBUG_ESC_NUM_CRC_ERRORS = 2,
    DEBUG_ESC_DATA_AGE = 3,
    DEBUG_ESC_RPM = 4,
    DEBUG_ESC_TEMP = 5,
    DEBUG_ESC_VOLTAGE = 6,
    DEBUG_ESC_CURRENT = 7,
};

typedef enum {
    ESC_SENSOR_FRAME_PENDING = 0,
    ESC_SENSOR_FRAME_COMPLETE = 1,
    ESC_SENSOR_FRAME_FAILED = 2
} escTlmFrameState_t;

typedef enum {
    ESC_SENSOR_TRIGGER_STARTUP = 0,
    ESC_SENSOR_TRIGGER_PENDING = 1,
} escSensorTriggerState_t;

#define ESC_SENSOR_BAUDRATE     115200
#define ESC_BOOTTIME            5000            // 5 seconds
#define ESC_REQUEST_TIMEOUT     100             // 100 ms (data transfer takes only 900us)

#define TELEMETRY_BUFFER_SIZE    40

#define KISS_TELEMETRY_FRAME_SIZE    10

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];

static uint8_t buffer[TELEMETRY_BUFFER_SIZE] = { 0, };

static volatile uint8_t *bufferPtr;
static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPosition = 0;

static escSensorTriggerState_t escSensorTriggerState = ESC_SENSOR_TRIGGER_STARTUP;
static uint32_t escTriggerTimestamp;
static uint8_t escSensorMotor = 0;

static escSensorData_t combinedEscSensorData;
static bool combinedDataNeedsUpdate = true;

static uint32_t totalTimeoutCount = 0;
static uint32_t totalCrcErrorCount = 0;


void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    bufferPtr = frameBuffer;
    bufferPosition = 0;
    bufferSize = frameLength;
}

uint8_t getNumberEscBytesRead(void)
{
    return bufferPosition;
}

bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint16_t getEscSensorRPM(uint8_t motorNumber)
{
    return escSensorData[motorNumber].rpm;
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return NULL;
    }

    if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
        if (motorNumber < getMotorCount()) {
            return &escSensorData[motorNumber];
        }
        else if (motorNumber == ESC_SENSOR_COMBINED) {
            if (combinedDataNeedsUpdate && getMotorCount() > 0) {
                combinedEscSensorData.dataAge = 0;
                combinedEscSensorData.temperature = 0;
                combinedEscSensorData.voltage = 0;
                combinedEscSensorData.current = 0;
                combinedEscSensorData.consumption = 0;
                combinedEscSensorData.rpm = 0;

                for (int i = 0; i < getMotorCount(); i++) {
                    combinedEscSensorData.dataAge = MAX(combinedEscSensorData.dataAge, escSensorData[i].dataAge);
                    combinedEscSensorData.temperature = MAX(combinedEscSensorData.temperature, escSensorData[i].temperature);
                    combinedEscSensorData.voltage += escSensorData[i].voltage;
                    combinedEscSensorData.current += escSensorData[i].current;
                    combinedEscSensorData.consumption += escSensorData[i].consumption;
                    combinedEscSensorData.rpm += escSensorData[i].rpm;
                }

                combinedEscSensorData.voltage = combinedEscSensorData.voltage / getMotorCount();
                combinedEscSensorData.rpm = combinedEscSensorData.rpm / getMotorCount();

                combinedDataNeedsUpdate = false;

                DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_DATA_AGE, combinedEscSensorData.dataAge);
            }

            return &combinedEscSensorData;
        }
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        return &escSensorData[0];
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK) {
        return &escSensorData[0];
    }

    return NULL;
}

static inline bool isFrameComplete(void)
{
    return bufferPosition == bufferSize;
}

static FAST_CODE void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    if (bufferPosition < bufferSize) {
        bufferPtr[bufferPosition++] = c;
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

        // Initialize serial port
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, escSensorDataReceive, NULL, ESC_SENSOR_BAUDRATE, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

        // Initialize serial port with no callback. We will just process the buffer.
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 19200, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_EVEN | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

        // Initialize serial port with no callback. We will just process the buffer.
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

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

static uint8_t decodeEscFrame(void)
{
    if (!isFrameComplete()) {
        return ESC_SENSOR_FRAME_PENDING;
    }

    // Get CRC8 checksum
    uint16_t chksum = calculateCrc8(buffer, KISS_TELEMETRY_FRAME_SIZE - 1);
    uint16_t tlmsum = buffer[KISS_TELEMETRY_FRAME_SIZE - 1];     // last byte contains CRC value
    uint8_t frameStatus;
    if (chksum == tlmsum) {
        escSensorData[escSensorMotor].dataAge = 0;
        escSensorData[escSensorMotor].temperature = buffer[0];
        escSensorData[escSensorMotor].voltage = buffer[1] << 8 | buffer[2];
        escSensorData[escSensorMotor].current = buffer[3] << 8 | buffer[4];
        escSensorData[escSensorMotor].consumption = buffer[5] << 8 | buffer[6];
        escSensorData[escSensorMotor].rpm = buffer[7] << 8 | buffer[8];

        combinedDataNeedsUpdate = true;

        frameStatus = ESC_SENSOR_FRAME_COMPLETE;

        DEBUG_SET(DEBUG_ESC_SENSOR_RPM, escSensorMotor, escSensorData[escSensorMotor].rpm);
        DEBUG_SET(DEBUG_ESC_SENSOR_TMP, escSensorMotor, escSensorData[escSensorMotor].temperature);
    } else {
        frameStatus = ESC_SENSOR_FRAME_FAILED;
    }

    return frameStatus;
}

static void increaseDataAge(void)
{
    if (escSensorData[escSensorMotor].dataAge < ESC_DATA_INVALID) {
        escSensorData[escSensorMotor].dataAge++;

        combinedDataNeedsUpdate = true;
    }
}

static void selectNextMotor(void)
{
    escSensorMotor++;
    if (escSensorMotor == getMotorCount()) {
        escSensorMotor = 0;
    }
}

static void setRequest(timeMs_t currentTimeMs)
{
    startEscDataRead(buffer, KISS_TELEMETRY_FRAME_SIZE);
    getMotorDmaOutput(escSensorMotor)->protocolControl.requestTelemetry = true;

    escSensorTriggerState = ESC_SENSOR_TRIGGER_PENDING;
    escTriggerTimestamp = currentTimeMs;

    DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_MOTOR_INDEX, escSensorMotor + 1);
}

static void kissSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (escSensorTriggerState) {
        case ESC_SENSOR_TRIGGER_STARTUP:
            // Wait period of time before requesting telemetry (let the system boot first)
            if (currentTimeMs >= ESC_BOOTTIME) {
                setRequest(currentTimeMs);
            }
            break;

        case ESC_SENSOR_TRIGGER_PENDING:
            if (currentTimeMs < escTriggerTimestamp + ESC_REQUEST_TIMEOUT) {
                uint8_t state = decodeEscFrame();
                switch (state) {
                    case ESC_SENSOR_FRAME_COMPLETE:
                        selectNextMotor();
                        setRequest(currentTimeMs);
                        break;
                    case ESC_SENSOR_FRAME_FAILED:
                        increaseDataAge();
                        selectNextMotor();
                        setRequest(currentTimeMs);
                        DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);
                        break;
                    case ESC_SENSOR_FRAME_PENDING:
                        break;
                }
            } else {
                // Move on to next ESC, we'll come back to this one
                increaseDataAge();
                selectNextMotor();
                setRequest(currentTimeMs);
                DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, ++totalTimeoutCount);
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

static uint8_t skipBytes = 0;
static uint8_t bytesRead = 0;

static timeUs_t dataUpdateUs = 0;
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

static bool processHW4TelemetryStream(uint8_t dataByte)
{
    if (skipBytes > 0) {
        // Ignore the data in these ?non-telemetry? packets
        skipBytes--;
    }
    else if (bytesRead == 0 && dataByte == 0x9B) {
        // Start of a potentially valid packet
        buffer[bytesRead++] = dataByte;
    }
    else if (bytesRead == 1 && dataByte == 0x9B) {
        // Signature packet - skip it
        bytesRead = 0;
        skipBytes = 11;
    }
    else if (bytesRead > 0) {
        // Store 19 bytes in the data buffer
        buffer[bytesRead++] = dataByte;
        if (bytesRead == 19) {
            bytesRead = 0;
            return true;
        }
    }

    return false;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        dataUpdateUs = currentTimeUs;
    }

    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW4TelemetryStream(serialRead(escSensorPort))) {
            if (buffer[4] < 4 && buffer[6] < 4 && buffer[8] < 4 &&
                buffer[11] < 0xF && buffer[13] < 0xF && buffer[15] < 0xF && buffer[17] < 0xF) {

                uint32_t cnt = buffer[1] << 16 | buffer[2] << 8 | buffer[3];
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

                DEBUG(ESC_SENSOR, DEBUG_ESC_MOTOR_INDEX, cnt);
                DEBUG(ESC_SENSOR, DEBUG_ESC_RPM, lrintf(rpm));
                DEBUG(ESC_SENSOR, DEBUG_ESC_TEMP, lrintf(tempFET));
                DEBUG(ESC_SENSOR, DEBUG_ESC_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR, DEBUG_ESC_CURRENT, lrintf(current * 100));

                DEBUG(ESC_SENSOR_RPM, 0, rpm);
                DEBUG(ESC_SENSOR_RPM, 1, thr);
                DEBUG(ESC_SENSOR_RPM, 2, pwm);

                DEBUG(ESC_SENSOR_TMP, 0, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR_TMP, 1, lrintf(tempBEC * 10));

                dataUpdateUs = currentTimeUs;
            }
            else {
                DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);
            }
        }
    }

    // Log the buffer size as "timeouts"
    DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, bytesRead);

    // Log the data age to see how old the data gets
    DEBUG(ESC_SENSOR, DEBUG_ESC_DATA_AGE, escSensorData[escSensorMotor].dataAge);

    // Calculate consumption using the last valid current reading
    totalConsumption += cmp32(currentTimeUs, consumptionUpdateUs) * escSensorData[0].current * 10.0f;
    consumptionUpdateUs = currentTimeUs;

    // Convert mAus to mAh
    escSensorData[0].consumption = lrintf(totalConsumption / 3600e6f);
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
    if (bytesRead == 0) {
        if (dataByte == 0x4B)
            buffer[bytesRead++] = dataByte;
    }
    else if (bytesRead == 1) {
        if (dataByte == 0x4F)
            buffer[bytesRead++] = dataByte;
        else
            bytesRead = 0;
    }
    else if (bytesRead == 2) {
        if (dataByte == 0x44)
            buffer[bytesRead++] = dataByte;
        else
            bytesRead = 0;
    }
    else if (bytesRead == 3) {
        if (dataByte == 0x4C)
            buffer[bytesRead++] = dataByte;
        else
            bytesRead = 0;
    }
    else if (bytesRead < 38) {
        buffer[bytesRead++] = dataByte;
    }
    else {
        bytesRead = 0;
        return true;
    }

    return false;
}

static void kontronikSensorProcess(timeUs_t currentTimeUs)
{
    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        dataUpdateUs = currentTimeUs;
    }

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

                //DEBUG(ESC_SENSOR, DEBUG_ESC_MOTOR_INDEX, cnt);
                DEBUG(ESC_SENSOR, DEBUG_ESC_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_TEMP, tempFET);
                DEBUG(ESC_SENSOR, DEBUG_ESC_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_CURRENT, current);

                DEBUG(ESC_SENSOR_RPM, 0, rpm);
                DEBUG(ESC_SENSOR_RPM, 2, pwm);

                DEBUG(ESC_SENSOR_TMP, 0, tempFET * 10);
                DEBUG(ESC_SENSOR_TMP, 1, tempBEC * 10);

                dataUpdateUs = currentTimeUs;
            }
            else {
                DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);
            }
        }
    }

    // Log the buffer size as "timeouts"
    DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, bytesRead);

    // Log the data age to see how old the data gets
    DEBUG(ESC_SENSOR, DEBUG_ESC_DATA_AGE, escSensorData[escSensorMotor].dataAge);
}


void escSensorProcess(timeUs_t currentTimeUs)
{
    if (escSensorPort && motorIsEnabled()) {
        if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
            kissSensorProcess(currentTimeUs);
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
            hw4SensorProcess(currentTimeUs);
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK) {
            kontronikSensorProcess(currentTimeUs);
        }
    }
}

#endif
