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
#include "common/crc.h"

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
        .protocol = ESC_SENSOR_PROTO_NONE,
        .halfDuplex = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
        .current_offset = 0,
        .hw4_current_offset = 0,
        .hw4_current_gain = 0,
        .hw4_voltage_gain = 0,
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
#define REQUEST_BUFFER_SIZE      40
#define PARAM_BUFFER_SIZE        96
#define PARAM_HEADER_SIZE        2
#define PARAM_HEADER_SIG         0
#define PARAM_HEADER_VER         1
#define PARAM_HEADER_VER_MASK    0x3F
#define PARAM_HEADER_CMD_MASK    0xC0
#define PARAM_HEADER_RDONLY      0x40
#define PARAM_HEADER_USER        0x80

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];
static escSensorData_t escSensorDataCombined;
static bool combinedNeedsUpdate = true;

static timeUs_t dataUpdateUs = 0;
static timeUs_t consumptionUpdateUs = 0;

static float consumptionDelta = 0.0f;
static float totalConsumption = 0.0f;

static uint32_t totalByteCount = 0;
static uint32_t totalFrameCount = 0;
static uint32_t totalTimeoutCount = 0;
static uint32_t totalCrcErrorCount = 0;
static uint32_t totalSyncErrorCount = 0;

static uint8_t buffer[TELEMETRY_BUFFER_SIZE] = { 0, };

static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPos = 0;

static volatile uint8_t readBytes = 0;
static volatile uint8_t readIngoreBytes = 0;
static uint32_t syncCount = 0;

static uint8_t reqLength = 0;
static uint8_t reqbuffer[REQUEST_BUFFER_SIZE] = { 0, };

static uint8_t paramPayloadLength = 0;
static uint8_t paramBuffer[PARAM_BUFFER_SIZE] = { 0, };
static uint8_t paramUpdBuffer[PARAM_BUFFER_SIZE] = { 0, };
static uint8_t *paramPayload = paramBuffer + PARAM_HEADER_SIZE;
static uint8_t *paramUpdPayload = paramUpdBuffer + PARAM_HEADER_SIZE;
static uint8_t paramSig = 0;
static uint8_t paramVer = 0;
static bool paramMspActive = false;

// called on MSP_SET_ESC_PARAMETERS when paramUpdPayload / paramUpdBuffer ready
typedef bool (*paramCommitCallbackPtr)(uint8_t cmd);
static paramCommitCallbackPtr paramCommit = NULL;


bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint32_t getEscSensorRPM(uint8_t motorNumber)
{
    return (escSensorData[motorNumber].age <= ESC_BATTERY_AGE_MAX) ? escSensorData[motorNumber].erpm : 0;
}

static void combinedDataUpdate(void)
{
    const int motorCount = getMotorCount();

    if (combinedNeedsUpdate && motorCount > 0) {
        memset(&escSensorDataCombined, 0, sizeof(escSensorDataCombined));

        for (int i = 0; i < motorCount; i++) {
            if (escSensorData[i].age < ESC_BATTERY_AGE_MAX) {
                escSensorDataCombined.age = MAX(escSensorDataCombined.age, escSensorData[i].age);
                escSensorDataCombined.pwm = MAX(escSensorDataCombined.pwm, escSensorData[i].pwm);
                escSensorDataCombined.erpm = MAX(escSensorDataCombined.erpm, escSensorData[i].erpm);
                escSensorDataCombined.voltage = MAX(escSensorDataCombined.voltage, escSensorData[i].voltage);
                escSensorDataCombined.current += escSensorData[i].current;
                escSensorDataCombined.consumption += escSensorData[i].consumption;
                escSensorDataCombined.temperature = MAX(escSensorDataCombined.temperature, escSensorData[i].temperature);
                escSensorDataCombined.temperature2 = MAX(escSensorDataCombined.temperature2, escSensorData[i].temperature2);
            }
        }

        combinedNeedsUpdate = false;
    }
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (getMotorCount() > 0 && featureIsEnabled(FEATURE_ESC_SENSOR)) {
        if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_NONE) {
            return NULL;
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_BLHELI32) {
            if (motorNumber < getMotorCount()) {
                return &escSensorData[motorNumber];
            }
            else if (motorNumber == ESC_SENSOR_COMBINED) {
                combinedDataUpdate();
                return &escSensorDataCombined;
            }
        }
        else {
            if (motorNumber == 0 || motorNumber == ESC_SENSOR_COMBINED)
                return &escSensorData[0];
        }
    }

    return NULL;
}


/*
 * Common functions
 */

static void frameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}

static void increaseDataAge(uint8_t motor)
{
    if (escSensorData[motor].age < ESC_DATA_INVALID) {
        escSensorData[motor].age++;
        combinedNeedsUpdate = true;
    }

    DEBUG_AXIS(ESC_SENSOR_DATA, motor, DEBUG_DATA_AGE, escSensorData[motor].age);
}

// Only for non-BLHeli32 protocols with single ESC support
static void checkFrameTimeout(timeUs_t currentTimeUs, timeDelta_t timeout)
{
    // Increment data age counter if no updates
    if (cmpTimeUs(currentTimeUs, dataUpdateUs) > timeout) {
        increaseDataAge(0);
        totalTimeoutCount++;
        dataUpdateUs = currentTimeUs;
    }
}

static void setConsumptionCurrent(float current)
{
    // Pre-convert Aµs to mAh
    consumptionDelta = current * (1000.0f / 3600e6f);

}

static void updateConsumption(timeUs_t currentTimeUs)
{
    // Increment consumption
    totalConsumption += cmpTimeUs(currentTimeUs, consumptionUpdateUs) * consumptionDelta;

    // Save update time
    consumptionUpdateUs = currentTimeUs;

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, totalConsumption);

    escSensorData[0].consumption = totalConsumption;
}


/*
 * BLHeli32 / KISS Telemetry Protocol
 *
 *     - Serial protocol is 115200,8N1
 *     - Big-Endian byte order
 *
 * Data Frame Format
 * ―――――――――――――――――――――――――――――――――――――――――――――――
 *     0:       Temperature
 *   1,2:       Voltage in 10mV
 *   3,4:       Current in 10mA
 *   5,6:       Consumption mAh
 *   7,8:       RPM in 100rpm steps
 *     9:       CRC8
 *
 */

#define BLHELI32_BOOT_DELAY       5000            // 5 seconds
#define BLHELI32_REQ_TIMEOUT      100             // 100 ms (data transfer takes only 900us)
#define BLHELI32_FRAME_SIZE       10

enum {
    BLHELI32_FRAME_FAILED    = 0,
    BLHELI32_FRAME_PENDING   = 1,
    BLHELI32_FRAME_COMPLETE  = 2,
};

enum {
    DSHOT_TRIGGER_WAIT = 0,
    DSHOT_TRIGGER_ACTIVE = 1,
};

static uint32_t dshotTriggerTimestamp = 0;
static uint8_t dshotTriggerState = DSHOT_TRIGGER_WAIT;

static uint8_t currentEsc = 0;


static FAST_CODE void blDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    totalByteCount++;

    if (bufferPos < bufferSize) {
        buffer[bufferPos++] = c;
    }
}

static void sendDShotTelemetryReqeust(timeMs_t currentTimeMs)
{
    bufferPos = 0;
    bufferSize = BLHELI32_FRAME_SIZE;

    dshotTriggerTimestamp = currentTimeMs;

    getMotorDmaOutput(currentEsc)->protocolControl.requestTelemetry = true;
}

static void blSelectNextEsc(void)
{
    currentEsc = (currentEsc + 1) % getMotorCount();
}

static uint8_t blDecodeTelemetryFrame(void)
{
    // First, check the variables that can change in the interrupt
    if (bufferPos < bufferSize)
        return BLHELI32_FRAME_PENDING;

    // Verify CRC8 checksum
    uint16_t chksum = crc8_kiss_update(0, buffer, BLHELI32_FRAME_SIZE - 1);
    uint16_t tlmsum = buffer[BLHELI32_FRAME_SIZE - 1];

    if (chksum == tlmsum) {
        uint16_t temp = buffer[0];
        uint16_t volt = buffer[1] << 8 | buffer[2];
        uint16_t curr = buffer[3] << 8 | buffer[4];
        uint16_t capa = buffer[5] << 8 | buffer[6];
        uint16_t erpm = buffer[7] << 8 | buffer[8];

        escSensorData[currentEsc].age = 0;
        escSensorData[currentEsc].erpm = erpm * 100;
        escSensorData[currentEsc].voltage = volt * 10;
        escSensorData[currentEsc].current = curr * 10;
        escSensorData[currentEsc].consumption = capa;
        escSensorData[currentEsc].temperature = temp * 10;

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

        return BLHELI32_FRAME_COMPLETE;
    }

    totalCrcErrorCount++;

    return BLHELI32_FRAME_FAILED;
}

static void blSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (dshotTriggerState) {
        case DSHOT_TRIGGER_WAIT:
            if (currentTimeMs >= BLHELI32_BOOT_DELAY) {
                dshotTriggerState = DSHOT_TRIGGER_ACTIVE;
                sendDShotTelemetryReqeust(currentTimeMs);
            }
            break;

        case DSHOT_TRIGGER_ACTIVE:
            if (currentTimeMs < dshotTriggerTimestamp + BLHELI32_REQ_TIMEOUT) {
                uint8_t state = blDecodeTelemetryFrame();
                switch (state) {
                    case BLHELI32_FRAME_PENDING:
                        break;
                    case BLHELI32_FRAME_FAILED:
                        increaseDataAge(currentEsc);
                        FALLTHROUGH;
                    case BLHELI32_FRAME_COMPLETE:
                        blSelectNextEsc();
                        sendDShotTelemetryReqeust(currentTimeMs);
                        break;
                }
            }
            else {
                increaseDataAge(currentEsc);
                blSelectNextEsc();
                sendDShotTelemetryReqeust(currentTimeMs);
                totalTimeoutCount++;
            }
            break;
    }
}


/*
 * Calculate temperature from an NTC sensor ADC reading
 *
 * Let
 *     Rᵣ = Reference R
 *     Rₙ = NTC nominal R
 *     Tₙ = NTC nominal temp (25°C)
 *     Tₖ = 0°C in Kelvin = 273.15K
 *     β  = NTC beta
 *
 * and
 *
 *     T₁ = Tₖ + Tₙ = 298.15K
 *
 * Then
 *
 *          x         Rᵣ
 *  R = ―――――――――― ⋅ ――――
 *       4096 - x     Rₙ
 *
 *
 *             1
 *  T = ――――――――――――――― - Tₖ
 *       ln(R)/β + 1/T₁
 *
 *
 * Simplify:
 *
 *            1
 *  T = ―――――――――――――― - Tₖ
 *        γ⋅ln(S) + δ
 *
 * Where
 *          x
 *  S = ――――――――――
 *       4096 - x
 *
 *  γ = 1 / β
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁
 *
 */

static float calcTempNTC(uint16_t adc, float gamma, float delta)
{
    const float X = constrainf(adc, 1, 4095);
    const float R = X / (4096 - X);
    const float A = logf(R) * gamma + delta;
    const float T = 1 / A - 273.15f;

    return T;
}


/*
 * Hobbywing V4 telemetry
 *
 *    - Serial protocol 19200,8N1
 *    - Frame rate running:20Hz idle:2.5Hz
 *    - Big-Endian fields
 *
 * Data Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *     0:       Sync 0x9B
 *   1-3:       Packet counter
 *   4-5:       Throttle %
 *   6-7:       PWM duty cycle %
 *  8-10:       RPM
 * 11-12:       Voltage ADC
 * 13-14:       Current ADC
 * 15-16:       Temperature ADC (FET)
 * 17-18:       Temperature ADC (CAP)
 *    19:       Sync 0xB9 (present only in slow rate)
 *
 * Info Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *     0:       Sync 0x9B
 *     1:       Sync 0x9B
 *   2-3:       Throttle step
 *     4:       RPM steps (always 1)
 *   5-6:       Voltage constants
 *   7-9:       Current constants
 * 10-11:       Temperature constants
 *    12:       Sync 0xB9
 *
 *
 * Gain values reported by the ESCs:
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 * Model        V1  V2  I1   I2  I3     Vgain Igain Ioffset
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 * 60A          8   91   0    1   0      109      0      0
 * 80A          8   91  33  150  90      109    146    409
 * 120A         8   91  33  113 110      109    110    377
 * HV130A       11  65  30  146   0      210    157      0
 * HV200A       11  65  30  146  98      210    157    477
 * FFHV160A     11  65  33   68 185      210     66    381
 *
 * Temp sensor design:
 * ―――――――――――――――――――
 *  β  = 3950
 *  Tₙ = 25°C
 *  Rᵣ = 10k
 *  Rₙ = 47k
 *
 *  γ = 1 / β = 0.0002531…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = γ⋅ln(10/47) + 1/298.15 = 0.002962…
 */

#define HW4_NTC_GAMMA   0.00025316455696f
#define HW4_NTC_DELTA   0.00296226896087f

#define calcTempHW(adc)  calcTempNTC(adc, HW4_NTC_GAMMA, HW4_NTC_DELTA)

#define HW4_VOLTAGE_SCALE    0.00008056640625f
#define HW4_CURRENT_SCALE    32.2265625f

static float hw4VoltageScale = 0;
static float hw4CurrentScale = 0;
static float hw4CurrentOffset = 0;

static inline float calcVoltHW(uint16_t voltADC)
{
    return voltADC * hw4VoltageScale;
}

static inline float calcCurrHW(uint16_t currentADC)
{
    return (currentADC > hw4CurrentOffset) ?
        (currentADC - hw4CurrentOffset) * hw4CurrentScale : 0;
}

#define HW4_FRAME_NONE   0
#define HW4_FRAME_INFO   1
#define HW4_FRAME_DATA   2

static uint8_t processHW4TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte == 0x9B) {
            syncCount++;
        }
        else if (dataByte == 0xB9) {
            readBytes = 0;
        }
        else {
            frameSyncError();
        }
    }
    else if (readBytes == 13) {
        if (buffer[1] == 0x9B && buffer[4] == 0x01 && buffer[12] == 0xB9) {
            readBytes = 0;
            if (syncCount > 3)
                return HW4_FRAME_INFO;
        }
    }
    else if (readBytes == 19) {
        readBytes = 0;
        return HW4_FRAME_DATA;
    }

    return HW4_FRAME_NONE;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        const uint8_t frameType = processHW4TelemetryStream(serialRead(escSensorPort));

        if (frameType == HW4_FRAME_DATA) {
            if (buffer[4] < 4 && buffer[6] < 4 && buffer[11] < 0x10 &&
                buffer[13] < 0x10 && buffer[15] < 0x10 && buffer[17] < 0x10) {

                //uint32_t cnt = buffer[1] << 16 | buffer[2] << 8 | buffer[3];
                uint16_t thr = buffer[4] << 8 | buffer[5];
                uint16_t pwm = buffer[6] << 8 | buffer[7];
                uint32_t rpm = buffer[8] << 16 | buffer[9] << 8 | buffer[10];
                uint16_t Vadc = buffer[11] << 8 | buffer[12];
                uint16_t Iadc = buffer[13] << 8 | buffer[14];
                uint16_t Tadc = buffer[15] << 8 | buffer[16];
                uint16_t Cadc = buffer[17] << 8 | buffer[18];

                float voltage = calcVoltHW(Vadc);
                float current = calcCurrHW(Iadc);
                float tempFET = calcTempHW(Tadc);
                float tempCAP = calcTempHW(Cadc);

                // When throttle changes to zero, the last current reading is
                // repeated until the motor has totally stopped.
                if (pwm == 0) {
                    current = 0;
                }

                setConsumptionCurrent(current);

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = thr;
                escSensorData[0].pwm = pwm;
                escSensorData[0].voltage = lrintf(voltage * 1000);
                escSensorData[0].current = lrintf(current * 1000);
                escSensorData[0].temperature = lrintf(tempFET * 10);
                escSensorData[0].temperature2 = lrintf(tempCAP * 10);

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, lrintf(current * 100));

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, Tadc);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, Vadc);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, Iadc);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, thr);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
        else if (frameType == HW4_FRAME_INFO) {
            if (escSensorConfig()->hw4_voltage_gain) {
                hw4VoltageScale = HW4_VOLTAGE_SCALE * escSensorConfig()->hw4_voltage_gain;
            }
            else {
                if (buffer[5] && buffer[6])
                    hw4VoltageScale = (float)buffer[5] / (float)buffer[6] / 10;
                else
                    hw4VoltageScale = 0;
            }

            if (escSensorConfig()->hw4_current_gain) {
                hw4CurrentScale = HW4_CURRENT_SCALE / escSensorConfig()->hw4_current_gain;
                hw4CurrentOffset = escSensorConfig()->hw4_current_offset;
            }
            else {
                if (buffer[7] && buffer[8]) {
                    hw4CurrentScale = (float)buffer[7] / (float)buffer[8];
                    hw4CurrentOffset = (float)buffer[9] / hw4CurrentScale;
                }
                else {
                    hw4CurrentScale = 0;
                    hw4CurrentOffset = 0;
                }
            }
        }
    }

    // Update consumption on every cycle
    updateConsumption(currentTimeUs);

    // Maximum data frame spacing 400ms
    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * Hobbywing V5 Telemetry
 *
 *    - Serial protocol 115200,8N1
 *    - Frame rate running:50Hz idle:2.5Hz
 *    - Little-Endian fields
 *    - Frame length over data (23)
 *    - CRC16-MODBUS (poly 0x8005, init 0xffff)
 *    - Fault code bits:
 *         0:  Motor locked protection
 *         1:  Over-temp protection
 *         2:  Input throttle error at startup
 *         3:  Throttle signal lost
 *         4:  Over-current error
 *         5:  Low-voltage error
 *         6:  Input-voltage error
 *         7:  Motor connection error
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    0-5:      Sync header (0xFE 0x01 0x00 0x03 0x30 0x5C)
 *      6:      Data frame length (23)
 *    7-8:      Data type 0x06 0x00
 *      9:      Throttle value in %
 *  10-11:      Unknown
 *     12:      Fault code
 *  13-14:      RPM in 10rpm steps
 *  15-16:      Voltage in 0.1V
 *  17-18:      Current in 0.1A
 *     19:      ESC Temperature in °C
 *     20:      BEC Temperature in °C
 *     21:      Motor Temperature in °C
 *     22:      BEC Voltage in 0.1V
 *     23:      BEC Current in 0.1A
 *  24-29:      Unused 0xFF
 *  30-31:      CRC16 MODBUS
 *
 */

static uint16_t calculateCRC16_MODBUS(const uint8_t *ptr, size_t len)
{
    uint16_t crc = ~0;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }

    return crc;
}

static bool processHW5TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xFE)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x01)
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 0x00)
            frameSyncError();
    }
    else if (readBytes == 4) {
        if (dataByte != 0x03)
            frameSyncError();
    }
    else if (readBytes == 5) {
        if (dataByte != 0x30)
            frameSyncError();
    }
    else if (readBytes == 6) {
        if (dataByte != 0x5C)
            frameSyncError();
    }
    else if (readBytes == 7) {
        if (dataByte != 0x17)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        return true;
    }

    return false;
}

static void hw5SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW5TelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[31] << 8 | buffer[30];

            if (calculateCRC16_MODBUS(buffer, 30) == crc) {
                uint32_t rpm = buffer[14] << 8 | buffer[13];
                uint16_t power = buffer[9];
                uint16_t fault = buffer[12];
                uint16_t voltage = buffer[16] << 8 | buffer[15];
                uint16_t current = buffer[18] << 8 | buffer[17];
                uint16_t tempFET = buffer[19];
                uint16_t tempBEC = buffer[20];
                uint16_t voltBEC = buffer[22];
                uint16_t currBEC = buffer[23];

                // When throttle changes to zero, the last current reading is
                // repeated until the motor has totally stopped.
                if (power == 0) {
                    current = 0;
                }

                setConsumptionCurrent(current * 0.1f);

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].throttle = power * 10;
                escSensorData[0].pwm = power * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].temperature = tempFET * 10;
                escSensorData[0].temperature2 = tempBEC * 10;
                escSensorData[0].bec_voltage = voltBEC * 100;
                escSensorData[0].bec_current = currBEC * 100;
                escSensorData[0].status = fault;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tempFET);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
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

    // Update consumption on every cycle
    updateConsumption(currentTimeUs);

    // Maximum frame spacing 400ms
    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * Kontronik Telemetry V4 (23.11.2018)
 *
 *    - Serial protocol is 115200,8E1
 *    - Frame rate 100Hz
 *    - Little-Endian fields
 *    - CRC32
 *    - Error flags:
 *         0:  Undervoltage on the battery
 *         1:  Overvoltage on the battery
 *         2:  Overcurrent error
 *         3:  Overcurrent warning
 *         4:  Temperature warning
 *         5:  Temperature error
 *         6:  BEC under-voltage error
 *         7:  BEC over-voltage error
 *         8:  BEC over-current error
 *         9:  BEC temperature error
 *         10: Switch-off by rudder movement
 *         11: Capacity limit reached
 *         12: Operational error
 *         13: Operational warning
 *         14: Self-test error
 *         15: EEPROM error
 *         16: Watchdog error
 *         17: Programming is still permitted
 *         18: Battery limit reached
 *         19: Current limit reached
 *         20: ESC temperature limit reached
 *         21: BEC temperature limit reached
 *         22: ESC current limit reached
 *         23: Capacity limit reached
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    0-3:      Sync 0x4B 0x4F 0x44 0x4C "KODL"
 *    4-7:      RPM
 *    8-9:      Battery voltage in 10mV
 *  10-11:      Battery current in 0.1A
 *  12-13:      Motor current average in 0.1A
 *  14-15:      Motor current peak in 0.1A
 *  16-17:      Capacity in mAh
 *  18-19:      BEC current in mA
 *  20-21:      BEC Voltage n mV
 *  22-23:      PWM in us
 *     24:      Throttle % (-100..100)
 *     25:      PWM duty cycle %
 *     26:      FET temperature -128..127°C
 *     27:      BEC temperature -128..127°C
 *  28-31:      Error Flags
 *     32:      Operational condition
 *     33:      Timing 0..30
 *  34-35:      Reserved
 *  36-39:      CRC32
 *
 */
#define KON_FRAME_LENGTH                40
#define KON_FRAME_LENGTH_LEGACY         38
#define KON_CRC_LENGTH                  4

static uint8_t kontronikPacketLength    = 0;    // 40 or 38 byte
static uint8_t kontronikCrcExclude      = 0;    // 0 or 2

static uint32_t calculateCRC32(const uint8_t *ptr, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }

    return ~crc;
}

static uint32_t kontronikDecodeCRC(uint8_t index)
{
    return buffer[index + 3] << 24 | buffer[index + 2] << 16 | buffer[index + 1] << 8 | buffer[index];
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
    else if (readBytes == kontronikPacketLength) {
        readBytes = 0;
        return true;
    }
    else if (kontronikPacketLength == 0 && readBytes == KON_FRAME_LENGTH_LEGACY) {
        // auto detect legacy 38 byte packet...
        uint32_t crc = kontronikDecodeCRC(KON_FRAME_LENGTH_LEGACY - KON_CRC_LENGTH);
        if (calculateCRC32(buffer, KON_FRAME_LENGTH_LEGACY - 2 - KON_CRC_LENGTH) == crc) {
            // ...w/ 32 byte payload (2 bytes excluded)
            kontronikPacketLength = KON_FRAME_LENGTH_LEGACY;
            kontronikCrcExclude = 2;
            readBytes = 0;
            return true;
        }
        else if (calculateCRC32(buffer, KON_FRAME_LENGTH_LEGACY - KON_CRC_LENGTH) == crc) {
            // ...w/ 34 byte payload
            kontronikPacketLength = KON_FRAME_LENGTH_LEGACY;
            kontronikCrcExclude = 0;
            readBytes = 0;
            return true;
        }
    }
    else if (kontronikPacketLength == 0 && readBytes == KON_FRAME_LENGTH) {
        // auto detect 40 byte packet...
        uint32_t crc = kontronikDecodeCRC(KON_FRAME_LENGTH - KON_CRC_LENGTH);
        if (calculateCRC32(buffer, KON_FRAME_LENGTH - KON_CRC_LENGTH) == crc) {
            // ...w/ 36 byte payload
            kontronikPacketLength = KON_FRAME_LENGTH;
            kontronikCrcExclude = 0;
        }
        readBytes = 0;
        return true;
    }

    return false;
}

static void kontronikSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processKontronikTelemetryStream(serialRead(escSensorPort))) {
            uint32_t crc = kontronikDecodeCRC(kontronikPacketLength - KON_CRC_LENGTH);
            if (calculateCRC32(buffer, kontronikPacketLength - kontronikCrcExclude - KON_CRC_LENGTH) == crc) {
                uint32_t rpm = buffer[7] << 24 | buffer[6] << 16 | buffer[5] << 8 | buffer[4];
                int16_t  throttle = (int8_t)buffer[24];
                uint16_t pwm = buffer[23] << 8 | buffer[22];
                uint16_t voltage = buffer[9] << 8 | buffer[8];
                uint16_t current = buffer[11] << 8 | buffer[10];
                uint16_t capacity = buffer[17] << 8 | buffer[16];
                int16_t  tempFET = (int8_t)buffer[26];
                int16_t  tempBEC = (int8_t)buffer[27];
                uint16_t currBEC = buffer[19] << 8 | buffer[18];
                uint16_t voltBEC = buffer[21] << 8 | buffer[20];
                uint32_t status = buffer[31] << 24 | buffer[30] << 16 | buffer[29] << 8 | buffer[28];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = (throttle + 100) * 5;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = tempFET * 10;
                escSensorData[0].temperature2 = tempBEC * 10;
                escSensorData[0].bec_voltage = voltBEC;
                escSensorData[0].bec_current = currBEC;
                escSensorData[0].status = status;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

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

    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * OMP Hobby M4 Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame rate 20Hz
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *        12:  Battery voltage error
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      Start Flag (0xdd)
 *      1:      Protocol version (0x01)
 *      2:      Frame lenght (32)
 *    3-4:      Battery voltage in 0.1V
 *    5-6:      Battery current in 0.1V
 *      7:      Throttle %
 *    8-9:      RPM in 10rpm steps
 *     10:      ESC Temperature °C
 *     11:      Motor Temperature °C
 *     12:      PWM duty cycle %
 *  13-14:      Status Code
 *  15-16:      Capacity mAh
 *  17-31:      Unused / Zeros
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
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void ompSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processOMPTelemetryStream(serialRead(escSensorPort))) {
            // Make sure this is OMP M4 ESC
            if (buffer[1] == 0x01 && buffer[2] == 0x20 && buffer[11] == 0 && buffer[18] == 0 && buffer[20] == 0) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t throttle = buffer[7];
                uint16_t pwm = buffer[12];
                uint16_t temp = buffer[10];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].throttle = throttle * 10;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = temp * 10;
                escSensorData[0].status = status;

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

    // Maximum frame spacing 50ms, sync after 3 frames
    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * ZTW Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame rate 20Hz
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Checksum (unknown)
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *         9:  UART throttle error
 *        10:  UART throttle lost
 *        11:  CAN throttle lost
 *        12:  Battery voltage error
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      Start Flag (0xdd)
 *      1:      Protocol version (0x01)
 *      2:      Frame lenght (32)
 *    3-4:      Battery voltage in 0.1V
 *    5-6:      Battery current in 0.1V
 *      7:      Throttle %
 *    8-9:      RPM in 10rpm steps
 *     10:      ESC Temperature °C
 *     11:      Motor Temperature °C
 *     12:      PWM duty cycle %
 *  13-14:      Status Code
 *  15-16:      Capacity mAh
 *     17:      Serial Throttle input (unused)
 *     18:      CAN Throttle input (unused)
 *     19:      BEC Voltage
 *  20-29:      Unused
 *  30-31:      Checksum
 *
 */

static bool processZTWTelemetryStream(uint8_t dataByte)
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
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void ztwSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processZTWTelemetryStream(serialRead(escSensorPort))) {
            if (buffer[1] == 0x01 && buffer[2] == 0x20) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t temp = buffer[10];
                uint16_t throttle = buffer[7];
                uint16_t power = buffer[12];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];
                uint16_t voltBEC = buffer[19];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].throttle = throttle * 10;
                escSensorData[0].pwm = power * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = temp * 10;
                escSensorData[0].bec_voltage = voltBEC * 1000;
                escSensorData[0].status = status;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
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

    // Maximum frame spacing 50ms, sync after 3 frames
    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * Advanced Power Drives UART Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Little-Endian fields
 *    - Fletcher16 checksum
 *    - Status Flags:
 *         0:  Motor started
 *         1:  Motor saturation
 *         2:  Over-temperature
 *         3:  Over-voltage
 *         4:  Under-voltage
 *         5:  Startup error
 *         6:  Unused
 *         7:  Unused
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    0-1:      Sync 0xFFFF
 *    2-3:      Voltage in 10mV steps
 *    4-5:      Temperature ADC
 *    6-7:      Current in 80mA steps
 *    8-9:      Unused
 *  10-13:      ERPM
 *  14-15:      Throttle in 0.1%
 *  16-17:      PWM duty cycle in 0.1%
 *     18:      Status flags
 *     19:      Unused
 *  20-21:      Checksum
 *
 * Temp sensor design:
 * ―――――――――――――――――――
 *  β  = 3455
 *  Rᵣ = 10k
 *  Rₙ = 10k
 *  Tₙ = 25°C
 *
 *  γ = 1 / β = 0.0002894…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = 1/298.15 = 0.0033540…
 *
 */

#define APD_NTC_GAMMA  0.0002894356005f
#define APD_NTC_DELTA  0.0033540164346f

#define calcTempAPD(value)   calcTempNTC(value, APD_NTC_GAMMA, APD_NTC_DELTA)

static uint16_t calculateFletcher16(const uint8_t *ptr, size_t len)
{
    uint16_t s0 = 0;
    uint16_t s1 = 0;

    while (len--) {
        s0 = (s0 + *ptr++) % 255;
        s1 = (s1 + s0) % 255;
    }
    return s1 << 8 | s0;
}

static bool processAPDTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xFF)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0xFF)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 22) {
        readBytes = 0;
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void apdSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processAPDTelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[21] << 8 | buffer[20];

            if (calculateFletcher16(buffer + 2, 18) == crc) {
                uint16_t rpm = buffer[13] << 24 | buffer[12] << 16 | buffer[11] << 8 | buffer[10];
                uint16_t tadc = buffer[3] << 8 | buffer[2];
                uint16_t throttle = buffer[15] << 8 | buffer[14];
                uint16_t power = buffer[17] << 8 | buffer[16];
                uint16_t voltage = buffer[1] << 8 | buffer[0];
                uint16_t current = buffer[5] << 8 | buffer[4];
                uint16_t status = buffer[18];

                float temp = calcTempAPD(tadc);

                setConsumptionCurrent(current * 0.08f);

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = throttle;
                escSensorData[0].pwm = power;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 80;
                escSensorData[0].temperature = lrintf(temp * 10);
                escSensorData[0].status = status;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(temp * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 8);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tadc);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
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

    // Update consumption on every cycle
    updateConsumption(currentTimeUs);

    checkFrameTimeout(currentTimeUs, 500000);
}


/*
 * RRFSM
 *
 */
// > 0: frame accepted, count as synced (may not be complete), continue accepting
// < 0: frame rejected, will be logged as sync error
//   0: continue accepting
typedef int8_t (*rrfsmAcceptCallbackPtr)(uint16_t c);

typedef bool (*rrfsmStartCallbackPtr)(timeMs_t currentTimeMs);  // return true to continue w/ default initialization (if in doubt return true)
typedef bool (*rrfsmDecodeCallbackPtr)(timeMs_t currentTimeMs); // return true if frame was decoded successfully
typedef bool (*rrfsmCrankCallbackPtr)(timeMs_t currentTimeMs);  // return true to continue w/ default loop (advanced, if in doubt return true)

static rrfsmAcceptCallbackPtr rrfsmAccept = NULL;
static rrfsmStartCallbackPtr rrfsmStart = NULL;
static rrfsmDecodeCallbackPtr rrfsmDecode = NULL;
static rrfsmCrankCallbackPtr rrfsmCrank = NULL;

static uint16_t rrfsmBootDelayMs = 0;
static uint8_t rrfsmMinFrameLength = 0;

static uint32_t rrfsmFrameTimestamp = 0;
static volatile uint8_t rrfsmFrameLength = 0;
static uint16_t rrfsmFramePeriod = 0;
static uint16_t rrfsmFrameTimeout = 0;

static void rrfsmFrameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}

static void rrfsmStartFrame(timeMs_t currentTimeMs)
{
    readBytes = 0;
    rrfsmFrameLength = rrfsmMinFrameLength;
    rrfsmFrameTimestamp = currentTimeMs;
}

static void rrfsmStartFrameAndSendPendingReq(timeMs_t currentTimeMs)
{
    rrfsmStartFrame(currentTimeMs);

    rrfsmFramePeriod = 0;
    readIngoreBytes = reqLength;
    if (reqLength != 0)
        serialWriteBuf(escSensorPort, reqbuffer, reqLength);
}

static void rrfsmInvalidateReq(void)
{
    reqbuffer[0] = 0x00;     // no req
    reqLength = 0;
    rrfsmFrameTimeout = 0;
}

static FAST_CODE void rrfsmDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    // don't listen to self
    if (readIngoreBytes > 0) {
        readIngoreBytes--;
        return;
    }

    totalByteCount++;

    if (readBytes >= TELEMETRY_BUFFER_SIZE) {
        // avoid buffer overrun
        rrfsmFrameSyncError();
    }
    else if (readBytes < rrfsmFrameLength) {
        buffer[readBytes++] = c;

        int8_t accepted = (rrfsmAccept != NULL ) ? rrfsmAccept(c) : -1;
        if (accepted > 0) {
            // frame accepted (may not be complete)
            syncCount++;
        }
        else if (accepted < 0) {
            // frame rejected
            rrfsmFrameSyncError();
        }
    }
}

static void rrfsmSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    // wait before initializing
    if (currentTimeMs < rrfsmBootDelayMs)
        return;

    // request first log record or just listen if in e.g. UNC mode
    if (rrfsmFrameTimestamp == 0) {
        if (rrfsmStart == NULL || rrfsmStart(currentTimeMs))
            rrfsmStartFrame(currentTimeMs);
        return;
    }

    // frame period in effect? request next frame if elapsed
    if (rrfsmFramePeriod != 0) {
        if (currentTimeMs > rrfsmFrameTimestamp + rrfsmFramePeriod) {
            rrfsmStartFrameAndSendPendingReq(currentTimeMs);
        }
        return;
    }

    // timeout waiting for frame? log error, request again
    if (rrfsmFrameTimeout != 0 && currentTimeMs > rrfsmFrameTimestamp + rrfsmFrameTimeout) {
        increaseDataAge(0);
        totalTimeoutCount++;
        rrfsmStartFrameAndSendPendingReq(currentTimeMs);
        return;
    }

    // custom execution (advanced)
    if (rrfsmCrank != NULL && !rrfsmCrank(currentTimeMs)) {
        return;
    }

    // frame incomplete? check later
    if (readBytes < rrfsmFrameLength) {
        return;
    }

    // frame complate, process, prepare for next frame
    if (rrfsmDecode == NULL || rrfsmDecode(currentTimeMs)) {
        // good frame, log, response handler will have prep'ed next request
        totalFrameCount++;
    }
    else {
        // bad frame, log error, retry
        increaseDataAge(0);
        totalCrcErrorCount++;
    }
    rrfsmStartFrame(currentTimeMs);
}


/*
 * Scorpion Telemetry
 *    - Serial protocol is 38400,8N1
 *    - Frame rate running:10Hz idle:1Hz
 *    - Little-Endian fields
 *    - CRC16-CCITT
 *    - Error Code bits:
 *         0:  N/A
 *         1:  BEC voltage error
 *         2:  Temperature error
 *         3:  Consumption error
 *         4:  Input voltage error
 *         5:  Current error
 *         6:  N/A
 *         7:  Throttle error
 *
 * Request Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      Req: MSN - read = 0x5, write = 0xD, LSN - region
 *  1,2,3:      Address: Starting address in region to access
 *      4:      Length: Data length to read/write, does not include Data crc
 *              Requested data length in bytes should not be more than 32 bytes and should  be a multiple of 2 bytes
 *      5:      CRC: Header CRC. 0 value means that crc is not used. And no additional bytes for data crc
 * Regions
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 * 0 – System region, read only
 * 1 – Status region, read only
 * 2 – Control region, stay away
 * 3 – Settings region, read/write access
 * 4 – FW region, write only, encrypted. Stay away
 * 5 - Log region, read only
 * 6 – User data

 *
 * Telemetry Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    0-5:      Header
 *  +0-+2:      Timestamp ms
 *     +3:      Throttle in 0.5%
 *  +4-+5:      Current in 0.1A
 *  +6-+7:      Voltage in 0.1V
 *  +8-+9:      Consumption in mAh
 *    +10:      Temperature in °C
 *    +11:      PWM duty cycle in 0.5%
 *    +12:      BEC voltage in 0.1V
 *+13-+14:      RPM in 5rpm steps
 *    +15:      Error code
 *+16-+17:      CRC16 CCITT
 *
 */
#define TRIB_REQ_WRITE                  0x80                // request write bit
#define TRIB_REQ_SIG                    0x50                // request signature bits
#define TRIB_BOOT_DELAY                 10000               // 10 seconds quiet time before expecting first frame
#define TRIB_HEADER_LENGTH              6                   // assume header only until actual length of current frame known
#define TRIB_UNC_HEADER_LENGTH          4                   // UNC packet header length
#define TRIB_FRAME_PERIOD               100                 // aim for approx. 20Hz (note UNC is 10Hz, may have to reduce this)
#define TRIB_REQ_BOOT_DELAY             4000                // 4 seconds quiet time before ESC requests
#define TRIB_REQ_READ_TIMEOUT           200                 // Response timeout for read requests
#define TRIB_REQ_WRITE_TIMEOUT          1200                // Response timeout for write requests
#define TRIB_RESP_FRAME_TIMEOUT         200                 // Response timeout depends on ESC business but no more than 200ms
                                                            // Host must wait ESC response or timeout before sending new packet
#define TRIB_UNC_FRAME_TIMEOUT          1200                // Timeout for UNC packets
#define TRIB_PARAM_FRAME_PERIOD         2                   // asap
#define TRIB_PARAM_READ_TIMEOUT         200                 // usually less than 200Us
#define TRIB_PARAM_WRITE_TIMEOUT        1200                // usually less than 1200Us
#define TRIB_PARAM_SIG                  0x53                // parameter payload signature for this ESC
#define TRIB_PARAM_IQ22_ADDR            0x18                // address of param range w/ IQ22 params
#define TRIB_PARAM_CAP_RESET            PARAM_HEADER_USER   // reset ESC capable
#define TRIB_PARAM_CMD_RESET            PARAM_HEADER_USER   // reset ESC command

#define TRIB_PAL_SYS_MASK               0x8000              // param address/length range system bit mask
#define TRIB_PAL_ADDR_MASK              0x7F00              // param address/length range address mask
#define TRIB_PAL_LEN_MASK               0x00FF              // param address/length range length mask

#define TRIB_PKT_READ_SYSTEM            0x50                // read system region packet
#define TRIB_PKT_READ_STATUS            0x51                // read status region packet
#define TRIB_PKT_READ_SETTINGS          0x53                // read settings region packet
#define TRIB_PKT_WRITE_SETTINGS         0xD3                // write settings region packet
#define TRIB_PKT_UNSOLICITED            0x55                // unsolicited packet

typedef enum {
    TRIB_UNCSETUP_INACTIVE = 0,
    TRIB_UNCSETUP_PARAMSREADY,
    TRIB_UNCSETUP_ABORTUNC,
    TRIB_UNCSETUP_ACTIVE,
    TRIB_UNCSETUP_WAIT,
} tribUncSetup_e;

static tribUncSetup_e tribUncSetup = TRIB_UNCSETUP_INACTIVE;

// param ranges - hibyte=addr (system region if 0x80 set or setting region otherwise), lobyte=length
static uint16_t tribParamAddrLen[] = { 0x0020, 0x1008, 0x230E, 0x8204, 0x8502, 0x1406, 0x1808, 0x3408 };

static uint16_t tribInvalidParams = 0;   // bit per param address range not yet received from ESC
static uint16_t tribDirtyParams = 0;     // bit per param address that needs to be written to ESC
static bool tribResetEsc = false;

static uint16_t calculateCRC16_CCITT(const uint8_t *ptr, size_t len)
{
    uint16_t crc = 0;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }

    return crc;
}

static bool tribParamCommit(uint8_t cmd)
{
    if (cmd == 0) {
        // save page
        // find dirty params, settings only
        uint8_t offset = 0;
        for (uint8_t i = 0; i < ARRAYLEN(tribParamAddrLen); i++) {
            const uint16_t *pal = tribParamAddrLen + i;
            const uint8_t len = *pal & TRIB_PAL_LEN_MASK;
            if ((*pal & TRIB_PAL_SYS_MASK) == 0) {
                // schedule writes for dirty address ranges
                if (memcmp(paramPayload + offset, paramUpdPayload + offset, len) != 0) {
                    // set dirty bit
                    tribDirtyParams |= (1U << i);
                    // invalidate param
                    tribInvalidParams |= (1 << i);
                    // invalidate param payload - will be available again when all params again cached
                    paramPayloadLength = 0;
                }
            }
            offset += len;
        }
        return true;
    }
    else if (cmd == TRIB_PARAM_CMD_RESET) {
        // reset ESC
        tribResetEsc = true;
        return true;
    }
    else {
        // invalid command
        return false;
    }
}

static void tribBuildReq(uint8_t req, uint8_t addr, void *src, uint8_t len, uint16_t framePeriod, uint16_t frameTimeout)
{
    reqbuffer[0] = req;     // req
    reqbuffer[1] = 0;       // addr
    reqbuffer[2] = 0;
    reqbuffer[3] = addr;
    reqbuffer[4] = len;     // length
    reqbuffer[5] = 0;
    reqLength = TRIB_HEADER_LENGTH;
    if (src != NULL) {
        memcpy(reqbuffer + TRIB_HEADER_LENGTH, src, len);
        reqLength += len;
    }
    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;
}

static void tribInvalidateParams(void)
{
    tribInvalidParams = ~(~1U << (ARRAYLEN(tribParamAddrLen) - 1));
}

static uint8_t tribCalcParamBufferLength()
{
    uint8_t len = 0;
    for (uint8_t j = 0; j < ARRAYLEN(tribParamAddrLen); j++)
        len += (tribParamAddrLen[j] & TRIB_PAL_LEN_MASK);
    return len;
}

static bool tribBuildNextParamReq(void)
{
    // pending reset request...
    if (tribResetEsc) {
        tribResetEsc = false;

        tribUncSetup = TRIB_UNCSETUP_INACTIVE;
        tribInvalidParams = 0;
        tribDirtyParams = 0;
        paramMspActive = false;

        uint16_t reset = 0;
        tribBuildReq(0xD0, 0x00, &reset, 2, TRIB_PARAM_FRAME_PERIOD, TRIB_PARAM_WRITE_TIMEOUT);
        return true;
    }

    // ...or pending write request...
    uint8_t offset = 0;
    for (uint16_t *pal = tribParamAddrLen, dirtybits = tribDirtyParams; dirtybits != 0; pal++, dirtybits >>= 1) {
        const uint8_t addr = (*pal & TRIB_PAL_ADDR_MASK) >> 8;
        const uint8_t len = *pal & TRIB_PAL_LEN_MASK;
        if ((dirtybits & 0x01) != 0) {
            void *payload;
            uint32_t iq22Payload[2];
            if (addr == TRIB_PARAM_IQ22_ADDR) {
                // convert scaled uint32 -> IQ22
                const uint32_t q22 = 1 << 22;
                uint32_t *pw = iq22Payload;
                uint32_t *pp = (uint32_t*)(paramUpdPayload + offset);
                *pw++ = (((float)*pp++) / 100 * q22);
                *pw = (((float)*pp) / 100000 * q22);
                payload = iq22Payload;
            }
            else {
                payload = paramUpdPayload + offset;
            }
            tribBuildReq(TRIB_PKT_WRITE_SETTINGS, addr, payload, len, TRIB_PARAM_FRAME_PERIOD, TRIB_PARAM_WRITE_TIMEOUT);
            return true;
        }
        offset += len;
    }

    // ...or schedule pending read request if no pending writes...
    for (uint16_t *pal = tribParamAddrLen, invalidbits = tribInvalidParams; invalidbits != 0; pal++, invalidbits >>= 1) {
        if ((invalidbits & 0x01) != 0) {
            uint8_t req = (*pal & TRIB_PAL_SYS_MASK) != 0 ? TRIB_PKT_READ_SYSTEM : TRIB_PKT_READ_SETTINGS;
            uint8_t addr = (*pal & TRIB_PAL_ADDR_MASK) >> 8;
            uint8_t len = *pal & TRIB_PAL_LEN_MASK;
            tribBuildReq(req, addr, NULL, len, TRIB_PARAM_FRAME_PERIOD, TRIB_PARAM_READ_TIMEOUT);
            return true;
        }
    }

    // ...or nothing pending
    return false;
}

static bool tribDecodeReadParamResp(uint8_t addr)
{
    uint8_t offset = 0;
    for (uint8_t i = 0; i < ARRAYLEN(tribParamAddrLen); i++) {
        const uint16_t *pal = tribParamAddrLen + i;
        const uint8_t len = *pal & TRIB_PAL_LEN_MASK;
        if ((*pal >> 8) == addr) {
            // cache params by addr
            if (addr == TRIB_PARAM_IQ22_ADDR) {
                // convert IQ22 -> scaled uint32
                const uint32_t q22 = 1 << 22;
                uint32_t *pr = (uint32_t*)(buffer + TRIB_HEADER_LENGTH);
                uint32_t *pp = (uint32_t*)(paramPayload + offset);
                *pp++ = roundf(((float)*pr++) / q22 * 100);
                *pp = roundf(((float)*pr) / q22 * 100000);
            }
            else {
                memcpy(paramPayload + offset, buffer + TRIB_HEADER_LENGTH, len);
            }
            // clear invalid bit
            tribInvalidParams &= ~(1 << i);

            // make param payload available if all params cached
            if (tribInvalidParams == 0 && paramPayloadLength == 0)
                tribUncSetup = TRIB_UNCSETUP_PARAMSREADY;
            return true;
        }
        offset += len;
    }
    return false;
}

static bool tribDecodeWriteParamResp(uint8_t addr)
{
    for (uint8_t i = 0; i < ARRAYLEN(tribParamAddrLen); i++) {
        const uint16_t *pal = tribParamAddrLen + i;
        if ((*pal >> 8) == addr) {
            // clear dirty bit and set invalid bit force read
            const uint16_t addrbit = (1 << i);
            tribDirtyParams &= ~addrbit;
            tribInvalidParams |= addrbit;
            return true;
        }
    }
    return false;
}

static bool tribValidateResponseHeader(void)
{
    // req and resp headers should match except for len ([4]) which may differ
    for (int i = 0; i < TRIB_HEADER_LENGTH; i++) {
        if (i == 4)
            continue;
        if (buffer[i] != reqbuffer[i])
            return false;
    }
    return true;
}

static bool tribDecodeReadSettingResp(uint8_t sysbit)
{
    // validate header
    if (!tribValidateResponseHeader())
        return false;

    const uint8_t addr = buffer[3];
    if (!tribDecodeReadParamResp(addr | sysbit) || !tribBuildNextParamReq())
        rrfsmInvalidateReq();

    return true;
}

static bool tribDecodeWriteSettingResp(void)
{
    // validate header
    if (!tribValidateResponseHeader())
        return false;

    const uint8_t addr = buffer[3];
    if (!tribDecodeWriteParamResp(addr) || !tribBuildNextParamReq())
        rrfsmInvalidateReq();

    return true;
}

static bool tribDecodeLogRecord(uint8_t hl)
{
    // payload: 16 byte (Log_rec_t)
    const uint16_t rpm = buffer[hl + 14] << 8 | buffer[hl + 13];
    const uint16_t temp = buffer[hl + 10];
    const uint16_t power = buffer[hl + 11];
    const uint16_t voltage = buffer[hl + 7] << 8 | buffer[hl + 6];
    const uint16_t current = buffer[hl + 5] << 8 | buffer[hl + 4];
    const uint16_t capacity = buffer[hl + 9] << 8 | buffer[hl + 8];
    const uint16_t status = buffer[hl + 15];
    const uint16_t voltBEC = buffer[hl + 12];

    escSensorData[0].age = 0;
    escSensorData[0].erpm = rpm * 5;
    escSensorData[0].pwm = power * 5;
    escSensorData[0].voltage = voltage * 100;
    escSensorData[0].current = current * 100;
    escSensorData[0].consumption = capacity;
    escSensorData[0].temperature = temp * 10;
    escSensorData[0].bec_voltage = voltBEC * 100;

    DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 5);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 100);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 100);

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

    return true;
}

static bool tribDecodeReadStatusResp(void)
{
    // validate header (no CRC)
    if (!tribValidateResponseHeader())
        return false;

    const uint8_t addr = buffer[3];
    if (tribUncSetup == TRIB_UNCSETUP_ABORTUNC && addr == 0) {
        tribUncSetup = TRIB_UNCSETUP_ACTIVE;
        paramPayloadLength = tribCalcParamBufferLength();
        rrfsmInvalidateReq();
        return true;
    }
    return false;
}

static bool tribDecodeUNCFrame(void)
{
    // validate CRC, decode as 4 byte header + 16 byte (Log_rec_t) payload
    const uint16_t crc = buffer[rrfsmFrameLength - 1] << 8 | buffer[rrfsmFrameLength - 2];
    if (calculateCRC16_CCITT(buffer, 20) != crc || !tribDecodeLogRecord(TRIB_UNC_HEADER_LENGTH))
        return false;

    rrfsmFrameTimeout = TRIB_UNC_FRAME_TIMEOUT;
    return true;
}

static bool tribDecode(timeMs_t currentTimeMs)
{
    UNUSED(currentTimeMs);

    const uint8_t req = buffer[0];
    switch (req) {
        case TRIB_PKT_READ_STATUS:
            return tribDecodeReadStatusResp();
        case TRIB_PKT_READ_SYSTEM:
            return tribDecodeReadSettingResp(0x80);
        case TRIB_PKT_READ_SETTINGS:
            return tribDecodeReadSettingResp(0x00);
        case TRIB_PKT_WRITE_SETTINGS:
            return tribDecodeWriteSettingResp();
        case TRIB_PKT_UNSOLICITED:
            return tribDecodeUNCFrame();
        default:
            return false;
    }
}

static bool tribCrankUncSetup(timeMs_t currentTimeMs)
{
    switch(tribUncSetup) {
        case TRIB_UNCSETUP_INACTIVE:
        case TRIB_UNCSETUP_ABORTUNC:
            // unexpected but required to keep compiler happy
            break;
        case TRIB_UNCSETUP_PARAMSREADY:
            if (paramMspActive && currentTimeMs < rrfsmFrameTimestamp + 4000) {
                // try to abort UNC mode
                tribUncSetup = TRIB_UNCSETUP_ABORTUNC;
                tribBuildReq(TRIB_PKT_READ_STATUS, 0, NULL, 0x10, TRIB_FRAME_PERIOD, TRIB_RESP_FRAME_TIMEOUT);
            }
            break;
        case TRIB_UNCSETUP_ACTIVE:
            if (tribBuildNextParamReq()) {
                tribUncSetup = TRIB_UNCSETUP_WAIT;
            }
            break;
        case TRIB_UNCSETUP_WAIT:
            if (tribInvalidParams == 0 && tribDirtyParams == 0) {
                tribUncSetup = TRIB_UNCSETUP_ACTIVE;
                rrfsmInvalidateReq();
            }
            break;
    }
    return true;
}

static bool tribStart(timeMs_t currentTimeMs)
{
    UNUSED(currentTimeMs);

    tribBuildNextParamReq();
    return true;
}

static int8_t tribAccept(uint16_t c)
{
    if (readBytes == 1) {
        // req / singature
        if ((c & TRIB_REQ_SIG) != TRIB_REQ_SIG)
            return -1;
    }
    else if (buffer[0] == TRIB_PKT_UNSOLICITED && readBytes == 3) {
        // UNC length
        if (c > TELEMETRY_BUFFER_SIZE) {
            // protect against buffer overflow
            return -1;
        }
        else {
            // new frame length for this frame (header + data)
            rrfsmFrameLength = c;
            return 1;
        }
    }
    else if (buffer[0] != TRIB_PKT_UNSOLICITED && readBytes == 5) {
        // STD length
        if (c > TELEMETRY_BUFFER_SIZE) {
            // protect against buffer overflow
            return -1;
        }
        else if((c & 0x01) || c > 32) {
            // invalid length (must not be more than 32 bytes and should be a multiple of 2 bytes)
            return -1;
        }
        else {
            // new frame length for this frame - read = (header + data), write = (header)
            rrfsmFrameLength = (buffer[0] & TRIB_REQ_WRITE) == 0 ? TRIB_HEADER_LENGTH + c : TRIB_HEADER_LENGTH;
            return 1;
        }
    }
    return 0;
}

static serialReceiveCallbackPtr tribSensorInit(bool bidirectional)
{
    rrfsmMinFrameLength = TRIB_HEADER_LENGTH;
    rrfsmAccept = tribAccept;
    rrfsmDecode = tribDecode;

    if (bidirectional) {
        // request/response telemetry
        rrfsmBootDelayMs = TRIB_REQ_BOOT_DELAY;
        rrfsmStart = tribStart;
    
        // enable ESC parameter reads and writes, reset
        paramSig = TRIB_PARAM_SIG;
        paramVer = 0 | TRIB_PARAM_CAP_RESET;
        paramCommit = tribParamCommit;
        tribInvalidateParams();
        
        // enable UNC setup FSM 
        rrfsmCrank = tribCrankUncSetup;
    }
    else {
        // telemetry data only
        rrfsmBootDelayMs = TRIB_BOOT_DELAY;
        rrfsmFrameTimeout = TRIB_UNC_FRAME_TIMEOUT;
    }

    return rrfsmDataReceive;
}


/*
 * OpenYGE Telemetry
 *
 *     - Serial protocol is 115200,8N1
 *     - Little-Endian byte order
 *     - Frame rate 20Hz
 *     - Thanks Fabian!
 *
 * Motor states (4 LSB of status1)
 * ―――――――――――――――――――――――――――――――――――――――――――――――
 *      STATE_DISARMED              = 0x00,     // Motor stopped
 *      STATE_POWER_CUT             = 0x01,     // Power cut maybe Overvoltage
 *      STATE_FAST_START            = 0x02,     // "Bailout" State
 *      STATE_RESERVED2             = 0x03,     // reserved
 *      STATE_ALIGN_FOR_POS         = 0x04,     // "Positioning"
 *      STATE_RESERVED3             = 0x05,     // reserved
 *      STATE_BRAKEING_NORM_FINI    = 0x06,
 *      STATE_BRAKEING_SYNC_FINI    = 0x07,
 *      STATE_STARTING              = 0x08,     // "Starting"
 *      STATE_BRAKEING_NORM         = 0x09,
 *      STATE_BRAKEING_SYNC         = 0x0A,
 *      STATE_RESERVED4             = 0x0B,     // reserved
 *      STATE_WINDMILLING           = 0x0C,     // still rotating no power drive can be named "Idle"
 *      STATE_RESERVED5             = 0x0D,
 *      STATE_RUNNING_NORM          = 0x0E,     // normal "Running"
 *      STATE_RESERVED6             = 0x0F,
 *
 * Warning/error codes (4 MSB of status1)
 * ―――――――――――――――――――――――――――――――――――――――――――――――
 *      WARN_DEVICE_MASK            = 0xC0       // device ID bit mask (note WARN_SETPOINT_NOISE = 0xC0)
 *      WARN_DEVICE_ESC             = 0x00       // warning indicators are for ESC
 *      WARN_DEVICE_BEC             = 0x80       // warning indicators are for BEC
 *
 *      WARN_OK                     = 0x00       // Overvoltage if Motor Status == STATE_POWER_CUT
 *      WARN_UNDERVOLTAGE           = 0x10       // Fail if Motor Status < STATE_STARTING
 *      WARN_OVERTEMP               = 0x20       // Fail if Motor Status == STATE_POWER_CUT
 *      WARN_OVERAMP                = 0x40       // Fail if Motor Status == STATE_POWER_CUT
 *      WARN_SETPOINT_NOISE         = 0xC0       // note this is special case (can never have OVERAMP w/ BEC hence reuse)
 */

#define OPENYGE_SYNC                        0xA5                // sync
#define OPENYGE_VERSION                     3                   // protocol version
#define OPENYGE_HEADER_LENGTH               6                   // header length
#define OPENYGE_HEADER_LENGTH_LEGACY        4                   // legacy (pre-v3) header length
#define OPENYGE_CRC_LENGTH                  2                   // CRC length
#define OPENYGE_BOOT_DELAY                  5000                // 5 seconds
#define OPENYGE_MIN_FRAME_LENGTH            OPENYGE_HEADER_LENGTH_LEGACY + OPENYGE_CRC_LENGTH   // assume minimum frame (legacy header + CRC) until actual length of current frame known
#define OPENYGE_AUTO_INITIAL_FRAME_TIMEOUT  900                 // intially ~800ms w/ progressive decreasing frame-period...
#define OPENYGE_AUTO_MIN_FRAME_TIMEOUT      60U                 // ...to final ~50ms (no less)

#define OPENYGE_FRAME_PERIOD_INIT           20                  // delay before sending first master request after v3+ auto telemetry frame seen (possibly last chained)
#define OPENYGE_FRAME_PERIOD                38                  // aim for approx. 50ms/20Hz
#define OPENYGE_PARAM_FRAME_PERIOD          4                   // TBD ASAP?
#define OPENYGE_REQ_READ_TIMEOUT            200                 // Response timeout for read requests   TBD: confirm
#define OPENYGE_REQ_WRITE_TIMEOUT           400                 // Response timeout for write requests  TBD: confirm

#define OPENYGE_PARAM_SIG                   0xA5                // parameter payload signature for this ESC
#define OPENYGE_PARAM_CACHE_SIZE_MAX        64                  // limited by use of uint64_t as bit array for oygeCachedParams

#define OPENYGE_FTYPE_TELE_AUTO             0x00                // auto telemetry frame
#define OPENYGE_FTYPE_TELE_RESP             0x02                // telemetry response
#define OPENYGE_FTYPE_TELE_REQ              0x03                // telemetry request
#define OPENYGE_FTYPE_WRITE_PARAM_RESP      0x04                // write param response
#define OPENYGE_FTYPE_WRITE_PARAM_REQ       0x05                // write param request

#define OPENYGE_DEV_MASTER                  0x80                // device address master role bit

#define OPENYGE_TEMP_OFFSET                 40

typedef struct {
    uint8_t  sync;                      // sync 0xA5
    uint8_t  version;                   // version
    uint8_t  frame_type;                // high bit 0x00 for read, 0x80 for write, e.g 0x00 - telemetry frame data, 0x81 - update parameter
    uint8_t  frame_length;              // frame length
    uint8_t  seq;                       // count for packet-control/assignment master counts, slave(ESC) sends this value back
    uint8_t  device;                    // ESC address, 0x80 for master and 7 bits for ESC address 0x01…0x7F where 0x00 might mean all ESCs
} OpenYGEHeader_t;

typedef struct {
    uint8_t  reserved;                  // reserved
    uint8_t  temperature;               // C degrees+40 (-40..215°C)
    uint16_t voltage;                   // 0.01V
    uint16_t current;                   // 0.01A
    uint16_t consumption;               // mAh
    uint16_t rpm;                       // 0.1erpm
    int8_t   pwm;                       // %
    int8_t   throttle;                  // %
    uint16_t bec_voltage;               // 0.001V
    uint16_t bec_current;               // 0.001A
    uint8_t  bec_temp;                  // C degrees
    uint8_t  status1;                   // see documentation
    uint8_t  cap_temp;                  // C degrees+40 (-40..215°C)
    uint8_t  aux_temp;                  // C degrees+40 (-40..215°C)
    uint8_t  status2;                   // Debug/Reserved
    uint8_t  reserved1;                 // Debug/Reserved maybe consumption high-byte for more than 65Ah(industrial)
    uint16_t pidx;                      // maybe future use
    uint16_t pdata;                     // maybe future use
} OpenYGETelemetryFrame_t;

typedef struct {
    uint16_t index;                     // parameter addr
    uint16_t param;                     // parameter value
} OpenYGEControlFrame_t;

static uint16_t oygeAutoFrameTimeout = OPENYGE_AUTO_INITIAL_FRAME_TIMEOUT;
static uint64_t oygeCachedParams = 0;
static uint64_t oygeDirtyParams = 0;

static uint16_t oygeCalculateCRC16_CCITT(const uint8_t *ptr, size_t len)
{
    uint16_t crc = 0;

    for (uint16_t j = 0; j < len; j++)
    {
        crc = crc ^ ptr[j] << 8;
        for (uint16_t i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        }
    }

    return crc;
}

static bool oygeParamCommit(uint8_t cmd)
{
    if (cmd == 0) {
        // save page
        // find dirty params, skip para[0] (parameter count)
        uint16_t *ygeParams = (uint16_t*)paramPayload;
        const uint16_t *ygeUpdParams = (uint16_t*)paramUpdPayload;
        if (ygeParams[0] != ygeUpdParams[0])
            return false;

        const uint16_t ygeParamCount = ygeParams[0];
        for (uint8_t idx = 1; idx < ygeParamCount; idx++) {
            // schedule writes for dirty params
            if (ygeParams[idx] != ygeUpdParams[idx]) {
                // set dirty bit
                oygeDirtyParams |= (1ULL << idx);
                // invalidate param
                oygeCachedParams &= ~(1ULL << idx);
                // invalidate param payload - will be available again when all params again cached
                paramPayloadLength = 0;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

static void oygeCacheParam(uint8_t pidx, uint16_t pdata)
{
    const uint8_t maxParams = PARAM_BUFFER_SIZE / 2;
    if (pidx >= maxParams || pidx >= OPENYGE_PARAM_CACHE_SIZE_MAX)
        return;

    // don't accept params until pending writes cleared
    if (oygeDirtyParams != 0)
        return;

    uint16_t *ygeParams = (uint16_t*)paramPayload;
    ygeParams[pidx] = pdata;
    oygeCachedParams |= (1ULL << pidx);

    // skip if count already known (parameter data ready) or count parameter not yet seen (param[0] for YGE)
    if (paramPayloadLength > 0 || (oygeCachedParams & 0x01) == 0)
        return;

    // make param payload available if all params cached
    const uint16_t ygeParamCount = ygeParams[0];
    if (ygeParamCount > 0 && ygeParamCount <= OPENYGE_PARAM_CACHE_SIZE_MAX &&
        ~(~1ULL << (ygeParamCount - 1)) == oygeCachedParams) {
        paramPayloadLength = ygeParamCount * 2;
    }
}

static void oygeBuildReq(uint8_t req, uint8_t device, void *payload, uint8_t len, uint16_t framePeriod, uint16_t frameTimeout)
{
    OpenYGEHeader_t *hdr = (OpenYGEHeader_t*)reqbuffer;
    reqLength = sizeof(*hdr) + len + OPENYGE_CRC_LENGTH;     // header + payload + crc
    if (reqLength > REQUEST_BUFFER_SIZE)
        return;

    hdr->sync = OPENYGE_SYNC;
    hdr->version = OPENYGE_VERSION;
    hdr->frame_type = req;
    hdr->frame_length = reqLength;
    hdr->seq++;                                 // advance sequence number, overlapped operations not supported by this implementation
    hdr->device = device | OPENYGE_DEV_MASTER;  // as master

    if (payload != NULL)
        memcpy(hdr + 1, payload, len);

    *((uint16_t*)(reqbuffer + reqLength - OPENYGE_CRC_LENGTH)) = oygeCalculateCRC16_CCITT(reqbuffer, reqLength - OPENYGE_CRC_LENGTH);

    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;
}

static void oygeBuildNextReq(const OpenYGEHeader_t *hdr)
{
    OpenYGEControlFrame_t ctl;

    // schedule pending write request...
    const uint16_t *ygeUpdParams = (uint16_t*)paramUpdPayload;
    for (uint8_t idx = 0; oygeDirtyParams != 0; idx++) {
        uint64_t bit = 1ULL << idx;
        // index dirty?
        if ((oygeDirtyParams & bit) != 0) {
            // clear dirty bit
            oygeDirtyParams &= ~(bit);

            // schedule write request
            ctl.index = idx;
            ctl.param = ygeUpdParams[idx];
            oygeBuildReq(OPENYGE_FTYPE_WRITE_PARAM_REQ, 1, &ctl, sizeof(ctl), OPENYGE_PARAM_FRAME_PERIOD, OPENYGE_REQ_WRITE_TIMEOUT);
            return;
        }
    }

    // ...or nothing pending, schedule read telemetry request
    ctl.index = ctl.param = 0;
    const uint8_t framePeriod = hdr->frame_type == OPENYGE_FTYPE_TELE_AUTO ? OPENYGE_FRAME_PERIOD_INIT : OPENYGE_FRAME_PERIOD;
    oygeBuildReq(OPENYGE_FTYPE_TELE_REQ, 1, &ctl, sizeof(ctl), framePeriod, OPENYGE_REQ_READ_TIMEOUT);
}

static void oygeDecodeTelemetryFrame(void)
{
    const OpenYGEHeader_t *hdr = (OpenYGEHeader_t*)buffer;
    const OpenYGETelemetryFrame_t *tele = (OpenYGETelemetryFrame_t*)(buffer + (hdr->version >= 3 ? OPENYGE_HEADER_LENGTH : OPENYGE_HEADER_LENGTH_LEGACY));

    int16_t temp = tele->temperature - OPENYGE_TEMP_OFFSET;
    int16_t tempBEC = tele->bec_temp - OPENYGE_TEMP_OFFSET;

    escSensorData[0].age = 0;
    escSensorData[0].erpm = tele->rpm * 10;
    escSensorData[0].pwm = tele->pwm * 10;
    escSensorData[0].throttle = tele->throttle * 10;
    escSensorData[0].voltage = tele->voltage * 10;
    escSensorData[0].current = tele->current * 10;
    escSensorData[0].consumption = tele->consumption;
    escSensorData[0].temperature = temp * 10;
    escSensorData[0].temperature2 = tempBEC * 10;
    escSensorData[0].bec_voltage = tele->bec_voltage;
    escSensorData[0].bec_current = tele->bec_current;
    escSensorData[0].status = tele->status1 | 0x0100;

    oygeCacheParam(tele->pidx, tele->pdata);

    DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, tele->rpm * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, tele->voltage * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, tele->current * 10);

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, tele->rpm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, tele->pwm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, tele->voltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, tele->current);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, tele->consumption);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tele->status1);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);
}

static const OpenYGEHeader_t *oygeGetHeaderWithCrcCheck()
{
    // get header (w/ paranoid buffer access)
    const OpenYGEHeader_t *hdr = (OpenYGEHeader_t*)buffer;
    const uint8_t len = hdr->frame_length;
    if (len < OPENYGE_MIN_FRAME_LENGTH || len > TELEMETRY_BUFFER_SIZE)
        return NULL;

    // check CRC
    const uint16_t crc = *(uint16_t*)(buffer + len - OPENYGE_CRC_LENGTH);
    if (oygeCalculateCRC16_CCITT(buffer, len - OPENYGE_CRC_LENGTH) != crc)
        return NULL;

    return hdr;
}

static bool oygeDecodeAuto(timeMs_t currentTimeMs)
{
    // get header w/ CRC check
    const OpenYGEHeader_t *hdr = oygeGetHeaderWithCrcCheck();
    if (hdr == NULL)
        return false;

    // decode payload
    oygeDecodeTelemetryFrame();

    // adjust auto frame timeout (10ms + last seen decreasing interval until min)
    if (oygeAutoFrameTimeout > OPENYGE_AUTO_MIN_FRAME_TIMEOUT && currentTimeMs - rrfsmFrameTimestamp < oygeAutoFrameTimeout) {
        oygeAutoFrameTimeout = MAX(OPENYGE_AUTO_MIN_FRAME_TIMEOUT, currentTimeMs - rrfsmFrameTimestamp + 10);
    }
    rrfsmFrameTimeout = oygeAutoFrameTimeout;

    return true;
}

static bool oygeDecodeTelemetry(const OpenYGEHeader_t *hdr, timeMs_t currentTimeMs)
{
    // switch to auto telemetry mode if ESC FW too old
    if (hdr->version < 3) {
        rrfsmDecode = oygeDecodeAuto;
        paramCommit = NULL;
        return oygeDecodeAuto(currentTimeMs);
    }

    // response sequence number should match request (ignore auto telemetry)
    const OpenYGEHeader_t *req = (OpenYGEHeader_t*)reqbuffer;
    if (hdr->frame_type != OPENYGE_FTYPE_TELE_AUTO && hdr->seq != req->seq)
        return false;

    // decode payload
    oygeDecodeTelemetryFrame();

    // schedule next request
    oygeBuildNextReq(hdr);

    return true;
}

static bool oygeDecode(timeMs_t currentTimeMs)
{
    // get header w/ CRC check
    const OpenYGEHeader_t *hdr = oygeGetHeaderWithCrcCheck();
    if (hdr == NULL)
        return false;

    switch (hdr->frame_type) {
        case OPENYGE_FTYPE_TELE_AUTO:
        case OPENYGE_FTYPE_TELE_RESP:
        case OPENYGE_FTYPE_WRITE_PARAM_RESP:
            return oygeDecodeTelemetry(hdr, currentTimeMs);
        default:
            return false;
    }
}

static int8_t oygeAccept(uint16_t c)
{
    if (readBytes == 1) {
        // sync
        if (c != OPENYGE_SYNC)
            return -1;
    }
    else if (readBytes == 3) {
        switch (c) {
            case OPENYGE_FTYPE_TELE_AUTO:
            case OPENYGE_FTYPE_TELE_REQ:
            case OPENYGE_FTYPE_TELE_RESP:
            case OPENYGE_FTYPE_WRITE_PARAM_REQ:
            case OPENYGE_FTYPE_WRITE_PARAM_RESP:
                break;
            default:
                // unsupported frame type
                return -1;
        }
    }
    else if (readBytes == 4) {
        // frame length
        if (c < OPENYGE_MIN_FRAME_LENGTH || c > TELEMETRY_BUFFER_SIZE) {
            // protect against buffer underflow/overflow
            return -1;
        }
        else {
            // new frame length for this frame
            rrfsmFrameLength = c;
            return 1;
        }
    }
    return 0;
}

static serialReceiveCallbackPtr oygeSensorInit(bool bidirectional)
{
    rrfsmBootDelayMs = OPENYGE_BOOT_DELAY;
    rrfsmMinFrameLength = OPENYGE_MIN_FRAME_LENGTH;
    rrfsmAccept = oygeAccept;

    paramSig = OPENYGE_PARAM_SIG;

    if (bidirectional) {
        // use request/response telemetry mode, enable parameter writes to ESC
        rrfsmDecode = oygeDecode;
        paramCommit = oygeParamCommit;
    }
    else {
        // use auto telemetry mode
        rrfsmDecode = oygeDecodeAuto;
    }

    return rrfsmDataReceive;
}


/*
 * Raw Telemetry Data Recorder
 */

static void recordSensorProcess(timeUs_t currentTimeUs)
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
            case ESC_SENSOR_PROTO_BLHELI32:
                blSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW4:
                hw4SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW5:
                hw5SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_SCORPION:
                rrfsmSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_KONTRONIK:
                kontronikSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_OMPHOBBY:
                ompSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_ZTW:
                ztwSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_APD:
                apdSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_OPENYGE:
                rrfsmSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_RECORD:
                recordSensorProcess(currentTimeUs);
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

bool INIT_CODE escSensorInit(void)
{
    const bool escHalfDuplex = escSensorConfig()->halfDuplex;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    serialReceiveCallbackPtr callback = NULL;
    portOptions_e options = 0;
    uint32_t baudrate = 0;

    if (!portConfig) {
        return false;
    }

    options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escHalfDuplex ? SERIAL_BIDIR : 0);

    switch (escSensorConfig()->protocol) {
        case ESC_SENSOR_PROTO_BLHELI32:
            callback = blDataReceive;
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_HW4:
            baudrate = 19200;
            break;
        case ESC_SENSOR_PROTO_SCORPION:
            callback = tribSensorInit(escHalfDuplex);
            baudrate = 38400;
            break;
        case ESC_SENSOR_PROTO_KONTRONIK:
            baudrate = 115200;
            options |= SERIAL_PARITY_EVEN;
            break;
        case ESC_SENSOR_PROTO_OMPHOBBY:
        case ESC_SENSOR_PROTO_ZTW:
        case ESC_SENSOR_PROTO_HW5:
        case ESC_SENSOR_PROTO_APD:
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_OPENYGE:
            callback = oygeSensorInit(escHalfDuplex);
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_RECORD:
            baudrate = baudRates[portConfig->telemetry_baudrateIndex];
            break;
    }

    if (baudrate) {
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, callback, NULL, baudrate, escHalfDuplex ? MODE_RXTX : MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].age = ESC_DATA_INVALID;
    }

    escSensorDataCombined.age = ESC_DATA_INVALID;

    return (escSensorPort != NULL);
}


uint8_t escGetParamBufferLength(void)
{
    paramMspActive = true;
    return paramPayloadLength != 0 ? PARAM_HEADER_SIZE + paramPayloadLength : 0;
}

uint8_t *escGetParamBuffer(void)
{
    paramBuffer[PARAM_HEADER_SIG] = paramSig;
    paramBuffer[PARAM_HEADER_VER] = paramVer | (paramCommit == NULL ? PARAM_HEADER_RDONLY : 0);
    return paramBuffer;
}

uint8_t *escGetParamUpdBuffer(void)
{
    return paramUpdBuffer;
}

bool escCommitParameters(void)
{
    return paramUpdBuffer[PARAM_HEADER_SIG] == paramBuffer[PARAM_HEADER_SIG] &&
        (paramUpdBuffer[PARAM_HEADER_VER] & PARAM_HEADER_VER_MASK) == (paramBuffer[PARAM_HEADER_VER] & PARAM_HEADER_VER_MASK) &&
        paramCommit != NULL && paramCommit(paramUpdBuffer[PARAM_HEADER_VER] & PARAM_HEADER_CMD_MASK);
}

#endif
