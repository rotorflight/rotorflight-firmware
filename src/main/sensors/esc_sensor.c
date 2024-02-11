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

static uint8_t  readBytes = 0;
static uint32_t syncCount = 0;


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
                uint16_t voltage = buffer[16] << 8 | buffer[15];
                uint16_t current = buffer[18] << 8 | buffer[17];
                uint16_t tempFET = buffer[19];
                uint16_t tempBEC = buffer[20];

                // When throttle changes to zero, the last current reading is
                // repeated until the motor has totally stopped.
                if (power == 0) {
                    current = 0;
                }

                setConsumptionCurrent(current * 0.1f);

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].pwm = power * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].temperature = tempFET * 10;
                escSensorData[0].temperature2 = tempBEC * 10;

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
 * Scorpion Unsolicited Telemetry
 *
 *    - ESC must be set to "Unsolicited mode"
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
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      Header Sync (0x55)
 *      1:      Message format version (0x00)
 *      2:      Message Length incl. header and CRC (22)
 *      3:      Device ID
 *    4-6:      Timestamp ms
 *      7:      Throttle in 0.5%
 *    8-9:      Current in 0.1A
 *  10-11:      Voltage in 0.1V
 *  12-13:      Consumption in mAh
 *     14:      Temperature in °C
 *     15:      PWM duty cycle in 0.5%
 *     16:      BEC voltage in 0.1V
 *  17-18:      RPM in 5rpm steps
 *     19:      Error code
 *  20-21:      CRC16 CCITT
 *
 */

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

static bool processUNCTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0x55)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x00)  // Proto v0
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 22)  // Message v0 is 22 bytes
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 22) {
        readBytes = 0;
        return true;
    }

    return false;
}

static void uncSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processUNCTelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[21] << 8 | buffer[20];

            if (calculateCRC16_CCITT(buffer, 20) == crc) {
                uint16_t rpm = buffer[18] << 8 | buffer[17];
                uint16_t temp = buffer[14];
                uint16_t power = buffer[15];
                uint16_t voltage = buffer[11] << 8 | buffer[10];
                uint16_t current = buffer[9] << 8 | buffer[8];
                uint16_t capacity = buffer[13] << 8 | buffer[12];
                uint16_t status = buffer[19];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 5;
                escSensorData[0].pwm = power * 5;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = temp * 10;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 5);
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

    // Maximum frame spacing 1000ms
    checkFrameTimeout(currentTimeUs, 1200000);
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
 *  34-37:      CRC32
 *
 */

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
                int16_t  tempFET = (int8_t)buffer[26];
                int16_t  tempBEC = (int8_t)buffer[27];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = tempFET * 10;
                escSensorData[0].temperature2 = tempBEC * 10;

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
                uint16_t pwm = buffer[12];
                uint16_t temp = buffer[10];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = temp * 10;

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
                uint16_t power = buffer[12];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].pwm = power * 10;
                escSensorData[0].voltage = voltage * 100;
                escSensorData[0].current = current * 100;
                escSensorData[0].consumption = capacity;
                escSensorData[0].temperature = temp * 10;

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
                uint16_t power = buffer[17] << 8 | buffer[16];
                uint16_t voltage = buffer[1] << 8 | buffer[0];
                uint16_t current = buffer[5] << 8 | buffer[4];
                uint16_t status = buffer[18];

                float temp = calcTempAPD(tadc);

                setConsumptionCurrent(current * 0.08f);

                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].pwm = power;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 80;
                escSensorData[0].temperature = lrintf(temp * 10);

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
 *
 * Data Frame Format
 * ―――――――――――――――――――――――――――――――――――――――――――――――
 *     0:       Number for protocol compat. (0 for first tests)
 *     1:       temperature;        // C degrees
 *   2,3:       voltage;            // 0.01V    Little endian!
 *   4,5:       current;            // 0.01A    Little endian!
 *   6,7:       consumption;        // mAh      Little endian!
 *   8,9:       rpm;                // 0.1rpm   Little endian!
 *    10:       pwm;                // %
 *    11:       throttle;           // %
 * 12,13:       bec_voltage;        // 0.01V    Little endian!
 * 14,15:       bec_current;        // 0.01A    Little endian!
 *    16:       bec_temp;           // C degrees
 *    17:       status1;            // see documentation
 *    18:       status2;            // Debug/Reserved
 *    19:       index;              // reserved
 *    20:       reserved1;          // indexed-data
 *    21:       reserved2;          // indexed data
 * 22,23:       crc16;              // CCITT, poly: 0x1021
 *
 */

#define OPENYGE_PROTOCOL_VERSION        0
#define OPENYGE_BOOT_DELAY              5000                // 5 seconds
#define OPENYGE_RAMP_INTERVAL           10000               // 10 seconds
#define OPENYGE_FRAME_PERIOD_INITIAL    900                 // intially 800 w/ progressive decreasing frame-period during ramp time...
#define OPENYGE_FRAME_PERIOD_FINAL      60                  // ...to the final 50ms
#define OPENYGE_FRAME_SIZE              24                  // 24 bytes

enum {
    OPENYGE_FRAME_FAILED                = 0,
    OPENYGE_FRAME_PENDING               = 1,
    OPENYGE_FRAME_COMPLETE              = 2,
};

static timeMs_t oygeRampTimer = 0;
static uint32_t oygeFrameTimestamp = 0;
static uint16_t oygeFramePeriod = OPENYGE_FRAME_PERIOD_INITIAL;


static uint16_t oygeCalculateCRC16_CCITT(const uint8_t *ptr, size_t len)
{
    uint16_t crc = 0;

    for(uint16_t j = 0; j < len; j++)
    {
        crc = crc ^ ptr[j] << 8;
        for(uint16_t i = 0; i < 8; i++)
        {
            if(crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
  return crc;
}

static void oygeFrameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}
  
static FAST_CODE void oygeDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    totalByteCount++;

    if (readBytes < bufferSize) {
        buffer[readBytes++] = c;

        if (readBytes == 1) {
            if (c != OPENYGE_PROTOCOL_VERSION)
                oygeFrameSyncError();
            else
                syncCount++;
        }
    }
}

static void oygeStartTelemetryFrame(timeMs_t currentTimeMs)
{
    readBytes = 0;
    bufferSize = OPENYGE_FRAME_SIZE;

    oygeFrameTimestamp = currentTimeMs;
}

static uint8_t oygeDecodeTelemetryFrame(void)
{
    // First, check the variables that can change in the interrupt
    if (readBytes < bufferSize)
        return OPENYGE_FRAME_PENDING;

    // verify CRC16 checksum
    uint16_t crc = buffer[OPENYGE_FRAME_SIZE - 1] << 8 | buffer[OPENYGE_FRAME_SIZE - 2];

    if (oygeCalculateCRC16_CCITT(buffer, OPENYGE_FRAME_SIZE - 2) == crc) {
        uint16_t temp = buffer[1];
        uint16_t volt = buffer[3] << 8 | buffer[2];
        uint16_t curr = buffer[5] << 8 | buffer[4];
        uint16_t capa = buffer[7] << 8 | buffer[6];
        uint16_t erpm = buffer[9] << 8 | buffer[8];
        uint8_t   pwm = buffer[10];
        uint16_t voltBEC = buffer[13] << 8 | buffer[12];
        uint16_t tempBEC = buffer[16];

        escSensorData[0].age = 0;
        escSensorData[0].erpm = erpm * 10;
        escSensorData[0].pwm = pwm * 10;
        escSensorData[0].voltage = volt * 10;
        escSensorData[0].current = curr * 10;
        escSensorData[0].consumption = capa;
        escSensorData[0].temperature = temp * 10;
        escSensorData[0].temperature2 = tempBEC * 10;

        totalFrameCount++;

        DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, erpm * 10);
        DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
        DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, volt);
        DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, curr);

        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, erpm);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, volt);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, curr);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capa);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, voltBEC);
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

        return OPENYGE_FRAME_COMPLETE;
    }

    totalCrcErrorCount++;

    return OPENYGE_FRAME_FAILED;
}

static void oygeSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    // wait before initializing
    if (currentTimeMs < OPENYGE_BOOT_DELAY) 
        return;

    // one time init
    if (oygeFrameTimestamp == 0) {
        oygeStartTelemetryFrame(currentTimeMs);
        return;
    }

    // switch to final frame timeout if ramp time complete
    if (oygeFramePeriod == OPENYGE_FRAME_PERIOD_INITIAL && oygeRampTimer != 0 && currentTimeMs > oygeRampTimer)
        oygeFramePeriod = OPENYGE_FRAME_PERIOD_FINAL;

    // timeout waiting for frame?
    if (currentTimeMs > oygeFrameTimestamp + oygeFramePeriod) {
        increaseDataAge(0);
        oygeStartTelemetryFrame(currentTimeMs);
        totalTimeoutCount++;
        return;
    }

    // attempt to decode frame
    uint8_t state = oygeDecodeTelemetryFrame();
    switch (state) {
        case OPENYGE_FRAME_PENDING:
            // frame not ready yet
            break;
        case OPENYGE_FRAME_FAILED:
            increaseDataAge(0);
            // next frame
            oygeStartTelemetryFrame(currentTimeMs);
            break;
        case OPENYGE_FRAME_COMPLETE:
            // start ramp timer if first frame seen
            if (oygeRampTimer == 0)
                oygeRampTimer = currentTimeMs + OPENYGE_RAMP_INTERVAL;
            // next frame
            oygeStartTelemetryFrame(currentTimeMs);
            break;
    }
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
                uncSensorProcess(currentTimeUs);
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
                oygeSensorProcess(currentTimeUs);
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
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    serialReceiveCallbackPtr callback = NULL;
    portOptions_e options = 0;
    uint32_t baudrate = 0;

    if (!portConfig) {
        return false;
    }

    options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

    switch (escSensorConfig()->protocol) {
        case ESC_SENSOR_PROTO_BLHELI32:
            callback = blDataReceive;
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_HW4:
            baudrate = 19200;
            break;
        case ESC_SENSOR_PROTO_SCORPION:
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
            callback = oygeDataReceive;
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_RECORD:
            baudrate = baudRates[portConfig->telemetry_baudrateIndex];
            break;
    }

    if (baudrate) {
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, callback, NULL, baudrate, MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].age = ESC_DATA_INVALID;
    }

    escSensorDataCombined.age = ESC_DATA_INVALID;

    return (escSensorPort != NULL);
}

#endif
