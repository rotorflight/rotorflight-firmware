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

#include "drivers/castle_telemetry_decode.h"
#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "flight/mixer.h"

#include "io/serial.h"

#include "esc_sensor.h"


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

#define TELEMETRY_BUFFER_SIZE    140
#define REQUEST_BUFFER_SIZE      64
#define PARAM_BUFFER_SIZE        96
#define PARAM_HEADER_SIZE        2
#define PARAM_HEADER_SIG         0
#define PARAM_HEADER_VER         1
#define PARAM_HEADER_VER_MASK    0x3F
#define PARAM_HEADER_CMD_MASK    0xC0
#define PARAM_HEADER_RDONLY      0x40
#define PARAM_HEADER_USER        0x80

// ESC signatures
#define ESC_SIG_NONE              0x00
#define ESC_SIG_BLHELI32          0xC8
#define ESC_SIG_HW4               0x9B
#define ESC_SIG_KON               0x4B
#define ESC_SIG_OMP               0xD0
#define ESC_SIG_ZTW               0xDD
#define ESC_SIG_APD               0xA0
#define ESC_SIG_PL5               0xFD
#define ESC_SIG_TRIB              0x53
#define ESC_SIG_OPENYGE           0xA5
#define ESC_SIG_XDFLY             0xA6
#define ESC_SIG_FLY               0x73
#define ESC_SIG_GRAUPNER          0xC0
#define ESC_SIG_CASTLE            0xCC
#define ESC_SIG_RESTART           0xFF

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

static uint8_t escSig = 0;

static uint8_t paramPayloadLength = 0;
static uint8_t paramBuffer[PARAM_BUFFER_SIZE] = { 0, };
static uint8_t paramUpdBuffer[PARAM_BUFFER_SIZE] = { 0, };
static uint8_t *paramPayload = paramBuffer + PARAM_HEADER_SIZE;
static uint8_t *paramUpdPayload = paramUpdBuffer + PARAM_HEADER_SIZE;
static uint8_t paramVer = 0;
static bool paramMspActive = false;

// called on MSP_SET_ESC_PARAMETERS when paramUpdPayload / paramUpdBuffer ready
typedef bool (*paramCommitCallbackPtr)(uint8_t cmd);
static paramCommitCallbackPtr paramCommit = NULL;

static void paramEscNeedRestart(void)
{
    escSensorData[0].age = 0;
    escSensorData[0].id = ESC_SIG_RESTART;
}


bool isEscSensorActive(void)
{
    return escSensorPort != NULL || isMotorProtocolCastlePWM();
}

uint32_t getEscSensorRPM(uint8_t motorNumber)
{
    return (escSensorData[motorNumber].age <= ESC_BATTERY_AGE_MAX) ? escSensorData[motorNumber].erpm : 0;
}

static uint32_t applyVoltageCorrection(uint32_t voltage)
{
    if (escSensorConfig()->voltage_correction == 0)
        return voltage;

    return (voltage * (100 + escSensorConfig()->voltage_correction)) / 100;
}

static uint32_t applyCurrentCorrection(uint32_t current)
{
    if (escSensorConfig()->current_correction == 0)
        return current;

    return (current * (100 + escSensorConfig()->current_correction)) / 100;
}

static uint32_t applyConsumptionCorrection(uint32_t consumption)
{
    if (escSensorConfig()->consumption_correction == 0)
        return consumption;

    return (consumption * (100 + escSensorConfig()->consumption_correction)) / 100;
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
#ifdef USE_TELEMETRY_CASTLE
            if (isMotorProtocolCastlePWM()) {
                if (motorNumber == 0 || motorNumber == ESC_SENSOR_COMBINED)
                    return &escSensorData[0];
            }
#endif
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

    escSensorData[0].consumption = applyConsumptionCorrection(lrintf(totalConsumption));
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

        escSensorData[currentEsc].id = ESC_SIG_BLHELI32;
        escSensorData[currentEsc].age = 0;
        escSensorData[currentEsc].erpm = erpm * 100;
        escSensorData[currentEsc].voltage = applyVoltageCorrection(volt * 10);
        escSensorData[currentEsc].current = applyCurrentCorrection(curr * 10);
        escSensorData[currentEsc].consumption = applyConsumptionCorrection(capa);
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
                if (thr == 0) {
                    current = 0;
                }

                setConsumptionCurrent(current);

                escSensorData[0].id = ESC_SIG_HW4;
                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = thr;
                escSensorData[0].pwm = pwm;
                escSensorData[0].voltage = applyVoltageCorrection(lrintf(voltage * 1000));
                escSensorData[0].current = applyCurrentCorrection(lrintf(current * 1000));
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
            if (buffer[5] && buffer[6]) {
                hw4VoltageScale = (float)buffer[5] / ((float)buffer[6] * 10);
            }
            if (buffer[7] && buffer[8]) {
                hw4CurrentScale = (float)buffer[7] / (float)buffer[8];
                hw4CurrentOffset = (float)buffer[9] / hw4CurrentScale;
            }
        }
    }

    // Update consumption on every cycle
    updateConsumption(currentTimeUs);

    // Maximum data frame spacing 400ms
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

                escSensorData[0].id = ESC_SIG_KON;
                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = (throttle + 100) * 5;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = applyVoltageCorrection(voltage * 10);
                escSensorData[0].current = applyCurrentCorrection(current * 100);
                escSensorData[0].consumption = applyConsumptionCorrection(capacity);
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

                escSensorData[0].id = ESC_SIG_OMP;
                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].throttle = throttle * 10;
                escSensorData[0].pwm = pwm * 10;
                escSensorData[0].voltage = applyVoltageCorrection(voltage * 100);
                escSensorData[0].current = applyCurrentCorrection(current * 100);
                escSensorData[0].consumption = applyConsumptionCorrection(capacity);
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

                escSensorData[0].id = ESC_SIG_ZTW;
                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm * 10;
                escSensorData[0].throttle = throttle * 10;
                escSensorData[0].pwm = power * 10;
                escSensorData[0].voltage = applyVoltageCorrection(voltage * 100);
                escSensorData[0].current = applyCurrentCorrection(current * 100);
                escSensorData[0].consumption = applyConsumptionCorrection(capacity);
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

                escSensorData[0].id = ESC_SIG_APD;
                escSensorData[0].age = 0;
                escSensorData[0].erpm = rpm;
                escSensorData[0].throttle = throttle;
                escSensorData[0].pwm = power;
                escSensorData[0].voltage = applyVoltageCorrection(voltage * 10);
                escSensorData[0].current = applyCurrentCorrection(current * 80);
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

typedef bool (*rrfsmStartCallbackPtr)(timeUs_t currentTimeUs);  // return true to continue w/ default initialization (if in doubt return true)
typedef bool (*rrfsmDecodeCallbackPtr)(timeUs_t currentTimeUs); // return true if frame was decoded successfully
typedef bool (*rrfsmCrankCallbackPtr)(timeUs_t currentTimeUs);  // return true to continue w/ default loop (advanced, if in doubt return true)

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

    // start
    if (rrfsmFrameTimestamp == 0) {
        if (rrfsmStart == NULL || rrfsmStart(currentTimeUs))
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
    if (rrfsmCrank != NULL && !rrfsmCrank(currentTimeUs)) {
        return;
    }

    // frame incomplete? check later
    if (readBytes < rrfsmFrameLength) {
        return;
    }

    // frame complate, process, prepare for next frame
    if (rrfsmDecode == NULL || rrfsmDecode(currentTimeUs)) {
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
 * FlyRotor Telemetry
 *
 *     - Serial protocol is 115200,8N1
 *     - Big-Endian byte order
 * 
 * Telemetry Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      start byte (0x73)
 *      1:      0 <- command (0 == data, 0x60 == read esc info etc)
 *    2-3:      data length
 * 
 * Telemetry Data
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    4-5:      Battery voltage x10mV [0-65535] 
 *    6-7:      Current x10mA [0-65535]
 *    8-9:      Capacity 1mAh [0-65535]
 *  10-11:      ERPM 10rpm [0-65535]
 *     12:      Throttle [0-100]
 *     13:      ESC Temperature 1°C [-30-225]
 *     14:      MCU Temperature 1°C [-30-225]
 *     15:      Motor Temperature 1°C [-30-225]
 *     16:      BEC Voltage x100mV [0-255]
 *     17:      Status flag
 *     18:      Mode [0-255]
 * 
 *     19:      CRC8
 *     20:      end byte (0x65)
 */
#define FLY_SYNC                            0x73                // start byte
#define FLY_END                             0x65                // end byte
#define FLY_HEADER_LENGTH                   4                   // header length
#define FLY_FOOTER_LENGTH                   2                   // CRC + end byte
#define FLY_MIN_FRAME_LENGTH                (FLY_HEADER_LENGTH + FLY_FOOTER_LENGTH)
#define FLY_BOOT_DELAY                      5000
#define FLY_TELE_FRAME_TIMEOUT              40

#define FLY_TEMP_OFFSET                     30

#define FLY_CMD_TELEMETRY_DATA              0xFF                // telemetry data (slave response only)
#define FLY_CMD_CONNECT_ESC                 0x50                // Connect ESC
#define FLY_CMD_CONNECT_ESC_RESP            0xAF                // Connect ESC resp
#define FLY_CMD_DISCONNECT_ESC              0x5F                // Disconnect ESC
#define FLY_CMD_DISCONNECT_ESC_RESP         0xA0                // Disconnect ESC resp
#define FLY_CMD_READ_INFO                   0x60                // Read ESC information
#define FLY_CMD_READ_INFO_RESP              0x9F                // Read ESC information resp
#define FLY_CMD_READ_BASIC                  0x61                // Read ESC basic parameters
#define FLY_CMD_READ_BASIC_RESP             0x9E                // Read ESC basic parameters resp
#define FLY_CMD_READ_ADV                    0x62                // Read ESC advanced parameters
#define FLY_CMD_READ_ADV_RESP               0x9D                // Read ESC advanced parameters resp
#define FLY_CMD_READ_REALTIME               0x63                // Read ESC real-time parameters
#define FLY_CMD_READ_OTHER                  0x64                // Read ESC other parameters
#define FLY_CMD_READ_OTHER_RESP             0x9B                // Read ESC other parameters resp
#define FLY_CMD_WRITE_BASIC                 0x90                // Write ESC basic parameters
#define FLY_CMD_WRITE_BASIC_RESP            0x6F                // Write ESC basic parameters resp
#define FLY_CMD_WRITE_ADV                   0x91                // Write ESC advanced parameters
#define FLY_CMD_WRITE_ADV_RESP              0x6E                // Write ESC advanced parameters resp
#define FLY_CMD_WRITE_OTHER                 0x92                // Write ESC other parameters
#define FLY_CMD_WRITE_OTHER_RESP            0x6D                // Write ESC other parameters resp

#define FLY_PARAM_FRAME_PERIOD              2                   // asap
#define FLY_PARAM_CONNECT_TIMEOUT           10
#define FLY_PARAM_DISCONNECT_TIMEOUT        100
#define FLY_PARAM_READ_TIMEOUT              100
#define FLY_PARAM_WRITE_TIMEOUT             100

#define FLY_PARAM_PAGE_INFO                 0
#define FLY_PARAM_PAGE_BASIC                1
#define FLY_PARAM_PAGE_ADV                  2
#define FLY_PARAM_PAGE_OTHER                3
#define FLY_PARAM_PAGE_ALL                  0x0F
#define FLY_PARAM_PAGE_LEN_MASK             0xFF

// param pages - hi-byte=offset, low-byte=length
static uint16_t flyParamPages[] = { 0x0016, 0x160C, 0x220A, 0x2C0A };
static uint8_t flyParamReadCmds[] = { FLY_CMD_READ_INFO, FLY_CMD_READ_BASIC, FLY_CMD_READ_ADV, FLY_CMD_READ_OTHER };
static uint8_t flyParamWriteCmds[] = { 0, FLY_CMD_WRITE_BASIC, FLY_CMD_WRITE_ADV, FLY_CMD_WRITE_OTHER };

static uint8_t flyInvalidParamPages = 0;
static uint8_t flyDirtyParamPages = 0;
static bool flyConnected = false;

uint8_t flyCalculateCRC8(uint8_t *pData, uint16_t usLength)
{
    uint16_t usCnt;
    uint8_t ucBit, ucResult = 0;

    for (usCnt = 0; usCnt < usLength; usCnt++)
    {
        ucResult ^= pData[usCnt];

        for (ucBit = 0; ucBit < 8; ucBit++)
        {
            if (ucResult & 0x80)
            {
                ucResult = (ucResult << 1) ^ 0x07;
            }
            else
            {
                ucResult <<= 1;
            }
        }
    }
    return ucResult;
}

static bool flyParamCommit(uint8_t cmd)
{
    // bail if invalid command
    if (cmd != 0)
        return false;

    // find dirty pages (ignore INFO page)
    for (uint8_t i = FLY_PARAM_PAGE_BASIC; i < ARRAYLEN(flyParamPages); i++) {
        // check page
        const uint8_t pageBit = (1 << i);
        const uint16_t page = flyParamPages[i];
        const uint8_t offset = page >> 8;
        const uint8_t len = page & FLY_PARAM_PAGE_LEN_MASK;
        if (memcmp(paramPayload + offset, paramUpdPayload + offset, len) != 0) {
            // set dirty bit
            flyDirtyParamPages |= pageBit;
            // invalidate page
            flyInvalidParamPages |= pageBit;
            // invalidate param payload - will be available again when all params again cached
            paramPayloadLength = 0;
        }
    }
    return true;
}

static void flyBuildReq(uint8_t cmd, void *data, uint8_t datalen, uint16_t framePeriod, uint16_t frameTimeout)
{
    uint8_t idx = 0;
    reqbuffer[idx++] = FLY_SYNC;
    reqbuffer[idx++] = cmd;
    reqbuffer[idx++] = 0;
    reqbuffer[idx++] = datalen;
    if(data != NULL) {
        memcpy(reqbuffer + idx, data, datalen);
        idx += datalen;
    }
    uint8_t crclen = idx;
    reqbuffer[idx++] = flyCalculateCRC8(reqbuffer, crclen);
    reqbuffer[idx++] = FLY_END;

    reqLength = idx;
    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;
}

static void flyBuildNextReq(void)
{
    // if not connected...
    if (!flyConnected) {
        // ...connect if writes or reads pending
        if (flyDirtyParamPages != 0 || flyInvalidParamPages != 0) {
            // connect
            uint8_t data = 0xA0;
            flyBuildReq(FLY_CMD_CONNECT_ESC, &data, 1, FLY_PARAM_FRAME_PERIOD, FLY_PARAM_CONNECT_TIMEOUT);
        }
    }
    else {
        // pending write request...
        for (uint8_t i = FLY_PARAM_PAGE_BASIC; i < ARRAYLEN(flyParamPages); i++) {
            const uint8_t pageBit = (1 << i);
            if ((flyDirtyParamPages & pageBit) != 0) {
                const uint16_t page = flyParamPages[i];
                const uint8_t offset = page >> 8;
                const uint8_t len = page & FLY_PARAM_PAGE_LEN_MASK;
                const uint8_t cmd = flyParamWriteCmds[i];
                flyBuildReq(cmd, paramUpdPayload + offset, len, FLY_PARAM_FRAME_PERIOD, FLY_PARAM_WRITE_TIMEOUT);
                return;
            }
        }

        // ...or schedule pending read request if no pending writes...
        for (uint8_t i = 0; i < ARRAYLEN(flyParamPages); i++) {
            const uint8_t pageBit = (1 << i);
            if ((flyInvalidParamPages & pageBit) != 0) {
                const uint8_t cmd = flyParamReadCmds[i];
                flyBuildReq(cmd, NULL, 0, FLY_PARAM_FRAME_PERIOD, FLY_PARAM_READ_TIMEOUT);
                return;
            }
        }
        
        // ...nothing pending, disconnect
        flyBuildReq(FLY_CMD_DISCONNECT_ESC, NULL, 0, FLY_PARAM_FRAME_PERIOD, FLY_PARAM_DISCONNECT_TIMEOUT);
    }
}

static void flyDecodeTelemetryFrame(void)
{
    const uint8_t hl = FLY_HEADER_LENGTH;

    const uint16_t rpm = buffer[hl + 6] << 8 | buffer[hl + 7];
    const int16_t temp = buffer[hl + 9] - FLY_TEMP_OFFSET;
    const int16_t motorTemp = buffer[hl + 11] - FLY_TEMP_OFFSET;
    const uint8_t power = buffer[hl + 8];
    const uint16_t voltage = buffer[hl + 0] << 8 | buffer[hl + 1];
    const uint16_t current = buffer[hl + 2] << 8 | buffer[hl + 3];
    const uint16_t consumption = buffer[hl + 4] << 8 | buffer[hl + 5];
    const uint16_t status = buffer[hl + 13];
    const uint16_t voltBEC = buffer[hl + 12];

    escSensorData[0].id = ESC_SIG_FLY;
    escSensorData[0].age = 0;
    escSensorData[0].erpm = rpm * 10;
    escSensorData[0].pwm = power * 10;
    escSensorData[0].voltage = applyVoltageCorrection(voltage * 10);
    escSensorData[0].current = applyCurrentCorrection(current * 10);
    escSensorData[0].consumption = applyConsumptionCorrection(consumption);
    escSensorData[0].temperature = temp * 10;
    escSensorData[0].temperature2 = motorTemp * 10;
    escSensorData[0].bec_voltage = voltBEC * 100;
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
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, consumption);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, paramPayloadLength);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);
}

static bool flyDecodeTelemetry(void)
{
    // decode payload
    flyDecodeTelemetryFrame();

    rrfsmFrameTimeout = FLY_TELE_FRAME_TIMEOUT;

    // tele frame means that ESC has exited setup mode
    flyConnected = false;

    // schedule next request
    flyBuildNextReq();

    return true;
}

static bool flyDecodeConnectResp(void)
{
    // connected
    flyConnected = true;

    // schedule next request
    flyBuildNextReq();

    return true;
}

static bool flyDecodeDisconnectResp(void)
{
    // disconnected
    flyConnected = false;

    // schedule next request
    flyBuildNextReq();

    return true;
}

static uint8_t flyCalcParamBufferLength(void)
{
    uint8_t len = 0;
    for (uint8_t i = 0; i < ARRAYLEN(flyParamPages); i++)
        len += (flyParamPages[i] & FLY_PARAM_PAGE_LEN_MASK);
    return len;
}

static bool flyDecodeReadResp(uint8_t paramPage)
{
    // cache
    const uint16_t page = flyParamPages[paramPage];
    const uint8_t offset = page >> 8;
    const uint8_t len = page & FLY_PARAM_PAGE_LEN_MASK;
    memcpy(paramPayload + offset, buffer + FLY_HEADER_LENGTH, len);

    // clear invalid bit
    flyInvalidParamPages &= ~(1 << paramPage);
    
    // make param payload available if all params cached
    if (flyInvalidParamPages == 0)
        paramPayloadLength = flyCalcParamBufferLength();

    // schedule next request
    flyBuildNextReq();

    return true;
}

static bool flyDecodeWriteResp(uint8_t paramPage)
{
    // clear dirty bit
    flyDirtyParamPages &= ~(1 << paramPage);

    // schedule next request
    flyBuildNextReq();

    return true;
}

static bool flyDecode(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // paranoid buffer access
    const uint8_t len = FLY_MIN_FRAME_LENGTH + buffer[3];
    if (len > TELEMETRY_BUFFER_SIZE)
        return false;

    // check CRC
    const uint8_t crc = buffer[len - FLY_FOOTER_LENGTH];
    if (flyCalculateCRC8(buffer, len - FLY_FOOTER_LENGTH) != crc)
        return false;

    // decode
    switch (buffer[1]) {
        case FLY_CMD_TELEMETRY_DATA:
            return flyDecodeTelemetry();
        case FLY_CMD_CONNECT_ESC_RESP:
            return flyDecodeConnectResp();
        case FLY_CMD_DISCONNECT_ESC_RESP:
            return flyDecodeDisconnectResp();
        case FLY_CMD_READ_INFO_RESP:
            return flyDecodeReadResp(FLY_PARAM_PAGE_INFO);
        case FLY_CMD_READ_BASIC_RESP:
            return flyDecodeReadResp(FLY_PARAM_PAGE_BASIC);
        case FLY_CMD_READ_ADV_RESP:
            return flyDecodeReadResp(FLY_PARAM_PAGE_ADV);
        case FLY_CMD_READ_OTHER_RESP:
            return flyDecodeReadResp(FLY_PARAM_PAGE_OTHER);
        case FLY_CMD_WRITE_BASIC_RESP:
            return flyDecodeWriteResp(FLY_PARAM_PAGE_BASIC);
        case FLY_CMD_WRITE_ADV_RESP:
            return flyDecodeWriteResp(FLY_PARAM_PAGE_ADV);
        case FLY_CMD_WRITE_OTHER_RESP:
            return flyDecodeWriteResp(FLY_PARAM_PAGE_OTHER);
        default:
            return false;
    }
}

static int8_t flyAccept(uint16_t c)
{
    if (readBytes == 1) {
        // [0] start byte
        if (c != 0x73)
            return -1;
    }
    else if (readBytes == 2) {
        switch (c) {
            case FLY_CMD_TELEMETRY_DATA:
            case FLY_CMD_CONNECT_ESC_RESP:
            case FLY_CMD_DISCONNECT_ESC_RESP:
            case FLY_CMD_READ_INFO_RESP:
            case FLY_CMD_READ_BASIC_RESP:
            case FLY_CMD_READ_ADV_RESP:
            case FLY_CMD_READ_OTHER_RESP:
            case FLY_CMD_WRITE_BASIC_RESP:
            case FLY_CMD_WRITE_ADV_RESP:
            case FLY_CMD_WRITE_OTHER_RESP:
                break;
            default:
                // unsupported frame type
                return -1;
        }
    }
    else if (readBytes == 4) {
        // [2-3] data length - hi bit always zero here, can be ignored
        if (FLY_MIN_FRAME_LENGTH + c > TELEMETRY_BUFFER_SIZE) {
            // protect against buffer underflow/overflow
            return -1;
        }
        else {
            // new frame length for this frame
            rrfsmFrameLength = FLY_MIN_FRAME_LENGTH + c;
            return 1;
        }
    }
    return 0;
}

static serialReceiveCallbackPtr flySensorInit(bool bidirectional)
{
    rrfsmBootDelayMs = FLY_BOOT_DELAY;
    rrfsmMinFrameLength = FLY_HEADER_LENGTH;
    rrfsmAccept = flyAccept;
    rrfsmDecode = flyDecode;

    escSig = ESC_SIG_FLY;

    if (bidirectional) {
        // invalidate all param pages
        flyInvalidParamPages = FLY_PARAM_PAGE_ALL;

        // enable parameter writes to ESC
        paramCommit = flyParamCommit;
    }

    return rrfsmDataReceive;
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
#define PL5_MIN_FRAME_LENGTH                8
#define PL5_BOOT_DELAY                      5000
#define PL5_TELE_FRAME_TIMEOUT              500
#define PL5_PARAM_FRAME_PERIOD              4
#define PL5_PARAM_READ_TIMEOUT              100
#define PL5_PARAM_WRITE_TIMEOUT             100

#define PL5_PING_FRAME_PERIOD               480
#define PL5_PING_TIMEOUT                    1600

#define PL5_ERR                             0x80

#define PL5_FRAME_TELE_TYPE                 0x5C30
#define PL5_FRAME_TELE_LENGTH               32
#define PL5_RESP_PING_TYPE                  0x242C
#define PL5_RESP_PING_LENGTH                11
#define PL5_RESP_DEVINFO_TYPE               0x252C
#define PL5_RESP_DEVINFO_LENGTH             73
#define PL5_RESP_DEVINFO_PAYLOAD_LENGTH     48
#define PL5_RESP_GETPARAMS_TYPE             0x0C30
#define PL5_RESP_GETPARAMS_LENGTH           137
#define PL5_RESP_GETPARAMS_PAYLOAD_LENGTH   31
#define PL5_REQ_WRITEPARAMS_LENGTH          63
#define PL5_RESP_WRITEPARAMS_TYPE           0x3835
#define PL5_RESP_WRITEPARAMS_LENGTH         58
#define PL5_RESP_WRITEPARAMS_ERR_LENGTH     9

static uint8_t pl5Ping[] = { 0x1, 0xFD, 0x3, 0x3, 0x2C, 0x24, 0x0, 0x1, 0x60, 0x60 };
static uint8_t pl5DevInfoReq[] = { 0x01, 0xFD, 0x03, 0x03, 0x2C, 0x25, 0x00, 0x20, 0xF1, 0xB8 };
static uint8_t pl5GetParamsReq[] = { 0x1, 0xFD, 0x3, 0x3, 0x30, 0xC, 0x0, 0x40, 0x27, 0xC8 };
static uint8_t pl5WriteParamsReq[] = { 0x1, 0xFD, 0x3, 0x17, 0x35, 0x38, 0x0, 0x18, 0x35, 0x38, 0x0, 0x18, 0x30, 0x0 };

typedef struct {
    uint8_t  throttle;                  // Throttle value in %
    uint8_t  reserved0[2];              // reserved
    uint8_t  fault;                     // Fault code
    uint16_t rpm;                       // RPM in 10rpm steps
    uint16_t voltage;                   // Voltage in 0.1V
    uint16_t current;                   // Current in 0.1A
    uint8_t  temperature;               // ESC Temperature in °C
    uint8_t  bec_temp;                  // BEC Temperature in °C
    uint8_t  motor_temp;                // Motor Temperature in °C
    uint8_t  bec_voltage;               // BEC Voltage in 0.1V
    uint8_t  bec_current;               // BEC Current in 0.1A
    uint8_t  reserved1[6];              // reserved
} Pl5TelemetryFrame_t;

static bool pl5CachedDevInfo = false;
static bool pl5CachedParams = false;
static bool pl5DirtyParams = false;

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

static bool pl5ParamCommit(uint8_t cmd)
{
    if (cmd == 0) {
        // info should never change
        if (memcmp(paramPayload, paramUpdPayload, PL5_RESP_DEVINFO_PAYLOAD_LENGTH) != 0)
            return false;

        // params dirty?
        if (memcmp(paramPayload + PL5_RESP_DEVINFO_PAYLOAD_LENGTH,
            paramUpdPayload + PL5_RESP_DEVINFO_PAYLOAD_LENGTH,
            PL5_RESP_GETPARAMS_PAYLOAD_LENGTH) != 0) {
            // set dirty flag, will schedule read
            pl5DirtyParams = true;
            // clear cached flag, will schedule write
            pl5CachedParams = false;
            // invalidate param payload - will be available again when params again cached (write response or re-read)
            paramPayloadLength = 0;
        }

        return true;
    }
    else {
        // unsupported command
        return false;
    }
}

static bool pl5SignSendFrame(uint8_t len, uint16_t framePeriod, uint16_t frameTimeout)
{
    *(uint16_t*)(reqbuffer + len - 2) = calculateCRC16_MODBUS(reqbuffer, len - 2);

    reqLength = len;

    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;

    return true;
}

static bool pl5CopySendFrame(void *req, uint8_t len, uint16_t framePeriod, uint16_t frameTimeout)
{
    if (len > REQUEST_BUFFER_SIZE)
        return false;

    memcpy(reqbuffer, req, len);
    reqLength = len;

    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;

    return true;
}

static void pl5BuildNextReq(void)
{
    // schedule pending param write, schedule request...
    if (pl5DirtyParams) {
        memset(reqbuffer, 0, PL5_REQ_WRITEPARAMS_LENGTH);
        const uint8_t hdrlen = sizeof(pl5WriteParamsReq);
        memcpy(reqbuffer, pl5WriteParamsReq, hdrlen);
        memcpy(reqbuffer + hdrlen, paramUpdPayload + PL5_RESP_DEVINFO_PAYLOAD_LENGTH, PL5_RESP_GETPARAMS_PAYLOAD_LENGTH);
        pl5SignSendFrame(PL5_REQ_WRITEPARAMS_LENGTH, PL5_PARAM_FRAME_PERIOD, PL5_PARAM_WRITE_TIMEOUT);
    }
    // ...or pending device info, schedule request...
    else if (!pl5CachedDevInfo) {
        pl5CopySendFrame(pl5DevInfoReq, sizeof(pl5DevInfoReq), PL5_PARAM_FRAME_PERIOD, PL5_PARAM_READ_TIMEOUT);
    }
    // ...or pending param read, schedule request...
    else if (!pl5CachedParams) {
        pl5CopySendFrame(pl5GetParamsReq, sizeof(pl5GetParamsReq), PL5_PARAM_FRAME_PERIOD, PL5_PARAM_READ_TIMEOUT);
    }
    // ...or nothing, schedule 480ms ping
    else {
        pl5CopySendFrame(pl5Ping, sizeof(pl5Ping), PL5_PING_FRAME_PERIOD, PL5_PING_TIMEOUT);
    }
}

static bool pl5DecodeTeleFrame(timeUs_t currentTimeUs)
{
    const Pl5TelemetryFrame_t *tele = (Pl5TelemetryFrame_t*)(buffer + 9);

    // When throttle changes to zero, the last current reading is
    // repeated until the motor has totally stopped.
    uint16_t current = tele->current;
    if (tele->throttle == 0)
        current = 0;
    setConsumptionCurrent(current * 0.1f);

    escSensorData[0].id = ESC_SIG_PL5;
    escSensorData[0].age = 0;
    escSensorData[0].erpm = tele->rpm * 10;
    escSensorData[0].throttle = tele->throttle * 10;
    escSensorData[0].pwm = tele->throttle * 10;
    escSensorData[0].voltage = applyVoltageCorrection(tele->voltage * 100);
    escSensorData[0].current = applyCurrentCorrection(current * 100);
    escSensorData[0].temperature = tele->temperature * 10;
    escSensorData[0].temperature2 = tele->bec_temp * 10;
    escSensorData[0].bec_voltage = tele->bec_voltage * 100;
    escSensorData[0].bec_current = tele->bec_current * 100;
    escSensorData[0].status = tele->fault;

    DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, tele->rpm * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tele->temperature * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, tele->voltage * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, tele->current * 10);

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, tele->rpm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, tele->throttle);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tele->temperature);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, tele->voltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, tele->current);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tele->bec_voltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

    dataUpdateUs = currentTimeUs;

    rrfsmFrameTimeout = PL5_TELE_FRAME_TIMEOUT;

    // schedule next request (only if halfduplex)
    if (paramMspActive && paramCommit != NULL)
        pl5BuildNextReq();

    return true;
}

static bool pl5DecodePingResp(void)
{
    pl5BuildNextReq();

    paramEscNeedRestart();

    return true;
}

static bool pl5DecodeGetDevInfoResp(void)
{
    // cache device info
    memcpy(paramPayload, buffer + 7, PL5_RESP_DEVINFO_PAYLOAD_LENGTH);
    pl5CachedDevInfo = true;

    pl5BuildNextReq();

    return true;
}

static bool pl5DecodeGetParamsResp(void)
{
    // cache parameters, payload complete
    memcpy(paramPayload + PL5_RESP_DEVINFO_PAYLOAD_LENGTH, buffer + 8, PL5_RESP_GETPARAMS_PAYLOAD_LENGTH);
    pl5CachedParams = true;

    // make param payload available
    paramPayloadLength = PL5_RESP_DEVINFO_PAYLOAD_LENGTH + PL5_RESP_GETPARAMS_PAYLOAD_LENGTH;

    pl5BuildNextReq();

    return true;
}

static bool pl5DecodeWriteParamsResp(void)
{
    if ((buffer[3] & PL5_ERR) == 0) {
        // success, cache parameters
        memcpy(paramPayload + PL5_RESP_DEVINFO_PAYLOAD_LENGTH, buffer + 9, PL5_RESP_GETPARAMS_PAYLOAD_LENGTH);
        pl5CachedParams = true;
    }

    // make param payload available
    paramPayloadLength = PL5_RESP_DEVINFO_PAYLOAD_LENGTH + PL5_RESP_GETPARAMS_PAYLOAD_LENGTH;

    // success or failure don't repeat
    pl5DirtyParams = false;

    pl5BuildNextReq();

    return true;
}

static bool pl5Decode(timeUs_t currentTimeUs)
{
    // check CRC
    const uint16_t crc = *(uint16_t*)(buffer + rrfsmFrameLength - 2);
    if (calculateCRC16_MODBUS(buffer, rrfsmFrameLength - 2) != crc)
        return false;

    // decode
    const uint16_t type = *(uint16_t*)(buffer + 4);
    switch(type) {
        case PL5_FRAME_TELE_TYPE:
            return pl5DecodeTeleFrame(currentTimeUs);
        case PL5_RESP_PING_TYPE:
            return pl5DecodePingResp();
        case PL5_RESP_DEVINFO_TYPE:
            return pl5DecodeGetDevInfoResp();
        case PL5_RESP_GETPARAMS_TYPE:
            return pl5DecodeGetParamsResp();
        case PL5_RESP_WRITEPARAMS_TYPE:
            return pl5DecodeWriteParamsResp();
        default:
            return false;
    }
}

static bool pl5Crank(timeUs_t currentTimeUs)
{
    // Update consumption on each cycle vs each frame in decode
    updateConsumption(currentTimeUs);

    return true;
}

static int8_t pl5Accept(uint16_t c)
{
    if (readBytes == 1) {
        // frame signature
        if (c != 0xFD && c != 0xFE)
            return -1;
    }
    else if (readBytes == 6) {
        // [3: version] [4-5: type]
        uint8_t len = 0;
        const uint16_t type = *(uint16_t*)(buffer + 4);
        switch (type)
        {
            case PL5_FRAME_TELE_TYPE:
                if (buffer[3] == 3)
                    len = PL5_FRAME_TELE_LENGTH;
                break;
            case PL5_RESP_PING_TYPE:
                if (buffer[3] == 3)
                    len = PL5_RESP_PING_LENGTH;
                break;
            case PL5_RESP_DEVINFO_TYPE:
                if (buffer[3] == 3)
                    len = PL5_RESP_DEVINFO_LENGTH;
                break;
            case PL5_RESP_GETPARAMS_TYPE:
                if (buffer[3] == 3)
                    len = PL5_RESP_GETPARAMS_LENGTH;
                break;
            case PL5_RESP_WRITEPARAMS_TYPE:
                if (buffer[3] == 0x17)
                    len = PL5_RESP_WRITEPARAMS_LENGTH;
                else if (buffer[3] == (PL5_ERR|0x17))
                    len = PL5_RESP_WRITEPARAMS_ERR_LENGTH;
                break;
        }
        if (len != 0) {
            rrfsmFrameLength = len;
            return 1;
        }
        return -1;
    }
    return 0;
}

static serialReceiveCallbackPtr pl5SensorInit(bool bidirectional)
{
    rrfsmBootDelayMs = PL5_BOOT_DELAY;
    rrfsmMinFrameLength = PL5_MIN_FRAME_LENGTH;
    rrfsmAccept = pl5Accept;
    rrfsmDecode = pl5Decode;
    rrfsmCrank = pl5Crank;

    escSig = ESC_SIG_PL5;

    // telemetry data only
    rrfsmFrameTimeout = PL5_TELE_FRAME_TIMEOUT;

    if (bidirectional) {
        // enable parameter writes to ESC
        paramCommit = pl5ParamCommit;
    }

    return rrfsmDataReceive;
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
 * 
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

    escSensorData[0].id = ESC_SIG_TRIB;
    escSensorData[0].age = 0;
    escSensorData[0].erpm = rpm * 5;
    escSensorData[0].pwm = power * 5;
    escSensorData[0].voltage = applyVoltageCorrection(voltage * 100);
    escSensorData[0].current = applyCurrentCorrection(current * 100);
    escSensorData[0].consumption = applyConsumptionCorrection(capacity);
    escSensorData[0].temperature = temp * 10;
    escSensorData[0].bec_voltage = voltBEC * 100;
    escSensorData[0].status = status;

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

static bool tribDecode(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

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

static bool tribCrankUncSetup(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;
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
            paramEscNeedRestart();
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

static bool tribStart(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

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
        escSig = ESC_SIG_TRIB;
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

    escSensorData[0].id = ESC_SIG_OPENYGE;
    escSensorData[0].age = 0;
    escSensorData[0].erpm = tele->rpm * 10;
    escSensorData[0].pwm = tele->pwm * 10;
    escSensorData[0].throttle = tele->throttle * 10;
    escSensorData[0].voltage = applyVoltageCorrection(tele->voltage * 10);
    escSensorData[0].current = applyCurrentCorrection(tele->current * 10);
    escSensorData[0].consumption = applyConsumptionCorrection(tele->consumption);
    escSensorData[0].temperature = temp * 10;
    escSensorData[0].temperature2 = tempBEC * 10;
    escSensorData[0].bec_voltage = tele->bec_voltage;
    escSensorData[0].bec_current = tele->bec_current;
    escSensorData[0].status = tele->status1;

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

static bool oygeDecode(timeUs_t currentTimeUs)
{
    // get header w/ CRC check
    const OpenYGEHeader_t *hdr = oygeGetHeaderWithCrcCheck();
    if (hdr == NULL)
        return false;

    timeMs_t currentTimeMs = currentTimeUs / 1000;
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

    escSig = ESC_SIG_OPENYGE;

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
 * Graupner Telemetry
 *
 *    - Serial protocol 19200,8N1
 *    - Little-Endian fields
 *    - Frame length 45 bytes
 *    - Warning flags:
 *          0:  Low voltage
 *          1:  Temp limit exceeded
 *          2:  Motor temp exceeded
 *          3:  Max current exceeded
 *          4:  RPM less than limit
 *          5:  Capacity exceeded
 *          6:  Current limit exceeded
 *
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *    0-1:      Sync header (0x7C 0x8C)
 *      2:      Warning tone
 *      3:      Sensor ID (0xC0)
 *    4-5:      Warning flags
 *    6-7:      Voltage
 *    8-9:      Min Voltage
 *  10-11:      Capacity
 *     12:      Temperature
 *     13:      Max temp
 *  14-15:      Current
 *  15-16:      Max Current
 *  17-18:      RPM
 *  19-20:      Max RPM
 *  21-42:      Reserved
 *     43:      End byte (0x7D)
 *     44:      Checksum
 *
 */

#define GRAUPNER_MIN_FRAME_LENGTH                45
#define GRAUPNER_BOOT_DELAY                      5000
#define GRAUPNER_TELE_FRAME_PERIOD               10
#define GRAUPNER_TELE_FRAME_TIMEOUT              200

static uint8_t graupnerTeleReq[] = { 0x80, 0x8C };

typedef struct {
    uint8_t     tone;
    uint8_t     sensor;
    uint16_t    flags;
    uint16_t    voltage;
    uint16_t    voltage_min;
    uint16_t    capacity;
    int8_t      temperature;
    int8_t      temperature_max;
    uint16_t    current;
    uint16_t    current_max;
    uint16_t    rpm;
    uint16_t    rpm_max;
} __packed GraupnerTelemetryFrame_t;


static uint8_t graupnerCalculateChecksum(uint8_t *ptr, size_t length)
{
    uint8_t sum = 0;

    while (length-- > 0)
        sum += *ptr++;

    return sum;
}

static bool graupnerCopySendFrame(void *req, size_t len, uint16_t framePeriod, uint16_t frameTimeout)
{
    if (len > REQUEST_BUFFER_SIZE)
        return false;

    memcpy(reqbuffer, req, len);
    reqLength = len;

    rrfsmFramePeriod = framePeriod;
    rrfsmFrameTimeout = frameTimeout;

    return true;
}

static void graupnerBuildNextReq(void)
{
    graupnerCopySendFrame(graupnerTeleReq, sizeof(graupnerTeleReq), GRAUPNER_TELE_FRAME_PERIOD, GRAUPNER_TELE_FRAME_TIMEOUT);
}

static bool graupnerDecodeTeleFrame(timeUs_t currentTimeUs)
{
    const GraupnerTelemetryFrame_t *tele = (GraupnerTelemetryFrame_t*)(buffer + 2);

    escSensorData[0].id = ESC_SIG_GRAUPNER;
    escSensorData[0].age = 0;
    escSensorData[0].erpm = tele->rpm * 10;
    escSensorData[0].voltage = applyVoltageCorrection(tele->voltage * 100);
    escSensorData[0].current = applyCurrentCorrection(tele->current * 100);
    escSensorData[0].consumption = applyConsumptionCorrection(tele->capacity * 10);
    escSensorData[0].temperature = tele->temperature * 10 - 200;
    escSensorData[0].status = tele->flags;

    DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, tele->rpm * 10);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tele->temperature * 10 - 200);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, tele->voltage * 100);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, tele->current * 100);

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, tele->rpm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tele->temperature);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, tele->voltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, tele->current);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, tele->capacity);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tele->flags);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

    dataUpdateUs = currentTimeUs;

    return true;
}

static bool graupnerDecode(timeUs_t currentTimeUs)
{
    const uint8_t framesum = graupnerCalculateChecksum(buffer, 44);
    const uint8_t checksum = buffer[44];
    bool ret = false;

    if (checksum == framesum) {
        ret = graupnerDecodeTeleFrame(currentTimeUs);
    }

    graupnerBuildNextReq();

    return ret;
}

static int8_t graupnerAccept(uint16_t c)
{
    if (readBytes == 1) {
        // frame signature
        if (c != 0x7C)
            return -1;
    }
    else if (readBytes == 2) {
        // frame signature
        if (c != 0x8C)
            return -1;
    }
    else if (readBytes == 4) {
        // sensor id
        if (c != 0xC0)
            return -1;
    }
    else if (readBytes == 44) {
        // end byte
        if (c != 0x7D)
            return -1;
    }

    return 0;
}

static bool graupnerStart(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    graupnerBuildNextReq();
    return true;
}

static serialReceiveCallbackPtr graupnerSensorInit(void)
{
    rrfsmBootDelayMs = GRAUPNER_BOOT_DELAY;
    rrfsmMinFrameLength = GRAUPNER_MIN_FRAME_LENGTH;
    rrfsmAccept = graupnerAccept;
    rrfsmDecode = graupnerDecode;
    rrfsmStart = graupnerStart;

    escSig = ESC_SIG_GRAUPNER;

    return rrfsmDataReceive;
}

/*
 * XDFLY
 *
 *     - Serial protocol is 115200,8N1
 *     - Big-Endian byte order
 * 
 * Frame Format
 * ――――――――――――――――――――――――――――――――――――――――――――――――――――――――
 *      0:      start byte (0xA5)
 *      1:      length in bytes (8)
 *      2:      command 
 *          0x03 handshake
 *          0x34 set_par
 *          0x33 get_par
 *      3:      param number
 *      4-5:    param value
 *      6-7:      CRC16
 * 
 * 
 * checking whether the param is valid for the connected ESC:
 *      if the highest bit of the value is set, the parameter is valid for the connected ESC
 *          bool paramValid = paramValueRaw & 0x8000 != 0 //param shall not be displayed
 *      the actual data is in the lower 15 bits
 *          uint16_t paramValue =  paramValueRaw & 0x7FFF
 * 
 * for setting a param the high bit of the value is set to 1
 *     uint16_t paramValueRaw = paramValue | 0x8000
 * 
 * when setting a param and the ESC responds with 0xFFFF, setting the param was not successful
 */
#define XDFLY_SYNC                          0xA5                // start byte
#define XDFLY_HEADER_LENGTH                 3                   // header length
#define XDFLY_PAYLOAD_LENGTH                3
#define XDFLY_FOOTER_LENGTH                 2                   // CRC
#define XDFLY_FRAME_LENGTH                  (XDFLY_HEADER_LENGTH + XDFLY_PAYLOAD_LENGTH + XDFLY_FOOTER_LENGTH)
#define XDFLY_BOOT_DELAY                    5000

#define XDFLY_SYNC_TELEM                    0xDD
#define XDFLY_TELEM_VERSION                 0x01
#define XDFLY_TELEM_FRAME_LENGTH            0x20
#define XDFLY_PARAM_FRAME_PERIOD            10
#define XDFLY_PARAM_FRAME_TIMEOUT           200

#define XDFLY_CMD_HANDSHAKE                 0x03
#define XDFLY_CMD_RESPONSE                  0xCD
#define XDFLY_CMD_SET_PARAM                 0x34
#define XDFLY_CMD_GET_PARAM                 0x33
#define XDFLY_NUM_VALIDITY_FIELDS           2

 enum {
    XDFLY_PARAM_MODEL = 0,
    XDFLY_PARAM_GOV_MODE,
    XDFLY_PARAM_CUTOFF,
    XDFLY_PARAM_TIMING,
    XDFLY_PARAM_LV_BEC_VOLT,
    XDFLY_PARAM_ROTATION_DIR,
    XDFLY_PARAM_GOV_P,
    XDFLY_PARAM_GOV_I,
    XDFLY_PARAM_ACCEL,
    XDFLY_PARAM_AUTO_RESTART_TIME,
    XDFLY_PARAM_HV_BEC_VOLT,
    XDFLY_PARAM_RSTARTUP_POWER,
    XDFLY_PARAM_BRAKE_TYPE,
    XDFLY_PARAM_RBRAKE_FORCE,
    XDFLY_PARAM_SR_FUNC,
    XDFLY_PARAM_CAPACITY_CORR,
    XDFLY_PARAM_MOTOR_POLES,
    XDFLY_PARAM_LED_COL,
    XDFLY_PARAM_SMART_FAN,
    XDFLY_VALID_1,
    XDFLY_VALID_2,
    XDFLY_PARAM_COUNT
};

uint16_t xdfly_params[XDFLY_PARAM_COUNT] = {0};
bool xdfly_params_active[XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS] = {0};
bool xdfly_param_cached[XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS] = {0};

enum xdfly_setup_status {
    XDFLY_INIT = 0,
    XDFLY_CACHE_PARAMS,
    XDFLY_PARAMS_READY,
    XDFLY_WAIT_FOR_HANDSHAKE,
    XDFLY_WRITE_PARAMS,
};

static enum xdfly_setup_status xdfly_setup_status = XDFLY_INIT;

static bool xdfly_connected = false;
static bool xdfly_send_handshake = false;
static bool xdfly_handshake_response_pending = false;
static timeMs_t xdfly_handshake_timestamp = 0;
static uint8_t xdfly_param_index = 0;
static uint8_t xdfly_write_param_index = 0;

static void xdfly_sensor_process(timeUs_t current_time_us)
{
    if (buffer[1] == 0x01 && buffer[2] == 0x20) {
        uint16_t rpm = buffer[8] << 8 | buffer[9];
        uint16_t temp = buffer[10];
        uint16_t throttle = buffer[7];
        uint16_t power = buffer[12];
        uint16_t voltage = buffer[3] << 8 | buffer[4];
        uint16_t current = buffer[5] << 8 | buffer[6];
        uint16_t capacity = buffer[15] << 8 | buffer[16];
        uint16_t status = buffer[13] << 8 | buffer[14];
        uint16_t volt_bec = buffer[19];

        escSensorData[0].id = ESC_SIG_XDFLY;
        escSensorData[0].age = 0;
        escSensorData[0].erpm = rpm * 10;
        escSensorData[0].throttle = throttle * 10;
        escSensorData[0].pwm = power * 10;
        escSensorData[0].voltage = applyVoltageCorrection(voltage * 100);
        escSensorData[0].current = applyCurrentCorrection(current * 100);
        escSensorData[0].consumption = applyConsumptionCorrection(capacity);
        escSensorData[0].temperature = temp * 10;
        escSensorData[0].bec_voltage = volt_bec * 1000;
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

        dataUpdateUs = current_time_us;
        totalFrameCount++;
    }
    checkFrameTimeout(current_time_us, 500000);
}

static void xdfly_build_req(uint8_t cmd, uint8_t param_idx, uint16_t frame_period, uint16_t frame_timeout)
{
    uint8_t idx = 0;

    reqbuffer[idx++] = XDFLY_SYNC;
    reqbuffer[idx++] = XDFLY_FRAME_LENGTH;
    reqbuffer[idx++] = cmd;

    if (cmd == XDFLY_CMD_HANDSHAKE) {
        reqbuffer[idx++] = 0;
        reqbuffer[idx++] = 0;
        reqbuffer[idx++] = 0;
    } else if (cmd == XDFLY_CMD_GET_PARAM) {
        reqbuffer[idx++] = param_idx;
        reqbuffer[idx++] = 0;
        reqbuffer[idx++] = 0;
    } else if (cmd == XDFLY_CMD_SET_PARAM) {
        reqbuffer[idx++] = param_idx;
        reqbuffer[idx++] = paramUpdPayload[param_idx * 2 + 1] | 0x80;
        reqbuffer[idx++] = paramUpdPayload[param_idx * 2];
    }

    uint8_t crclen = idx;
    uint16_t crc16 = calculateCRC16_MODBUS(reqbuffer, crclen);
    reqbuffer[idx++] = crc16 >> 8;
    reqbuffer[idx++] = crc16 & 0xFF;

    reqLength = idx;
    rrfsmFramePeriod = frame_period;
    rrfsmFrameTimeout = frame_timeout;
}

static void xdfly_get_next_param(void)
{
    if (!xdfly_param_cached[xdfly_param_index])
        xdfly_build_req(XDFLY_CMD_GET_PARAM, xdfly_param_index, 1, 0);
}

static void xdfly_write_next_param(void)
{
    xdfly_build_req(XDFLY_CMD_SET_PARAM, xdfly_write_param_index, 1, 0);
}

static void xdfly_start_caching_params(void)
{
    xdfly_setup_status = XDFLY_CACHE_PARAMS;
    xdfly_param_index = 0;

    for (uint8_t i = 0; i < XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS; i++) {
        xdfly_param_cached[i] = false;
        xdfly_params_active[i] = false;
    }

    xdfly_params[XDFLY_VALID_1] = 0;
    xdfly_params[XDFLY_VALID_2] = 0;
    xdfly_get_next_param();
}

static bool xdfly_decode(timeUs_t current_time_us)
{
    if (buffer[0] == XDFLY_SYNC) {
        if (buffer[1] != XDFLY_FRAME_LENGTH)
            return false;

        uint16_t crc16 = buffer[6] << 8 | buffer[7];
        if (calculateCRC16_MODBUS(buffer, XDFLY_FRAME_LENGTH - XDFLY_FOOTER_LENGTH) != crc16)
            return false;

        switch (buffer[2]) {
        case XDFLY_CMD_HANDSHAKE:
        case XDFLY_CMD_RESPONSE:
            if (buffer[3] == 0x00 && buffer[4] == 0x55 && buffer[5] == 0x55) {
                rrfsmFramePeriod = 0;
                xdfly_handshake_timestamp = current_time_us;
                xdfly_handshake_response_pending = false;
                xdfly_connected = true;
                rrfsmInvalidateReq();
                xdfly_start_caching_params();
                return true;
            } else if (xdfly_setup_status == XDFLY_WRITE_PARAMS) {
                if (buffer[3] == xdfly_write_param_index) {
                    xdfly_write_param_index++;
                    while (xdfly_write_param_index < XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS &&
                           !xdfly_params_active[xdfly_write_param_index]) {
                        xdfly_write_param_index++;
                    }
                }
                if (xdfly_write_param_index >= XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS) {
                    xdfly_start_caching_params();
                } else {
                    xdfly_write_next_param();
                }
                return true;
            } else if (buffer[3] == xdfly_param_index || xdfly_param_index == 0) {
                xdfly_param_cached[xdfly_param_index] = true;
                xdfly_params[xdfly_param_index] = (buffer[4] << 8 | buffer[5]) & 0x7FFF;
                xdfly_params_active[xdfly_param_index] = buffer[4] & 0x80;
                xdfly_param_index++;

                if (xdfly_param_index >= XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS) {
                    for (uint8_t i = 0; i < XDFLY_PARAM_COUNT - XDFLY_NUM_VALIDITY_FIELDS; i++) {
                        if (i < 16) {
                            xdfly_params[XDFLY_VALID_1] |= xdfly_params_active[i] << i;
                        } else {
                            xdfly_params[XDFLY_VALID_2] |= xdfly_params_active[i] << (i - 16);
                        }
                    }
                    xdfly_setup_status = XDFLY_PARAMS_READY;
                    paramPayloadLength = XDFLY_PARAM_COUNT * 2;
                    memcpy(paramPayload, xdfly_params, paramPayloadLength);
                } else {
                    xdfly_get_next_param();
                }
                return true;
            }
            break;

        default:
            return false;
        }
    } else if (buffer[0] == XDFLY_SYNC_TELEM) {
        if (buffer[1] != XDFLY_TELEM_VERSION)
            return false;

        if (buffer[2] != XDFLY_TELEM_FRAME_LENGTH)
            return false;

        xdfly_sensor_process(current_time_us);

        if (xdfly_setup_status == XDFLY_CACHE_PARAMS) {
            xdfly_get_next_param();
        } else if (xdfly_setup_status == XDFLY_WRITE_PARAMS) {
            xdfly_write_next_param();
        }
        return true;
    }
    return false;
}

static bool xdfly_param_commit(uint8_t cmd)
{
    xdfly_write_param_index = 1;
    xdfly_setup_status = XDFLY_WRITE_PARAMS;
    UNUSED(cmd);
    return true;
}

static bool xdfly_crank_unc_setup(timeUs_t current_time_us)
{
    switch (xdfly_setup_status) {
    case XDFLY_INIT:
        xdfly_sensor_process(current_time_us);
        if (!xdfly_connected) {
            xdfly_setup_status = XDFLY_WAIT_FOR_HANDSHAKE;
            rrfsmFrameLength = XDFLY_FRAME_LENGTH;
            rrfsmMinFrameLength = XDFLY_FRAME_LENGTH;
            xdfly_send_handshake = true;
        }
        break;

    case XDFLY_WAIT_FOR_HANDSHAKE:
        if (current_time_us > xdfly_handshake_timestamp + 10000 && xdfly_handshake_response_pending) {
            xdfly_send_handshake = true;
        }
        if (xdfly_send_handshake) {
            xdfly_send_handshake = false;
            xdfly_handshake_response_pending = true;
            xdfly_build_req(XDFLY_CMD_HANDSHAKE, 0, XDFLY_PARAM_FRAME_PERIOD, FLY_PARAM_CONNECT_TIMEOUT);
            rrfsmFrameLength = XDFLY_FRAME_LENGTH;
            rrfsmMinFrameLength = XDFLY_FRAME_LENGTH;
        }
        break;

    case XDFLY_CACHE_PARAMS:
        break;

    case XDFLY_PARAMS_READY:
        break;

    case XDFLY_WRITE_PARAMS:
        break;
    }
    return true;
}

static int8_t xdfly_accept(uint16_t c)
{
    static uint8_t got_telem_frame = false;

    if (readBytes == 1) {
        if (c == XDFLY_SYNC_TELEM) {
            got_telem_frame = true;
        } else if (c == XDFLY_SYNC) {
            got_telem_frame = false;
        } else {
            return -1;
        }
    } else if (readBytes == 2) {
        if (got_telem_frame) {
            if (c != XDFLY_TELEM_VERSION)
                return -1;
        } else {
            if (c != XDFLY_FRAME_LENGTH)
                return -1;
            rrfsmFrameLength = c;
            return 1;
        }
    } else if (readBytes == 3) {
        if (got_telem_frame) {
            if (c != XDFLY_TELEM_FRAME_LENGTH) {
                return -1;
            } else {
                rrfsmFrameLength = c;
                return 1;
            }
        }
    }
    return 0;
}

static serialReceiveCallbackPtr xdflySensorInit(void)
{
    rrfsmCrank = xdfly_crank_unc_setup;
    rrfsmDecode = xdfly_decode;
    rrfsmAccept = xdfly_accept;
    paramCommit = xdfly_param_commit;
    escSig = ESC_SIG_XDFLY;

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


/*
 * Castle Live Link Telemetry Data Recorder
 */

#define CASTLE_BATTERY_VOLTAGE_SCALE 20000.0f // in mV, not volts
#define CASTLE_RIPPLE_VOLTAGE_SCALE 4000.0f  // in mV, not volts
#define CASTLE_BATTERY_CURRENT_SCALE 50000.0f // in mA, not amps
#define CASTLE_THROTTLE_SCALE 1.0f // in ms.
// Note the doc for POWER_SCALE says the unit is percent for scale 0.2502,
// but that actually puts it in a 0..1 range, not 0..100.
#define CASTLE_POWER_SCALE 250.2f // in per-thousands (mils)
#define CASTLE_RPM_SCALE 20416.7f
#define CASTLE_BEC_VOLTAGE_SCALE 4000.0f  // in mV, not volts
#define CASTLE_BEC_CURRENT_SCALE 4000.0f  // in mA, not amps
#define CASTLE_LIN_TEMP_SCALE 30.0f // in degrees C
// CASTLE_NTC_TEMP_SCALE is modified from the Castle-documented value of 63.8125
// to match the range of 1-4096 used by calcTempNTC() instead of 1-255 as assumed by Castle.
#define CASTLE_NTC_TEMP_SCALE 1025.0039215686f

// Castle Link telemetry is not digital; it's the length of time between the rising edge of the
// output (inverted) pwm and the falling edge of the input "tick" generated by the ESC.  To
// calibrate this time, each frame includes minimum-length (0.5ms) and 1ms calibration items.
// The "max" below is required because a data item could be smaller than the 0.5ms calibration
// item due to measurement error.
#define CASTLE_DECODE_1(tele, item, scale, cal0p5) \
    ((MAX(tele->item, cal0p5) - cal0p5) * scale / tele->oneMs)

#define CASTLE_DECODE(tele, item, scale, cal0p5) \
    CASTLE_DECODE_1(tele, item, CASTLE_##scale##_SCALE, cal0p5)

/* Temp sensor design for Castle ESCs equipped with an NTC sensor:
 * ―――――――――――――――――――
 *  β  = 3455
 *  Rᵣ = 10.2k
 *  Rₙ = 10k
 *  Tₙ = 25°C = 298.15K
 *
 *  γ = 1 / β = 0.0002894…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = γ⋅ln(102/100) + 1/298.15 = 0.0033597…
 *
 * Note Castle's official values are Tₙ = 24.85°C and Tₖ = 273K.  These are unlikely to be
 * correct. The discrepency with the official calculation is only about 0.2 degrees at the upper end
 * of the temperature range (and less elsewhere).
 */
#define CASTLE_NTC_GAMMA  0.0002894356005f
#define CASTLE_NTC_DELTA  0.0033597480200f

#ifdef USE_TELEMETRY_CASTLE
static float castleDecodeTemperature(castleTelemetry_t* tele)
{
    if (tele->linTempOrHalfMs < tele->ntcTempOrHalfMs) {
        float value = CASTLE_DECODE(tele, ntcTempOrHalfMs, NTC_TEMP, tele->linTempOrHalfMs);
        return calcTempNTC(value, CASTLE_NTC_GAMMA, CASTLE_NTC_DELTA);
    } else {
        return CASTLE_DECODE(tele, linTempOrHalfMs, NTC_TEMP, tele->ntcTempOrHalfMs);
    }
}

static void castleDecodeTeleFrame(timeUs_t currentTimeUs, castleTelemetry_t* tele)
{
    // The half-millisecond (minimum) calibration value is in the same
    // field as whichever temperature sensor is not installed, so we
    // find it by choosing the lesser of the two temperature fields.
    uint16_t halfMs = tele->linTempOrHalfMs;
    bool linTemp = false;
    if (tele->ntcTempOrHalfMs < halfMs) {
        linTemp = true;
        halfMs = tele->ntcTempOrHalfMs;
    }

    // We might want to use the configured min/max throttle values to create the telemetry
    // per-1000 value instead of the standard 1ms-2ms.
    float throttleMs = constrainf(CASTLE_DECODE(tele, throttle, THROTTLE, halfMs), 1.0, 2.0);
    uint16_t throttle = lrintf((throttleMs - 1.0) * 1000.0);
    uint16_t pwm = lrintf(CASTLE_DECODE(tele, outputPower, POWER, halfMs));
    uint32_t rpm = lrintf(CASTLE_DECODE(tele, rpm, RPM, halfMs));
    uint32_t voltage = lrintf(CASTLE_DECODE(tele, battVoltage, BATTERY_VOLTAGE, halfMs));
    uint32_t current = lrintf(CASTLE_DECODE(tele, battCurrent, BATTERY_CURRENT, halfMs));
    uint32_t becVoltage = lrintf(CASTLE_DECODE(tele, becVoltage, BEC_VOLTAGE, halfMs));
    uint32_t becCurrent = lrintf(CASTLE_DECODE(tele, becCurrent, BEC_CURRENT, halfMs));
    int16_t temperature = lrintf(castleDecodeTemperature(tele) * 10.0);

    escSensorData[0].id = ESC_SIG_CASTLE;
    escSensorData[0].age = 0;
    escSensorData[0].throttle = throttle;
    escSensorData[0].pwm = pwm;
    escSensorData[0].erpm = rpm;
    escSensorData[0].voltage = applyVoltageCorrection(voltage);
    escSensorData[0].current = applyCurrentCorrection(current);
    escSensorData[0].bec_voltage = becVoltage;
    escSensorData[0].bec_current = becCurrent;
    escSensorData[0].temperature = temperature;

    setConsumptionCurrent(current / 1000.0);

    DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temperature);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
    DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current);

    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, tele->rpm);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, tele->outputPower);
    if (linTemp) {
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tele->linTempOrHalfMs);
    } else {
        DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tele->ntcTempOrHalfMs);
    }
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, tele->battVoltage);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, tele->battCurrent);
    // We don't use DEBUG_DATA_CAPACITY here, because it is not in the Castle Link telemetry.
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tele->throttle);
    DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

    dataUpdateUs = currentTimeUs;
}

static void castleSensorProcess(timeUs_t currentTimeUs)
{
    // buffer[0..1] holds our current generation, then the next 22 bytes hold
    // the telemetry value
    castleTelemetry_t *rawTelemetry = (castleTelemetry_t*)&buffer[2];
    getCastleTelemetry(rawTelemetry);
    if (rawTelemetry->generation == *(uint16_t*)buffer) {
        // We expect every 12 PWM frames we'll get new data.  Max frame
        // length is 20ms for 50Hz, and we allow one frame extra.
        checkFrameTimeout(currentTimeUs,
                          (CASTLE_TELEM_NFRAMES + 1) *
                          CASTLE_PWM_PERIOD_MS_MAX * 1000);
        return;
    }
    dataUpdateUs = currentTimeUs;
    *(uint16_t*)buffer = rawTelemetry->generation;
    castleDecodeTeleFrame(currentTimeUs, rawTelemetry);
    updateConsumption(currentTimeUs);
}
#endif // USE_TELEMETRY_CASTLE

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
                rrfsmSensorProcess(currentTimeUs);
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
            case ESC_SENSOR_PROTO_FLY:
                rrfsmSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_GRAUPNER:
                rrfsmSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_XDFLY:
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
#ifdef USE_TELEMETRY_CASTLE
    else if (isMotorProtocolCastlePWM()) {
        castleSensorProcess(currentTimeUs);
    }
#endif
}

void INIT_CODE validateAndFixEscSensorConfig(void)
{
    switch (escSensorConfig()->protocol) {
        case ESC_SENSOR_PROTO_GRAUPNER:
        case ESC_SENSOR_PROTO_XDFLY:
            escSensorConfigMutable()->halfDuplex = true;
            break;
#ifdef USE_TELEMETRY_CASTLE
        case ESC_SENSOR_PROTO_NONE:
            if (isMotorProtocolCastlePWM()) {
                escSensorConfigMutable()->update_hz = motorConfig()->dev.motorPwmRate / CASTLE_TELEM_NFRAMES;
            }
            break;
#endif
        default:
            break;
    }
}

void INIT_CODE escSensorCommonInit(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].age = ESC_DATA_INVALID;
    }

    escSensorDataCombined.age = ESC_DATA_INVALID;
}

bool INIT_CODE escSensorInit(void)
{
    const bool escHalfDuplex = escSensorConfig()->halfDuplex;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    serialReceiveCallbackPtr callback = NULL;
    portOptions_e options = 0;
    uint32_t baudrate = 0;

    if (isMotorProtocolCastlePWM() && (!portConfig || escSensorConfig()->protocol == ESC_SENSOR_PROTO_NONE)) {
        escSensorCommonInit();
        return true;
    }
    if (!portConfig) {
        return false;
    }

    options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED |
        (escHalfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (escSensorConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

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
        case ESC_SENSOR_PROTO_APD:
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_HW5:
            callback = pl5SensorInit(escHalfDuplex);
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_OPENYGE:
            callback = oygeSensorInit(escHalfDuplex);
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_FLY:
            callback = flySensorInit(escHalfDuplex);
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_GRAUPNER:
            callback = graupnerSensorInit();
            baudrate = 19200;
            break;
        case ESC_SENSOR_PROTO_XDFLY:
            callback = xdflySensorInit();
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_RECORD:
            baudrate = baudRates[portConfig->telemetry_baudrateIndex];
            break;
    }

    if (baudrate) {
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, callback, NULL, baudrate, escHalfDuplex ? MODE_RXTX : MODE_RX, options);
    }

    escSensorCommonInit();
    return (escSensorPort != NULL);
}


uint8_t escGetParamBufferLength(void)
{
    paramMspActive = true;
    return paramPayloadLength != 0 ? PARAM_HEADER_SIZE + paramPayloadLength : 0;
}

uint8_t *escGetParamBuffer(void)
{
    paramBuffer[PARAM_HEADER_SIG] = escSig;
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
