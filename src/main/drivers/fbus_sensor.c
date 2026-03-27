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

#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "pg/fbus_master.h"
#include "pg/gps.h"

#include "common/maths.h"
#include "config/feature.h"
#include "drivers/time.h"
#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "drivers/fbus_sensor.h"

#ifdef USE_FBUS_MASTER

static const char* const fbusSensorNames[] = {
    [FBUS_SENSOR_VARIO2]        = "VARIO2",
    [FBUS_SENSOR_FLVSS]         = "FLVSS",
    [FBUS_SENSOR_CURRENT]       = "CURRENT",
    [FBUS_SENSOR_GPS]           = "GPS",
    [FBUS_SENSOR_RPM]           = "RPM",
    [FBUS_SENSOR_SP2UART_A]     = "SP2UART_A",
    [FBUS_SENSOR_SP2UART_B]     = "SP2UART_B",
    [FBUS_SENSOR_FAS_150S]      = "FAS_150S",
    [FBUS_SENSOR_SBEC]          = "SBEC",
    [FBUS_SENSOR_AIR_SPEED]     = "AIR_SPEED",
    [FBUS_SENSOR_ESC]           = "ESC",
    [FBUS_SENSOR_PSD_30]        = "PSD_30",
    [FBUS_SENSOR_XACT_SERVO]    = "XACT_SERVO",
    [FBUS_SENSOR_SD1]           = "SD1",
    [FBUS_SENSOR_VS600]         = "VS600",
    [FBUS_SENSOR_GASSUIT]       = "GASSUIT",
    [FBUS_SENSOR_FSD]           = "FSD",
    [FBUS_SENSOR_GATEWAY]       = "GATEWAY",
    [FBUS_SENSOR_REDUNDANCY_BUS] = "REDUNDANCY_BUS",
    [FBUS_SENSOR_S6R]           = "S6R",
};

static fbusGpsData_t fbusGps;
static fbusServoData_t fbusServo;
static fbusCurrentData_t fbusCurrent;
static fbusEscData_t fbusEsc;

#define FBUS_SENSOR_CACHE_SIZE 32
static fbusSensorData_t sensorCache[FBUS_SENSOR_CACHE_SIZE];
static uint8_t sensorCacheIndex = 0;
static fbusObservedSensor_t observedSensors[FBUS_MAX_OBSERVED_SENSORS];
static uint8_t observedSensorCount = 0;
static fbusSensorForwardBuffer_t forwardBuffers[FBUS_MASTER_MAX_FORWARDED_SENSORS];

void fbusSensorInit(void)
{
    memset(&fbusGps, 0, sizeof(fbusGpsData_t));
    memset(&fbusServo, 0, sizeof(fbusServoData_t));
    memset(&fbusCurrent, 0, sizeof(fbusCurrentData_t));
    memset(&fbusEsc, 0, sizeof(fbusEscData_t));
    memset(sensorCache, 0, sizeof(sensorCache));
    sensorCacheIndex = 0;
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
    
    fbusSensorInitForwarding();
}

int32_t fbusGpsConvertLatLon(uint32_t fbusData)
{
    bool isNegative = (fbusData & FBUS_GPS_LAT_LON_BIT30) != 0;
    uint32_t minutesTimes10000 = fbusData & FBUS_GPS_LAT_LON_DATA_MASK;
    
    float minutes = (float)minutesTimes10000 / 10000.0f;
    float degrees = minutes / 60.0f;
    
    int32_t result = (int32_t)(degrees * 1e7f);
    
    if (isNegative) {
        result = -result;
    }
    
    return result;
}

int32_t fbusGpsConvertAltitude(uint32_t fbusData)
{
    return (int32_t)fbusData;
}

uint16_t fbusGpsConvertSpeed(uint32_t fbusData)
{
    float knots = (float)fbusData / 1000.0f;
    float metersPerSec = knots * 0.514444f;
    uint16_t result = (uint16_t)(metersPerSec * 10.0f);
    
    return result;
}

uint16_t fbusGpsConvertCourse(uint32_t fbusData)
{
    uint16_t result = (uint16_t)(fbusData / 10);
    
    if (result >= 3600) {
        result = result % 3600;
    }
    
    return result;
}

bool fbusGpsConvertTime(uint32_t fbusData, fbusGpsTime_t *time)
{
    if (time) {
        memset(time, 0, sizeof(*time));
    }

    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;

    if (type == FBUS_GPS_TIME_TYPE_TIME && time) {
        time->seconds = (fbusData >> 16) & 0xFF;
        time->minutes = (fbusData >> 8) & 0xFF;
        time->hours = fbusData & 0xFF;
        return true;
    }

    return false;
}

bool fbusGpsConvertDate(uint32_t fbusData, fbusGpsDate_t *date)
{
    if (date) {
        memset(date, 0, sizeof(*date));
    }

    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;

    if (type == FBUS_GPS_TIME_TYPE_DATE && date) {
        date->day = (fbusData >> 16) & 0xFF;
        date->month = (fbusData >> 8) & 0xFF;
        date->year = 2000 + (fbusData & 0xFF);  // Assuming year is offset from 2000
        return true;
    }

    return false;
}

void fbusServoConvertData(uint32_t fbusData, uint16_t *current, uint16_t *voltage, uint16_t *temperature)
{
    *current = (fbusData & FBUS_SERVO_CURRENT_MASK);
    *voltage = ((fbusData & FBUS_SERVO_VOLTAGE_MASK) >> 8);
    *temperature = ((fbusData & FBUS_SERVO_TEMP_MASK) >> 16);
}

static fbusDetectedSensorType_e classifySensorTypeByAppId(uint16_t appId)
{
    // FrSky S.Port protocol: Application ID and physical ID are independent.
    // Classify by App ID signatures/ranges so sensors remain detectable even
    // when physical IDs are reconfigured.

    // GPS signature block: 0x0800, 0x0820, 0x0830, 0x0840, 0x0850 (+ low nibble group)
    if ((appId >= FBUS_GPS_LATITUDE_BASE && appId <= (FBUS_GPS_LATITUDE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_ALTITUDE_BASE && appId <= (FBUS_GPS_ALTITUDE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_SPEED_BASE && appId <= (FBUS_GPS_SPEED_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_COURSE_BASE && appId <= (FBUS_GPS_COURSE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_TIME_BASE && appId <= (FBUS_GPS_TIME_BASE + 0x0F))) {
        return FBUS_DETECTED_SENSOR_GPS;
    }

    // XACT Servo signature block: 0x6800~0x680F
    if (appId >= FBUS_SERVO_DATA_BASE && appId <= (FBUS_SERVO_DATA_BASE + 0x0F)) {
        return FBUS_DETECTED_SENSOR_XACT_SERVO;
    }

    // ESC signature blocks from FrSky S.Port v3.5.8 table:
    // 0x0B50~0x0B5F (ESC_V/ESC_C), 0x0B60~0x0B6F (RPM/Consumption), 0x0B70~0x0B7F (Temp)
    if ((appId >= FBUS_ESC_POWER_BASE && appId <= (FBUS_ESC_POWER_BASE + 0x0F)) ||
        (appId >= FBUS_ESC_RPM_CONS_BASE && appId <= (FBUS_ESC_RPM_CONS_BASE + 0x0F)) ||
        (appId >= FBUS_ESC_TEMP_BASE && appId <= (FBUS_ESC_TEMP_BASE + 0x0F))) {
        return FBUS_DETECTED_SENSOR_ESC;
    }

    // FAS current/voltage sensor signatures seen in field captures and FrSky IDs:
    // current 0x0200~0x020F, voltage 0x0210~0x021F, high-precision current 0x0220~0x022F
    if ((appId >= 0x0200 && appId <= 0x020F) ||
        (appId >= 0x0210 && appId <= 0x021F) ||
        (appId >= 0x0220 && appId <= 0x022F)) {
        return FBUS_DETECTED_SENSOR_FAS_150S;
    }

    return FBUS_DETECTED_SENSOR_UNKNOWN;
}

static fbusObservedSensor_t* getObservedSensorByPhysicalId(uint8_t physicalId)
{
    for (uint8_t i = 0; i < observedSensorCount; i++) {
        if (observedSensors[i].physicalId == physicalId) {
            return &observedSensors[i];
        }
    }
    return NULL;
}

static fbusObservedSensor_t* trackObservedSensor(uint8_t physicalId, uint16_t appId, timeUs_t currentTimeUs)
{
    // Find existing sensor or add new one
    fbusObservedSensor_t *sensor = getObservedSensorByPhysicalId(physicalId);
    if (!sensor && observedSensorCount < FBUS_MAX_OBSERVED_SENSORS) {
        sensor = &observedSensors[observedSensorCount++];
        sensor->physicalId = physicalId;
        sensor->appIdCount = 0;
        sensor->packetCount = 0;
        sensor->detectedType = FBUS_DETECTED_SENSOR_UNKNOWN;
    }

    if (sensor) {
        sensor->lastSeenUs = currentTimeUs;
        sensor->packetCount++;

        bool appIdExists = false;
        for (uint8_t i = 0; i < sensor->appIdCount; i++) {
            if (sensor->appIds[i] == appId) {
                appIdExists = true;
                break;
            }
        }

        if (!appIdExists && sensor->appIdCount < 16) {
            sensor->appIds[sensor->appIdCount++] = appId;
        }

        if (sensor->detectedType == FBUS_DETECTED_SENSOR_UNKNOWN) {
            const fbusDetectedSensorType_e detected = classifySensorTypeByAppId(appId);
            if (detected != FBUS_DETECTED_SENSOR_UNKNOWN) {
                sensor->detectedType = detected;
            }
        }
    }

    return sensor;
}

void fbusSensorInitForwarding(void)
{
    memset(forwardBuffers, 0, sizeof(forwardBuffers));

    // Initialize buffer for each configured forwarded sensor
    for (uint8_t i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
        uint8_t physicalId = fbusMasterConfig()->forwardedSensors[i];
        forwardBuffers[i].physicalId = (physicalId <= FBUS_MAX_PHYS_ID) ? physicalId : FBUS_INVALID_PHYSICAL_ID;
        forwardBuffers[i].writeIndex = 0;
        forwardBuffers[i].readIndex = 0;
        forwardBuffers[i].count = 0;
        forwardBuffers[i].startupFrameSent = false;
    }
}

bool fbusSensorIsForwarded(uint8_t physicalId)
{
    for (uint8_t i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
        if (forwardBuffers[i].physicalId <= FBUS_MAX_PHYS_ID && forwardBuffers[i].physicalId == physicalId) {
            return true;
        }
    }
    return false;
}

static fbusSensorForwardBuffer_t* getForwardBuffer(uint8_t physicalId)
{
    for (uint8_t i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
        if (forwardBuffers[i].physicalId <= FBUS_MAX_PHYS_ID && forwardBuffers[i].physicalId == physicalId) {
            return &forwardBuffers[i];
        }
    }
    return NULL;
}

static void addFrameToBuffer(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (!buffer) {
        return;
    }
    
    // If buffer is full, discard oldest frame
    if (buffer->count >= FBUS_FORWARDED_FRAME_BUFFER_SIZE) {
        buffer->readIndex = (buffer->readIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
        buffer->count--;
    }
    
    buffer->frames[buffer->writeIndex].physicalId = physicalId;
    buffer->frames[buffer->writeIndex].appId = appId;
    buffer->frames[buffer->writeIndex].data = data;
    buffer->frames[buffer->writeIndex].valid = true;
    
    buffer->writeIndex = (buffer->writeIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
    buffer->count++;
}

bool fbusSensorGetForwardedFrame(uint8_t physicalId, fbusSensorFrame_t *frame)
{
    if (!frame) {
        return false;
    }
    
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (!buffer) {
        return false;
    }
    
    if (buffer->count == 0) {
        frame->physicalId = physicalId;
        frame->appId = 0;
        frame->data = 0;
        frame->valid = false;
        return true;
    }
    
    memcpy(frame, &buffer->frames[buffer->readIndex], sizeof(fbusSensorFrame_t));
    
    buffer->readIndex = (buffer->readIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
    buffer->count--;
    
    return true;
}

bool fbusSensorNeedsStartupFrame(uint8_t physicalId)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (buffer) {
        return !buffer->startupFrameSent;
    }

    return false;
}

void fbusSensorMarkStartupFrameSent(uint8_t physicalId)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (buffer) {
        buffer->startupFrameSent = true;
    }
}

bool fbusSensorProcessData(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    timeUs_t currentTimeUs = micros();
    fbusObservedSensor_t *observed = trackObservedSensor(physicalId, appId, currentTimeUs);
    const fbusDetectedSensorType_e detectedType = observed ? observed->detectedType : FBUS_DETECTED_SENSOR_UNKNOWN;
    
    // Store in cache
    sensorCache[sensorCacheIndex].physicalId = physicalId;
    sensorCache[sensorCacheIndex].appId = appId;
    sensorCache[sensorCacheIndex].data = data;
    sensorCache[sensorCacheIndex].valid = true;
    sensorCache[sensorCacheIndex].lastUpdateUs = currentTimeUs;
    sensorCacheIndex = (sensorCacheIndex + 1) % FBUS_SENSOR_CACHE_SIZE;

    if (fbusSensorIsForwarded(physicalId)) {
        addFrameToBuffer(physicalId, appId, data);
    }

    if (detectedType == FBUS_DETECTED_SENSOR_GPS) {
        if (appId >= FBUS_GPS_LATITUDE_BASE && appId <= (FBUS_GPS_LATITUDE_BASE + 0x0F)) {
            if (data & FBUS_GPS_LAT_LON_BIT31) {
                fbusGps.longitude = fbusGpsConvertLatLon(data);
            } else {
                fbusGps.latitude = fbusGpsConvertLatLon(data);
            }
            fbusGps.hasPosition = true;
            fbusGps.lastUpdateUs = currentTimeUs;  

        } else if (appId >= FBUS_GPS_ALTITUDE_BASE && appId <= (FBUS_GPS_ALTITUDE_BASE + 0x0F)) {
            fbusGps.altitudeCm = fbusGpsConvertAltitude(data);
            fbusGps.hasAltitude = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_SPEED_BASE && appId <= (FBUS_GPS_SPEED_BASE + 0x0F)) {
            fbusGps.speedMilliKnots = data;
            fbusGps.hasSpeed = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_COURSE_BASE && appId <= (FBUS_GPS_COURSE_BASE + 0x0F)) {
            fbusGps.courseDeg = fbusGpsConvertCourse(data);
            fbusGps.hasCourse = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_TIME_BASE && appId <= (FBUS_GPS_TIME_BASE + 0x0F)) {
            uint8_t type = (data >> 24) & 0xFF;
            if (type == FBUS_GPS_TIME_TYPE_TIME) {
                fbusGps.hasTime = fbusGpsConvertTime(data, &fbusGps.time);
            } else if (type == FBUS_GPS_TIME_TYPE_DATE) {
                fbusGps.hasDate = fbusGpsConvertDate(data, &fbusGps.date);
            }
            fbusGps.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }
    
    if (detectedType == FBUS_DETECTED_SENSOR_XACT_SERVO) {
        if (appId >= FBUS_SERVO_DATA_BASE && appId <= (FBUS_SERVO_DATA_BASE + 0x0F)) {
            fbusServoConvertData(data, &fbusServo.currentDeciAmps,
                                &fbusServo.voltageDeciVolts,
                                &fbusServo.temperatureDegC);
            fbusServo.hasData = true;
            fbusServo.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_FAS_150S) {
        if (appId >= FBUS_CURRENT_BASE && appId <= (FBUS_CURRENT_BASE + 0x0F)) {
            fbusCurrent.currentDeciAmps = data;
            fbusCurrent.hasCurrent = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        if (appId >= FBUS_VOLTAGE_BASE && appId <= (FBUS_VOLTAGE_BASE + 0x0F)) {
            fbusCurrent.voltageCentiVolts = data;
            fbusCurrent.hasVoltage = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        if (appId >= FBUS_HIGH_PREC_CURRENT_BASE && appId <= (FBUS_HIGH_PREC_CURRENT_BASE + 0x0F)) {
            fbusCurrent.currentMilliAmps = (uint16_t)(data & 0xFFFF);
            fbusCurrent.hasHighPrecisionCurrent = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        return false;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_ESC) {
        // ESC_POWER 0x0B50~0x0B5F:
        // bits 0..15 voltage (V/100), bits 16..31 current (A/100)
        if (appId >= FBUS_ESC_POWER_BASE && appId <= (FBUS_ESC_POWER_BASE + 0x0F)) {
            fbusEsc.voltageCentiVolts = (uint16_t)(data & 0xFFFFU);
            fbusEsc.currentCentiAmps = (uint16_t)((data >> 16) & 0xFFFFU);
            fbusEsc.hasPower = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        // ESC_R&C 0x0B60~0x0B6F:
        // bits 0..15 ERPM, bits 16..31 consumption (mAh)
        if (appId >= FBUS_ESC_RPM_CONS_BASE && appId <= (FBUS_ESC_RPM_CONS_BASE + 0x0F)) {
            fbusEsc.erpm = (uint16_t)(data & 0xFFFFU);
            fbusEsc.consumptionMah = (uint16_t)((data >> 16) & 0xFFFFU);
            fbusEsc.hasRpmConsumption = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        // ESC_TEMP 0x0B70~0x0B7F:
        // bits 0..7 temperature (C)
        if (appId >= FBUS_ESC_TEMP_BASE && appId <= (FBUS_ESC_TEMP_BASE + 0x0F)) {
            fbusEsc.temperatureDegC = (uint8_t)(data & 0xFFU);
            fbusEsc.hasTemperature = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        return false;
    }
    
    return false;
}

void fbusSensorUpdate(timeUs_t currentTimeUs)
{
    if (fbusGps.lastUpdateUs > 0 && (currentTimeUs - fbusGps.lastUpdateUs) > 1000000) {
        fbusGps.hasPosition = false;
        fbusGps.hasAltitude = false;
        fbusGps.hasSpeed = false;
        fbusGps.hasCourse = false;
        fbusGps.hasTime = false;
        fbusGps.hasDate = false;
    }
    
    if (fbusServo.lastUpdateUs > 0 && (currentTimeUs - fbusServo.lastUpdateUs) > 1000000) {
        fbusServo.hasData = false;
    }

    if (fbusCurrent.lastUpdateUs > 0 && (currentTimeUs - fbusCurrent.lastUpdateUs) > 1000000) {
        fbusCurrent.hasCurrent = false;
        fbusCurrent.hasVoltage = false;
        fbusCurrent.hasHighPrecisionCurrent = false;
    }

    if (fbusEsc.lastUpdateUs > 0 && (currentTimeUs - fbusEsc.lastUpdateUs) > 1000000) {
        fbusEsc.hasPower = false;
        fbusEsc.hasRpmConsumption = false;
        fbusEsc.hasTemperature = false;
    }

#ifdef USE_GPS
    if (gpsUsesFbusTransport()) {
        if (fbusGps.hasPosition) {
            gpsSol.llh.lat = fbusGps.latitude;
            gpsSol.llh.lon = fbusGps.longitude;

            if (fbusGps.hasAltitude) {
                gpsSol.llh.altCm = fbusGps.altitudeCm;
            }

            if (fbusGps.hasSpeed) {
                gpsSol.groundSpeed = fbusGpsConvertSpeed(fbusGps.speedMilliKnots);
                gpsSol.speed3d = gpsSol.groundSpeed;
            }

            if (fbusGps.hasCourse) {
                gpsSol.groundCourse = fbusGps.courseDeg;
            }

            gpsSol.numSat = 5;
            gpsSol.hdop = 100;
            gpsData.lastMessage = millis();
            gpsSetFixState(true);
            GPS_update |= GPS_MSP_UPDATE;
        } else {
            gpsSetFixState(false);
            gpsSol.numSat = 0;
        }
    }
#endif
}

void fbusSensorGetGpsData(fbusGpsData_t *gpsData)
{
    if (gpsData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(gpsData, &fbusGps, sizeof(fbusGpsData_t));
        }
    }
}

bool fbusSensorHasGpsData(void)
{
    timeUs_t currentTimeUs = micros();
    
    if (fbusGps.hasPosition && fbusGps.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusGps.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

void fbusSensorGetServoData(fbusServoData_t *servoData)
{
    if (servoData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(servoData, &fbusServo, sizeof(fbusServoData_t));
        }
    }
}

bool fbusSensorHasServoData(void)
{
    timeUs_t currentTimeUs = micros();
    
    if (fbusServo.hasData && fbusServo.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusServo.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

void fbusSensorGetCurrentData(fbusCurrentData_t *currentData)
{
    if (currentData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(currentData, &fbusCurrent, sizeof(fbusCurrentData_t));
        }
    }
}

bool fbusSensorHasCurrentData(void)
{
    timeUs_t currentTimeUs = micros();

    if ((fbusCurrent.hasCurrent || fbusCurrent.hasVoltage || fbusCurrent.hasHighPrecisionCurrent)
        && fbusCurrent.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusCurrent.lastUpdateUs) < 1000000) {
            return true;
        }
    }

    return false;
}

void fbusSensorGetEscData(fbusEscData_t *escData)
{
    if (escData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(escData, &fbusEsc, sizeof(fbusEscData_t));
        }
    }
}

bool fbusSensorHasEscData(void)
{
    timeUs_t currentTimeUs = micros();

    if ((fbusEsc.hasPower || fbusEsc.hasRpmConsumption || fbusEsc.hasTemperature)
        && fbusEsc.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusEsc.lastUpdateUs) < 1000000) {
            return true;
        }
    }

    return false;
}

uint8_t fbusSensorGetObservedCount(void)
{
    return observedSensorCount;
}

const fbusObservedSensor_t* fbusSensorGetObserved(uint8_t index)
{
    if (index < observedSensorCount) {
        return &observedSensors[index];
    }
    return NULL;
}

void fbusSensorClearObserved(void)
{
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
}

const char* fbusSensorGetName(uint8_t physicalId)
{
    fbusObservedSensor_t *observed = getObservedSensorByPhysicalId(physicalId);
    if (observed) {
        switch (observed->detectedType) {
            case FBUS_DETECTED_SENSOR_GPS:
                return "GPS";
            case FBUS_DETECTED_SENSOR_ESC:
                return "ESC";
            case FBUS_DETECTED_SENSOR_FAS_150S:
                return "FAS_150S";
            case FBUS_DETECTED_SENSOR_XACT_SERVO:
                return "XACT_SERVO";
            case FBUS_DETECTED_SENSOR_UNKNOWN:
            default:
                break;
        }
    }

    if (physicalId < sizeof(fbusSensorNames) / sizeof(fbusSensorNames[0]) && fbusSensorNames[physicalId]) {
        return fbusSensorNames[physicalId];
    }
    return "UNKNOWN";
}

#endif // USE_FBUS_MASTER

