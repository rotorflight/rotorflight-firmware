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

#ifdef USE_FBUS_MASTER

#include "fbus_sensor.h"
#include "io/gps.h"
#include "common/maths.h"
#include "drivers/time.h"
#include <string.h>

// Internal GPS data storage
static fbusGpsData_t fbusGps;

// Internal Servo data storage
static fbusServoData_t fbusServo;

// Sensor data cache
#define FBUS_SENSOR_CACHE_SIZE 32
static fbusSensorData_t sensorCache[FBUS_SENSOR_CACHE_SIZE];
static uint8_t sensorCacheIndex = 0;

// Observed sensors tracking
static fbusObservedSensor_t observedSensors[FBUS_MAX_OBSERVED_SENSORS];
static uint8_t observedSensorCount = 0;

/**
 * Initialize FBUS sensor system
 */
void fbusSensorInit(void)
{
    memset(&fbusGps, 0, sizeof(fbusGpsData_t));
    memset(&fbusServo, 0, sizeof(fbusServoData_t));
    memset(sensorCache, 0, sizeof(sensorCache));
    sensorCacheIndex = 0;
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
}

/**
 * Convert FBUS GPS Latitude/Longitude format to Rotorflight format
 * FBUS format: Minute/10000 with bits 31:30 for direction
 * Rotorflight format: degrees * 1e7
 * 
 * @param fbusData Raw FBUS GPS lat/lon data
 * @return Latitude or Longitude in degrees * 1e7
 */
int32_t fbusGpsConvertLatLon(uint32_t fbusData)
{
    // Extract direction bits
    bool isNegative = (fbusData & FBUS_GPS_LAT_LON_BIT30) != 0;  // South or West
    
    // Extract data (minutes / 10000)
    uint32_t minutesTimes10000 = fbusData & FBUS_GPS_LAT_LON_DATA_MASK;
    
    // Convert minutes to degrees
    // minutes / 60 = degrees
    // (minutes / 10000) / 60 = degrees / 10000
    // degrees = (minutes / 10000) / 60 * 10000 = minutes / 60
    float minutes = (float)minutesTimes10000 / 10000.0f;
    float degrees = minutes / 60.0f;
    
    // Convert to Rotorflight format (degrees * 1e7)
    int32_t result = (int32_t)(degrees * 1e7f);
    
    // Apply sign based on direction
    if (isNegative) {
        result = -result;
    }
    
    return result;
}

/**
 * Convert FBUS GPS Altitude format to Rotorflight format
 * FBUS format: cm (signed 32-bit)
 * Rotorflight format: cm (signed 32-bit)
 * 
 * @param fbusData Raw FBUS GPS altitude data
 * @return Altitude in cm
 */
int32_t fbusGpsConvertAltitude(uint32_t fbusData)
{
    // Direct conversion - both use cm
    return (int32_t)fbusData;
}

/**
 * Convert FBUS GPS Speed format to Rotorflight format
 * FBUS format: Knots/1000 (unsigned 32-bit)
 * Rotorflight format: 0.1 m/s (uint16_t)
 * 
 * @param fbusData Raw FBUS GPS speed data
 * @return Speed in 0.1 m/s
 */
uint16_t fbusGpsConvertSpeed(uint32_t fbusData)
{
    // Convert knots/1000 to m/s
    // 1 knot = 0.514444 m/s
    // knots/1000 * 0.514444 = m/s
    // (m/s) * 10 = 0.1 m/s units
    float knots = (float)fbusData / 1000.0f;
    float metersPerSec = knots * 0.514444f;
    uint16_t result = (uint16_t)(metersPerSec * 10.0f);
    
    return result;
}

/**
 * Convert FBUS GPS Course format to Rotorflight format
 * FBUS format: Degree/100 (0~359.99 degrees)
 * Rotorflight format: degrees * 10
 * 
 * @param fbusData Raw FBUS GPS course data
 * @return Course in degrees * 10
 */
uint16_t fbusGpsConvertCourse(uint32_t fbusData)
{
    // FBUS: degrees/100
    // Rotorflight: degrees * 10
    // Conversion: (degrees/100) * 10 = degrees/10
    uint16_t result = (uint16_t)(fbusData / 10);
    
    // Ensure within valid range (0-3599 for degrees * 10)
    if (result >= 3600) {
        result = result % 3600;
    }
    
    return result;
}

/**
 * Convert FBUS GPS Time format
 * FBUS format: D3:0x00, D4~6:DATA (Second/Minute/Hour)
 * 
 * @param fbusData Raw FBUS GPS time data
 * @param hours Output hours
 * @param minutes Output minutes
 * @param seconds Output seconds
 */
void fbusGpsConvertTime(uint32_t fbusData, uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;
    
    if (type == FBUS_GPS_TIME_TYPE_TIME) {
        *seconds = (fbusData >> 16) & 0xFF;
        *minutes = (fbusData >> 8) & 0xFF;
        *hours = fbusData & 0xFF;
    }
}

/**
 * Convert FBUS GPS Date format
 * FBUS format: D3:0x01, D4~6:DATA (Date/Month/Year)
 *
 * @param fbusData Raw FBUS GPS date data
 * @param day Output day
 * @param month Output month
 * @param year Output year
 */
void fbusGpsConvertDate(uint32_t fbusData, uint8_t *day, uint8_t *month, uint16_t *year)
{
    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;
    
    if (type == FBUS_GPS_TIME_TYPE_DATE) {
        *day = (fbusData >> 16) & 0xFF;
        *month = (fbusData >> 8) & 0xFF;
        *year = 2000 + (fbusData & 0xFF);  // Assuming year is offset from 2000
    }
}

/**
 * Convert FBUS Servo data format
 * FBUS format: Bits 0-7: Current (0.1A), Bits 8-15: Voltage (0.1V), Bits 16-23: Temp (1°C)
 *
 * @param fbusData Raw FBUS servo data
 * @param current Output current in 0.1A
 * @param voltage Output voltage in 0.1V
 * @param temperature Output temperature in °C
 */
void fbusServoConvertData(uint32_t fbusData, uint16_t *current, uint16_t *voltage, uint16_t *temperature)
{
    // Extract current (bits 0-7): 0.1A units, range 0~25.5A
    *current = (fbusData & FBUS_SERVO_CURRENT_MASK);
    
    // Extract voltage (bits 8-15): 0.1V units, range 0~25.5V
    *voltage = ((fbusData & FBUS_SERVO_VOLTAGE_MASK) >> 8);
    
    // Extract temperature (bits 16-23): 1°C units, range 0~255°C
    *temperature = ((fbusData & FBUS_SERVO_TEMP_MASK) >> 16);
}

/**
 * Track observed sensor
 */
static void trackObservedSensor(uint8_t physicalId, uint16_t appId, timeUs_t currentTimeUs)
{
    // Find existing sensor or add new one
    fbusObservedSensor_t *sensor = NULL;
    for (uint8_t i = 0; i < observedSensorCount; i++) {
        if (observedSensors[i].physicalId == physicalId) {
            sensor = &observedSensors[i];
            break;
        }
    }
    
    // Add new sensor if not found and space available
    if (!sensor && observedSensorCount < FBUS_MAX_OBSERVED_SENSORS) {
        sensor = &observedSensors[observedSensorCount++];
        sensor->physicalId = physicalId;
        sensor->appIdCount = 0;
        sensor->packetCount = 0;
    }
    
    if (sensor) {
        // Update last seen time and packet count
        sensor->lastSeenUs = currentTimeUs;
        sensor->packetCount++;
        
        // Track app ID if not already tracked
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
    }
}

/**
 * Process incoming FBUS sensor data
 *
 * @param physicalId Physical ID of the sensor
 * @param appId Application ID (data type)
 * @param data Raw data value
 * @return true if data was processed successfully
 */
bool fbusSensorProcessData(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    timeUs_t currentTimeUs = micros();
    
    // Track this sensor observation
    trackObservedSensor(physicalId, appId, currentTimeUs);
    
    // Store in cache
    sensorCache[sensorCacheIndex].physicalId = physicalId;
    sensorCache[sensorCacheIndex].appId = appId;
    sensorCache[sensorCacheIndex].data = data;
    sensorCache[sensorCacheIndex].valid = true;
    sensorCache[sensorCacheIndex].lastUpdateUs = currentTimeUs;
    sensorCacheIndex = (sensorCacheIndex + 1) % FBUS_SENSOR_CACHE_SIZE;
    
    // Process GPS data
    if (physicalId == FBUS_SENSOR_GPS) {
        // Check which GPS data type this is
        if (appId >= FBUS_GPS_LATITUDE_BASE && appId <= (FBUS_GPS_LATITUDE_BASE + 0x0F)) {
            // Latitude or Longitude (differentiated by bit 31)
            if (data & FBUS_GPS_LAT_LON_BIT31) {
                // Longitude
                fbusGps.longitude = fbusGpsConvertLatLon(data);
            } else {
                // Latitude
                fbusGps.latitude = fbusGpsConvertLatLon(data);
            }
            fbusGps.hasPosition = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_ALTITUDE_BASE && appId <= (FBUS_GPS_ALTITUDE_BASE + 0x0F)) {
            // Altitude
            fbusGps.altitudeCm = fbusGpsConvertAltitude(data);
            fbusGps.hasAltitude = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_SPEED_BASE && appId <= (FBUS_GPS_SPEED_BASE + 0x0F)) {
            // Speed
            fbusGps.speedKnots = data;  // Store raw knots*1000 for reference
            fbusGps.hasSpeed = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_COURSE_BASE && appId <= (FBUS_GPS_COURSE_BASE + 0x0F)) {
            // Course
            fbusGps.courseDeg = fbusGpsConvertCourse(data);
            fbusGps.hasCourse = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_TIME_BASE && appId <= (FBUS_GPS_TIME_BASE + 0x0F)) {
            // Time or Date
            uint8_t type = (data >> 24) & 0xFF;
            if (type == FBUS_GPS_TIME_TYPE_TIME) {
                fbusGpsConvertTime(data, &fbusGps.hours, &fbusGps.minutes, &fbusGps.seconds);
                fbusGps.hasTime = true;
            } else if (type == FBUS_GPS_TIME_TYPE_DATE) {
                fbusGpsConvertDate(data, &fbusGps.day, &fbusGps.month, &fbusGps.year);
                fbusGps.hasDate = true;
            }
            fbusGps.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }
    
    // Process Servo data (XACT Servo sensor)
    if (physicalId == FBUS_SENSOR_XACT_SERVO) {
        // Check if this is servo data
        if (appId >= FBUS_SERVO_DATA_BASE && appId <= (FBUS_SERVO_DATA_BASE + 0x0F)) {
            // Convert servo data
            fbusServoConvertData(data, &fbusServo.currentDeciAmps,
                                &fbusServo.voltageDeciVolts,
                                &fbusServo.temperatureDegC);
            fbusServo.hasData = true;
            fbusServo.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }
    
    // Add processing for other sensor types here in the future
    
    return false;
}

/**
 * Update FBUS sensor system (called periodically)
 * 
 * @param currentTimeUs Current time in microseconds
 */
void fbusSensorUpdate(timeUs_t currentTimeUs)
{
    // Check for GPS data timeout (1 second)
    if (fbusGps.lastUpdateUs > 0 && (currentTimeUs - fbusGps.lastUpdateUs) > 1000000) {
        // Clear GPS data validity flags on timeout
        fbusGps.hasPosition = false;
        fbusGps.hasAltitude = false;
        fbusGps.hasSpeed = false;
        fbusGps.hasCourse = false;
        fbusGps.hasTime = false;
        fbusGps.hasDate = false;
    }
    
    // Check for Servo data timeout (1 second)
    if (fbusServo.lastUpdateUs > 0 && (currentTimeUs - fbusServo.lastUpdateUs) > 1000000) {
        // Clear Servo data validity flag on timeout
        fbusServo.hasData = false;
    }
    
    // Update Rotorflight GPS data if we have valid FBUS GPS data
    if (fbusGps.hasPosition) {
        gpsSol.llh.lat = fbusGps.latitude;
        gpsSol.llh.lon = fbusGps.longitude;
        
        if (fbusGps.hasAltitude) {
            gpsSol.llh.altCm = fbusGps.altitudeCm;
        }
        
        if (fbusGps.hasSpeed) {
            // Convert knots*1000 to 0.1 m/s for groundSpeed
            gpsSol.groundSpeed = fbusGpsConvertSpeed(fbusGps.speedKnots);
            gpsSol.speed3d = gpsSol.groundSpeed;  // Use same value for 3D speed
        }
        
        if (fbusGps.hasCourse) {
            gpsSol.groundCourse = fbusGps.courseDeg;
        }
        
        // Set GPS fix state
        gpsSetFixState(true);
        
        // Trigger GPS update
        GPS_update |= GPS_MSP_UPDATE;
    }
    
    // Servo data can be used for telemetry or monitoring
    // Integration with servo monitoring system can be added here in the future
}

/**
 * Get current GPS data from FBUS sensors
 * 
 * @param gpsData Output GPS data structure
 */
void fbusSensorGetGpsData(fbusGpsData_t *gpsData)
{
    if (gpsData) {
        memcpy(gpsData, &fbusGps, sizeof(fbusGpsData_t));
    }
}

/**
 * Check if GPS data is available from FBUS sensors
 * 
 * @return true if GPS data is valid and recent
 */
bool fbusSensorHasGpsData(void)
{
    timeUs_t currentTimeUs = micros();
    
    // Check if we have position data and it's recent (within 1 second)
    if (fbusGps.hasPosition && fbusGps.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusGps.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

/**
 * Get current Servo data from FBUS sensors
 *
 * @param servoData Output Servo data structure
 */
void fbusSensorGetServoData(fbusServoData_t *servoData)
{
    if (servoData) {
        memcpy(servoData, &fbusServo, sizeof(fbusServoData_t));
    }
}

/**
 * Check if Servo data is available from FBUS sensors
 *
 * @return true if Servo data is valid and recent
 */
bool fbusSensorHasServoData(void)
{
    timeUs_t currentTimeUs = micros();
    
    // Check if we have servo data and it's recent (within 1 second)
    if (fbusServo.hasData && fbusServo.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusServo.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

/**
 * Get count of observed sensors
 *
 * @return Number of unique physical IDs observed
 */
uint8_t fbusSensorGetObservedCount(void)
{
    return observedSensorCount;
}

/**
 * Get observed sensor by index
 *
 * @param index Index of sensor (0 to count-1)
 * @return Pointer to observed sensor data, or NULL if invalid index
 */
const fbusObservedSensor_t* fbusSensorGetObserved(uint8_t index)
{
    if (index < observedSensorCount) {
        return &observedSensors[index];
    }
    return NULL;
}

/**
 * Clear all observed sensor data
 */
void fbusSensorClearObserved(void)
{
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
}

#endif // USE_FBUS_MASTER