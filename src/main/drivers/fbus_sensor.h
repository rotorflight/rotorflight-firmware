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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/time.h"

// Frame forwarding configuration
#define FBUS_FORWARDED_FRAME_BUFFER_SIZE 10

// FBUS Sensor Physical IDs
typedef enum {
    FBUS_SENSOR_VARIO2      = 0x00,
    FBUS_SENSOR_FLVSS       = 0x01,
    FBUS_SENSOR_CURRENT     = 0x02,
    FBUS_SENSOR_GPS         = 0x03,
    FBUS_SENSOR_RPM         = 0x04,
    FBUS_SENSOR_SP2UART_A   = 0x05,  // Host
    FBUS_SENSOR_SP2UART_B   = 0x06,  // Remote
    FBUS_SENSOR_FAS_150S    = 0x07,
    FBUS_SENSOR_SBEC        = 0x08,
    FBUS_SENSOR_AIR_SPEED   = 0x09,
    FBUS_SENSOR_ESC         = 0x0A,
    FBUS_SENSOR_PSD_30      = 0x0B,
    FBUS_SENSOR_XACT_SERVO  = 0x0C,
    FBUS_SENSOR_SD1         = 0x10,
    FBUS_SENSOR_VS600       = 0x12,
    FBUS_SENSOR_GASSUIT     = 0x16,
    FBUS_SENSOR_FSD         = 0x17,
    FBUS_SENSOR_GATEWAY     = 0x18,
    FBUS_SENSOR_REDUNDANCY_BUS = 0x19,
    FBUS_SENSOR_S6R         = 0x1A,
} fbusSensorPhysicalId_e;

// FBUS GPS Data IDs
typedef enum {
    FBUS_GPS_LATITUDE_BASE  = 0x0800,  // 0x0800~0x080f
    FBUS_GPS_LONGITUDE_BASE = 0x0800,  // Same base, differentiated by bit31
    FBUS_GPS_ALTITUDE_BASE  = 0x0820,  // 0x0820~0x082f
    FBUS_GPS_SPEED_BASE     = 0x0830,  // 0x0830~0x083f
    FBUS_GPS_COURSE_BASE    = 0x0840,  // 0x0840~0x084f
    FBUS_GPS_TIME_BASE      = 0x0850,  // 0x0850~0x085f
} fbusGpsDataId_e;

// GPS Latitude/Longitude bit definitions
#define FBUS_GPS_LAT_LON_BIT31      (1U << 31)  // 0=Latitude, 1=Longitude
#define FBUS_GPS_LAT_LON_BIT30      (1U << 30)  // 0=North/East, 1=South/West
#define FBUS_GPS_LAT_LON_DATA_MASK  0x3FFFFFFF  // Bits 0-29: Data

// GPS Time data byte definitions
#define FBUS_GPS_TIME_TYPE_TIME     0x00  // D3: UTC Time (Second/Minute/Hour)
#define FBUS_GPS_TIME_TYPE_DATE     0x01  // D3: Date (Date/Month/Year)

// FBUS Servo Data IDs
typedef enum {
    FBUS_SERVO_DATA_BASE        = 0x6800,  // 0x6800~0x680F
} fbusServoDataId_e;

// Servo data byte definitions
#define FBUS_SERVO_CURRENT_MASK     0x000000FF  // Bits 0-7: Current (0.1A, 0~25.5A)
#define FBUS_SERVO_VOLTAGE_MASK     0x0000FF00  // Bits 8-15: Voltage (0.1V, 0~25.5V)
#define FBUS_SERVO_TEMP_MASK        0x00FF0000  // Bits 16-23: Temperature (1°C, 0~255°C)

// FBUS Sensor data structure
typedef struct {
    uint8_t physicalId;
    uint16_t appId;
    uint32_t data;
    bool valid;
    timeUs_t lastUpdateUs;
} fbusSensorData_t;

// Frame buffer for forwarding to receiver
typedef struct {
    uint8_t physicalId;
    uint16_t appId;
    uint32_t data;
    bool valid;
} fbusSensorFrame_t;

// Forwarded sensor buffer structure
typedef struct {
    uint8_t physicalId;  // Physical ID this buffer is for
    fbusSensorFrame_t frames[FBUS_FORWARDED_FRAME_BUFFER_SIZE];
    uint8_t writeIndex;  // Next position to write
    uint8_t readIndex;   // Next position to read
    uint8_t count;       // Number of frames in buffer
    bool startupFrameSent;  // Track if empty startup frame was sent
} fbusSensorForwardBuffer_t;

// GPS specific data structure
typedef struct {
    int32_t latitude;       // Latitude in degrees * 1e7 (Rotorflight format)
    int32_t longitude;      // Longitude in degrees * 1e7 (Rotorflight format)
    int32_t altitudeCm;     // Altitude in cm
    uint16_t speedKnots;    // Speed in knots * 1000
    uint16_t courseDeg;     // Course in degrees * 100
    uint8_t hours;          // UTC hours
    uint8_t minutes;        // UTC minutes
    uint8_t seconds;        // UTC seconds
    uint8_t day;            // Day
    uint8_t month;          // Month
    uint16_t year;          // Year
    bool hasPosition;       // Position data valid
    bool hasAltitude;       // Altitude data valid
    bool hasSpeed;          // Speed data valid
    bool hasCourse;         // Course data valid
    bool hasTime;           // Time data valid
    bool hasDate;           // Date data valid
    timeUs_t lastUpdateUs;
} fbusGpsData_t;

// Servo specific data structure
typedef struct {
    uint16_t currentDeciAmps;   // Current in 0.1A (0~255 = 0~25.5A)
    uint16_t voltageDeciVolts;  // Voltage in 0.1V (0~255 = 0~25.5V)
    uint16_t temperatureDegC;   // Temperature in °C (0~255°C)
    bool hasData;               // Data valid
    timeUs_t lastUpdateUs;
} fbusServoData_t;

// Function prototypes
void fbusSensorInit(void);
void fbusSensorUpdate(timeUs_t currentTimeUs);
bool fbusSensorProcessData(uint8_t physicalId, uint16_t appId, uint32_t data);
void fbusSensorGetGpsData(fbusGpsData_t *gpsData);
bool fbusSensorHasGpsData(void);
void fbusSensorGetServoData(fbusServoData_t *servoData);
bool fbusSensorHasServoData(void);

// GPS data conversion functions
int32_t fbusGpsConvertLatLon(uint32_t fbusData);
int32_t fbusGpsConvertAltitude(uint32_t fbusData);
uint16_t fbusGpsConvertSpeed(uint32_t fbusData);
uint16_t fbusGpsConvertCourse(uint32_t fbusData);
void fbusGpsConvertTime(uint32_t fbusData, uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
void fbusGpsConvertDate(uint32_t fbusData, uint8_t *day, uint8_t *month, uint16_t *year);

// Servo data conversion functions
void fbusServoConvertData(uint32_t fbusData, uint16_t *current, uint16_t *voltage, uint16_t *temperature);

// Observed sensor tracking
#define FBUS_MAX_OBSERVED_SENSORS 32
typedef struct {
    uint8_t physicalId;
    uint16_t appIds[16];  // Track up to 16 different app IDs per physical ID
    uint8_t appIdCount;
    uint32_t lastSeenUs;
    uint32_t packetCount;
} fbusObservedSensor_t;

uint8_t fbusSensorGetObservedCount(void);
const fbusObservedSensor_t* fbusSensorGetObserved(uint8_t index);
void fbusSensorClearObserved(void);
const char* fbusSensorGetName(uint8_t physicalId);

// Frame forwarding functions
void fbusSensorInitForwarding(void);
bool fbusSensorIsForwarded(uint8_t physicalId);
bool fbusSensorGetForwardedFrame(uint8_t physicalId, fbusSensorFrame_t *frame);
bool fbusSensorNeedsStartupFrame(uint8_t physicalId);
void fbusSensorMarkStartupFrameSent(uint8_t physicalId);