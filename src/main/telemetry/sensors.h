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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pg/pg.h"
#include "pg/telemetry.h"

#include "common/streambuf.h"

#include "flight/motors.h"
#include "flight/servos.h"


/** Custom telemetry sensor types **/

typedef enum
{
    TELEM_NONE                          = 0,

    TELEM_HEARTBEAT                     = 1,

    TELEM_BATTERY                       = 2,
    TELEM_BATTERY_VOLTAGE               = 3,
    TELEM_BATTERY_CURRENT               = 4,
    TELEM_BATTERY_CONSUMPTION           = 5,
    TELEM_BATTERY_CHARGE_LEVEL          = 6,
    TELEM_BATTERY_CELL_COUNT            = 7,
    TELEM_BATTERY_CELL_VOLTAGES         = 8,

    TELEM_CONTROL                       = 9,
    TELEM_PITCH_CONTROL                 = 10,
    TELEM_ROLL_CONTROL                  = 11,
    TELEM_YAW_CONTROL                   = 12,
    TELEM_COLLECTIVE_CONTROL            = 13,
    TELEM_THROTTLE_CONTROL              = 14,

    TELEM_ESC1_DATA                     = 15,
    TELEM_ESC1_VOLTAGE                  = 16,
    TELEM_ESC1_CURRENT                  = 17,
    TELEM_ESC1_CAPACITY                 = 18,
    TELEM_ESC1_ERPM                     = 19,
    TELEM_ESC1_POWER                    = 20,
    TELEM_ESC1_THROTTLE                 = 21,
    TELEM_ESC1_TEMP1                    = 22,
    TELEM_ESC1_TEMP2                    = 23,
    TELEM_ESC1_BEC_VOLTAGE              = 24,
    TELEM_ESC1_BEC_CURRENT              = 25,
    TELEM_ESC1_STATUS                   = 26,

    TELEM_ESC2_DATA                     = 27,
    TELEM_ESC2_VOLTAGE                  = 28,
    TELEM_ESC2_CURRENT                  = 29,
    TELEM_ESC2_CAPACITY                 = 30,
    TELEM_ESC2_ERPM                     = 31,
    TELEM_ESC2_POWER                    = 32,
    TELEM_ESC2_THROTTLE                 = 33,
    TELEM_ESC2_TEMP1                    = 34,
    TELEM_ESC2_TEMP2                    = 35,
    TELEM_ESC2_BEC_VOLTAGE              = 36,
    TELEM_ESC2_BEC_CURRENT              = 37,
    TELEM_ESC2_STATUS                   = 38,

    TELEM_ESC_VOLTAGE                   = 39,
    TELEM_BEC_VOLTAGE                   = 40,
    TELEM_BUS_VOLTAGE                   = 41,
    TELEM_MCU_VOLTAGE                   = 42,

    TELEM_ESC_CURRENT                   = 43,
    TELEM_BEC_CURRENT                   = 44,
    TELEM_BUS_CURRENT                   = 45,
    TELEM_MCU_CURRENT                   = 46,

    TELEM_ESC_TEMP                      = 47,
    TELEM_BEC_TEMP                      = 48,
    TELEM_MCU_TEMP                      = 49,
    TELEM_AIR_TEMP                      = 50,
    TELEM_MOTOR_TEMP                    = 51,
    TELEM_BATTERY_TEMP                  = 52,
    TELEM_EXHAUST_TEMP                  = 53,

    TELEM_HEADING                       = 54,
    TELEM_ALTITUDE                      = 55,
    TELEM_VARIOMETER                    = 56,

    TELEM_HEADSPEED                     = 57,
    TELEM_TAILSPEED                     = 58,
    TELEM_MOTOR_RPM                     = 59,
    TELEM_TRANS_RPM                     = 60,

    TELEM_ATTITUDE                      = 61,
    TELEM_ATTITUDE_PITCH                = 62,
    TELEM_ATTITUDE_ROLL                 = 63,
    TELEM_ATTITUDE_YAW                  = 64,

    TELEM_ACCEL                         = 65,
    TELEM_ACCEL_X                       = 66,
    TELEM_ACCEL_Y                       = 67,
    TELEM_ACCEL_Z                       = 68,

    TELEM_GPS                           = 69,
    TELEM_GPS_SATS                      = 70,
    TELEM_GPS_PDOP                      = 71,
    TELEM_GPS_HDOP                      = 72,
    TELEM_GPS_VDOP                      = 73,
    TELEM_GPS_COORD                     = 74,
    TELEM_GPS_ALTITUDE                  = 75,
    TELEM_GPS_HEADING                   = 76,
    TELEM_GPS_GROUNDSPEED               = 77,
    TELEM_GPS_HOME_DISTANCE             = 78,
    TELEM_GPS_HOME_DIRECTION            = 79,
    TELEM_GPS_DATE_TIME                 = 80,

    TELEM_LOAD                          = 81,
    TELEM_CPU_LOAD                      = 82,
    TELEM_SYS_LOAD                      = 83,
    TELEM_RT_LOAD                       = 84,

    TELEM_MODEL_ID                      = 85,
    TELEM_FLIGHT_MODE                   = 86,
    TELEM_ARMING_FLAGS                  = 87,
    TELEM_RESCUE_STATE                  = 88,
    TELEM_GOVERNOR_STATE                = 89,

    TELEM_PROFILES                      = 90,
    TELEM_PID_PROFILE                   = 91,
    TELEM_RATES_PROFILE                 = 92,
    TELEM_BATTERY_PROFILE               = 93,
    TELEM_LED_PROFILE                   = 94,
    TELEM_OSD_PROFILE                   = 95,

    TELEM_ADJFUNC                       = 96,

    TELEM_DEBUG_0                       = 97,
    TELEM_DEBUG_1                       = 98,
    TELEM_DEBUG_2                       = 99,
    TELEM_DEBUG_3                       = 100,
    TELEM_DEBUG_4                       = 101,
    TELEM_DEBUG_5                       = 102,
    TELEM_DEBUG_6                       = 103,
    TELEM_DEBUG_7                       = 104,

    TELEM_SENSOR_COUNT,

} sensor_id_e;


typedef struct telemetrySensor_s telemetrySensor_t;

typedef void (*telemetryEncode_f)(sbuf_t *buf, telemetrySensor_t *sensor);

typedef uint16_t sensor_code_t;

struct telemetrySensor_s {

    sensor_id_e             telid;
    sensor_code_t           tcode;

    uint16_t                min_interval;
    uint16_t                max_interval;

    bool                    active;
    bool                    update;
    int                     bucket;
    int                     value;

    telemetryEncode_f       encode;
};


int telemetrySensorValue(sensor_id_e id);
bool telemetrySensorActive(sensor_id_e id);
bool telemetryIsSensorIdEnabled(sensor_id_e sensor_id);


/** Legacy sensors **/

typedef enum {
    SENSOR_VOLTAGE         = BIT(0),
    SENSOR_CURRENT         = BIT(1),
    SENSOR_FUEL            = BIT(2),
    SENSOR_MODE            = BIT(3),
    SENSOR_ACC_X           = BIT(4),
    SENSOR_ACC_Y           = BIT(5),
    SENSOR_ACC_Z           = BIT(6),
    SENSOR_PITCH           = BIT(7),
    SENSOR_ROLL            = BIT(8),
    SENSOR_HEADING         = BIT(9),
    SENSOR_ALTITUDE        = BIT(10),
    SENSOR_VARIO           = BIT(11),
    SENSOR_LAT_LONG        = BIT(12),
    SENSOR_GROUND_SPEED    = BIT(13),
    SENSOR_DISTANCE        = BIT(14),
    ESC_SENSOR_CURRENT     = BIT(15),
    ESC_SENSOR_VOLTAGE     = BIT(16),
    ESC_SENSOR_RPM         = BIT(17),
    ESC_SENSOR_TEMPERATURE = BIT(18),
    SENSOR_TEMPERATURE     = BIT(19),
    SENSOR_CAP_USED        = BIT(20),
    SENSOR_ADJUSTMENT      = BIT(21),
    SENSOR_GOV_MODE        = BIT(22),
} sensor_e;

sensor_e telemetryGetLegacySensor(sensor_id_e sensor_id);

void legacySensorInit(void);

bool telemetryIsSensorEnabled(uint32_t sensor_bits);
