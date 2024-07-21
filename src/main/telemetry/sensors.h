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
    TELEM_BATTERY_CELL_VOLTAGE          = 8,
    TELEM_BATTERY_CELL_VOLTAGES         = 9,

    TELEM_CONTROL                       = 10,
    TELEM_PITCH_CONTROL                 = 11,
    TELEM_ROLL_CONTROL                  = 12,
    TELEM_YAW_CONTROL                   = 13,
    TELEM_COLLECTIVE_CONTROL            = 14,
    TELEM_THROTTLE_CONTROL              = 15,

    TELEM_ESC1_DATA                     = 16,
    TELEM_ESC1_VOLTAGE                  = 17,
    TELEM_ESC1_CURRENT                  = 18,
    TELEM_ESC1_CAPACITY                 = 19,
    TELEM_ESC1_ERPM                     = 20,
    TELEM_ESC1_POWER                    = 21,
    TELEM_ESC1_THROTTLE                 = 22,
    TELEM_ESC1_TEMP1                    = 23,
    TELEM_ESC1_TEMP2                    = 24,
    TELEM_ESC1_BEC_VOLTAGE              = 25,
    TELEM_ESC1_BEC_CURRENT              = 26,
    TELEM_ESC1_STATUS                   = 27,
    TELEM_ESC1_MODEL                    = 28,

    TELEM_ESC2_DATA                     = 29,
    TELEM_ESC2_VOLTAGE                  = 30,
    TELEM_ESC2_CURRENT                  = 31,
    TELEM_ESC2_CAPACITY                 = 32,
    TELEM_ESC2_ERPM                     = 33,
    TELEM_ESC2_POWER                    = 34,
    TELEM_ESC2_THROTTLE                 = 35,
    TELEM_ESC2_TEMP1                    = 36,
    TELEM_ESC2_TEMP2                    = 37,
    TELEM_ESC2_BEC_VOLTAGE              = 38,
    TELEM_ESC2_BEC_CURRENT              = 39,
    TELEM_ESC2_STATUS                   = 40,
    TELEM_ESC2_MODEL                    = 41,

    TELEM_ESC_VOLTAGE                   = 42,
    TELEM_BEC_VOLTAGE                   = 43,
    TELEM_BUS_VOLTAGE                   = 44,
    TELEM_MCU_VOLTAGE                   = 45,

    TELEM_ESC_CURRENT                   = 46,
    TELEM_BEC_CURRENT                   = 47,
    TELEM_BUS_CURRENT                   = 48,
    TELEM_MCU_CURRENT                   = 49,

    TELEM_ESC_TEMP                      = 50,
    TELEM_BEC_TEMP                      = 51,
    TELEM_MCU_TEMP                      = 52,
    TELEM_AIR_TEMP                      = 53,
    TELEM_MOTOR_TEMP                    = 54,
    TELEM_BATTERY_TEMP                  = 55,
    TELEM_EXHAUST_TEMP                  = 56,

    TELEM_HEADING                       = 57,
    TELEM_ALTITUDE                      = 58,
    TELEM_VARIOMETER                    = 59,

    TELEM_HEADSPEED                     = 60,
    TELEM_TAILSPEED                     = 61,
    TELEM_MOTOR_RPM                     = 62,
    TELEM_TRANS_RPM                     = 63,

    TELEM_ATTITUDE                      = 64,
    TELEM_ATTITUDE_PITCH                = 65,
    TELEM_ATTITUDE_ROLL                 = 66,
    TELEM_ATTITUDE_YAW                  = 67,

    TELEM_ACCEL                         = 68,
    TELEM_ACCEL_X                       = 69,
    TELEM_ACCEL_Y                       = 70,
    TELEM_ACCEL_Z                       = 71,

    TELEM_GPS                           = 72,
    TELEM_GPS_SATS                      = 73,
    TELEM_GPS_PDOP                      = 74,
    TELEM_GPS_HDOP                      = 75,
    TELEM_GPS_VDOP                      = 76,
    TELEM_GPS_COORD                     = 77,
    TELEM_GPS_ALTITUDE                  = 78,
    TELEM_GPS_HEADING                   = 79,
    TELEM_GPS_GROUNDSPEED               = 80,
    TELEM_GPS_HOME_DISTANCE             = 81,
    TELEM_GPS_HOME_DIRECTION            = 82,
    TELEM_GPS_DATE_TIME                 = 83,

    TELEM_LOAD                          = 84,
    TELEM_CPU_LOAD                      = 85,
    TELEM_SYS_LOAD                      = 86,
    TELEM_RT_LOAD                       = 87,

    TELEM_MODEL_ID                      = 88,
    TELEM_FLIGHT_MODE                   = 89,
    TELEM_ARMING_FLAGS                  = 90,
    TELEM_ARMING_DISABLE_FLAGS          = 91,
    TELEM_RESCUE_STATE                  = 92,
    TELEM_GOVERNOR_STATE                = 93,
    TELEM_GOVERNOR_FLAGS                = 94,

    TELEM_PID_PROFILE                   = 95,
    TELEM_RATES_PROFILE                 = 96,
    TELEM_BATTERY_PROFILE               = 97,
    TELEM_LED_PROFILE                   = 98,

    TELEM_ADJFUNC                       = 99,

    TELEM_DEBUG_0                       = 100,
    TELEM_DEBUG_1                       = 101,
    TELEM_DEBUG_2                       = 102,
    TELEM_DEBUG_3                       = 103,
    TELEM_DEBUG_4                       = 104,
    TELEM_DEBUG_5                       = 105,
    TELEM_DEBUG_6                       = 106,
    TELEM_DEBUG_7                       = 107,

    TELEM_SENSOR_COUNT

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
