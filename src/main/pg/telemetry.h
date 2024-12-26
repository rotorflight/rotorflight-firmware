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

#include "common/unit.h"

#include "telemetry/ibus_shared.h"

#include "pg/pg.h"


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
    SENSOR_MODEL_ID        = BIT(23),
    SENSOR_PID_PROFILE     = BIT(24),
    SENSOR_RATES_PROFILE   = BIT(25),
    SENSOR_BEC_VOLTAGE     = BIT(26),
    SENSOR_HEADSPEED       = BIT(27),
    SENSOR_TAILSPEED       = BIT(28),
    SENSOR_THROTTLE_CONTROL= BIT(29),
    SENSOR_ARMING_FLAGS    = BIT(30),
} sensor_e;

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

enum {
    CRSF_TELEMETRY_MODE_NATIVE = 0,
    CRSF_TELEMETRY_MODE_CUSTOM,
};

#define TELEM_SENSOR_SLOT_COUNT 40

typedef struct telemetryConfig_s {
    int16_t gpsNoFixLatitude;
    int16_t gpsNoFixLongitude;
    uint8_t telemetry_inverted;
    uint8_t halfDuplex;
    uint8_t pinSwap;
    uint8_t frsky_coordinate_format;
    uint8_t frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t hottAlarmSoundInterval;
    uint8_t report_cell_voltage;
    uint8_t flysky_sensors[IBUS_SENSOR_COUNT];
    uint16_t mavlink_mah_as_heading_divisor;
    uint32_t enableSensors;
    uint8_t crsf_telemetry_mode;
    uint16_t crsf_telemetry_link_rate;
    uint16_t crsf_telemetry_link_ratio;
    uint16_t telemetry_sensors[TELEM_SENSOR_SLOT_COUNT];
    uint16_t telemetry_interval[TELEM_SENSOR_SLOT_COUNT];
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

