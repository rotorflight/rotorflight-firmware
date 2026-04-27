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
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    SMARTFUEL_SOURCE_CURRENT = 0,
    SMARTFUEL_SOURCE_VOLTAGE,
    SMARTFUEL_SOURCE_COUNT,
} smartFuelSource_e;

enum {
    CRSF_TELEMETRY_MODE_NATIVE = 0,
    CRSF_TELEMETRY_MODE_CUSTOM,
};

#define TELEM_SENSOR_SLOT_COUNT 40
#define SMARTFUEL_PARAM_COUNT 5

#define SMARTFUEL_STABILIZE_DELAY_DEFAULT_MS                 1500
#define SMARTFUEL_STABILIZE_DELAY_MAX_MS                     10000
#define SMARTFUEL_STABLE_WINDOW_DEFAULT_CV                   15
#define SMARTFUEL_STABLE_WINDOW_MAX_CV                       100
#define SMARTFUEL_VOLTAGE_FALL_LIMIT_DEFAULT_CVPS            5
#define SMARTFUEL_VOLTAGE_FALL_LIMIT_MAX_CVPS                100
#define SMARTFUEL_FUEL_DROP_RATE_DEFAULT_TENTHS_PERCENT_PER_S 10
#define SMARTFUEL_FUEL_DROP_RATE_MAX_TENTHS_PERCENT_PER_S    500
#define SMARTFUEL_SAG_MULTIPLIER_DEFAULT_PERCENT             70
#define SMARTFUEL_SAG_MULTIPLIER_MAX_PERCENT                 200

typedef enum {
    SMARTFUEL_PARAM_STABILIZE_DELAY_MS = 0,
    SMARTFUEL_PARAM_STABLE_WINDOW_CV,
    SMARTFUEL_PARAM_VOLTAGE_FALL_CVPS,
    SMARTFUEL_PARAM_FUEL_DROP_TENTHS_PERCENT_PER_S,
    SMARTFUEL_PARAM_SAG_MULTIPLIER_PERCENT,
} smartFuelParamIndex_e;

typedef struct telemetryConfig_s {
    int16_t gpsNoFixLatitude;
    int16_t gpsNoFixLongitude;
    uint8_t telemetry_inverted;
    uint8_t halfDuplex;
    uint8_t pinSwap;
    uint8_t frsky_coordinate_format;
    uint8_t frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t smartfuel;
    uint8_t smartfuel_source;
    uint8_t hottAlarmSoundInterval;
    uint8_t report_cell_voltage;
    uint8_t flysky_sensors[IBUS_SENSOR_COUNT];
    uint16_t mavlink_mah_as_heading_divisor;
    uint8_t crsf_telemetry_mode;
    uint16_t crsf_telemetry_link_rate;
    uint16_t crsf_telemetry_link_ratio;
    uint16_t smartfuel_params[SMARTFUEL_PARAM_COUNT];
    uint16_t telemetry_sensors[TELEM_SENSOR_SLOT_COUNT];
    uint16_t telemetry_interval[TELEM_SENSOR_SLOT_COUNT];
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);
