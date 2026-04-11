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

#include "types.h"
#include "platform.h"

#include "common/unit.h"

#include "pg/pg_ids.h"
#include "pg/telemetry.h"


PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 8);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inverted = false,
    .halfDuplex = 1,
    .pinSwap = 0,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = UNIT_METRIC,
    .frsky_vfas_precision = 0,
    .smartfuel_source = SMARTFUEL_SOURCE_CURRENT,
    .hottAlarmSoundInterval = 5,
    .report_cell_voltage = false,
    .flysky_sensors = {
        IBUS_SENSOR_TYPE_TEMPERATURE,
        IBUS_SENSOR_TYPE_RPM_FLYSKY,
        IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE
    },
    .mavlink_mah_as_heading_divisor = 0,
    .crsf_telemetry_mode = CRSF_TELEMETRY_MODE_NATIVE,
    .crsf_telemetry_link_rate = 250,
    .crsf_telemetry_link_ratio = 8,
    .smartfuel_params = {
        SMARTFUEL_STABILIZE_DELAY_DEFAULT_MS,                  // stabilize delay ms (shared by current and voltage modes)
        SMARTFUEL_STABLE_WINDOW_DEFAULT_CV,                    // stable window centivolts (shared by current and voltage modes)
        SMARTFUEL_VOLTAGE_FALL_LIMIT_DEFAULT_CVPS,             // voltage fall limit centivolts per second
        SMARTFUEL_FUEL_DROP_RATE_DEFAULT_TENTHS_PERCENT_PER_S, // fuel drop tenths percent per second
        SMARTFUEL_SAG_MULTIPLIER_DEFAULT_PERCENT,              // sag multiplier percent
    },
    .telemetry_sensors = INIT_ZERO,
    .telemetry_interval = INIT_ZERO,
);
