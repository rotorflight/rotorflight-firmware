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

#ifdef USE_TELEMETRY

#include "common/time.h"
#include "common/unit.h"

#include "pg/pg_ids.h"
#include "pg/telemetry.h"

#include "telemetry/sensors.h"

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
    .hottAlarmSoundInterval = 5,
    .report_cell_voltage = false,
    .flysky_sensors = {
        IBUS_SENSOR_TYPE_TEMPERATURE,
        IBUS_SENSOR_TYPE_RPM_FLYSKY,
        IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE
    },
    .mavlink_mah_as_heading_divisor = 0,
    // Custom: the Rotorflight radio Lua suites decode the custom sensor frames.
    .crsf_telemetry_mode = CRSF_TELEMETRY_MODE_CUSTOM,
    .crsf_telemetry_link_rate = 250,
    .crsf_telemetry_link_ratio = 8,
    // Default sensor selection: what the Rotorflight radio Lua suites read. Same IDs, in the
    // same ascending order, as the Ethos suite writes for its "Default" button
    // (rotorflight-lua-ethos-suite lib/telemetry_sensor_catalog.lua DEFAULT_IDS), so a model
    // set up either way shows no telemetry_sensors diff. Keep the two in sync.
    .telemetry_sensors = {
        TELEM_BATTERY_VOLTAGE,          // 3
        TELEM_BATTERY_CURRENT,          // 4
        TELEM_BATTERY_CONSUMPTION,      // 5
        TELEM_BATTERY_CHARGE_LEVEL,     // 6
        TELEM_THROTTLE_CONTROL,         // 15
        TELEM_ESC1_TEMP1,               // 23
        TELEM_BEC_VOLTAGE,              // 43
        TELEM_HEADSPEED,                // 60
        TELEM_ARMING_FLAGS,             // 90
        TELEM_ARMING_DISABLE_FLAGS,     // 91
        TELEM_GOVERNOR_STATE,           // 93
        TELEM_PID_PROFILE,              // 95
        TELEM_RATES_PROFILE,            // 96
        TELEM_BATTERY_PROFILE,          // 97
        TELEM_ADJFUNC,                  // 99
        TELEM_SYSTEM_STATUS,            // 120
        TELEM_SYSTEM_CONFIG,            // 121
    },
    .telemetry_interval = INIT_ZERO,
);

#endif
