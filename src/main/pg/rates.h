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

#include "types.h"
#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


#define MAX_RATE_PROFILE_NAME_LENGTH 8

typedef struct controlRateConfig_s {
    char profileName[MAX_RATE_PROFILE_NAME_LENGTH + 1];

    uint8_t rates_type;

    uint8_t rcRates[4];
    uint8_t rcExpo[4];
    uint8_t rates[4];

    uint8_t levelExpo[2];                   // roll/pitch level mode expo
    uint8_t quickRatesRcExpo;               // Sets expo on rc command for quick rates

    uint8_t response_time[4];
    uint16_t accel_limit[4];

    uint8_t cyclic_ring;

    uint8_t setpoint_boost_gain[4];
    uint8_t setpoint_boost_cutoff[4];

    uint8_t yaw_dynamic_ceiling_gain;
    uint8_t yaw_dynamic_deadband_gain;
    uint8_t yaw_dynamic_deadband_cutoff;
    uint8_t yaw_dynamic_deadband_filter;

} controlRateConfig_t;

PG_DECLARE_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);
