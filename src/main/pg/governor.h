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

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint16_t gov_startup_time;
    uint16_t gov_spoolup_time;
    uint16_t gov_tracking_time;
    uint16_t gov_recovery_time;
    uint16_t gov_zero_throttle_timeout;
    uint16_t gov_lost_headspeed_timeout;
    uint16_t gov_autorotation_timeout;
    uint16_t gov_autorotation_bailout_time;
    uint16_t gov_autorotation_min_entry_time;
    uint8_t  gov_handover_throttle;
    uint8_t  gov_pwr_filter;
    uint8_t  gov_rpm_filter;
    uint8_t  gov_tta_filter;
    uint8_t  gov_ff_filter;
    uint8_t  gov_ff_filter_type;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


