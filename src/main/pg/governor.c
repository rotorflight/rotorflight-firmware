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

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/governor.h"


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .gov_mode = 0,
    .gov_startup_time = 200,
    .gov_spoolup_time = 100,
    .gov_tracking_time = 20,
    .gov_recovery_time = 20,
    .gov_spooldown_time = 30,
    .gov_throttle_hold_timeout = 50,
    .gov_handover_throttle = 20,
    .gov_pwr_filter = 5,
    .gov_rpm_filter = 10,
    .gov_tta_filter = 0,
    .gov_ff_filter = 5,
    .gov_d_cutoff = 50,
);

