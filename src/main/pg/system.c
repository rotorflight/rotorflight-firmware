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

#include "pg/system.h"

#include "target/common_pre.h"


PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 3);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = 0,
    .debug_axis = 0,
    .task_statistics = true,
    .cpu_overclock = DEFAULT_CPU_OVERCLOCK,
    .hseMhz = SYSTEM_HSE_VALUE,
    .configurationState = CONFIGURATION_STATE_DEFAULTS_BARE,
);

