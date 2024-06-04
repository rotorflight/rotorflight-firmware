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

#include "pg/pg_ids.h"
#include "pg/failsafe.h"

#include "flight/failsafe.h"

PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 2);

PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_throttle = 1000,                           // default throttle off.
    .failsafe_throttle_low_delay = 100,                  // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_delay = 15,                                // 1.5 sec stage 1 period, can regain control on signal recovery, at idle in drop mode
    .failsafe_off_delay = 10,                            // 1 sec in landing phase, if enabled
    .failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1, // default failsafe switch action is identical to rc link loss
    .failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT,    // default full failsafe procedure is 0: auto-landing
    .failsafe_recovery_delay = 10,                       // 1 sec of valid rx data needed to allow recovering from failsafe procedure
    .failsafe_stick_threshold = 30                       // 30 percent of stick deflection to exit GPS Rescue procedure
);
