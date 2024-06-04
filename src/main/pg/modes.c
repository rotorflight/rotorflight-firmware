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

#include "pg/modes.h"

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 2);


#if defined(USE_CUSTOM_BOX_NAMES)

PG_REGISTER_WITH_RESET_TEMPLATE(modeActivationConfig_t, modeActivationConfig, PG_MODE_ACTIVATION_CONFIG, 0);

PG_RESET_TEMPLATE(modeActivationConfig_t, modeActivationConfig,
    .box_user_1_name = INIT_ZERO,
    .box_user_2_name = INIT_ZERO,
    .box_user_3_name = INIT_ZERO,
    .box_user_4_name = INIT_ZERO,
);

#endif

