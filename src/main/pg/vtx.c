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
#include "pg/vtx.h"

#ifdef USE_VTX_COMMON

extern void pgResetFn_vtxSettingsConfig(vtxSettingsConfig_t *vtxSettingsConfig);

PG_REGISTER_WITH_RESET_FN(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 1);

PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 1);

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
    .serial_options = 0
);

#endif