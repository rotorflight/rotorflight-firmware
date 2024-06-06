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

#ifdef USE_OSD

#include "pg/pg_ids.h"
#include "pg/osd.h"

extern void pgResetFn_osdConfig(osdConfig_t *osdConfig);

PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 9);

extern void pgResetFn_osdElementConfig(osdElementConfig_t *osdElementConfig);

PG_REGISTER_WITH_RESET_FN(osdElementConfig_t, osdElementConfig, PG_OSD_ELEMENT_CONFIG, 0);

#endif
