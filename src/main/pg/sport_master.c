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

#include "pg/pg_ids.h"
#include "pg/sport_master.h"

#ifdef USE_SPORT_MASTER

PG_REGISTER_WITH_RESET_FN(sportMasterConfig_t, sportMasterConfig,
                          PG_DRIVER_SPORT_MASTER_CONFIG, 1);

void pgResetFn_sportMasterConfig(sportMasterConfig_t *config)
{
    config->pinSwap = 1;
    config->inverted = 1;
}

#endif
