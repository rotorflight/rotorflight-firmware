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

#include "pg/pg_ids.h"
#include "platform.h"

#include "pg/fbus_master.h"
#include "drivers/serial.h"

#ifdef USE_FBUS_MASTER

PG_REGISTER_WITH_RESET_FN(fbusMasterConfig_t, fbusMasterConfig,
                          PG_DRIVER_FBUS_MASTER_CONFIG, 4);

void pgResetFn_fbusMasterConfig(fbusMasterConfig_t *config) {
    config->frameRate = 500;
    config->pinSwap = 0;
    // Default to inverted F.Bus (normal for F.Bus receivers).
    config->inverted = 1;
}

#endif
