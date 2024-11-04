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

#include "pg/sbus_output.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_FN(sbusOutConfig_t, sbusOutConfig, PG_DRIVER_SBUS_OUT_CONFIG, 0);

void pgResetFn_sbusOutConfig(sbusOutConfig_t *config) {
    config->interval = 10;
    for (int i = 0; i < SBUS_OUT_CHANNELS; i++) {
        config->sourceType[i] = SBUS_OUT_SOURCE_RX;
        config->sourceIndex[i] = i;
        config->min[i] = 1000;
        config->max[i] = 2000;
    }
}