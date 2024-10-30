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
#include "pg/sbus_output.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(sbusOutConfigChannel_t, SBUS_OUT_CHANNELS, sbusOutConfig, PG_DRIVER_SBUS_OUT_CONFIG, 0);

void pgResetFn_sbusOutConfig(sbusOutConfigChannel_t *config) {
    // Set default min max
    for (int i = 0; i < 16; ++i) {
        // full range channels
        config[i].min = 192;
        config[i].max = 1792;
    }
    for (int i = 16; i < SBUS_OUT_CHANNELS; ++i) {
        config[i].min = 0;
        config[i].max = 1;
    }

    // Set default source
    for (int i = 0; i < SBUS_OUT_CHANNELS; ++i) {
        config[i].sourceType = SBUS_OUT_SOURCE_RX; 
        config[i].sourceChannel = i;
    }
}
