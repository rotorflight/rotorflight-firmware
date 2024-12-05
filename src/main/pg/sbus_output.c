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

#include "pg/sbus_output.h"

#ifdef USE_SBUS_OUTPUT

// The config struct is quite large. A ResetFn is smaller than a ResetTemplate.
PG_REGISTER_WITH_RESET_FN(sbusOutConfig_t, sbusOutConfig,
                          PG_DRIVER_SBUS_OUT_CONFIG, 0);

void pgResetFn_sbusOutConfig(sbusOutConfig_t *config) {
    for (int i = 0; i < SBUS_OUT_CHANNELS; i++) {
        config->sourceType[i] = SBUS_OUT_SOURCE_RX;
        config->sourceIndex[i] = i;
        config->sourceRangeLow[i] = 1000;
        config->sourceRangeHigh[i] = 2000;
    }
    config->frameRate = 50;

    config->pinSwap = 0;
}

#endif
