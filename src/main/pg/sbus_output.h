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

#pragma once

#include "pg/pg.h"
#include "common/utils.h"

#define SBUS_OUT_CHANNELS 18

typedef enum {
    SBUS_OUT_SOURCE_RX = 0,
    SBUS_OUT_SOURCE_MIXER = 1,
    SBUS_OUT_SOURCE_SERVO = 2
} sbusOutSourceType_e;

typedef struct sbusOutConfig_s {
    uint8_t sourceType[SBUS_OUT_CHANNELS];

    // channel index, rule index or servo index.
    uint8_t sourceIndex[SBUS_OUT_CHANNELS]; 

    // source value maps to the min sbus value (192 for full-scale channels or 0
    // for digital channels). Typically:
    //   * 1000 or 988 for receiver channels
    //   * (need verify) -1000 for mixer rule (treat as -1.0f)
    //   * 1000 (us) for wideband servo, 500 (us) for narrowband servo
    int16_t min[SBUS_OUT_CHANNELS];

    // source value maps to the max sbus value (1792 for full-scale channels or
    // 1 for digital channels). Typically:
    //   * 2000 or 2012 for receiver channels
    //   * (need verify) +1000 for mixer rule (treat as +1.0f)
    //   * 2000 (us) for wideband servo, 1000 (us) for narrowband servo
    int16_t max[SBUS_OUT_CHANNELS];

} sbusOutConfig_t;

PG_DECLARE(sbusOutConfig_t, sbusOutConfig);
