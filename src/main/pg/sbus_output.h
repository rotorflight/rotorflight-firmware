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

#define SBUS_OUT_CHANNELS 18

typedef enum {
    SBUS_OUT_SOURCE_RX = 0,
    SBUS_OUT_SOURCE_MIXER = 1,
    SBUS_OUT_SOURCE_SERVO = 2
} sbusOutSourceType_e;

typedef struct sbusOutConfigChannel_s {
    sbusOutSourceType_e sourceType;
    uint8_t sourceIndex; // channel index, rule index or servo index.
    // source value maps to the min sbus value (192 for full-scale channels or 0
    // for digital channels). Typically 1000 (us) for wideband servo
    uint16_t min;
    // source value maps to the max sbus value (1792 for full-scale channels or
    // 1 for digital channels). Typically 2000 (us) for wideband servo
    uint16_t max;
} sbusOutConfigChannel_t;

PG_DECLARE_ARRAY(sbusOutConfigChannel_t, SBUS_OUT_CHANNELS, sbusOutConfig);