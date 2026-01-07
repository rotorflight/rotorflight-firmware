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

#include "types.h"

#include "pg/pg.h"
#include "pg/modes.h"

#define ADJFUN_DECLARE(id) \
    int get_ADJUSTMENT_##id(void); \
    void set_ADJUSTMENT_##id(int);

typedef struct {
    uint8_t function;
    uint8_t enaChannel;
    channelRange_t enaRange;
    uint8_t adjChannel;
    channelRange_t adjRange1;
    channelRange_t adjRange2;
    int16_t adjMin;
    int16_t adjMax;
    uint8_t adjStep;
} adjustmentRange_t;

#define MAX_ADJUSTMENT_RANGE_COUNT 42

PG_DECLARE_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges);

