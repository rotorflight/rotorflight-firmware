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
#include "platform.h"

#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct {
    uint16_t    mid;     // center (mid) point
    int16_t     min;     // lower limit in us from the midpoint
    int16_t     max;     // upper limit in us from the midpoint
    uint16_t    rneg;    // negative scale (slope) in us
    uint16_t    rpos;    // positive scale (slope) in us
    uint16_t    rate;    // servo update rate Hz
    uint16_t    speed;   // speed limit
    uint16_t    flags;   // feature flags
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct {
    ioTag_t  ioTags[MAX_SUPPORTED_SERVOS];
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);
