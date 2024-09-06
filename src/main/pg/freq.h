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

#include "drivers/io_types.h"
#include "drivers/freq.h"

#include "pg/pg.h"

enum {
    FREQ_INPUT_NOPULL,
    FREQ_INPUT_PULLUP,
    FREQ_INPUT_PULLDOWN,
};

enum {
    FREQ_INPUT_FALLING_EDGE,
    FREQ_INPUT_RISING_EDGE,
};

typedef struct freqConfig_s {
    ioTag_t ioTag[FREQ_SENSOR_PORT_COUNT];
    uint8_t pullupdn;
    uint8_t polarity;
} freqConfig_t;

PG_DECLARE(freqConfig_t, freqConfig);

