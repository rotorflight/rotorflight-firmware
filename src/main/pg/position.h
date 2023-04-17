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

#include <stdint.h>
#include <stdbool.h>

enum {
    ALT_SOURCE_DEFAULT = 0,
    ALT_SOURCE_BARO_ONLY,
    ALT_SOURCE_GPS_ONLY,
};

typedef struct positionConfig_s {
    uint8_t alt_source;
    uint8_t baro_alt_lpf;
    uint8_t baro_offset_lpf;
    uint8_t gps_alt_lpf;
    uint8_t gps_offset_lpf;
    uint8_t gps_min_sats;
    uint8_t vario_lpf;
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

