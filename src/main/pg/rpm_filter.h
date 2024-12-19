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

#define RPM_FILTER_AXIS_COUNT    3
#define RPM_FILTER_NOTCH_COUNT   16

typedef struct
{
    uint8_t notch_source[RPM_FILTER_AXIS_COUNT][RPM_FILTER_NOTCH_COUNT];     // RPM source index
    int16_t notch_center[RPM_FILTER_AXIS_COUNT][RPM_FILTER_NOTCH_COUNT];     // Center correction *10000
    uint8_t notch_q[RPM_FILTER_AXIS_COUNT][RPM_FILTER_NOTCH_COUNT];          // Notch Q *10

} rpmNotchConfig_t;

typedef struct
{
    // Filter preset: 0 = custom, 1 = low, 2 = normal, 3 = high
    uint8_t preset;

    // Hz limit
    uint8_t min_hz;

    // Custom preset
    rpmNotchConfig_t custom;

} rpmFilterConfig_t;


PG_DECLARE(rpmFilterConfig_t, rpmFilterConfig);

