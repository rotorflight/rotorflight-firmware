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

#include "config/config.h"

#include "pg/pg.h"

typedef struct {
    uint32_t    functionMask;
    int8_t      identifier;
    uint8_t     msp_baudrateIndex;
    uint8_t     gps_baudrateIndex;
    uint8_t     blackbox_baudrateIndex;
    uint8_t     telemetry_baudrateIndex;
} serialPortConfig_t;

typedef struct {
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
    uint16_t    serial_update_rate_hz;
    uint8_t     reboot_character;
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);
