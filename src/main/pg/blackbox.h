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

#include "pg/pg.h"


typedef enum {
    BLACKBOX_DEVICE_NONE = 0,
    BLACKBOX_DEVICE_FLASH = 1,
    BLACKBOX_DEVICE_SDCARD = 2,
    BLACKBOX_DEVICE_SERIAL = 3
} BlackboxDevice_e;

typedef enum {
    BLACKBOX_MODE_OFF = 0,
    BLACKBOX_MODE_NORMAL,
    BLACKBOX_MODE_ARMED,
    BLACKBOX_MODE_SWITCH,
} BlackboxMode_e;


typedef struct blackboxConfig_s {
    uint8_t     device;
    uint8_t     mode;
    uint16_t    denom;
    uint32_t    fields;
    uint16_t    initialEraseFreeSpaceKiB;
    uint8_t     rollingErase;
} blackboxConfig_t;

PG_DECLARE(blackboxConfig_t, blackboxConfig);
