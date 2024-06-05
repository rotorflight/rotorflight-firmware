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

#include "drivers/io.h"

#include "pg/pg.h"

#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
# ifdef USE_SOFTSERIAL2
#  define SERIAL_PORT_MAX_INDEX (RESOURCE_SOFT_OFFSET + 2)
# else
#  define SERIAL_PORT_MAX_INDEX (RESOURCE_SOFT_OFFSET + 1)
# endif
#else
# define SERIAL_PORT_MAX_INDEX RESOURCE_SOFT_OFFSET
#endif

typedef struct serialPinConfig_s {
    ioTag_t ioTagTx[SERIAL_PORT_MAX_INDEX];
    ioTag_t ioTagRx[SERIAL_PORT_MAX_INDEX];
    ioTag_t ioTagInverter[SERIAL_PORT_MAX_INDEX];
} serialPinConfig_t;

PG_DECLARE(serialPinConfig_t, serialPinConfig);
