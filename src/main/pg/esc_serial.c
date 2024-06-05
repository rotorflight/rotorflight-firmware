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

#include "types.h"
#include "platform.h"

#include "config/config_reset.h"

#include "pg/pg_ids.h"
#include "pg/esc_serial.h"

#ifndef ESCSERIAL_TIMER_TX_PIN
#define ESCSERIAL_TIMER_TX_PIN NONE
#endif

PG_RESET_TEMPLATE(escSerialConfig_t, escSerialConfig,
    .ioTag = IO_TAG(ESCSERIAL_TIMER_TX_PIN),
);

PG_REGISTER_WITH_RESET_TEMPLATE(escSerialConfig_t, escSerialConfig, PG_ESCSERIAL_CONFIG, 0);

