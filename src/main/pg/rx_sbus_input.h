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

#include "common/utils.h"
#include "pg/pg.h"

// Electrical settings for the SBUS-In Fallback port (drivers/rx_sbus_input.c).
// Deliberately independent from the main RX's serialrx_inverted/serialrx_pinswap
// (pg/rx.h) - the fallback port is a different physical UART and may need
// different wiring/inversion than the main receiver.
typedef struct sbusInputConfig_s {
    // When OFF (0, default), the UART applies the electrical inversion normal
    // SBUS wiring needs (matches primary SBUS RX's serialrx_inverted default).
    // When ON, the port is treated as already inverted upstream (e.g. an
    // external inverter) and the UART is left non-inverted.
    uint8_t inverted;

    // Swaps the UART's RX/TX pins, for boards where the fallback port's
    // natural RX pin isn't the one that's actually wired.
    uint8_t pinSwap;
} sbusInputConfig_t;

PG_DECLARE(sbusInputConfig_t, sbusInputConfig);
