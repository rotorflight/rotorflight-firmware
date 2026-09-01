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

// Electrical/protocol settings for the backup RX port (drivers/rx_input_backup.c).
// Inversion/pin-swap are deliberately independent from the main RX's
// serialrx_inverted/serialrx_pinswap (pg/rx.h) - the backup port is a different
// physical UART and may need different wiring/inversion than the main receiver.
typedef struct rxInputBackupConfig_s {
    // Which protocol to decode on this port. Plain uint8_t (not the
    // rxInputBackupProvider_e enum from drivers/rx_input_backup.h) matching how
    // pg/rx.h's serialrx_provider is stored - see cli/settings.c's
    // lookupTableRxInputBackupProvider[] for the CLI-visible names, which must
    // stay in the same order as the drivers/rx_input_backup.h enum.
    uint8_t provider;

    // When OFF (0, default), the UART applies whichever electrical inversion
    // this provider's own protocol natively needs (matches primary RX's
    // serialrx_inverted default) - each provider's own Init function (e.g.
    // rx_input_backup_sbus.c) translates this shared flag into the correct
    // SERIAL_INVERTED/SERIAL_NOT_INVERTED direction for its own protocol,
    // since that native direction differs by protocol (SBUS vs FBUS/FPort).
    // When ON, the port is treated as already inverted upstream (e.g. an
    // external inverter) and the UART is left in the opposite state.
    uint8_t inverted;

    // Half-duplex (single-wire) operation, for receivers that only expose one
    // shared RX/TX pin for this protocol. Off (0, default) leaves the UART in
    // normal unidirectional RX-only mode. Like `inverted`, the exact
    // SERIAL_BIDIR variant this maps to is protocol-specific and applied by
    // each provider's own Init function.
    uint8_t halfDuplex;

    // Swaps the UART's RX/TX pins, for boards where the backup port's natural
    // RX pin isn't the one that's actually wired.
    uint8_t pinSwap;
} rxInputBackupConfig_t;

PG_DECLARE(rxInputBackupConfig_t, rxInputBackupConfig);
