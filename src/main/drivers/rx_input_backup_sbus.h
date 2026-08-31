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

#include <stdbool.h>

#include "drivers/rx_input_backup.h"

// SBUS provider for the generic backup-RX framework (rx_input_backup.c). Only
// included/called by that file, guarded by USE_RX_INPUT_BACKUP_SBUS - this is
// the template a future FBUS/FPort provider (rx_input_backup_fbus.c, etc.)
// would follow: same *Init(rxInputBackupOps_t *ops) shape, own private frame
// struct/ISR/decode, no telemetry.

// Fills *ops and returns true. (Always succeeds today - only fails to compile
// in when USE_RX_INPUT_BACKUP_SBUS isn't built, same as the dispatch switch in
// rx_input_backup.c guards each case.)
bool rxInputBackupSbusInit(rxInputBackupOps_t *ops);
