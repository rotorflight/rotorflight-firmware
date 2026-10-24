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

// FBUS and FPort2 providers for the generic backup-RX framework
// (rx_input_backup.c). Both use the exact same length-prefixed frame format -
// rx/fbus.c's own fbusRxInit(rxConfig, rxRuntimeState, isFPORT2) confirms
// isFPORT2 only ever changes the baud rate passed to openSerialPort(), not
// the framing/checksum/dispatch logic - so one shared parser covers both,
// parameterized only by baud. Scoped to 16-channel frames only (the common
// case for modern D16/ACCESS receivers in FBUS mode); FBUS's 8ch/24ch
// frame-length variants are not decoded here (see rx_input_backup_fbus.c's
// top-of-file comment for why, and how to add them later).

bool rxInputBackupFbusInit(rxInputBackupOps_t *ops);
bool rxInputBackupFport2Init(rxInputBackupOps_t *ops);
