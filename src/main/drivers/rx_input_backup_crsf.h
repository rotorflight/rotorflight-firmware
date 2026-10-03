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

// CRSF (Crossfire/ELRS-family) provider for the generic backup-RX framework
// (rx_input_backup.c). Unlike every other provider here, a real CRSF link
// multiplexes several distinct frame types on the same wire (RC channels,
// link statistics, MSP, device ping, etc.) even with no telemetry reply ever
// sent back - so this provider tracks generic address+length-prefixed
// framing exactly like rx/crsf.c itself does (byte-accumulate to a
// dynamically-computed frame length, verify CRC, then look at the type byte)
// rather than hardcoding one fixed frame shape the way the length-prefixed
// providers here (FBUS, EX Bus) do; only CRSF_FRAMETYPE_RC_CHANNELS_PACKED
// is ever decoded, everything else is silently skipped once its own CRC
// check passes, keeping frame sync intact across frame types this backup
// link has no use for.

bool rxInputBackupCrsfInit(rxInputBackupOps_t *ops);
