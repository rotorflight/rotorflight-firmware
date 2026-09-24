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

// Jeti EX Bus provider for the generic backup-RX framework
// (rx_input_backup.c). Decodes only the fixed 16-channel "Channel Data"
// frame (0x3E ...) rx/jetiexbus.c itself decodes - the other frame type on
// this bus, the FC-addressed telemetry/Jetibox request (0x3D ...), is simply
// never recognized as a frame start and so is ignored outright, matching
// this framework's usual "receive-only, no telemetry, ever" scope (this
// backup link never answers a request, so it should essentially never see
// one addressed to it in the first place).
//
// Unlike every other provider here, real EX Bus receivers are documented as
// requiring an active bus master (the FC) present on the wire, and
// rx/jetiexbus.c itself always opens MODE_RXTX + unconditional SERIAL_BIDIR
// for exactly that reason. This backup link never transmits at all
// (MODE_RX only, imposed generically by rx_input_backup.c) - verified from
// the protocol documentation that a Jeti EX Bus receiver broadcasts its
// channel-data frames on its own fixed schedule regardless of whether
// anything ever replies (the FC's reply only matters for telemetry), same
// assumption already relied on for every other provider's optional
// telemetry-response frame, but flagged here explicitly since this is the
// one protocol whose own reference driver treats transmit as mandatory
// rather than optional. Bench-test before trusting this in the air, same as
// every other provider added without direct hardware access this pass.

bool rxInputBackupExbusInit(rxInputBackupOps_t *ops);
