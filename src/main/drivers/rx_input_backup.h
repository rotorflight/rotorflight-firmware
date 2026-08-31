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
#include <stdint.h>

#include "drivers/serial.h"

// A secondary, independent RX input used as a fallback when the main RX link's
// signal is lost. See rx_input_backup.c for the parser/failover rationale and
// rx.c's detectAndApplySignalLossBehaviour() for the takeover logic itself.
//
// Provider-selectable (pg/rx_input_backup.h's `provider` field): only SBUS is
// implemented today, but the split between this generic file and one file per
// provider (e.g. rx_input_backup_sbus.c) exists so adding another protocol is
// "add one file + one dispatch case", not a rework of this file. There is
// deliberately no telemetry support on this link, ever - it exists purely to
// hand over channel data, same as a physical backup satellite receiver would.

// Keep in sync with cli/settings.c's lookupTableRxInputBackupProvider[] (same
// order) and pg/rx_input_backup.h's `provider` field width (uint8_t).
typedef enum {
    RX_INPUT_BACKUP_SBUS = 0,
    // RX_INPUT_BACKUP_FBUS, RX_INPUT_BACKUP_FPORT, ... added here as they land.
} rxInputBackupProvider_e;

#define RX_INPUT_BACKUP_MAX_CHANNEL 18

// Populated by a provider's own Init function (see rx_input_backup_sbus.h for
// the SBUS example) and consumed only by rx_input_backup.c - not part of the
// public API the rest of the firmware uses (that's the plain functions below).
typedef struct rxInputBackupOps_s {
    uint32_t baudRate;
    portOptions_e portOptions;  // protocol-fixed framing/direction; electrical
                                 // inversion/pin-swap are applied on top from
                                 // the user's own config by rx_input_backup.c,
                                 // uniformly across every provider.
    serialReceiveCallbackPtr isrFn;
    uint8_t channelCount;       // <= RX_INPUT_BACKUP_MAX_CHANNEL

    // Called once per rxInputBackupPoll() cycle. Returns true and fills
    // channels[0..channelCount) if a new, valid frame was decoded this call;
    // returns false (leaving channels untouched) otherwise - no frame pending
    // yet, or the pending frame was rejected (e.g. dropped/failsafe).
    bool (*update)(float *channels, uint8_t channelCount);
} rxInputBackupOps_t;

void rxInputBackupInit(void);

// Decodes any newly-completed frame and refreshes channel/freshness state.
// Must be called every cycle from the RX task regardless of main-link state -
// see rx.c's detectAndApplySignalLossBehaviour() for why this can't just be a
// side effect of rxInputBackupIsActive() any more.
void rxInputBackupPoll(void);

// True once a serial port has been assigned FUNCTION_RX_INPUT_BACKUP.
bool rxInputBackupIsEnabled(void);

// True when enabled AND a valid frame has been decoded within the freshness
// window - i.e. the backup link is currently healthy and its channel data
// should be trusted as a fallback for the main RX.
bool rxInputBackupIsActive(void);

// Number of channels the selected provider decodes.
uint8_t rxInputBackupGetChannelCount(void);

// Currently selected provider (valid once rxInputBackupIsEnabled() is true).
rxInputBackupProvider_e rxInputBackupGetProvider(void);

// Channel value in the same convention as rx/rx.c's rcInput[]/rcChannel[] (~880-2012us).
float rxInputBackupGetChannel(uint8_t channel);
