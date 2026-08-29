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

// A secondary, independent SBUS receiver input used as a fallback when the main RX
// link's signal is lost - bypasses the *staged* failsafe machinery entirely, but is
// bounded by the main RX's own existing ~100ms signal-loss detection window
// (rxSignalReceived/DELAY_100_MS in rx.c's rxFrameCheck()), not per-missed-frame; see
// the comment above the takeover branch in rx.c's detectAndApplySignalLossBehaviour()
// for why that's a deliberate choice over a feature-specific fixed threshold.
// Electrical settings (inversion/pin swap) are its own config, pg/rx_sbus_input.h -
// independent from the main RX's serialrx_inverted/serialrx_pinswap, since this is a
// different physical UART.
//
// This intentionally does not reuse rx/sbus.c's sbusInit()/sbusDataReceive() - that
// module keeps its frame-assembly state in function-local statics tied to the single
// main RX instance (and touches other main-RX-only globals like rssiSource), so it
// cannot be safely instantiated a second time. Only the reentrant channel decode in
// rx/sbus_channels.c (sbusChannelsDecode()) is shared between the two.

#define SBUS_INPUT_MAX_CHANNEL 18

void sbusInputInit(void);

// Decodes any newly-completed SBUS frame and refreshes channel/freshness
// state. Must be called every cycle from the RX task regardless of main-link
// state - see rx.c's detectAndApplySignalLossBehaviour() for why this can't
// just be a side effect of sbusInputIsActive() any more.
void sbusInputPoll(void);

// True once a serial port has been assigned FUNCTION_RX_SBUS_INPUT.
bool sbusInputIsEnabled(void);

// True when enabled AND a valid SBUS frame has been decoded within the last
// SBUS_INPUT_STALE_MS - i.e. the SBUS-in link is currently healthy and its
// channel data should be trusted as a fallback for the main RX.
bool sbusInputIsActive(void);

// Number of channels the SBUS-in decoder provides (always SBUS_INPUT_MAX_CHANNEL).
uint8_t sbusInputGetChannelCount(void);

// Channel value in the same convention as rx/rx.c's rcInput[]/rcChannel[] (~880-2012us).
float sbusInputGetChannel(uint8_t channel);
