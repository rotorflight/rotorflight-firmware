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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_SBUS_INPUT

#include "drivers/rx_sbus_input.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rx_sbus_input.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

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

// Same fixed baud/framing as the primary SBUS receiver (rx/sbus.c): 100000 baud, 8E2.
// Inversion/pin-swap are this port's own settings (pg/rx_sbus_input.h), independent
// of the main RX's serialrx_inverted/serialrx_pinswap - different physical UART.
#define SBUS_INPUT_BAUDRATE 100000
#if !defined(SBUS_INPUT_PORT_OPTIONS)
#define SBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_INPUT_FRAME_BEGIN_BYTE 0x0F
#define SBUS_INPUT_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)
#define SBUS_INPUT_TIME_NEEDED_PER_FRAME_US 4000

// Minimum silence before a 0x0F byte is trusted as a real frame start rather than a
// stray value occurring mid-frame. Only needs to comfortably clear normal back-to-back
// inter-byte spacing within a frame (~0us at this baud rate, since bytes are sent with
// no idle time between them) while staying well under the tightest real inter-frame
// gap this driver needs to support: a 25-byte frame takes ~3ms to transmit at 100000
// baud, and sbus_out_frame_rate goes up to 250Hz (4ms period), leaving only ~1ms of
// genuine idle gap at that rate - a larger threshold here would reject every real
// frame boundary and break reception entirely at the high end of a range this same
// codebase lets you configure.
#define SBUS_INPUT_INTERFRAME_GAP_US 500

// How long without a decoded frame before the SBUS-in link is considered down.
// ~3 missed SBUS frames at the fastest common frame rate (~6-14ms/frame).
#define SBUS_INPUT_STALE_MS 50

typedef struct sbusInputFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    uint8_t endByte;
} __attribute__((__packed__)) sbusInputFrame_t;

typedef union sbusInputFrameBuf_u {
    uint8_t bytes[SBUS_INPUT_FRAME_SIZE];
    sbusInputFrame_t frame;
} sbusInputFrameBuf_t;

typedef struct sbusInputFrameData_s {
    sbusInputFrameBuf_t frame;
    volatile timeUs_t startAtUs;
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
    volatile bool done;
} sbusInputFrameData_t;

static serialPort_t *sbusInputPort = NULL;

static sbusInputFrameData_t sbusInputFrameData;
static sbusChannels_t sbusInputPendingChannels;
static volatile bool sbusInputPendingFrame = false;

static uint16_t sbusInputChannelData[SBUS_INPUT_MAX_CHANNEL];
static float sbusInputChannel[SBUS_INPUT_MAX_CHANNEL];
static timeMs_t sbusInputLastValidFrameMs = 0;

// False until the first genuinely valid (non-dropped, non-failsafe) frame has been
// decoded. Without this, sbusInputIsActive() would read as "active" for up to
// SBUS_INPUT_STALE_MS right after boot/config-change purely because
// sbusInputLastValidFrameMs's zero-init happens to be within that window of
// millis()'s own startup value - reporting fallback available (and, if the main
// link were already down at that moment, feeding zeroed channels) before any real
// frame has ever been seen.
static bool sbusInputHasValidFrame = false;
static uint16_t sbusInputFrameErrorCount = 0;

// Minimal rxRuntimeState_t used only to satisfy sbusChannelsDecode()'s interface -
// only its channelData pointer is touched by that function.
static rxRuntimeState_t sbusInputRxRuntimeState;

// Note: deliberately does not validate the frame's trailing endByte against known
// SBUS1/SBUS2 markers, and does not sanity-range-check decoded channel values beyond
// what sbusChannelsDecode() itself already does. The primary RX path (rx/sbus.c) never
// rejects a frame for either reason either - it just doesn't recognize an unfamiliar
// endByte as an SBUS2 telemetry page - and being stricter here than that proven
// decoder risks discarding real, valid frames from otherwise-fine hardware for no
// actual gain.
static void sbusInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sbusInputFrameData.startAtUs = 0;
        sbusInputFrameData.lastByteAtUs = 0;
        sbusInputFrameData.position = 0;
        sbusInputFrameData.done = false;
        sbusInputPendingFrame = false;
    }
}

static FAST_CODE void sbusInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t frameTime = cmpTimeUs(nowUs, sbusInputFrameData.startAtUs);
    // No prior byte to compare against yet (right after init/reset) - treat that as
    // "definitely enough silence" rather than "definitely not enough", so the very
    // next real sync byte isn't unconditionally thrown away.
    const timeDelta_t byteGap = sbusInputFrameData.lastByteAtUs == 0
        ? (timeDelta_t)SBUS_INPUT_INTERFRAME_GAP_US
        : cmpTimeUs(nowUs, sbusInputFrameData.lastByteAtUs);
    sbusInputFrameData.lastByteAtUs = nowUs;

    if (frameTime > (long)(SBUS_INPUT_TIME_NEEDED_PER_FRAME_US + 500)) {
        sbusInputFrameData.position = 0;
    }

    if (sbusInputFrameData.position == 0) {
        if (c != SBUS_INPUT_FRAME_BEGIN_BYTE || byteGap < SBUS_INPUT_INTERFRAME_GAP_US) {
            return;
        }
        sbusInputFrameData.startAtUs = nowUs;
    }

    if (sbusInputFrameData.position < SBUS_INPUT_FRAME_SIZE) {
        sbusInputFrameData.frame.bytes[sbusInputFrameData.position++] = (uint8_t)c;
        if (sbusInputFrameData.position >= SBUS_INPUT_FRAME_SIZE) {
            // Snapshot into a separate holding buffer right here, rather than leaving
            // the completed frame sitting in sbusInputFrameData for the consumer to
            // read later - that would leave a window for the next frame's first byte
            // (landing back at position 0) to start overwriting the same buffer being
            // decoded, tearing adjacent 11-bit channel fields across two frames.
            memcpy(&sbusInputPendingChannels, &sbusInputFrameData.frame.frame.channels, sizeof(sbusInputPendingChannels));
            sbusInputPendingFrame = true;
            sbusInputFrameData.position = 0;
            sbusInputFrameData.done = true;
        } else {
            sbusInputFrameData.done = false;
        }
    }
}

// Called from the RX task (rx/rx.c), not an ISR - safe to do the heavier decode/convert work here.
static void sbusInputUpdate(void)
{
    // Snapshot the completed frame into a local copy under a brief interrupt mask,
    // rather than decoding directly out of sbusInputFrameData - which the receive
    // ISR owns and can start overwriting (a new frame's first byte landing at
    // position 0) at any point after `done` is observed true but before the decode
    // below finishes reading it. That window used to be able to produce a torn read
    // mixing bytes from two different frames, corrupting adjacent 11-bit channel
    // fields (e.g. one channel's movement bleeding into its neighbour).
    sbusChannels_t channels;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sbusInputPendingFrame) {
            memcpy(&channels, &sbusInputPendingChannels, sizeof(channels));
            sbusInputPendingFrame = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return;
    }

    const uint8_t frameStatus = sbusChannelsDecode(&sbusInputRxRuntimeState, &channels);
    if (frameStatus & (RX_FRAME_DROPPED | RX_FRAME_FAILSAFE)) {
        // RX_FRAME_DROPPED: repeated/stale data from the satellite itself.
        // RX_FRAME_FAILSAFE: the satellite's own internal failsafe is active - it may
        // still be sending numerically valid-looking (repeated/center) channel data,
        // but this must not count as a fresh valid frame, same as rxFrameCheck() never
        // treats a main-RX RX_FRAME_FAILSAFE frame as "signal received" either. Letting
        // it through here would let a satellite that has itself lost its own uplink
        // keep reporting the fallback as active and calling failsafeOnValidDataReceived()
        // (rx.c) - suppressing real failsafe in exactly the scenario, both links
        // actually down, that it exists to catch.
        sbusInputFrameErrorCount++;
        sbusInputResetParser();
        return;
    }

    for (int i = 0; i < SBUS_INPUT_MAX_CHANNEL; i++) {
        sbusInputChannel[i] = (5.0f * (float)sbusInputChannelData[i] / 8.0f) + 880.0f;
    }

    sbusInputHasValidFrame = true;
    sbusInputLastValidFrameMs = millis();
}

bool sbusInputIsEnabled(void)
{
    return sbusInputPort != NULL;
}

// Decodes any newly-completed frame and updates the channel/freshness state.
// Must be called every cycle regardless of whether the main RX link is up or
// the fallback is currently "needed" - it used to be called only as a side
// effect of sbusInputIsActive(), which rx.c only evaluates once the main
// link is already down (short-circuiting `!rxSignalReceived && ...`). That
// starved this of any real-time decoding whenever the main link was healthy,
// leaving diagnostics/MSP polling as the only thing driving it (once every
// poll interval instead of every cycle) and meaning the very first fallback
// frame used at the instant of a real failover could already be stale.
void sbusInputPoll(void)
{
    if (!sbusInputIsEnabled()) {
        return;
    }

    sbusInputUpdate();

    if (sbusInputHasValidFrame && (timeMs_t)(millis() - sbusInputLastValidFrameMs) >= SBUS_INPUT_STALE_MS) {
        sbusInputResetParser();
        sbusInputHasValidFrame = false;
        sbusInputFrameErrorCount++;
    }
}

bool sbusInputIsActive(void)
{
    if (!sbusInputIsEnabled() || !sbusInputHasValidFrame) {
        return false;
    }

    return (timeMs_t)(millis() - sbusInputLastValidFrameMs) < SBUS_INPUT_STALE_MS;
}

uint8_t sbusInputGetChannelCount(void)
{
    return SBUS_INPUT_MAX_CHANNEL;
}

float sbusInputGetChannel(uint8_t channel)
{
    if (channel >= SBUS_INPUT_MAX_CHANNEL) {
        return 0;
    }
    return sbusInputChannel[channel];
}

void sbusInputInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SBUS_INPUT);
    if (!portConfig) {
        sbusInputPort = NULL;
        return;
    }

    sbusInputRxRuntimeState.channelData = sbusInputChannelData;
    sbusInputFrameData.startAtUs = 0;
    sbusInputFrameData.lastByteAtUs = 0;
    sbusInputFrameData.position = 0;
    sbusInputFrameData.done = false;
    sbusInputPendingFrame = false;
    sbusInputLastValidFrameMs = 0;
    sbusInputHasValidFrame = false;
    sbusInputFrameErrorCount = 0;

    sbusInputPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SBUS_INPUT,
        sbusInputDataReceive,
        NULL,
        SBUS_INPUT_BAUDRATE,
        MODE_RX,
        SBUS_INPUT_PORT_OPTIONS |
            (sbusInputConfig()->inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED) |
            (sbusInputConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

#endif // USE_RX_SBUS_INPUT
