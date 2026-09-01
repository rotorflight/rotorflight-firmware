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

#ifdef USE_RX_INPUT_BACKUP_FBUS

#include "drivers/rx_input_backup_fbus.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

#include "rx/frsky_crc.h"
#include "rx/rx.h"
#include "rx/sbus_channels.h"

// This intentionally does not reuse rx/fbus.c's fbusRxInit()/fbusDataReceive() -
// that module keeps its frame-assembly state in function-local statics tied to
// the single main RX instance, plus a 15-deep ring buffer and a large amount of
// telemetry/OTA state we have no use for here (this link is receive-only, no
// telemetry, ever). This is a from-scratch minimal reimplementation of just the
// RC-frame framing, verified line-by-line against rx/fbus.c, reusing only the
// same reentrant rx/sbus_channels.c decode already shared by every provider in
// this framework, plus rx/frsky_crc.c's checksum helper fbus.c itself builds on.
//
// Scoped to 16-channel frames only: rx/fbus.c also supports 8ch/24ch frame
// lengths by dispatching to sbusChannelsDecode8ch()/24ch() instead, but this
// framework's ops.channelCount is fixed once at Init time (before any real
// frame has been seen), so supporting all three variants here would mean
// either guessing which one to declare before the first frame arrives, or
// re-plumbing the generic layer to support a channel count that can change
// after Init - out of scope for this pass. Adding 8ch/24ch support later is a
// small, additive change to this same file (one more length-to-decode-function
// case, exactly mirroring rx/fbus.c's own switch), not a rework.
//
// FBUS and FPort2 share this exact same frame format - rx/fbus.c's own
// fbusRxInit(rxConfig, rxRuntimeState, isFPORT2) confirms isFPORT2 only ever
// changes which baud constant gets passed to openSerialPort(), nothing else -
// so both rxInputBackupFbusInit() and rxInputBackupFport2Init() below share
// this file's one static parser/decoder, differing only in the baud they
// report back to the generic layer via ops->baudRate.

// Frame layout, no byte-stuffing: [length=24][type=0xFF][23-byte sbusChannels_t]
// [rssi][checksum], 27 bytes total on the wire. Constants reused verbatim from
// rx/fbus.c so they match the proven main-RX implementation exactly.
#define FBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define FBUS_INPUT_CONTROL_FRAME_LENGTH_16CH 24
#define FBUS_INPUT_FRAME_TYPE_RC 0xFF
#define FBUS_INPUT_FRAME_SIZE (FBUS_INPUT_CONTROL_FRAME_LENGTH_16CH + 3) // 27

// rx/fbus.c's own inter-byte timeout (FBUS_RX_TIMEOUT) - reused for two
// distinct purposes here, exactly as the real driver uses it for one: (a) a
// candidate length-byte at the start of a new frame is only trusted after
// this much silence, guarding against a coincidental in-frame byte value of
// 24 being mistaken for a frame start (fbus.c itself doesn't need this since
// it treats byte 0 purely as a value-match and lets the type-byte+checksum
// reject false positives instead - this is a deliberate addition, not
// something fbus.c is missing); (b) a genuine per-byte gap watchdog while
// mid-frame, matching fbus.c's own stuck-frame recovery mechanism exactly -
// unlike a total-elapsed-time-only check, this catches a glitch byte arriving
// after a long gap without waiting out the full frame-timeout window first.
#define FBUS_INPUT_INTERBYTE_TIMEOUT_US 120

// Total-frame-duration ceiling, for recovering a parser stuck mid-frame with
// no further gap ever appearing (belt-and-braces alongside the per-byte gap
// watchdog above, matching the SBUS provider's own style of a frameTime
// ceiling). Sized above FPort2's slower worst-case transmission time for this
// frame (27 bytes at 115200 baud ~= 2344us) with margin, then reused as-is
// for FBUS's own faster 460800 baud dispatch too - it only governs recovery
// latency after something has already gone wrong, not steady-state
// correctness, so one shared, generously-sized constant is safe for both.
#define FBUS_INPUT_TIME_NEEDED_PER_FRAME_US 3000

typedef struct fbusInputFrameData_s {
    uint8_t bytes[FBUS_INPUT_FRAME_SIZE];
    volatile timeUs_t startAtUs;
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
} fbusInputFrameData_t;

static fbusInputFrameData_t fbusInputFrameData;
static uint8_t fbusInputPendingFrame[FBUS_INPUT_FRAME_SIZE];
static volatile bool fbusInputPendingFrameReady = false;

static uint16_t fbusInputChannelData[RX_INPUT_BACKUP_MAX_CHANNEL];
static rxRuntimeState_t fbusInputRxRuntimeState;

static void fbusInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        fbusInputFrameData.startAtUs = 0;
        fbusInputFrameData.lastByteAtUs = 0;
        fbusInputFrameData.position = 0;
        fbusInputPendingFrameReady = false;
    }
}

static FAST_CODE void fbusInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t byteGap = fbusInputFrameData.lastByteAtUs == 0
        ? (timeDelta_t)FBUS_INPUT_INTERBYTE_TIMEOUT_US + 1
        : cmpTimeUs(nowUs, fbusInputFrameData.lastByteAtUs);
    const timeDelta_t frameTime = cmpTimeUs(nowUs, fbusInputFrameData.startAtUs);
    fbusInputFrameData.lastByteAtUs = nowUs;

    // Per-byte gap watchdog while mid-frame, matching rx/fbus.c's own stuck-frame
    // recovery (FBUS_RX_TIMEOUT) - a real gap this large can only mean the frame
    // in progress is stale (sender reset, byte lost, electrical glitch), not a
    // legitimate continuation.
    if (fbusInputFrameData.position > 0 && byteGap > FBUS_INPUT_INTERBYTE_TIMEOUT_US) {
        fbusInputFrameData.position = 0;
    }

    // Total-duration ceiling, belt-and-braces alongside the per-byte watchdog above.
    if (fbusInputFrameData.position > 0 && frameTime > FBUS_INPUT_TIME_NEEDED_PER_FRAME_US) {
        fbusInputFrameData.position = 0;
    }

    if (fbusInputFrameData.position == 0) {
        // Matches rx/fbus.c's own FS_CONTROL_FRAME_START exactly: accept purely
        // on value match, no silence requirement - the type-byte check just
        // below plus the checksum in update() reject false positives instead.
        // (An earlier draft of this file added a byteGap>120us requirement
        // here too, which isn't present in the real driver at all; that turned
        // out to be over-strict against real hardware - if the true inter-frame
        // gap in practice ever drops to <=120us, every frame would be silently
        // dropped and the link would never come up. Removed to match the
        // proven implementation instead of guessing at extra robustness.)
        if (c != FBUS_INPUT_CONTROL_FRAME_LENGTH_16CH) {
            return;
        }
        fbusInputFrameData.startAtUs = nowUs;
    }

    fbusInputFrameData.bytes[fbusInputFrameData.position++] = (uint8_t)c;

    if (fbusInputFrameData.position == 2 && fbusInputFrameData.bytes[1] != FBUS_INPUT_FRAME_TYPE_RC) {
        // Wrong type (OTA, or garbage) - abort immediately rather than collecting
        // 25 more bytes for a frame we're not going to accept anyway, same as
        // rx/fbus.c's own FS_CONTROL_FRAME_TYPE state does.
        fbusInputFrameData.position = 0;
        return;
    }

    if (fbusInputFrameData.position >= FBUS_INPUT_FRAME_SIZE) {
        // Snapshot into a separate holding buffer right here, rather than leaving
        // the completed frame sitting in fbusInputFrameData for the consumer to
        // read later - the next frame's first byte (landing back at position 0)
        // could otherwise start overwriting the same buffer being decoded,
        // tearing adjacent 11-bit channel fields across two frames.
        memcpy(fbusInputPendingFrame, fbusInputFrameData.bytes, FBUS_INPUT_FRAME_SIZE);
        fbusInputPendingFrameReady = true;
        fbusInputFrameData.position = 0;
    }
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static bool fbusInputUpdate(float *channels, uint8_t channelCount)
{
    uint8_t frame[FBUS_INPUT_FRAME_SIZE];
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (fbusInputPendingFrameReady) {
            memcpy(frame, fbusInputPendingFrame, FBUS_INPUT_FRAME_SIZE);
            fbusInputPendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return false;
    }

    // Checksum covers everything except the leading length byte (type, payload,
    // rssi, and the trailing checksum byte itself) - matches rx/fbus.c's own
    // frskyCheckSumIsGood((uint8_t *)buffer + 2, buflen - 2), which skips its
    // internal envelope-type tag byte plus the on-wire length byte; this buffer
    // has no such envelope byte, so only the length byte itself is skipped.
    if (!frskyCheckSumIsGood(&frame[1], FBUS_INPUT_FRAME_SIZE - 1)) {
        fbusInputResetParser();
        return false;
    }

    sbusChannels_t *wireChannels = (sbusChannels_t *)&frame[2];
    const uint8_t frameStatus = sbusChannelsDecode(&fbusInputRxRuntimeState, wireChannels);
    if (frameStatus & (RX_FRAME_DROPPED | RX_FRAME_FAILSAFE)) {
        // Same rationale as the SBUS provider: a dropped/failsafe frame from the
        // satellite itself must not count as a fresh valid frame.
        fbusInputResetParser();
        return false;
    }

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (5.0f * (float)fbusInputChannelData[i] / 8.0f) + 880.0f;
    }

    return true;
}

static bool fbusInputInitCommon(rxInputBackupOps_t *ops, uint32_t baudRate)
{
    fbusInputRxRuntimeState.channelData = fbusInputChannelData;
    fbusInputResetParser();

    // Matches rx/fbus.c's own direction/variant exactly (identical for FBUS and
    // FPort2 - isFPORT2 only changes baud, confirmed above): FBUS/FPort2's
    // signal is natively non-inverted, so inverted=OFF (normal wiring) leaves
    // the UART non-inverted by default - the opposite direction from SBUS's
    // own provider, which is why this can't be handled generically in
    // rx_input_backup.c. Half-duplex uses SERIAL_BIDIR|SERIAL_BIDIR_PP (push-
    // pull), unlike SBUS's plain SERIAL_BIDIR.
    ops->baudRate = baudRate;
    ops->portOptions = FBUS_INPUT_PORT_OPTIONS
        | (rxInputBackupConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (rxInputBackupConfig()->halfDuplex ? (SERIAL_BIDIR | SERIAL_BIDIR_PP) : SERIAL_UNIDIR);
    ops->isrFn = fbusInputDataReceive;
    ops->channelCount = RX_INPUT_BACKUP_MAX_CHANNEL;
    ops->update = fbusInputUpdate;

    return true;
}

bool rxInputBackupFbusInit(rxInputBackupOps_t *ops)
{
    return fbusInputInitCommon(ops, 460800);
}

bool rxInputBackupFport2Init(rxInputBackupOps_t *ops)
{
    return fbusInputInitCommon(ops, 115200);
}

#endif // USE_RX_INPUT_BACKUP_FBUS
