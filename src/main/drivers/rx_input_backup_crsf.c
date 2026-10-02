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

#ifdef USE_RX_INPUT_BACKUP_CRSF

#include "drivers/rx_input_backup_crsf.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

// This intentionally does not reuse rx/crsf.c's crsfRxInit()/crsfDataReceive() -
// that module keeps its frame-assembly state in function-local/file-scope
// statics tied to the single main RX instance, plus telemetry, MSP-over-CRSF,
// link-statistics/RSSI, and CRSFv3 baud-negotiation handling this backup link
// has no use for (receive-only, no telemetry, ever). This is a from-scratch
// minimal reimplementation of just the address+length-prefixed framing and
// RC_CHANNELS_PACKED decode, verified line-by-line against rx/crsf.c,
// reusing only the generic, already-reentrant common/crc.c's crc8_dvb_s2()
// the real driver itself builds on.
//
// Unlike the length-prefixed providers here with one fixed frame shape
// (FBUS, EX Bus), a real CRSF link legitimately sends several different
// frame types on the same wire even with no telemetry reply ever sent back -
// notably LINK_STATISTICS, broadcast by the receiver on its own schedule
// purely to report link quality/RSSI, which this backup link has no
// equivalent surface for and simply ignores. So this parser tracks generic
// dynamic frame length the same way rx/crsf.c's own crsfDataReceive() does
// (byte-accumulate to a length computed from the frame's own second byte),
// rather than rejecting anything that isn't the one shape this provider
// cares about at a fixed offset the way FBUS/EX Bus do - doing that here
// would lose byte alignment every time a same-length-but-different-type
// frame (or any other type) arrived, since the only way to know how many
// bytes a CRSF frame occupies is to read its own length byte, whatever type
// it turns out to be.

#define CRSF_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define CRSF_INPUT_BAUDRATE 420000
#define CRSF_INPUT_TIME_NEEDED_PER_FRAME_US 1750

// Matches rx/crsf.c's own constants exactly.
#define CRSF_INPUT_FRAME_SIZE_MAX 64
#define CRSF_INPUT_FRAME_LENGTH_ADDRESS 1
#define CRSF_INPUT_FRAME_LENGTH_FRAMELENGTH 1
#define CRSF_INPUT_FRAME_LENGTH_TYPE_CRC 2
#define CRSF_INPUT_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_INPUT_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_INPUT_FRAME_RC_CHANNELS_PAYLOAD_SIZE 22 // 11 bits * 16 channels

// Legacy 0x16 RC frame scale/offset - matches rx/crsf.c's own crsfReadRawRC()
// comment: min 172->988us, mid 992->1500us, max 1811->2012us.
#define CRSF_INPUT_RC_CHANNEL_SCALE 0.62477120195241f
#define CRSF_INPUT_RC_CHANNEL_OFFSET 881.0f

typedef struct crsfInputFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_INPUT_FRAME_SIZE_MAX - 3];
} __attribute__((packed)) crsfInputFrameDef_t;

// Union, not a raw pointer cast of a plain byte buffer - see
// rx_input_backup_fbus.c's own comment on why (C11 effective-type rules;
// this is the same pattern the SBUS provider's sbusInputFrameBuf_t already
// uses, and rx/crsf.c's own crsfFrame_t union does too).
typedef union crsfInputFrameBuf_u {
    uint8_t bytes[CRSF_INPUT_FRAME_SIZE_MAX];
    crsfInputFrameDef_t frame;
} crsfInputFrameBuf_t;

typedef struct crsfInputPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__((packed)) crsfInputPayloadRcChannelsPacked_t;

typedef struct crsfInputFrameData_s {
    crsfInputFrameBuf_t frame;
    volatile timeUs_t startAtUs;
    volatile uint8_t position;
} crsfInputFrameData_t;

static crsfInputFrameData_t crsfInputFrameData;
static crsfInputFrameBuf_t crsfInputPendingFrame;
static volatile bool crsfInputPendingFrameReady = false;

static uint32_t crsfInputChannelData[RX_INPUT_BACKUP_MAX_CHANNEL];

static void crsfInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        crsfInputFrameData.position = 0;
        crsfInputPendingFrameReady = false;
    }
}

// Receive ISR callback. Matches rx/crsf.c's own crsfDataReceive() dynamic
// frame-length tracking exactly, minus the immediate CRC/type dispatch it
// does inline - that's deferred to crsfInputUpdate() instead, consistent
// with every other provider in this framework doing checksum verification
// outside the ISR.
static FAST_CODE void crsfInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();

    if (crsfInputFrameData.position > 0
        && cmpTimeUs(nowUs, crsfInputFrameData.startAtUs) > CRSF_INPUT_TIME_NEEDED_PER_FRAME_US) {
        crsfInputFrameData.position = 0;
    }

    if (crsfInputFrameData.position == 0) {
        crsfInputFrameData.startAtUs = nowUs;
    }

    // Assume a 5-byte frame until enough bytes have arrived to trust the
    // real length field - matches rx/crsf.c's own fullFrameLength calc
    // exactly, including its use of 5 (not 3) as the floor: bumping the
    // switchover to position 3 (rather than 2, right when the length byte
    // itself becomes available) is harmless since 5 is smaller than any
    // frame this provider ever needs to fully accumulate.
    const uint8_t fullFrameLength = crsfInputFrameData.position < 3
        ? 5
        : MIN((uint16_t)crsfInputFrameData.frame.frame.frameLength + CRSF_INPUT_FRAME_LENGTH_ADDRESS + CRSF_INPUT_FRAME_LENGTH_FRAMELENGTH, CRSF_INPUT_FRAME_SIZE_MAX);

    if (crsfInputFrameData.position < fullFrameLength) {
        crsfInputFrameData.frame.bytes[crsfInputFrameData.position++] = (uint8_t)c;
        if (crsfInputFrameData.position >= fullFrameLength) {
            // Snapshot into a separate holding buffer right here, rather than
            // leaving the completed frame sitting in crsfInputFrameData for
            // the consumer to read later - see the SBUS/FBUS/FPort providers'
            // own identical comment for why.
            memcpy(&crsfInputPendingFrame, &crsfInputFrameData.frame, fullFrameLength);
            crsfInputPendingFrameReady = true;
            crsfInputFrameData.position = 0;
        }
    }
}

// Matches rx/crsf.c's own crsfFrameCRC(): CRC covers the type byte plus
// payload, not the leading address/length bytes or the trailing CRC byte
// itself. Takes an explicit, already-bounds-checked payload length rather
// than reading frame->frame.frameLength (the raw, unclamped wire byte)
// directly the way rx/crsf.c's own version does - that version loops
// straight off the untrusted length byte into a payload array only sized
// for the realistic max, which is a latent out-of-bounds read for a
// corrupt/hostile length byte. In rx/crsf.c that only ever over-reads its
// own static frame buffer (still UB, but functionally inert there); here
// `frame` is a stack local in the caller, where the same pattern would be a
// genuine stack over-read. Not worth carrying that risk over just to match
// the reference implementation line-for-line.
static uint8_t crsfInputFrameCRC(const crsfInputFrameBuf_t *frame, uint8_t payloadLength)
{
    uint8_t crc = crc8_dvb_s2(0, frame->frame.type);
    for (uint8_t i = 0; i < payloadLength; i++) {
        crc = crc8_dvb_s2(crc, frame->frame.payload[i]);
    }
    return crc;
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static bool crsfInputUpdate(float *channels, uint8_t channelCount)
{
    crsfInputFrameBuf_t frame;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (crsfInputPendingFrameReady) {
            memcpy(&frame, &crsfInputPendingFrame, sizeof(frame));
            crsfInputPendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return false;
    }

    // Widened to uint16_t and MIN-capped exactly like the ISR's own
    // fullFrameLength calc - frame.frame.frameLength is a raw uint8_t off
    // the wire (up to 255), so address+framelength+that would overflow a
    // uint8_t before the cap could apply. Capping here too keeps this in
    // sync with how many bytes the ISR actually captured into the buffer
    // being read below (it applied the same cap before ever setting
    // crsfInputPendingFrameReady).
    const uint16_t fullFrameLength = MIN((uint16_t)frame.frame.frameLength + CRSF_INPUT_FRAME_LENGTH_ADDRESS + CRSF_INPUT_FRAME_LENGTH_FRAMELENGTH, CRSF_INPUT_FRAME_SIZE_MAX);
    if (fullFrameLength < CRSF_INPUT_FRAME_LENGTH_ADDRESS + CRSF_INPUT_FRAME_LENGTH_FRAMELENGTH + CRSF_INPUT_FRAME_LENGTH_TYPE_CRC) {
        return false;
    }

    // Derived from the already-capped fullFrameLength, not frame.frame.type's
    // raw sibling frameLength byte directly - see crsfInputFrameCRC()'s own
    // comment for why. This is always <= CRSF_INPUT_FRAME_SIZE_MAX - 4,
    // safely within the payload[] array's bounds.
    const uint8_t payloadLength = (uint8_t)(fullFrameLength - CRSF_INPUT_FRAME_LENGTH_ADDRESS - CRSF_INPUT_FRAME_LENGTH_FRAMELENGTH - CRSF_INPUT_FRAME_LENGTH_TYPE_CRC);
    if (crsfInputFrameCRC(&frame, payloadLength) != frame.bytes[fullFrameLength - 1]) {
        return false;
    }

    // Anything other than an RC_CHANNELS_PACKED frame addressed to the FC -
    // LINK_STATISTICS, MSP, device ping, etc. - is silently ignored here,
    // same as a rejected/dropped frame elsewhere in this framework. Frame
    // sync is preserved regardless (the byte-accumulation above already
    // consumed exactly this frame's own length), so the next frame, of
    // whatever type, still starts cleanly.
    if (frame.frame.deviceAddress != CRSF_INPUT_ADDRESS_FLIGHT_CONTROLLER
        || frame.frame.type != CRSF_INPUT_FRAMETYPE_RC_CHANNELS_PACKED
        || frame.frame.frameLength != (CRSF_INPUT_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_INPUT_FRAME_LENGTH_TYPE_CRC)) {
        return false;
    }

    // Copied into a genuine object rather than pointer-cast straight out of
    // frame.frame.payload (a uint8_t array) - see rx_input_backup_fbus.c's
    // identical fix for why (C11 effective-type rules).
    crsfInputPayloadRcChannelsPacked_t rcChannels;
    memcpy(&rcChannels, frame.frame.payload, sizeof(rcChannels));
    crsfInputChannelData[0] = rcChannels.chan0;
    crsfInputChannelData[1] = rcChannels.chan1;
    crsfInputChannelData[2] = rcChannels.chan2;
    crsfInputChannelData[3] = rcChannels.chan3;
    crsfInputChannelData[4] = rcChannels.chan4;
    crsfInputChannelData[5] = rcChannels.chan5;
    crsfInputChannelData[6] = rcChannels.chan6;
    crsfInputChannelData[7] = rcChannels.chan7;
    crsfInputChannelData[8] = rcChannels.chan8;
    crsfInputChannelData[9] = rcChannels.chan9;
    crsfInputChannelData[10] = rcChannels.chan10;
    crsfInputChannelData[11] = rcChannels.chan11;
    crsfInputChannelData[12] = rcChannels.chan12;
    crsfInputChannelData[13] = rcChannels.chan13;
    crsfInputChannelData[14] = rcChannels.chan14;
    crsfInputChannelData[15] = rcChannels.chan15;

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = CRSF_INPUT_RC_CHANNEL_SCALE * (float)crsfInputChannelData[i] + CRSF_INPUT_RC_CHANNEL_OFFSET;
    }

    return true;
}

bool rxInputBackupCrsfInit(rxInputBackupOps_t *ops)
{
    memset(&crsfInputFrameData, 0, sizeof(crsfInputFrameData));
    crsfInputResetParser();

    // rx/crsf.c's own driver doesn't expose inverted/half-duplex as
    // configurable at all (CRSF_PORT_OPTIONS has neither, and CRSF_PORT_MODE
    // is unconditionally MODE_RXTX) - its own top-of-file comment describes
    // the signal as "not inverted" and the link as "single wire half duplex"
    // in the abstract, but a real board's own resource config, not a runtime
    // option, is what makes that specific wiring work for the main RX. This
    // backup link never transmits, so it's treated the same as every other
    // non-inverted provider here: natively non-inverted, with half-duplex
    // left user-configurable (plain SERIAL_BIDIR, matching SBUS/EX Bus) for
    // whichever way a given backup port is actually wired, rather than
    // assuming one specific convention CRSF's own driver doesn't actually
    // encode as a portOptions bit to copy.
    ops->baudRate = CRSF_INPUT_BAUDRATE;
    ops->portOptions = CRSF_INPUT_PORT_OPTIONS
        | (rxInputBackupConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (rxInputBackupConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = crsfInputDataReceive;
    ops->channelCount = 16;
    ops->update = crsfInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_CRSF
