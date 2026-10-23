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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Parameter log: state, the shadow, the journal ring, the capture, operations, the drain, the scan and the
 * runtime polls.
 *
 * Journal events (CUSTOM_STRING, at most 128 chars):
 *
 *   P<type><seq> <at> <fields>*<crc>
 *
 *   type    C change, A apply, R runtime, M marker, L lost, Q end, + continuation of the record <seq>
 *   seq     record number in this log, lowercase hex, from 1. The S-frame field paramSeq is the last one.
 *   at      "p" (before T0), <iteration>.<tick>, or <iteration>.<tick>~<iteration>.<tick> (an interval)
 *   crc     CRC-8 (polynomial 0xD5, initial value 0) of the chars before '*', two uppercase hex digits
 *
 *   C   s=<src>[.<arg>] [pgs=<pgn> [part=1]] n=<items> <key>=<new><<old> ...   ('+' events continue the items)
 *   A   <loader>[/<slot>] [fp=<hex>]
 *   R   gov_mode=<n> features=<hex> fp.pid=<hex> fp.gov=<hex> fp.sp=<hex>   (any subset)
 *   M   eesave us=<n> | eeload | escparam n=<len> fnv=<hex> | shadow-reset
 *   L   pgs=<pgn>,...                                                     ('+' events continue the list)
 *   Q   unsent=<n> lostrec=<n>
 *
 * Records of source u (no hook saw the write), v (a change between logs) and y (a resync after a ring
 * overflow) have their own record for each group, with an interval from the last point where the group was
 * proved equal to the shadow. A y record gives "pgs=<pgn>", also when it has no items. A y record that does not
 * complete its group has "part=1": the 'L' loss of the group goes on until a y record without it.
 *
 * The task that changes the configuration captures the change: a word compare of the groups with the shadow
 * (the PG copies) and a walk of the changed words. The records wait in a RAM ring. The PID task writes at most
 * one event after each logged frame, and only when 256 B stay free in the device buffer for frames.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "cli/settings.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "fc/core.h"

#include "flight/governor.h"
#include "flight/pid.h"
#include "flight/setpoint.h"

#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_rotorflight.h"

#include "pg/blackbox.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/system.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"
#include "blackbox/blackbox_params.h"
#include "blackbox/blackbox_params_impl.h"

// Optional bench instrumentation. Read these counters with a debugger; no log fields or controller state change.
// Index: HeaderTick, WriteHeader, Running, AfterFrame, OpBegin, OpEnd, outer operation span.
// Counts and maxima include interrupt latency; the operation span also includes the configuration writer/loaders.
#if defined(BLACKBOX_PARAMS_PROFILE) && !defined(UNIT_TEST) && !defined(SIMULATOR_BUILD)
#define BBP_PROFILE_ENABLED
volatile uint32_t bbpProfileCalls[7], bbpProfileMaxCycles[7];
static uint32_t bbpProfileOpStart;
#define blackboxParamsHeaderTick bbpInnerHeaderTick
#define blackboxParamsWriteHeader bbpInnerWriteHeader
#define blackboxParamsRunning bbpInnerRunning
#define blackboxParamsAfterFrame bbpInnerAfterFrame
#define blackboxParamsOpBegin bbpInnerOpBegin
#define blackboxParamsOpEnd bbpInnerOpEnd
static void bbpProfileRecord(unsigned i, uint32_t start)
{
    const uint32_t elapsed = DWT->CYCCNT - start;
    bbpProfileCalls[i]++;
    if (elapsed > bbpProfileMaxCycles[i]) {
        bbpProfileMaxCycles[i] = elapsed;
    }
}
#endif

#define BBP_RING_SIZE       1024    // binary records, a power of 2
#define BBP_RING_KEEP       32      // ring bytes kept free for an 'L' record
#define BBP_REC_HEADER      22
#define BBP_FRAME_RESERVE   256     // device bytes left free for frames after an event (about 4 frames)
#define BBP_HOLD_MS         100     // longest delay of LOG_END while records wait
#define BBP_RESYNC_FREE     256     // ring bytes free before a resync after an overflow
#define BBP_DIFF_WORDS      (BBP_PG_MAX_BYTES / 128)
#define BBP_ARG_PART        0x8000  // ring arg of a 'y' record (the pgn): the record does not complete the group
#define BBP_STEP_UNITS      128     // work of one capture step in the PID task (units of about 10 cycles, spec 3.7)

STATIC_ASSERT((BBP_RING_SIZE & (BBP_RING_SIZE - 1)) == 0, bbp_ring_size_power_of_2);

// Runtime values that the journal polls
enum {
    BBP_RT_GOV_MODE = 0,
    BBP_RT_FEATURES,
    BBP_RT_FP_PID,
    BBP_RT_FP_GOV,
    BBP_RT_FP_SP,
    BBP_RT_COUNT
};

static const char * const runtimeNames[BBP_RT_COUNT] = {
    "gov_mode=", "features=", "fp.pid=", "fp.gov=", "fp.sp="
};

static const char * const markerNames[BBP_MARKER_COUNT] = {
    "eesave", "eeload", "escparam", "shadow-reset"
};

static const bbpPoint_t pointPre = { BBP_PRE, 0 };

bbpPg_t bbpPg[BBP_TRACKED];
uint32_t bbpPgAllInHeader;

// Changes of the configuration that the journal saw, and loads: blackboxParamsChangeCount()
static uint32_t bbpChanges;
#ifdef UNIT_TEST
unsigned bbpTestHashBytes;
#endif

// One header line or one journal event
static char bbpLine[(BBP_LINE_MAX > BBP_EVENT_MAX) ? BBP_LINE_MAX : BBP_EVENT_MAX + 1];

static uint8_t bbpRing[BBP_RING_SIZE];

// The last point at which group t was proved equal to the shadow, whole
static bbpPoint_t bbpVerified[BBP_TRACKED];

static struct {
    bool        active;             // the log has a parameter section and a journal
    bool        shadowValid;        // the shadow is a complete copy of the tracked groups
    bool        t0;                 // the log is RUNNING (T0 passed)

    uint32_t    allMask;            // the tracked groups of this build
    uint32_t    wordMask;           // groups that the capture compares in words (aligned live RAM and shadow)
    uint32_t    profileOnlyMask;    // groups whose valueTable entries are all profile values
    int8_t      systemIndex;        // SYSTEM_CONFIG in bbpTrackedPgs

    // Header
    uint8_t     lineLength;         // the header line in bbpLine
    uint8_t     linePos;            // its bytes that are written

    // Operations
    uint8_t     opDepth;
    char        opSrc;
    uint16_t    opArg;

    // Records
    uint32_t    seq;                // records created in this log: S-frame paramSeq
    uint32_t    prevMask;           // groups not yet compared with the previous log
    uint32_t    lostMask;           // groups with differences that the ring did not take
    uint32_t    lostListed;         // ... that an 'L' record lists
    bbpPoint_t  lostAt;
    uint16_t    lostRec;            // records with a seq that are not in the ring (A, R, M)
    uint16_t    ringHead;           // free running
    uint16_t    ringTail;

    // Drain
    bool        eventReady;         // an event is in bbpLine
    bool        drainStarted;       // the first event of the record at ringTail is written
    bool        drainDone;          // the event in bbpLine is the last of its record
    uint16_t    drainPos;           // next item of that record (L: next group)
    uint16_t    drainNext;
    uint8_t     eventLength;

    // Scan
    uint8_t     scanPg;
    uint16_t    scanOff;
    bbpPoint_t  scanPassStart;

    // Runtime polls
    uint8_t     rtIndex;
    uint8_t     rtTick;
    uint32_t    rtLast[BBP_RT_COUNT];
    bbpPoint_t  rtPolled[BBP_RT_COUNT];

    // LOG_END hold
    bool        holding;
    uint32_t    holdIteration;
    timeMs_t    holdStart;
} bbp;

// The record that a capture builds
static struct {
    bool        open;
    bool        partial;            // the record does not complete its group (items did not fit)
    uint8_t     type;
    char        src;
    uint16_t    arg;
    uint32_t    seq;
    bbpPoint_t  from;
    bbpPoint_t  at;
    uint16_t    start;
    uint16_t    pos;
    uint16_t    count;
} rec;

// A walk of the changed words of a group (capture): its stage and position
enum {
    WALK_COMPARE = 0,               // the steps of the PID task only: the compare, into the bitmap
    WALK_NAMED,
    WALK_ELEMENTS,
    WALK_RAW,
    WALK_DONE
};

typedef struct walk_s {
    uint8_t     stage;
    uint8_t     k;                  // named: the instance
    uint16_t    i;                  // compare, raw: the byte. Named: the valueTable entry. Elements: the element.
    int32_t     budget;             // work units left
} walk_t;

// The capture of one group in the PID task, in steps (stepGroup)
static struct {
    int8_t      t;                  // the group, -1: none
    bbpPoint_t  compared;           // the point of the compare: the verified point when the walk is done
    walk_t      walk;
    uint32_t    map[BBP_DIFF_WORDS];
} step;

/* Tracked groups */

// bbpTrackedIndex(): the last group that it looked for. valueTable is grouped by parameter group.
static pgn_t cachedPgn = BBP_NONE;
static int8_t cachedIndex = -1;

int bbpTrackedIndex(pgn_t pgn)
{
    if (pgn != cachedPgn) {
        cachedPgn = pgn;
        cachedIndex = -1;
        for (int t = 0; t < BBP_TRACKED; t++) {
            if (bbpTrackedPgs[t].pgn == pgn) {
                if (bbpPg[t].reg) {
                    cachedIndex = t;
                }
                break;
            }
        }
    }
    return cachedIndex;
}

static bool aligned(const void *p)
{
    return ((uintptr_t)p & 3) == 0;
}

void blackboxParamsInit(void)
{
#ifdef BBP_PROFILE_ENABLED
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    bbp.allMask = 0;
    bbp.wordMask = 0;
    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = pgFind(bbpTrackedPgs[t].pgn);
        // A size other than that of the table would overrun the copies
        bbpPg[t].reg = (reg && pgSize(reg) == bbpTrackedPgs[t].size && pgSize(reg) <= BBP_DIFF_WORDS * 128) ? reg : NULL;
        bbpPg[t].vtFirst = 0;
        bbpPg[t].vtEnd = 0;
        if (bbpPg[t].reg) {
            bbp.allMask |= BBP_BIT(t);
            if (aligned(reg->address) && aligned(bbpShadow(t))) {
                bbp.wordMask |= BBP_BIT(t);
            }
        }
    }
    cachedPgn = BBP_NONE;
    bbp.systemIndex = bbpTrackedIndex(PG_SYSTEM_CONFIG);
    step.t = -1;

    bbpPgAllInHeader = 0;
    uint32_t tooLong = 0;
    uint32_t master = 0;
    for (unsigned i = 0; i < valueTableEntryCount; i++) {
        const int t = bbpTrackedIndex(valueTable[i].pgn);
        if (t >= 0) {
            if (bbpPg[t].vtEnd == 0) {
                bbpPg[t].vtFirst = i;
            }
            bbpPg[t].vtEnd = i + 1;
            if (!bbpValueInHeader(&valueTable[i])) {
                tooLong |= BBP_BIT(t);
            }
            const int section = valueTable[i].type & VALUE_SECTION_MASK;
            if (section != PROFILE_VALUE && section != PROFILE_RATE_VALUE) {
                master |= BBP_BIT(t);
            }
        }
    }
    bbpPgAllInHeader = ~tooLong;
    bbp.profileOnlyMask = ~master;

    bbpTablesInit();
    bbpCoverageInit();
}

/* Word loops: newlib-nano memcpy and memcmp work one byte per loop */

static void copyToShadow(int t)
{
    const pgRegistry_t *reg = bbpPg[t].reg;
    const unsigned size = pgSize(reg);
    unsigned i = 0;

    if (bbp.wordMask & BBP_BIT(t)) {
        const uint32_t *src = (const uint32_t *)reg->address;
        uint32_t *dst = (uint32_t *)bbpShadow(t);
        for (; i < size / 4; i++) {
            dst[i] = src[i];
        }
        i *= 4;
    }
    for (; i < size; i++) {
        bbpShadow(t)[i] = reg->address[i];
    }
}

// The first block of 8 words from word w with a difference, or the first block that does not fit before wEnd.
// The hot loop of the capture: about 1.2 cycles per byte on a Cortex-M4.
static unsigned firstChangedBlock(const uint32_t *a, const uint32_t *b, unsigned w, unsigned wEnd)
{
    const uint32_t *p = a + w;
    const uint32_t *q = b + w;

    for (; w + 8 <= wEnd; w += 8, p += 8, q += 8) {
        const uint32_t d = (p[0] ^ q[0]) | (p[1] ^ q[1]) | (p[2] ^ q[2]) | (p[3] ^ q[3]) |
            (p[4] ^ q[4]) | (p[5] ^ q[5]) | (p[6] ^ q[6]) | (p[7] ^ q[7]);
        if (d) {
            break;
        }
    }
    return w;
}

// Bytes start..end-1 of group t differ from the shadow. start is a multiple of 4. The test of most groups in
// most captures: no bitmap.
static bool groupDiffers(int t, unsigned start, unsigned end)
{
    const uint8_t *live = bbpPg[t].reg->address;
    const uint8_t *shadow = bbpShadow(t);
    unsigned i = start;

    if (bbp.wordMask & BBP_BIT(t)) {
        const uint32_t *a = (const uint32_t *)live;
        const uint32_t *b = (const uint32_t *)shadow;
        const unsigned wEnd = end / 4;
        unsigned w = firstChangedBlock(a, b, start / 4, wEnd);
        if (w + 8 <= wEnd) {
            return true;
        }
        for (; w < wEnd; w++) {
            if (a[w] != b[w]) {
                return true;
            }
        }
        i = wEnd * 4;
    }
    for (; i < end; i++) {
        if (live[i] != shadow[i]) {
            return true;
        }
    }
    return false;
}

// Compare bytes start..end-1 of group t with the shadow, into map. Bit w of map: bytes 4w..4w+3 differ.
static bool diffRange(int t, unsigned start, unsigned end, uint32_t *map)
{
    const uint8_t *live = bbpPg[t].reg->address;
    const uint8_t *shadow = bbpShadow(t);
    uint32_t any = 0;

    unsigned w = start / 4;
    if (bbp.wordMask & BBP_BIT(t)) {
        const uint32_t *a = (const uint32_t *)live;
        const uint32_t *b = (const uint32_t *)shadow;
        const unsigned wEnd = end / 4;
        while (w + 8 <= wEnd) {
            w = firstChangedBlock(a, b, w, wEnd);
            if (w + 8 > wEnd) {
                break;
            }
            for (unsigned k = w; k < w + 8; k++) {
                if (a[k] != b[k]) {
                    map[k / 32] |= BBP_BIT(k % 32);
                }
            }
            any = 1;
            w += 8;
        }
        for (; w < wEnd; w++) {
            if (a[w] != b[w]) {
                map[w / 32] |= BBP_BIT(w % 32);
                any = 1;
            }
        }
    }
    for (unsigned i = w * 4; i < end; i++) {
        if (live[i] != shadow[i]) {
            map[i / 128] |= BBP_BIT((i / 4) % 32);
            any = 1;
        }
    }
    return any;
}

static void clearMap(uint32_t *map)
{
    for (int i = 0; i < BBP_DIFF_WORDS; i++) {
        map[i] = 0;
    }
}

static bool diffGroup(int t, unsigned start, unsigned end, uint32_t *map)
{
    clearMap(map);
    return diffRange(t, start, end, map);
}

// A changed word covers some of the bytes offset..offset+width-1. One test for each bitmap word (128 B).
static bool mapHas(const uint32_t *map, unsigned offset, unsigned width)
{
    const unsigned first = offset / 4;
    const unsigned last = (offset + width - 1) / 4;

    for (unsigned w = first; w <= last; w = (w | 31) + 1) {
        uint32_t bits = map[w / 32] >> (w % 32);
        if (last - w < 31) {
            bits &= (2u << (last - w)) - 1;
        }
        if (bits) {
            return true;
        }
    }
    return false;
}

static bool bytesDiffer(const uint8_t *a, const uint8_t *b, unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        if (a[i] != b[i]) {
            return true;
        }
    }
    return false;
}

static void copyBytes(uint8_t *dst, const uint8_t *src, unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        dst[i] = src[i];
    }
}

// Little endian, 1-4 bytes
static uint32_t readBytes(const uint8_t *p, unsigned count)
{
    uint32_t value = 0;
    for (unsigned i = count; i > 0; i--) {
        value = (value << 8) | p[i - 1];
    }
    return value;
}

static void writeBytes(uint8_t *p, unsigned count, uint32_t value)
{
    for (unsigned i = 0; i < count; i++) {
        p[i] = value >> (8 * i);
    }
}

/* Points */

static bbpPoint_t pointNow(void)
{
    if (!blackboxIsLogRunning()) {
        return pointPre;
    }
    // Ticks 0..c-1 of this PID cycle have run. With c > b, blackboxUpdate of this cycle already advanced the
    // iteration: the subtasks of the next frame with a tick up to b run after the change, so the point is N.0. A late
    // subtask (tick after b: flush, and the filter update with pid_process_denom 3) acts on the next frame from the
    // current cycle. For it, the stamp N.0 is uncertain in frame N, and N.c with c >= 1 is old in frame N (spec 2.5).
    const uint8_t c = getPidUpdateCounter();
    const bbpPoint_t point = { blackboxGetIteration(), (c <= coreSubtaskTick(CORE_ST_BLACKBOX)) ? c : 0 };
    return point;
}

// In the PID task, after the frame of this iteration: the next frame is new in all fields
static bbpPoint_t pointAfterFrame(void)
{
    const bbpPoint_t point = { blackboxGetIteration() + 1, 0 };
    return point;
}

static bool pointAfter(bbpPoint_t a, bbpPoint_t b)
{
    if (a.n == BBP_PRE) {
        return false;
    }
    if (b.n == BBP_PRE) {
        return true;
    }
    return a.n > b.n || (a.n == b.n && a.c > b.c);
}

// The start of an interval at the last verified point of group t. Before T0 there are no frames: 0.0.
static bbpPoint_t intervalStart(int t, bbpPoint_t at)
{
    if (at.n == BBP_PRE) {
        return at;
    }
    if (bbpVerified[t].n == BBP_PRE) {
        const bbpPoint_t first = { 0, 0 };
        return first;
    }
    return bbpVerified[t];
}

/* Ring */

static uint16_t ringUsed(void)
{
    return (uint16_t)(bbp.ringHead - bbp.ringTail);
}

static unsigned ringFree(void)
{
    return BBP_RING_SIZE - ringUsed();
}

static uint8_t ringGet(uint16_t pos)
{
    return bbpRing[pos & (BBP_RING_SIZE - 1)];
}

static void ringPut(uint16_t pos, uint8_t value)
{
    bbpRing[pos & (BBP_RING_SIZE - 1)] = value;
}

static uint32_t ringGetN(uint16_t pos, unsigned count)
{
    uint32_t value = 0;
    for (unsigned i = count; i > 0; i--) {
        value = (value << 8) | ringGet(pos + i - 1);
    }
    return value;
}

static void ringPutN(uint16_t pos, unsigned count, uint32_t value)
{
    for (unsigned i = 0; i < count; i++) {
        ringPut(pos + i, value >> (8 * i));
    }
}

// Record header
#define REC_TYPE    0
#define REC_SRC     1
#define REC_ARG     2
#define REC_SEQ     4
#define REC_N0      8
#define REC_C0      12
#define REC_N1      13
#define REC_C1      17
#define REC_COUNT   18
#define REC_LEN     20

static bool recBegin(uint8_t type, char src, uint16_t arg, bbpPoint_t from, bbpPoint_t at)
{
    const unsigned keep = (type == 'L') ? 0 : BBP_RING_KEEP;
    if (ringFree() < BBP_REC_HEADER + keep) {
        if (type != 'C') {
            // The seq is used: the reader finds the gap
            bbp.seq++;
            bbp.lostRec++;
        }
        // A change that does not fit stays in the lost groups: 'L', then 'y' records
        return false;
    }
    rec.seq = ++bbp.seq;

    rec.open = true;
    rec.partial = false;
    rec.type = type;
    rec.src = src;
    rec.arg = arg;
    rec.from = from;
    rec.at = at;
    rec.start = bbp.ringHead;
    rec.pos = bbp.ringHead + BBP_REC_HEADER;
    rec.count = 0;
    return true;
}

static bool recSpace(unsigned bytes)
{
    const unsigned keep = (rec.type == 'L') ? 0 : BBP_RING_KEEP;
    return BBP_RING_SIZE - (unsigned)(uint16_t)(rec.pos - bbp.ringTail) >= bytes + keep;
}

static void recPut(const uint8_t *p, unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        ringPut(rec.pos++, p[i]);
    }
}

static void recPutN(unsigned count, uint32_t value)
{
    ringPutN(rec.pos, count, value);
    rec.pos += count;
}

// Item: code (2), slot, index, length, new bytes, old bytes
static bool recItem(uint16_t code, uint8_t slot, uint8_t index, const uint8_t *newBytes, const uint8_t *oldBytes, uint8_t length)
{
    if (!recSpace(5 + 2 * length)) {
        rec.partial = true;
        return false;
    }
    recPutN(2, code);
    recPutN(1, slot);
    recPutN(1, index);
    recPutN(1, length);
    recPut(newBytes, length);
    recPut(oldBytes, length);
    rec.count++;
    return true;
}

static void recCommit(void)
{
    if (!rec.open) {
        return;
    }
    rec.open = false;

    if (rec.type == 'C' && rec.count == 0 && (rec.src != 'y' || rec.partial)) {
        // Nothing to record. The items that did not fit are in the lost groups. A 'y' record without items tells
        // that the group is back at its recorded values, but only when it completes the group.
        if (rec.seq == bbp.seq) {
            bbp.seq--;
        }
        return;
    }

    const uint16_t h = rec.start;
    ringPutN(h + REC_TYPE, 1, rec.type);
    ringPutN(h + REC_SRC, 1, rec.src);
    ringPutN(h + REC_ARG, 2, (rec.src == 'y' && rec.partial) ? rec.arg | BBP_ARG_PART : rec.arg);
    ringPutN(h + REC_SEQ, 4, rec.seq);
    ringPutN(h + REC_N0, 4, rec.from.n);
    ringPutN(h + REC_C0, 1, rec.from.c);
    ringPutN(h + REC_N1, 4, rec.at.n);
    ringPutN(h + REC_C1, 1, rec.at.c);
    ringPutN(h + REC_COUNT, 2, rec.count);
    ringPutN(h + REC_LEN, 2, (uint16_t)(rec.pos - h - BBP_REC_HEADER));
    bbp.ringHead = rec.pos;
}

// A record with a payload of 32-bit values
static void recValues(uint8_t type, char src, uint16_t arg, bbpPoint_t from, bbpPoint_t at, const uint32_t *values, int count)
{
    if (!recBegin(type, src, arg, from, at)) {
        return;
    }
    if (!recSpace(4 * count)) {
        // The seq is used: the reader finds the gap
        rec.open = false;
        bbp.lostRec++;
        return;
    }
    for (int i = 0; i < count; i++) {
        recPutN(4, values[i]);
    }
    recCommit();
}

/* Capture */

// The bytes of an item from live RAM and the shadow. The shadow is synced.
static bool itemFromLive(int t, uint16_t code, uint8_t slot, uint8_t index, unsigned offset, unsigned length)
{
    const uint8_t *live = bbpPg[t].reg->address + offset;
    uint8_t *shadow = bbpShadow(t) + offset;

    if (!recItem(code, slot, index, live, shadow, length)) {
        return false;
    }
    copyBytes(shadow, live, length);
    return true;
}

// A string whose bytes after the first NUL are NUL: its text gives every byte
static bool stringIsText(const uint8_t *p, unsigned length)
{
    unsigned i = 0;
    while (i < length && p[i]) {
        i++;
    }
    while (i < length && !p[i]) {
        i++;
    }
    return i == length;
}

/*
 * The walks: the items of the changed words of group t, bytes start..end-1, in three stages. A walk has a budget of
 * work units (about 10 cycles each on a Cortex-M4). When the budget ends, the walk keeps its position and stops; the
 * next call goes on from there. A capture outside the PID task has no limit.
 */
#define BBP_COST_BLOCK      11      // the compare of 32 B (block compare, and the bitmap of a changed block)
#define BBP_COST_ENTRY      8       // a valueTable entry of a changed instance
#define BBP_COST_ITEM       12      // an item into the ring
#define BBP_COST_ELEMENT    3       // an element
#define BBP_COST_FORMAT     100     // a changed element: its text is formatted to test its length
#define BBP_COST_RAW_BYTE   1       // a byte of a changed word in the raw walk, or of a raw item
#define BBP_NO_LIMIT        INT32_MAX

static void walkBegin(walk_t *w, int t, int32_t budget)
{
    w->stage = WALK_NAMED;
    w->k = 0;
    w->i = bbpPg[t].vtFirst;
    w->budget = budget;
}

// Values that the CLI names: one item for each changed value, or each changed element of an array
static bool walkNamed(int t, const uint32_t *map, unsigned start, unsigned end, walk_t *w)
{
    const pgRegistry_t *reg = bbpPg[t].reg;
    const pgn_t pgn = pgN(reg);
    const unsigned elementSize = pgElementSize(reg);
    const uint8_t *live = reg->address;
    uint8_t *shadow = bbpShadow(t);

    // Instance k: the profile values of profile k. Instance 0 also has the other values (offsets in the whole
    // group). A profile without a changed word costs one bitmap test.
    for (; w->k < reg->length; w->k++, w->i = bbpPg[t].vtFirst) {
        const unsigned k = w->k;
        const unsigned base = k * elementSize;
        const bool profileChanged = base < end && base + elementSize > start && mapHas(map, base, elementSize);
        if (!profileChanged && (k > 0 || (bbp.profileOnlyMask & BBP_BIT(t)))) {
            continue;
        }
        for (; w->i < bbpPg[t].vtEnd; w->i++) {
            if (w->budget <= 0) {
                return true;
            }
            w->budget -= BBP_COST_ENTRY;
            const unsigned i = w->i;
            const clivalue_t *v = &valueTable[i];
            if (v->pgn != pgn) {
                continue;
            }
            const int section = v->type & VALUE_SECTION_MASK;
            const bool perProfile = (section == PROFILE_VALUE || section == PROFILE_RATE_VALUE);
            if (perProfile ? !profileChanged : k > 0) {
                continue;
            }
            const int mode = v->type & VALUE_MODE_MASK;
            const unsigned size = bbpItemWidth(v);
            const unsigned width = (mode == MODE_ARRAY) ? v->config.array.length * size : size;
            const unsigned offset = (perProfile ? base : 0) + v->offset;
            if (offset < start || offset + width > end || !mapHas(map, offset, width)) {
                continue;
            }
            if (!bbpItemFits(v) || (mode == MODE_STRING && !(stringIsText(live + offset, width) && stringIsText(shadow + offset, width)))) {
                continue;   // raw items
            }
            const uint8_t slot = perProfile ? k : BBP_SLOT_NONE;
            if (mode == MODE_BITSET) {
                const uint32_t bit = BBP_BIT(v->config.bitpos);
                const uint32_t liveWord = readBytes(live + offset, size);
                const uint32_t shadowWord = readBytes(shadow + offset, size);
                if ((liveWord ^ shadowWord) & bit) {
                    if (!recItem(i, slot, BBP_SLOT_NONE, live + offset, shadow + offset, size)) {
                        return false;
                    }
                    writeBytes(shadow + offset, size, (shadowWord & ~bit) | (liveWord & bit));
                    w->budget -= BBP_COST_ITEM;
                }
            } else if (mode == MODE_ARRAY) {
                for (unsigned e = 0; e < v->config.array.length; e++) {
                    const unsigned at = offset + e * size;
                    if (bytesDiffer(live + at, shadow + at, size)) {
                        if (!itemFromLive(t, i, slot, e, at, size)) {
                            return false;
                        }
                        w->budget -= BBP_COST_ITEM;
                    }
                }
            } else if (bytesDiffer(live + offset, shadow + offset, width)) {
                if (!itemFromLive(t, i, slot, BBP_SLOT_NONE, offset, width)) {
                    return false;
                }
                w->budget -= BBP_COST_ITEM;
            }
        }
    }
    w->stage = WALK_ELEMENTS;
    w->i = 0;
    return true;
}

static bool elementItemFits(uint16_t code, const uint8_t *newBytes, const uint8_t *oldBytes, unsigned size)
{
    char text[BBP_ITEM_MAX];
    bbpWriter_t w = { text, text + sizeof(text), false, 0, false };

    return size <= BBP_ITEM_BYTES_MAX && bbpPutItem(&w, code, BBP_SLOT_NONE, BBP_SLOT_NONE, newBytes, oldBytes, size);
}

// Elements (servo, mixer input and rule, rx failsafe, features) and the profile indices of SYSTEM_CONFIG
static bool walkElements(int t, const uint32_t *map, unsigned start, unsigned end, walk_t *w)
{
    const pgRegistry_t *reg = bbpPg[t].reg;
    const uint8_t *live = reg->address;
    uint8_t *shadow = bbpShadow(t);
    const bbpElementKind_t *kind = bbpElementKind(pgN(reg));

    if (kind) {
        const unsigned size = pgElementSize(reg);
        const unsigned count = kind->indexed ? reg->length : 1;
        for (; w->i < count; w->i++) {
            if (w->budget <= 0) {
                return true;
            }
            w->budget -= BBP_COST_ELEMENT;
            const unsigned i = w->i;
            const unsigned offset = i * size;
            if (offset < start || offset + size > end || !mapHas(map, offset, size)) {
                continue;
            }
            bool differs = false;
            for (int f = 0; f < kind->fieldCount; f++) {
                const unsigned at = offset + kind->fields[f].offset;
                differs = differs || bytesDiffer(live + at, shadow + at, bbpFieldWidth(&kind->fields[f]));
            }
            if (!differs) {
                continue;
            }
            w->budget -= BBP_COST_FORMAT;
            const uint16_t code = BBP_CODE_ELEMENT | (unsigned)(kind - bbpElementKinds) << 8 | i;
            if (!elementItemFits(code, live + offset, shadow + offset, size)) {
                continue;   // too long: raw items
            }
            if (!recItem(code, BBP_SLOT_NONE, BBP_SLOT_NONE, live + offset, shadow + offset, size)) {
                return false;
            }
            w->budget -= BBP_COST_ITEM;
            // The fields only: other bytes of the element are raw items
            for (int f = 0; f < kind->fieldCount; f++) {
                const unsigned at = offset + kind->fields[f].offset;
                copyBytes(shadow + at, live + at, bbpFieldWidth(&kind->fields[f]));
            }
        }
    }

    if (t == bbp.systemIndex) {
        static const struct {
            uint16_t code;
            uint8_t offset;
        } indices[] = {
            { BBP_CODE_PID_INDEX,   offsetof(systemConfig_t, pidProfileIndex) },
            { BBP_CODE_RATE_INDEX,  offsetof(systemConfig_t, activeRateProfile) },
        };
        for (unsigned i = 0; i < ARRAYLEN(indices); i++) {
            const unsigned at = indices[i].offset;
            if (at >= start && at < end && live[at] != shadow[at] && !itemFromLive(t, indices[i].code, BBP_SLOT_NONE,
    BBP_SLOT_NONE, at, 1)) {
                return false;
            }
        }
    }
    w->stage = WALK_RAW;
    w->i = start;
    return true;
}

// Bytes that still differ: items of at most 16 bytes
static bool walkRaw(int t, const uint32_t *map, unsigned end, walk_t *w)
{
    const uint8_t *live = bbpPg[t].reg->address;
    const uint8_t *shadow = bbpShadow(t);

    for (unsigned i = w->i; i < end; ) {
        const uint32_t bits = map[i / 128] >> ((i / 4) % 32);
        if (!bits) {
            i = (i | 127) + 1;      // no changed word in the rest of this bitmap word
            continue;
        }
        if (!(bits & 1)) {
            i = (i | 3) + 1;
            continue;
        }
        if (w->budget <= 0) {
            w->i = i;
            return true;
        }
        w->budget -= BBP_COST_RAW_BYTE;
        if (live[i] == shadow[i]) {
            i++;
            continue;
        }
        unsigned last = i;
        for (unsigned j = i + 1; j < end && j < i + BBP_RAW_ITEM_BYTES; j++) {
            if (live[j] != shadow[j]) {
                last = j;
            }
        }
        if (!itemFromLive(t, BBP_CODE_RAW | t, i & 0xFF, i >> 8, i, last - i + 1)) {
            return false;
        }
        w->budget -= BBP_COST_ITEM + 2 * BBP_RAW_ITEM_BYTES;
        i = last + 1;
    }
    w->stage = WALK_DONE;
    return true;
}

// Walk until the walk is done or the budget ends. False: an item did not fit in the ring.
static bool walkGroup(int t, const uint32_t *map, unsigned start, unsigned end, walk_t *w)
{
    while (w->stage != WALK_DONE && w->budget > 0) {
        bool fits;
        switch (w->stage) {
        case WALK_NAMED:
            fits = walkNamed(t, map, start, end, w);
            break;
        case WALK_ELEMENTS:
            fits = walkElements(t, map, start, end, w);
            break;
        default:
            fits = walkRaw(t, map, end, w);
            break;
        }
        if (!fits) {
            return false;
        }
    }
    return true;
}

/* The state of a group after a compare */

// The whole group was equal to the shadow at 'verified', and is at 'at'
static void groupEqual(int t, bbpPoint_t at, bbpPoint_t verified)
{
    if (bbp.lostMask & BBP_BIT(t)) {
        // The lost bytes are back at their recorded values: a 'y' record without items. Its interval starts at the
        // last verified point, before this one.
        if (rec.open) {
            recCommit();
        }
        if (recBegin('C', 'y', pgN(bbpPg[t].reg), intervalStart(t, at), at)) {
            recCommit();
            bbp.lostMask &= ~BBP_BIT(t);
            bbp.lostListed &= ~BBP_BIT(t);
        }
    }
    bbpVerified[t] = verified;
    bbp.prevMask &= ~BBP_BIT(t);
}

// All differences of the group are recorded. The group was equal to the shadow at 'verified'.
static void groupSynced(int t, bbpPoint_t verified)
{
    bbpVerified[t] = verified;
    bbp.prevMask &= ~BBP_BIT(t);
    bbp.lostMask &= ~BBP_BIT(t);
    bbp.lostListed &= ~BBP_BIT(t);
}

// The ring did not take all differences of the group: the PID task writes 'L', then 'y' records
static void groupLost(int t, bbpPoint_t at)
{
    if (!(bbp.lostMask & ~bbp.lostListed)) {
        bbp.lostAt = at;
    }
    bbp.lostMask |= BBP_BIT(t);
    bbp.prevMask &= ~BBP_BIT(t);
}

static bool mapEmpty(const uint32_t *map)
{
    for (int i = 0; i < BBP_DIFF_WORDS; i++) {
        if (map[i]) {
            return false;
        }
    }
    return true;
}

static int lowestBit(uint32_t mask)
{
    return __builtin_ctz(mask);
}

/*
 * The capture of a group in the PID task, in steps with a limit of work (spec 3.7, 3.12). The compare of the group goes
 * into the bitmap in blocks of 32 B, then the walk makes the items. A step with items is a record of its own (u, v
 * or y, with an interval from the last verified point of the group), and a 'y' record that does not complete the
 * group has part=1. When the walk is done, the group was equal to the shadow at the point of the compare. Any other
 * capture of the group stops the steps: its own record has the differences that are left.
 * True when the capture of group t is complete.
 */
static bool stepGroup(int t, bbpPoint_t at, int32_t *budget)
{
    const pgRegistry_t *reg = bbpPg[t].reg;
    const unsigned size = pgSize(reg);

    if (step.t != t) {
        step.t = t;
        step.compared = at;
        clearMap(step.map);
        step.walk.stage = WALK_COMPARE;
        step.walk.i = 0;
    }
    step.walk.budget = *budget;

    if (step.walk.stage == WALK_COMPARE) {
        while (step.walk.i < size && step.walk.budget > 0) {
            const unsigned next = MIN(step.walk.i + 32u, size);
            diffRange(t, step.walk.i, next, step.map);
            step.walk.i = next;
            step.walk.budget -= BBP_COST_BLOCK;
        }
        *budget = step.walk.budget;
        if (step.walk.i < size) {
            return false;
        }
        if (mapEmpty(step.map)) {
            step.t = -1;
            groupEqual(t, at, step.compared);
            return true;
        }
        bbpChanges++;
        walkBegin(&step.walk, t, 0);
        if (*budget <= 0) {
            return false;
        }
        step.walk.budget = *budget;
    }

    const char s = (bbp.lostMask & BBP_BIT(t)) ? 'y' : (bbp.prevMask & BBP_BIT(t)) ? 'v' : 'u';
    if (!recBegin('C', s, (s == 'y') ? pgN(reg) : BBP_ARG_NONE, intervalStart(t, at), at)) {
        step.t = -1;
        groupLost(t, at);
        return true;
    }
    const bool fits = walkGroup(t, step.map, 0, size, &step.walk);
    *budget = step.walk.budget;
    if (!fits) {
        step.t = -1;
        recCommit();
        groupLost(t, at);
        return true;
    }
    if (step.walk.stage != WALK_DONE) {
        rec.partial = true;
        recCommit();
        return false;
    }
    step.t = -1;
    recCommit();
    groupSynced(t, step.compared);
    return true;
}

// One step of work on the groups in mask, the capture in progress first, while the budget and the ring allow
static void stepGroups(uint32_t mask, bbpPoint_t at, unsigned ringFreeMin)
{
    int32_t budget = BBP_STEP_UNITS;

    while (budget > 0 && ringFree() >= ringFreeMin) {
        const int t = (step.t >= 0) ? step.t : mask ? lowestBit(mask) : -1;
        if (t < 0 || !stepGroup(t, at, &budget)) {
            return;
        }
        mask &= ~BBP_BIT(t);
    }
}

/*
 * Compare the groups in 'set' with the shadow. In group slotGroup only profile 'slot' is compared, when slot >= 0.
 * Each difference becomes an item with the new and the old bytes, and the shadow is synced item by item.
 * attributed: the differences are of the operation, source src. Otherwise, and for groups with differences
 * from before (lost or between logs), each group with a difference gets its own record with an interval from
 * its last verified point.
 */
static void capture(bool attributed, char src, uint16_t arg, bbpPoint_t at, uint32_t set, int slot, int slotGroup)
{
    uint32_t map[BBP_DIFF_WORDS];
    int t;

    for (t = 0; t < BBP_TRACKED; t++) {
        if (!(set & BBP_BIT(t))) {
            continue;
        }
        if (step.t == t) {
            step.t = -1;
        }
        const pgRegistry_t *reg = bbpPg[t].reg;
        const bool whole = (slot < 0 || t != slotGroup);
        const unsigned start = whole ? 0 : slot * pgElementSize(reg);
        const unsigned end = whole ? pgSize(reg) : start + pgElementSize(reg);

        if (!groupDiffers(t, start & ~3, end) || !diffGroup(t, start & ~3, end, map)) {
            if (whole) {
                groupEqual(t, at, at);
            }
            continue;
        }
        bbpChanges++;

        const bool own = !attributed || ((bbp.lostMask | bbp.prevMask) & BBP_BIT(t));
        if (own) {
            if (rec.open) {
                recCommit();
            }
            const char s = (bbp.lostMask & BBP_BIT(t)) ? 'y' : (bbp.prevMask & BBP_BIT(t)) ? 'v' : 'u';
            if (!recBegin('C', s, (s == 'y') ? pgN(reg) : BBP_ARG_NONE, intervalStart(t, at), at)) {
                goto lost;
            }
        } else if (!rec.open && !recBegin('C', src, arg, at, at)) {
            goto lost;
        }

        walk_t w;
        walkBegin(&w, t, BBP_NO_LIMIT);
        if (!walkGroup(t, map, start, end, &w)) {
            goto lost;
        }

        if (whole) {
            groupSynced(t, at);
        }
        if (own) {
            recCommit();
        }
    }
    recCommit();
    return;

lost:
    // Items that fit stay valid. The shadow of the other differences stays unsynced: the PID task writes 'L',
    // then a 'y' record for each group. A 'y' record of a group that is lost already gets "part=1", so its 'L'
    // stays open.
    recCommit();
    groupLost(t, at);
    for (t++; t < BBP_TRACKED; t++) {
        if (set & BBP_BIT(t)) {
            const bool whole = (slot < 0 || t != slotGroup);
            const unsigned start = whole ? 0 : slot * pgElementSize(bbpPg[t].reg);
            const unsigned end = whole ? pgSize(bbpPg[t].reg) : start + pgElementSize(bbpPg[t].reg);
            if (step.t == t) {
                step.t = -1;
            }
            if (groupDiffers(t, start & ~3, end)) {
                groupLost(t, at);
            }
        }
    }
}

/* Journal state is owned by scheduler tasks. SPI receiver MSP can execute in an ISR;
 * its writes are discovered later by the bounded scan, with an interval rather than a false exact stamp. */
#ifdef UNIT_TEST
bool bbpTestInInterrupt;
#endif
static bool inInterrupt(void)
{
#ifdef UNIT_TEST
    return bbpTestInInterrupt;
#elif defined(__arm__) || defined(__thumb__)
    return __get_IPSR() != 0;
#else
    return false;
#endif
}

/* Operations */

void blackboxParamsOpBegin(char source, uint16_t arg)
{
    if (inInterrupt() || !bbp.active || bbp.opDepth++ > 0) {
        return;
    }
    bbp.opSrc = source;
    bbp.opArg = arg;
    // Differences from before the operation are not of the operation: u, v or y records with an interval
    capture(false, 'u', BBP_ARG_NONE, pointNow(), bbp.allMask, -1, -1);
}

void blackboxParamsOpEnd(void)
{
    if (inInterrupt() || !bbp.active || bbp.opDepth == 0 || --bbp.opDepth > 0) {
        return;
    }
    capture(true, bbp.opSrc, bbp.opArg, pointNow(), bbp.allMask, -1, -1);
}

bool blackboxParamsMspBegin(int16_t cmd)
{
    if (inInterrupt() || !bbp.active) {
        return false;
    }
    // Only commands that can write tracked configuration. Unknown ids and read requests are free.
    switch (cmd) {
    case MSP_SELECT_SETTING:
    case MSP_COPY_PROFILE:
    case MSP_SET_ACC_TRIM:
    case MSP_SET_DEBUG_CONFIG:
    case MSP_SET_ARMING_CONFIG:
    case MSP_SET_PID_TUNING:
    case MSP_SET_RC_TUNING:
    case MSP_SET_MOTOR_CONFIG:
    case MSP_SET_SERVO_CONFIGURATION:
    case MSP_SET_SERVO_CENTER:
    case MSP_SET_RESET_CURR_PID:
    case MSP_SET_SENSOR_ALIGNMENT:
    case MSP_SET_ADVANCED_CONFIG:
    case MSP_SET_FILTER_CONFIG:
    case MSP_SET_RPM_FILTER_V2:
    case MSP_SET_PID_PROFILE:
    case MSP_SET_RESCUE_PROFILE:
    case MSP_SET_GOVERNOR_PROFILE:
    case MSP_SET_SENSOR_CONFIG:
    case MSP_ACC_CALIBRATION:
    case MSP_SET_ESC_SENSOR_CONFIG:
    case MSP_SET_ESC_PARAMETERS:
    case MSP_EEPROM_WRITE:
    case MSP_SET_BLACKBOX_CONFIG:
    case MSP_SET_FEATURE_CONFIG:
    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
    case MSP_SET_MIXER_CONFIG:
    case MSP_SET_MIXER_INPUT:
    case MSP_SET_MIXER_RULE:
    case MSP_SET_RX_CONFIG:
    case MSP_SET_FAILSAFE_CONFIG:
    case MSP_SET_RXFAIL_CONFIG:
    case MSP_SET_RSSI_CONFIG:
    case MSP_SET_RX_MAP:
    case MSP_SET_RC_CONFIG:
    case MSP_SET_GOVERNOR_CONFIG:
    case MSP_SET_VOLTAGE_METER_CONFIG:
    case MSP_SET_CURRENT_METER_CONFIG:
    case MSP_SET_BATTERY_CONFIG:
    case MSP_SET_BATTERY_PROFILE:
    case MSP_SET_SERVO_CONFIG:
    case MSP_RESET_CONF:
    case MSP2_ESC_SENSOR_TRIAL:
    case MSP2_RX_SERIAL_TRIAL:
    case MSP2_SET_SMARTFUEL_CONFIG:
        blackboxParamsOpBegin(BBP_SRC_MSP, cmd);
        return true;
    default:
        return false;
    }
}

static uint32_t runtimeValue(int i)
{
    switch (i) {
    case BBP_RT_GOV_MODE:
        return getGovernorMode();
    case BBP_RT_FEATURES:
        return featureRuntimeMask();
    case BBP_RT_FP_PID:
        return pidParamFingerprint();
    case BBP_RT_FP_GOV:
        return governorParamFingerprint();
    default:
        return setpointParamFingerprint();
    }
}

/*
 * Loader runs while no log is open. The journal does not record them, so a change between logs ('v')
 * of a key that a loader reads has no 'A' record: the viewer cannot tell whether the loader ran after it. Keep the hash
 * of the region that the loader read at its last run with no log. At T0, a loader whose region has the same hash read
 * the values of the T0 state: an 'A' record stamped p.
 */
static struct {
    uint32_t hash;
    int8_t slot;
} bbpIdleApply[BBP_LOADER_COUNT];
static uint32_t bbpIdleApplied;

// Verify idle loader regions in bounded steps during the header, never as one T0 hash sweep.
static struct {
    uint32_t revision;
    uint32_t matched;
    uint32_t hash;
    uint16_t offset;
    uint8_t loader;
    uint8_t group;
} idleCheck;

static void idleCheckReset(void)
{
    memset(&idleCheck, 0, sizeof(idleCheck));
    idleCheck.revision = bbpChanges;
    idleCheck.hash = FNV_OFFSET_BASIS;
}

static void idleCheckStep(void)
{
    if (idleCheck.revision != bbpChanges) {
        idleCheckReset();
    }
    unsigned budget = BBP_SCAN_BYTES;
    while (budget && idleCheck.loader < BBP_LOADER_COUNT) {
        const int l = idleCheck.loader;
        if (!(bbpIdleApplied & BBP_BIT(l))) {
            idleCheck.loader++;
            budget--;
            continue;
        }
        if (idleCheck.group == BBP_TRACKED) {
            if (idleCheck.hash == bbpIdleApply[l].hash) {
                idleCheck.matched |= BBP_BIT(l);
            }
            idleCheck.loader++;
            idleCheck.group = 0;
            idleCheck.offset = 0;
            idleCheck.hash = FNV_OFFSET_BASIS;
            budget--;
            continue;
        }
        const int t = idleCheck.group;
        const pgRegistry_t *reg = bbpPg[t].reg;
        if (!(bbpLoaderRegion[l] & BBP_BIT(t)) || !reg) {
            idleCheck.group++;
            budget--;
            continue;
        }
        unsigned start = 0, length = pgSize(reg);
        const int slot = bbpIdleApply[l].slot;
        if (t == bbpLoaderSlotGroup(l) && slot >= 0) {
            start = slot * pgElementSize(reg);
            length = pgElementSize(reg);
        }
        const unsigned count = MIN(budget, length - idleCheck.offset);
        // The shadow is the state established by the header's completed capture. Any later capture restarts this check.
#ifdef UNIT_TEST
        bbpTestHashBytes += count;
#endif
        idleCheck.hash = fnv_update(idleCheck.hash, bbpShadow(t) + start + idleCheck.offset, count);
        idleCheck.offset += count;
        budget -= count;
        if (idleCheck.offset == length) {
            idleCheck.group++;
            idleCheck.offset = 0;
        }
    }
}

static bool idleCheckComplete(void)
{
    return idleCheck.revision == bbpChanges && idleCheck.loader == BBP_LOADER_COUNT;
}

static uint32_t regionHash(int loader, int slot)
{
    const int slotGroup = bbpLoaderSlotGroup(loader);
    uint32_t hash = FNV_OFFSET_BASIS;
    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = bbpPg[t].reg;
        if (!(bbpLoaderRegion[loader] & BBP_BIT(t)) || !reg) {
            continue;
        }
        unsigned start = 0, end = pgSize(reg);
        if (t == slotGroup && slot >= 0) {
            start = slot * pgElementSize(reg);
            end = start + pgElementSize(reg);
        }
#ifdef UNIT_TEST
        bbpTestHashBytes += end - start;
#endif
        hash = fnv_update(hash, reg->address + start, end - start);
    }
    return hash;
}

static int loaderFingerprint(int loader, uint32_t *fp)
{
    switch (loader) {
    case BBP_LOADER_PID:
        *fp = runtimeValue(BBP_RT_FP_PID);
        return 1;
    case BBP_LOADER_GOVERNOR:
        *fp = runtimeValue(BBP_RT_FP_GOV);
        return 1;
    case BBP_LOADER_SETPOINT:
        *fp = runtimeValue(BBP_RT_FP_SP);
        return 1;
    case BBP_LOADER_FEATURE:
        *fp = runtimeValue(BBP_RT_FEATURES);
        return 1;
    default:
        *fp = 0;
        return 0;
    }
}

static int loaderSlot(int loader, int slot)
{
    const int slotGroup = bbpLoaderSlotGroup(loader);
    if (slotGroup < 0 || !bbpPg[slotGroup].reg || slot >= bbpPg[slotGroup].reg->length) {
        return -1;
    }
    return slot;
}

void blackboxParamsApplied(bbpLoader_e loader, int slot)
{
    if (inInterrupt() || (!bbp.active && blackboxConfig()->params == BLACKBOX_PARAMS_OFF)) {
        return;
    }
    bbpChanges++;
    if (!bbp.active) {
        // No log (and blackboxParamsInit has run): keep the hash of what the loader read
        if (bbpLoaderRegion[loader]) {
            const int s = loaderSlot(loader, slot);
            bbpIdleApply[loader].slot = s;
            bbpIdleApply[loader].hash = regionHash(loader, s);
            bbpIdleApplied |= BBP_BIT(loader);
        }
        return;
    }
    const bool outside = (bbp.opDepth == 0);
    if (outside) {
        blackboxParamsOpBegin('l', loader);
    }

    // A profile that is not in the group of profiles: the whole group
    const int slotGroup = bbpLoaderSlotGroup(loader);
    if (slotGroup < 0 || slot >= bbpPg[slotGroup].reg->length) {
        slot = -1;
    }

    // The writes that the loader read come before its 'A' record
    const bbpPoint_t at = pointNow();
    capture(true, bbp.opSrc, bbp.opArg, at, bbpLoaderRegion[loader], slot, slotGroup);

    uint32_t fp;
    int count = 1;
    switch (loader) {
    case BBP_LOADER_PID:
        fp = runtimeValue(BBP_RT_FP_PID);
        break;
    case BBP_LOADER_GOVERNOR:
        fp = runtimeValue(BBP_RT_FP_GOV);
        break;
    case BBP_LOADER_SETPOINT:
        fp = runtimeValue(BBP_RT_FP_SP);
        break;
    case BBP_LOADER_FEATURE:
        fp = runtimeValue(BBP_RT_FEATURES);
        break;
    default:
        fp = 0;
        count = 0;
        break;
    }
    recValues('A', loader, (slot >= 0) ? slot : BBP_ARG_NONE, at, at, &fp, count);

    if (outside) {
        blackboxParamsOpEnd();
    }
}

void blackboxParamsMarker(bbpMarker_e marker, uint32_t a, uint32_t b)
{
    if (!inInterrupt() && bbp.active) {
        const bbpPoint_t at = pointNow();
        const uint32_t values[2] = { a, b };
        recValues('M', marker, BBP_ARG_NONE, at, at, values, 2);
    }
}

bool blackboxParamsActive(void)
{
    return bbp.active && !inInterrupt();
}

/* Events */

static void putPoint(bbpWriter_t *w, bbpPoint_t point)
{
    bbpPutUint(w, point.n);
    bbpPutChar(w, '.');
    bbpPutUint(w, point.c);
}

static void putAt(bbpWriter_t *w, bbpPoint_t from, bbpPoint_t at)
{
    if (at.n == BBP_PRE) {
        bbpPutChar(w, 'p');
        return;
    }
    if (from.n != at.n || from.c != at.c) {
        putPoint(w, from);
        bbpPutChar(w, '~');
    }
    putPoint(w, at);
}

static void putHex8(bbpWriter_t *w, uint32_t value)
{
    static const char digits[] = "0123456789abcdef";
    for (int shift = 28; shift >= 0; shift -= 4) {
        bbpPutChar(w, digits[(value >> shift) & 15]);
    }
}

// CRC-8 with polynomial 0xD5 (crc8_dvb_s2_update) a nibble at a time: the PID task formats the events
static uint8_t eventCrc(const char *p, int length)
{
    static const uint8_t table[16] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D
    };
    uint8_t crc = 0;

    for (int i = 0; i < length; i++) {
        crc ^= (uint8_t)p[i];
        crc = (uint8_t)(crc << 4) ^ table[crc >> 4];
        crc = (uint8_t)(crc << 4) ^ table[crc >> 4];
    }
    return crc;
}

// "*<crc>" and the end of the string
static int endEvent(bbpWriter_t *w, char *line)
{
    static const char digits[] = "0123456789ABCDEF";
    const int length = w->ptr - line;
    const uint8_t crc = w->crcEnabled ? w->crc : eventCrc(line, length);
    w->crcEnabled = false;

    w->end = line + BBP_EVENT_MAX;
    bbpPutChar(w, '*');
    bbpPutChar(w, digits[crc >> 4]);
    bbpPutChar(w, digits[crc & 15]);
    *w->ptr = 0;
    return w->ptr - line;
}

static bool lostBitNext(uint32_t mask, uint16_t *bit)
{
    while (*bit < BBP_TRACKED && !(mask & BBP_BIT(*bit))) {
        (*bit)++;
    }
    return *bit < BBP_TRACKED;
}

// The next event of the record at the ring tail, in bbpLine
static void formatEvent(void)
{
    const uint16_t h = bbp.ringTail;
    const uint8_t type = ringGet(h + REC_TYPE);
    const uint8_t src = ringGet(h + REC_SRC);
    const uint16_t arg = ringGetN(h + REC_ARG, 2);
    const bbpPoint_t from = { ringGetN(h + REC_N0, 4), ringGet(h + REC_C0) };
    const bbpPoint_t at = { ringGetN(h + REC_N1, 4), ringGet(h + REC_C1) };
    const uint16_t payload = h + BBP_REC_HEADER;
    const uint16_t end = payload + ringGetN(h + REC_LEN, 2);

    // Room for "*XX"
    bbpWriter_t w = { bbpLine, bbpLine + BBP_EVENT_MAX - 3, false, 0, true };

    bbpPutChar(&w, 'P');
    bbpPutChar(&w, bbp.drainStarted ? '+' : type);
    bbpPutHex(&w, ringGetN(h + REC_SEQ, 4));
    bbpPutChar(&w, ' ');
    putAt(&w, from, at);

    uint16_t pos = bbp.drainStarted ? bbp.drainPos : payload;
    bool done = true;

    switch (type) {
    case 'C': {
        if (!bbp.drainStarted) {
            bbpPutStr(&w, " s=");
            bbpPutChar(&w, src);
            if (src == 'y') {
                bbpPutStr(&w, " pgs=");
                bbpPutUint(&w, arg & ~BBP_ARG_PART);
                if (arg & BBP_ARG_PART) {
                    bbpPutStr(&w, " part=1");
                }
            } else if (arg != BBP_ARG_NONE) {
                bbpPutChar(&w, '.');
                bbpPutUint(&w, arg);
            }
            bbpPutStr(&w, " n=");
            bbpPutUint(&w, ringGetN(h + REC_COUNT, 2));
        }
        int placed = 0;
        for (unsigned items = 0; pos != end && items < BBP_RING_SIZE / 5; items++) {
            const unsigned remaining = (uint16_t)(end - pos);
            const unsigned bytes = ringGet(pos + 4);
            if (remaining < 5 || remaining > BBP_RING_SIZE || bytes > BBP_ITEM_BYTES_MAX || 5 + 2 * bytes > remaining) {
                bbp.lostRec++;
                pos = end;
                break;
            }
            uint32_t newBytes[BBP_ITEM_BYTES_MAX / 4];
            uint32_t oldBytes[BBP_ITEM_BYTES_MAX / 4];
            const uint16_t code = ringGetN(pos, 2);
            const uint8_t slot = ringGet(pos + 2);
            const uint8_t index = ringGet(pos + 3);
            const uint8_t length = MIN(ringGet(pos + 4), BBP_ITEM_BYTES_MAX);
            for (unsigned i = 0; i < length; i++) {
                ((uint8_t *)newBytes)[i] = ringGet(pos + 5 + i);
                ((uint8_t *)oldBytes)[i] = ringGet(pos + 5 + length + i);
            }
            char *mark = w.ptr;
            const uint8_t savedCrc = w.crc;
            bbpPutChar(&w, ' ');
            if (!bbpPutItem(&w, code, slot, index, (uint8_t *)newBytes, (uint8_t *)oldBytes, length)) {
                w.ptr = mark;
                w.crc = savedCrc;
                w.overflow = false;
                if (placed == 0 && bbp.drainStarted) {
                    // Cannot happen: an item fits in a continuation. Do not stall on it.
                    pos += 5 + 2 * length;
                    bbp.lostRec++;
                    continue;
                }
                break;
            }
            placed++;
            pos += 5 + 2 * length;
        }
        done = (pos == end);
        break;
    }
    case 'A':
        bbpPutChar(&w, ' ');
        bbpPutStr(&w, bbpLoaderNames[src < BBP_LOADER_COUNT ? src : 0]);
        if (arg != BBP_ARG_NONE) {
            bbpPutChar(&w, '/');
            bbpPutUint(&w, arg);
        }
        if (pos != end) {
            bbpPutStr(&w, " fp=");
            putHex8(&w, ringGetN(pos, 4));
        }
        break;
    case 'R': {
        const uint8_t mask = src;
        for (int i = 0; i < BBP_RT_COUNT; i++) {
            if (mask & BBP_BIT(i)) {
                const uint32_t value = ringGetN(pos, 4);
                pos += 4;
                bbpPutChar(&w, ' ');
                bbpPutStr(&w, runtimeNames[i]);
                if (i == BBP_RT_GOV_MODE) {
                    bbpPutUint(&w, value);
                } else if (i == BBP_RT_FEATURES) {
                    bbpPutHex(&w, value);
                } else {
                    putHex8(&w, value);
                }
            }
        }
        break;
    }
    case 'M': {
        const uint32_t a = ringGetN(pos, 4);
        const uint32_t b = ringGetN(pos + 4, 4);
        bbpPutChar(&w, ' ');
        bbpPutStr(&w, markerNames[src < BBP_MARKER_COUNT ? src : 0]);
        if (src == BBP_MARKER_EESAVE) {
            bbpPutStr(&w, " us=");
            bbpPutUint(&w, a);
        } else if (src == BBP_MARKER_ESCPARAM) {
            bbpPutStr(&w, " n=");
            bbpPutUint(&w, a);
            bbpPutStr(&w, " fnv=");
            putHex8(&w, b);
        }
        break;
    }
    case 'L': {
        // drainPos: the next group of the list
        const uint32_t mask = ringGetN(payload, 4);
        uint16_t bit = bbp.drainStarted ? bbp.drainPos : 0;
        bbpPutStr(&w, " pgs=");
        bool first = true;
        while (lostBitNext(mask, &bit)) {
            char *mark = w.ptr;
            const uint8_t savedCrc = w.crc;
            if (!first) {
                bbpPutChar(&w, ',');
            }
            bbpPutUint(&w, pgN(bbpPg[bit].reg));
            if (w.overflow) {
                w.ptr = mark;
                w.crc = savedCrc;
                w.overflow = false;
                break;
            }
            first = false;
            bit++;
        }
        done = !lostBitNext(mask, &bit);
        pos = bit;
        break;
    }
    default:
        break;
    }

    bbp.eventLength = endEvent(&w, bbpLine);
    bbp.drainNext = pos;
    bbp.drainDone = done;
    bbp.eventReady = true;
}

// At most one event in each logged iteration, and only when BBP_FRAME_RESERVE bytes stay free for frames
static void drainOneEvent(void)
{
    if (!bbp.eventReady) {
        if (bbp.ringTail == bbp.ringHead) {
            return;
        }
        formatEvent();
    }

    // 'E', the event, the length and the chars
    if (blackboxDeviceFreeSpace() < bbp.eventLength + 3 + BBP_FRAME_RESERVE) {
        return;
    }
    blackboxLogCustomString(bbpLine);

    bbp.eventReady = false;
    if (bbp.drainDone) {
        bbp.ringTail += BBP_REC_HEADER + ringGetN(bbp.ringTail + REC_LEN, 2);
        bbp.drainStarted = false;
    } else {
        bbp.drainPos = bbp.drainNext;
        bbp.drainStarted = true;
    }
}

/* PID task */

static void writeLost(void)
{
    const uint32_t mask = bbp.lostMask & ~bbp.lostListed;
    // The other records keep BBP_RING_KEEP bytes free for it
    if (ringFree() < BBP_REC_HEADER + 4) {
        return;
    }
    if (recBegin('L', 0, BBP_ARG_NONE, bbp.lostAt, bbp.lostAt)) {
        recPutN(4, mask);
        recCommit();
        bbp.lostListed |= mask;
    }
}

/*
 * Compare BBP_SCAN_BYTES of one group with the shadow. A group that is larger than the step is compared over
 * more than one logged iteration: its verified point becomes the point at which the pass started.
 * A difference is a write that no hook saw: 'u' records with an interval, in steps.
 */
static void scanStep(bbpPoint_t now)
{
    for (int i = 0; i < BBP_TRACKED && !(bbp.allMask & BBP_BIT(bbp.scanPg)); i++) {
        bbp.scanPg = (bbp.scanPg + 1) % BBP_TRACKED;
        bbp.scanOff = 0;
    }
    const int t = bbp.scanPg;
    if (!(bbp.allMask & BBP_BIT(t))) {
        return;
    }

    if (bbp.scanOff == 0) {
        bbp.scanPassStart = now;
    }
    const unsigned size = pgSize(bbpPg[t].reg);
    const unsigned end = MIN(bbp.scanOff + (unsigned)BBP_SCAN_BYTES, size);

    if (groupDiffers(t, bbp.scanOff, end)) {
        // A capture in steps: the next logged iterations go on with it
        stepGroups(BBP_BIT(t), now, 0);
        bbp.scanOff = size;
    } else if (end >= size) {
        if (pointAfter(bbp.scanPassStart, bbpVerified[t])) {
            bbpVerified[t] = bbp.scanPassStart;
        }
        bbp.scanOff = size;
    } else {
        bbp.scanOff = end;
    }

    if (bbp.scanOff >= size) {
        bbp.scanPg = (t + 1) % BBP_TRACKED;
        bbp.scanOff = 0;
    }
}

// Poll one runtime value: an 'R' record when it changed since the last poll
static void pollOneRuntime(bbpPoint_t now)
{
    const int i = bbp.rtIndex;
    const uint32_t value = runtimeValue(i);

    if (value != bbp.rtLast[i]) {
        // Only when it fits: else the next poll tries again
        if (ringFree() < BBP_REC_HEADER + 4 + BBP_RING_KEEP) {
            return;
        }
        recValues('R', BBP_BIT(i), BBP_ARG_NONE, bbp.rtPolled[i], now, &value, 1);
        bbp.rtLast[i] = value;
    }
    bbp.rtPolled[i] = now;
    bbp.rtIndex = (i + 1) % BBP_RT_COUNT;
}

void blackboxParamsAfterFrame(void)
{
    if (!bbp.active || !bbp.t0) {
        return;
    }
    const bbpPoint_t now = pointAfterFrame();

    if (bbp.lostMask & ~bbp.lostListed) {
        writeLost();
    }

    drainOneEvent();

    // One step of a capture, or of the resync of the lost groups, or of the scan
    if (step.t >= 0 || bbp.lostMask) {
        stepGroups(bbp.lostMask, now, BBP_RESYNC_FREE);
    } else {
        scanStep(now);
    }

    if ((++bbp.rtTick & 3) == 0) {
        pollOneRuntime(now);
    }
}

/* Log lifecycle */

void blackboxParamsBoot(void)
{
    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = bbpPg[t].reg;
        if (reg) {
            if (bbpTrackedPgs[t].flags & BBP_PG_BOOT) {
                memcpy(bbpBootCopy + bbpBootOffset(t), reg->address, pgSize(reg));
            }
            copyToShadow(t);
        }
    }
    bbp.shadowValid = true;
}

void blackboxParamsShadowLost(void)
{
#ifndef BLACKBOX_PARAMS_OWN_SHADOW
    // The journal stops: the CLI stops the log, and leaves by a reboot
    bbp.shadowValid = false;
    bbp.active = false;
#endif
}

static void clearJournal(void)
{
    bbp.t0 = false;
    bbp.opDepth = 0;
    bbp.seq = 0;
    bbp.prevMask = 0;
    bbp.lostMask = 0;
    bbp.lostListed = 0;
    bbp.lostRec = 0;
    bbp.ringHead = 0;
    bbp.ringTail = 0;
    bbp.eventReady = false;
    bbp.drainStarted = false;
    bbp.holding = false;
    rec.open = false;
    step.t = -1;
}

void blackboxParamsStart(void)
{
    uint8_t mode = blackboxConfig()->params;

    // The FULL snapshot (about 15 KB) at 6000 B/s would add more than 2 s without frames after arming: CHANGES
    if (mode == BLACKBOX_PARAMS_FULL && blackboxHeaderRateLimited()) {
        mode = BLACKBOX_PARAMS_CHANGES;
    }

    clearJournal();
    bbpChanges++;
    bbp.active = (mode != BLACKBOX_PARAMS_OFF);
    bbp.lineLength = 0;
    bbp.linePos = 0;

    if (!bbp.active) {
        return;
    }

    for (int t = 0; t < BBP_TRACKED; t++) {
        bbpVerified[t] = pointPre;
    }

    if (bbp.shadowValid) {
        // Compared one group in each header iteration: 'v' records
        bbp.prevMask = bbp.allMask;
    } else {
        for (int t = 0; t < BBP_TRACKED; t++) {
            if (bbpPg[t].reg) {
                copyToShadow(t);
            }
        }
        bbp.shadowValid = true;
        const uint32_t values[2] = { 0, 0 };
        recValues('M', BBP_MARKER_SHADOW_RESET, BBP_ARG_NONE, pointPre, pointPre, values, 2);
    }

    idleCheckReset();
    bbpHeaderBegin(mode != BLACKBOX_PARAMS_CHANGES);
}

// One step of the 'v' compare in each header iteration. The section ends after the compare.
void blackboxParamsHeaderTick(void)
{
    if (!bbp.active) {
        return;
    }
    if (bbp.prevMask) {
        stepGroups(bbp.prevMask, pointPre, 0);
    } else {
        idleCheckStep();
    }
}

#ifndef BBP_HEADER_LINES_PER_ITERATION
#define BBP_HEADER_LINES_PER_ITERATION 4    // bound the formatting work in one header iteration
#endif

bool blackboxParamsWriteHeader(void)
{
    if (!bbp.active) {
        return true;
    }

    // At most 64 B in each call (the rate of the other header states, and the 64 B that blackboxWriteSysinfo()
    // reserved), from up to BBP_HEADER_LINES_PER_ITERATION lines.
    const int start = MIN(blackboxHeaderBudget, BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION);
    int room = start;
    int lines = 0;
    while (room > 0) {
        if (bbp.linePos >= bbp.lineLength) {
            if (lines == BBP_HEADER_LINES_PER_ITERATION) {
                break;
            }
            const int length = bbpHeaderNextLine(bbpLine);
            if (length == 0) {
                // The formatter exhausted its work budget. Do not start another scan in this PID iteration.
                return false;
            }
            if (length < 0) {
                // The 'v' compare is in the PID task, in steps: T0 waits for it. A call that ends the section writes
                // nothing.
                return room == start && !bbp.prevMask && idleCheckComplete();
            }
            bbp.lineLength = length;
            bbp.linePos = 0;
            lines++;
        }
        const int count = MIN(bbp.lineLength - bbp.linePos, room);
        for (int i = 0; i < count; i++) {
            blackboxWrite(bbpLine[bbp.linePos + i]);
        }
        bbp.linePos += count;
        blackboxHeaderBudget -= count;
        room -= count;
    }

    return false;
}

void blackboxParamsRunning(void)
{
    if (!bbp.active) {
        return;
    }
    // The header gate has completed the bounded capture and idle-loader verification.
    // If a write occurred during CACHE_FLUSH, withhold inferred application records.

    // The loaders that ran with no log and read the values of the T0 state: 'A' records stamped p, after the 'v' records
    for (int l = 0; l < BBP_LOADER_COUNT; l++) {
        if (idleCheckComplete() && (idleCheck.matched & BBP_BIT(l))) {
            uint32_t fp;
            const int count = loaderFingerprint(l, &fp);
            recValues('A', l, (bbpIdleApply[l].slot >= 0) ? bbpIdleApply[l].slot : BBP_ARG_NONE, pointPre, pointPre, &fp, count);
        }
    }
    bbpIdleApplied = 0;

    // The base of the polls
    uint32_t values[BBP_RT_COUNT];
    const bbpPoint_t first = { 0, 0 };
    for (int i = 0; i < BBP_RT_COUNT; i++) {
        values[i] = runtimeValue(i);
        bbp.rtLast[i] = values[i];
        bbp.rtPolled[i] = first;
    }
    recValues('R', BBP_BIT(BBP_RT_COUNT) - 1, BBP_ARG_NONE, pointPre, pointPre, values, BBP_RT_COUNT);
    bbp.rtIndex = 0;
    bbp.rtTick = 0;

    bbp.scanPg = 0;
    bbp.scanOff = 0;
    bbp.t0 = true;
}

bool blackboxParamsHoldLogEnd(void)
{
    if (!bbp.active || !bbp.t0 || bbp.ringTail == bbp.ringHead) {
        return false;
    }
    // A new hold when the last request was not in the previous iteration (the log went on)
    const uint32_t iteration = blackboxGetIteration();
    if (!bbp.holding || iteration - bbp.holdIteration > 1) {
        bbp.holding = true;
        bbp.holdStart = millis();
    }
    bbp.holdIteration = iteration;
    return cmp32(millis(), bbp.holdStart) < BBP_HOLD_MS;
}

void blackboxParamsEnd(void)
{
    if (!bbp.active || !bbp.t0) {
        return;
    }

    // Records that are not completely written
    unsigned unsent = 0;
    uint16_t h = bbp.ringTail;
    unsigned remaining = (uint16_t)(bbp.ringHead - h);
    for (unsigned records = 0; remaining && records < BBP_RING_SIZE / BBP_REC_HEADER + 1; records++) {
        unsent++;
        if (remaining < BBP_REC_HEADER || remaining > BBP_RING_SIZE) {
            break;
        }
        const unsigned size = BBP_REC_HEADER + ringGetN(h + REC_LEN, 2);
        if (size > remaining) {
            break;
        }
        h += size;
        remaining -= size;
    }

    const bbpPoint_t at = { blackboxGetIteration(), 0 };
    bbpWriter_t w = { bbpLine, bbpLine + BBP_EVENT_MAX - 3, false, 0, true };
    bbpPutStr(&w, "PQ");
    bbpPutHex(&w, ++bbp.seq);
    bbpPutChar(&w, ' ');
    putAt(&w, at, at);
    bbpPutStr(&w, " unsent=");
    bbpPutUint(&w, unsent);
    bbpPutStr(&w, " lostrec=");
    bbpPutUint(&w, bbp.lostRec);
    endEvent(&w, bbpLine);
    blackboxLogCustomString(bbpLine);
}

// The log ended: SHUTTING_DOWN (also from a header state), FULL or an erase
void blackboxParamsStop(void)
{
    if (bbp.ringTail != bbp.ringHead) {
        // Records that the log did not get. Their changes are in the shadow, so the next log cannot find them in
        // its 'v' compare: it starts with 'M shadow-reset'.
        bbp.shadowValid = false;
    }
    clearJournal();
    bbp.active = false;
}

uint32_t blackboxParamsSeq(void)
{
    return bbp.seq;
}

uint32_t blackboxParamsChangeCount(void)
{
    return bbpChanges;
}


#ifdef BBP_PROFILE_ENABLED
#undef blackboxParamsHeaderTick
#undef blackboxParamsWriteHeader
#undef blackboxParamsRunning
#undef blackboxParamsAfterFrame
#undef blackboxParamsOpBegin
#undef blackboxParamsOpEnd
void blackboxParamsHeaderTick(void)
{
    const uint32_t start = DWT->CYCCNT;
    bbpInnerHeaderTick();
    bbpProfileRecord(0, start);
}
bool blackboxParamsWriteHeader(void)
{
    const uint32_t start = DWT->CYCCNT;
    const bool done = bbpInnerWriteHeader();
    bbpProfileRecord(1, start);
    return done;
}
void blackboxParamsRunning(void)
{
    const uint32_t start = DWT->CYCCNT;
    bbpInnerRunning();
    bbpProfileRecord(2, start);
}
void blackboxParamsAfterFrame(void)
{
    const uint32_t start = DWT->CYCCNT;
    bbpInnerAfterFrame();
    bbpProfileRecord(3, start);
}
void blackboxParamsOpBegin(char source, uint16_t arg)
{
    if (__get_IPSR()) {
        bbpInnerOpBegin(source, arg);
        return;
    }
    const uint32_t start = DWT->CYCCNT;
    if (bbp.active && bbp.opDepth == 0) {
        bbpProfileOpStart = start;
    }
    bbpInnerOpBegin(source, arg);
    bbpProfileRecord(4, start);
}
void blackboxParamsOpEnd(void)
{
    if (__get_IPSR()) {
        bbpInnerOpEnd();
        return;
    }
    const uint32_t start = DWT->CYCCNT;
    const bool outer = bbp.active && bbp.opDepth == 1;
    bbpInnerOpEnd();
    bbpProfileRecord(5, start);
    if (outer) {
        bbpProfileRecord(6, bbpProfileOpStart);
    }
}
#endif

#endif
