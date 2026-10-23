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
 * Parameter log: value text and the header section.
 *
 * Header lines, in this order ("H " before each key, "\n" after each value):
 *
 *   Param log:1
 *   param_mode:FULL|CHANGES
 *   param_loop:<pid_denom>,<filter_denom>,<P interval>,<scan bytes per logged iteration>
 *   param_phase:pos=<t>,sp=<t>,pid=<t>,mix=<t>,mot=<t>,fupd=<t>,bb=<t>,flush=<t>
 *   param_pgs:<pgn>.<version>/<size>,...           (continued in param_pgs.1, param_pgs.2, ...)
 *   param_fixes:-
 *   param_rt:gov_mode=<n>,features=<hex>,pid_denom=<n>,filter_denom=<n>,looptime=<us>,debug_mode=<n>,motors=<n>,servos=<n>
 *   param_pid_profile:<0-5>
 *   param_rate_profile:<0-5>
 * FULL only:
 *   set.<name>:<value>                             master and hardware values
 *   set@p.<name>:<v0>|<v1>|...|<v5>                PID profiles; an empty field is the text of the previous profile
 *   set@p<k>.<name>:<v>                            instead of set@p. when that line would be too long
 *   set@r.<name>:...  set@r<k>.<name>:<v>          rate profiles, the same
 *   el.<kind>.<i>[-<j>]:<fields>                   elements (servo, mixin, mixrule, rxfail); i-j: the same bytes
 *   el.feature:<hex>
 *   pg.<pgn>+<offset>:<hex>                        bytes that no line above gives, at most 64 per line
 *   boot.<key>:<value>                             boot-cached groups: the value at boot, where it differs
 * All modes:
 *   param_end:<lines>,<fnv>                        lines and FNV-1 hash of the lines from "Param log"
 *
 * A line has at most BBP_LINE_MAX chars. Values have the text that CLI "get" prints. In strings, a space,
 * "%|,<>&*=:", non-printable bytes and the string "-" (an empty string is "-") are written as %XX.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "cli/settings.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/core.h"
#include "fc/rc_rates.h"

#include "flight/governor.h"
#include "flight/motors.h"
#include "flight/servos.h"

#include "pg/blackbox.h"
#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/rates.h"

#include "sensors/gyro.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_params_impl.h"

/* Text writer */

void bbpPutChar(bbpWriter_t *w, char c)
{
    if (w->ptr < w->end) {
        *w->ptr++ = c;
        if (w->crcEnabled) {
            static const uint8_t table[16] = { 0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54,
                                              0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d };
            w->crc ^= (uint8_t)c;
            w->crc = (uint8_t)(w->crc << 4) ^ table[w->crc >> 4];
            w->crc = (uint8_t)(w->crc << 4) ^ table[w->crc >> 4];
        }
    } else {
        w->overflow = true;
    }
}

void bbpPutStr(bbpWriter_t *w, const char *s)
{
    while (*s) {
        bbpPutChar(w, *s++);
    }
}

// The CLI prints numbers with tfp_format, which uses i2a() and ui2a()
void bbpPutInt(bbpWriter_t *w, int32_t value)
{
    char buf[12];
    i2a(value, buf);
    bbpPutStr(w, buf);
}

void bbpPutUint(bbpWriter_t *w, uint32_t value)
{
    char buf[12];
    ui2a(value, 10, 0, buf);
    bbpPutStr(w, buf);
}

void bbpPutHex(bbpWriter_t *w, uint32_t value)
{
    char buf[12];
    ui2a(value, 16, 0, buf);
    bbpPutStr(w, buf);
}

static const char hexDigits[] = "0123456789abcdef";

void bbpPutHexBytes(bbpWriter_t *w, const uint8_t *p, int count)
{
    for (int i = 0; i < count; i++) {
        bbpPutChar(w, hexDigits[p[i] >> 4]);
        bbpPutChar(w, hexDigits[p[i] & 15]);
    }
}

/* Values */

static int typeSize(const clivalue_t *v)
{
    switch (v->type & VALUE_TYPE_MASK) {
    case VAR_UINT16:
    case VAR_INT16:
        return 2;
    case VAR_UINT32:
        return 4;
    default:
        return 1;
    }
}

// Element i of the value, read as printValuePointer() reads it
static int32_t readValue(const clivalue_t *v, const void *p, int i)
{
    switch (v->type & VALUE_TYPE_MASK) {
    case VAR_INT8:
        return ((const int8_t *)p)[i];
    case VAR_UINT16:
        return ((const uint16_t *)p)[i];
    case VAR_INT16:
        return ((const int16_t *)p)[i];
    case VAR_UINT32:
        return ((const uint32_t *)p)[i];
    default:
        return ((const uint8_t *)p)[i];
    }
}

// The CLI prints a string as it is, and "-" for an empty string. Here a space, the chars that the log format
// uses, non-printable bytes and the string "-" are written as %XX.
static void putString(bbpWriter_t *w, const char *s, int maxLength)
{
    static const char hexUpper[] = "0123456789ABCDEF";

    if (!s[0]) {
        bbpPutChar(w, '-');
        return;
    }
    const bool dash = (s[0] == '-' && (maxLength == 1 || !s[1]));
    for (int i = 0; i < maxLength && s[i]; i++) {
        const uint8_t c = s[i];
        if (c <= ' ' || c >= 0x7F || strchr("%|,<>&*=:", c) || dash) {
            bbpPutChar(w, '%');
            bbpPutChar(w, hexUpper[c >> 4]);
            bbpPutChar(w, hexUpper[c & 15]);
        } else {
            bbpPutChar(w, c);
        }
    }
}

void bbpPutValue(bbpWriter_t *w, const clivalue_t *v, const void *p)
{
    const bool u32 = (v->type & VALUE_TYPE_MASK) == VAR_UINT32;

    switch (v->type & VALUE_MODE_MASK) {
    case MODE_ARRAY:
        for (int i = 0; i < v->config.array.length; i++) {
            if (i) {
                bbpPutChar(w, ',');
            }
            if (u32) {
                bbpPutUint(w, readValue(v, p, i));
            } else {
                bbpPutInt(w, readValue(v, p, i));
            }
        }
        break;
    case MODE_LOOKUP: {
        const int32_t value = readValue(v, p, 0);
        const lookupTableEntry_t *table = &lookupTables[v->config.lookup.tableIndex];
        if (value >= 0 && value < table->valueCount) {
            bbpPutStr(w, table->values[value]);
        } else {
            // The CLI prints no name, and reports a corrupted config
            bbpPutChar(w, '?');
            bbpPutInt(w, value);
        }
        break;
    }
    case MODE_BITSET:
        bbpPutStr(w, ((uint32_t)readValue(v, p, 0) & BBP_BIT(v->config.bitpos)) ? "ON" : "OFF");
        break;
    case MODE_STRING:
        putString(w, p, v->config.string.maxlength);
        break;
    default:
        if (u32) {
            bbpPutUint(w, readValue(v, p, 0));
        } else {
            bbpPutInt(w, readValue(v, p, 0));
        }
        break;
    }
}

static int numberMaxLength(const clivalue_t *v)
{
    switch (v->type & VALUE_TYPE_MASK) {
    case VAR_INT8:
        return 4;       // -128
    case VAR_UINT16:
        return 5;       // 65535
    case VAR_INT16:
        return 6;       // -32768
    case VAR_UINT32:
        return 11;      // 4294967295, or -2147483648 after '?'
    default:
        return 3;       // 255
    }
}

int bbpValueMaxLength(const clivalue_t *v)
{
    switch (v->type & VALUE_MODE_MASK) {
    case MODE_ARRAY:
        return v->config.array.length * (numberMaxLength(v) + 1);
    case MODE_LOOKUP: {
        const lookupTableEntry_t *table = &lookupTables[v->config.lookup.tableIndex];
        int length = 1 + numberMaxLength(v);
        for (int i = 0; i < table->valueCount; i++) {
            length = MAX(length, (int)strlen(table->values[i]));
        }
        return length;
    }
    case MODE_BITSET:
        return 3;
    case MODE_STRING:
        return MAX(1, 3 * v->config.string.maxlength);
    default:
        return numberMaxLength(v);
    }
}

int bbpValueWidth(const clivalue_t *v)
{
    switch (v->type & VALUE_MODE_MASK) {
    case MODE_ARRAY:
        return v->config.array.length * typeSize(v);
    case MODE_STRING:
        return v->config.string.maxlength;
    case MODE_BITSET:
        return 0;       // only one bit: the byte is given as raw hex too
    default:
        return typeSize(v);
    }
}

bool bbpValueInHeader(const clivalue_t *v)
{
    // The longest key is "boot.set.<name>"
    return (int)(strlen("H boot.set.:\n") + strlen(v->name)) + bbpValueMaxLength(v) <= BBP_LINE_MAX;
}

static bool valueDiffers(const clivalue_t *v, const uint8_t *a, const uint8_t *b)
{
    if ((v->type & VALUE_MODE_MASK) == MODE_BITSET) {
        return (((uint32_t)readValue(v, a, 0) ^ (uint32_t)readValue(v, b, 0)) >> v->config.bitpos) & 1;
    }
    return memcmp(a, b, bbpValueWidth(v)) != 0;
}

/* Elements */

int bbpFieldWidth(const bbpField_t *field)
{
    switch (field->type) {
    case BBP_F_U16:
    case BBP_F_S16:
        return 2;
    case BBP_F_HEX32:
        return 4;
    default:
        return 1;
    }
}

void bbpPutElement(bbpWriter_t *w, const bbpElementKind_t *kind, const uint8_t *p)
{
    for (int f = 0; f < kind->fieldCount; f++) {
        const bbpField_t *field = &kind->fields[f];
        const uint8_t *q = p + field->offset;
        if (f) {
            bbpPutChar(w, ',');
        }
        switch (field->type) {
        case BBP_F_S8:
            bbpPutInt(w, *(const int8_t *)q);
            break;
        case BBP_F_U16:
            bbpPutUint(w, *(const uint16_t *)q);
            break;
        case BBP_F_S16:
            bbpPutInt(w, *(const int16_t *)q);
            break;
        case BBP_F_HEX32:
            bbpPutHex(w, *(const uint32_t *)q);
            break;
        default:
            bbpPutUint(w, *q);
            break;
        }
    }
}

static bool elementDiffers(const bbpElementKind_t *kind, const uint8_t *a, const uint8_t *b)
{
    for (int f = 0; f < kind->fieldCount; f++) {
        const bbpField_t *field = &kind->fields[f];
        if (memcmp(a + field->offset, b + field->offset, bbpFieldWidth(field))) {
            return true;
        }
    }
    return false;
}

/* Journal items */

int bbpItemWidth(const clivalue_t *v)
{
    switch (v->type & VALUE_MODE_MASK) {
    case MODE_STRING:
        return v->config.string.maxlength;
    default:
        // A bitset: its variable. An array: one element.
        return typeSize(v);
    }
}

bool bbpItemFits(const clivalue_t *v)
{
    // "p5.<name>[255]=<new><<old>"
    const bool array = (v->type & VALUE_MODE_MASK) == MODE_ARRAY;
    const int valueLength = array ? numberMaxLength(v) : bbpValueMaxLength(v);
    const int width = array ? typeSize(v) : bbpItemWidth(v);

    return width <= BBP_ITEM_BYTES_MAX &&
        (int)strlen("p5.[255]=<") + (int)strlen(v->name) + 2 * valueLength <= BBP_ITEM_MAX;
}

static void putItemValue(bbpWriter_t *w, const clivalue_t *v, uint8_t index, const uint8_t *p)
{
    if (index != BBP_SLOT_NONE) {
        // One element of an array
        if ((v->type & VALUE_TYPE_MASK) == VAR_UINT32) {
            bbpPutUint(w, readValue(v, p, 0));
        } else {
            bbpPutInt(w, readValue(v, p, 0));
        }
    } else {
        bbpPutValue(w, v, p);
    }
}

bool bbpPutItem(bbpWriter_t *w, uint16_t code, uint8_t slot, uint8_t index, const uint8_t *newBytes,
    const uint8_t *oldBytes, uint8_t length)
{
    if (code < BBP_CODE_ELEMENT) {
        // "<name>", "p<k>.<name>", "r<k>.<name>", with "[<i>]" for one element of an array
        const clivalue_t *v = &valueTable[code];
        if (slot != BBP_SLOT_NONE) {
            bbpPutChar(w, ((v->type & VALUE_SECTION_MASK) == PROFILE_RATE_VALUE) ? 'r' : 'p');
            bbpPutUint(w, slot);
            bbpPutChar(w, '.');
        }
        bbpPutStr(w, v->name);
        if (index != BBP_SLOT_NONE) {
            bbpPutChar(w, '[');
            bbpPutUint(w, index);
            bbpPutChar(w, ']');
        }
        bbpPutChar(w, '=');
        putItemValue(w, v, index, newBytes);
        bbpPutChar(w, '<');
        putItemValue(w, v, index, oldBytes);
    } else if (code >= BBP_CODE_RAW) {
        // "pg.<pgn>+<offset>=<hex><<hex>"
        bbpPutStr(w, "pg.");
        bbpPutUint(w, pgN(bbpPg[code & 0xFF].reg));
        bbpPutChar(w, '+');
        bbpPutUint(w, slot | (index << 8));
        bbpPutChar(w, '=');
        bbpPutHexBytes(w, newBytes, length);
        bbpPutChar(w, '<');
        bbpPutHexBytes(w, oldBytes, length);
    } else if (code == BBP_CODE_PID_INDEX || code == BBP_CODE_RATE_INDEX) {
        bbpPutStr(w, (code == BBP_CODE_PID_INDEX) ? "pid_profile=" : "rate_profile=");
        bbpPutUint(w, newBytes[0]);
        bbpPutChar(w, '<');
        bbpPutUint(w, oldBytes[0]);
    } else {
        // "el.<kind>[.<i>]=<fields><<fields>"
        const bbpElementKind_t *kind = &bbpElementKinds[(code >> 8) & 0x0F];
        bbpPutStr(w, "el.");
        bbpPutStr(w, kind->name);
        if (kind->indexed) {
            bbpPutChar(w, '.');
            bbpPutUint(w, code & 0xFF);
        }
        bbpPutChar(w, '=');
        bbpPutElement(w, kind, newBytes);
        bbpPutChar(w, '<');
        bbpPutElement(w, kind, oldBytes);
    }
    return !w->overflow;
}

/* Coverage: the bytes of a group that a set or element line gives */

static void markCoverage(int t, unsigned start, unsigned width)
{
    const unsigned size = pgSize(bbpPg[t].reg);
    for (unsigned i = start; i < start + width && i < size; i++) {
        bbpPg[t].coverage[i >> 3] |= BBP_BIT(i & 7);
    }
}

void bbpCoverageInit(void)
{
    unsigned offset = 0;
    for (int t = 0; t < BBP_TRACKED; t++) {
        bbpPg_t *pg = &bbpPg[t];
        const unsigned bytes = (bbpTrackedPgs[t].size + 7) / 8;
        pg->coverage = bbpCoverage + offset;
        offset += bytes;
        memset(pg->coverage, 0, bytes);
        if (!pg->reg) {
            continue;
        }
        const pgn_t pgn = pgN(pg->reg);
        const unsigned elementSize = pgElementSize(pg->reg);
        for (unsigned i = pg->vtFirst; i < pg->vtEnd; i++) {
            const clivalue_t *v = &valueTable[i];
            if (v->pgn != pgn || !bbpEntryInHeader(t, v)) {
                continue;
            }
            const int section = v->type & VALUE_SECTION_MASK;
            const unsigned count = (section == PROFILE_VALUE || section == PROFILE_RATE_VALUE) ? pg->reg->length : 1;
            for (unsigned k = 0; k < count; k++) {
                markCoverage(t, v->offset + k * elementSize, bbpValueWidth(v));
            }
        }
        const bbpElementKind_t *kind = bbpElementKind(pgn);
        if (kind) {
            const unsigned count = kind->indexed ? pg->reg->length : 1;
            for (unsigned k = 0; k < count; k++) {
                for (int f = 0; f < kind->fieldCount; f++) {
                    markCoverage(t, k * elementSize + kind->fields[f].offset, bbpFieldWidth(&kind->fields[f]));
                }
            }
        }
    }
}

bool bbpByteCovered(int t, unsigned offset)
{
    return offset < bbpTrackedPgs[t].size && (bbpPg[t].coverage[offset >> 3] & BBP_BIT(offset & 7));
}

/* Header section */

enum {
    HDR_VERSION = 0,
    HDR_MODE,
    HDR_LOOP,
    HDR_PHASE,
    HDR_PGS,
    HDR_FIXES,
    HDR_RUNTIME,
    HDR_PID_PROFILE,
    HDR_RATE_PROFILE,
    HDR_SET,
    HDR_SET_PROFILE,
    HDR_SET_RATE,
    HDR_ELEMENTS,
    HDR_RAW,
    HDR_BOOT,
    HDR_END,
    HDR_DONE,
};

// Bound each call by entries, elements or individual raw bytes examined. Coverage is precomputed at init.
#define HDR_WORK_PER_CALL       64

static struct {
    uint8_t     stage;
    uint8_t     sub;            // param_pgs: the line; set@p/set@r: 1 + the profile of a split line; boot: the part
    uint16_t    index;          // valueTable entry, element or tracked group
    uint16_t    offset;         // byte in the group (raw), or entry or element (boot)
    uint16_t    lines;
    uint32_t    hash;
    bool        full;
} hdr;

void bbpHeaderBegin(bool full)
{
    memset(&hdr, 0, sizeof(hdr));
    hdr.hash = FNV_OFFSET_BASIS;
    hdr.full = full;
}

static void beginLine(bbpWriter_t *w, char *line, const char *key)
{
    w->ptr = line;
    w->end = line + BBP_LINE_MAX - 1;   // room for '\n'
    w->overflow = false;
    w->crcEnabled = false;
    bbpPutStr(w, "H ");
    bbpPutStr(w, key);
}

// The length of the line, or 0 when it did not fit
static int endLine(bbpWriter_t *w, char *line)
{
    if (w->overflow) {
        return 0;
    }
    *w->ptr++ = '\n';
    const int length = w->ptr - line;
    hdr.hash = fnv_update(hdr.hash, line, length);
    hdr.lines++;
    return length;
}

// "set@p.<name>:<v0>|<v1>|..." with an empty field for the same text as the previous profile
static int profileLine(char *line, const clivalue_t *v, const pgRegistry_t *reg, char letter)
{
    bbpWriter_t w;
    const char *previous = NULL;
    int previousLength = 0;

    beginLine(&w, line, "set@");
    bbpPutChar(&w, letter);
    bbpPutChar(&w, '.');
    bbpPutStr(&w, v->name);
    bbpPutChar(&w, ':');
    for (int k = 0; k < reg->length; k++) {
        if (k) {
            bbpPutChar(&w, '|');
        }
        char *text = w.ptr;
        bbpPutValue(&w, v, reg->address + k * pgElementSize(reg) + v->offset);
        const int length = w.ptr - text;
        if (previous && length == previousLength && memcmp(previous, text, length) == 0) {
            w.ptr = text;
        } else {
            previous = text;
            previousLength = length;
        }
    }
    return endLine(&w, line);
}

// The next run of bytes of group t, at or after *offset, that no set or element line gives.
// 1: the run is in start and count. 0: no more in this group. -1: the work of this call is used up.
static int nextRawRun(int t, uint16_t *offset, unsigned *start, unsigned *count, int *work)
{
    const unsigned size = pgSize(bbpPg[t].reg);

    while (*offset < size && bbpByteCovered(t, *offset)) {
        if (++*work > HDR_WORK_PER_CALL) {
            return -1;
        }
        (*offset)++;
    }
    if (*offset >= size) {
        return 0;
    }
    *start = *offset;
    *count = 0;
    while (*offset < size && *count < BBP_RAW_MAX && !bbpByteCovered(t, *offset)) {
        (*offset)++;
        (*count)++;
    }
    return 1;
}

static bool bootBytesDiffer(pgn_t pgn, const uint8_t *live, const uint8_t *boot, unsigned start, unsigned count)
{
    for (unsigned i = start; i < start + count; i++) {
        if (live[i] != boot[i] && !bbpBootIgnored(pgn, i)) {
            return true;
        }
    }
    return false;
}

static void putPgKey(bbpWriter_t *w, pgn_t pgn, unsigned offset)
{
    bbpPutUint(w, pgn);
    bbpPutChar(w, '+');
    bbpPutUint(w, offset);
    bbpPutChar(w, ':');
}

static uint8_t profileIndex(const void *profile, const void *first, size_t size)
{
    return ((const uint8_t *)profile - (const uint8_t *)first) / size;
}

static int headerNextLine(char *line)
{
    static const char * const subtaskNames[CORE_ST_COUNT] = {
        "pos", "sp", "pid", "mix", "mot", "fupd", "bb", "flush"
    };
    bbpWriter_t w;
    int work = 0;

    while (true) {
        switch (hdr.stage) {
        case HDR_VERSION:
            hdr.stage++;
            beginLine(&w, line, "Param log:");
            bbpPutUint(&w, BBP_LOG_VERSION);
            return endLine(&w, line);

        case HDR_MODE:
            hdr.stage++;
            beginLine(&w, line, "param_mode:");
            bbpPutStr(&w, hdr.full ? "FULL" : "CHANGES");
            return endLine(&w, line);

        case HDR_LOOP:
            hdr.stage++;
            beginLine(&w, line, "param_loop:");
            bbpPutUint(&w, activePidLoopDenom);
            bbpPutChar(&w, ',');
            bbpPutUint(&w, activeFilterLoopDenom);
            bbpPutChar(&w, ',');
            bbpPutUint(&w, blackboxGetPInterval());
            bbpPutChar(&w, ',');
            bbpPutUint(&w, BBP_SCAN_BYTES);
            return endLine(&w, line);

        case HDR_PHASE:
            hdr.stage++;
            beginLine(&w, line, "param_phase:");
            for (int i = 0; i < CORE_ST_COUNT; i++) {
                if (i) {
                    bbpPutChar(&w, ',');
                }
                bbpPutStr(&w, subtaskNames[i]);
                bbpPutChar(&w, '=');
                bbpPutUint(&w, coreSubtaskTick(i));
            }
            return endLine(&w, line);

        case HDR_PGS: {
            beginLine(&w, line, "param_pgs");
            if (hdr.sub) {
                bbpPutChar(&w, '.');
                bbpPutUint(&w, hdr.sub);
            }
            bbpPutChar(&w, ':');
            bool any = false;
            while (hdr.index < BBP_TRACKED) {
                const pgRegistry_t *reg = bbpPg[hdr.index].reg;
                if (reg) {
                    char *mark = w.ptr;
                    if (any) {
                        bbpPutChar(&w, ',');
                    }
                    bbpPutUint(&w, pgN(reg));
                    bbpPutChar(&w, '.');
                    bbpPutUint(&w, pgVersion(reg));
                    bbpPutChar(&w, '/');
                    bbpPutUint(&w, pgSize(reg));
                    if (w.overflow && any) {
                        // Continue in the next line
                        w.ptr = mark;
                        w.overflow = false;
                        break;
                    }
                    any = true;
                }
                hdr.index++;
            }
            if (hdr.index < BBP_TRACKED) {
                hdr.sub++;
            } else {
                hdr.stage++;
                hdr.sub = 0;
                hdr.index = 0;
            }
            return endLine(&w, line);
        }

        case HDR_FIXES:
            // The behaviour fixes that are compiled in: none
            hdr.stage++;
            beginLine(&w, line, "param_fixes:-");
            return endLine(&w, line);

        case HDR_RUNTIME:
            hdr.stage++;
            beginLine(&w, line, "param_rt:gov_mode=");
            bbpPutUint(&w, getGovernorMode());
            bbpPutStr(&w, ",features=");
            bbpPutHex(&w, featureRuntimeMask());
            bbpPutStr(&w, ",pid_denom=");
            bbpPutUint(&w, activePidLoopDenom);
            bbpPutStr(&w, ",filter_denom=");
            bbpPutUint(&w, activeFilterLoopDenom);
            bbpPutStr(&w, ",looptime=");
            bbpPutUint(&w, gyro.targetLooptime);
            bbpPutStr(&w, ",debug_mode=");
            bbpPutUint(&w, debugMode);
            bbpPutStr(&w, ",motors=");
            bbpPutUint(&w, getMotorCount());
            bbpPutStr(&w, ",servos=");
            bbpPutUint(&w, getServoCount());
            return endLine(&w, line);

        case HDR_PID_PROFILE:
            // The profiles in use: the controllers take the pointers
            hdr.stage++;
            beginLine(&w, line, "param_pid_profile:");
            bbpPutUint(&w, profileIndex(currentPidProfile, pidProfiles(0), sizeof(pidProfile_t)));
            return endLine(&w, line);

        case HDR_RATE_PROFILE:
            hdr.stage = hdr.full ? HDR_SET : HDR_END;
            beginLine(&w, line, "param_rate_profile:");
            bbpPutUint(&w, profileIndex(currentControlRateProfile, controlRateProfiles(0), sizeof(controlRateConfig_t)));
            return endLine(&w, line);

        case HDR_SET:
            while (hdr.index < valueTableEntryCount) {
                if (++work > HDR_WORK_PER_CALL) {
                    return 0;
                }
                const clivalue_t *v = &valueTable[hdr.index++];
                const int section = v->type & VALUE_SECTION_MASK;
                if (section == PROFILE_VALUE || section == PROFILE_RATE_VALUE) {
                    continue;
                }
                const int t = bbpTrackedIndex(v->pgn);
                if (t < 0 || !bbpEntryInHeader(t, v)) {
                    continue;
                }
                beginLine(&w, line, "set.");
                bbpPutStr(&w, v->name);
                bbpPutChar(&w, ':');
                bbpPutValue(&w, v, bbpPg[t].reg->address + v->offset);
                return endLine(&w, line);
            }
            hdr.stage++;
            hdr.index = 0;
            hdr.sub = 0;
            break;

        case HDR_SET_PROFILE:
        case HDR_SET_RATE: {
            const int section = (hdr.stage == HDR_SET_PROFILE) ? PROFILE_VALUE : PROFILE_RATE_VALUE;
            const char letter = (hdr.stage == HDR_SET_PROFILE) ? 'p' : 'r';
            while (hdr.index < valueTableEntryCount) {
                const clivalue_t *v = &valueTable[hdr.index];
                const int t = ((v->type & VALUE_SECTION_MASK) == section) ? bbpTrackedIndex(v->pgn) : -1;
                if (t < 0 || !bbpEntryInHeader(t, v)) {
                    hdr.index++;
                    if (++work > HDR_WORK_PER_CALL) {
                        return 0;
                    }
                    continue;
                }
                const pgRegistry_t *reg = bbpPg[t].reg;
                if (hdr.sub == 0) {
                    const int length = profileLine(line, v, reg, letter);
                    if (length > 0) {
                        hdr.index++;
                        return length;
                    }
                    hdr.sub = 1;        // too long: one line for each profile
                }
                const int k = hdr.sub - 1;
                if (k + 1 < reg->length) {
                    hdr.sub++;
                } else {
                    hdr.sub = 0;
                    hdr.index++;
                }
                beginLine(&w, line, "set@");
                bbpPutChar(&w, letter);
                bbpPutUint(&w, k);
                bbpPutChar(&w, '.');
                bbpPutStr(&w, v->name);
                bbpPutChar(&w, ':');
                bbpPutValue(&w, v, reg->address + k * pgElementSize(reg) + v->offset);
                return endLine(&w, line);
            }
            hdr.stage++;
            hdr.index = 0;
            hdr.sub = 0;
            break;
        }

        case HDR_ELEMENTS:
            while (hdr.sub < BBP_ELEMENT_KINDS) {
                const bbpElementKind_t *kind = &bbpElementKinds[hdr.sub];
                const int t = bbpTrackedIndex(kind->pgn);
                const pgRegistry_t *reg = (t >= 0) ? bbpPg[t].reg : NULL;
                const unsigned count = !reg ? 0 : kind->indexed ? reg->length : 1;
                if (hdr.index >= count) {
                    hdr.sub++;
                    hdr.index = 0;
                    continue;
                }
                // A run of elements with the same bytes has the same text: one line
                const unsigned size = pgElementSize(reg);
                const unsigned first = hdr.index;
                unsigned last = first;
                while (last + 1 < count && memcmp(reg->address + first * size, reg->address + (last + 1) * size, size) == 0) {
                    last++;
                }
                hdr.index = last + 1;
                beginLine(&w, line, "el.");
                bbpPutStr(&w, kind->name);
                if (kind->indexed) {
                    bbpPutChar(&w, '.');
                    bbpPutUint(&w, first);
                    if (last > first) {
                        bbpPutChar(&w, '-');
                        bbpPutUint(&w, last);
                    }
                }
                bbpPutChar(&w, ':');
                bbpPutElement(&w, kind, reg->address + first * size);
                return endLine(&w, line);
            }
            hdr.stage++;
            hdr.index = 0;
            hdr.sub = 0;
            hdr.offset = 0;
            break;

        case HDR_RAW:
            while (hdr.index < BBP_TRACKED) {
                const pgRegistry_t *reg = bbpPg[hdr.index].reg;
                unsigned start = 0, count = 0;
                const int found = reg ? nextRawRun(hdr.index, &hdr.offset, &start, &count, &work) : 0;
                if (found < 0) {
                    return 0;
                }
                if (found == 0) {
                    hdr.index++;
                    hdr.offset = 0;
                    continue;
                }
                beginLine(&w, line, "pg.");
                putPgKey(&w, pgN(reg), start);
                bbpPutHexBytes(&w, reg->address + start, count);
                return endLine(&w, line);
            }
            hdr.stage++;
            hdr.index = 0;
            hdr.sub = 0;
            hdr.offset = 0;
            break;

        case HDR_BOOT:
            while (hdr.index < BBP_TRACKED) {
                const int t = hdr.index;
                const pgRegistry_t *reg = bbpPg[t].reg;
                if (!reg || !(bbpTrackedPgs[t].flags & BBP_PG_BOOT)) {
                    hdr.index++;
                    continue;
                }
                const pgn_t pgn = pgN(reg);
                const uint8_t *boot = bbpBootCopy + bbpBootOffset(t);

                if (hdr.sub == 0) {
                    // Values that the CLI names
                    if (hdr.offset < bbpPg[t].vtFirst) {
                        hdr.offset = bbpPg[t].vtFirst;
                    }
                    while (hdr.offset < bbpPg[t].vtEnd) {
                        if (++work > HDR_WORK_PER_CALL) {
                            return 0;
                        }
                        const clivalue_t *v = &valueTable[hdr.offset++];
                        const int section = v->type & VALUE_SECTION_MASK;
                        if (v->pgn != pgn || section == PROFILE_VALUE || section == PROFILE_RATE_VALUE ||
                            !bbpEntryInHeader(t, v) || !valueDiffers(v, reg->address + v->offset, boot + v->offset)) {
                            continue;
                        }
                        beginLine(&w, line, "boot.set.");
                        bbpPutStr(&w, v->name);
                        bbpPutChar(&w, ':');
                        bbpPutValue(&w, v, boot + v->offset);
                        return endLine(&w, line);
                    }
                    hdr.sub = 1;
                    hdr.offset = 0;
                }

                if (hdr.sub == 1) {
                    // Elements
                    const bbpElementKind_t *kind = bbpElementKind(pgn);
                    const unsigned count = !kind ? 0 : kind->indexed ? reg->length : 1;
                    const unsigned size = pgElementSize(reg);
                    while (hdr.offset < count) {
                        if (++work > HDR_WORK_PER_CALL) {
                            return 0;
                        }
                        const unsigned i = hdr.offset++;
                        if (!elementDiffers(kind, reg->address + i * size, boot + i * size)) {
                            continue;
                        }
                        beginLine(&w, line, "boot.el.");
                        bbpPutStr(&w, kind->name);
                        if (kind->indexed) {
                            bbpPutChar(&w, '.');
                            bbpPutUint(&w, i);
                        }
                        bbpPutChar(&w, ':');
                        bbpPutElement(&w, kind, boot + i * size);
                        return endLine(&w, line);
                    }
                    hdr.sub = 2;
                    hdr.offset = 0;
                }

                // Raw bytes
                unsigned start = 0, count = 0;
                int found;
                while ((found = nextRawRun(t, &hdr.offset, &start, &count, &work)) > 0) {
                    if (bootBytesDiffer(pgn, reg->address, boot, start, count)) {
                        beginLine(&w, line, "boot.pg.");
                        putPgKey(&w, pgn, start);
                        bbpPutHexBytes(&w, boot + start, count);
                        return endLine(&w, line);
                    }
                }
                if (found < 0) {
                    return 0;
                }
                hdr.index++;
                hdr.sub = 0;
                hdr.offset = 0;
            }
            hdr.stage = HDR_END;
            break;

        case HDR_END: {
            // Not in the hash
            hdr.stage++;
            beginLine(&w, line, "param_end:");
            bbpPutUint(&w, hdr.lines);
            bbpPutChar(&w, ',');
            for (int shift = 28; shift >= 0; shift -= 4) {
                bbpPutChar(&w, hexDigits[(hdr.hash >> shift) & 15]);
            }
            *w.ptr++ = '\n';
            return w.ptr - line;
        }

        default:
            return -1;
        }
    }
}

#ifdef UNIT_TEST
unsigned bbpTestHeaderCalls, bbpTestLastHeaderYield;
#endif
int bbpHeaderNextLine(char *line)
{
#ifdef UNIT_TEST
    bbpTestHeaderCalls++;
#endif
    const int length = headerNextLine(line);
#ifdef UNIT_TEST
    if (length == 0) {
        bbpTestLastHeaderYield = bbpTestHeaderCalls;
    }
#endif
    return length;
}

#endif
