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
 * Parameter log: definitions shared by blackbox_params.c, blackbox_params_format.c and
 * blackbox_params_tables.c. Not an API for other modules.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cli/settings.h"
#include "pg/pg.h"

#define BBP_LOG_VERSION     1       // "H Param log:<n>"
#ifndef BBP_LINE_MAX
#define BBP_LINE_MAX        192     // longest header line, "H " and "\n" included
#endif
#define BBP_SCAN_BYTES      128     // parameter bytes compared per logged iteration by the journal scan
#define BBP_BIT(n)          (UINT32_C(1) << (n)) // changed-word bit 31 must use an unsigned shift
#define BBP_PG_MAX_BYTES    1024    // capacity of the changed-word bitmap

// Bytes in one "pg.<pgn>+<off>" line: 64, or less when "H boot.pg.4095+4095:" and the hex do not fit in a line
#define BBP_RAW_MAX         ((BBP_LINE_MAX - 21) / 2 < 64 ? (BBP_LINE_MAX - 21) / 2 : 64)

#define BBP_NONE            0xFFFF

/* Tracked parameter groups */

#define BBP_TRACKED         30

#define BBP_PG_BOOT         0x01    // its module caches it at boot: the header also gives the boot values

typedef struct bbpTrackedPg_s {
    pgn_t       pgn;
    uint16_t    size;               // sizeof the group
    uint8_t     flags;              // BBP_PG_*
} bbpTrackedPg_t;

extern const bbpTrackedPg_t bbpTrackedPgs[BBP_TRACKED];

// The copy of the boot-cached groups at the end of init(), at bbpBootOffset(t)
extern uint8_t bbpBootCopy[];
uint16_t bbpBootOffset(int t);

#ifdef BLACKBOX_PARAMS_OWN_SHADOW
extern uint8_t bbpOwnShadow[];
uint16_t bbpShadowOffset(int t);
#endif

// Bytes of a boot-cached group that change at run time and are not boot values
bool bbpBootIgnored(pgn_t pgn, unsigned offset);

typedef struct bbpPg_s {
    const pgRegistry_t *reg;        // NULL: the group is not in this build
    uint16_t    vtFirst;            // its valueTable entries are in vtFirst..vtEnd-1 (with entries of other groups
    uint16_t    vtEnd;              // between them if the table is not grouped)
    uint8_t     *coverage;          // one bit per byte, built once at initialization
} bbpPg_t;

extern bbpPg_t bbpPg[BBP_TRACKED];

// Bit t: the header gives every valueTable entry of group t as text (bbpValueInHeader)
extern uint32_t bbpPgAllInHeader;

// Index in bbpTrackedPgs of a registered tracked group, or -1
int bbpTrackedIndex(pgn_t pgn);

static inline uint8_t *bbpShadow(int t)
{
#ifdef BLACKBOX_PARAMS_OWN_SHADOW
    return bbpOwnShadow + bbpShadowOffset(t);
#else
    return bbpPg[t].reg->copy;
#endif
}

/* Elements: groups that the CLI sets with commands, not with "set" */

typedef enum {
    BBP_F_U8 = 0,
    BBP_F_S8,
    BBP_F_U16,
    BBP_F_S16,
    BBP_F_HEX32,
} bbpFieldType_e;

typedef struct bbpField_s {
    uint8_t     offset;
    uint8_t     type;               // bbpFieldType_e
} bbpField_t;

typedef struct bbpElementKind_s {
    const char *name;               // "el.<name>.<i>"
    pgn_t       pgn;
    uint8_t     indexed;            // false: one element, "el.<name>"
    uint8_t     fieldCount;
    const bbpField_t *fields;
} bbpElementKind_t;

#define BBP_ELEMENT_KINDS   5

extern const bbpElementKind_t bbpElementKinds[BBP_ELEMENT_KINDS];

const bbpElementKind_t *bbpElementKind(pgn_t pgn);

// Bytes of a field of an element
int bbpFieldWidth(const bbpField_t *field);

/* Text */

// A bounded text writer. Nothing is written past 'end'; 'overflow' tells that the text did not fit.
typedef struct bbpWriter_s {
    char       *ptr;
    char       *end;
    bool        overflow;
    uint8_t     crc;
    bool        crcEnabled;
} bbpWriter_t;

void bbpPutChar(bbpWriter_t *w, char c);
void bbpPutStr(bbpWriter_t *w, const char *s);
void bbpPutInt(bbpWriter_t *w, int32_t value);
void bbpPutUint(bbpWriter_t *w, uint32_t value);
void bbpPutHex(bbpWriter_t *w, uint32_t value);
void bbpPutHexBytes(bbpWriter_t *w, const uint8_t *p, int count);

// The text that CLI "get" prints (printValuePointer), with %XX escapes in strings
void bbpPutValue(bbpWriter_t *w, const clivalue_t *v, const void *p);
// The longest text that bbpPutValue() can write for this entry
int bbpValueMaxLength(const clivalue_t *v);
// Bytes of the value at v->offset that its text gives (bitsets: none)
int bbpValueWidth(const clivalue_t *v);

void bbpPutElement(bbpWriter_t *w, const bbpElementKind_t *kind, const uint8_t *p);

// A valueTable entry that the header gives as text. Other bytes are given as raw hex.
bool bbpValueInHeader(const clivalue_t *v);

// The same for an entry of group t, with the result of the boot-time check of the group
static inline bool bbpEntryInHeader(int t, const clivalue_t *v)
{
    return (bbpPgAllInHeader & (1U << t)) || bbpValueInHeader(v);
}

// Byte of group t that a set or element line gives
bool bbpByteCovered(int t, unsigned offset);
void bbpCoverageInit(void);
extern uint8_t bbpCoverage[];

/* Journal */

#define BBP_PRE             0xFFFFFFFF  // point before T0, the first RUNNING entry of the log

// A point in the log: after ticks 0..c-1 of the PID cycle that logged iteration n records
typedef struct bbpPoint_s {
    uint32_t    n;
    uint8_t     c;
} bbpPoint_t;

#define BBP_EVENT_MAX       128     // chars in one journal event (the decoder limit is 252)
#define BBP_ITEM_MAX        80      // chars in one item "KEY=NEW<OLD": it fits in a continuation event
#define BBP_ITEM_BYTES_MAX  32      // bytes of one side of an item
#define BBP_RAW_ITEM_BYTES  16      // bytes in one "pg.<pgn>+<off>" item

// Item codes
#define BBP_CODE_ELEMENT    0x8000  // | kind << 8 | element
#define BBP_CODE_PID_INDEX  0x9000  // pid_profile
#define BBP_CODE_RATE_INDEX 0x9001  // rate_profile
#define BBP_CODE_RAW        0xC000  // | tracked group; the offset is in slot (low) and index (high)
#define BBP_SLOT_NONE       0xFF

// Text of one item from its bytes. False when it did not fit.
bool bbpPutItem(bbpWriter_t *w, uint16_t code, uint8_t slot, uint8_t index, const uint8_t *newBytes,
    const uint8_t *oldBytes, uint8_t length);

// A valueTable entry whose items fit in BBP_ITEM_MAX chars and BBP_ITEM_BYTES_MAX bytes. Other bytes are raw items.
bool bbpItemFits(const clivalue_t *v);

// Bytes of an item of the entry: a bitset gives its whole variable, an array one element
int bbpItemWidth(const clivalue_t *v);

// Loader regions: bit t for each tracked group that the loader reads
extern uint32_t bbpLoaderRegion[];
// Index in bbpTrackedPgs of the group of profile slots that the loader reads (PID or rate profiles), or -1
int bbpLoaderSlotGroup(int loader);
void bbpTablesInit(void);

extern const char * const bbpLoaderNames[];

/* Header section */

void bbpHeaderBegin(bool full);
// The next header line in line[BBP_LINE_MAX]: its length, 0 when there is no line in this call, -1 when done
int bbpHeaderNextLine(char *line);
