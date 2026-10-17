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
 * Parameter log (blackbox_params): value formatter and header section.
 *
 * - The formatter gives the text of CLI "get" for every valueTable entry, compared with printValuePointer() of
 *   cli.c (copied from cli.c at build time) for random bytes.
 * - The header section is decoded back into parameter group bytes. Every byte of every tracked group must
 *   come back, from exactly one kind of line.
 * - The line rules: one line per call, the header budget, the line length, param_end count and hash.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/crc.h"
    #include "common/utils.h"

    #include "cli/settings.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/blackbox.h"
    #include "pg/feature.h"
    #include "pg/governor.h"
    #include "pg/mixer.h"
    #include "pg/pid.h"
    #include "pg/rates.h"
    #include "pg/servos.h"
    #include "pg/system.h"

    #include "fc/core.h"

    #include "sensors/gyro.h"

    #include "blackbox/blackbox_io.h"
    #include "blackbox/blackbox_params.h"
    #include "blackbox/blackbox_params_impl.h"

    #include "blackbox_params_unittest_cli.h"

    #include "msp/msp_protocol.h"
    #include "msp/msp_protocol_v2_rotorflight.h"
    extern bool bbpTestInInterrupt;
    extern unsigned bbpTestHashBytes, bbpTestHeaderCalls, bbpTestLastHeaderYield;

    // Stubs of the firmware
    uint8_t activePidLoopDenom = 2;
    uint8_t activeFilterLoopDenom = 2;
    gyro_t gyro;
    pidProfile_t *currentPidProfile;
    controlRateConfig_t *currentControlRateProfile;
    int32_t blackboxHeaderBudget;

    void blackboxWrite(uint8_t value);
    uint32_t blackboxGetPInterval(void);
    uint32_t blackboxGetIteration(void);
    bool blackboxIsLogRunning(void);
    void blackboxLogCustomString(const char *ptr);
    int32_t blackboxDeviceFreeSpace(void);
    bool blackboxHeaderRateLimited(void);
    uint8_t coreSubtaskTick(coreSubtask_e subtask);
    uint8_t getPidUpdateCounter(void);
    int getGovernorMode(void);
    uint32_t featureRuntimeMask(void);
    uint32_t pidParamFingerprint(void);
    uint32_t governorParamFingerprint(void);
    uint32_t setpointParamFingerprint(void);
    uint8_t getMotorCount(void);
    uint8_t getServoCount(void);
    timeMs_t millis(void);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static std::string deviceBytes;

// The simulated firmware around the module
static struct {
    bool running;                       // blackboxIsLogRunning()
    uint32_t iteration;                 // blackboxGetIteration()
    uint8_t counter;                    // getPidUpdateCounter()
    uint8_t ticks[CORE_ST_COUNT];       // coreSubtaskTick()
    int32_t freeSpace;                  // blackboxDeviceFreeSpace()
    bool rateLimited;                   // blackboxHeaderRateLimited()
    timeMs_t millis;
    int govMode;
    uint32_t fpPid, fpGov, fpSp;
    std::vector<std::string> events;    // blackboxLogCustomString()
} sim;

// pid_process_denom 2 (core.c)
static const uint8_t ticksDenom2[CORE_ST_COUNT] = { 0, 0, 0, 1, 1, 1, 1, 0 };

void blackboxWrite(uint8_t value) { deviceBytes.push_back(value); }
uint32_t blackboxGetPInterval(void) { return 8; }
uint32_t blackboxGetIteration(void) { return sim.iteration; }
bool blackboxIsLogRunning(void) { return sim.running; }
void blackboxLogCustomString(const char *ptr) { sim.events.push_back(ptr); }
int32_t blackboxDeviceFreeSpace(void) { return sim.freeSpace; }
bool blackboxHeaderRateLimited(void) { return sim.rateLimited; }
uint8_t coreSubtaskTick(coreSubtask_e subtask) { return sim.ticks[subtask]; }
uint8_t getPidUpdateCounter(void) { return sim.counter; }
int getGovernorMode(void) { return sim.govMode; }
uint32_t featureRuntimeMask(void) { return featureConfig()->enabledFeatures; }
uint32_t pidParamFingerprint(void) { return sim.fpPid; }
uint32_t governorParamFingerprint(void) { return sim.fpGov; }
uint32_t setpointParamFingerprint(void) { return sim.fpSp; }
timeMs_t millis(void) { return sim.millis; }
uint8_t getMotorCount(void) { return 1; }
uint8_t getServoCount(void) { return 4; }

/* Helpers */

static std::mt19937 rng(20261006);

static const pgRegistry_t *trackedReg(int t) { return bbpPg[t].reg; }

static void resetSim(void)
{
    bbpTestInInterrupt = false;
    sim.running = false;
    sim.iteration = 0;
    sim.counter = 0;
    memcpy(sim.ticks, ticksDenom2, sizeof(sim.ticks));
    sim.freeSpace = 100000;
    sim.rateLimited = false;
    sim.millis = 1000;
    sim.govMode = 0;
    sim.fpPid = 0x1111;
    sim.fpGov = 0x2222;
    sim.fpSp = 0x3333;
    sim.events.clear();
}

static void setupConfig(uint8_t mode)
{
    resetSim();
    gyro.targetLooptime = 250;
    pgResetAll();
    blackboxConfigMutable()->params = mode;
    currentPidProfile = pidProfilesMutable(0);
    currentControlRateProfile = controlRateProfilesMutable(0);
    blackboxParamsInit();
    blackboxParamsBoot();
}

static void fillRandom(uint8_t *p, int count)
{
    for (int i = 0; i < count; i++) {
        p[i] = rng();
    }
}

// Strings stop at the first NUL, as the CLI and MSP write them: zero the bytes after it
static void tidyStrings(void)
{
    for (int i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *v = &valueTable[i];
        const int t = bbpTrackedIndex(v->pgn);
        if (t < 0 || (v->type & VALUE_MODE_MASK) != MODE_STRING) {
            continue;
        }
        const pgRegistry_t *reg = trackedReg(t);
        const int section = v->type & VALUE_SECTION_MASK;
        const int instances = (section == PROFILE_VALUE || section == PROFILE_RATE_VALUE) ? reg->length : 1;
        for (int k = 0; k < instances; k++) {
            char *s = (char *)reg->address + k * pgElementSize(reg) + v->offset;
            bool end = false;
            for (int c = 0; c < v->config.string.maxlength; c++) {
                end = end || !s[c];
                if (end) {
                    s[c] = 0;
                }
            }
        }
    }
}

struct HeaderRun {
    std::string text;
    int calls;
};

// The calls of blackboxWriteSysinfo(): the budget grows by budgetPerCall up to 256 (blackboxReplenishHeaderBudget),
// and a call is made only with 64 B of budget (blackboxDeviceReserveBufferSpace(64))
static HeaderRun writeHeader(int budgetPerCall = 64)
{
    HeaderRun run = { "", 0 };
    deviceBytes.clear();
    blackboxHeaderBudget = 0;
    blackboxParamsStart();
    for (int iteration = 0; iteration < 1000000; iteration++) {
        blackboxParamsHeaderTick();
        blackboxHeaderBudget = MIN(blackboxHeaderBudget + budgetPerCall, 256);
        if (blackboxHeaderBudget < 64) {
            continue;
        }
        const size_t before = deviceBytes.size();
        const int32_t budget = blackboxHeaderBudget;
        const bool done = blackboxParamsWriteHeader();
        run.calls++;
        const std::string chunk = deviceBytes.substr(before);
        EXPECT_LE((int)chunk.size(), budget);
        EXPECT_LE((int)chunk.size(), BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION);
        EXPECT_EQ(budget - (int)chunk.size(), blackboxHeaderBudget);
        // Header batching: the end of a line of the previous call, then up to BBP_HEADER_LINES_PER_ITERATION (4) lines
        EXPECT_LE((int)std::count(chunk.begin(), chunk.end(), '\n'), 5) << chunk;
        if (done) {
            EXPECT_EQ(0u, chunk.size());
            break;
        }
    }
    run.text = deviceBytes;
    return run;
}

static std::vector<std::string> splitLines(const std::string &text)
{
    std::vector<std::string> lines;
    size_t start = 0;
    while (start < text.size()) {
        const size_t end = text.find('\n', start);
        EXPECT_NE(std::string::npos, end) << "last line without newline";
        if (end == std::string::npos) {
            break;
        }
        lines.push_back(text.substr(start, end - start + 1));
        start = end + 1;
    }
    return lines;
}

static std::vector<std::string> split(const std::string &s, char sep)
{
    std::vector<std::string> parts;
    size_t start = 0;
    while (true) {
        const size_t end = s.find(sep, start);
        parts.push_back(s.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if (end == std::string::npos) {
            return parts;
        }
        start = end + 1;
    }
}

static int typeSizeOf(const clivalue_t *v)
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

static void storeNumber(uint8_t *p, int size, int64_t value)
{
    const uint32_t u = (uint32_t)value;
    memcpy(p, &u, size);    // little endian
}

static int64_t readNumber(const clivalue_t *v, const uint8_t *p)
{
    switch (v->type & VALUE_TYPE_MASK) {
    case VAR_INT8: return *(const int8_t *)p;
    case VAR_UINT16: { uint16_t x; memcpy(&x, p, 2); return x; }
    case VAR_INT16: { int16_t x; memcpy(&x, p, 2); return x; }
    case VAR_UINT32: { uint32_t x; memcpy(&x, p, 4); return x; }
    default: return *p;
    }
}

static bool parseInteger(const std::string &s, int64_t *value, int base = 10)
{
    if (s.empty()) {
        return false;
    }
    char *end;
    *value = strtoll(s.c_str(), &end, base);
    return *end == 0;
}

static std::string unescape(const std::string &s)
{
    std::string r;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '%' && i + 2 < s.size()) {
            r.push_back((char)strtol(s.substr(i + 1, 2).c_str(), NULL, 16));
            i += 2;
        } else {
            r.push_back(s[i]);
        }
    }
    return r;
}

// The bytes of a value from its header text. Bitsets set their bit only.
static bool parseValue(const clivalue_t *v, const std::string &text, uint8_t *p)
{
    const int size = typeSizeOf(v);
    int64_t value;

    switch (v->type & VALUE_MODE_MASK) {
    case MODE_ARRAY: {
        const std::vector<std::string> parts = split(text, ',');
        if ((int)parts.size() != v->config.array.length) {
            return false;
        }
        for (int i = 0; i < v->config.array.length; i++) {
            if (!parseInteger(parts[i], &value)) {
                return false;
            }
            storeNumber(p + i * size, size, value);
        }
        return true;
    }
    case MODE_LOOKUP:
        if (text[0] == '?') {
            if (!parseInteger(text.substr(1), &value)) {
                return false;
            }
        } else {
            const lookupTableEntry_t *table = &lookupTables[v->config.lookup.tableIndex];
            value = -1;
            for (int i = 0; i < table->valueCount; i++) {
                if (text == table->values[i]) {
                    value = i;
                    break;
                }
            }
            if (value < 0) {
                return false;
            }
        }
        storeNumber(p, size, value);
        return true;
    case MODE_BITSET: {
        int64_t word = readNumber(v, p);
        if (text == "ON") {
            word |= (1LL << v->config.bitpos);
        } else if (text == "OFF") {
            word &= ~(1LL << v->config.bitpos);
        } else {
            return false;
        }
        storeNumber(p, size, word);
        return true;
    }
    case MODE_STRING: {
        const std::string s = (text == "-") ? "" : unescape(text);
        if ((int)s.size() > v->config.string.maxlength) {
            return false;
        }
        memset(p, 0, v->config.string.maxlength);
        memcpy(p, s.data(), s.size());
        return true;
    }
    default:
        if (!parseInteger(text, &value)) {
            return false;
        }
        storeNumber(p, size, value);
        return true;
    }
}

static int fieldSize(const bbpField_t *field)
{
    return (field->type == BBP_F_U16 || field->type == BBP_F_S16) ? 2 : (field->type == BBP_F_HEX32) ? 4 : 1;
}

/*
 * Decoder: the parameter group bytes from the header section
 */
struct Decoded {
    std::vector<std::vector<uint8_t>> bytes;
    std::vector<std::vector<uint8_t>> known;    // 1: from a set or element line, 2: from a pg line
    std::map<std::string, std::string> keys;    // all keys and values
    std::vector<std::string> keyOrder;
    std::vector<std::string> errors;
    int lines = 0;
    uint32_t hash = FNV_OFFSET_BASIS;
    bool endSeen = false;
    unsigned endLines = 0;
    uint32_t endHash = 0;
};

static const clivalue_t *findEntry(const std::string &name, int sectionClass)
{
    for (int i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *v = &valueTable[i];
        int section = v->type & VALUE_SECTION_MASK;
        if (section == HARDWARE_VALUE) {
            section = MASTER_VALUE;
        }
        if (section == sectionClass && name == v->name) {
            return v;
        }
    }
    return NULL;
}

static void markKnown(Decoded &d, int t, unsigned offset, unsigned count, uint8_t how)
{
    for (unsigned i = offset; i < offset + count; i++) {
        if (d.known[t][i] && how == 2) {
            d.errors.push_back("raw byte also given by a set or element line: pg " + std::to_string(pgN(trackedReg(t))) + "+" + std::to_string(i));
        }
        d.known[t][i] = how;
    }
}

static void applyValue(Decoded &d, const clivalue_t *v, int instance, const std::string &text)
{
    const int t = bbpTrackedIndex(v->pgn);
    if (t < 0) {
        d.errors.push_back(std::string("untracked entry ") + v->name);
        return;
    }
    const unsigned offset = instance * pgElementSize(trackedReg(t)) + v->offset;
    if (!parseValue(v, text, &d.bytes[t][offset])) {
        d.errors.push_back(std::string("bad value ") + v->name + ":" + text);
        return;
    }
    if (bbpValueWidth(v) > 0) {
        markKnown(d, t, offset, bbpValueWidth(v), 1);
    }
}

static Decoded decodeHeader(const std::string &text)
{
    Decoded d;
    for (int t = 0; t < BBP_TRACKED; t++) {
        const int size = trackedReg(t) ? pgSize(trackedReg(t)) : 0;
        d.bytes.push_back(std::vector<uint8_t>(size, 0));
        d.known.push_back(std::vector<uint8_t>(size, 0));
    }

    for (const std::string &line : splitLines(text)) {
        if (line.size() > BBP_LINE_MAX) {
            d.errors.push_back("line too long: " + line);
        }
        if (line.compare(0, 2, "H ") != 0) {
            d.errors.push_back("not a header line: " + line);
            continue;
        }
        const size_t colon = line.find(':');
        const std::string key = line.substr(2, colon - 2);
        const std::string value = line.substr(colon + 1, line.size() - colon - 2);
        if (d.keys.count(key)) {
            d.errors.push_back("key twice: " + key);
        }
        d.keys[key] = value;
        d.keyOrder.push_back(key);

        if (key == "param_end") {
            d.endSeen = true;
            const std::vector<std::string> parts = split(value, ',');
            d.endLines = strtoul(parts[0].c_str(), NULL, 10);
            d.endHash = strtoul(parts[1].c_str(), NULL, 16);
            if (parts[1].size() != 8) {
                d.errors.push_back("hash width");
            }
            continue;
        }
        if (d.endSeen) {
            d.errors.push_back("line after param_end: " + line);
        }
        d.lines++;
        d.hash = fnv_update(d.hash, line.data(), line.size());

        if (key.compare(0, 4, "set.") == 0) {
            const clivalue_t *v = findEntry(key.substr(4), MASTER_VALUE);
            if (!v) {
                d.errors.push_back("unknown " + key);
                continue;
            }
            applyValue(d, v, 0, value);
        } else if (key.compare(0, 4, "set@") == 0) {
            const int section = (key[4] == 'p') ? PROFILE_VALUE : PROFILE_RATE_VALUE;
            const size_t dot = key.find('.');
            const clivalue_t *v = findEntry(key.substr(dot + 1), section);
            if (!v) {
                d.errors.push_back("unknown " + key);
                continue;
            }
            if (dot == 5) {
                const std::vector<std::string> fields = split(value, '|');
                const int t = bbpTrackedIndex(v->pgn);
                if (t < 0 || (int)fields.size() != trackedReg(t)->length) {
                    d.errors.push_back("profile count " + key);
                    continue;
                }
                std::string previous;
                for (size_t k = 0; k < fields.size(); k++) {
                    if (k == 0 && fields[k].empty()) {
                        d.errors.push_back("empty first profile " + key);
                    }
                    if (!fields[k].empty()) {
                        previous = fields[k];
                    }
                    applyValue(d, v, k, previous);
                }
            } else {
                applyValue(d, v, atoi(key.substr(5, dot - 5).c_str()), value);
            }
        } else if (key.compare(0, 3, "el.") == 0) {
            const std::vector<std::string> parts = split(key, '.');
            const bbpElementKind_t *kind = NULL;
            for (int k = 0; k < BBP_ELEMENT_KINDS; k++) {
                if (parts[1] == bbpElementKinds[k].name) {
                    kind = &bbpElementKinds[k];
                }
            }
            const int t = kind ? bbpTrackedIndex(kind->pgn) : -1;
            if (t < 0) {
                d.errors.push_back("unknown " + key);
                continue;
            }
            unsigned first = 0, last = 0;
            if (kind->indexed) {
                const std::vector<std::string> range = split(parts[2], '-');
                first = last = strtoul(range[0].c_str(), NULL, 10);
                if (range.size() > 1) {
                    last = strtoul(range[1].c_str(), NULL, 10);
                    if (last <= first) {
                        d.errors.push_back("bad range " + key);
                    }
                }
            }
            const std::vector<std::string> fields = split(value, ',');
            if ((int)fields.size() != kind->fieldCount) {
                d.errors.push_back("field count " + key);
                continue;
            }
            const unsigned size = pgElementSize(trackedReg(t));
            for (unsigned i = first; i <= last; i++) {
                for (int f = 0; f < kind->fieldCount; f++) {
                    int64_t number;
                    if (!parseInteger(fields[f], &number, kind->fields[f].type == BBP_F_HEX32 ? 16 : 10)) {
                        d.errors.push_back("bad field " + key);
                    }
                    storeNumber(&d.bytes[t][i * size + kind->fields[f].offset], fieldSize(&kind->fields[f]), number);
                    markKnown(d, t, i * size + kind->fields[f].offset, fieldSize(&kind->fields[f]), 1);
                }
            }
        } else if (key.compare(0, 3, "pg.") == 0) {
            const size_t plus = key.find('+');
            const int t = bbpTrackedIndex(atoi(key.substr(3, plus - 3).c_str()));
            const unsigned offset = atoi(key.substr(plus + 1).c_str());
            if (t < 0 || value.size() % 2 || value.size() / 2 > BBP_RAW_MAX || offset + value.size() / 2 > d.bytes[t].size()) {
                d.errors.push_back("bad raw " + line);
                continue;
            }
            for (size_t i = 0; i < value.size() / 2; i++) {
                d.bytes[t][offset + i] = strtoul(value.substr(2 * i, 2).c_str(), NULL, 16);
            }
            markKnown(d, t, offset, value.size() / 2, 2);
        }
    }
    return d;
}

// The decoded bytes equal the parameter groups, and each byte is given once
static void expectExact(const Decoded &d)
{
    for (const std::string &e : d.errors) {
        ADD_FAILURE() << e;
    }
    EXPECT_TRUE(d.endSeen);
    EXPECT_EQ((unsigned)d.lines, d.endLines);
    EXPECT_EQ(d.hash, d.endHash);

    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = trackedReg(t);
        if (!reg) {
            continue;
        }
        int unknown = 0, wrong = 0;
        for (unsigned i = 0; i < pgSize(reg); i++) {
            unknown += !d.known[t][i];
            if (d.known[t][i] && d.bytes[t][i] != reg->address[i]) {
                wrong++;
                ADD_FAILURE() << "pg " << pgN(reg) << "+" << i << ": " << (int)d.bytes[t][i] << " instead of " << (int)reg->address[i];
            }
        }
        EXPECT_EQ(0, unknown) << "pg " << pgN(reg) << ": bytes not in the header";
        EXPECT_EQ(0, wrong) << "pg " << pgN(reg) << ": bytes that differ";
    }
}

static int countKeys(const Decoded &d, const char *prefix)
{
    int n = 0;
    for (const std::string &key : d.keyOrder) {
        n += key.compare(0, strlen(prefix), prefix) == 0;
    }
    return n;
}

/* Tests */

TEST(BlackboxParamsTest, FormatterMatchesCli)
{
    alignas(4) uint8_t buf[256];        // larger than any value (telemetry_sensors: 40 x 2 B)
    char cli[512];
    char text[512];
    int compared = 0;

    for (int i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *v = &valueTable[i];
        for (int n = 0; n < 400; n++) {
            if (n == 0) {
                memset(buf, 0, sizeof(buf));
            } else if (n == 1) {
                memset(buf, 0xFF, sizeof(buf));
            } else if (n == 2) {
                memset(buf, 0x80, sizeof(buf));
            } else {
                fillRandom(buf, sizeof(buf));
            }
            if ((v->type & VALUE_MODE_MASK) == MODE_STRING) {
                // The CLI prints up to the NUL
                const int max = v->config.string.maxlength;
                buf[rng() % (max + 1)] = 0;
                buf[max] = 0;
            }

            bbpWriter_t w = { text, text + sizeof(text) - 1, false };
            bbpPutValue(&w, v, buf);
            *w.ptr = 0;
            ASSERT_FALSE(w.overflow);
            EXPECT_LE((int)strlen(text), bbpValueMaxLength(v)) << v->name << " " << text;

            const bool lookup = (v->type & VALUE_MODE_MASK) == MODE_LOOKUP;
            const int64_t value = readNumber(v, buf);
            if (lookup && value < 0) {
                // The CLI reads before the lookup table: compare with the rule only
                EXPECT_EQ("?" + std::to_string(value), std::string(text)) << v->name;
                continue;
            }
            const bool valid = cliTestPrintValue(cli, v, buf);
            if (lookup && !valid) {
                // Out of range: the CLI prints no name
                EXPECT_EQ("?" + std::to_string(value), std::string(text)) << v->name;
            } else if ((v->type & VALUE_MODE_MASK) == MODE_STRING) {
                EXPECT_EQ(std::string(cli), unescape(text)) << v->name;
                for (const char *c = text; *c; c++) {
                    EXPECT_TRUE(*c > ' ' && *c < 0x7F && !strchr("|,<>&*=:", *c)) << v->name << " " << text;
                }
            } else {
                EXPECT_STREQ(cli, text) << v->name;
            }
            compared++;
        }
    }
    printf("formatter: %d valueTable entries, %d comparisons with the CLI\n", valueTableEntryCount, compared);
}

TEST(BlackboxParamsTest, OffWritesNothing)
{
    setupConfig(BLACKBOX_PARAMS_OFF);
    deviceBytes.clear();
    blackboxHeaderBudget = 256;
    blackboxParamsStart();
    EXPECT_TRUE(blackboxParamsWriteHeader());
    EXPECT_EQ(0u, deviceBytes.size());
    EXPECT_EQ(256, blackboxHeaderBudget);
    EXPECT_EQ(0u, blackboxParamsSeq());
}

TEST(BlackboxParamsTest, ChangesHasMetaLinesOnly)
{
    setupConfig(BLACKBOX_PARAMS_CHANGES);
    const HeaderRun run = writeHeader();
    const Decoded d = decodeHeader(run.text);
    for (const std::string &e : d.errors) {
        ADD_FAILURE() << e;
    }
    const std::vector<std::string> expected = {
        "Param log", "param_mode", "param_loop", "param_phase", "param_pgs", "param_fixes",
        "param_rt", "param_pid_profile", "param_rate_profile", "param_end",
    };
    // param_pgs.1, param_pgs.2, ... continue param_pgs
    std::vector<std::string> keys;
    int continued = 0;
    for (const std::string &key : d.keyOrder) {
        if (key == "param_pgs." + std::to_string(continued + 1)) {
            continued++;
        } else {
            keys.push_back(key);
        }
    }
    EXPECT_EQ(expected, keys);
    EXPECT_GE(continued, 1);
    EXPECT_EQ("1", d.keys.at("Param log"));
    EXPECT_EQ("CHANGES", d.keys.at("param_mode"));
    EXPECT_EQ("2,2,8,128", d.keys.at("param_loop"));
    EXPECT_EQ("pos=0,sp=0,pid=0,mix=1,mot=1,fupd=1,bb=1,flush=0", d.keys.at("param_phase"));
    EXPECT_EQ("-", d.keys.at("param_fixes"));
    EXPECT_EQ("0", d.keys.at("param_pid_profile"));
    EXPECT_TRUE(d.endSeen);
    EXPECT_EQ((unsigned)d.lines, d.endLines);
    EXPECT_EQ(d.hash, d.endHash);
    printf("CHANGES header: %zu bytes, %d lines, %d calls\n", run.text.size(), d.lines + 1, run.calls);
}

// A serial logger below 1 Mbaud (6000 B/s): FULL gives the CHANGES section
TEST(BlackboxParamsTest, SlowSerialGivesChanges)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    sim.rateLimited = true;
    const std::string slow = writeHeader().text;
    setupConfig(BLACKBOX_PARAMS_CHANGES);
    const std::string changes = writeHeader().text;
    EXPECT_EQ(changes, slow);
    EXPECT_NE(std::string::npos, slow.find("H param_mode:CHANGES\n"));
}

TEST(BlackboxParamsTest, ParamPgsListsTheTrackedGroups)
{
    setupConfig(BLACKBOX_PARAMS_CHANGES);
    const Decoded d = decodeHeader(writeHeader().text);
    std::string expected, actual = d.keys.at("param_pgs");
    for (int k = 1; d.keys.count("param_pgs." + std::to_string(k)); k++) {
        actual += "," + d.keys.at("param_pgs." + std::to_string(k));
    }
    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = trackedReg(t);
        if (reg) {
            expected += (expected.empty() ? "" : ",") + std::to_string(pgN(reg)) + "." + std::to_string(pgVersion(reg)) + "/" + std::to_string(pgSize(reg));
        }
    }
    EXPECT_EQ(expected, actual);
}

TEST(BlackboxParamsTest, DefaultConfigFullIsExact)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    const HeaderRun run = writeHeader();
    const Decoded d = decodeHeader(run.text);
    expectExact(d);

    // No boot lines: nothing changed since boot
    EXPECT_EQ(0, countKeys(d, "boot."));
    // The profiles of the defaults are the same: all fields after the first are empty
    for (const std::string &key : d.keyOrder) {
        if (key.compare(0, 6, "set@p.") == 0 || key.compare(0, 6, "set@r.") == 0) {
            const std::vector<std::string> fields = split(d.keys.at(key), '|');
            EXPECT_EQ(std::vector<std::string>(fields.size() - 1, ""), std::vector<std::string>(fields.begin() + 1, fields.end())) << key;
        }
    }

    int tracked = 0, bytes = 0;
    for (int t = 0; t < BBP_TRACKED; t++) {
        if (trackedReg(t)) {
            tracked++;
            bytes += pgSize(trackedReg(t));
        } else {
            printf("  group %d is not in this build\n", bbpTrackedPgs[t].pgn);
        }
    }
    if (getenv("BBP_ENTRIES_DUMP")) {
        FILE *f = fopen(getenv("BBP_ENTRIES_DUMP"), "w");
        for (int i = 0; i < valueTableEntryCount; i++) {
            if (bbpTrackedIndex(valueTable[i].pgn) >= 0) {
                fprintf(f, "%s\n", valueTable[i].name);
            }
        }
        fclose(f);
    }
    printf("FULL header, default configuration: %zu bytes, %d lines, %d calls at 64 B per call\n",
        run.text.size(), d.lines + 1, run.calls);
    printf("  %d tracked groups (%d bytes): %d set, %d set@p, %d set@r, %d el, %d pg lines\n", tracked, bytes,
        countKeys(d, "set."), countKeys(d, "set@p"), countKeys(d, "set@r"), countKeys(d, "el."), countKeys(d, "pg."));
    if (getenv("BBP_HEADER_DUMP")) {
        FILE *f = fopen(getenv("BBP_HEADER_DUMP"), "w");
        fputs(run.text.c_str(), f);
        fclose(f);
    }
}

// The time of the slowest call on this computer: the shortest of 50 runs for each call
TEST(BlackboxParamsTest, CallTime)
{
    std::vector<double> best;
    for (int run = 0; run < 50; run++) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        blackboxParamsStart();
        deviceBytes.clear();
        for (int call = 0; ; call++) {
            blackboxParamsHeaderTick();
            blackboxHeaderBudget = 256;
            const auto start = std::chrono::steady_clock::now();
            const bool done = blackboxParamsWriteHeader();
            const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
            if ((int)best.size() <= call) {
                best.push_back(us);
            }
            best[call] = std::min(best[call], us);
            if (done) {
                break;
            }
        }
    }
    double worst = 0, total = 0;
    size_t worstCall = 0;
    for (size_t call = 0; call < best.size(); call++) {
        if (best[call] > worst) {
            worst = best[call];
            worstCall = call;
        }
        total += best[call];
    }
    std::vector<double> sorted = best;
    std::sort(sorted.begin(), sorted.end());
    printf("call time on this computer: slowest %.2f us (call %zu), 99th percentile %.2f us, mean %.3f us, %zu calls\n",
        worst, worstCall, sorted[sorted.size() * 99 / 100], total / best.size(), best.size());
    if (getenv("BBP_CALL_TIMES")) {
        for (size_t call = 0; call < best.size(); call++) {
            printf("%zu %.3f\n", call, best[call]);
        }
    }
}

TEST(BlackboxParamsTest, ElementRunsAndProfileFields)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    // Servos 0-3 the same, 4 different, 5 and up the same as 0-3
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoParamsMutable(i)->mid = 1500;
    }
    servoParamsMutable(4)->mid = 1520;
    pidProfilesMutable(2)->pid[0].P = 77;
    const Decoded d = decodeHeader(writeHeader().text);
    expectExact(d);
    EXPECT_EQ(1u, d.keys.count("el.servo.0-3"));
    EXPECT_EQ(1u, d.keys.count("el.servo.4"));
    EXPECT_EQ(1u, d.keys.count("el.servo.5-" + std::to_string(MAX_SUPPORTED_SERVOS - 1)));
    EXPECT_EQ(0u, d.keys.at("el.servo.4").find("1520,"));
    const std::string p = std::to_string(pidProfiles(0)->pid[0].P);
    std::string expected = p + "||77";
    for (int k = 3; k < PID_PROFILE_COUNT; k++) {
        expected += (k == 3) ? "|" + p : "|";
    }
    EXPECT_EQ(expected, d.keys.at("set@p.roll_p_gain"));
}

TEST(BlackboxParamsTest, RandomConfigFullIsExact)
{
    const int iterations = getenv("BBP_RANDOM_ITERATIONS") ? atoi(getenv("BBP_RANDOM_ITERATIONS")) : 40;
    for (int n = 0; n < iterations; n++) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        for (int t = 0; t < BBP_TRACKED; t++) {
            if (trackedReg(t)) {
                fillRandom(trackedReg(t)->address, pgSize(trackedReg(t)));
            }
        }
        tidyStrings();
        blackboxConfigMutable()->params = BLACKBOX_PARAMS_FULL;
        if (n % 2) {
            // Profiles that repeat
            for (int k = 1; k < PID_PROFILE_COUNT; k += 2) {
                *pidProfilesMutable(k) = *pidProfiles(k - 1);
            }
        }
        currentPidProfile = pidProfilesMutable(n % PID_PROFILE_COUNT);
        const HeaderRun run = writeHeader(n % 3 ? 64 : 40);
        const Decoded d = decodeHeader(run.text);
        expectExact(d);
        EXPECT_EQ(std::to_string(n % PID_PROFILE_COUNT), d.keys.at("param_pid_profile"));
        // The values are the boot values: boot lines only where a group differs from the boot copy
        EXPECT_GT(countKeys(d, "boot."), 0);
        if (HasFailure()) {
            printf("%s", run.text.c_str());
            break;
        }
    }
}

TEST(BlackboxParamsTest, BootLines)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    const uint8_t mode = governorConfig()->gov_mode;
    const uint16_t mid = servoParams(1)->mid;
    const char board0 = systemConfig()->boardIdentifier[0];

    // Changes after boot in boot-cached groups
    governorConfigMutable()->gov_mode = mode + 1;
    servoParamsMutable(1)->mid = mid + 10;
    systemConfigMutable()->boardIdentifier[0] = board0 + 1;
    // Not a boot value: the profile index
    systemConfigMutable()->pidProfileIndex = 3;
    // Not a boot-cached group
    pidProfilesMutable(0)->pid[0].P += 1;

    const Decoded d = decodeHeader(writeHeader().text);
    expectExact(d);

    const clivalue_t *v = findEntry("gov_mode", MASTER_VALUE);
    ASSERT_NE(nullptr, v);
    char text[64];
    bbpWriter_t w = { text, text + sizeof(text) - 1, false };
    uint8_t boot = mode;
    bbpPutValue(&w, v, &boot);
    *w.ptr = 0;
    EXPECT_EQ(text, d.keys.at("boot.set.gov_mode"));

    EXPECT_EQ(0u, d.keys.at("boot.el.servo.1").find(std::to_string(mid) + ","));
    EXPECT_EQ(1u, d.keys.count("boot.pg.18+0"));
    EXPECT_EQ(3, countKeys(d, "boot."));

    // A profile switch alone gives no boot line
    setupConfig(BLACKBOX_PARAMS_FULL);
    systemConfigMutable()->pidProfileIndex = 2;
    systemConfigMutable()->activeRateProfile = 1;
    const Decoded d2 = decodeHeader(writeHeader().text);
    expectExact(d2);
    EXPECT_EQ(0, countKeys(d2, "boot."));
}

TEST(BlackboxParamsTest, LongLinesFallBack)
{
    // set@p lines that are too long become one line per profile
    setupConfig(BLACKBOX_PARAMS_FULL);
    for (int k = 0; k < PID_PROFILE_COUNT; k++) {
        // 22 chars of text for each profile
        memset(pidProfilesMutable(k)->profileName, '%', MAX_PROFILE_NAME_LENGTH);
        pidProfilesMutable(k)->profileName[MAX_PROFILE_NAME_LENGTH - 1] = 'a' + k;
    }
    const Decoded d = decodeHeader(writeHeader().text);
    expectExact(d);
#if BBP_LINE_MAX < 192
    EXPECT_EQ(1u, d.keys.count("set@p0.profile_name"));
    EXPECT_EQ(0u, d.keys.count("set@p.profile_name"));
#else
    EXPECT_EQ(1u, d.keys.count("set@p.profile_name"));
#endif
}

/*
 * Journal: records in CUSTOM_STRING events
 */

struct JItem {
    std::string key, value, old;
};

struct JRecord {
    char type = 0;
    uint32_t seq = 0;
    std::string at;
    bool pre = false;
    bool interval = false;
    uint32_t n0 = 0, c0 = 0, n1 = 0, c1 = 0;
    std::map<std::string, std::string> fields;  // name=value tokens without '<'
    std::vector<std::string> words;             // tokens without '='
    std::vector<JItem> items;
    int events = 0;
};

struct Journal {
    std::vector<JRecord> records;
    std::vector<std::string> errors;
    int events = 0;
    size_t longest = 0;
};

static bool parsePoint(const std::string &s, uint32_t *n, uint32_t *c)
{
    const size_t dot = s.find('.');
    if (dot == std::string::npos) {
        return false;
    }
    int64_t a, b;
    if (!parseInteger(s.substr(0, dot), &a) || !parseInteger(s.substr(dot + 1), &b)) {
        return false;
    }
    *n = a;
    *c = b;
    return true;
}

static Journal parseJournal(const std::vector<std::string> &events)
{
    Journal j;
    uint32_t expectedSeq = 1;

    auto finish = [&](void) {
        if (j.records.empty()) {
            return;
        }
        const JRecord &r = j.records.back();
        if (r.type == 'C' && (!r.fields.count("n") || atoi(r.fields.at("n").c_str()) != (int)r.items.size())) {
            j.errors.push_back("item count of record " + std::to_string(r.seq));
        }
    };

    for (const std::string &e : events) {
        if (e.size() < 2 || e[0] != 'P' || !strchr("CARMLQ+", e[1])) {
            continue;   // not a journal event
        }
        j.events++;
        j.longest = std::max(j.longest, e.size());
        if (e.size() > BBP_EVENT_MAX) {
            j.errors.push_back("event longer than 128: " + e);
        }
        const size_t star = e.rfind('*');
        if (star == std::string::npos || star + 3 != e.size()) {
            j.errors.push_back("no CRC: " + e);
            continue;
        }
        char crc[3];
        snprintf(crc, sizeof(crc), "%02X", crc8_dvb_s2_update(0, e.data(), star));
        if (e.substr(star + 1) != crc) {
            j.errors.push_back("bad CRC: " + e);
        }
        const std::vector<std::string> tokens = split(e.substr(2, star - 2), ' ');
        if (tokens.size() < 2) {
            j.errors.push_back("short event: " + e);
            continue;
        }
        for (char ch : tokens[0]) {
            if (!isxdigit(ch) || isupper(ch)) {
                j.errors.push_back("seq not lowercase hex: " + e);
            }
        }
        const uint32_t seq = strtoul(tokens[0].c_str(), NULL, 16);

        JRecord *r;
        if (e[1] == '+') {
            if (j.records.empty() || j.records.back().seq != seq || j.records.back().at != tokens[1]) {
                j.errors.push_back("continuation without its record: " + e);
                continue;
            }
            r = &j.records.back();
        } else {
            finish();
            if (seq != expectedSeq) {
                j.errors.push_back("seq " + std::to_string(seq) + " instead of " + std::to_string(expectedSeq));
            }
            expectedSeq = seq + 1;
            j.records.push_back(JRecord());
            r = &j.records.back();
            r->type = e[1];
            r->seq = seq;
            r->at = tokens[1];
            if (r->at == "p") {
                r->pre = true;
            } else {
                const size_t tilde = r->at.find('~');
                bool ok;
                if (tilde == std::string::npos) {
                    ok = parsePoint(r->at, &r->n1, &r->c1);
                    r->n0 = r->n1;
                    r->c0 = r->c1;
                } else {
                    r->interval = true;
                    ok = parsePoint(r->at.substr(0, tilde), &r->n0, &r->c0) && parsePoint(r->at.substr(tilde + 1), &r->n1, &r->c1);
                    if (ok && (r->n0 > r->n1 || (r->n0 == r->n1 && r->c0 >= r->c1))) {
                        j.errors.push_back("empty interval: " + e);
                    }
                }
                if (!ok) {
                    j.errors.push_back("bad point: " + e);
                }
            }
        }
        r->events++;
        for (size_t i = 2; i < tokens.size(); i++) {
            const std::string &tok = tokens[i];
            const size_t eq = tok.find('=');
            const size_t lt = tok.find('<');
            if (eq == std::string::npos) {
                r->words.push_back(tok);
            } else if (lt == std::string::npos || r->type != 'C') {
                const std::string name = tok.substr(0, eq);
                if (name == "pgs" && r->fields.count("pgs")) {
                    r->fields[name] += "," + tok.substr(eq + 1);     // an L list that continues
                } else {
                    r->fields[name] = tok.substr(eq + 1);
                }
            } else {
                r->items.push_back({ tok.substr(0, eq), tok.substr(eq + 1, lt - eq - 1), tok.substr(lt + 1) });
            }
        }
    }
    finish();
    return j;
}

static const JRecord *findRecord(const Journal &j, char type, const std::string &src = "")
{
    for (const JRecord &r : j.records) {
        if (r.type == type && (src.empty() || (r.fields.count("s") && r.fields.at("s") == src))) {
            return &r;
        }
    }
    return NULL;
}

static int countRecords(const Journal &j, char type, const std::string &src = "")
{
    int n = 0;
    for (const JRecord &r : j.records) {
        n += r.type == type && (src.empty() || (r.fields.count("s") && r.fields.at("s").compare(0, src.size(), src) == 0));
    }
    return n;
}

// The bytes that an item gives: group, offset and the value as bytes. False when the key is not known.
static bool applyItemText(std::vector<std::vector<uint8_t>> &bytes, const std::string &key, const std::string &text, std::string *error)
{
    if (key == "pid_profile" || key == "rate_profile") {
        const int t = bbpTrackedIndex(PG_SYSTEM_CONFIG);
        const unsigned offset = (key == "pid_profile") ? offsetof(systemConfig_t, pidProfileIndex) : offsetof(systemConfig_t, activeRateProfile);
        bytes[t][offset] = atoi(text.c_str());
        return true;
    }
    if (key.compare(0, 3, "pg.") == 0) {
        const size_t plus = key.find('+');
        const int t = bbpTrackedIndex(atoi(key.substr(3, plus - 3).c_str()));
        const unsigned offset = atoi(key.substr(plus + 1).c_str());
        if (t < 0 || text.size() % 2 || text.size() / 2 > BBP_RAW_ITEM_BYTES || offset + text.size() / 2 > bytes[t].size()) {
            *error = "bad raw item " + key;
            return false;
        }
        for (size_t i = 0; i < text.size() / 2; i++) {
            bytes[t][offset + i] = strtoul(text.substr(2 * i, 2).c_str(), NULL, 16);
        }
        return true;
    }
    if (key.compare(0, 3, "el.") == 0) {
        const std::vector<std::string> parts = split(key, '.');
        const bbpElementKind_t *kind = NULL;
        for (int k = 0; k < BBP_ELEMENT_KINDS; k++) {
            if (parts[1] == bbpElementKinds[k].name) {
                kind = &bbpElementKinds[k];
            }
        }
        const int t = kind ? bbpTrackedIndex(kind->pgn) : -1;
        const std::vector<std::string> fields = split(text, ',');
        if (t < 0 || (int)fields.size() != kind->fieldCount) {
            *error = "bad element item " + key + "=" + text;
            return false;
        }
        const unsigned base = kind->indexed ? atoi(parts[2].c_str()) * pgElementSize(trackedReg(t)) : 0;
        for (int f = 0; f < kind->fieldCount; f++) {
            int64_t number;
            if (!parseInteger(fields[f], &number, kind->fields[f].type == BBP_F_HEX32 ? 16 : 10)) {
                *error = "bad field " + key;
                return false;
            }
            storeNumber(&bytes[t][base + kind->fields[f].offset], fieldSize(&kind->fields[f]), number);
        }
        return true;
    }

    // [p<k>.|r<k>.]<name>[[<i>]]
    std::string name = key;
    int section = MASTER_VALUE;
    unsigned instance = 0;
    if ((key[0] == 'p' || key[0] == 'r') && isdigit(key[1]) && key.find('.') != std::string::npos) {
        section = (key[0] == 'p') ? PROFILE_VALUE : PROFILE_RATE_VALUE;
        instance = atoi(key.c_str() + 1);
        name = key.substr(key.find('.') + 1);
    }
    int element = -1;
    const size_t bracket = name.find('[');
    if (bracket != std::string::npos) {
        element = atoi(name.c_str() + bracket + 1);
        name = name.substr(0, bracket);
    }
    const clivalue_t *v = findEntry(name, section);
    const int t = v ? bbpTrackedIndex(v->pgn) : -1;
    if (t < 0) {
        *error = "unknown item " + key;
        return false;
    }
    unsigned offset = instance * pgElementSize(trackedReg(t)) + v->offset;
    if (element >= 0) {
        int64_t number;
        if ((v->type & VALUE_MODE_MASK) != MODE_ARRAY || element >= v->config.array.length || !parseInteger(text, &number)) {
            *error = "bad array item " + key + "=" + text;
            return false;
        }
        storeNumber(&bytes[t][offset + element * typeSizeOf(v)], typeSizeOf(v), number);
        return true;
    }
    if (!parseValue(v, text, &bytes[t][offset])) {
        *error = "bad value " + key + "=" + text;
        return false;
    }
    return true;
}

// Apply the C records to the state of the header: the old value of each item must be the state before it
static void applyJournal(Decoded &d, const Journal &j, std::vector<std::string> &errors)
{
    for (const JRecord &r : j.records) {
        if (r.type != 'C') {
            continue;
        }
        for (const JItem &it : r.items) {
            std::string error;
            if (!r.pre) {
                std::vector<std::vector<uint8_t>> check = d.bytes;
                if (!applyItemText(check, it.key, it.old, &error)) {
                    errors.push_back(error);
                    continue;
                }
                if (check != d.bytes) {
                    errors.push_back("record " + std::to_string(r.seq) + ": old value of " + it.key + " is not the state");
                }
            }
            if (!applyItemText(d.bytes, it.key, it.value, &error)) {
                errors.push_back(error);
            }
        }
    }
}

static void expectStateIsLive(const Decoded &d)
{
    for (int t = 0; t < BBP_TRACKED; t++) {
        const pgRegistry_t *reg = trackedReg(t);
        if (!reg) {
            continue;
        }
        int wrong = 0;
        for (unsigned i = 0; i < pgSize(reg); i++) {
            if (d.bytes[t][i] != reg->address[i]) {
                if (wrong++ < 5) {
                    ADD_FAILURE() << "pg " << pgN(reg) << "+" << i << ": " << (int)d.bytes[t][i] << " instead of " << (int)reg->address[i];
                }
            }
        }
        EXPECT_EQ(0, wrong) << "pg " << pgN(reg);
    }
}

static void expectNoErrors(const Journal &j)
{
    for (const std::string &e : j.errors) {
        ADD_FAILURE() << e;
    }
}

// The header states: one header line or part in each iteration, and the comparison with the previous log
static std::string runHeader(int changeAtCall = -1, void (*change)(void) = NULL)
{
    deviceBytes.clear();
    sim.running = false;
    sim.iteration = 0;
    blackboxHeaderBudget = 0;
    blackboxParamsStart();
    for (int call = 0; call < 100000; call++) {
        blackboxParamsHeaderTick();
        if (call == changeAtCall && change) {
            change();
        }
        blackboxHeaderBudget = MIN(blackboxHeaderBudget + 64, 256);
        if (blackboxParamsWriteHeader()) {
            break;
        }
    }
    return deviceBytes;
}

// T0: the log is RUNNING
static void runningLog(void)
{
    sim.running = true;
    sim.iteration = 0;
    blackboxParamsRunning();
}

// One logged frame: the PID task after the frame, then the next iteration
static void logFrame(void)
{
    sim.counter = 0;
    blackboxParamsAfterFrame();
    sim.iteration++;
}

// Frames until no event for 64 frames
static void drainJournal(void)
{
    size_t count = sim.events.size();
    for (int quiet = 0; quiet < 64; ) {
        logFrame();
        if (sim.events.size() == count) {
            quiet++;
        } else {
            quiet = 0;
            count = sim.events.size();
        }
    }
}

static std::string hexText(uint32_t value)
{
    char text[16];
    snprintf(text, sizeof(text), "%x", value);
    return text;
}

static void mutateRandomBytes(int count)
{
    for (int i = 0; i < count; i++) {
        int t;
        do {
            t = rng() % BBP_TRACKED;
        } while (!trackedReg(t) || bbpTrackedPgs[t].pgn == PG_BLACKBOX_CONFIG);
        trackedReg(t)->address[rng() % pgSize(trackedReg(t))] = rng();
    }
    // Bytes after the NUL of a string: raw items
    if (rng() % 2) {
        tidyStrings();
    }
}

TEST(BlackboxParamsJournalTest, OffWritesNoRecord)
{
    setupConfig(BLACKBOX_PARAMS_OFF);
    runHeader();
    runningLog();
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);
    pidProfilesMutable(0)->pid[0].P += 1;
    blackboxParamsOpEnd();
    drainJournal();
    EXPECT_EQ(0u, sim.events.size());
    EXPECT_EQ(0u, blackboxParamsSeq());
}

// Random changes in operations: the header and the records give every byte at the end, and each old value is
// the state before the record
TEST(BlackboxParamsJournalTest, RandomChangesAreExact)
{
    int records = 0, events = 0;
    size_t longest = 0;
    for (int run = 0; run < 20; run++) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        Decoded d = decodeHeader(runHeader());
        expectExact(d);
        runningLog();
        for (int i = 0; i < 400; i++) {
            if (rng() % 4 == 0) {
                const uint16_t cmd = 100 + rng() % 100;
                sim.counter = rng() % 2;
                blackboxParamsOpBegin(BBP_SRC_MSP, cmd);
                mutateRandomBytes(1 + rng() % 6);
                if (rng() % 3 == 0) {
                    blackboxParamsApplied(BBP_LOADER_PID, rng() % PID_PROFILE_COUNT);
                }
                blackboxParamsOpEnd();
            }
            logFrame();
        }
        drainJournal();
        const Journal j = parseJournal(sim.events);
        expectNoErrors(j);
        std::vector<std::string> errors;
        applyJournal(d, j, errors);
        for (const std::string &e : errors) {
            ADD_FAILURE() << e;
        }
        expectStateIsLive(d);
        // Every change is of an operation, at an exact point
        for (const JRecord &r : j.records) {
            if (r.type == 'C') {
                EXPECT_EQ('m', r.fields.at("s")[0]) << r.seq;
                EXPECT_FALSE(r.interval) << r.seq;
            }
        }
        EXPECT_EQ(blackboxParamsSeq(), j.records.empty() ? 0 : j.records.back().seq);
        records += j.records.size();
        events += j.events;
        longest = std::max(longest, j.longest);
        if (HasFailure()) {
            break;
        }
    }
    printf("random changes: %d records in %d events, longest event %zu chars\n", records, events, longest);
}

// A change of a whole PID profile: one record in more than one event
TEST(BlackboxParamsJournalTest, ContinuationEvents)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog();
    blackboxParamsOpBegin(BBP_SRC_MSP, 95);
    fillRandom((uint8_t *)pidProfilesMutable(3), sizeof(pidProfile_t));
    tidyStrings();
    blackboxParamsOpEnd();
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *r = findRecord(j, 'C', "m.95");
    ASSERT_NE(nullptr, r);
    EXPECT_GT(r->events, 3);
    EXPECT_EQ(atoi(r->fields.at("n").c_str()), (int)r->items.size());
    for (const std::string &e : sim.events) {
        EXPECT_LE(e.size(), (size_t)BBP_EVENT_MAX);
    }
    std::vector<std::string> errors;
    applyJournal(d, j, errors);
    EXPECT_TRUE(errors.empty());
    expectStateIsLive(d);
    printf("one PID profile: %zu items in %d events\n", r->items.size(), r->events);
}

// A bad CRC is found
TEST(BlackboxParamsJournalTest, CrcFindsAChange)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);
    pidProfilesMutable(0)->pid[0].P += 1;
    blackboxParamsOpEnd();
    drainJournal();
    std::vector<std::string> events = sim.events;
    EXPECT_TRUE(parseJournal(events).errors.empty());
    for (std::string &e : events) {
        if (e.compare(0, 2, "PC") == 0) {
            e[e.find('=') + 1] ^= 1;
        }
    }
    EXPECT_FALSE(parseJournal(events).errors.empty());
}

// MSP, then changePidProfile, then the loaders of pidLoadProfile: one source, and the change before the 'A'
TEST(BlackboxParamsJournalTest, NestedOperations)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    for (int i = 0; i < 10; i++) {
        logFrame();
    }
    sim.counter = 1;
    blackboxParamsOpBegin(BBP_SRC_MSP, 210);                // MSP_SELECT_SETTING
    blackboxParamsOpBegin(BBP_SRC_PROFILE, 2);              // changePidProfile(2)
    systemConfigMutable()->pidProfileIndex = 2;
    currentPidProfile = pidProfilesMutable(2);
    pidProfilesMutable(2)->governor.gain += 1;              // governorInitProfile: validateAndFixGovernorProfile
    blackboxParamsApplied(BBP_LOADER_GOVERNOR, 2);
    blackboxParamsApplied(BBP_LOADER_RESCUE, 2);
    blackboxParamsApplied(BBP_LOADER_PID, 2);
    blackboxParamsOpEnd();
    blackboxParamsOpEnd();
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    std::string order;
    for (const JRecord &r : j.records) {
        if (r.type == 'C' || r.type == 'A') {
            order += std::string(1, r.type) + (r.type == 'C' ? r.fields.at("s") : r.words[0]) + " ";
            EXPECT_EQ("10.1", r.at);
        }
    }
    // The governor change before A gov, the profile index (SYSTEM_CONFIG is in the pid region) before A pid
    EXPECT_EQ("Cm.210 Agov/2 Arsc/2 Cm.210 Apid/2 ", order);
    const JRecord *first = findRecord(j, 'C', "m.210");
    ASSERT_NE(nullptr, first);
    EXPECT_EQ("p2.gov_gain", first->items[0].key);
    EXPECT_EQ(0, countRecords(j, 'C', "p"));
    const JRecord *a = findRecord(j, 'A');
    ASSERT_NE(nullptr, a);
    EXPECT_EQ(hexText(sim.fpGov), std::string(a->fields.at("fp")).erase(0, 4));
}

// A write that no hook saw, before an operation: its own 'u' record with an interval; the operation keeps its
// exact point
TEST(BlackboxParamsJournalTest, PreCaptureOfAnUnhookedWrite)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog();
    sim.iteration = 90;
    sim.counter = 1;
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);                  // verifies every group at 90.1
    blackboxParamsOpEnd();
    sim.iteration = 100;
    controlRateProfilesMutable(1)->cyclic_ring += 7;        // the unhooked write
    sim.iteration = 105;
    sim.counter = 0;
    blackboxParamsOpBegin(BBP_SRC_MSP, 202);
    pidProfilesMutable(0)->pid[2].P += 3;
    blackboxParamsOpEnd();
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *u = findRecord(j, 'C', "u");
    const JRecord *m = findRecord(j, 'C', "m.202");
    ASSERT_NE(nullptr, u);
    ASSERT_NE(nullptr, m);
    EXPECT_EQ("90.1~105.0", u->at);
    EXPECT_EQ(1u, u->items.size());
    EXPECT_EQ("r1.cyclic_ring", u->items[0].key);
    EXPECT_EQ("105.0", m->at);
    EXPECT_EQ(1u, m->items.size());
    EXPECT_EQ("p0.yaw_p_gain", m->items[0].key);
    EXPECT_LT(u->seq, m->seq);
    std::vector<std::string> errors;
    applyJournal(d, j, errors);
    EXPECT_TRUE(errors.empty());
    expectStateIsLive(d);
}

// The ring overflows: the items that fit, then 'L', then a 'y' record for each group, with an interval from
// the last verified point. A later operation on another group keeps its exact point.
TEST(BlackboxParamsJournalTest, RingOverflowGivesLostThenResync)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog();
    sim.iteration = 50;
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);                  // verifies every group at 50.0
    blackboxParamsOpEnd();

    // A slow device: nothing drains. Operations change more than the ring takes.
    sim.freeSpace = 0;
    for (int i = 0; i < 100; i++) {
        sim.iteration = 60 + i;
        blackboxParamsOpBegin(BBP_SRC_MSP, 95);
        pidProfilesMutable(i % PID_PROFILE_COUNT)->pid[i % 3].P += 1;
        controlRateProfilesMutable(i % CONTROL_RATE_PROFILE_COUNT)->rcRates[i % 3] += 1;
        blackboxParamsOpEnd();
    }
    sim.iteration = 170;
    blackboxParamsAfterFrame();     // 'L'

    // The device takes events again. When the first group is back, an operation on another group.
    sim.freeSpace = 100000;
    while (sim.iteration < 1000 && std::none_of(sim.events.begin(), sim.events.end(),
        [](const std::string &e) { return e.find(" s=y ") != std::string::npos; })) {
        logFrame();
    }
    const uint32_t opIteration = sim.iteration;
    blackboxParamsOpBegin(BBP_SRC_MSP, 211);
    governorConfigMutable()->gov_spoolup_time += 1;
    blackboxParamsOpEnd();
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *lost = findRecord(j, 'L');
    ASSERT_NE(nullptr, lost);
    EXPECT_NE(std::string::npos, ("," + lost->fields.at("pgs") + ",").find("," + std::to_string(PG_PID_PROFILE) + ","));
    EXPECT_GE(countRecords(j, 'C', "y"), 1);
    // The first operation that did not fit verified the groups in its pre-capture, at the point of 'L'
    EXPECT_EQ(lost->at, std::to_string(lost->n1) + ".0");
    EXPECT_GT(lost->n1, 60u);
    for (const JRecord &r : j.records) {
        if (r.type == 'C' && r.fields.at("s") == "y") {
            EXPECT_TRUE(r.interval);
            EXPECT_EQ(lost->at + "~", r.at.substr(0, lost->at.size() + 1)) << "the interval starts at the last verified point";
            EXPECT_GT(r.seq, lost->seq);
        }
    }
    const JRecord *gov = findRecord(j, 'C', "m.211");
    if (!gov || getenv("BBP_EVENTS")) {
        for (const std::string &e : sim.events) {
            printf("%s\n", e.substr(0, 100).c_str());
        }
    }
    ASSERT_NE(nullptr, gov);
    EXPECT_EQ(std::to_string(opIteration) + ".0", gov->at);
    ASSERT_EQ(1u, gov->items.size());
    EXPECT_EQ("gov_spoolup_time", gov->items[0].key);
    EXPECT_EQ("60.0", findRecord(j, 'C', "m.95")->at);
    EXPECT_GT(countRecords(j, 'C', "m.95"), 10);

    // The state is exact again after the resync
    std::vector<std::string> errors;
    Decoded state = d;
    applyJournal(state, j, errors);
    expectStateIsLive(state);

    // The end record
    sim.millis = 5000;
    EXPECT_FALSE(blackboxParamsHoldLogEnd());
    blackboxParamsEnd();
    const Journal end = parseJournal(sim.events);
    const JRecord *q = findRecord(end, 'Q');
    ASSERT_NE(nullptr, q);
    EXPECT_EQ("0", q->fields.at("unsent"));
    EXPECT_EQ("0", q->fields.at("lostrec"));
    printf("overflow: %d records, %d y records, L pgs=%s, Q lostrec=%s\n", (int)j.records.size(),
        countRecords(j, 'C', "y"), lost->fields.at("pgs").c_str(), q->fields.at("lostrec").c_str());
}

// A resync that does not fit in the ring: 'y' records with part=1 keep the 'L' loss of the group open. When a 'y'
// record without part=1 ends the loss, every byte of the group is known again (spec 2.6.5).
TEST(BlackboxParamsJournalTest, PartialResyncKeepsTheLossOpen)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog();
    sim.iteration = 50;
    sim.freeSpace = 0;
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);
    for (int k = 0; k < PID_PROFILE_COUNT; k++) {
        uint8_t *p = (uint8_t *)pidProfilesMutable(k);
        for (unsigned i = 16; i < sizeof(pidProfile_t); i++) {
            p[i] ^= 1;
        }
    }
    blackboxParamsOpEnd();
    sim.freeSpace = 100000;
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const int t = bbpTrackedIndex(PG_PID_PROFILE);
    const std::string pgs = std::to_string(PG_PID_PROFILE);
    int parts = 0, ends = 0;
    for (const JRecord &r : j.records) {
        Journal one;
        one.records.push_back(r);
        std::vector<std::string> errors;
        applyJournal(d, one, errors);
        for (const std::string &e : errors) {
            ADD_FAILURE() << e;
        }
        if (r.type != 'C' || r.fields.at("s") != "y" || r.fields.at("pgs") != pgs) {
            continue;
        }
        if (r.fields.count("part")) {
            EXPECT_EQ("1", r.fields.at("part"));
            EXPECT_GT(r.items.size(), 0u) << "a part without items";
            parts++;
            continue;
        }
        // The end of the loss: the state of the reader is the live state
        ends++;
        EXPECT_EQ(0, memcmp(d.bytes[t].data(), trackedReg(t)->address, pgSize(trackedReg(t)))) << "y record " << r.seq;
    }
    EXPECT_GE(parts, 1);
    EXPECT_EQ(1, ends);
    expectStateIsLive(d);
    printf("partial resync: %d records, %d y parts\n", (int)j.records.size(), parts);
}

// A change and its revert are both lost while the ring is full: the resync is a 'y' record without items, with an
// interval from the last verified point of the group (spec 2.4)
TEST(BlackboxParamsJournalTest, ItemlessResyncHasAnInterval)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog();
    sim.freeSpace = 0;
    for (int i = 0; i < 100; i++) {
        sim.iteration = 60 + i;
        blackboxParamsOpBegin(BBP_SRC_MSP, 95);
        pidProfilesMutable(i % PID_PROFILE_COUNT)->pid[i % 3].P += 1;
        blackboxParamsOpEnd();
    }
    sim.iteration = 170;
    blackboxParamsOpBegin(BBP_SRC_MSP, 211);
    governorConfigMutable()->gov_spoolup_time += 1;
    blackboxParamsOpEnd();
    sim.iteration = 171;
    blackboxParamsOpBegin(BBP_SRC_MSP, 211);
    governorConfigMutable()->gov_spoolup_time -= 1;
    blackboxParamsOpEnd();
    sim.freeSpace = 100000;
    drainJournal();

    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const std::string gov = std::to_string(PG_GOVERNOR_CONFIG);
    const JRecord *y = NULL;
    for (const JRecord &r : j.records) {
        if (r.type == 'C' && r.fields.at("s") == "y" && r.fields.at("pgs") == gov) {
            y = &r;
        }
    }
    ASSERT_NE(nullptr, y);
    EXPECT_EQ(0u, y->items.size());
    EXPECT_TRUE(y->interval) << y->at;
    EXPECT_LT(y->n0, 171u) << "the interval starts before the revert";
    std::vector<std::string> errors;
    applyJournal(d, j, errors);
    expectStateIsLive(d);
}

// Before T0 every stamp is 'p'. Changes between two logs are 'v' records. After the CLI, 'M shadow-reset'.
TEST(BlackboxParamsJournalTest, BeforeT0AndBetweenLogs)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    // A change during the header
    Decoded d = decodeHeader(runHeader(100, [](void) {
        blackboxParamsOpBegin(BBP_SRC_MSP, 7);
        pidProfilesMutable(1)->pid[0].I += 2;
        blackboxParamsOpEnd();
    }));
    runningLog();
    drainJournal();
    Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *pre = findRecord(j, 'C', "m.7");
    ASSERT_NE(nullptr, pre);
    EXPECT_TRUE(pre->pre);
    const JRecord *rt = findRecord(j, 'R');
    ASSERT_NE(nullptr, rt);
    EXPECT_TRUE(rt->pre);
    EXPECT_EQ(5u, rt->fields.size());
    std::vector<std::string> errors;
    applyJournal(d, j, errors);
    expectStateIsLive(d);

    // The end of the log, changes between the logs (no log: no operation records), the next log
    blackboxParamsStop();
    blackboxParamsOpBegin(BBP_SRC_MSP, 8);
    pidProfilesMutable(4)->pid[1].D += 5;
    servoParamsMutable(2)->rate += 3;
    blackboxParamsOpEnd();
    sim.events.clear();
    Decoded d2 = decodeHeader(runHeader());
    expectExact(d2);
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(2, countRecords(j, 'C', "v"));
    for (const JRecord &r : j.records) {
        if (r.type == 'C') {
            EXPECT_EQ("v", r.fields.at("s"));
            EXPECT_TRUE(r.pre);
        }
    }
    EXPECT_EQ(0, countRecords(j, 'M'));

    // The CLI overwrote the shadow
    blackboxParamsStop();
    blackboxParamsShadowLost();
    pidProfilesMutable(4)->pid[1].D += 5;
    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *m = findRecord(j, 'M');
#ifdef BLACKBOX_PARAMS_OWN_SHADOW
    // CLI copy buffers do not alias the private journal shadow.
    EXPECT_EQ(nullptr, m);
    EXPECT_EQ(1, countRecords(j, 'C', "v"));
#else
    ASSERT_NE(nullptr, m);
    EXPECT_EQ("shadow-reset", m->words.at(0));
    EXPECT_EQ(1u, m->seq);
    EXPECT_EQ(0, countRecords(j, 'C'));
#endif
}

// Between-log loaders: a loader that runs while no log is open gives an 'A' record stamped p at the next T0, after the 'v'
// records, when its region has not changed since the run. A loader whose region changed after its run gives none.
TEST(BlackboxParamsJournalTest, LoaderRunBetweenLogsGivesApplyAtT0)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    drainJournal();
    blackboxParamsStop();

    pidProfilesMutable(0)->pid[0].P += 7;           // a change with its loader run after it
    blackboxParamsApplied(BBP_LOADER_PID, 0);
    blackboxParamsApplied(BBP_LOADER_SETPOINT, 0);
    controlRateProfilesMutable(0)->cyclic_ring += 3; // a change after the run of its loader

    sim.events.clear();
    Decoded d = decodeHeader(runHeader());
    expectExact(d);
    runningLog();
    drainJournal();
    Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(2, countRecords(j, 'C', "v"));
    int pid = 0, sp = 0;
    uint32_t lastV = 0, firstA = 0;
    for (const JRecord &r : j.records) {
        if (r.type == 'C') {
            lastV = MAX(lastV, r.seq);
        }
        if (r.type == 'A') {
            EXPECT_TRUE(r.pre);
            ASSERT_FALSE(r.words.empty());
            pid += r.words[0] == "pid/0";
            sp += r.words[0].compare(0, 2, "sp") == 0;
            firstA = firstA ? firstA : r.seq;
        }
    }
    EXPECT_EQ(1, pid);
    EXPECT_EQ(0, sp);
    EXPECT_GT(firstA, lastV);

    // The next log: no loader ran in between, so no 'A' record
    blackboxParamsStop();
    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(0, countRecords(j, 'A'));
}

// A log that ends before its records are written: in a header state (an arm blip), or with records that wait. Their
// changes are in the shadow, so the next log has no 'v' record of them: it starts with 'M shadow-reset'.
TEST(BlackboxParamsJournalTest, LostRecordsGiveShadowReset)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    drainJournal();
    blackboxParamsStop();                       // log 1 ends with all its records
    pidProfilesMutable(0)->pid[0].P += 7;       // a change between the logs

    // Log 2: the pilot disarms before the 'v' compare of the PID profiles has made its record. Nothing is lost:
    // log 3 has the 'v' record.
    sim.events.clear();
    blackboxParamsStart();
    blackboxParamsHeaderTick();
    blackboxParamsStop();
    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(1, countRecords(j, 'C', "v"));
    EXPECT_EQ(0, countRecords(j, 'M'));
    blackboxParamsStop();
    pidProfilesMutable(0)->pid[0].P += 7;

    // Log 2b: the header compares the groups ('v' records in the ring), then the pilot disarms before T0
    sim.events.clear();
    blackboxParamsStart();
    for (int i = 0; i < 100; i++) {
        blackboxParamsHeaderTick();
    }
    blackboxParamsStop();
    EXPECT_TRUE(sim.events.empty());

    // Log 3
    sim.events.clear();
    Decoded d = decodeHeader(runHeader());
    expectExact(d);
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *m = findRecord(j, 'M');
    ASSERT_NE(nullptr, m);
    EXPECT_EQ("shadow-reset", m->words.at(0));
    EXPECT_EQ(1u, m->seq);
    EXPECT_EQ(0, countRecords(j, 'C'));

    // Log 3 ends while a record waits (LOG_END after the hold): log 4 starts with 'M shadow-reset'
    sim.freeSpace = 0;
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);
    pidProfilesMutable(1)->pid[0].P += 1;
    blackboxParamsOpEnd();
    blackboxParamsEnd();
    blackboxParamsStop();
    sim.freeSpace = 100000;
    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    m = findRecord(j, 'M');
    ASSERT_NE(nullptr, m);
    EXPECT_EQ("shadow-reset", m->words.at(0));

    // Log 4 ends with all its records: log 5 compares with the shadow again
    blackboxParamsStop();
    pidProfilesMutable(2)->pid[0].P += 1;
    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    j = parseJournal(sim.events);
    EXPECT_EQ(0, countRecords(j, 'M'));
    EXPECT_EQ(1, countRecords(j, 'C', "v"));
}

// The device is full (BLACKBOX_STATE_FULL stops the journal): an adjustment after it is a 'v' record of the next log
TEST(BlackboxParamsJournalTest, FullDeviceStopsTheJournal)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    drainJournal();
    blackboxParamsStop();
    sim.running = false;
    blackboxParamsOpBegin(BBP_SRC_ADJUSTMENT, 3);
    pidProfilesMutable(0)->pid[1].P += 9;
    blackboxParamsOpEnd();

    sim.events.clear();
    runHeader();
    runningLog();
    drainJournal();
    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(1, countRecords(j, 'C', "v"));
    EXPECT_EQ(0, countRecords(j, 'M'));
}

// The stamp rule against a model of taskMainPidLoop() (core.c) for pid_process_denom 1-16: an operation
// between two ticks, and which subtasks of which frame used the new value. Also the late subtasks (a tick after
// blackboxUpdate: flush, and the filter update with pid_process_denom 3), with the rule that the reader applies.
TEST(BlackboxParamsJournalTest, StampRuleForEveryDenominator)
{
    // The tick of each subtask (pos, sp, pid, mix, mot, fupd, bb, flush) in core.c
    static const uint8_t schedule[8][CORE_ST_COUNT] = {
        { 0, 0, 0, 0, 0, 0, 0, 0 },     // 1
        { 0, 0, 0, 1, 1, 1, 1, 0 },     // 2
        { 0, 0, 0, 0, 1, 2, 1, 2 },     // 3
        { 0, 0, 0, 1, 1, 2, 2, 3 },     // 4
        { 0, 0, 0, 1, 2, 2, 3, 4 },     // 5
        { 0, 0, 1, 2, 3, 3, 4, 5 },     // 6
        { 0, 0, 1, 2, 3, 4, 5, 6 },     // 7
        { 0, 1, 2, 3, 4, 5, 6, 7 },     // 8 and more
    };
    int checked = 0, uncertain = 0;
    for (int denom = 1; denom <= 16; denom++) {
        const uint8_t *ticks = schedule[MIN(denom, 8) - 1];
        const int b = ticks[CORE_ST_BLACKBOX];
        for (int c = 0; c < denom; c++) {
            setupConfig(BLACKBOX_PARAMS_FULL);
            memcpy(sim.ticks, ticks, sizeof(sim.ticks));
            runHeader();
            runningLog();
            // Cycle 0 logged frame 0, cycle 1 logs frame 1. The write comes after ticks 0..c-1 of cycle 1.
            sim.iteration = (c > b) ? 2 : 1;
            sim.counter = c;
            blackboxParamsOpBegin(BBP_SRC_MSP, 1);
            pidProfilesMutable(0)->pid[0].P += 1;
            blackboxParamsOpEnd();
            drainJournal();
            const Journal j = parseJournal(sim.events);
            const JRecord *r = findRecord(j, 'C', "m.1");
            ASSERT_NE(nullptr, r);
            ASSERT_FALSE(r->interval);
            for (int frame = 0; frame < 4; frame++) {
                for (int s = 0; s < CORE_ST_COUNT; s++) {
                    // A late subtask (after blackboxUpdate in its cycle) acts on the next frame: frame F has its run
                    // of cycle F - 1. That run used the new value when it came after the write (cycle 1, tick c).
                    const bool late = ticks[s] > b;
                    const int cycle = late ? frame - 1 : frame;
                    const bool used = cycle > 1 || (cycle == 1 && ticks[s] >= c);
                    // The rule of the reader (spec 2.5)
                    const uint32_t n = r->n1;
                    const int stampTick = r->c1;
                    if (late && (uint32_t)frame == n && stampTick == 0) {
                        uncertain++;    // uncertain: either value is allowed
                        continue;
                    }
                    const bool stamped = late ? (uint32_t)frame > n
                        : (uint32_t)frame > n || ((uint32_t)frame == n && ticks[s] >= stampTick);
                    EXPECT_EQ(used, stamped) << "denom " << denom << " c " << c << " frame " << frame << " subtask " << s << " at " << r->at;
                    checked++;
                }
            }
            EXPECT_LE((int)r->c1, b);
        }
    }
    printf("stamp rule: %d subtask runs checked for pid_process_denom 1-16, %d late runs uncertain\n", checked, uncertain);
}

// The scan: a write that no hook saw is found. Its interval starts where the previous pass of the group started.
TEST(BlackboxParamsJournalTest, ScanVerifiedPointIsTheStartOfThePass)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    // Scan steps of one pass over all groups
    int steps = 0;
    for (int t = 0; t < BBP_TRACKED; t++) {
        if (trackedReg(t)) {
            steps += (pgSize(trackedReg(t)) + BBP_SCAN_BYTES - 1) / BBP_SCAN_BYTES;
        }
    }
    // Pass 1 of PID_PROFILE (group 0) starts after frame 0: point 1.0. Pass 2 starts after frame 'steps'. The scan
    // finds the write after frame steps + chunk (point steps + 1 + chunk), and the capture records it in steps.
    for (int i = 0; i < steps + 3; i++) {
        logFrame();
    }
    // A write in a part of the PID profiles that pass 2 has not compared yet
    pidProfilesMutable(PID_PROFILE_COUNT - 1)->pid[0].P += 1;
    const int chunk = ((PID_PROFILE_COUNT - 1) * sizeof(pidProfile_t) + offsetof(pidProfile_t, pid)) / BBP_SCAN_BYTES;
    ASSERT_GE(chunk, 3);
    for (int i = 0; i < steps + 3; i++) {
        logFrame();
    }
    drainJournal();
    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    const JRecord *u = findRecord(j, 'C', "u");
    ASSERT_NE(nullptr, u);
    EXPECT_TRUE(u->interval);
    // Not the end of pass 1, and not the start of pass 2: the start of pass 1, the last complete pass. The end of
    // the interval is the step of the capture that made the record.
    EXPECT_EQ(1u, u->n0);
    EXPECT_EQ(0u, u->c0);
    EXPECT_GE(u->n1, (uint32_t)(steps + 1 + chunk));
    EXPECT_LE(u->n1, (uint32_t)(steps + 1 + chunk + 16));
    printf("scan: one pass is %d logged iterations, write found in %s\n", steps, u->at.c_str());
}

// Runtime values: an 'R' record with an interval when a polled value changes
TEST(BlackboxParamsJournalTest, RuntimePoll)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    for (int i = 0; i < 40; i++) {
        logFrame();
    }
    sim.fpGov = 0xabcdef12;
    for (int i = 0; i < 40; i++) {
        logFrame();
    }
    drainJournal();
    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    ASSERT_EQ(2, countRecords(j, 'R'));
    const JRecord &r = j.records.back();
    EXPECT_EQ('R', r.type);
    EXPECT_EQ("abcdef12", r.fields.at("fp.gov"));
    EXPECT_EQ(1u, r.fields.size());
    EXPECT_TRUE(r.interval);
    EXPECT_EQ(r.n0 + 20, r.n1);
}

// LOG_END waits for the records, for at most 100 ms
TEST(BlackboxParamsJournalTest, HoldLogEnd)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    EXPECT_TRUE(blackboxParamsHoldLogEnd());    // the 'R' record of T0
    drainJournal();
    EXPECT_FALSE(blackboxParamsHoldLogEnd());
    sim.freeSpace = 0;
    blackboxParamsOpBegin(BBP_SRC_MSP, 1);
    pidProfilesMutable(0)->pid[0].P += 1;
    blackboxParamsOpEnd();
    for (int ms = 0; ms < 100; ms += 10) {
        EXPECT_TRUE(blackboxParamsHoldLogEnd());
        logFrame();
        sim.millis += 10;
    }
    EXPECT_FALSE(blackboxParamsHoldLogEnd());
    blackboxParamsEnd();
    const Journal j = parseJournal(sim.events);
    const JRecord *q = findRecord(j, 'Q');
    ASSERT_NE(nullptr, q);
    EXPECT_EQ("1", q->fields.at("unsent"));
}

// The time of one operation: the pre-capture and the post-capture of all groups, with one change
TEST(BlackboxParamsJournalTest, OperationTime)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    double best = 1e9, bestEmpty = 1e9, bestScan = 1e9, bestDrain = 1e9;
    for (int run = 0; run < 2000; run++) {
        auto start = std::chrono::steady_clock::now();
        blackboxParamsOpBegin(BBP_SRC_MSP, 1);
        pidProfilesMutable(run % PID_PROFILE_COUNT)->pid[0].P += 1;
        blackboxParamsOpEnd();
        best = std::min(best, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());

        start = std::chrono::steady_clock::now();
        blackboxParamsOpBegin(BBP_SRC_MSP, 1);
        blackboxParamsOpEnd();
        bestEmpty = std::min(bestEmpty, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());

        // A drain step with an event, and one without
        const size_t before = sim.events.size();
        start = std::chrono::steady_clock::now();
        blackboxParamsAfterFrame();
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
        if (sim.events.size() > before) {
            bestDrain = std::min(bestDrain, us);
        }
        sim.iteration++;
        drainJournal();
        start = std::chrono::steady_clock::now();
        blackboxParamsAfterFrame();
        bestScan = std::min(bestScan, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());
        sim.iteration++;
        sim.events.clear();
    }
    printf("on this computer: operation with one change %.2f us, operation without a change %.2f us, "
        "PID task with an event %.2f us, idle PID task step %.3f us\n", best, bestEmpty, bestDrain, bestScan);
}

// Change every value of the PID profiles and the rate profiles
static void changeAllProfiles(void)
{
    for (int k = 0; k < PID_PROFILE_COUNT; k++) {
        uint8_t *p = (uint8_t *)pidProfilesMutable(k);
        for (unsigned i = 16; i < sizeof(pidProfile_t); i++) {
            p[i] ^= 1;
        }
    }
    for (int k = 0; k < CONTROL_RATE_PROFILE_COUNT; k++) {
        uint8_t *p = (uint8_t *)controlRateProfilesMutable(k);
        for (unsigned i = 16; i < sizeof(controlRateConfig_t); i++) {
            p[i] ^= 1;
        }
    }
}

// The slowest call of a sequence of calls in the PID task. The sequence runs again and again, and each call keeps
// its best time (the host scheduler adds time to some calls), so the result is the cost of the slowest call.
struct SlowestCall {
    double us = 0;
    int calls = 0;
};

static double untimedUs;

// A part of a timed call that does not count
static bool untimed(bool (*part)(void))
{
    const auto start = std::chrono::steady_clock::now();
    const bool result = part();
    untimedUs += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    return result;
}

static SlowestCall slowestCall(void (*setup)(void), bool (*call)(void))
{
    std::vector<double> best;
    for (int run = 0; run < 20; run++) {
        setup();
        for (size_t i = 0; ; i++) {
            untimedUs = 0;
            const auto start = std::chrono::steady_clock::now();
            const bool more = call();
            const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() - untimedUs;
            if (i >= best.size()) {
                best.push_back(us);
            } else {
                best[i] = std::min(best[i], us);
            }
            if (!more || i > 100000) {
                break;
            }
        }
    }
    SlowestCall r;
    if (getenv("BBP_DEBUG_SLOW")) {
        std::vector<size_t> idx(best.size());
        for (size_t i = 0; i < idx.size(); i++) idx[i] = i;
        std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) { return best[a] > best[b]; });
        for (int i = 0; i < 8 && i < (int)idx.size(); i++) printf("  call %zu: %.3f us\n", idx[i], best[idx[i]]);
    }
    r.us = *std::max_element(best.begin(), best.end());
    r.calls = best.size();
    return r;
}

// The worst cases of the PID task (spec 3.12): every value of the 6 PID profiles and the 6 rate profiles changed,
// (a) between two logs: the 'v' compare in the header states, (b) by an operation that the ring does not take: the
// resync after the overflow, (c) by a write that no hook saw: the scan. The host time of the slowest call, against
// the operation without a change of OperationTime. The bench test (6.4) measures the target.
TEST(BlackboxParamsJournalTest, WorstPidTaskCall)
{
    const SlowestCall header = slowestCall([](void) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        runHeader();
        runningLog();
        drainJournal();
        blackboxParamsStop();
        changeAllProfiles();
        sim.events.clear();
        sim.running = false;
        blackboxHeaderBudget = 0;
        blackboxParamsStart();
    }, [](void) {
        // The header line of the same iteration is outside the time: CallTime measures it
        blackboxParamsHeaderTick();
        return blackboxParamsActive() && !untimed([](void) {
            blackboxHeaderBudget = MIN(blackboxHeaderBudget + 64, 256);
            return blackboxParamsWriteHeader();
        });
    });

    const SlowestCall resync = slowestCall([](void) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        runHeader();
        runningLog();
        drainJournal();
        sim.freeSpace = 0;
        blackboxParamsOpBegin(BBP_SRC_MSP, 1);
        changeAllProfiles();
        blackboxParamsOpEnd();
        sim.freeSpace = 100000;
    }, [](void) {
        logFrame();
        return blackboxParamsHoldLogEnd() || sim.iteration < 4000;
    });

    const SlowestCall scan = slowestCall([](void) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        runHeader();
        runningLog();
        drainJournal();
        changeAllProfiles();
    }, [](void) {
        logFrame();
        return blackboxParamsHoldLogEnd() || sim.iteration < 4000;
    });

    // For comparison: the drain of records that wait, without a capture (an event of up to 128 chars each call)
    const SlowestCall drain = slowestCall([](void) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        runHeader();
        runningLog();
        drainJournal();
        sim.freeSpace = 0;
        for (int i = 0; i < 12; i++) {
            blackboxParamsOpBegin(BBP_SRC_MSP, 1);
            for (int a = 0; a < 3; a++) {
                pidProfilesMutable(i % PID_PROFILE_COUNT)->pid[a].P += 1;
                pidProfilesMutable(i % PID_PROFILE_COUNT)->pid[a].I += 1;
            }
            blackboxParamsOpEnd();
        }
        sim.freeSpace = 100000;
    }, [](void) {
        logFrame();
        return blackboxParamsHoldLogEnd();
    });

    // The state is exact after each case
    for (int c = 0; c < 2; c++) {
        setupConfig(BLACKBOX_PARAMS_FULL);
        Decoded d = decodeHeader(runHeader());
        runningLog();
        if (c == 0) {
            sim.freeSpace = 0;
            blackboxParamsOpBegin(BBP_SRC_MSP, 1);
        }
        changeAllProfiles();
        if (c == 0) {
            blackboxParamsOpEnd();
            sim.freeSpace = 100000;
        }
        drainJournal();
        const Journal j = parseJournal(sim.events);
        expectNoErrors(j);
        std::vector<std::string> errors;
        applyJournal(d, j, errors);
        for (const std::string &e : errors) {
            ADD_FAILURE() << e;
        }
        expectStateIsLive(d);
    }

    printf("slowest PID task call on this computer: header 'v' %.2f us (%d calls), resync %.2f us, scan 'u' %.2f us; "
        "drain only %.2f us\n", header.us, header.calls, resync.us, scan.us, drain.us);
}

// blackboxParamsChangeCount(): rc_adjustments.c skips the operation of a set that had no effect, while the count
// stays the same. It changes with each change that a capture finds, each loader and each log start.
TEST(BlackboxParamsJournalTest, ChangeCount)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    drainJournal();
    uint32_t count = blackboxParamsChangeCount();

    // A set that clamps to the stored value: no change
    blackboxParamsOpBegin(BBP_SRC_ADJUSTMENT, 3);
    controlRateProfilesMutable(0)->rcRates[0] = controlRateProfiles(0)->rcRates[0];
    blackboxParamsOpEnd();
    drainJournal();
    EXPECT_EQ(count, blackboxParamsChangeCount());

    blackboxParamsOpBegin(BBP_SRC_ADJUSTMENT, 3);
    controlRateProfilesMutable(0)->rcRates[0] += 1;
    blackboxParamsOpEnd();
    EXPECT_NE(count, blackboxParamsChangeCount());
    count = blackboxParamsChangeCount();

    blackboxParamsApplied(BBP_LOADER_SETPOINT, 0);
    EXPECT_NE(count, blackboxParamsChangeCount());
    count = blackboxParamsChangeCount();

    // A write that no hook saw: the scan finds it
    controlRateProfilesMutable(1)->rcRates[1] += 1;
    drainJournal();
    for (int i = 0; i < 100; i++) {
        logFrame();
    }
    EXPECT_NE(count, blackboxParamsChangeCount());
    count = blackboxParamsChangeCount();

    blackboxParamsStop();
    blackboxParamsStart();
    EXPECT_NE(count, blackboxParamsChangeCount());
}

// Markers, and a loader outside an operation
TEST(BlackboxParamsJournalTest, MarkersAndLoaderOutsideAnOperation)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader();
    runningLog();
    sim.iteration = 30;
    blackboxParamsMarker(BBP_MARKER_EESAVE, 71230, 0);
    blackboxParamsMarker(BBP_MARKER_EELOAD, 0, 0);
    blackboxParamsMarker(BBP_MARKER_ESCPARAM, 12, 0xdeadbeef);
    mixerConfigMutable()->swash_ring += 1;
    blackboxParamsApplied(BBP_LOADER_MIXER, -1);
    drainJournal();
    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    std::vector<std::string> words;
    for (const JRecord &r : j.records) {
        if (r.type == 'M') {
            words.push_back(r.words[0] + (r.fields.count("us") ? " us=" + r.fields.at("us") : "") +
                (r.fields.count("fnv") ? " n=" + r.fields.at("n") + " fnv=" + r.fields.at("fnv") : ""));
        }
    }
    EXPECT_EQ(std::vector<std::string>({ "eesave us=71230", "eeload", "escparam n=12 fnv=deadbeef" }), words);
    // Outside an operation the loader is its own operation (source l): a write before it is not its write
    const JRecord *u = findRecord(j, 'C', "u");
    ASSERT_NE(nullptr, u);
    EXPECT_EQ("swash_ring", u->items.at(0).key);
    EXPECT_TRUE(u->interval);
    const JRecord &a = j.records.back();
    EXPECT_EQ('A', a.type);
    EXPECT_EQ("mix", a.words.at(0));
    EXPECT_EQ("30.0", a.at);
    EXPECT_LT(u->seq, a.seq);
}


TEST(BlackboxParamsTest, HeaderYieldsWithoutStartingAnotherScan)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    blackboxParamsStart();
    unsigned yields = 0;
    bool complete = false;
    for (int call = 0; call < 10000 && !complete; call++) {
        const unsigned hashBefore = bbpTestHashBytes;
        blackboxParamsHeaderTick();
        EXPECT_LE(bbpTestHashBytes - hashBefore, BBP_SCAN_BYTES);
        blackboxHeaderBudget = 256;
        const unsigned yieldBefore = bbpTestLastHeaderYield;
        complete = blackboxParamsWriteHeader();
        if (bbpTestLastHeaderYield != yieldBefore) {
            yields++;
            EXPECT_EQ(bbpTestLastHeaderYield, bbpTestHeaderCalls);
        }
    }
    EXPECT_TRUE(complete);
    EXPECT_GT(yields, 0u);
}

TEST(BlackboxParamsJournalTest, IdleLoaderVerificationIsBoundedAndInvalidatedByWrites)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    for (int l = 0; l < BBP_LOADER_COUNT; l++) {
        blackboxParamsApplied((bbpLoader_e)l, 0);
    }
    runHeader();
    const unsigned hashBefore = bbpTestHashBytes;
    runningLog();
    EXPECT_EQ(hashBefore, bbpTestHashBytes); // T0 never hashes configuration regions
    drainJournal();
    expectNoErrors(parseJournal(sim.events));

    blackboxParamsStop();
    blackboxParamsApplied(BBP_LOADER_PID, 0);
    sim.events.clear();
    runHeader();
    // A write after the header, while buffers flush, invalidates inferred applications.
    blackboxParamsOpBegin(BBP_SRC_MSP, MSP_SET_PID_TUNING);
    pidProfilesMutable(0)->pid[0].P++;
    blackboxParamsOpEnd();
    runningLog();
    drainJournal();
    EXPECT_EQ(0, countRecords(parseJournal(sim.events), 'A'));
}

TEST(BlackboxParamsJournalTest, ReadOnlyMspCommandsDoNotTouchTheJournal)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader(); runningLog(); drainJournal();
    const uint32_t seq = blackboxParamsSeq();
    for (int cmd : { MSP_BOXNAMES, MSP_BOXIDS, MSP_GET_SERVO_CONFIG, MSP_GET_MIXER_INPUT,
                     MSP_GET_ADJUSTMENT_RANGE, MSP_RPM_FILTER_V2, MSP_MULTIPLE_MSP, 32000 }) {
        EXPECT_FALSE(blackboxParamsMspBegin(cmd)) << cmd;
    }
    EXPECT_EQ(seq, blackboxParamsSeq());
    for (int cmd : { MSP_SET_SERVO_CONFIG, MSP_RESET_CONF, MSP_SET_PID_TUNING, MSP2_SET_SMARTFUEL_CONFIG,
                     MSP2_ESC_SENSOR_TRIAL, MSP2_RX_SERIAL_TRIAL }) {
        EXPECT_TRUE(blackboxParamsMspBegin(cmd)) << cmd;
        blackboxParamsOpEnd();
    }
}

TEST(BlackboxParamsJournalTest, InterruptWritesCannotReenterAnOperation)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    Decoded d = decodeHeader(runHeader());
    runningLog(); drainJournal();
    const uint32_t seq = blackboxParamsSeq();
    bbpTestInInterrupt = true;
    EXPECT_FALSE(blackboxParamsMspBegin(MSP_SET_PID_TUNING));
    blackboxParamsOpBegin(BBP_SRC_MSP, MSP_SET_PID_TUNING);
    pidProfilesMutable(0)->pid[0].P++;
    blackboxParamsApplied(BBP_LOADER_PID, 0);
    blackboxParamsMarker(BBP_MARKER_EESAVE, 123, 0);
    blackboxParamsOpEnd();
    EXPECT_FALSE(blackboxParamsActive());
    EXPECT_EQ(seq, blackboxParamsSeq());
    bbpTestInInterrupt = false;
    for (int i = 0; i < 300; i++) logFrame();
    drainJournal();
    const Journal j = parseJournal(sim.events);
    expectNoErrors(j);
    EXPECT_EQ(1, countRecords(j, 'C', "u"));
    EXPECT_EQ(0, countRecords(j, 'A'));
    std::vector<std::string> errors;
    applyJournal(d, j, errors);
    EXPECT_TRUE(errors.empty());
    expectStateIsLive(d);
}

TEST(BlackboxParamsJournalTest, OffDoesNotHashLoaderRegions)
{
    setupConfig(BLACKBOX_PARAMS_OFF);
    const unsigned before = bbpTestHashBytes;
    for (int l = 0; l < BBP_LOADER_COUNT; l++) blackboxParamsApplied((bbpLoader_e)l, 0);
    EXPECT_EQ(before, bbpTestHashBytes);
}

TEST(BlackboxParamsJournalTest, EventKeepsAtLeastHalfTheFlashBufferFree)
{
    setupConfig(BLACKBOX_PARAMS_FULL);
    runHeader(); runningLog();
    sim.freeSpace = 256;
    for (int i = 0; i < 100; i++) logFrame();
    EXPECT_TRUE(sim.events.empty());
    sim.freeSpace = 512;
    drainJournal();
    ASSERT_FALSE(sim.events.empty());
    for (const auto &event : sim.events) EXPECT_LE(event.size() + 3 + 256, 512u);
    expectNoErrors(parseJournal(sim.events));
}
