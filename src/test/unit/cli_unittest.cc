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

/*
 * cliSet() against a value table of test-only settings, stored in a
 * parameter group reserved for testing. cli/settings.c is not linked;
 * the table below replaces it.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

extern "C" {
#include "platform.h"

#include "cli/cli.h"
#include "cli/settings.h"

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

void cliSet(const char *cmdName, char *cmdline);
int cliGetSettingIndex(char *name, uint8_t length);
void *cliGetValuePointer(const clivalue_t *value);

typedef struct {
    int8_t array[3];
    uint8_t str[17];
    uint8_t wos[17];
} unitTestData_t;

PG_DECLARE(unitTestData_t, unitTestData);
PG_REGISTER(unitTestData_t, unitTestData, PG_RESERVED_FOR_TESTING_1, 0);

const clivalue_t valueTable[] = {
    { "array_unit_test", VAR_INT8  | MODE_ARRAY  | MASTER_VALUE, .config.array.length = 3,      PG_RESERVED_FOR_TESTING_1, offsetof(unitTestData_t, array) },
    { "str_unit_test",   VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config.string = { 0, 16, 0 }, PG_RESERVED_FOR_TESTING_1, offsetof(unitTestData_t, str) },
    { "wos_unit_test",   VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config.string = { 0, 16, STRING_FLAGS_WRITEONCE }, PG_RESERVED_FOR_TESTING_1, offsetof(unitTestData_t, wos) },
};
const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
const lookupTableEntry_t lookupTables[] = {};
}

#include "gtest/gtest.h"

#include "cli_stubs.h"

TEST(CLIUnittest, TestCliSetArray)
{
    char *str = (char *)"array_unit_test    =   123,  -3  , 1";
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 15);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];

    int8_t *data = (int8_t *)cliGetValuePointer(&val);

    EXPECT_EQ(123, data[0]);
    EXPECT_EQ( -3, data[1]);
    EXPECT_EQ(  1, data[2]);

    // ...stored in the parameter group
    EXPECT_EQ(123, unitTestData()->array[0]);
}

TEST(CLIUnittest, TestCliSetStringNoFlags)
{
    memset(unitTestDataMutable(), 0, sizeof(unitTestData_t));

    char *str = (char *)"str_unit_test    =   SAMPLE";
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];

    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);
}

TEST(CLIUnittest, TestCliSetStringWriteOnce)
{
    memset(unitTestDataMutable(), 0, sizeof(unitTestData_t));

    char *str1 = (char *)"wos_unit_test    =   SAMPLE";
    char *str2 = (char *)"wos_unit_test    =   ELPMAS";
    cliSet("", str1);

    const uint16_t index = cliGetSettingIndex(str1, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];

    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str2);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str1);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

}

