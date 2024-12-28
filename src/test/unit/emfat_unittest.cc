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

#include "msc/emfat.h"
#include "msc/emfat_file.h"

#include "gtest/gtest.h"

extern "C" {
// C functions under test.
uint8_t sfn_checksum(const char* short_name, const char* extension);
void emfat_init_sfn(emfat_entry_t *entry);
bool emfat_init_entries(emfat_entry_t *entries);
void fill_dir_sector(emfat_t *emfat, uint8_t *data, emfat_entry_t *entry, uint32_t rel_sect);
}

TEST(SfnChecksum, SpotTest)
{
    EXPECT_EQ(sfn_checksum("RTFL_1~1", "BBL"), 0x6F);
}

TEST(EmfatInitSfn, ShortFileNameNoDot)
{
    emfat_entry_t entry = {
        .name = "abcd",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCD    ", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "   ", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 1);
}

TEST(EmfatInitSfn, ShortFileNameWithDot)
{
    emfat_entry_t entry = {
        .name = "abcd.xyz",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCD    ", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "XYZ", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 1);
}

TEST(EmfatInitSfn, ShortDirName)
{
    emfat_entry_t entry = {
        .name = "abcd.xy",
        .attr = ATTR_DIR,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCD.XY ", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "   ", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 1);
}

TEST(EmfatInitSfn, LongFileNameNoDot)
{
    emfat_entry_t entry = {
        .name = "abcdefghi",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCDEFGH", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "   ", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 2);
}

TEST(EmfatInitSfn, LongFileNameWithDot)
{
    emfat_entry_t entry = {
        .name = "abcdefghi.txt",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCDEFGH", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "TXT", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 2);
}

TEST(EmfatInitSfn, LongFileNameEntry1)
{
    emfat_entry_t entry = {
        .name = "1234567890123",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "12345678", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "   ", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 2);
}

TEST(EmfatInitSfn, LongFileNameEntry2)
{
    emfat_entry_t entry = {
        .name = "1234567890123456789012.456",
        .attr = 0,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "12345678", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "456", 3) == 0);
    EXPECT_EQ(entry.priv.num_entry, 3);
}

TEST(EmfatInitSfn, LongDirName)
{
    emfat_entry_t entry = {
        .name = "abcdef.xyz",
        .attr = ATTR_DIR,
    };
    emfat_init_sfn(&entry);
    EXPECT_TRUE(memcmp(entry.priv.short_name, "ABCDEF.X", 8) == 0);
    EXPECT_TRUE(memcmp(entry.priv.extension, "   ", 3) == 0);
}

class EmfatTestBase : public ::testing::Test
{
  public:
    void SetUp() override
    {
        ASSERT_TRUE(emfat_init(&emfat_, "RTFL       ", entries_));
    }

    emfat_entry_t entries_[10] = {
    // name            dir    attr         lvl offset  size max_size user  time       read   write
    { "",              true,  0,           0,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "autorun.inf",   false, ATTR_HIDDEN, 1,  0,      1,   1,       1,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "icon.ico",      false, ATTR_HIDDEN, 1,  0,      1,   1,       2,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "readme.txt",    false, 0,           1,  0,      1,   1,       3,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "RTFL_ALL.BBL",  0,     0,           1,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "PADDING.TXT",   0,     ATTR_HIDDEN, 1,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "LongFileName1", 0,     0,           1,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "LongDirectory", 1,     0,           1,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { "LongFileN.exe", 0,     0,           2,  0,      0,   0,       0,    {0, 0, 0}, NULL,  NULL, { 0 } },
    { NULL, }
    };

    emfat_t emfat_ = {};
};

TEST_F(EmfatTestBase, SFN)
{
    EXPECT_TRUE(memcmp(entries_[1].priv.short_name, "AUTORUN ", 8) == 0);
    EXPECT_TRUE(memcmp(entries_[1].priv.extension, "INF", 3) == 0);

    EXPECT_TRUE(memcmp(entries_[6].priv.short_name, "LONGFILE", 8) == 0);
    EXPECT_TRUE(memcmp(entries_[6].priv.extension, "   ", 3) == 0);

    EXPECT_TRUE(memcmp(entries_[7].priv.short_name, "LONGDIRE", 8) == 0);
    EXPECT_TRUE(memcmp(entries_[7].priv.extension, "   ", 3) == 0);

    EXPECT_TRUE(memcmp(entries_[8].priv.short_name, "LONGFILE", 8) == 0);
    EXPECT_TRUE(memcmp(entries_[8].priv.extension, "EXE", 3) == 0);
}

struct dir_entry
{
    uint8_t name[11];
    uint8_t attr;
    uint8_t reserved[10];
    uint16_t time;
    uint16_t date;
    uint16_t first_clust;
    uint32_t size;
};

static_assert(sizeof(dir_entry) == 32, "dir_entry size");


TEST_F(EmfatTestBase, DirEntries)
{
    uint8_t data[512] = {};
    dir_entry *de = (dir_entry *)data;

    fill_dir_sector(&emfat_, data, &entries_[0], 0);
    EXPECT_EQ(de[0].name[0], 'R');  // volume label (RTFL)
    EXPECT_EQ(de[1].name[0], 'A');  // autorun.inf
    EXPECT_EQ(de[2].name[0], 'I');  // icon.ico

    EXPECT_EQ(de[6].name[0], 0x41); // LFN entry of LongFileName1
    EXPECT_EQ(de[6].name[1], 'L');
    EXPECT_EQ(de[6].name[3], 'o');  // (unicode)
    EXPECT_EQ(de[7].name[0], 'L');  // SFN entry of LongFileName1
    EXPECT_EQ(de[7].name[1], 'O');
    EXPECT_EQ(de[8].name[0], 0x41); // LFN entry of LongDirectory
    EXPECT_EQ(de[8].name[1], 'L');
    EXPECT_EQ(de[8].name[3], 'o');  // (unicode)
    EXPECT_EQ(de[9].name[0], 'L');  // SFN entry of LongDirectory
    EXPECT_EQ(de[9].name[1], 'O');

    fill_dir_sector(&emfat_, data, &entries_[0], 1);
    EXPECT_EQ(de[0].name[0], 0);
    EXPECT_EQ(de[1].name[0], 0);

    fill_dir_sector(&emfat_, data, &entries_[7], 0);
    EXPECT_EQ(de[0].name[0], '.');  // .
    EXPECT_EQ(de[1].name[0], '.');  // ..
    EXPECT_EQ(de[2].name[0], 0x41); // LFN entry of LongFileN.exe
    EXPECT_EQ(de[2].name[1], 'L');
    EXPECT_EQ(de[2].name[3], 'o');
    EXPECT_EQ(de[3].name[0], 'L');  // SFN entry of LongFileN.exe
}

TEST(Emfat, LFNAtSectorBoundary)
{
    std::unique_ptr<emfat_entry_t[]> entries =
        std::make_unique<emfat_entry_t[]>(16);
    std::unique_ptr<std::string[]> filenames =
        std::make_unique<std::string[]>(16);

    entries[0] = {
        .name = "",
        .dir = true,
        .level = 0,
    };

    // Fill trivial emfat_entries 1 - 11
    // These will fill dir_entries 1 - 11.
    for (int i = 1; i < 13; ++i) {
        filenames[i] = std::to_string(i);
        entries[i] = {
            .name = filenames[i].c_str(),
            .dir = false,
            .level = 1,
        };
    }

    // Fill long filename emfat_entry 12
    // Filename is exact 13 char. This emfat entry occupies dir_entry 12 and 13.
    entries[12] = {
        .name = "LongFileName1",
        .dir = false,
        .level = 1,
    };
    EXPECT_EQ(strlen(entries[12].name), 13);

    // Fill very long filename emfat_entry 13
    // Filename is more than 26 char. This emfat entry occupies dir_entry 14,
    // 15, 16, where dir_entry 16 is located in the next sector.
    entries[13] = {
        .name = "ExtraExtraLongLongFileName.txt",
        .dir = false,
        .level = 1,
    };
    EXPECT_GT(strlen(entries[13].name), 26);

    entries[14] = {
        NULL,
    };

    emfat_t emfat = {};
    ASSERT_TRUE(emfat_init(&emfat, "RTFL       ", entries.get()));

    uint8_t data[512] = {};
    dir_entry *de = (dir_entry *)data;

    fill_dir_sector(&emfat, data, &entries[0], 0);
    EXPECT_EQ(de[12].name[0], 0x41); // (final) LFN entry of "LongFileName.txt"
    EXPECT_EQ(de[12].name[1], 'L');
    EXPECT_EQ(de[13].name[0], 'L');  // SFN entry of "LongFileName.txt"

    EXPECT_EQ(de[14].name[0], 0x43); // 3rd (final) LFN entry of Extra...
    EXPECT_EQ(de[15].name[0], 0x02); // 2nd LFN entry

    fill_dir_sector(&emfat, data, &entries[0], 1);
    EXPECT_EQ(de[0].name[0], 0x01); // 1st LFN entry
    EXPECT_EQ(de[1].name[0], 'E');  // SFN entry
    EXPECT_EQ(de[2].name[0], 0);    // NULL
}
