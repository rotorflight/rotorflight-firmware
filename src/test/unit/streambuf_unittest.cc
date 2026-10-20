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

// Writes past an sbuf's end are dropped instead of overwriting what follows
// the buffer, and the writer can tell that it ran out of room. MSP replies
// rely on this: MSP_ADJUSTMENT_RANGES (588 bytes) used to overrun the 320-byte
// telemetry reply buffer.

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "common/streambuf.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

const uint8_t GUARD = 0xA5;

// An 8-byte buffer followed by guard bytes that no write may touch.
struct Guarded {
    uint8_t data[8];
    uint8_t guard[8];
    sbuf_t buf;

    Guarded()
    {
        memset(data, 0, sizeof(data));
        memset(guard, GUARD, sizeof(guard));
        sbufInit(&buf, data, data + sizeof(data));
    }

    bool guardIntact() const
    {
        for (unsigned i = 0; i < sizeof(guard); i++) {
            if (guard[i] != GUARD) {
                return false;
            }
        }
        return true;
    }
};

}

TEST(StreamBufTest, WritesThatFitAreUnchanged)
{
    Guarded g;
    sbufWriteU8(&g.buf, 1);
    sbufWriteU16(&g.buf, 0x0302);
    sbufWriteU32(&g.buf, 0x07060504);
    sbufWriteU8(&g.buf, 8);

    EXPECT_FALSE(sbufWriteOverflowed(&g.buf));
    EXPECT_EQ(0, sbufBytesRemaining(&g.buf));
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(i + 1, g.data[i]);
    }
    EXPECT_TRUE(g.guardIntact());
}

TEST(StreamBufTest, BytesPastTheEndAreDroppedAndFlagged)
{
    Guarded g;
    for (int i = 0; i < 12; i++) {
        sbufWriteU8(&g.buf, i + 1);
    }

    EXPECT_TRUE(g.guardIntact());
    EXPECT_TRUE(sbufWriteOverflowed(&g.buf));
    EXPECT_EQ(0, sbufBytesRemaining(&g.buf));
    EXPECT_EQ(8, g.data[7]);
}

TEST(StreamBufTest, MultiByteWritesStopAtTheEnd)
{
    Guarded g;
    sbufWriteU32(&g.buf, 0x04030201);
    sbufWriteU16(&g.buf, 0x0605);
    sbufWriteU32(&g.buf, 0x0A090807);   // only 0x07 0x08 fit

    EXPECT_TRUE(g.guardIntact());
    EXPECT_TRUE(sbufWriteOverflowed(&g.buf));
    EXPECT_EQ(7, g.data[6]);
    EXPECT_EQ(8, g.data[7]);
}

TEST(StreamBufTest, WriteDataAndFillCopyOnlyWhatFits)
{
    Guarded g;
    const uint8_t src[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    sbufWriteData(&g.buf, src, sizeof(src));

    EXPECT_TRUE(g.guardIntact());
    EXPECT_EQ(0, memcmp(g.data, src, 8));
    EXPECT_EQ(0, sbufBytesRemaining(&g.buf));
    EXPECT_TRUE(sbufWriteOverflowed(&g.buf));

    Guarded f;
    sbufWriteU8(&f.buf, 0);
    sbufFill(&f.buf, 0xEE, 20);
    EXPECT_TRUE(f.guardIntact());
    EXPECT_EQ(0xEE, f.data[7]);
    EXPECT_TRUE(sbufWriteOverflowed(&f.buf));

    // Nothing more is copied once the buffer is full.
    sbufWriteData(&f.buf, src, sizeof(src));
    EXPECT_TRUE(f.guardIntact());
}

TEST(StreamBufTest, StringWritesStopAtTheEnd)
{
    Guarded g;
    sbufWriteString(&g.buf, "0123456789AB");

    EXPECT_TRUE(g.guardIntact());
    EXPECT_EQ(0, sbufBytesRemaining(&g.buf));
    EXPECT_EQ(0, memcmp(g.data, "01234567", 8));

    Guarded z;
    sbufWriteStringWithZeroTerminator(&z.buf, "0123456789");
    EXPECT_TRUE(z.guardIntact());
    EXPECT_EQ(0, sbufBytesRemaining(&z.buf));
    EXPECT_TRUE(sbufWriteOverflowed(&z.buf));
}

TEST(StreamBufTest, ReaderNeverReadsPastTheBuffer)
{
    Guarded g;
    for (int i = 0; i < 12; i++) {
        sbufWriteU8(&g.buf, i + 1);
    }
    sbufSwitchToReader(&g.buf, g.data);

    EXPECT_EQ(8, sbufBytesRemaining(&g.buf));
    EXPECT_EQ(g.data + 8, g.buf.end);
    EXPECT_FALSE(sbufWriteOverflowed(&g.buf));
}

TEST(StreamBufTest, ReaderAfterShortWriteSeesOnlyTheWrittenBytes)
{
    Guarded g;
    sbufWriteU16(&g.buf, 0x0201);
    sbufSwitchToReader(&g.buf, g.data);

    EXPECT_EQ(2, sbufBytesRemaining(&g.buf));
    EXPECT_EQ(1, sbufReadU8(&g.buf));
    EXPECT_EQ(2, sbufReadU8(&g.buf));
}

TEST(StreamBufTest, InitClearsTheOverflow)
{
    Guarded g;
    sbufFill(&g.buf, 0, 9);
    EXPECT_TRUE(sbufWriteOverflowed(&g.buf));

    sbufInit(&g.buf, g.data, g.data + sizeof(g.data));
    EXPECT_FALSE(sbufWriteOverflowed(&g.buf));
    EXPECT_EQ(8, sbufBytesRemaining(&g.buf));
}
