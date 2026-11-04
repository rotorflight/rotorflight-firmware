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
 * Blackbox frame intervals set up by blackboxInit(): P frames every
 * blackbox_rate_denom loops, I frames every 32 ms (at most 64 P frames)
 * on a multiple of the P interval, S frames every 5 s and GPS home frames
 * every 10 s.
 */

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"

#include "blackbox/blackbox.h"

#include "pg/pg.h"
#include "pg/blackbox.h"

#include "sensors/gyro.h"
}

#include "gtest/gtest.h"

#include "blackbox_stubs.h"

struct IntervalCase {
    uint16_t rateHz;
    uint16_t denom;
    uint32_t p, i, s, g;
};

class BlackboxIntervalTest : public ::testing::TestWithParam<IntervalCase> {};

TEST_P(BlackboxIntervalTest, Intervals)
{
    const IntervalCase &c = GetParam();
    pgResetAll();
    gyro.targetRateHz = c.rateHz;
    blackboxConfigMutable()->denom = c.denom;

    blackboxInit();

    EXPECT_EQ(c.p, blackboxPInterval);
    EXPECT_EQ(c.i, blackboxIInterval);
    EXPECT_EQ(c.s, blackboxSInterval);
    EXPECT_EQ(c.g, blackboxGInterval);

    // I frames always fall on a P frame
    EXPECT_EQ(0u, blackboxIInterval % blackboxPInterval);
}

INSTANTIATE_TEST_SUITE_P(Rates, BlackboxIntervalTest, ::testing::Values(
    //            rate denom   P    I      S     G
    IntervalCase{  500,   1,   1,  16,  2500,  312 },
    IntervalCase{ 1000,   1,   1,  32,  5000,  312 },
    IntervalCase{ 2000,   1,   1,  64, 10000,  312 },
    IntervalCase{ 4000,   2,   2, 128, 10000,  312 },
    IntervalCase{ 8000,   1,   1,  64, 40000, 1250 },   // capped at 64 P frames
    IntervalCase{ 8000,   8,   8, 256,  5000,  312 },
    IntervalCase{ 1000,  64,  64,  64,    78,  156 },   // slower than 32 ms: I every P
    IntervalCase{ 1000,   0,   1,  32,  5000,  312 },   // denom clamped to 1
    IntervalCase{ 8000, 9000, 8000, 8000,   5,   10 }   // and to 8000
));

TEST(BlackboxTest, ValidateDevice)
{
    pgResetAll();
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
    blackboxValidateConfig();
    EXPECT_EQ(BLACKBOX_DEVICE_SERIAL, blackboxConfig()->device);

    // Not built into the unit test target: falls back to none
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_SDCARD;
    blackboxValidateConfig();
    EXPECT_EQ(BLACKBOX_DEVICE_NONE, blackboxConfig()->device);

    blackboxConfigMutable()->device = 42;
    blackboxValidateConfig();
    EXPECT_EQ(BLACKBOX_DEVICE_NONE, blackboxConfig()->device);
}

TEST(BlackboxTest, MayEditConfigWhenNotLogging)
{
    pgResetAll();
    gyro.targetRateHz = 1000;
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    blackboxInit();
    EXPECT_TRUE(blackboxMayEditConfig());

    blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
    blackboxInit();
    EXPECT_TRUE(blackboxMayEditConfig());
}
