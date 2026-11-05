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
 * LED strip configuration in io/ledstrip.c: the CLI "led" string format
 * (with Rotorflight's blink pattern, blink pause and alternate color
 * fields), counting, grid dimensions, colors and mode colors.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
#include "platform.h"

#include "pg/pg.h"
#include "pg/ledstrip.h"

#include "io/ledstrip.h"

extern uint8_t ledGridRows;
extern int8_t highestYValueForNorth;
extern int8_t lowestYValueForSouth;
extern int8_t highestXValueForWest;
extern int8_t lowestXValueForEast;
extern ledCounts_t ledCounts;
}

#include "gtest/gtest.h"

#include "ledstrip_stubs.h"

class LedStripTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        memset(ledStripStatusModeConfigMutable()->ledConfigs, 0, sizeof(ledStripStatusModeConfig()->ledConfigs));
        reevaluateLedConfig();
    }

    static const ledConfig_t *led(int i) { return &ledStripStatusModeConfig()->ledConfigs[i]; }

    static std::string generate(int i)
    {
        char buf[64];
        generateLedConfig((ledConfig_t *)led(i), buf, sizeof(buf));
        return buf;
    }
};

TEST_F(LedStripTest, ParseBasic)
{
    ASSERT_TRUE(parseLedStripConfig(0, "9,9:S:FW:0"));
    EXPECT_EQ(9, ledGetX(led(0)));
    EXPECT_EQ(9, ledGetY(led(0)));
    EXPECT_TRUE(ledGetDirectionBit(led(0), LED_DIRECTION_SOUTH));
    EXPECT_FALSE(ledGetDirectionBit(led(0), LED_DIRECTION_NORTH));
    EXPECT_EQ(LED_FUNCTION_FLIGHT_MODE, ledGetFunction(led(0)));
    EXPECT_TRUE(ledGetOverlayBit(led(0), LED_OVERLAY_WARNING));
    EXPECT_FALSE(ledGetOverlayBit(led(0), LED_OVERLAY_INDICATOR));
    EXPECT_EQ(0, ledGetColor(led(0)));
}

TEST_F(LedStripTest, ParseAllDirectionsAndOverlays)
{
    ASSERT_TRUE(parseLedStripConfig(0, "1,2:NESWUD:CTOBVIWKD:5"));
    for (int dir = 0; dir < LED_DIRECTION_COUNT; dir++)
        EXPECT_TRUE(ledGetDirectionBit(led(0), dir)) << "direction " << dir;
    for (int ol = 0; ol < LED_OVERLAY_COUNT; ol++)
        EXPECT_TRUE(ledGetOverlayBit(led(0), ol)) << "overlay " << ol;
    EXPECT_EQ(LED_FUNCTION_COLOR, ledGetFunction(led(0)));
    EXPECT_EQ(5, ledGetColor(led(0)));
}

TEST_F(LedStripTest, ParseBlinkFields)
{
    // x,y:directions:functions:color:blink pattern:blink pause:alt color
    ASSERT_TRUE(parseLedStripConfig(0, "3,4:N:C:2:43690:3:7"));
    EXPECT_EQ(2, ledGetColor(led(0)));
    EXPECT_EQ(43690, ledGetBlinkPattern(led(0)));
    EXPECT_EQ(3, ledGetBlinkPause(led(0)));
    EXPECT_EQ(7, ledGetAltColor(led(0)));

    // The extra fields are optional
    ASSERT_TRUE(parseLedStripConfig(1, "3,4:N:C:2"));
    EXPECT_EQ(2, ledGetColor(led(1)));
    EXPECT_EQ(0, ledGetBlinkPattern(led(1)));
    EXPECT_EQ(0, ledGetBlinkPause(led(1)));
    EXPECT_EQ(0, ledGetAltColor(led(1)));
}

TEST_F(LedStripTest, ParseFourFieldFormatKeepsColor)
{
    // Classic x,y:directions:functions:color, as in older dumps and presets
    ASSERT_TRUE(parseLedStripConfig(0, "5,6:E:L:12"));
    EXPECT_EQ(12, ledGetColor(led(0)));
    EXPECT_EQ("5,6:E:L:12:0:0:0", generate(0));

    // The functions must be followed by a separator; an empty color is 0
    EXPECT_FALSE(parseLedStripConfig(1, "5,6:E:L"));
    ASSERT_TRUE(parseLedStripConfig(1, "5,6:E:L:"));
    EXPECT_EQ(0, ledGetColor(led(1)));

    // Some of the optional fields given
    ASSERT_TRUE(parseLedStripConfig(2, "5,6:E:L:12:255"));
    EXPECT_EQ(12, ledGetColor(led(2)));
    EXPECT_EQ(255, ledGetBlinkPattern(led(2)));
    EXPECT_EQ(0, ledGetBlinkPause(led(2)));
}

TEST_F(LedStripTest, ParseOutOfRangeColors)
{
    ASSERT_TRUE(parseLedStripConfig(0, "0,0:N:C:16:0:0:16"));
    EXPECT_EQ(0, ledGetColor(led(0)));
    EXPECT_EQ(0, ledGetAltColor(led(0)));
}

TEST_F(LedStripTest, ParseInvalid)
{
    EXPECT_FALSE(parseLedStripConfig(LED_MAX_STRIP_LENGTH, "0,0:N:C:0"));
    EXPECT_FALSE(parseLedStripConfig(0, "0"));
    EXPECT_FALSE(parseLedStripConfig(0, "0,0"));
    EXPECT_FALSE(parseLedStripConfig(0, "0,0:N"));
}

TEST_F(LedStripTest, GenerateRoundTrip)
{
    const char *configs[] = {
        "0,0:N:C:0:0:0:0",
        "15,15:NESWUD:FTW:15:65535:7:3",
        "7,8:SW:AO:4:255:1:2",
        "2,3:U:RI:9:0:0:0",
    };
    for (unsigned i = 0; i < sizeof(configs) / sizeof(configs[0]); i++) {
        ASSERT_TRUE(parseLedStripConfig(i, configs[i]));
        EXPECT_EQ(configs[i], generate(i));
    }
}

TEST_F(LedStripTest, CountsLedsRingsAndScanners)
{
    parseLedStripConfig(0, "0,0:N:C:0");
    parseLedStripConfig(1, "1,0:N:R:0");
    parseLedStripConfig(2, "2,0:N:R:0");
    parseLedStripConfig(3, "3,0:N:CO:0");

    EXPECT_EQ(4, ledCounts.count);
    EXPECT_EQ(2, ledCounts.ring);
    EXPECT_EQ(1, ledCounts.larson);
}

TEST_F(LedStripTest, CountStopsAtFirstEmptySlot)
{
    parseLedStripConfig(0, "0,0:N:C:0");
    parseLedStripConfig(2, "2,0:N:C:0");
    EXPECT_EQ(1, ledCounts.count);
}

TEST_F(LedStripTest, GridWithCenter)
{
    // 3x3 grid around a center LED
    parseLedStripConfig(0, "1,1:N:C:0");
    parseLedStripConfig(1, "0,0:N:C:0");
    parseLedStripConfig(2, "2,2:N:C:0");

    EXPECT_EQ(3, ledGridRows);
    EXPECT_EQ(0, highestXValueForWest);
    EXPECT_EQ(2, lowestXValueForEast);
    EXPECT_EQ(0, highestYValueForNorth);
    EXPECT_EQ(2, lowestYValueForSouth);
}

TEST_F(LedStripTest, GridWithoutCenter)
{
    // 2x2 grid: no row/column is shared between the halves
    parseLedStripConfig(0, "0,0:N:C:0");
    parseLedStripConfig(1, "1,1:N:C:0");

    EXPECT_EQ(2, ledGridRows);
    EXPECT_EQ(0, highestXValueForWest);
    EXPECT_EQ(1, lowestXValueForEast);
    EXPECT_EQ(0, highestYValueForNorth);
    EXPECT_EQ(1, lowestYValueForSouth);
}

TEST_F(LedStripTest, ParseColor)
{
    ASSERT_TRUE(parseColor(0, "359,255,255"));
    const hsvColor_t *c = &ledStripStatusModeConfig()->colors[0];
    EXPECT_EQ(359, c->h);
    EXPECT_EQ(255, c->s);
    EXPECT_EQ(255, c->v);

    ASSERT_TRUE(parseColor(1, "120,0,30"));
    EXPECT_EQ(120, ledStripStatusModeConfig()->colors[1].h);
    EXPECT_EQ(30, ledStripStatusModeConfig()->colors[1].v);
}

TEST_F(LedStripTest, ParseColorInvalidClears)
{
    parseColor(2, "100,100,100");

    EXPECT_FALSE(parseColor(2, "360,0,0"));      // hue over 359
    EXPECT_EQ(0, ledStripStatusModeConfig()->colors[2].h);

    parseColor(2, "100,100,100");
    EXPECT_FALSE(parseColor(2, "100,100"));      // missing value
    EXPECT_EQ(0, ledStripStatusModeConfig()->colors[2].s);

    EXPECT_FALSE(parseColor(2, "0,256,0"));
}

TEST_F(LedStripTest, SetModeColor)
{
    EXPECT_TRUE(setModeColor(LED_MODE_ANGLE, LED_DIRECTION_EAST, 7));
    EXPECT_EQ(7, ledStripStatusModeConfig()->modeColors[LED_MODE_ANGLE].color[LED_DIRECTION_EAST]);

    EXPECT_TRUE(setModeColor(LED_SPECIAL, 0, 3));
    EXPECT_EQ(3, ledStripStatusModeConfig()->specialColors.color[0]);

    EXPECT_TRUE(setModeColor(LED_AUX_CHANNEL, 0, 5));
    EXPECT_EQ(5, ledStripStatusModeConfig()->ledstrip_aux_channel);

    EXPECT_FALSE(setModeColor(LED_MODE_ANGLE, LED_DIRECTION_COUNT, 1));
    EXPECT_FALSE(setModeColor(LED_SPECIAL, LED_SPECIAL_COLOR_COUNT, 1));
    EXPECT_FALSE(setModeColor(LED_AUX_CHANNEL, 1, 1));
    EXPECT_FALSE(setModeColor(LED_MODE_ANGLE, 0, LED_CONFIGURABLE_COLOR_COUNT));
}
