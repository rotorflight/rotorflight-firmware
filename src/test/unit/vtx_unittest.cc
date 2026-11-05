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

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"

#include "pg/pg.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "io/vtx.h"

vtxSettingsConfig_t vtxGetSettings(void);
}

#include "gtest/gtest.h"

static bool pitModeSwitch;
static bool failsafeActive;

extern "C" {
uint8_t armingFlags;

bool IS_RC_MODE_ACTIVE(boxId_e box) { return box == BOXVTXPITMODE && pitModeSwitch; }
bool failsafeIsActive(void) { return failsafeActive; }
void saveConfigAndNotify(void) {}
void vtxControlInputPoll(void) {}
uint8_t cliMode;
}

class VtxTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        armingFlags = 0;
        pitModeSwitch = false;
        failsafeActive = false;

        vtxSettingsConfigMutable()->band = 2;
        vtxSettingsConfigMutable()->channel = 3;
        vtxSettingsConfigMutable()->freq = 5800;
        vtxSettingsConfigMutable()->power = 4;
        vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_OFF;
    }
};

TEST_F(VtxTest, PlainSettings)
{
    const vtxSettingsConfig_t s = vtxGetSettings();
    EXPECT_EQ(2, s.band);
    EXPECT_EQ(3, s.channel);
    EXPECT_EQ(5800, s.freq);
    EXPECT_EQ(4, s.power);
}

TEST_F(VtxTest, PitModeFrequency)
{
    vtxSettingsConfigMutable()->pitModeFreq = 5300;
    EXPECT_EQ(5800, vtxGetSettings().freq);

    // The pit mode switch moves to the pit frequency at lowest power
    pitModeSwitch = true;
    const vtxSettingsConfig_t s = vtxGetSettings();
    EXPECT_EQ(5300, s.freq);
    EXPECT_EQ(0, s.band);
    EXPECT_EQ(VTX_TABLE_DEFAULT_POWER, s.power);
}

TEST_F(VtxTest, PitModeWithoutPitFrequency)
{
    vtxSettingsConfigMutable()->pitModeFreq = 0;
    pitModeSwitch = true;
    EXPECT_EQ(5800, vtxGetSettings().freq);
    EXPECT_EQ(2, vtxGetSettings().band);
}

TEST_F(VtxTest, LowPowerWhenDisarmedAlways)
{
    vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_ALWAYS;
    EXPECT_EQ(VTX_TABLE_DEFAULT_POWER, vtxGetSettings().power);

    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
    EXPECT_EQ(4, vtxGetSettings().power);

    DISABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(VTX_TABLE_DEFAULT_POWER, vtxGetSettings().power);
}

TEST_F(VtxTest, LowPowerUntilFirstArm)
{
    vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM;
    EXPECT_EQ(VTX_TABLE_DEFAULT_POWER, vtxGetSettings().power);

    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
    EXPECT_EQ(4, vtxGetSettings().power);

    // Stays at full power after the first disarm
    DISABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(4, vtxGetSettings().power);
}

TEST_F(VtxTest, NoLowPowerInFailsafe)
{
    // Disarmed by failsafe: keep transmitting at full power
    vtxSettingsConfigMutable()->lowPowerDisarm = VTX_LOW_POWER_DISARM_ALWAYS;
    failsafeActive = true;
    EXPECT_EQ(4, vtxGetSettings().power);
}
