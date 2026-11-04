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
#include <string.h>

extern "C" {
#include "platform.h"

#include "common/bitarray.h"

#include "pg/pg.h"
#include "pg/modes.h"

#include "fc/rc_modes.h"

#include "rx/rx.h"
}

#include "gtest/gtest.h"

static timeUs_t nowUs;

extern "C" {
float rcInput[MAX_SUPPORTED_RC_CHANNEL_COUNT];
timeUs_t micros(void) { return nowUs; }
void pinioBoxTaskControl(void) {}
}

class RcModesTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        for (auto &v : rcInput) v = 1500;
        nowUs = 10 * 1000 * 1000;   // well past the sticky-mode boot delay
        macCount = 0;
        analyze();
    }

    // Add a range condition: mode active while aux channel is in [start, end) steps
    int addRange(boxId_e mode, uint8_t aux, int8_t start, int8_t end, uint8_t logic = MODELOGIC_OR)
    {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(macCount);
        mac->modeId = mode;
        mac->auxChannelIndex = aux;
        mac->range.startStep = start;
        mac->range.endStep = end;
        mac->modeLogic = logic;
        mac->linkedTo = 0;
        return macCount++;
    }

    int addLink(boxId_e mode, boxId_e linkedTo)
    {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(macCount);
        mac->modeId = mode;
        mac->linkedTo = linkedTo;
        mac->modeLogic = MODELOGIC_OR;
        return macCount++;
    }

    void analyze() { analyzeModeActivationConditions(); }

    void setAux(int aux, float value) { rcInput[CONTROL_CHANNEL_COUNT + aux] = value; }

    bool update(boxId_e mode)
    {
        updateActivatedModes();
        return IS_RC_MODE_ACTIVE(mode);
    }

    int macCount;
};

TEST_F(RcModesTest, StepConversion)
{
    EXPECT_EQ(1500, STEP_TO_CHANNEL_VALUE(0));
    EXPECT_EQ(1000, STEP_TO_CHANNEL_VALUE(-100));
    EXPECT_EQ(2000, STEP_TO_CHANNEL_VALUE(100));
    EXPECT_EQ(-100, CHANNEL_VALUE_TO_STEP(1000));
    EXPECT_EQ(50, CHANNEL_VALUE_TO_STEP(1750));
}

TEST_F(RcModesTest, NoConditionsNoModes)
{
    setAux(0, 2000);
    updateActivatedModes();
    for (int i = 0; i < CHECKBOX_ITEM_COUNT; i++)
        EXPECT_FALSE(IS_RC_MODE_ACTIVE((boxId_e)i)) << "mode " << i;
}

TEST_F(RcModesTest, RangeIsHalfOpen)
{
    addRange(BOXARM, 0, 50, 100);   // 1750 .. 2000
    analyze();

    setAux(0, 1500);
    EXPECT_FALSE(update(BOXARM));
    setAux(0, 1749);
    EXPECT_FALSE(update(BOXARM));
    setAux(0, 1750);
    EXPECT_TRUE(update(BOXARM));
    setAux(0, 1999);
    EXPECT_TRUE(update(BOXARM));
    setAux(0, 2000);
    EXPECT_FALSE(update(BOXARM));
}

TEST_F(RcModesTest, UnusableRangeNeverActive)
{
    // start must be below end, and both within +-125 steps
    addRange(BOXANGLE, 0, 50, 50);
    addRange(BOXHORIZON, 1, 60, 20);
    analyze();

    setAux(0, 1750);
    setAux(1, 1700);
    updateActivatedModes();
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXANGLE));
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXHORIZON));

    channelRange_t r = { -126, 0 };
    EXPECT_FALSE(isRangeUsable(&r));
    r = { -125, 125 };
    EXPECT_TRUE(isRangeUsable(&r));
}

TEST_F(RcModesTest, ModesAreIndependent)
{
    addRange(BOXARM, 0, 50, 125);
    addRange(BOXANGLE, 1, 50, 125);
    analyze();

    setAux(0, 2000);
    setAux(1, 1000);
    updateActivatedModes();
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXARM));
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXANGLE));

    setAux(1, 2000);
    updateActivatedModes();
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXANGLE));
}

TEST_F(RcModesTest, OrLogic)
{
    addRange(BOXANGLE, 0, 50, 125);
    addRange(BOXANGLE, 1, 50, 125);
    analyze();

    setAux(0, 1000); setAux(1, 1000);
    EXPECT_FALSE(update(BOXANGLE));
    setAux(0, 2000); setAux(1, 1000);
    EXPECT_TRUE(update(BOXANGLE));
    setAux(0, 1000); setAux(1, 2000);
    EXPECT_TRUE(update(BOXANGLE));
    setAux(0, 2000); setAux(1, 2000);
    EXPECT_TRUE(update(BOXANGLE));
}

TEST_F(RcModesTest, AndLogic)
{
    addRange(BOXANGLE, 0, 50, 125, MODELOGIC_AND);
    addRange(BOXANGLE, 1, 50, 125, MODELOGIC_AND);
    analyze();

    setAux(0, 1000); setAux(1, 1000);
    EXPECT_FALSE(update(BOXANGLE));
    setAux(0, 2000); setAux(1, 1000);
    EXPECT_FALSE(update(BOXANGLE));
    setAux(0, 1000); setAux(1, 2000);
    EXPECT_FALSE(update(BOXANGLE));
    setAux(0, 2000); setAux(1, 2000);
    EXPECT_TRUE(update(BOXANGLE));
}

TEST_F(RcModesTest, OrOverridesAnd)
{
    // (aux0 AND aux1) OR aux2
    addRange(BOXANGLE, 0, 50, 125, MODELOGIC_AND);
    addRange(BOXANGLE, 1, 50, 125, MODELOGIC_AND);
    addRange(BOXANGLE, 2, 50, 125, MODELOGIC_OR);
    analyze();

    setAux(0, 2000); setAux(1, 1000); setAux(2, 1000);
    EXPECT_FALSE(update(BOXANGLE));
    setAux(2, 2000);
    EXPECT_TRUE(update(BOXANGLE));
    setAux(0, 1000);
    EXPECT_TRUE(update(BOXANGLE));
    setAux(2, 1000);
    EXPECT_FALSE(update(BOXANGLE));
}

TEST_F(RcModesTest, LinkedMode)
{
    addRange(BOXANGLE, 0, 50, 125);
    addLink(BOXBEEPERON, BOXANGLE);
    analyze();

    EXPECT_TRUE(isModeActivationConditionLinked(BOXBEEPERON));
    EXPECT_FALSE(isModeActivationConditionLinked(BOXANGLE));

    setAux(0, 1000);
    EXPECT_FALSE(update(BOXBEEPERON));
    setAux(0, 2000);
    EXPECT_TRUE(update(BOXBEEPERON));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXANGLE));
}

TEST_F(RcModesTest, LinkedModeFollowsAndLogic)
{
    addRange(BOXANGLE, 0, 50, 125, MODELOGIC_AND);
    addRange(BOXANGLE, 1, 50, 125, MODELOGIC_AND);
    addLink(BOXBEEPERON, BOXANGLE);
    analyze();

    setAux(0, 2000); setAux(1, 1000);
    EXPECT_FALSE(update(BOXBEEPERON));
    setAux(1, 2000);
    EXPECT_TRUE(update(BOXBEEPERON));
}

TEST_F(RcModesTest, ParalyzeIsStickyAndNeedsSwitchOffAfterBoot)
{
    addRange(BOXPARALYZE, 0, 50, 125);
    analyze();

    // Switch already on at boot: ignored during the boot delay...
    nowUs = 1 * 1000 * 1000;
    setAux(0, 2000);
    EXPECT_FALSE(update(BOXPARALYZE));

    // ...and still ignored after it, until the switch has been seen off
    nowUs = 6 * 1000 * 1000;
    EXPECT_FALSE(update(BOXPARALYZE));

    setAux(0, 1000);
    EXPECT_FALSE(update(BOXPARALYZE));

    // Now it activates, and stays on when the switch goes back off
    setAux(0, 2000);
    EXPECT_TRUE(update(BOXPARALYZE));
    setAux(0, 1000);
    EXPECT_TRUE(update(BOXPARALYZE));
    EXPECT_TRUE(update(BOXPARALYZE));
}

TEST_F(RcModesTest, ConditionPresent)
{
    addRange(BOXANGLE, 0, 50, 125);
    addRange(BOXHORIZON, 1, 50, 50);   // unusable
    addLink(BOXBEEPERON, BOXANGLE);

    EXPECT_TRUE(isModeActivationConditionPresent(BOXANGLE));
    EXPECT_FALSE(isModeActivationConditionPresent(BOXHORIZON));
    EXPECT_TRUE(isModeActivationConditionPresent(BOXBEEPERON));
    EXPECT_FALSE(isModeActivationConditionPresent(BOXRESCUE));
}

TEST_F(RcModesTest, RemoveCondition)
{
    addRange(BOXARM, 0, 50, 125);
    addRange(BOXANGLE, 1, 50, 125);
    addRange(BOXANGLE, 2, 50, 125);
    addRange(BOXHORIZON, 3, 50, 125);

    removeModeActivationCondition(BOXANGLE);

    // All conditions for the mode are gone and the rest are compacted
    EXPECT_FALSE(isModeActivationConditionPresent(BOXANGLE));
    EXPECT_EQ(BOXARM, modeActivationConditions(0)->modeId);
    EXPECT_EQ(BOXHORIZON, modeActivationConditions(1)->modeId);
    EXPECT_EQ(3, modeActivationConditions(1)->auxChannelIndex);

    modeActivationCondition_t empty;
    memset(&empty, 0, sizeof(empty));
    for (int i = 2; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++)
        EXPECT_FALSE(isModeActivationConditionConfigured(modeActivationConditions(i), &empty)) << "slot " << i;
}

TEST_F(RcModesTest, RcModeUpdateSetsMask)
{
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    bitArraySet(&mask, BOXFAILSAFE);
    rcModeUpdate(&mask);

    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXFAILSAFE));
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXARM));
}
