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

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "config/config.h"
#include "drivers/time.h"
#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/governor.h"
#include "flight/mixer.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
}

namespace {
struct {
    float throttle;
    float yaw;
    bool throttleOff;
    bool bypass;
    timeMs_t now;
} input;
}

// Only external inputs are stubbed. Governor state, TTA, filters and slew
// limiting are all supplied by the production C implementation.
extern "C" {
uint8_t armingFlags;
pidProfile_t *currentPidProfile;
gyro_t gyro;

float getThrottle(void) { return input.throttle; }
bool isThrottleOff(void) { return input.throttleOff; }
float getRcDeflection(int) { return 0; }
float mixerGetInput(uint8_t index) { return index == MIXER_IN_STABILIZED_YAW ? input.yaw : 0; }
float getCyclicDeflection(void) { return 0; }
bool IS_RC_MODE_ACTIVE(boxId_e boxId) { return boxId == BOXGOVBYPASS && input.bypass; }
uint8_t getMotorCount(void) { return 1; }
float getMainGearRatio(void) { return 0.1f; }
float getMotorRawRPMf(uint8_t) { return 20000; }
bool isMotorFastRpmSourceActive(uint8_t) { return true; }
timeMs_t millis(void) { return input.now; }
float pidGetDT(void) { return 0.001f; }
void setArmingDisabled(armingDisableFlags_e) {}
uint8_t getBatteryVoltageSource(void) { return VOLTAGE_METER_ADC; }
uint8_t getBatteryCellCount(void) { return 6; }
const voltageMeter_t *getBatteryVoltageMeter(void)
{
    static const voltageMeter_t meter = { .sample = 22200 };
    return &meter;
}
}

class GovernorDirectTest : public ::testing::Test {
protected:
    pidProfile_t profile;

    void SetUp() override
    {
        input = {};
        input.throttleOff = true;
        input.now = 1000;
        armingFlags = 0;
        gyro = {};
        gyro.targetRateHz = 1000;
        gyro.targetLooptime = 1000;
        pgResetAll();
        profile = {};
        currentPidProfile = &profile;

        auto *config = governorConfigMutable();
        config->gov_mode = GOV_MODE_DIRECT;
        config->gov_throttle_type = GOV_THROTTLE_NORMAL;
        config->gov_handover_throttle = 25;
        config->gov_idle_throttle = 0;
        config->gov_auto_throttle = 0;
        config->gov_startup_time = 0;
        config->gov_spoolup_time = 0;
        config->gov_tracking_time = 0;
        config->gov_recovery_time = 0;
        config->gov_spooldown_time = 0;
        config->gov_tta_filter = 0;
        mixerConfigMutable()->main_rotor_dir = DIR_CW;
        profile.governor.headspeed = 2000;
        profile.governor.min_throttle = 10;
        profile.governor.max_throttle = 100;
        profile.governor.tta_gain = 125;
        profile.governor.tta_limit = 50;
    }

    void Update()
    {
        ++input.now;
        governorUpdate();
    }

    void InitDisarmed()
    {
        armingFlags = 0;
        governorInit(&profile);
        // Init alone does not clear static throttle output and slew history.
        Update();
        ASSERT_EQ(GOV_STATE_THROTTLE_OFF, getGovernorState());
        ASSERT_FLOAT_EQ(0, getGovernorOutput());
    }

    void EnterActive(float throttle = 0.30f)
    {
        ASSERT_NO_FATAL_FAILURE(InitDisarmed());
        input.throttle = throttle;
        input.throttleOff = false;
        input.yaw = 0;
        armingFlags = ARMED;
        for (int i = 0; i < 16 && getGovernorState() != GOV_STATE_ACTIVE; ++i) {
            Update();
        }
        ASSERT_EQ(GOV_STATE_ACTIVE, getGovernorState());
        // TTA is calculated before the state transition: settle in ACTIVE.
        Update();
    }

    void ExpectActiveOutput(float output, float assist)
    {
        ASSERT_EQ(GOV_STATE_ACTIVE, getGovernorState());
        EXPECT_NEAR(assist, getTTAIncrease(), 1e-6f);
        EXPECT_NEAR(output, getGovernorOutput(), 1e-6f);
    }
};

TEST_F(GovernorDirectTest, EnabledZeroAssistPreservesThrottle)
{
    ASSERT_NO_FATAL_FAILURE(EnterActive());
    ExpectActiveOutput(0.30f, 0);
}

TEST_F(GovernorDirectTest, DisabledAssistPreservesThrottle)
{
    for (bool disableGain : {true, false}) {
        SCOPED_TRACE(disableGain ? "zero gain" : "zero limit");
        profile.governor.tta_gain = disableGain ? 0 : 125;
        profile.governor.tta_limit = disableGain ? 50 : 0;
        ASSERT_NO_FATAL_FAILURE(EnterActive());
        input.yaw = 0.10f;
        Update();
        ExpectActiveOutput(0.30f, 0);
    }
}

TEST_F(GovernorDirectTest, PositiveAssistIsProportional)
{
    ASSERT_NO_FATAL_FAILURE(EnterActive());
    input.yaw = 0.10f;
    Update();
    ExpectActiveOutput(0.33f, 0.10f);
}

TEST_F(GovernorDirectTest, AssistDirectionIsRespected)
{
    for (const auto direction : {DIR_CW, DIR_CCW}) {
        SCOPED_TRACE(direction);
        mixerConfigMutable()->main_rotor_dir = direction;
        ASSERT_NO_FATAL_FAILURE(EnterActive());
        input.yaw = direction == DIR_CW ? 0.10f : -0.10f;
        Update();
        ExpectActiveOutput(0.33f, 0.10f);
        input.yaw = -input.yaw;
        Update();
        ExpectActiveOutput(0.30f, 0);
    }
}

TEST_F(GovernorDirectTest, AssistRespectsConfiguredLimit)
{
    profile.governor.tta_limit = 20;
    ASSERT_NO_FATAL_FAILURE(EnterActive());
    input.yaw = 0.50f;
    Update();
    ExpectActiveOutput(0.36f, 0.20f);
}

TEST_F(GovernorDirectTest, MinimumIsAppliedBeforeAssist)
{
    profile.governor.min_throttle = 40;
    ASSERT_NO_FATAL_FAILURE(EnterActive(0.50f));
    input.throttle = 0.30f;
    input.yaw = 0.10f;
    Update();
    ExpectActiveOutput(0.44f, 0.10f);
}

TEST_F(GovernorDirectTest, OutputRespectsMaximum)
{
    // Saturated outputs check protection; the unsaturated tests above catch
    // the doubled baseline, which can otherwise be hidden by this ceiling.
    const struct {
        uint8_t maximum;
        float yaw;
        float output;
    } cases[] = {{80, 0.10f, 0.80f}, {100, 0.50f, 1.00f}};

    for (const auto &test : cases) {
        SCOPED_TRACE(test.maximum);
        profile.governor.max_throttle = test.maximum;
        ASSERT_NO_FATAL_FAILURE(EnterActive(0.75f));
        input.yaw = test.yaw;
        Update();
        ExpectActiveOutput(test.output, test.yaw);
    }
}

TEST_F(GovernorDirectTest, SlewIsAppliedBeforeAssist)
{
    governorConfigMutable()->gov_tracking_time = 1;
    ASSERT_NO_FATAL_FAILURE(EnterActive());
    input.yaw = 0.10f;
    Update();
    ExpectActiveOutput(0.330f, 0.10f);

    // At 1 kHz and tracking time 1, the baseline moves 0.01 per update.
    input.throttle = 0.50f;
    Update();
    ExpectActiveOutput(0.341f, 0.10f);
    Update();
    ExpectActiveOutput(0.352f, 0.10f);
    input.throttle = 0.30f;
    Update();
    ExpectActiveOutput(0.341f, 0.10f);
    Update();
    ExpectActiveOutput(0.330f, 0.10f);
}

TEST_F(GovernorDirectTest, AssistDoesNotAccumulateIntoSlew)
{
    governorConfigMutable()->gov_tracking_time = 1;
    ASSERT_NO_FATAL_FAILURE(EnterActive());
    ExpectActiveOutput(0.30f, 0);
    input.yaw = 0.20f;
    for (int i = 0; i < 5; ++i) {
        SCOPED_TRACE(i);
        Update();
        ExpectActiveOutput(0.36f, 0.20f);
    }
    input.yaw = 0;
    Update();
    ExpectActiveOutput(0.30f, 0);
}

TEST_F(GovernorDirectTest, DisarmAndThrottleOffRemainEffective)
{
    for (bool disarm : {true, false}) {
        SCOPED_TRACE(disarm ? "disarm" : "throttle off");
        ASSERT_NO_FATAL_FAILURE(EnterActive());
        input.yaw = 0.20f;
        Update();
        ExpectActiveOutput(0.36f, 0.20f);
        if (disarm) {
            armingFlags = 0;
        } else {
            input.throttle = 0;
            input.throttleOff = true;
        }
        Update();
        EXPECT_EQ(disarm ? GOV_STATE_THROTTLE_OFF : GOV_STATE_THROTTLE_HOLD, getGovernorState());
        EXPECT_FLOAT_EQ(0, getGovernorOutput());
    }
}

TEST_F(GovernorDirectTest, IdleAndSpoolupBaselineRemainsUnchanged)
{
    ASSERT_NO_FATAL_FAILURE(InitDisarmed());
    input.throttle = 0.30f;
    input.throttleOff = false;
    armingFlags = ARMED;
    Update();
    ASSERT_EQ(GOV_STATE_THROTTLE_IDLE, getGovernorState());
    EXPECT_NEAR(0.25f, getGovernorOutput(), 1e-6f);
    Update();
    ASSERT_EQ(GOV_STATE_SPOOLUP, getGovernorState());
    EXPECT_NEAR(0.30f, getGovernorOutput(), 1e-6f);
    Update();
    ExpectActiveOutput(0.30f, 0);
}

TEST_F(GovernorDirectTest, BypassRemainsCurveDriven)
{
    for (auto &point : governorConfigMutable()->gov_bypass_throttle) {
        point = 80; // Curve units are 1/200: every point maps to 0.40.
    }
    ASSERT_NO_FATAL_FAILURE(EnterActive(0.50f));
    input.yaw = 0.20f;
    Update();
    ExpectActiveOutput(0.60f, 0.20f);
    input.bypass = true;
    Update();
    ASSERT_EQ(GOV_STATE_BYPASS, getGovernorState());
    EXPECT_NEAR(0.20f, getTTAIncrease(), 1e-6f);
    EXPECT_NEAR(0.40f, getGovernorOutput(), 1e-6f);
}
