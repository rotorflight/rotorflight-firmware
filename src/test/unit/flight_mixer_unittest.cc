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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/mixer.h"
#include "pg/pg.h"
}

// Inputs to the mixer, set by the tests
static float pidOutput[3];
static float pidCollective;
static float rcDeflection[4];
static float governorOutput;
static bool  spooledUp;
static float ttaIncrease;
static bool  wiggleOn;
static float wiggleAxis[4];
static timeMs_t nowMs;

extern "C" {
uint8_t armingFlags;
float rcCommand[MAX_SUPPORTED_RC_CHANNEL_COUNT];

float pidGetOutput(int axis) { return pidOutput[axis]; }
float pidGetCollective(void) { return pidCollective; }
float getRcDeflection(int axis) { return rcDeflection[axis]; }
float getThrottle(void) { return 0; }

void  governorUpdate(void) {}
float getGovernorOutput(void) { return governorOutput; }
bool  isSpooledUp(void) { return spooledUp; }
float getTTAIncrease(void) { return ttaIncrease; }

void  wiggleInit(void) {}
void  wiggleUpdate(timeUs_t) {}
bool  wiggleActive(void) { return wiggleOn; }
float wiggleGetAxis(int axis) { return wiggleAxis[axis]; }

timeMs_t millis(void) { return nowMs; }
}

static constexpr float EPS = 1e-5f;

// Stabilized inputs are scaled by the default input rate (250/1000) on the way to the swash
static constexpr float RATE = 0.25f;

static float servo(int i) { return mixerGetServoOutput(i); }
static float motor(int i) { return mixerGetMotorOutput(i); }

class MixerTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();

        // Plain geometry unless a test asks for more
        mixerConfigMutable()->swash_ring = 0;
        mixerConfigMutable()->collective_tilt_correction_neg = 0;

        for (auto &v : pidOutput) v = 0;
        for (auto &v : rcDeflection) v = 0;
        for (auto &v : wiggleAxis) v = 0;
        for (auto &v : rcCommand) v = 0;
        pidCollective = 0;
        governorOutput = 0;
        spooledUp = true;
        ttaIncrease = 0;
        wiggleOn = false;
        nowMs = 1000;
        armingFlags = 0;

        init();

        // Mixer state is static: drain saturation left over from earlier tests
        for (int i = 0; i <= MIXER_SATURATION_TIME; i++)
            mixerUpdate(0);
        mixerTakeStabilizedSaturation();
    }

    void init()
    {
        mixerInit();
    }

    void setPid(float roll, float pitch, float yaw, float coll)
    {
        pidOutput[PID_ROLL] = roll;
        pidOutput[PID_PITCH] = pitch;
        pidOutput[PID_YAW] = yaw;
        pidCollective = coll;
    }
};

/** Swashplate geometry **/

TEST_F(MixerTest, Swash120Collective)
{
    setPid(0, 0, 0, 1.0f);
    mixerUpdate(0);

    const float SC = RATE * 1.0f;
    EXPECT_NEAR(0.5f * SC, servo(0), EPS);
    EXPECT_NEAR(0.5f * SC, servo(1), EPS);
    EXPECT_NEAR(0.5f * SC, servo(2), EPS);
}

TEST_F(MixerTest, Swash120Pitch)
{
    setPid(0, 1.0f, 0, 0);
    mixerUpdate(0);

    const float SP = RATE * 1.0f;
    EXPECT_NEAR(-SP, servo(0), EPS);
    EXPECT_NEAR(0.5f * SP, servo(1), EPS);
    EXPECT_NEAR(0.5f * SP, servo(2), EPS);
}

TEST_F(MixerTest, Swash120Roll)
{
    setPid(1.0f, 0, 0, 0);
    mixerUpdate(0);

    const float SR = RATE * 1.0f;
    EXPECT_NEAR(0, servo(0), EPS);
    EXPECT_NEAR(0.8660254f * SR, servo(1), EPS);
    EXPECT_NEAR(-0.8660254f * SR, servo(2), EPS);
}

TEST_F(MixerTest, Swash120TailServoAndThrottle)
{
    mixerConfigMutable()->tail_center_trim = 100;
    init();

    setPid(0, 0, 0.4f, 0);
    governorOutput = 0.6f;
    mixerUpdate(0);

    EXPECT_NEAR(RATE * 0.4f + 0.1f, servo(3), EPS);
    EXPECT_NEAR(0.6f, motor(0), EPS);   // throttle rate is 1000
    EXPECT_NEAR(0, motor(1), EPS);
}

TEST_F(MixerTest, Swash135)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_135;
    init();

    setPid(0.4f, 0.8f, 0, 1.0f);
    mixerUpdate(0);

    const float SR = RATE * 0.4f, SP = RATE * 0.8f, SC = RATE * 1.0f;
    EXPECT_NEAR(0.5f * SC - SP, servo(0), EPS);
    EXPECT_NEAR(0.5f * SC + SR + SP, servo(1), EPS);
    EXPECT_NEAR(0.5f * SC - SR + SP, servo(2), EPS);
}

TEST_F(MixerTest, Swash140)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_140;
    init();

    setPid(0.4f, 0.8f, 0, 1.0f);
    mixerUpdate(0);

    const float SR = RATE * 0.4f, SP = RATE * 0.8f, SC = RATE * 1.0f;
    EXPECT_NEAR(0.5f * SC - SP, servo(0), EPS);
    EXPECT_NEAR(0.5f * SC + 0.866025f * SR + SP, servo(1), EPS);
    EXPECT_NEAR(0.5f * SC - 0.866025f * SR + SP, servo(2), EPS);
}

TEST_F(MixerTest, Swash90L)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_90L;
    init();

    setPid(0.4f, 0.8f, 0, 1.0f);
    mixerUpdate(0);

    EXPECT_NEAR(RATE * 0.8f, servo(0), EPS);
    EXPECT_NEAR(RATE * 0.4f, servo(1), EPS);
    EXPECT_NEAR(0, servo(2), EPS);
}

TEST_F(MixerTest, Swash90V)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_90V;
    init();

    setPid(0.4f, 0.8f, 0, 0);
    mixerUpdate(0);

    const float SR = RATE * 0.4f, SP = RATE * 0.8f;
    EXPECT_NEAR(0.70710678f * (SR + SP), servo(0), EPS);
    EXPECT_NEAR(0.70710678f * (SP - SR), servo(1), EPS);
}

TEST_F(MixerTest, SwashThru)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_THRU;
    init();

    setPid(0.4f, 0.8f, 0, 1.0f);
    mixerUpdate(0);

    EXPECT_NEAR(RATE * 0.8f, servo(0), EPS);
    EXPECT_NEAR(RATE * 0.4f, servo(1), EPS);
    EXPECT_NEAR(RATE * 1.0f, servo(2), EPS);
}

TEST_F(MixerTest, SwashNoneLeavesOutputsAlone)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_NONE;
    init();

    setPid(0.4f, 0.8f, 0.2f, 1.0f);
    governorOutput = 0.5f;
    mixerUpdate(0);

    for (int i = 0; i < 4; i++)
        EXPECT_EQ(0, servo(i));
    EXPECT_EQ(0, motor(0));
}

TEST_F(MixerTest, SwashTrim)
{
    mixerConfigMutable()->swash_trim[0] = 10;   // roll
    mixerConfigMutable()->swash_trim[1] = 20;   // pitch
    mixerConfigMutable()->swash_trim[2] = 30;   // collective
    init();

    mixerUpdate(0);

    const float SR = 0.010f, SP = 0.020f, SC = 0.030f;
    EXPECT_NEAR(0.5f * SC - SP, servo(0), EPS);
    EXPECT_NEAR(0.5f * SC + 0.8660254f * SR + 0.5f * SP, servo(1), EPS);
    EXPECT_NEAR(0.5f * SC - 0.8660254f * SR + 0.5f * SP, servo(2), EPS);
}

TEST_F(MixerTest, CyclicServoMapping)
{
    EXPECT_TRUE(mixerIsCyclicServo(0));
    EXPECT_TRUE(mixerIsCyclicServo(1));
    EXPECT_TRUE(mixerIsCyclicServo(2));
    EXPECT_FALSE(mixerIsCyclicServo(3));
}

/** Collective corrections **/

TEST_F(MixerTest, CollectiveGeoCorrection)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_THRU;
    mixerConfigMutable()->swash_geo_correction = 100;   // +10%
    init();

    setPid(0, 0, 0, 1.0f);
    mixerUpdate(0);
    EXPECT_NEAR(RATE * 1.0f * 1.1f, servo(2), EPS);

    // Negative collective is reduced by the same ratio
    setPid(0, 0, 0, -1.0f);
    mixerUpdate(0);
    EXPECT_NEAR(RATE * -1.0f * 0.9f, servo(2), EPS);
}

TEST_F(MixerTest, CollectiveTiltCorrection)
{
    mixerConfigMutable()->swash_type = SWASH_TYPE_THRU;
    mixerConfigMutable()->collective_tilt_correction_pos = 50;
    mixerConfigMutable()->collective_tilt_correction_neg = 20;
    init();

    const float SR = RATE * 1.0f, SP = RATE * 1.0f, SC = RATE * 1.0f;

    setPid(1.0f, 1.0f, 0, 1.0f);
    mixerUpdate(0);
    EXPECT_NEAR(SC * (1 + 0.5f * (SR * SR + SP * SP)), servo(2), EPS);

    setPid(1.0f, 1.0f, 0, -1.0f);
    mixerUpdate(0);
    EXPECT_NEAR(-SC * (1 - 0.2f * (SR * SR + SP * SP)), servo(2), EPS);
}

TEST_F(MixerTest, CollectiveTTAPrecomp)
{
    mixerConfigMutable()->swash_tta_precomp = 50;
    init();

    setPid(0, 0, 0, 1.0f);
    ttaIncrease = 0.2f;
    mixerUpdate(0);

    const float ratio = 1.0f + 0.2f * 0.5f;
    EXPECT_NEAR(1.0f / (ratio * ratio), mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE), EPS);
}

/** Input limits and saturation **/

TEST_F(MixerTest, InputLimitAndSaturation)
{
    setPid(2.0f, -2.0f, 0, 0);
    mixerUpdate(0);

    // Default stabilized limits are +-1250
    EXPECT_NEAR(1.25f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(-1.25f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));
    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_PITCH));
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_YAW));
    EXPECT_TRUE(pidAxisSaturated(PID_ROLL));

    // Saturation holds for MIXER_SATURATION_TIME updates, then clears
    setPid(0, 0, 0, 0);
    for (int i = 0; i < MIXER_SATURATION_TIME - 1; i++) {
        mixerUpdate(0);
        EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_ROLL)) << "update " << i;
    }
    mixerUpdate(0);
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));
}

TEST_F(MixerTest, StabilizedSaturationLatch)
{
    EXPECT_FALSE(mixerTakeStabilizedSaturation());

    setPid(0, 0, 0, 5.0f);
    mixerUpdate(0);
    setPid(0, 0, 0, 0);
    mixerUpdate(0);   // latch is taken at the start of the next update

    EXPECT_TRUE(mixerTakeStabilizedSaturation());
    EXPECT_FALSE(mixerTakeStabilizedSaturation());
}

TEST_F(MixerTest, SaturateOutputMarksMappedInputs)
{
    // Servo 0 on a 120 swash is driven by pitch and collective, not roll
    mixerSaturateServoOutput(0);

    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_PITCH));
    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_COLLECTIVE));
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_YAW));
}

/** Cyclic limits and phasing **/

TEST_F(MixerTest, SwashRing)
{
    mixerConfigMutable()->swash_ring = 100;   // ring limit = 1.0
    init();

    // Full roll and pitch together are outside the ring and get scaled back to it
    setPid(1.25f, 1.25f, 0, 0);
    mixerUpdate(0);

    const float scaled = 1.25f / sqrtf(2.0f);
    EXPECT_NEAR(scaled, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(scaled, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
    EXPECT_NEAR(1.25f, getCyclicDeflection(), EPS);
    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));

    // A single axis just short of full travel is inside the ring
    // (at exactly full travel the float ring limit puts it a hair outside)
    setPid(1.2f, 0, 0, 0);
    for (int i = 0; i <= MIXER_SATURATION_TIME; i++)
        mixerUpdate(0);
    EXPECT_NEAR(1.2f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));
}

TEST_F(MixerTest, TotalPitchLimit)
{
    mixerConfigMutable()->swash_pitch_limit = 1500;
    init();

    // 1.0 collective leaves 0.5 for cyclic
    setPid(1.0f, 0, 0, 1.0f);
    mixerUpdate(0);

    EXPECT_NEAR(0.5f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(0.5f, getCyclicDeflection(), EPS);
    EXPECT_TRUE(mixerSaturated(MIXER_IN_STABILIZED_ROLL));

    // Less collective, no limiting
    setPid(0.3f, 0.4f, 0, 0.5f);
    mixerUpdate(0);
    EXPECT_NEAR(0.3f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(0.4f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
}

TEST_F(MixerTest, SwashPhase)
{
    mixerConfigMutable()->swash_phase = 900;   // 90.0 degrees
    init();

    // Pitch is rotated onto roll
    setPid(0, 1.0f, 0, 0);
    mixerUpdate(0);

    EXPECT_NEAR(1.0f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(0, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
    EXPECT_NEAR(1.0f, getCyclicDeflection(), EPS);
}

/** Overrides **/

TEST_F(MixerTest, OverrideWhenDisarmed)
{
    EXPECT_FALSE(isMixerOverrideActive());

    mixerSetOverride(MIXER_IN_STABILIZED_PITCH, 500);
    setPid(0, 0.1f, 0, 0);
    mixerUpdate(0);

    EXPECT_TRUE(isMixerOverrideActive());
    EXPECT_EQ(500, mixerGetOverride(MIXER_IN_STABILIZED_PITCH));
    EXPECT_NEAR(0.5f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);

    mixerSetOverride(MIXER_IN_STABILIZED_PITCH, MIXER_OVERRIDE_OFF);
    mixerUpdate(0);
    EXPECT_FALSE(isMixerOverrideActive());
    EXPECT_NEAR(0.1f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
}

TEST_F(MixerTest, OverrideIgnoredWhenArmed)
{
    mixerSetOverride(MIXER_IN_STABILIZED_PITCH, 500);
    ENABLE_ARMING_FLAG(ARMED);

    setPid(0, 0.1f, 0, 0);
    mixerUpdate(0);

    EXPECT_NEAR(0.1f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
}

TEST_F(MixerTest, TimedOverrideExpires)
{
    mixerSetOverrideTimed(MIXER_IN_STABILIZED_ROLL, 300, 1000);

    nowMs += 999;
    mixerUpdate(0);
    EXPECT_TRUE(isMixerOverrideActive());
    EXPECT_NEAR(0.3f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);

    nowMs += 1;
    mixerUpdate(0);
    EXPECT_FALSE(isMixerOverrideActive());
    EXPECT_EQ(MIXER_OVERRIDE_OFF, mixerGetOverride(MIXER_IN_STABILIZED_ROLL));
    EXPECT_NEAR(0, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
}

TEST_F(MixerTest, TimedOverrideTimeoutIsClamped)
{
    // Shorter than OVERRIDE_TIMEOUT_MIN_MS is raised to it
    mixerSetOverrideTimed(MIXER_IN_STABILIZED_ROLL, 300, 10);

    nowMs += OVERRIDE_TIMEOUT_MIN_MS - 1;
    EXPECT_TRUE(isMixerOverrideActive());
    nowMs += 1;
    EXPECT_FALSE(isMixerOverrideActive());
}

TEST_F(MixerTest, PlainOverrideClearsTimeout)
{
    mixerSetOverrideTimed(MIXER_IN_STABILIZED_ROLL, 300, 1000);
    mixerSetOverride(MIXER_IN_STABILIZED_ROLL, 200);

    nowMs += 5000;
    EXPECT_EQ(200, mixerGetOverride(MIXER_IN_STABILIZED_ROLL));
}

TEST_F(MixerTest, PassthroughOverride)
{
    for (int i = MIXER_IN_STABILIZED_ROLL; i <= MIXER_IN_STABILIZED_COLLECTIVE; i++)
        mixerSetOverride(i, MIXER_OVERRIDE_PASSTHROUGH);
    EXPECT_TRUE(isMixerOverrideActive());

    // RC is scaled by 120% onto the input range
    rcDeflection[ROLL] = 0.5f;
    rcDeflection[PITCH] = -0.5f;
    rcDeflection[YAW] = 0.5f;
    rcDeflection[COLLECTIVE] = 1.0f;
    setPid(0.9f, 0.9f, 0.9f, 0.9f);
    mixerUpdate(0);

    EXPECT_NEAR(0.6f * 1.25f, mixerGetInput(MIXER_IN_STABILIZED_ROLL), EPS);
    EXPECT_NEAR(-0.6f * 1.25f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
    EXPECT_NEAR(-0.6f * 1.25f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);   // yaw is reversed
    EXPECT_NEAR(1.25f, mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE), EPS);   // 1.2 * 1.25 hits the limit
}

TEST_F(MixerTest, WiggleWhenDisarmed)
{
    wiggleOn = true;
    wiggleAxis[FD_PITCH] = 0.2f;
    setPid(0, 0.7f, 0, 0);
    mixerUpdate(0);
    EXPECT_NEAR(0.2f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);

    ENABLE_ARMING_FLAG(ARMED);
    mixerUpdate(0);
    EXPECT_NEAR(0.7f, mixerGetInput(MIXER_IN_STABILIZED_PITCH), EPS);
}

/** Tail rotor modes **/

TEST_F(MixerTest, MotorizedTail)
{
    mixerConfigMutable()->tail_rotor_mode = TAIL_MODE_MOTORIZED;
    mixerConfigMutable()->tail_motor_idle = 100;   // 10%
    init();
    ASSERT_EQ(-1, mixerRotationSign());   // CW main rotor

    governorOutput = 0.5f;

    // Yaw against torque raises tail throttle
    setPid(0, 0, -0.4f, 0);
    mixerUpdate(0);
    EXPECT_NEAR(0.4f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);
    EXPECT_NEAR(RATE * 0.4f, motor(1), EPS);
    EXPECT_EQ(0, servo(3));

    // Yaw with torque is held at idle
    setPid(0, 0, 0.4f, 0);
    mixerUpdate(0);
    EXPECT_NEAR(0.1f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);

    // Not spooled up and no main throttle: tail motor off
    spooledUp = false;
    governorOutput = 0;
    mixerUpdate(0);
    EXPECT_NEAR(0, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);
}

TEST_F(MixerTest, BidirectionalTail)
{
    mixerConfigMutable()->tail_rotor_mode = TAIL_MODE_BIDIRECTIONAL;
    mixerConfigMutable()->tail_motor_idle = 100;
    init();

    governorOutput = 0.5f;

    setPid(0, 0, 0.4f, 0);
    mixerUpdate(0);
    EXPECT_NEAR(-0.4f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);

    // Inside the idle band the motor keeps idling in its last direction
    setPid(0, 0, 0.05f, 0);
    mixerUpdate(0);
    EXPECT_NEAR(-0.1f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);

    setPid(0, 0, -0.4f, 0);
    mixerUpdate(0);
    setPid(0, 0, -0.05f, 0);
    mixerUpdate(0);
    EXPECT_NEAR(0.1f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);

    // Slow spool-up: scaled between 5% and 10% main throttle
    spooledUp = false;
    setPid(0, 0, -0.4f, 0);
    governorOutput = 0.075f;
    mixerUpdate(0);
    EXPECT_NEAR(0.4f * 0.75f, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);

    governorOutput = 0.04f;
    mixerUpdate(0);
    EXPECT_NEAR(0, mixerGetInput(MIXER_IN_STABILIZED_YAW), EPS);
}

/** Mixer rules **/

TEST_F(MixerTest, Rules)
{
    const uint8_t out = MIXER_SERVO_OFFSET + 4;

    *mixerRulesMutable(0) = { MIXER_OP_SET, MIXER_IN_RC_CHANNEL_AUX1, out, 100, 500 };
    *mixerRulesMutable(1) = { MIXER_OP_ADD, MIXER_IN_RC_CHANNEL_AUX2, out, 0, 1000 };
    *mixerRulesMutable(2) = { MIXER_OP_MUL, MIXER_IN_NONE,            out, 2000, 0 };
    init();

    rcCommand[AUX1] = 250;   // input 0.5
    rcCommand[AUX2] = -100;  // input -0.2
    mixerUpdate(0);

    // ((100 + 500 * 0.5) + (1000 * -0.2)) * 2 / 1000
    EXPECT_NEAR((0.35f - 0.2f) * 2.0f, servo(4), EPS);

    // Saturating the output saturates both rule inputs
    mixerSaturateServoOutput(4);
    EXPECT_TRUE(mixerSaturated(MIXER_IN_RC_CHANNEL_AUX1));
    EXPECT_TRUE(mixerSaturated(MIXER_IN_RC_CHANNEL_AUX2));
}

TEST_F(MixerTest, RuleSetOverridesSwash)
{
    *mixerRulesMutable(0) = { MIXER_OP_SET, MIXER_IN_NONE, MIXER_SERVO_OFFSET + 0, 123, 0 };
    init();

    setPid(0, 1.0f, 0, 0);
    mixerUpdate(0);
    EXPECT_NEAR(0.123f, servo(0), EPS);

    // SET replaces the swash mapping of that servo
    mixerSaturateServoOutput(0);
    EXPECT_FALSE(mixerSaturated(MIXER_IN_STABILIZED_PITCH));
}

/** Config validation **/

TEST_F(MixerTest, ValidateRaisesPitchLimit)
{
    // Must leave at least 500 for cyclic above the collective range (1250)
    mixerConfigMutable()->swash_pitch_limit = 1000;
    validateAndFixMixerConfig();
    EXPECT_EQ(1750, mixerConfig()->swash_pitch_limit);

    mixerConfigMutable()->swash_pitch_limit = 2000;
    validateAndFixMixerConfig();
    EXPECT_EQ(2000, mixerConfig()->swash_pitch_limit);

    mixerConfigMutable()->swash_pitch_limit = 0;
    validateAndFixMixerConfig();
    EXPECT_EQ(0, mixerConfig()->swash_pitch_limit);
}

TEST_F(MixerTest, ValidateRules)
{
    *mixerRulesMutable(0) = { 9, 200, 200, 30000, -30000 };
    *mixerRulesMutable(1) = { MIXER_OP_NUL, 5, 6, 7, 8 };
    validateAndFixMixerConfig();

    const mixerRule_t *r0 = mixerRules(0);
    EXPECT_EQ(MIXER_OP_COUNT - 1, r0->oper);
    EXPECT_EQ(MIXER_INPUT_COUNT - 1, r0->input);
    EXPECT_EQ(MIXER_OUTPUT_COUNT - 1, r0->output);
    EXPECT_EQ(MIXER_INPUT_MAX, r0->offset);
    EXPECT_EQ(MIXER_WEIGHT_MIN, r0->weight);

    const mixerRule_t *r1 = mixerRules(1);
    EXPECT_EQ(0, r1->input);
    EXPECT_EQ(0, r1->output);
    EXPECT_EQ(0, r1->offset);
    EXPECT_EQ(0, r1->weight);
}
