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

#include <cstring>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "blackbox/blackbox.h"
#include "drivers/time.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_modes.h"
#include "pg/adjustments.h"
#include "rx/rx.h"

float rcInput[MAX_SUPPORTED_RC_CHANNEL_COUNT];

static timeMs_t now;
static int beeps;
static int dirty;

// Pitch P lives in the PID profile, so it gets one value per profile here.
static int pitchP[2];
static int profile;

timeMs_t millis(void) { return now; }
bool rxIsReceivingSignal(void) { return true; }
void beeperConfirmationBeeps(uint8_t count) { beeps += count; }
void setConfigDirty(void) { dirty++; }
void blackboxLogEvent(FlightLogEvent, flightLogEventData_t *) {}

int get_ADJUSTMENT_PITCH_P_GAIN(void) { return pitchP[profile]; }
void set_ADJUSTMENT_PITCH_P_GAIN(int value) { pitchP[profile] = value; }

// Every other function is a plain value; the tests don't touch them.
#define ADJ_STUB(id) \
    int get_ADJUSTMENT_##id(void) { return 0; } \
    void set_ADJUSTMENT_##id(int) {}

ADJ_STUB(RATE_PROFILE) ADJ_STUB(PID_PROFILE) ADJ_STUB(LED_PROFILE) ADJ_STUB(OSD_PROFILE)
ADJ_STUB(PITCH_SRATE) ADJ_STUB(ROLL_SRATE) ADJ_STUB(YAW_SRATE)
ADJ_STUB(PITCH_RC_RATE) ADJ_STUB(ROLL_RC_RATE) ADJ_STUB(YAW_RC_RATE)
ADJ_STUB(PITCH_RC_EXPO) ADJ_STUB(ROLL_RC_EXPO) ADJ_STUB(YAW_RC_EXPO)
ADJ_STUB(PITCH_SP_BOOST_GAIN) ADJ_STUB(ROLL_SP_BOOST_GAIN) ADJ_STUB(YAW_SP_BOOST_GAIN) ADJ_STUB(COLL_SP_BOOST_GAIN)
ADJ_STUB(YAW_DYN_CEILING_GAIN) ADJ_STUB(YAW_DYN_DEADBAND_GAIN) ADJ_STUB(YAW_DYN_DEADBAND_FILTER)
ADJ_STUB(PITCH_I_GAIN) ADJ_STUB(PITCH_D_GAIN) ADJ_STUB(PITCH_F_GAIN)
ADJ_STUB(ROLL_P_GAIN) ADJ_STUB(ROLL_I_GAIN) ADJ_STUB(ROLL_D_GAIN) ADJ_STUB(ROLL_F_GAIN)
ADJ_STUB(YAW_P_GAIN) ADJ_STUB(YAW_I_GAIN) ADJ_STUB(YAW_D_GAIN) ADJ_STUB(YAW_F_GAIN)
ADJ_STUB(PITCH_B_GAIN) ADJ_STUB(ROLL_B_GAIN) ADJ_STUB(YAW_B_GAIN)
ADJ_STUB(PITCH_O_GAIN) ADJ_STUB(ROLL_O_GAIN)
ADJ_STUB(CYCLIC_MASTER_GAIN) ADJ_STUB(YAW_MASTER_GAIN)
ADJ_STUB(YAW_CW_GAIN) ADJ_STUB(YAW_CCW_GAIN) ADJ_STUB(YAW_CYCLIC_FF) ADJ_STUB(YAW_COLLECTIVE_FF)
ADJ_STUB(PITCH_COLLECTIVE_FF)
ADJ_STUB(PITCH_GYRO_CUTOFF) ADJ_STUB(ROLL_GYRO_CUTOFF) ADJ_STUB(YAW_GYRO_CUTOFF)
ADJ_STUB(PITCH_DTERM_CUTOFF) ADJ_STUB(ROLL_DTERM_CUTOFF) ADJ_STUB(YAW_DTERM_CUTOFF)
ADJ_STUB(YAW_PRECOMP_CUTOFF) ADJ_STUB(INERTIA_PRECOMP_GAIN) ADJ_STUB(INERTIA_PRECOMP_CUTOFF)
ADJ_STUB(CROSS_COUPLING_GAIN) ADJ_STUB(CROSS_COUPLING_RATIO) ADJ_STUB(CROSS_COUPLING_CUTOFF)
ADJ_STUB(ANGLE_LEVEL_GAIN) ADJ_STUB(HORIZON_LEVEL_GAIN) ADJ_STUB(ACRO_TRAINER_GAIN)
ADJ_STUB(RESCUE_CLIMB_COLLECTIVE) ADJ_STUB(RESCUE_HOVER_COLLECTIVE) ADJ_STUB(RESCUE_HOVER_ALTITUDE)
ADJ_STUB(RESCUE_ALT_P_GAIN) ADJ_STUB(RESCUE_ALT_I_GAIN) ADJ_STUB(RESCUE_ALT_D_GAIN)
ADJ_STUB(GOV_GAIN) ADJ_STUB(GOV_P_GAIN) ADJ_STUB(GOV_I_GAIN) ADJ_STUB(GOV_D_GAIN) ADJ_STUB(GOV_F_GAIN)
ADJ_STUB(GOV_TTA_GAIN) ADJ_STUB(GOV_YAW_FF) ADJ_STUB(GOV_CYCLIC_FF) ADJ_STUB(GOV_COLLECTIVE_FF)
ADJ_STUB(GOV_IDLE_THROTTLE) ADJ_STUB(GOV_AUTO_THROTTLE) ADJ_STUB(GOV_MAX_THROTTLE) ADJ_STUB(GOV_MIN_THROTTLE)
ADJ_STUB(GOV_HEADSPEED) ADJ_STUB(ACC_TRIM_PITCH) ADJ_STUB(ACC_TRIM_ROLL) ADJ_STUB(BATTERY_PROFILE)
}

// Adjustment channel is AUX1. Below 1200us steps down, above 1800us steps up.
#define ADJ_CHANNEL     0
#define CENTER_US       1500
#define DOWN_US         1000
#define UP_US           2000

class StepAdjustmentTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            memset(adjustmentRangesMutable(i), 0, sizeof(adjustmentRange_t));
        }

        adjustmentRange_t *range = adjustmentRangesMutable(0);
        range->function = ADJUSTMENT_PITCH_P_GAIN;
        range->enaChannel = 0xff;
        range->adjChannel = ADJ_CHANNEL;
        range->adjRange1.startStep = CHANNEL_VALUE_TO_STEP(875);
        range->adjRange1.endStep = CHANNEL_VALUE_TO_STEP(1200);
        range->adjRange2.startStep = CHANNEL_VALUE_TO_STEP(1800);
        range->adjRange2.endStep = CHANNEL_VALUE_TO_STEP(2125);
        range->adjMin = 0;
        range->adjMax = 250;
        range->adjStep = 1;

        now = 1000;
        beeps = 0;
        dirty = 0;
        profile = 0;
        pitchP[0] = 110;
        pitchP[1] = 110;

        setChannel(CENTER_US);
        adjustmentRangeInit();
        settle();
    }

    void setChannel(int us)
    {
        rcInput[ADJ_CHANNEL + CONTROL_CHANNEL_COUNT] = us;
    }

    // Run the adjustment task every 10ms for a while, as the scheduler would.
    void run(int ms)
    {
        for (int i = 0; i < ms / 10; i++) {
            now += 10;
            processRcAdjustments();
        }
    }

    void settle()
    {
        run(500);
    }

    // Flick the switch to one side and back. 150ms is past the trigger delay
    // but short of the repeat delay, so this is exactly one step.
    void flick(int us)
    {
        setChannel(us);
        run(150);
        setChannel(CENTER_US);
        settle();
    }
};

TEST_F(StepAdjustmentTest, StepsUpAndDown)
{
    flick(UP_US);
    EXPECT_EQ(111, pitchP[0]);
    flick(UP_US);
    EXPECT_EQ(112, pitchP[0]);
    flick(DOWN_US);
    EXPECT_EQ(111, pitchP[0]);
    EXPECT_EQ(3, beeps);
}

// Issue #561: the new profile holds the value the slot wrote last on the old one.
TEST_F(StepAdjustmentTest, StepsAfterProfileSwitchToSameValue)
{
    flick(UP_US);
    EXPECT_EQ(111, pitchP[0]);
    EXPECT_EQ(1, beeps);

    profile = 1;

    flick(UP_US);
    EXPECT_EQ(111, pitchP[1]);
    EXPECT_EQ(2, beeps);

    flick(UP_US);
    EXPECT_EQ(112, pitchP[1]);
    EXPECT_EQ(3, beeps);

    // The first profile is left alone.
    EXPECT_EQ(111, pitchP[0]);
}

// Same thing without a profile switch: the value is set back from MSP or the CLI.
TEST_F(StepAdjustmentTest, StepsAfterExternalWrite)
{
    flick(UP_US);
    EXPECT_EQ(111, pitchP[0]);

    pitchP[0] = 110;

    flick(UP_US);
    EXPECT_EQ(111, pitchP[0]);
    EXPECT_EQ(2, beeps);
}

// Holding a step at the limit must not write or beep again and again.
TEST_F(StepAdjustmentTest, NoRepeatAtLimit)
{
    adjustmentRangesMutable(0)->adjMax = 111;

    flick(UP_US);
    EXPECT_EQ(111, pitchP[0]);
    EXPECT_EQ(1, beeps);
    EXPECT_EQ(1, dirty);

    setChannel(UP_US);
    for (int i = 0; i < 10; i++) {
        settle();
    }
    EXPECT_EQ(111, pitchP[0]);
    EXPECT_EQ(1, beeps);
    EXPECT_EQ(1, dirty);
}
