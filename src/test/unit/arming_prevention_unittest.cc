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
 * Arming checks in fc/core.c: updateArmingStatus(), tryArm() and disarm(),
 * with the real mode activation logic (rc_modes.c) and arming flags
 * (runtime_config.c). Everything core.c reaches beyond that is stubbed;
 * the inputs the checks look at are controlled from the tests.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <vector>

extern "C" {
#include "platform.h"

#include "common/bitarray.h"

#include "pg/pg.h"
#include "pg/arming.h"
#include "pg/mixer.h"
#include "pg/modes.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
}

#include "gtest/gtest.h"

// Inputs to the arming checks
static timeUs_t nowUs;
static bool haveRx;
static bool throttleOff;
static bool upright;
static bool gyroCalibrated;
static bool accCalibrated;
static uint16_t systemLoad;
static bool servoOverride;
static bool mixerOverride;
static bool motorProtocolEnabled;
static bool motorInitFailed;
static uint8_t motorCount;

static std::vector<beeperMode_e> beeps;
static std::vector<uint8_t> warningBeeps;

extern "C" {
float rcInput[MAX_SUPPORTED_RC_CHANNEL_COUNT];

timeUs_t micros(void) { return nowUs; }
timeMs_t millis(void) { return nowUs / 1000; }

bool rxIsReceivingSignal(void) { return haveRx; }
bool isThrottleOff(void) { return throttleOff; }
bool isUpright(void) { return upright; }
bool gyroIsCalibrationComplete(void) { return gyroCalibrated; }
bool accIsCalibrationComplete(void) { return true; }
bool accHasBeenCalibrated(void) { return accCalibrated; }
bool isFirstArmingGyroCalibrationRunning(void) { return false; }

uint16_t getMaxRealTimeLoad(void) { return systemLoad; }
uint16_t getAverageCPULoad(void) { return systemLoad; }
uint16_t getAverageSystemLoad(void) { return systemLoad; }

bool isServoOverrideActive(void) { return servoOverride; }
bool isMixerOverrideActive(void) { return mixerOverride; }
bool isMotorProtocolEnabled(void) { return motorProtocolEnabled; }
bool isMotorInitFailed(void) { return motorInitFailed; }
uint8_t getMotorCount(void) { return motorCount; }

bool isUsingSticksForArming(void) { return false; }
bool wiggleEnabled(int) { return false; }
bool wiggleActive(void) { return false; }
void wiggleTrigger(int, int) {}

void beeper(beeperMode_e mode) { beeps.push_back(mode); }
void beeperWarningBeeps(uint8_t count) { warningBeeps.push_back(count); }

bool isConfigDirty(void) { return false; }
}

#include "core_stubs.h"

class ArmingTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        armingFlags = 0;
        unsetArmingDisabled((armingDisableFlags_e)0xFFFFFFFF);
        sensorsClear(0xFFFFFFFF);
        sensorsSet(SENSOR_GYRO);

        nowUs = 10 * 1000 * 1000;    // past the boot grace time
        haveRx = true;
        throttleOff = true;
        upright = true;
        gyroCalibrated = true;
        accCalibrated = true;
        systemLoad = 100;
        servoOverride = false;
        mixerOverride = false;
        motorProtocolEnabled = true;
        motorInitFailed = false;
        motorCount = 1;
        beeps.clear();
        warningBeeps.clear();

        for (auto &v : rcInput) v = 1500;
        armOn = false;
        resetArmingDisabled();

        // ARM on aux 1 high
        addMode(BOXARM, 0);
        analyzeModeActivationConditions();
        update();
    }

    void addMode(boxId_e mode, uint8_t aux)
    {
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            if (mac->range.startStep == 0 && mac->range.endStep == 0 && !mac->linkedTo) {
                mac->modeId = mode;
                mac->auxChannelIndex = aux;
                mac->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
                mac->range.endStep = CHANNEL_VALUE_TO_STEP(2100);
                analyzeModeActivationConditions();
                return;
            }
        }
    }

    void setAux(uint8_t aux, bool on) { rcInput[CONTROL_CHANNEL_COUNT + aux] = on ? 2000 : 1000; }

    void setArmSwitch(bool on) { armOn = on; setAux(0, on); }

    // One pass of mode and arming evaluation
    void update()
    {
        nowUs += 10 * 1000;
        updateActivatedModes();
        updateArmingStatus();
    }

    bool blockedBy(armingDisableFlags_e flag) { return getArmingDisableFlags() & flag; }

    // Flip the arm switch on and try to arm, like processRxModes() does
    bool switchArm()
    {
        setArmSwitch(true);
        update();
        tryArm();
        return ARMING_FLAG(ARMED);
    }

    bool armOn;
};

TEST_F(ArmingTest, ArmsWhenClear)
{
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_TRUE(switchArm());
    EXPECT_FALSE(beeps.empty());
    EXPECT_EQ(BEEPER_ARMING, beeps.back());
}

TEST_F(ArmingTest, DisarmSetsWasEverArmed)
{
    ASSERT_TRUE(switchArm());
    disarm(DISARM_REASON_SWITCH);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(ARMING_FLAG(WAS_EVER_ARMED));
    EXPECT_EQ(BEEPER_DISARMING, beeps.back());
}

TEST_F(ArmingTest, BootGraceTime)
{
    // Default grace time is 3 s after power on
    nowUs = 1000 * 1000;
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_BOOT_GRACE_TIME));

    nowUs = 3 * 1000 * 1000;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_BOOT_GRACE_TIME));
}

TEST_F(ArmingTest, ThrottleMustBeOff)
{
    throttleOff = false;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_THROTTLE));
    EXPECT_FALSE(switchArm());

    throttleOff = true;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_THROTTLE));
}

TEST_F(ArmingTest, MustBeUpright)
{
    upright = false;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_ANGLE));
    upright = true;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_ANGLE));
}

TEST_F(ArmingTest, GyroCalibration)
{
    gyroCalibrated = false;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_CALIBRATING));
    gyroCalibrated = true;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_CALIBRATING));
}

TEST_F(ArmingTest, AccMustBeCalibrated)
{
    sensorsSet(SENSOR_ACC);
    accCalibrated = false;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_ACC_CALIBRATION));
    accCalibrated = true;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_ACC_CALIBRATION));
}

TEST_F(ArmingTest, HighLoad)
{
    systemLoad = 800;    // 80 %
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_LOAD));
    systemLoad = 700;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_LOAD));
}

TEST_F(ArmingTest, Overrides)
{
    mixerOverride = true;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_OVERRIDE));
    mixerOverride = false;
    servoOverride = true;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_OVERRIDE));
    servoOverride = false;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_OVERRIDE));
}

TEST_F(ArmingTest, MotorProtocolCauses)
{
    EXPECT_EQ(MOTOR_PROTO_CAUSE_NONE, getMotorProtocolCause());

    motorProtocolEnabled = false;
    EXPECT_EQ(MOTOR_PROTO_CAUSE_DISABLED, getMotorProtocolCause());
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_MOTOR_PROTOCOL));

    motorProtocolEnabled = true;
    motorInitFailed = true;
    EXPECT_EQ(MOTOR_PROTO_CAUSE_INIT_FAILED, getMotorProtocolCause());

    // Motorized tail needs a second motor output
    motorInitFailed = false;
    mixerConfigMutable()->tail_rotor_mode = TAIL_MODE_MOTORIZED;
    EXPECT_EQ(MOTOR_PROTO_CAUSE_NO_TAIL_MOTOR, getMotorProtocolCause());
    motorCount = 2;
    EXPECT_EQ(MOTOR_PROTO_CAUSE_NONE, getMotorProtocolCause());
}

TEST_F(ArmingTest, ArmSwitchOnWhileBlocked)
{
    // Switch flipped on while arming is blocked: it has to go off and on again
    throttleOff = false;
    EXPECT_FALSE(switchArm());
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_ARM_SWITCH));

    throttleOff = true;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_ARM_SWITCH));
    tryArm();
    EXPECT_FALSE(ARMING_FLAG(ARMED));

    setArmSwitch(false);
    update();
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_TRUE(switchArm());
}

TEST_F(ArmingTest, BlockedArmBeepsReasonOnce)
{
    throttleOff = false;
    switchArm();
    tryArm();
    tryArm();
    // ffs() of the throttle flag, beeped only once for the same reason
    ASSERT_EQ(1u, warningBeeps.size());
    EXPECT_EQ(8, warningBeeps[0]);
}

TEST_F(ArmingTest, RxRecoveryWithArmSwitchOn)
{
    // Link drops with the switch on, then comes back
    haveRx = false;
    setArmSwitch(true);
    update();

    haveRx = true;
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_BAD_RX_RECOVERY));
    tryArm();
    EXPECT_FALSE(ARMING_FLAG(ARMED));

    // Cleared once the switch is seen off with a good link
    setArmSwitch(false);
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_BAD_RX_RECOVERY));
}

TEST_F(ArmingTest, FailsafeSwitch)
{
    addMode(BOXFAILSAFE, 1);
    setAux(1, true);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_BOXFAILSAFE));
    setAux(1, false);
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_BOXFAILSAFE));
}

TEST_F(ArmingTest, RescueSwitch)
{
    addMode(BOXRESCUE, 1);
    setAux(1, true);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_RESC));
    setAux(1, false);
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_RESC));
}

TEST_F(ArmingTest, ParalyzeIsPermanent)
{
    addMode(BOXPARALYZE, 1);
    setAux(1, false);
    update();               // seen off after the boot delay
    setAux(1, true);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_PARALYZE));

    setAux(1, false);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_PARALYZE));
    EXPECT_FALSE(switchArm());
}

TEST_F(ArmingTest, Prearm)
{
    addMode(BOXPREARM, 1);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_NOPREARM));

    setAux(1, true);
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_NOPREARM));
    ASSERT_TRUE(switchArm());
    EXPECT_TRUE(ARMING_FLAG(WAS_ARMED_WITH_PREARM));

    // After a flight the prearm switch has to be cycled
    disarm(DISARM_REASON_SWITCH);
    setArmSwitch(false);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_NOPREARM));
}

TEST_F(ArmingTest, GyroCalOnFirstArmIgnoresCalibrating)
{
    // With gyro_cal_on_first_arm, an unfinished calibration doesn't latch the arm switch flag
    armingConfigMutable()->gyro_cal_on_first_arm = 1;
    gyroCalibrated = false;
    setArmSwitch(true);
    update();
    EXPECT_TRUE(blockedBy(ARMING_DISABLED_CALIBRATING));
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_ARM_SWITCH));
}

TEST_F(ArmingTest, NoChecksWhileArmed)
{
    ASSERT_TRUE(switchArm());
    throttleOff = false;
    upright = false;
    update();
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_THROTTLE));
    EXPECT_FALSE(blockedBy(ARMING_DISABLED_ANGLE));
    EXPECT_TRUE(ARMING_FLAG(ARMED));
}
