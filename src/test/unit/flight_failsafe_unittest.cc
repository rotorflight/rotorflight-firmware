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
 * Rotorflight drives failsafe through the RX link tracking in failsafe.c:
 * rx.c reports valid/failed frames, the scheduler checks the failure period,
 * and the result gates arming. The stage 2 state machine in
 * failsafeUpdateState() only runs while monitoring, and
 * failsafeStartMonitoring() is currently disabled, so it is covered here
 * only by a test that pins that down.
 */

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"

#include "pg/pg.h"
#include "pg/failsafe.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/beeper.h"

uint32_t failsafeFailurePeriodMs(void);
}

#include "gtest/gtest.h"

static timeMs_t nowMs;
static bool receivingSignal;
static int disarmCount;

extern "C" {
timeMs_t millis(void) { return nowMs; }
bool rxIsReceivingSignal(void) { return receivingSignal; }

bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool isThrottleOff(void) { return true; }
bool isHandsOn(void) { return false; }
void disarm(flightLogDisarmReason_e) { disarmCount++; }
void beeper(beeperMode_e) {}
void beeperConfirmationBeeps(uint8_t) {}
}

class FailsafeTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        nowMs = 10000;
        receivingSignal = false;
        disarmCount = 0;
        armingFlags = 0;
        flightModeFlags = 0;
        unsetArmingDisabled((armingDisableFlags_e)0xFFFFFFFF);

        failsafeInit();
        failsafeReset();
    }

    // rx.c reports valid frames every 10 ms while the link is good
    void receive(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 10) {
            nowMs += 10;
            failsafeOnValidDataReceived();
            failsafeCheckDataFailurePeriod();
        }
    }

    // ...and failed frames while it is not
    void lose(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 10) {
            nowMs += 10;
            failsafeOnValidDataFailed();
            failsafeCheckDataFailurePeriod();
        }
    }

    // No frame-level reports at all, only the scheduler check
    void silence(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 10) {
            nowMs += 10;
            failsafeCheckDataFailurePeriod();
        }
    }

    static bool armingBlockedBy(armingDisableFlags_e flag)
    {
        return getArmingDisableFlags() & flag;
    }

    // Defaults: failsafe_delay 1.5 s, failsafe_recovery_delay 1.0 s
    static constexpr uint32_t FAILURE_MS = 1500;
    static constexpr uint32_t RECOVERY_MS = 1000;
};

TEST_F(FailsafeTest, InitialState)
{
    EXPECT_FALSE(failsafeIsReceivingRxData());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_FALSE(failsafeIsMonitoring());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(FAILURE_MS, failsafeFailurePeriodMs());
}

TEST_F(FailsafeTest, PeriodsHaveA200msMinimum)
{
    failsafeConfigMutable()->failsafe_delay = 0;
    failsafeConfigMutable()->failsafe_recovery_delay = 0;
    failsafeReset();
    EXPECT_EQ(200u, failsafeFailurePeriodMs());

    // Link comes up after just over 200 ms of valid data
    receive(200);
    EXPECT_FALSE(failsafeIsReceivingRxData());
    receive(20);
    EXPECT_TRUE(failsafeIsReceivingRxData());
}

TEST_F(FailsafeTest, LinkUpAfterRecoveryPeriod)
{
    receive(10);
    EXPECT_FALSE(failsafeIsReceivingRxData());
    // Arming stays blocked until the link has been good for the recovery period
    EXPECT_TRUE(armingBlockedBy(ARMING_DISABLED_BST));
    EXPECT_FALSE(armingBlockedBy(ARMING_DISABLED_RX_FAILSAFE));

    receive(RECOVERY_MS - 10);
    EXPECT_FALSE(failsafeIsReceivingRxData());

    // Strictly longer than the recovery period
    receive(20);
    EXPECT_TRUE(failsafeIsReceivingRxData());
    EXPECT_FALSE(armingBlockedBy(ARMING_DISABLED_BST));
}

TEST_F(FailsafeTest, ShortDropoutKeepsLink)
{
    receive(2000);
    ASSERT_TRUE(failsafeIsReceivingRxData());

    // Failed frames block arming at once, but the link holds for the failsafe delay
    lose(FAILURE_MS - 10);
    EXPECT_TRUE(failsafeIsReceivingRxData());
    EXPECT_TRUE(armingBlockedBy(ARMING_DISABLED_RX_FAILSAFE));

    // A good frame clears the arming block and the link was never lost
    receive(10);
    EXPECT_TRUE(failsafeIsReceivingRxData());
    EXPECT_FALSE(armingBlockedBy(ARMING_DISABLED_RX_FAILSAFE));
}

TEST_F(FailsafeTest, LinkDownAfterFailurePeriod)
{
    receive(2000);
    ASSERT_TRUE(failsafeIsReceivingRxData());

    lose(FAILURE_MS + 10);
    EXPECT_FALSE(failsafeIsReceivingRxData());
    EXPECT_TRUE(armingBlockedBy(ARMING_DISABLED_RX_FAILSAFE));
}

TEST_F(FailsafeTest, RecoveryNeedsFullPeriodOfGoodData)
{
    receive(2000);
    lose(FAILURE_MS + 10);
    ASSERT_FALSE(failsafeIsReceivingRxData());

    // Good data, but not yet for long enough
    receive(RECOVERY_MS / 2);
    EXPECT_FALSE(failsafeIsReceivingRxData());

    // Another dropout restarts the recovery period
    lose(50);
    receive(RECOVERY_MS - 10);
    EXPECT_FALSE(failsafeIsReceivingRxData());
    receive(20);
    EXPECT_TRUE(failsafeIsReceivingRxData());
}

TEST_F(FailsafeTest, SchedulerCheckDetectsSilence)
{
    // No reports at all (e.g. the receiver stopped sending): the scheduler check drops the link
    receive(2000);
    ASSERT_TRUE(failsafeIsReceivingRxData());

    silence(FAILURE_MS);
    EXPECT_TRUE(failsafeIsReceivingRxData());

    silence(10);
    EXPECT_FALSE(failsafeIsReceivingRxData());
    EXPECT_TRUE(armingBlockedBy(ARMING_DISABLED_RX_FAILSAFE));
}

TEST_F(FailsafeTest, RxSuspendExtendsFailurePeriod)
{
    receive(2000);

    // An EEPROM write suspends RX for 1.5 s; that time does not count as loss
    failsafeOnRxSuspend(1500 * 1000);
    silence(FAILURE_MS + 1000);
    EXPECT_TRUE(failsafeIsReceivingRxData());

    silence(600);
    EXPECT_FALSE(failsafeIsReceivingRxData());
}

TEST_F(FailsafeTest, RxResumeWithSignalRestoresLink)
{
    receive(2000);
    lose(FAILURE_MS + 10);
    ASSERT_FALSE(failsafeIsReceivingRxData());

    receivingSignal = true;
    failsafeOnRxResume();
    EXPECT_TRUE(failsafeIsReceivingRxData());

    // ...and restarts the failure timer
    silence(FAILURE_MS);
    EXPECT_TRUE(failsafeIsReceivingRxData());
}

TEST_F(FailsafeTest, RxResumeWithoutSignalKeepsLinkDown)
{
    // A config write after disarm must not report a dead link as recovered
    receive(2000);
    lose(FAILURE_MS + 10);
    ASSERT_FALSE(failsafeIsReceivingRxData());

    receivingSignal = false;
    failsafeOnRxResume();
    EXPECT_FALSE(failsafeIsReceivingRxData());
}

TEST_F(FailsafeTest, StateMachineIsNotMonitoring)
{
    // failsafeStartMonitoring() is disabled in Rotorflight, so the stage 2
    // state machine never runs: losing the link while armed changes nothing.
    // If monitoring is re-enabled this test needs to be replaced.
    failsafeStartMonitoring();
    EXPECT_FALSE(failsafeIsMonitoring());

    ENABLE_ARMING_FLAG(ARMED);
    receive(2000);
    lose(FAILURE_MS + 10);
    for (int i = 0; i < 500; i++) {
        nowMs += 10;
        failsafeUpdateState();
    }

    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_FALSE(FLIGHT_MODE(FAILSAFE_MODE));
    EXPECT_EQ(0, disarmCount);
}
