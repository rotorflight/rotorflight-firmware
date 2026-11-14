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

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

float throttle;

// Dummies
extern "C" {
#include "common/filter.h"
#include "flight/pid.h"
#include "pg/pid.h"
#include "sensors/gyro.h"

bool gyroOverflowDetected(void) { return false; }
void beeperConfirmationBeeps(uint8_t) {}
float getThrottle(void) { return throttle; }
void governorInitProfile(const pidProfile_t *) {}
void levelingInit(const pidProfile_t *) {}
void levelingUpdate(void) {}
pidProfile_t *currentPidProfile;
uint8_t getCurrentPidProfileIndex(void) { return 0; }
void changePidProfile(uint8_t) {}
void INIT_CODE rescueInitProfile(const pidProfile_t *) {}
bool isSpooledUp(void) { return true; }
void setpointInitProfile(void) {}
bool isAirborne(void) { return true; }
float getHeadSpeedf(void) { return 0; }
float rescueApply(uint8_t, float setpoint) { return setpoint; }
float angleModeApply(int, float pidSetpoint) { return pidSetpoint; }
float horizonModeApply(int, float pidSetpoint) { return pidSetpoint; }
float getSpoolUpRatio(void) { return 1.0f; }
float mixerGetInput(uint8_t) { return 0.0; }
bool mixerSaturated(uint8_t) { return false; }
} // extern "C"

// Mocks
class MockInterface {
  public:
    MOCK_METHOD(float, getDeflection, (int axis), ());
} *g_mock = nullptr;

extern "C" {
// Fixed 1000Hz
gyro_t gyro = {.targetLooptime = 1000000 / 1000};
pidProfile_t *mockPidProfile = pidProfilesMutable(0);

float getSetpoint(int axis) { return g_mock->getDeflection(axis) * 360; }
float getDeflection(int axis) { return g_mock->getDeflection(axis); }
float getCyclicDeflection() {
    float SP = g_mock->getDeflection(1);
    float SR = g_mock->getDeflection(0);
    return sqrtf(sq(SP) + sq(SR));
}
}

TEST(Empty, BuildTest) {}

// 4xN matrix for PID input/output
using PIDIO = std::array<std::vector<float>, 4>;

using ::testing::StrictMock;

class PIDTestBase : public ::testing::Test {
  public:
    void SetUp() override {
        g_mock = &mock;
        pgResetAll();
        pidInit(mockPidProfile);
    }
    void TearDown() override { g_mock = nullptr; }
    PIDIO getResponse(PIDIO &input)
    {
        PIDIO output;

        uint32_t time = 0;
        for (size_t i = 0; i < input[0].size(); i++) {
            time += gyro.targetLooptime;
            for (int axis = 0; axis < 4; axis++) {
                EXPECT_CALL(mock, getDeflection(axis))
                    .WillRepeatedly(testing::Return(input[axis][i]));
            }
            pidController(mockPidProfile, time);
            for (int axis = 0; axis < 4; axis++) {
                output[axis].push_back(pidGetOutput(axis));
            }
        }

        return output;
    }
    StrictMock<MockInterface> mock;
};

TEST_F(PIDTestBase, Mode0)
// Mode 0 sanity check
{
    mockPidProfile->pid_mode = 0;
    mockPidProfile->pid[0].F = 1;
    pidLoadProfile(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 100; i++) {
        input[0].push_back(0);
        input[1].push_back(0);
        input[2].push_back(0);
        input[3].push_back(0);
    }
    for (int i = 0; i < 100; i++) {
        input[0].push_back(1);
        input[1].push_back(1);
        input[2].push_back(1);
        input[3].push_back(1);
    }

    PIDIO output = getResponse(input);
    for (int i = 0; i < 100; i++) {
        EXPECT_EQ(output[0][i], 0);
    }
    for (int i = 100; i < 200; i++) {
        EXPECT_NE(output[0][i], 0);
    }
}

class PIDFBTest : public PIDTestBase {
  public:
};

TEST_F(PIDFBTest, B)
// Basic B test: with every other term off, the roll output is the B-term,
// Kb times the band-limited derivative of the setpoint
{
    mockPidProfile->pid_mode = 3;
    mockPidProfile->pid[0].P = 0;
    mockPidProfile->pid[0].I = 0;
    mockPidProfile->pid[0].D = 0;
    mockPidProfile->pid[0].F = 0;
    mockPidProfile->pid[0].B = 100;
    mockPidProfile->bterm_cutoff[0] = 30;
    pidLoadProfile(mockPidProfile);

    // Step on roll only, so no cross-coupling from the other axes
    PIDIO input;
    for (int i = 0; i < 1000; i++) {
        input[0].push_back(i < 500 ? 0 : 1);
        input[1].push_back(0);
        input[2].push_back(0);
        input[3].push_back(0);
    }

    PIDIO output = getResponse(input);

    // Expected: the same differentiator on the roll setpoint (360 deg/s per unit)
    const float Kb = ROLL_B_TERM_SCALE * 100;
    difFilter_t bterm;
    difFilterInit(&bterm, 30, 1000);

    for (int i = 0; i < 1000; i++) {
        const float expected = Kb * difFilterApply(&bterm, input[0][i] * 360);
        EXPECT_NEAR(expected, output[0][i], 1e-6f) << "sample " << i;
    }

    // Zero before the step, a positive kick on it, decaying back to zero
    EXPECT_EQ(0, output[0][499]);
    EXPECT_GT(output[0][500], 0);
    EXPECT_LT(output[0][501], output[0][500]);
    EXPECT_NEAR(0, output[0][999], output[0][500] * 1e-3f);
}

TEST_F(PIDFBTest, BScalesWithGain)
// The loader sets Kb from the profile: doubling B doubles the B-term
{
    mockPidProfile->pid_mode = 3;
    mockPidProfile->pid[0].P = 0;
    mockPidProfile->pid[0].I = 0;
    mockPidProfile->pid[0].D = 0;
    mockPidProfile->pid[0].F = 0;
    mockPidProfile->bterm_cutoff[0] = 30;

    PIDIO input;
    for (int i = 0; i < 20; i++) {
        input[0].push_back(i < 10 ? 0 : 1);
        input[1].push_back(0);
        input[2].push_back(0);
        input[3].push_back(0);
    }

    mockPidProfile->pid[0].B = 0;
    pidInit(mockPidProfile);
    PIDIO off = getResponse(input);

    mockPidProfile->pid[0].B = 100;
    pidInit(mockPidProfile);
    PIDIO single = getResponse(input);

    mockPidProfile->pid[0].B = 200;
    pidInit(mockPidProfile);
    PIDIO dbl = getResponse(input);

    for (int i = 0; i < 20; i++) {
        EXPECT_EQ(0, off[0][i]) << "sample " << i;
        EXPECT_NEAR(2 * single[0][i], dbl[0][i], 1e-6f) << "sample " << i;
    }
    EXPECT_GT(single[0][10], 0);
}
