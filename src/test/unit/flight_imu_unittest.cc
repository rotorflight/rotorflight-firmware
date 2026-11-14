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
#include <cmath>

extern "C" {
#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "pg/pg.h"
#include "pg/imu.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

void imuComputeRotationMatrix(void);
void imuUpdateEulerAngles(void);

extern quaternion q;
extern bool attitudeIsEstablished;
}

#include "gtest/gtest.h"

static const float sqrt2over2 = sqrtf(2) / 2.0f;

// Sensor inputs, set by the tests
static float accInput[XYZ_AXIS_COUNT];
static bool  accHasData;
static uint32_t sensorMask;

extern "C" {
gyro_t gyro;
acc_t acc;
mag_t mag;
gpsSolutionData_t gpsSol;

uint8_t stateFlags;
uint8_t armingFlags;

bool sensors(uint32_t mask) { return sensorMask & mask; }

bool accGetAccumulationAverage(float *average)
{
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        average[i] = accInput[i];
    return accHasData;
}

bool compassIsHealthy(void) { return false; }
bool gpsRescueDisableMag(void) { return false; }
void schedulerIgnoreTaskStateTime(void) {}
timeUs_t micros(void) { return 0; }
timeMs_t millis(void) { return 0; }
}

static void setQ(float w, float x, float y, float z)
{
    q.w = w; q.x = x; q.y = y; q.z = z;
}

/** Quaternion -> rotation matrix -> Euler angles **/

TEST(FlightImuTest, RotationMatrixIdentity)
{
    setQ(1, 0, 0, 0);
    imuComputeRotationMatrix();

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EXPECT_FLOAT_EQ(i == j ? 1.0f : 0.0f, rMat[i][j]) << i << "," << j;
}

TEST(FlightImuTest, RotationMatrixYaw90)
{
    setQ(sqrt2over2, 0, 0, sqrt2over2);
    imuComputeRotationMatrix();

    const float expected[3][3] = { { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EXPECT_NEAR(expected[i][j], rMat[i][j], 1e-6f) << i << "," << j;
}

TEST(FlightImuTest, RotationMatrixRoll60)
{
    setQ(0.866f, 0.5f, 0, 0);
    imuComputeRotationMatrix();

    const float expected[3][3] = { { 1, 0, 0 }, { 0, 0.5f, -0.866f }, { 0, 0.866f, 0.5f } };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EXPECT_NEAR(expected[i][j], rMat[i][j], 1e-3f) << i << "," << j;
}

TEST(FlightImuTest, EulerAnglesLevel)
{
    setQ(1, 0, 0, 0);
    imuComputeRotationMatrix();
    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(0, attitude.values.yaw);
}

TEST(FlightImuTest, EulerAnglesYaw)
{
    // Yaw is reported 0..3600
    memset(rMat, 0, sizeof(rMat));
    rMat[0][0] = sqrt2over2;
    rMat[0][1] = sqrt2over2;
    rMat[1][0] = -sqrt2over2;
    rMat[1][1] = sqrt2over2;
    rMat[2][2] = 1;
    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(450, attitude.values.yaw);

    rMat[1][0] = sqrt2over2;
    rMat[0][1] = -sqrt2over2;
    imuUpdateEulerAngles();
    EXPECT_EQ(3150, attitude.values.yaw);
}

TEST(FlightImuTest, EulerAnglesRoll)
{
    // 30 degrees about X
    setQ(cosf(M_PIf / 12), sinf(M_PIf / 12), 0, 0);
    imuComputeRotationMatrix();
    imuUpdateEulerAngles();

    EXPECT_NEAR(300, attitude.values.roll, 1);
    EXPECT_NEAR(0, attitude.values.pitch, 1);
}

/** Upright / upside down **/

TEST(FlightImuTest, UprightAndUpsideDown)
{
    sensorMask = SENSOR_ACC;

    attitudeIsEstablished = true;
    setQ(1, 0, 0, 0);
    imuComputeRotationMatrix();
    EXPECT_FLOAT_EQ(1.0f, getCosTiltAngle());
    EXPECT_TRUE(isUpright());
    EXPECT_FALSE(isUpsidedown());

    // Rolled 180 degrees
    setQ(0, 1, 0, 0);
    imuComputeRotationMatrix();
    EXPECT_FLOAT_EQ(-1.0f, getCosTiltAngle());
    EXPECT_TRUE(isUpsidedown());

    // No attitude yet: not upright, and not known to be upside down
    attitudeIsEstablished = false;
    EXPECT_FALSE(isUpright());
    EXPECT_FALSE(isUpsidedown());

    // Without an accelerometer there is no attitude to wait for
    sensorMask = 0;
    EXPECT_TRUE(isUpright());
    EXPECT_FALSE(isUpsidedown());
}

/** Attitude estimator **/

class ImuEstimatorTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        imuConfigure();
        imuInit();

        memset(&gyro, 0, sizeof(gyro));
        memset(&acc, 0, sizeof(acc));
        acc.dev.acc_1G_rec = 1.0f;    // acc input in g
        acc.isAccelUpdatedAtLeastOnce = true;
        sensorMask = SENSOR_ACC;
        accHasData = true;
        armingFlags = 0;

        // The estimator keeps its last update time and the attitude across
        // tests: run one level step so neither depends on the test before
        setQ(1, 0, 0, 0);
        imuComputeRotationMatrix();
        setGravity(0, 0, 1);
        now += 1000;
        imuUpdateAttitude(now);
        setQ(1, 0, 0, 0);
        imuComputeRotationMatrix();
    }

    void setGravity(float x, float y, float z)
    {
        accInput[X] = x;
        accInput[Y] = y;
        accInput[Z] = z;
    }

    // Run the estimator at 1 kHz
    void run(float seconds)
    {
        for (int i = 0; i < seconds * 1000; i++) {
            now += 1000;
            imuUpdateAttitude(now);
        }
    }

    static timeUs_t now;
};

timeUs_t ImuEstimatorTest::now = 0;

TEST_F(ImuEstimatorTest, StaysLevel)
{
    setGravity(0, 0, 1);
    run(1);

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_NEAR(1.0f, getCosTiltAngle(), 1e-4f);
}

TEST_F(ImuEstimatorTest, ConvergesToRoll)
{
    setGravity(0, sinf(DEGREES_TO_RADIANS(30)), cosf(DEGREES_TO_RADIANS(30)));
    run(5);

    EXPECT_NEAR(300, attitude.values.roll, 2);
    EXPECT_NEAR(0, attitude.values.pitch, 2);
}

TEST_F(ImuEstimatorTest, ConvergesToPitch)
{
    setGravity(sinf(DEGREES_TO_RADIANS(20)), 0, cosf(DEGREES_TO_RADIANS(20)));
    run(5);

    EXPECT_NEAR(0, attitude.values.roll, 2);
    EXPECT_NEAR(-200, attitude.values.pitch, 2);
}

TEST_F(ImuEstimatorTest, IgnoresAccOutsideOneG)
{
    // 1.5 g is rejected as unhealthy: attitude is held, not pulled to the tilt
    setGravity(0, 1.5f * sinf(DEGREES_TO_RADIANS(30)), 1.5f * cosf(DEGREES_TO_RADIANS(30)));
    run(2);

    EXPECT_EQ(0, attitude.values.roll);
}

TEST_F(ImuEstimatorTest, IntegratesGyro)
{
    // No acc correction: one second at +90 deg/s turns 90 degrees. Gyro Z is
    // right-handed (counter-clockwise from above), heading grows clockwise.
    accHasData = false;
    gyro.gyroADCf[Z] = 90;
    run(1);

    EXPECT_NEAR(2700, attitude.values.yaw, 5);
    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
}

TEST_F(ImuEstimatorTest, NoAccelerometer)
{
    // Without an accelerometer the estimator does not run
    sensorMask = 0;
    gyro.gyroADCf[X] = 100;
    run(1);

    EXPECT_EQ(0, attitude.values.roll);
}
