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

#include <limits.h>
#include <algorithm>

extern "C" {
    #include "platform.h"

    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/accgyro/accgyro_mpu.h"
    #include "drivers/sensor.h"
    #include "io/beeper.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/gyro_init.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"

    gyroHardware_e gyroDetect(gyroDev_t *dev);
    struct gyroSensor_s;
    void performGyroCalibration(struct gyroSensor_s *gyroSensor);
    bool fakeGyroRead(gyroDev_t *gyro);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static gyroDev_t *gyroDev(void) { return gyroActiveDev(); }
static gyroSensor_t *gyroSensor(void) { return &gyro.gyroSensor1; }

TEST(SensorGyro, Detect)
{
    const gyroHardware_e detected = gyroDetect(gyroDev());
    EXPECT_EQ(GYRO_FAKE, detected);
}

TEST(SensorGyro, Init)
{
    pgResetAll();
    const bool initialised = gyroInit();
    EXPECT_TRUE(initialised);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    pgResetAll();
    gyroInit();
    gyroDev_t *dev = gyroDev();
    fakeGyroSet(dev, 5, 6, 7);
    const bool read = dev->readFn(dev);
    EXPECT_TRUE(read);
    EXPECT_EQ(5, dev->gyroADCRaw[X]);
    EXPECT_EQ(6, dev->gyroADCRaw[Y]);
    EXPECT_EQ(7, dev->gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    pgResetAll();
    gyroInit();
    gyroSetLooptime(1, 1);
    gyroConfigMutable()->gyroCalibrationDuration = 125;
    gyroConfigMutable()->gyroMovementCalibrationThreshold = 100;
    gyroStartCalibration(false);
    EXPECT_FALSE(gyroIsCalibrationComplete());
    gyroDev_t *dev = gyroDev();
    fakeGyroSet(dev, 5, 6, 7);
    dev->readFn(dev);
    dev->gyroZero[X] = 8;
    dev->gyroZero[Y] = 9;
    dev->gyroZero[Z] = 10;
    performGyroCalibration(gyroSensor());
    EXPECT_EQ(8, dev->gyroZero[X]);
    EXPECT_EQ(9, dev->gyroZero[Y]);
    EXPECT_EQ(10, dev->gyroZero[Z]);
    static const int maxIterations = 50000;
    int iterations = 0;
    while (!gyroIsCalibrationComplete() && iterations < maxIterations) {
        fakeGyroSet(dev, 5, 6, 7);
        dev->readFn(dev);
        performGyroCalibration(gyroSensor());
        iterations++;
    }
    EXPECT_LT(iterations, maxIterations);
    EXPECT_NEAR(5.0f, dev->gyroZero[X], 1e-3f);
    EXPECT_NEAR(6.0f, dev->gyroZero[Y], 1e-3f);
    EXPECT_NEAR(7.0f, dev->gyroZero[Z], 1e-3f);
}

TEST(SensorGyro, Update)
{
    pgResetAll();
    gyroConfigMutable()->gyroCalibrationDuration = 125;
    gyroConfigMutable()->gyroMovementCalibrationThreshold = 100;
    gyroConfigMutable()->gyro_lpf1_static_hz = 0;
    gyroConfigMutable()->gyro_lpf2_static_hz = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    gyroInit();
    gyroSetLooptime(1, 1);
    gyroDev_t *dev = gyroDev();
    dev->readFn = fakeGyroRead;
    gyroStartCalibration(false);
    EXPECT_FALSE(gyroIsCalibrationComplete());

    fakeGyroSet(dev, 5, 6, 7);
    gyroUpdate();
    static const int maxIterations = 50000;
    int iterations = 0;
    while (!gyroIsCalibrationComplete() && iterations < maxIterations) {
        fakeGyroSet(dev, 5, 6, 7);
        gyroUpdate();
        iterations++;
    }
    EXPECT_TRUE(gyroIsCalibrationComplete());
    EXPECT_NEAR(5.0f, dev->gyroZero[X], 1e-3f);
    EXPECT_NEAR(6.0f, dev->gyroZero[Y], 1e-3f);
    EXPECT_NEAR(7.0f, dev->gyroZero[Z], 1e-3f);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    gyroUpdate();
    // expect zero values since gyro is calibrated
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    fakeGyroSet(dev, 15, 26, 97);
    gyroUpdate();
    EXPECT_NEAR(10 * dev->scale, gyro.gyroADC[X], 1e-3); // gyro.gyroADC values are scaled
    EXPECT_NEAR(20 * dev->scale, gyro.gyroADC[Y], 1e-3);
    EXPECT_NEAR(90 * dev->scale, gyro.gyroADC[Z], 1e-3);
}

// STUBS

extern "C" {

uint32_t micros(void) {return 0;}
void beeper(beeperMode_e) {}
uint8_t detectedSensors[] = { GYRO_NONE, ACC_NONE };
timeDelta_t getGyroUpdateRate(void) {return gyro.targetLooptime;}
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(taskId_e) {}
int getArmingDisableFlags(void) {return 0;}
void writeEEPROM(void) {}
float getFullHeadSpeedRatio(void) {return 1.0f;}
}
