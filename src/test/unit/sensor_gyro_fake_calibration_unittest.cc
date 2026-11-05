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

// gyro.c built with FAKE_GYRO_SKIP_CALIBRATION, as on a fake-gyro target
// (SITL): starting a calibration must complete it at once, otherwise the
// CALIB arming-disable flag never clears.

#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/sensor.h"
    #include "io/beeper.h"
    #include "pg/pg.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/gyro_init.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SensorGyroFakeCalibration, CompletesWhenStarted)
{
    pgResetAll();
    ASSERT_TRUE(gyroInit());
    ASSERT_EQ(GYRO_FAKE, gyro.gyroSensor1.gyroDev.gyroHardware);
    gyroSetLooptime(1, 1);

    gyroStartCalibration(false);
    EXPECT_TRUE(gyroIsCalibrationComplete());
}

TEST(SensorGyroFakeCalibration, CompletesOnFirstArmingCalibration)
{
    pgResetAll();
    ASSERT_TRUE(gyroInit());
    gyroSetLooptime(1, 1);

    gyroStartCalibration(true);
    EXPECT_TRUE(gyroIsCalibrationComplete());
    EXPECT_FALSE(isFirstArmingGyroCalibrationRunning());
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
