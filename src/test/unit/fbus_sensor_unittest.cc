/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/fbus_master.h"
#include "pg/gps.h"

#include "drivers/fbus_sensor.h"
#include "io/gps.h"

// the storage PG_REGISTER would define (its designated initializer is not C++)
fbusMasterConfig_t fbusMasterConfig_System;
fbusMasterConfig_t fbusMasterConfig_Copy;

static timeUs_t testTimeUs = 1000;

timeUs_t micros(void) { return testTimeUs; }
timeMs_t millis(void) { return testTimeUs / 1000; }
void fbusXactTrackServo(uint8_t, uint16_t, timeUs_t) {}

// GPS (an FBUS GPS feeds it; not tested here)
uint8_t atomic_BASEPRI;
gpsConfig_t gpsConfig_System;
gpsData_t gpsData;
gpsSolutionData_t gpsSol;
uint8_t GPS_update;
bool gpsUsesFbusTransport(void) { return false; }
void gpsSetFixState(bool) {}
void sensorsClear(uint32_t) {}
}

// An RB on 0x19: battery 0x0F00 now and then, acceleration Z 0x0F60 all the time
static const uint8_t RB = 0x19;
static const uint16_t RB_BATT = 0x0F00;
static const uint16_t RB_ACCZ = 0x0F60;

class FbusSensorForwardTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        testTimeUs = 1000;
        memset(fbusMasterConfigMutable(), 0, sizeof(fbusMasterConfig_t));
        for (int i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
            fbusMasterConfigMutable()->forwardedSensors[i] = FBUS_INVALID_PHYSICAL_ID;
        }
        fbusMasterConfigMutable()->forwardedSensors[0] = RB;
        fbusSensorInit();
    }

    static fbusSensorFrame_t next(uint8_t physicalId)
    {
        fbusSensorFrame_t frame = {};
        EXPECT_TRUE(fbusSensorGetForwardedFrame(physicalId, &frame));
        return frame;
    }
};

TEST_F(FbusSensorForwardTest, NotForwardedIsNotAnswered)
{
    fbusSensorProcessData(0x05, 0x0210, 123);

    fbusSensorFrame_t frame;
    EXPECT_FALSE(fbusSensorGetForwardedFrame(0x05, &frame));
    EXPECT_FALSE(fbusSensorPeekForwardedFrame(0x05, &frame));
}

TEST_F(FbusSensorForwardTest, NothingReceivedIsNotAnswered)
{
    fbusSensorFrame_t frame;
    EXPECT_FALSE(fbusSensorGetForwardedFrame(RB, &frame));
}

TEST_F(FbusSensorForwardTest, FrequentValueDoesNotCrowdOutOthers)
{
    fbusSensorProcessData(RB, RB_BATT, 74);
    for (int i = 0; i < 20; i++) {
        fbusSensorProcessData(RB, RB_ACCZ, 1000 + i);
    }

    const fbusSensorFrame_t first = next(RB);
    const fbusSensorFrame_t second = next(RB);
    EXPECT_TRUE(first.valid);
    EXPECT_TRUE(second.valid);
    EXPECT_NE(first.appId, second.appId);
    EXPECT_TRUE(first.appId == RB_BATT || second.appId == RB_BATT);

    // only the latest acceleration value is kept
    const fbusSensorFrame_t accz = first.appId == RB_ACCZ ? first : second;
    EXPECT_EQ(1019u, accz.data);
}

TEST_F(FbusSensorForwardTest, EmptyFrameWhenNothingNew)
{
    fbusSensorProcessData(RB, RB_BATT, 74);
    EXPECT_TRUE(next(RB).valid);

    // still sending, nothing new: an empty answer keeps the sensor on the radio
    fbusSensorProcessData(RB, 0, 0);
    const fbusSensorFrame_t frame = next(RB);
    EXPECT_FALSE(frame.valid);
}

TEST_F(FbusSensorForwardTest, ValuesTakeTurns)
{
    for (uint16_t i = 0; i < 9; i++) {
        fbusSensorProcessData(RB, 0x0F00 + 0x10 * i, i);
    }

    // a new acceleration value after each send must not starve the rest
    uint16_t seen = 0;
    for (int i = 0; i < 9; i++) {
        const fbusSensorFrame_t frame = next(RB);
        ASSERT_TRUE(frame.valid);
        seen |= 1 << ((frame.appId - 0x0F00) / 0x10);
        fbusSensorProcessData(RB, RB_ACCZ, 2000 + i);
    }
    EXPECT_EQ(0x1FF, seen);
}

TEST_F(FbusSensorForwardTest, SilentSensorStopsBeingAnswered)
{
    fbusSensorProcessData(RB, RB_BATT, 74);
    testTimeUs += FBUS_FORWARDED_STALE_US + 1;

    fbusSensorFrame_t frame;
    EXPECT_FALSE(fbusSensorGetForwardedFrame(RB, &frame));
}

TEST_F(FbusSensorForwardTest, PeekShowsLatestWithoutTakingIt)
{
    fbusSensorProcessData(RB, RB_BATT, 74);
    fbusSensorProcessData(RB, RB_ACCZ, 1000);

    fbusSensorFrame_t frame;
    ASSERT_TRUE(fbusSensorPeekForwardedFrame(RB, &frame));
    EXPECT_TRUE(frame.valid);
    EXPECT_EQ(RB_ACCZ, frame.appId);
    EXPECT_EQ(1000u, frame.data);

    // both values are still to be sent
    EXPECT_TRUE(next(RB).valid);
    EXPECT_TRUE(next(RB).valid);
    EXPECT_FALSE(next(RB).valid);
}

TEST_F(FbusSensorForwardTest, FullTableReusesSentValues)
{
    for (uint16_t i = 0; i < FBUS_FORWARDED_VALUES; i++) {
        fbusSensorProcessData(RB, 0x6800 + i, i);
    }

    // all unsent: a new App ID is dropped
    fbusSensorProcessData(RB, 0x7000, 1);
    for (int i = 0; i < FBUS_FORWARDED_VALUES; i++) {
        const fbusSensorFrame_t frame = next(RB);
        ASSERT_TRUE(frame.valid);
        EXPECT_NE(0x7000, frame.appId);
    }

    // once sent, a slot is reused
    fbusSensorProcessData(RB, 0x7000, 1);
    const fbusSensorFrame_t frame = next(RB);
    EXPECT_TRUE(frame.valid);
    EXPECT_EQ(0x7000, frame.appId);
}

TEST_F(FbusSensorForwardTest, FindsDeviceForDeviceConfig)
{
    fbusSensorProcessData(RB, RB_BATT, 74);
    fbusSensorProcessData(0x0C, 0x6806, 1);

    EXPECT_TRUE(fbusSensorIsObservedOnFbus(RB));
    EXPECT_TRUE(fbusSensorIsObservedOnFbus(0x0C));
    EXPECT_FALSE(fbusSensorIsObservedOnFbus(0x0D));

    EXPECT_EQ(RB, fbusSensorFindFbusPhysicalId(RB_BATT));
    EXPECT_EQ(0x0C, fbusSensorFindFbusPhysicalId(0x6806));
    EXPECT_EQ(-1, fbusSensorFindFbusPhysicalId(0x6807));
    EXPECT_EQ(-1, fbusSensorFindFbusPhysicalId(0));
}

TEST_F(FbusSensorForwardTest, SportSensorIsNotOnTheFbusLine)
{
    fbusSensorProcessDataWithSource(0x05, 0x0210, 123, FBUS_SENSOR_SOURCE_SPORT);

    EXPECT_FALSE(fbusSensorIsObservedOnFbus(0x05));
    EXPECT_EQ(-1, fbusSensorFindFbusPhysicalId(0x0210));
}
