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
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/governor.h"
#include "flight/rescue.h"
#include "pg/governor.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "telemetry/status.h"

uint8_t armingFlags;
uint8_t stateFlags;
uint16_t flightModeFlags;

// Everything status.c reads, reset per test.
static struct {
    bool airborne, motorsRunning, rxLink;
    failsafePhase_e failsafe;
    bool gpsHealthy, spooledUp;
    uint32_t sensorMask;
    batteryState_e battery;
    bool saturationLatch;
    timeMs_t now;
    bool gyroOverflow, accCalibrated;
    bool servoOverride, mixerOverride, motorOverride;
    rescueState_e rescue;
    govState_e governor;
    bool blackboxLogging, blackboxFull;
    uint8_t pidProfile, ratesProfile, batteryProfile;
    bool dirty, saving, reboot, beeper, rpm;
} fc;

bool isAirborne(void) { return fc.airborne; }
bool areMotorsRunning(void) { return fc.motorsRunning; }
uint8_t getMotorCount(void) { return 2; }
bool hasMotorOverride(uint8_t motor) { return fc.motorOverride && motor == 1; }
bool isRpmSourceActive(void) { return fc.rpm; }
bool rxIsReceivingSignal(void) { return fc.rxLink; }
failsafePhase_e failsafePhase(void) { return fc.failsafe; }
bool gpsIsHealthy(void) { return fc.gpsHealthy; }
bool isSpooledUp(void) { return fc.spooledUp; }
bool sensors(uint32_t mask) { return fc.sensorMask & mask; }
batteryState_e getBatteryState(void) { return fc.battery; }
bool mixerTakeStabilizedSaturation(void)
{
    const bool latched = fc.saturationLatch;
    fc.saturationLatch = false;
    return latched;
}
timeMs_t millis(void) { return fc.now; }
bool gyroOverflowDetected(void) { return fc.gyroOverflow; }
bool accHasBeenCalibrated(void) { return fc.accCalibrated; }
bool isServoOverrideActive(void) { return fc.servoOverride; }
bool isMixerOverrideActive(void) { return fc.mixerOverride; }
int getRescueState(void) { return fc.rescue; }
int getGovernorState(void) { return fc.governor; }
bool blackboxIsLogging(void) { return fc.blackboxLogging; }
bool isBlackboxDeviceFull(void) { return fc.blackboxFull; }
uint8_t getCurrentPidProfileIndex(void) { return fc.pidProfile; }
uint8_t getCurrentControlRateProfileIndex(void) { return fc.ratesProfile; }
uint8_t getCurrentBatteryProfileIndex(void) { return fc.batteryProfile; }
bool isConfigDirty(void) { return fc.dirty; }
bool isEepromWriteInProgress(void) { return fc.saving; }
bool getRebootRequired(void) { return fc.reboot; }
bool isBeeperOn(void) { return fc.beeper; }
}

class TelemetryStatusTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // The saturation hold is static state in status.c: move the clock well past any hold
        // left by the previous test and let it expire before starting.
        static timeMs_t clock = 1000000;
        clock += 10000;

        memset(&fc, 0, sizeof(fc));
        armingFlags = 0;
        stateFlags = 0;
        governorConfigMutable()->gov_mode = GOV_MODE_NONE;
        fc.accCalibrated = true;
        fc.now = clock;
        telemetrySystemStatus();
    }
};

// Pins every bit position: the radio Lua decodes by position, so a move here must be matched
// in the Lua suites.
TEST_F(TelemetryStatusTest, StatusLayoutIsFixed)
{
    armingFlags = ARMED;
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    fc.airborne = fc.motorsRunning = fc.rxLink = true;
    fc.failsafe = FAILSAFE_GPS_RESCUE;                // 6
    fc.gpsHealthy = true;
    fc.spooledUp = true;
    fc.battery = BATTERY_CRITICAL;                    // 2
    fc.saturationLatch = true;
    fc.gyroOverflow = true;
    fc.servoOverride = true;
    fc.rescue = RESCUE_STATE_EXIT;                    // 5
    fc.blackboxLogging = true;
    fc.governor = GOV_STATE_BYPASS;                   // 9

    const uint32_t expected =
        (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | // armed, airborne, motors, RX link
        (6u << 6) |                                   // failsafe phase
        (2u << 9) | (1u << 11) |                      // fix + home, GPS healthy
        (1u << 12) |                                  // spooled up
        (2u << 14) |                                  // battery critical
        (1u << 17) | (1u << 18) |                     // saturated, gyro overflow
        (1u << 20) |                                  // override
        (5u << 21) |                                  // rescue exit
        (1u << 24) |                                  // blackbox logging
        (9u << 25);                                   // governor bypass

    EXPECT_EQ(expected, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, IdleModelReportsNothing)
{
    EXPECT_EQ(0u, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, GpsFixField)
{
    auto fix = [] { return (telemetrySystemStatus() >> TELEM_STATUS_GPS_FIX_SHIFT) & TELEM_STATUS_GPS_FIX_MASK; };

    EXPECT_EQ(TELEM_GPS_FIX_NONE, fix());
    stateFlags = GPS_FIX;
    EXPECT_EQ(TELEM_GPS_FIX_OK, fix());
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    EXPECT_EQ(TELEM_GPS_FIX_HOME, fix());
}

TEST_F(TelemetryStatusTest, ControlSaturationIsHeldBriefly)
{
    fc.saturationLatch = true;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    fc.now += 499;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    fc.now += 1;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    // Once expired it stays clear, even after millis() wraps past the old timestamp.
    fc.now += 0x80000000u;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);
}

TEST_F(TelemetryStatusTest, AccNotCalibratedOnlyWithAcc)
{
    fc.accCalibrated = false;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_ACC_NOT_CALIBRATED);
    fc.sensorMask = SENSOR_ACC;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_ACC_NOT_CALIBRATED);
}

TEST_F(TelemetryStatusTest, AnyOverrideSetsItsBit)
{
    fc.motorOverride = true;
    EXPECT_EQ(TELEM_STATUS_OVERRIDE_ACTIVE, telemetrySystemStatus());
    fc.motorOverride = false;
    fc.mixerOverride = true;
    EXPECT_EQ(TELEM_STATUS_OVERRIDE_ACTIVE, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, ConfigLayoutIsFixed)
{
    fc.pidProfile = 5;          // reported 1-based
    fc.ratesProfile = 0;
    fc.batteryProfile = 2;
    fc.dirty = fc.saving = fc.reboot = fc.beeper = true;
    fc.sensorMask = SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_GPS;
    fc.blackboxFull = true;
    fc.rpm = true;
    governorConfigMutable()->gov_mode = GOV_MODE_NITRO; // 4

    const uint32_t expected =
        (6u << 0) | (1u << 3) | (3u << 6) |
        (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15) |
        (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) |
        (1u << 21) | (1u << 22) |
        (4u << 23);

    EXPECT_EQ(expected, telemetrySystemConfig());
}

TEST_F(TelemetryStatusTest, SignBitIsNeverSet)
{
    for (bool *flag : {&fc.airborne, &fc.motorsRunning, &fc.rxLink, &fc.gpsHealthy, &fc.spooledUp,
            &fc.saturationLatch, &fc.gyroOverflow, &fc.servoOverride, &fc.mixerOverride,
            &fc.motorOverride, &fc.blackboxLogging, &fc.blackboxFull,
            &fc.dirty, &fc.saving, &fc.reboot, &fc.beeper, &fc.rpm}) {
        *flag = true;
    }
    fc.sensorMask = UINT32_MAX;
    fc.failsafe = FAILSAFE_GPS_RESCUE;
    fc.battery = BATTERY_INIT;
    fc.rescue = RESCUE_STATE_EXIT;
    fc.governor = GOV_STATE_BYPASS;
    fc.pidProfile = fc.ratesProfile = fc.batteryProfile = 5;
    governorConfigMutable()->gov_mode = GOV_MODE_NITRO;
    armingFlags = 0xFF;
    stateFlags = 0xFF;

    EXPECT_FALSE(telemetrySystemStatus() & 0x80000000u);
    EXPECT_FALSE(telemetrySystemConfig() & 0x80000000u);
}
