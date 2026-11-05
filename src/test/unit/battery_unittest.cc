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
 * sensors/battery.c with the ADC voltage and current sensors stubbed:
 * presence and cell count detection, voltage alarm hysteresis, consumption
 * alarms, charge level, battery profiles and config validation.
 */

#include <stdint.h>
#include <stdbool.h>
#include <vector>

extern "C" {
#include "platform.h"

#include "build/debug.h"

#include "pg/pg.h"
#include "pg/battery.h"

#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

int get_ADJUSTMENT_BATTERY_PROFILE(void);
}

#include "gtest/gtest.h"

// Sensor readings, set by the tests
static uint32_t voltageMv;
static uint32_t currentMa;
static uint32_t capacityMah;

static timeMs_t nowMs;
static std::vector<beeperMode_e> beeps;

extern "C" {
uint8_t armingFlags;
uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];

timeMs_t millis(void) { return nowMs; }
void beeper(beeperMode_e mode) { beeps.push_back(mode); }

void voltageSensorADCInit(void) {}
void voltageSensorADCRefresh(void) {}
bool voltageSensorADCRead(voltageSensorADC_e, voltageMeter_t *meter)
{
    meter->sample = voltageMv;
    meter->voltage = voltageMv;
    return true;
}
void voltageMeterReset(voltageMeter_t *meter)
{
    meter->sample = 0;
    meter->voltage = 0;
}

void currentSensorADCInit(void) {}
void currentSensorADCRefresh(timeUs_t) {}
bool currentSensorADCRead(currentSensorADC_e, currentMeter_t *meter)
{
    meter->sample = currentMa;
    meter->current = currentMa;
    meter->capacity = capacityMah;
    return true;
}
void currentMeterReset(currentMeter_t *meter)
{
    meter->sample = 0;
    meter->current = 0;
    meter->capacity = 0;
}
}

class BatteryTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
        batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
        armingFlags = 0;
        voltageMv = 0;
        currentMa = 0;
        capacityMah = 0;
        nowMs = 1000;
        beeps.clear();
        batteryInit();
    }

    // Run the voltage, current and alert tasks for a while at 50 Hz
    void run(uint32_t mv, int samples = 200)
    {
        voltageMv = mv;
        for (int i = 0; i < samples; i++) {
            nowMs += 20;
            taskBatteryVoltageUpdate(nowMs * 1000);
            taskBatteryCurrentUpdate(nowMs * 1000);
            taskBatteryAlerts(nowMs * 1000);
        }
    }
};

TEST_F(BatteryTest, InitialState)
{
    EXPECT_EQ(BATTERY_INIT, getBatteryState());
    EXPECT_EQ(0, getBatteryCellCount());
    EXPECT_TRUE(isBatteryVoltageConfigured());
    EXPECT_STREQ("INIT", getBatteryStateString());
}

TEST_F(BatteryTest, NoBattery)
{
    run(0);
    EXPECT_EQ(BATTERY_NOT_PRESENT, getBatteryState());
    EXPECT_EQ(0, getBatteryCellCount());
    EXPECT_EQ(0, getBatteryAverageCellVoltage());
}

TEST_F(BatteryTest, UsbPowerIsNotABattery)
{
    // ~5 V from USB is between one cell's maximum and two "absent" cells
    run(5000);
    EXPECT_EQ(BATTERY_NOT_PRESENT, getBatteryState());
}

TEST_F(BatteryTest, VoltageUnits)
{
    run(12600);
    EXPECT_EQ(1260, getBatteryVoltage());        // 0.01 V
    EXPECT_EQ(126, getLegacyBatteryVoltage());   // 0.1 V
    EXPECT_EQ(420, getBatteryAverageCellVoltage());
}

struct CellCase { uint32_t mv; uint8_t cells; };

class BatteryCellCountTest : public BatteryTest, public ::testing::WithParamInterface<CellCase> {};

TEST_P(BatteryCellCountTest, AutoDetect)
{
    run(GetParam().mv);
    EXPECT_EQ(BATTERY_OK, getBatteryState());
    EXPECT_EQ(GetParam().cells, getBatteryCellCount());
}

// Charged and storage-level packs (default 3.30..4.30 V per cell)
INSTANTIATE_TEST_SUITE_P(Packs, BatteryCellCountTest, ::testing::Values(
    CellCase{  4200,  1 },
    CellCase{  7600,  2 },
    CellCase{ 12600,  3 },
    CellCase{ 11400,  3 },
    CellCase{ 16800,  4 },
    CellCase{ 25200,  6 },
    CellCase{ 22800,  6 },
    CellCase{ 33600,  8 },
    CellCase{ 42000, 10 },
    CellCase{ 50400, 12 },
    CellCase{ 45600, 12 }
));

TEST_F(BatteryTest, ProfileCellCountOverridesDetection)
{
    batteryConfigMutable()->batteryCellCount[0] = 6;
    run(16800);    // would detect as 4S
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(280, getBatteryAverageCellVoltage());
}

TEST_F(BatteryTest, VoltageAlarmsWithHysteresis)
{
    // 3S: warning at 3 x 3.50 V, critical at 3 x 3.30 V, hysteresis 0.01 V.
    // The filtered voltage is truncated to 0.01 V, so stay 20 mV off each edge.
    run(12000);
    ASSERT_EQ(3, getBatteryCellCount());
    EXPECT_EQ(BATTERY_OK, getBatteryState());

    run(10520);
    EXPECT_EQ(BATTERY_OK, getBatteryState());
    run(10480);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());

    // Must rise above the warning voltage, not just above the hysteresis, to clear
    run(10495);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());
    run(10520);
    EXPECT_EQ(BATTERY_OK, getBatteryState());

    run(10000);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());
    run(9880);
    EXPECT_EQ(BATTERY_CRITICAL, getBatteryState());

    run(9895);
    EXPECT_EQ(BATTERY_CRITICAL, getBatteryState());
    run(9920);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());
}

TEST_F(BatteryTest, WarningNeedsSustainedLowVoltage)
{
    batteryConfigMutable()->vbatDurationForWarning = 10;   // 1 s
    run(12000);

    run(10400, 25);    // 0.5 s
    EXPECT_EQ(BATTERY_OK, getBatteryState());
    run(10400, 50);    // another 1 s
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());
}

TEST_F(BatteryTest, VoltageAlertBeeps)
{
    run(12000);
    EXPECT_TRUE(beeps.empty());

    run(10400, 10);
    ASSERT_FALSE(beeps.empty());
    EXPECT_EQ(BEEPER_BAT_LOW, beeps.back());

    run(9800, 10);
    EXPECT_EQ(BEEPER_BAT_CRIT_LOW, beeps.back());

    batteryConfigMutable()->useVoltageAlerts = false;
    beeps.clear();
    run(9800, 10);
    EXPECT_TRUE(beeps.empty());
}

TEST_F(BatteryTest, ChargeLevelFromCapacity)
{
    batteryConfigMutable()->batteryCapacity[0] = 2000;
    capacityMah = 500;
    run(12000);

    EXPECT_EQ(75, calculateBatteryPercentageRemaining());
    EXPECT_EQ(75, getBatteryChargeLevel());
    EXPECT_EQ(500u, getBatteryCapacityUsed());

    capacityMah = 2500;
    run(12000, 1);
    EXPECT_EQ(0, calculateBatteryPercentageRemaining());
}

TEST_F(BatteryTest, ChargeLevelFromVoltage)
{
    // No capacity set: 3.80 V per cell between 3.30 and 4.30 is 50 %
    run(11400);
    ASSERT_EQ(3, getBatteryCellCount());
    EXPECT_EQ(50, calculateBatteryPercentageRemaining());
    EXPECT_TRUE(isBatteryChargeLevelAvailable());
}

TEST_F(BatteryTest, ConsumptionAlarms)
{
    batteryConfigMutable()->useConsumptionAlerts = true;
    batteryConfigMutable()->batteryCapacity[0] = 1000;
    run(12000);
    EXPECT_EQ(BATTERY_OK, getConsumptionState());

    capacityMah = 700;    // 30 % left, below the 35 % warning
    run(12000, 2);
    EXPECT_EQ(BATTERY_WARNING, getConsumptionState());
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());

    capacityMah = 1000;
    run(12000, 2);
    EXPECT_EQ(BATTERY_CRITICAL, getConsumptionState());
    EXPECT_EQ(BATTERY_CRITICAL, getBatteryState());
    EXPECT_EQ(BATTERY_OK, getVoltageState());
}

TEST_F(BatteryTest, CurrentUnits)
{
    currentMa = 25000;
    run(12000);
    EXPECT_EQ(2500, getBatteryCurrent());         // 0.01 A
    EXPECT_EQ(250, getLegacyBatteryCurrent());    // 0.1 A
}

TEST_F(BatteryTest, DisconnectWhileDisarmed)
{
    run(12000);
    ASSERT_EQ(3, getBatteryCellCount());

    run(0);
    EXPECT_EQ(BATTERY_NOT_PRESENT, getBatteryState());
    EXPECT_EQ(0, getBatteryCellCount());

    // Reconnecting a different pack detects it again
    run(16800);
    EXPECT_EQ(4, getBatteryCellCount());
}

TEST_F(BatteryTest, NoPresenceChangesWhileArmed)
{
    run(12000);
    ENABLE_ARMING_FLAG(ARMED);

    run(0);
    EXPECT_EQ(3, getBatteryCellCount());
    EXPECT_EQ(BATTERY_CRITICAL, getBatteryState());
}

TEST_F(BatteryTest, ProfileChangeWhenDisarmed)
{
    batteryConfigMutable()->batteryCellCount[1] = 6;
    run(16800);
    ASSERT_EQ(4, getBatteryCellCount());

    changeBatteryProfile(1);
    EXPECT_EQ(1, getCurrentBatteryProfileIndex());
    run(16800, 5);
    EXPECT_EQ(6, getBatteryCellCount());
}

TEST_F(BatteryTest, ProfileChangeWhenArmedWaitsForDisarm)
{
    batteryConfigMutable()->batteryCellCount[1] = 6;
    run(16800);
    ENABLE_ARMING_FLAG(ARMED);

    changeBatteryProfile(1);
    EXPECT_EQ(0, getCurrentBatteryProfileIndex());
    EXPECT_EQ(2, get_ADJUSTMENT_BATTERY_PROFILE());   // pending, 1-based
    run(16800, 5);
    EXPECT_EQ(4, getBatteryCellCount());

    DISABLE_ARMING_FLAG(ARMED);
    run(16800, 5);
    EXPECT_EQ(1, getCurrentBatteryProfileIndex());
    EXPECT_EQ(6, getBatteryCellCount());

    // Out of range is ignored
    changeBatteryProfile(BATTERY_PROFILE_COUNT);
    EXPECT_EQ(1, getCurrentBatteryProfileIndex());
}

TEST_F(BatteryTest, ValidateConfig)
{
    batteryConfig_t *config = batteryConfigMutable();
    config->batteryProfile = BATTERY_PROFILE_COUNT;
    config->batteryCellCount[0] = BATTERY_CELL_COUNT_MAX + 1;
    config->vbatmincellvoltage[1] = 450;          // min above max
    config->vbatmaxcellvoltage[1] = 420;
    config->vbatfullcellvoltage[2] = 440;         // full above max
    config->vbatwarningcellvoltage[3] = 300;      // warning below min

    validateAndFixBatteryConfig();

    EXPECT_EQ(0, config->batteryProfile);
    EXPECT_EQ(0, config->batteryCellCount[0]);
    EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_MIN, config->vbatmincellvoltage[1]);
    EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_MAX, config->vbatmaxcellvoltage[1]);
    EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_FULL, config->vbatfullcellvoltage[2]);
    EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_WARN, config->vbatwarningcellvoltage[3]);

    // Ordering min <= warning <= full <= max holds for every profile
    for (int i = 0; i < BATTERY_PROFILE_COUNT; i++) {
        EXPECT_LE(config->vbatmincellvoltage[i], config->vbatwarningcellvoltage[i]) << i;
        EXPECT_LE(config->vbatwarningcellvoltage[i], config->vbatfullcellvoltage[i]) << i;
        EXPECT_LE(config->vbatfullcellvoltage[i], config->vbatmaxcellvoltage[i]) << i;
    }
}
