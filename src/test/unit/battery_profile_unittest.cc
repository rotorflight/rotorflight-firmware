#include "gmock/gmock.h"
#include "gtest/gtest.h"

extern "C" {
#include "pg/battery.h"
#include "pg/system.h"
#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/voltage.h"
#include "pg/pg.h"
}

// Dummy functions
extern "C" {
#include "io/beeper.h"
uint8_t armingFlags = 0;
void beeper(beeperMode_e) {}

// voltage sensor
void voltageSensorADCInit(void) {}
void voltageSensorADCRefresh(void) {}
void voltageMeterReset(voltageMeter_t *) {}

// current sensor
void currentSensorADCInit(void) {}
void currentSensorADCRefresh(timeUs_t) {}
void currentMeterReset(currentMeter_t *) {}

uint32_t millis(void) { return 0; }
}

// Mocks
class MockInterface {
  public:
    MOCK_METHOD(bool, voltageSensorADCRead,
                (voltageSensorADC_e sensor, voltageMeter_t *meter), ());
    MOCK_METHOD(bool, currentSensorADCRead,
                (currentSensorADC_e sensor, currentMeter_t *meter), ());
} *g_mock = nullptr;

extern "C" {
bool voltageSensorADCRead(voltageSensorADC_e sensor, voltageMeter_t *meter)
{
    return g_mock->voltageSensorADCRead(sensor, meter);
}
bool currentSensorADCRead(currentSensorADC_e sensor, currentMeter_t *meter)
{
    return g_mock->currentSensorADCRead(sensor, meter);
}
}

using ::testing::_;
using ::testing::StrictMock;

TEST(SanityCheck, Init) { batteryInit(); }

class BatteryProfileTestBase : public ::testing::Test {
  public:
    void SetUp() override
    {
        pgResetAll();
        g_mock = &mock;
        batteryInit();
    }
    void TearDown() override { g_mock = nullptr; }
    StrictMock<MockInterface> mock;
};

class BatteryProfileSwitchTest : public BatteryProfileTestBase {
  public:
    void SetUp() override
    {
        BatteryProfileTestBase::SetUp();
        /*
         * Set up battery profiles:
         *    0: 0mAh, 0 cells (voltage mapped capacity, auto cell count)
         *    1: 1000mAh, 1 cell
         *    ....
         *    5: 5000mAh, 5 cells
         */
        for (int i = 0; i < 6; i++) {
            batteryProfile_t *profile =
                &batteryConfigMutable()->batteryProfiles[i];
            profile->batteryCapacity = i * 1000;
            profile->batteryCellCount = i;
        }
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
        batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
        // Disable LPF
        batteryConfigMutable()->vbatLpfHz = 0;
        batteryConfigMutable()->ibatLpfHz = 0;

        batteryInit();
    }
};

TEST_F(BatteryProfileSwitchTest, SwitchCellCount)
{
    // Return 12V stable
    EXPECT_CALL(mock, voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, _))
        .WillRepeatedly([](voltageSensorADC_e, voltageMeter_t *meter) {
            // Unit is in mV integer.
            meter->sample = 12000;
            meter->voltage = 12000;
            return true;
        });

    // Initially profile = 3;
    systemConfigMutable()->batteryProfileIndex = 3;
    batteryInit();
    // The states take a few iteration to transit.
    for (int i = 0; i < 10; i++) {
        taskBatteryVoltageUpdate(0);
        taskBatteryAlerts(0);
    }

    // Unit is in 10mV integer.
    EXPECT_EQ(getBatteryAverageCellVoltage(), 1200 / 3);
    EXPECT_EQ(getBatteryCellCount(), 3);
    // 4V per cell, normal
    EXPECT_EQ(getBatteryState(), BATTERY_OK);

    // Switch to profile 5
    systemConfigMutable()->batteryProfileIndex = 5;
    loadBatteryProfile();

    for (int i = 0; i < 10; i++) {
        taskBatteryVoltageUpdate(0);
        taskBatteryAlerts(0);
    }

    // Unit is in 10mV integer.
    EXPECT_EQ(getBatteryAverageCellVoltage(), 1200 / 5);
    EXPECT_EQ(getBatteryCellCount(), 5);
    // 2.4V per cell, critically low
    EXPECT_EQ(getBatteryState(), BATTERY_CRITICAL);
}

TEST_F(BatteryProfileSwitchTest, SwitchCapacity)
{
    /*
     * currentSensorADCRead sets:
     *   * meter->sample in mA
     *   * meter->current in mA
     *   * meter->capacity in mAh
     */
    EXPECT_CALL(mock, currentSensorADCRead(CURRENT_SENSOR_ADC_BAT, _))
        .WillRepeatedly([](currentSensorADC_e, currentMeter_t *meter) {
            meter->sample = 1000;
            meter->current = 1000;
            meter->capacity = 3000;
            return true;
        });

    batteryConfigMutable()->useVoltageAlerts = false;
    batteryConfigMutable()->useConsumptionAlerts = true;

    // Initially profile = 3;
    systemConfigMutable()->batteryProfileIndex = 3;
    batteryInit();
    for (int i = 0; i < 10; i++) {
        taskBatteryCurrentUpdate(0);
        taskBatteryAlerts(0);
    }

    EXPECT_EQ(getBatteryCapacity(), 3000);
    EXPECT_EQ(getBatteryCapacityUsed(), 3000);
    EXPECT_EQ(calculateBatteryPercentageRemaining(), 0);
    EXPECT_EQ(getConsumptionState(), BATTERY_CRITICAL);

    // Switch to profile 5
    systemConfigMutable()->batteryProfileIndex = 5;
    loadBatteryProfile();
    for (int i = 0; i < 10; i++) {
        taskBatteryCurrentUpdate(0);
        taskBatteryAlerts(0);
    }

    EXPECT_EQ(getBatteryCapacity(), 5000);
    EXPECT_EQ(getBatteryCapacityUsed(), 3000);
    EXPECT_EQ(calculateBatteryPercentageRemaining(), 100 - 3000 * 100 / 5000);
    EXPECT_EQ(getConsumptionState(), BATTERY_OK);
}

TEST_F(BatteryProfileSwitchTest, SwitchCellCountReGuess)
{
    // Return 12V stable
    EXPECT_CALL(mock, voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, _))
        .WillRepeatedly([](voltageSensorADC_e, voltageMeter_t *meter) {
            meter->sample = 12000;
            meter->voltage = 12000;
            return true;
        });

    // Return 3000 used capacity
    EXPECT_CALL(mock, currentSensorADCRead(CURRENT_SENSOR_ADC_BAT, _))
        .WillRepeatedly([](currentSensorADC_e, currentMeter_t *meter) {
            meter->sample = 1000;
            meter->current = 1000;
            meter->capacity = 3000;
            return true;
        });

    systemConfigMutable()->batteryProfileIndex = 3;
    batteryConfigMutable()->batteryProfiles[3].batteryCellCount = 0;
    batteryConfigMutable()->batteryProfiles[5].batteryCellCount = 0;

    batteryInit();
    for (int i = 0; i < 10; i++) {
        taskBatteryVoltageUpdate(0);
        taskBatteryCurrentUpdate(0);
        taskBatteryAlerts(0);
    }

    // Should guess 3 cells
    EXPECT_EQ(getBatteryCellCount(), 3);

    // Change to 24V stable
    EXPECT_CALL(mock, voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, _))
        .WillRepeatedly([](voltageSensorADC_e, voltageMeter_t *meter) {
            meter->sample = 24000;
            meter->voltage = 24000;
            return true;
        });

    for (int i = 0; i < 10; i++) {
        taskBatteryVoltageUpdate(0);
        taskBatteryCurrentUpdate(0);
        taskBatteryAlerts(0);
    }

    // Should NOT update the guess (yet).
    EXPECT_EQ(getBatteryCellCount(), 3);

    // Switch profile
    systemConfigMutable()->batteryProfileIndex = 5;
    loadBatteryProfile();
    for (int i = 0; i < 10; i++) {
        taskBatteryVoltageUpdate(0);
        taskBatteryCurrentUpdate(0);
        taskBatteryAlerts(0);
    }

    // Now it should guess 6 cells.
    EXPECT_EQ(getBatteryCellCount(), 6);
}
