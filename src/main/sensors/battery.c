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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/fbus_sensor.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/motors.h"

#include "io/beeper.h"

#include "pg/battery.h"
#include "pg/telemetry.h"

#include "scheduler/scheduler.h"

#include "sensors/battery.h"


/**
 * terminology: meter vs sensors
 *
 * voltage and current sensors are used to collect data.
 * - e.g. voltage at an MCU ADC input pin, value from an ESC sensor.
 *   sensors require very specific configuration, such as resistor values.
 *
 * voltage and current meters are used to process and expose data collected from sensors to the rest of the system.
 * - e.g. a meter exposes normalized, and often filtered, values from a sensor.
 *   meters require different or little configuration.
 *   meters also have different precision concerns, and may use different units to the sensors.
 *
 */

#define VBAT_STABLE_MAX_DELTA           200         // mV
#define LVC_AFFECT_TIME                 10000000    // 10 secs for the LVC to slowly kick in
#define SMARTFUEL_MAX_SAG_COMP_V        0.5f
#define SMARTFUEL_STABLE_SAMPLE_COUNT   5


const char * const batteryVoltageSourceNames[VOLTAGE_METER_COUNT] = {
    [VOLTAGE_METER_NONE]    = "NONE",
    [VOLTAGE_METER_ADC]     = "ADC",
    [VOLTAGE_METER_ESC]     = "ESC",
    [VOLTAGE_METER_FBUS]    = "FBUS",
};

const char * const batteryCurrentSourceNames[CURRENT_METER_COUNT] = {
    [CURRENT_METER_NONE]    = "NONE",
    [CURRENT_METER_ADC]     = "ADC",
    [CURRENT_METER_ESC]     = "ESC",
    [CURRENT_METER_FBUS]    = "FBUS",
};


// Note: Cell count can be 0 when no battery is detected or
//       when the battery voltage sensor is missing or disabled
static uint8_t batteryCellCount;

static uint32_t batteryVoltage;
static uint32_t batteryCurrent;

static filter_t currentFilter;
static filter_t voltageFilter;

static voltageMeter_t voltageMeter;
static currentMeter_t currentMeter;

static lowVoltageCutoff_t lowVoltageCutoff;

static uint16_t batteryWarningVoltage;
static uint16_t batteryCriticalVoltage;
static uint16_t batteryWarningHysteresisVoltage;
static uint16_t batteryCriticalHysteresisVoltage;

static batteryState_e batteryState;
static batteryState_e voltageState;
static batteryState_e consumptionState;

typedef struct smartFuelConfigSig_s {
    uint8_t profile;
    uint8_t cellCount;
    uint8_t source;
    uint8_t reserved;
    uint16_t capacity;
    uint16_t reserve;
    uint16_t vbatMin;
    uint16_t vbatMax;
    uint16_t vbatFull;
} smartFuelConfigSig_t;

typedef struct smartFuelState_s {
    bool voltageStabilized;
    bool voltageFiltered;
    bool startStateValid;
    bool lastPercentValid;
    bool lastHeadspeedValid;
    bool wasArmed;
    bool hadFlight;
    timeUs_t stabilizeNotBeforeUs;
    timeUs_t lastUpdateUs;
    uint8_t voltageSampleCount;
    float voltageSamples[SMARTFUEL_STABLE_SAMPLE_COUNT];
    float filteredVoltage;
    float startPercent;
    float startConsumption;
    float lastPercent;
    float lastHeadspeed;
    uint8_t percent;
    smartFuelConfigSig_t config;
} smartFuelState_t;

#ifdef USE_SMARTFUEL
static smartFuelState_t smartFuel = INIT_ZERO;

static uint16_t smartFuelVoltageParam(unsigned index, uint16_t fallback)
{
    if (index < SMARTFUEL_PARAM_COUNT) {
        return telemetryConfig()->smartfuel_params[index];
    }

    return fallback;
}

static timeUs_t smartFuelStabilizeDelayUs(void)
{
    return (timeUs_t)smartFuelVoltageParam(SMARTFUEL_PARAM_STABILIZE_DELAY_MS, SMARTFUEL_STABILIZE_DELAY_DEFAULT_MS) * 1000;
}

static float smartFuelStableWindowVolts(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_STABLE_WINDOW_CV, SMARTFUEL_STABLE_WINDOW_DEFAULT_CV) / 100.0f;
}

static float smartFuelVoltageFallLimitPerSecond(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_VOLTAGE_FALL_CVPS, SMARTFUEL_VOLTAGE_FALL_LIMIT_DEFAULT_CVPS) / 100.0f;
}

static float smartFuelDropPerSecond(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_FUEL_DROP_TENTHS_PERCENT_PER_S, SMARTFUEL_FUEL_DROP_RATE_DEFAULT_TENTHS_PERCENT_PER_S) / 10.0f;
}

static float smartFuelRisePerSecond(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_FUEL_RISE_TENTHS_PERCENT_PER_S, SMARTFUEL_FUEL_RISE_RATE_DEFAULT_TENTHS_PERCENT_PER_S) / 10.0f;
}

static float smartFuelSagMultiplier(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_SAG_MULTIPLIER_PERCENT, SMARTFUEL_SAG_MULTIPLIER_DEFAULT_PERCENT) / 100.0f;
}

static smartFuelSource_e smartFuelGetConfiguredSource(void)
{
    return (smartFuelSource_e)telemetryConfig()->smartfuel_source;
}

static smartFuelConfigSig_t smartFuelGetConfig(void)
{
    return (smartFuelConfigSig_t) {
        .profile = batteryConfig()->batteryProfile,
        .cellCount = batteryCellCount,
        .source = smartFuelGetConfiguredSource(),
        .reserved = 0,
        .capacity = getBatteryCapacity(),
        .reserve = batteryConfig()->consumptionWarningPercentage,
        .vbatMin = batteryConfig()->vbatmincellvoltage,
        .vbatMax = batteryConfig()->vbatmaxcellvoltage,
        .vbatFull = batteryConfig()->vbatfullcellvoltage,
    };
}

static void smartFuelResetState(timeUs_t currentTimeUs)
{
    smartFuel.voltageStabilized = false;
    smartFuel.voltageFiltered = false;
    smartFuel.startStateValid = false;
    smartFuel.lastPercentValid = false;
    smartFuel.lastHeadspeedValid = false;
    smartFuel.hadFlight = false;
    smartFuel.stabilizeNotBeforeUs = currentTimeUs + smartFuelStabilizeDelayUs();
    smartFuel.lastUpdateUs = currentTimeUs;
    smartFuel.voltageSampleCount = 0;
    memset(smartFuel.voltageSamples, 0, sizeof(smartFuel.voltageSamples));
    smartFuel.filteredVoltage = 0.0f;
    smartFuel.startPercent = 0.0f;
    smartFuel.startConsumption = 0.0f;
    smartFuel.lastPercent = 0.0f;
    smartFuel.lastHeadspeed = 0.0f;
    smartFuel.percent = 0;
}

static uint8_t smartFuelClampReserve(uint8_t reserve)
{
    if (reserve < 15 || reserve > 60) {
        return 35;
    }

    return reserve;
}

static float smartFuelGetUsableCapacity(void)
{
    const float packCapacity = getBatteryCapacity();
    if (packCapacity < 10.0f) {
        return 0.0f;
    }

    float usableCapacity = packCapacity * (1.0f - smartFuelClampReserve(batteryConfig()->consumptionWarningPercentage) / 100.0f);
    if (usableCapacity < 10.0f) {
        usableCapacity = packCapacity;
    }

    return usableCapacity;
}

static float smartFuelPercentFromVoltageOnly(float voltage, uint8_t cellCount)
{
    if (cellCount == 0) {
        return 0.0f;
    }

    const float fullCellVoltage = batteryConfig()->vbatfullcellvoltage / 100.0f;
    const float minCellVoltage = batteryConfig()->vbatmincellvoltage / 100.0f;
    const float reserve = smartFuelClampReserve(batteryConfig()->consumptionWarningPercentage) / 100.0f;
    const float adjustedMinVoltage = minCellVoltage + (fullCellVoltage - minCellVoltage) * reserve * 1.4f;
    const float usableSpan = fullCellVoltage - adjustedMinVoltage;

    if (usableSpan <= 0.0f) {
        return 0.0f;
    }

    const float voltagePerCell = constrainf(voltage / cellCount, 3.3f, fullCellVoltage);
    const float scaledVoltage = constrainf(
        3.3f + ((voltagePerCell - adjustedMinVoltage) / usableSpan) * 0.9f,
        3.3f, 4.2f);

    return constrainf((scaledVoltage - 3.3f) * (100.0f / 0.9f), 0.0f, 100.0f);
}

static void smartFuelAddVoltageSample(float voltage)
{
    if (smartFuel.voltageSampleCount < SMARTFUEL_STABLE_SAMPLE_COUNT) {
        smartFuel.voltageSamples[smartFuel.voltageSampleCount++] = voltage;
        return;
    }

    memmove(&smartFuel.voltageSamples[0], &smartFuel.voltageSamples[1],
        sizeof(smartFuel.voltageSamples[0]) * (SMARTFUEL_STABLE_SAMPLE_COUNT - 1));
    smartFuel.voltageSamples[SMARTFUEL_STABLE_SAMPLE_COUNT - 1] = voltage;
}

static bool smartFuelVoltageIsStable(void)
{
    if (smartFuel.voltageSampleCount < SMARTFUEL_STABLE_SAMPLE_COUNT) {
        return false;
    }

    float minVoltage = smartFuel.voltageSamples[0];
    float maxVoltage = smartFuel.voltageSamples[0];

    for (unsigned i = 1; i < smartFuel.voltageSampleCount; i++) {
        minVoltage = MIN(minVoltage, smartFuel.voltageSamples[i]);
        maxVoltage = MAX(maxVoltage, smartFuel.voltageSamples[i]);
    }

    return (maxVoltage - minVoltage) <= smartFuelStableWindowVolts();
}

static float smartFuelApplySagCompensation(float voltage)
{
    if (!ARMING_FLAG(ARMED)) {
        smartFuel.lastHeadspeedValid = false;
        return voltage;
    }

    const float roll = fabsf(mixerGetInput(MIXER_IN_STABILIZED_ROLL));
    const float pitch = fabsf(mixerGetInput(MIXER_IN_STABILIZED_PITCH));
    const float collective = fabsf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE));
    const float stickLoad = constrainf(roll + pitch + collective * 1.2f, 0.0f, 1.0f);

    float rpmDrop = 0.0f;
    const float headSpeed = getHeadSpeed();
    if (headSpeed >= 100.0f) {
        if (smartFuel.lastHeadspeedValid && smartFuel.lastHeadspeed > 0.0f) {
            rpmDrop = MAX((smartFuel.lastHeadspeed - headSpeed) / smartFuel.lastHeadspeed, 0.0f);
        }
        smartFuel.lastHeadspeed = headSpeed;
        smartFuel.lastHeadspeedValid = true;
    } else {
        smartFuel.lastHeadspeedValid = false;
    }

    const float sagFactor = MAX(stickLoad, rpmDrop);
    const float compensationScale = powf(smartFuelSagMultiplier(), 1.5f);

    return voltage + compensationScale * sagFactor * SMARTFUEL_MAX_SAG_COMP_V;
}

static void smartFuelUpdate(timeUs_t currentTimeUs)
{
    const bool armed = ARMING_FLAG(ARMED);
    const smartFuelConfigSig_t config = smartFuelGetConfig();

    if (memcmp(&smartFuel.config, &config, sizeof(config)) != 0) {
        smartFuel.config = config;
        smartFuelResetState(currentTimeUs);
    }

    if (!isBatteryVoltageConfigured() || batteryCellCount == 0) {
        smartFuel.wasArmed = armed;
        smartFuelResetState(currentTimeUs);
        return;
    }

    if (armed && !smartFuel.wasArmed) {
        smartFuel.hadFlight = true;
    }
    smartFuel.wasArmed = armed;

    const float voltage = getBatteryVoltage() / 100.0f;
    if (voltage < 2.0f) {
        smartFuelResetState(currentTimeUs);
        return;
    }

    const smartFuelSource_e source = smartFuelGetConfiguredSource();
    float percent = smartFuelPercentFromVoltageOnly(voltage, batteryCellCount);

    const float dt = smartFuel.lastUpdateUs ? cmpTimeUs(currentTimeUs, smartFuel.lastUpdateUs) * 1e-6f : 0.0f;
    smartFuel.lastUpdateUs = currentTimeUs;

    const float previousVoltage = smartFuel.voltageSampleCount ?
        smartFuel.voltageSamples[smartFuel.voltageSampleCount - 1] : voltage;

    if (cmpTimeUs(currentTimeUs, smartFuel.stabilizeNotBeforeUs) >= 0) {
        smartFuelAddVoltageSample(voltage);

        if (!smartFuel.voltageStabilized) {
            smartFuel.voltageStabilized = smartFuelVoltageIsStable();
        } else if (!armed && !smartFuel.hadFlight && voltage > previousVoltage + smartFuelStableWindowVolts()) {
            smartFuelResetState(currentTimeUs);
            percent = smartFuelPercentFromVoltageOnly(voltage, batteryCellCount);
        }
    }

    if (!smartFuel.voltageStabilized) {
        smartFuel.lastPercentValid = false;
        return;
    }

    if (source == SMARTFUEL_SOURCE_CURRENT) {
        const float usableCapacity = smartFuelGetUsableCapacity();
        if (usableCapacity < 10.0f) {
            smartFuel.lastPercentValid = false;
            return;
        }

        if (!smartFuel.startStateValid) {
            smartFuel.startPercent = percent;
            smartFuel.startConsumption = getBatteryCapacityUsed();
            smartFuel.startStateValid = true;
        }

        const float usedCapacity = getBatteryCapacityUsed() - smartFuel.startConsumption;
        percent = smartFuel.startPercent - (usedCapacity * 100.0f / usableCapacity);
    } else {
        float filteredVoltage = voltage;
        if (smartFuel.voltageFiltered && dt > 0.0f && voltage < smartFuel.filteredVoltage) {
            filteredVoltage = MAX(voltage, smartFuel.filteredVoltage - dt * smartFuelVoltageFallLimitPerSecond());
        } else {
            smartFuel.voltageFiltered = true;
        }
        smartFuel.filteredVoltage = filteredVoltage;
        percent = smartFuelPercentFromVoltageOnly(smartFuelApplySagCompensation(filteredVoltage), batteryCellCount);

        if ((armed || smartFuel.hadFlight) && smartFuel.lastPercentValid && dt > 0.0f) {
            if (percent < smartFuel.lastPercent) {
                percent = MAX(percent, smartFuel.lastPercent - dt * smartFuelDropPerSecond());
            } else if (percent > smartFuel.lastPercent) {
                percent = MIN(percent, smartFuel.lastPercent + dt * smartFuelRisePerSecond());
            }
        }
    }

    smartFuel.lastPercent = constrainf(percent, 0.0f, 100.0f);
    smartFuel.lastPercentValid = true;
    smartFuel.percent = lrintf(smartFuel.lastPercent);
}
#endif


/** Access function **/

const lowVoltageCutoff_t *getLowVoltageCutoff(void)
{
    return &lowVoltageCutoff;
}

bool isBatteryVoltageConfigured(void)
{
    return batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
}

uint8_t getBatteryVoltageSource(void)
{
    return batteryConfig()->voltageMeterSource;
}

const voltageMeter_t * getBatteryVoltageMeter()
{
    return &voltageMeter;
}

uint16_t getBatteryVoltage(void)
{
    return batteryVoltage / 10;
}

uint16_t getLegacyBatteryVoltage(void)
{
    return batteryVoltage / 100;
}

uint16_t getBatteryVoltageSample(void)
{
    return voltageMeter.sample / 10;
}

uint8_t getBatteryCellCount(void)
{
    return batteryCellCount;
}

uint16_t getBatteryCellVoltage(uint8_t cell)
{
    UNUSED(cell);
    return getBatteryAverageCellVoltage(); // Not implemented
}

uint16_t getBatteryAverageCellVoltage(void)
{
    return (batteryCellCount > 0) ? getBatteryVoltage() / batteryCellCount : 0;
}

bool isBatteryCurrentConfigured(void)
{
    return batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
}

const currentMeter_t * getBatteryCurrentMeter()
{
    return &currentMeter;
}

uint16_t getBatteryCurrent(void) {
    return batteryCurrent / 10;
}

uint16_t getLegacyBatteryCurrent(void)
{
    return batteryCurrent / 100;
}

uint16_t getBatteryCurrentSample(void)
{
    return currentMeter.sample / 10;
}

uint16_t getBatteryCapacity(void)
{
    return batteryConfig()->batteryCapacity[batteryConfig()->batteryProfile];
}

uint32_t getBatteryCapacityUsed(void)
{
    return currentMeter.capacity;
}

int getBatterySmartFuel(void)
{
#ifdef USE_SMARTFUEL
    return smartFuel.lastPercentValid ? smartFuel.percent : -1;
#else
    return -1;
#endif
}

int getBatterySmartConsumption(void)
{
#ifdef USE_SMARTFUEL
    if (smartFuelGetConfiguredSource() == SMARTFUEL_SOURCE_CURRENT) {
        return getBatteryCapacityUsed();
    }

    if (!smartFuel.lastPercentValid) {
        return 0;
    }

    const float usableCapacity = smartFuelGetUsableCapacity();
    if (usableCapacity < 10.0f) {
        return 0;
    }

    const float usedPercent = constrainf(100.0f - smartFuel.lastPercent, 0.0f, 100.0f);
    return lrintf(usedPercent * usableCapacity / 100.0f);
#else
    return getBatteryCapacityUsed();
#endif
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

batteryState_e getVoltageState(void)
{
    return voltageState;
}

batteryState_e getConsumptionState(void)
{
    return consumptionState;
}

static const char * const batteryStateStrings[] = { "OK", "WARNING", "CRITICAL", "NOT PRESENT", "INIT" };

const char * getBatteryStateString(void)
{
    return batteryStateStrings[getBatteryState()];
}


uint8_t calculateBatteryPercentageRemaining(void)
{
    int batteryPercentage = 0;
    int batteryCapacity = getBatteryCapacity();

    if (batteryCapacity > 0) {
        batteryPercentage = 100 * (batteryCapacity - (int)currentMeter.capacity) / batteryCapacity;
    } else if (batteryCellCount > 0) {
        batteryPercentage = 100 * ((int)getBatteryAverageCellVoltage() - (int)batteryConfig()->vbatmincellvoltage) /
            (batteryConfig()->vbatmaxcellvoltage - batteryConfig()->vbatmincellvoltage);
    }

    return constrain(batteryPercentage, 0, 100);
}

void changeBatteryProfile(uint8_t profileIndex)
{
    if (profileIndex < BATTERY_PROFILE_COUNT) {
        batteryConfigMutable()->batteryProfile = profileIndex;
    }
}

uint8_t getCurrentBatteryProfileIndex(void)
{
    return batteryConfig()->batteryProfile;
}

int get_ADJUSTMENT_BATTERY_PROFILE(void)
{
    return getCurrentBatteryProfileIndex() + 1;
}

void set_ADJUSTMENT_BATTERY_PROFILE(int value)
{
    changeBatteryProfile(value - 1);
}


/** Internal functions **/

static void updateBatteryBeeperAlert(void)
{
    switch (getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);
            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);
            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
        case BATTERY_INIT:
            break;
    }
}

static void batteryUpdateAlarms(void)
{
    // use the state to trigger beeper alerts
    if (batteryConfig()->useVoltageAlerts) {
        updateBatteryBeeperAlert();
    }
}

static bool isVoltageStable(void)
{
    return ABS(cmp32(batteryVoltage, voltageMeter.sample)) <= VBAT_STABLE_MAX_DELTA;
}

static bool isVoltageFromBat(void)
{
    const uint32_t voltage = getBatteryVoltage();

    // We want to disable battery getting detected around USB voltage or 0V
    return (voltage >= batteryConfig()->vbatnotpresentcellvoltage         // Above ~0V
            && voltage <= batteryConfig()->vbatmaxcellvoltage)            // 1s max cell voltage check
            || voltage > batteryConfig()->vbatnotpresentcellvoltage * 2;  // USB voltage - 2s or more check
}

void batteryUpdatePresence(void)
{
    if ((voltageState == BATTERY_NOT_PRESENT || voltageState == BATTERY_INIT) && isVoltageFromBat() && isVoltageStable()) {
        // Battery has just been connected - calculate cells, warning voltages and reset state
        consumptionState = voltageState = BATTERY_OK;

        if (batteryConfig()->batteryCellCount != 0) {
            batteryCellCount = batteryConfig()->batteryCellCount;
        }
        else {
            static const unsigned auto_cells[] = { 1, 2, 3, 4, 5, 6, 7, 8, 10, 12 };
            unsigned voltage = getBatteryVoltage();
            batteryCellCount = 1;

            for (unsigned index = 0; index < ARRAYLEN(auto_cells); index++) {
                if (voltage >= auto_cells[index] * batteryConfig()->vbatmincellvoltage &&
                    voltage <= auto_cells[index] * batteryConfig()->vbatmaxcellvoltage) {
                    batteryCellCount = auto_cells[index];
                    break;
                }
            }
        }

        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
        batteryWarningHysteresisVoltage = (batteryWarningVoltage > batteryConfig()->vbathysteresis) ? batteryWarningVoltage - batteryConfig()->vbathysteresis : 0;
        batteryCriticalHysteresisVoltage = (batteryCriticalVoltage > batteryConfig()->vbathysteresis) ? batteryCriticalVoltage - batteryConfig()->vbathysteresis : 0;
        lowVoltageCutoff.percentage = 100;
        lowVoltageCutoff.startTime = 0;
    }
    else if (voltageState != BATTERY_NOT_PRESENT && isVoltageStable() && !isVoltageFromBat()) {
        // battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of batteryConfig()->vbatnotpresentcellvoltage
        consumptionState = voltageState = BATTERY_NOT_PRESENT;

        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
        batteryWarningHysteresisVoltage = 0;
        batteryCriticalHysteresisVoltage = 0;
    }
}

static void batteryUpdateVoltageState(void)
{
    // alerts are currently used by beeper, osd and other subsystems
    static uint32_t lastVoltageChangeMs;

    const uint32_t voltage = getBatteryVoltage();

    switch (voltageState) {
        case BATTERY_OK:
            if (voltage <= batteryWarningHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForWarning * 100) {
                    voltageState = BATTERY_WARNING;
                }
            } else {
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_WARNING:
            if (voltage <= batteryCriticalHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForCritical * 100) {
                    voltageState = BATTERY_CRITICAL;
                }
            } else {
                if (voltage > batteryWarningVoltage) {
                    voltageState = BATTERY_OK;
                }
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_CRITICAL:
            if (voltage > batteryCriticalVoltage) {
                voltageState = BATTERY_WARNING;
                lastVoltageChangeMs = millis();
            }
            break;

        default:
            break;
    }

}

static void batteryUpdateLVC(timeUs_t currentTimeUs)
{
    if (batteryConfig()->lvcPercentage < 100) {
        if (voltageState == BATTERY_CRITICAL && !lowVoltageCutoff.enabled) {
            lowVoltageCutoff.enabled = true;
            lowVoltageCutoff.startTime = currentTimeUs;
            lowVoltageCutoff.percentage = 100;
        }
        if (lowVoltageCutoff.enabled) {
            if (cmp32(currentTimeUs,lowVoltageCutoff.startTime) < LVC_AFFECT_TIME) {
                lowVoltageCutoff.percentage = 100 - (cmp32(currentTimeUs,lowVoltageCutoff.startTime) * (100 - batteryConfig()->lvcPercentage) / LVC_AFFECT_TIME);
            }
            else {
                lowVoltageCutoff.percentage = batteryConfig()->lvcPercentage;
            }
        }
    }
}

static void batteryUpdateConsumptionState(void)
{
    if (batteryConfig()->useConsumptionAlerts && getBatteryCapacity() > 0 && batteryCellCount > 0) {
        uint8_t batteryPercentageRemaining = calculateBatteryPercentageRemaining();

        if (batteryPercentageRemaining == 0) {
            consumptionState = BATTERY_CRITICAL;
        } else if (batteryPercentageRemaining <= batteryConfig()->consumptionWarningPercentage) {
            consumptionState = BATTERY_WARNING;
        } else {
            consumptionState = BATTERY_OK;
        }
    }
}

static void batteryUpdateStates(timeUs_t currentTimeUs)
{
    batteryUpdateVoltageState();
    batteryUpdateConsumptionState();
    batteryUpdateLVC(currentTimeUs);

    batteryState = MAX(voltageState, consumptionState);
}


/** Battery Alert Task **/

void taskBatteryAlerts(timeUs_t currentTimeUs)
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }

    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}


/** Battery Voltage Task **/

void taskBatteryVoltageUpdate(timeUs_t currentTimeUs)
{
    voltageSensorADCRefresh();

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        voltageSensorESCRefresh();
    }
#endif

#ifdef USE_FBUS_MASTER
    voltageSensorFBUSRefresh();
#endif

    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ADC:
            voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, &voltageMeter);
            batteryVoltage = filterApply(&voltageFilter, voltageMeter.sample);
            break;
        case VOLTAGE_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                voltageSensorESCReadTotal(&voltageMeter);
                batteryVoltage = filterApply(&voltageFilter, voltageMeter.sample);
            }
#endif
            break;

        case VOLTAGE_METER_FBUS:
#ifdef USE_FBUS_MASTER
            if (voltageSensorFBUSRead(&voltageMeter)) {
                batteryVoltage = filterApply(&voltageFilter, voltageMeter.sample);
            } else {
                voltageMeterReset(&voltageMeter);
                batteryVoltage = 0;
            }
#else
            voltageMeterReset(&voltageMeter);
            batteryVoltage = 0;
#endif
            break;

        default:
            voltageMeterReset(&voltageMeter);
            batteryVoltage = 0;
            break;
    }

    DEBUG(BATTERY, 0, voltageMeter.sample);
    DEBUG(BATTERY, 1, batteryVoltage);

#ifdef USE_SMARTFUEL
    smartFuelUpdate(currentTimeUs);
#endif
}


/** Battery Current Task **/

void taskBatteryCurrentUpdate(timeUs_t currentTimeUs)
{
    currentSensorADCRefresh(currentTimeUs);

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        currentSensorESCRefresh();
    }
#endif

#ifdef USE_FBUS_MASTER
    currentSensorFBUSRefresh();
#endif

    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentSensorADCRead(CURRENT_SENSOR_ADC_BAT, &currentMeter);
            batteryCurrent = filterApply(&currentFilter, currentMeter.sample);
            break;

        case CURRENT_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                currentSensorESCReadTotal(&currentMeter);
                batteryCurrent = filterApply(&currentFilter, currentMeter.sample);
            }
#endif
            break;

        case CURRENT_METER_FBUS:
#ifdef USE_FBUS_MASTER
            if (currentSensorFBUSRead(&currentMeter)) {
                batteryCurrent = filterApply(&currentFilter, currentMeter.sample);
            } else {
                currentMeterReset(&currentMeter);
                batteryCurrent = 0;
            }
#else
            currentMeterReset(&currentMeter);
            batteryCurrent = 0;
#endif
            break;

        default:
            currentMeterReset(&currentMeter);
            batteryCurrent = 0;
            break;
    }

    DEBUG(BATTERY, 2, currentMeter.sample);
    DEBUG(BATTERY, 3, batteryCurrent);
}


void batteryInit(void)
{
    if (batteryConfig()->batteryProfile >= BATTERY_PROFILE_COUNT) {
        batteryConfigMutable()->batteryProfile = 0;
    }

    voltageMeterReset(&voltageMeter);
    currentMeterReset(&currentMeter);

    voltageSensorADCInit();
    currentSensorADCInit();

#ifdef USE_ESC_SENSOR
    voltageSensorESCInit();
    currentSensorESCInit();
#endif

#ifdef USE_FBUS_MASTER
    voltageSensorFBUSInit();
    currentSensorFBUSInit();
#endif

    lowpassFilterInit(&voltageFilter, LPF_DAMPED,
        batteryConfig()->vbatLpfHz,
        batteryConfig()->vbatUpdateHz, 0);

    lowpassFilterInit(&currentFilter, LPF_DAMPED,
        batteryConfig()->ibatLpfHz,
        batteryConfig()->ibatUpdateHz, 0);

    // presence
    batteryState = BATTERY_INIT;
    batteryCellCount = 0;

    // voltage
    voltageState = BATTERY_INIT;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
    batteryWarningHysteresisVoltage = 0;
    batteryCriticalHysteresisVoltage = 0;
    lowVoltageCutoff.enabled = false;
    lowVoltageCutoff.percentage = 100;
    lowVoltageCutoff.startTime = 0;

    // current
    consumptionState = BATTERY_OK;

#ifdef USE_SMARTFUEL
    memset(&smartFuel, 0, sizeof(smartFuel));
#endif
}
