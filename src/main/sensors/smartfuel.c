/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/motors.h"

#include "pg/battery.h"
#include "pg/telemetry.h"

#include "sensors/battery.h"
#include "sensors/smartfuel.h"

#ifdef USE_SMARTFUEL

#define SMARTFUEL_MAX_SAG_COMP_V      0.5f
#define SMARTFUEL_STABLE_SAMPLE_COUNT 5

typedef struct smartFuelConfigSig_s {
    uint8_t enabled;
    uint8_t profile;
    uint8_t cellCount;
    uint8_t source;
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
    bool virtualConsumptionValid;
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
    float virtualConsumption;
    float lastPercent;
    float lastHeadspeed;
    uint8_t percent;
    smartFuelConfigSig_t config;
} smartFuelState_t;

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

static float smartFuelSagMultiplier(void)
{
    return smartFuelVoltageParam(SMARTFUEL_PARAM_SAG_MULTIPLIER_PERCENT, SMARTFUEL_SAG_MULTIPLIER_DEFAULT_PERCENT) / 100.0f;
}

static smartFuelSource_e smartFuelGetConfiguredSource(void)
{
    return (smartFuelSource_e)telemetryConfig()->smartfuel_source;
}

bool smartFuelIsEnabled(void)
{
    return telemetryConfig()->smartfuel;
}

static smartFuelConfigSig_t smartFuelGetConfig(void)
{
    return (smartFuelConfigSig_t) {
        .enabled = smartFuelIsEnabled(),
        .profile = batteryConfig()->batteryProfile,
        .cellCount = getBatteryCellCount(),
        .source = smartFuelGetConfiguredSource(),
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
    smartFuel.virtualConsumptionValid = false;
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
    smartFuel.virtualConsumption = 0.0f;
    smartFuel.lastPercent = 0.0f;
    smartFuel.lastHeadspeed = 0.0f;
    smartFuel.percent = 0;
}

void smartFuelInit(void)
{
    memset(&smartFuel, 0, sizeof(smartFuel));
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
    const uint8_t reserve = smartFuelClampReserve(batteryConfig()->consumptionWarningPercentage);
    const float voltagePerCell = voltage / cellCount;

    if (voltagePerCell >= fullCellVoltage) {
        return 100.0f;
    } else if (voltagePerCell <= minCellVoltage) {
        return 0.0f;
    }

    const float scaledVoltage = constrainf(
        3.0f + ((voltagePerCell - minCellVoltage) / (fullCellVoltage - minCellVoltage)) * 1.2f,
        3.0f, 4.2f);
    const float rawPercent = constrainf(100.0f / (1.0f + expf(-12.0f * (scaledVoltage - 3.7f))), 0.0f, 100.0f);
    const float usableSpan = 100.0f - reserve;

    if (usableSpan <= 0.0f) {
        return rawPercent;
    }

    if (rawPercent <= reserve) {
        return 0.0f;
    }

    return constrainf(((rawPercent - reserve) / usableSpan) * 100.0f, 0.0f, 100.0f);
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

    return voltage + powf(smartFuelSagMultiplier(), 1.5f) * MAX(stickLoad, rpmDrop) * SMARTFUEL_MAX_SAG_COMP_V;
}

void smartFuelUpdate(timeUs_t currentTimeUs)
{
    const bool armed = ARMING_FLAG(ARMED);
    const smartFuelConfigSig_t config = smartFuelGetConfig();

    if (memcmp(&smartFuel.config, &config, sizeof(config)) != 0) {
        smartFuel.config = config;
        smartFuelResetState(currentTimeUs);
    }

    if (!smartFuelIsEnabled()) {
        smartFuel.wasArmed = armed;
        return;
    }

    if (!isBatteryVoltageConfigured() || getBatteryCellCount() == 0) {
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

    float percent = smartFuelPercentFromVoltageOnly(voltage, getBatteryCellCount());
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
            percent = smartFuelPercentFromVoltageOnly(voltage, getBatteryCellCount());
        }
    }

    if (!smartFuel.voltageStabilized) {
        smartFuel.lastPercentValid = false;
        return;
    }

    const float usableCapacity = smartFuelGetUsableCapacity();
    if (usableCapacity < 10.0f) {
        smartFuel.lastPercentValid = false;
        return;
    }

    if (smartFuelGetConfiguredSource() == SMARTFUEL_SOURCE_CURRENT) {
        if (!smartFuel.startStateValid) {
            smartFuel.startPercent = percent;
            smartFuel.startConsumption = getBatteryCapacityUsed() - usableCapacity * (1.0f - smartFuel.startPercent / 100.0f);
            smartFuel.startStateValid = true;
        }

        const float usedCapacity = getBatteryCapacityUsed() - smartFuel.startConsumption;
        percent = 100.0f - (usedCapacity * 100.0f / usableCapacity);
    } else {
        float filteredVoltage = voltage;
        if (smartFuel.voltageFiltered && dt > 0.0f && voltage < smartFuel.filteredVoltage) {
            filteredVoltage = MAX(voltage, smartFuel.filteredVoltage - dt * smartFuelVoltageFallLimitPerSecond());
        } else {
            smartFuel.voltageFiltered = true;
        }

        smartFuel.filteredVoltage = filteredVoltage;
        const float targetPercent = smartFuelPercentFromVoltageOnly(smartFuelApplySagCompensation(filteredVoltage), getBatteryCellCount());
        const float targetConsumption = usableCapacity * (100.0f - targetPercent) / 100.0f;

        if (!smartFuel.virtualConsumptionValid) {
            smartFuel.virtualConsumption = targetConsumption;
            smartFuel.virtualConsumptionValid = true;
        } else if ((armed || smartFuel.hadFlight) && dt > 0.0f && targetConsumption > smartFuel.virtualConsumption) {
            const float maxConsumptionIncrease = dt * smartFuelDropPerSecond() * usableCapacity / 100.0f;
            smartFuel.virtualConsumption = MIN(targetConsumption, smartFuel.virtualConsumption + maxConsumptionIncrease);
        }

        percent = 100.0f - (smartFuel.virtualConsumption * 100.0f / usableCapacity);
    }

    smartFuel.lastPercent = constrainf(percent, 0.0f, 100.0f);
    smartFuel.lastPercentValid = true;
    smartFuel.percent = lrintf(smartFuel.lastPercent);
}

uint32_t smartFuelGetConsumption(void)
{
    if (!smartFuelIsEnabled()) {
        return getBatteryCapacityUsed();
    }

    if (smartFuelGetConfiguredSource() == SMARTFUEL_SOURCE_CURRENT || !smartFuel.virtualConsumptionValid || !smartFuel.lastPercentValid) {
        return getBatteryCapacityUsed();
    }

    return lrintf(MAX(smartFuel.virtualConsumption, 0.0f));
}

uint8_t smartFuelGetPercent(void)
{
    if (!smartFuelIsEnabled() || !smartFuel.lastPercentValid) {
        return calculateBatteryPercentageRemaining();
    }

    return smartFuel.percent;
}

#else

void smartFuelInit(void)
{
}

void smartFuelUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

bool smartFuelIsEnabled(void)
{
    return false;
}

uint32_t smartFuelGetConsumption(void)
{
    return getBatteryCapacityUsed();
}

uint8_t smartFuelGetPercent(void)
{
    return calculateBatteryPercentageRemaining();
}

#endif
