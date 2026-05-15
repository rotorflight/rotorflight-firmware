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

#include "platform.h"

#ifdef USE_SMARTFUEL

#include "common/maths.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/airborne.h"
#include "flight/mixer.h"

#include "pg/battery.h"

#include "sensors/battery.h"
#include "sensors/smartfuel.h"

typedef struct smartFuelConfig_s {
    smartFuelMode_e mode;
    float vCellMin;
    float vCellFull;
    float voltageDropPerSample;
    float chargeDropPerSample;
    float sagCompensation;
} smartFuelConfig_t;

typedef struct smartFuelState_s {
    float chargeLevel;
    float initialChargeLevel;
    float lastCellVoltage;
    float initialCellVoltage;
    smartFuelConfig_t config;
} smartFuelState_t;

static smartFuelState_t smartFuel = INIT_ZERO;


bool smartFuelIsEnabled(void)
{
    return smartFuel.config.mode != SMARTFUEL_MODE_OFF;
}

uint8_t smartFuelChargeLevel(void)
{
    return lrintf(smartFuel.chargeLevel * 100.0f);
}

float smartFuelChargeLevelf(void)
{
    return smartFuel.chargeLevel;
}

static void smartFuelResetState(void)
{
    smartFuel.chargeLevel = 0.0f;
    smartFuel.initialChargeLevel = 0.0f;
    smartFuel.lastCellVoltage = 0.0f;
    smartFuel.initialCellVoltage = 0.0f;
}

void INIT_CODE validateAndFixSmartFuelConfig(void)
{
    batteryConfig_t *config = batteryConfigMutable();

    if (!isBatteryVoltageConfigured()) {
        config->smartfuel_mode = SMARTFUEL_MODE_OFF;
    }
    if (config->smartfuel_mode >= SMARTFUEL_MODE_COUNT) {
        config->smartfuel_mode = SMARTFUEL_MODE_OFF;
    }
    if (config->smartfuel_voltage_drop_rate > SMARTFUEL_VOLTAGE_DROP_RATE_MAX) {
        config->smartfuel_voltage_drop_rate = SMARTFUEL_VOLTAGE_DROP_RATE_DEFAULT;
    }
    if (config->smartfuel_charge_drop_rate > SMARTFUEL_CHARGE_DROP_RATE_MAX) {
        config->smartfuel_charge_drop_rate = SMARTFUEL_CHARGE_DROP_RATE_DEFAULT;
    }
    if (config->smartfuel_sag_gain > SMARTFUEL_SAG_GAIN_MAX) {
        config->smartfuel_sag_gain = SMARTFUEL_SAG_GAIN_DEFAULT;
    }
}

void INIT_CODE smartFuelInit(void)
{
    validateAndFixSmartFuelConfig();

    memset(&smartFuel, 0, sizeof(smartFuel));

    const float dT = 1.0f / batteryConfig()->vbatUpdateHz;

    smartFuel.config.mode = batteryConfig()->smartfuel_mode;

    smartFuel.config.vCellMin = batteryConfig()->vbatmincellvoltage / 100.0f;
    smartFuel.config.vCellFull = batteryConfig()->vbatfullcellvoltage / 100.0f;

    smartFuel.config.voltageDropPerSample = (batteryConfig()->smartfuel_voltage_drop_rate / 1000.0f) * dT;
    smartFuel.config.chargeDropPerSample = (batteryConfig()->smartfuel_charge_drop_rate / 10000.0f) * dT;

    smartFuel.config.sagCompensation = batteryConfig()->smartfuel_sag_gain / 100.0f;
}

static float smartFuelChargeLevelFromVoltage(float cellVoltage)
{
    const float fullCellVoltage = smartFuel.config.vCellFull;
    const float minCellVoltage = smartFuel.config.vCellMin;

    if (cellVoltage >= fullCellVoltage)
        return 1.0f;
    else if (cellVoltage <= minCellVoltage)
        return 0.0f;

    const float scaledVoltage = constrainf(
        3.0f + ((cellVoltage - minCellVoltage) / (fullCellVoltage - minCellVoltage)) * 1.2f,
        3.0f, 4.2f);

    return constrainf(1.0f / (1.0f + exp_approx(-12.0f * (scaledVoltage - 3.7f))), 0.0f, 1.0f);
}

static float smartFuelApplySagCompensation(float cellVoltage)
{
    if (isAirborne()) {
        const float cyclic = getCyclicDeflection();
        const float collective = getCollectiveDeflectionAbs();
        const float stickLoad = constrainf(collective * collective + cyclic * 0.2f, 0.0f, 1.0f);

        cellVoltage += smartFuel.config.sagCompensation * stickLoad;
    }

    return cellVoltage;
}

static float smartFuelChargeLevelFromVoltageEstimation(float estimation)
{
    if (ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) {
        estimation = slewDownLimit(smartFuel.chargeLevel, estimation, smartFuel.config.chargeDropPerSample);
    }
    return estimation;
}

static float smartFuelChargeLevelFromCurrentEstimation(float estimation)
{
    const float capacity = getBatteryCapacity();
    const float used = getBatteryCapacityUsed();

    if (capacity > 0 && used > 0)
        estimation = smartFuel.initialChargeLevel - used / capacity;
    else
        estimation = smartFuelChargeLevelFromVoltageEstimation(estimation);

    return estimation;
}

static float smartFuelChargeLevelFromCombinedEstimation(float estimation)
{
    float volt_estimation = smartFuelChargeLevelFromVoltageEstimation(estimation);
    float curr_estimation = smartFuelChargeLevelFromCurrentEstimation(estimation);

    return fminf(volt_estimation, curr_estimation);
}

void smartFuelUpdate(void)
{
    if (!smartFuelIsEnabled()) {
        return;
    }

    if (getBatteryCellCount() == 0 || getVoltageState() == BATTERY_NOT_PRESENT) {
        smartFuelResetState();
        return;
    }

    float cellVoltage = (getBatteryVoltage() / 100.0f) / getBatteryCellCount();

    if (smartFuel.initialCellVoltage == 0)
        smartFuel.initialCellVoltage = cellVoltage;

    cellVoltage = slewDownLimit(smartFuel.lastCellVoltage, cellVoltage, smartFuel.config.voltageDropPerSample);
    smartFuel.lastCellVoltage = cellVoltage;

    const float compensatedVoltage = smartFuelApplySagCompensation(cellVoltage);
    float estimation = smartFuelChargeLevelFromVoltage(compensatedVoltage);

    if (smartFuel.initialChargeLevel == 0) {
        smartFuel.chargeLevel = estimation;
        smartFuel.initialChargeLevel = estimation;
    }

    estimation = fminf(smartFuel.initialChargeLevel, estimation);

    if (smartFuel.config.mode == SMARTFUEL_MODE_VOLTAGE)
        estimation = smartFuelChargeLevelFromVoltageEstimation(estimation);
    else if (smartFuel.config.mode == SMARTFUEL_MODE_CURRENT)
        estimation = smartFuelChargeLevelFromCurrentEstimation(estimation);
    else if (smartFuel.config.mode == SMARTFUEL_MODE_COMBINED)
        estimation = smartFuelChargeLevelFromCombinedEstimation(estimation);

    estimation = fminf(estimation, smartFuel.chargeLevel);

    smartFuel.chargeLevel = constrainf(estimation, 0.0f, 1.0f);
}

#endif
