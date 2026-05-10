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

#ifdef USE_SMARTFUEL

#include "common/maths.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/motors.h"

#include "pg/battery.h"

#include "sensors/battery.h"
#include "sensors/smartfuel.h"

#define SMARTFUEL_MAX_SAG_COMP_V      0.5f

typedef struct smartFuelConfig_s {
    uint8_t enabled;
    float vbatMin;
    float vbatFull;
    float voltageFallRatePerSample;
    float chargeDropPerSample;
    float sagCompensation;
} smartFuelConfig_t;

typedef struct smartFuelState_s {
    float levelPercent;
    float lastHeadspeed;
    float lastVoltage;
    smartFuelConfig_t config;
} smartFuelState_t;

static smartFuelState_t smartFuel = INIT_ZERO;


bool smartFuelIsEnabled(void)
{
    return smartFuel.config.enabled;
}

uint8_t smartFuelChargeLevel(void)
{
    return lrintf(smartFuel.levelPercent);
}

float smartFuelChargeLevelf(void)
{
    return smartFuel.levelPercent;
}

static void smartFuelResetState(void)
{
    smartFuel.levelPercent = 0.0f;
    smartFuel.lastHeadspeed = 0.0f;
    smartFuel.lastVoltage = 0.0f;
}

static inline float pow1_5(float x)
{
    return x * sqrtf(x);
}

void INIT_CODE validateAndFixSmartFuelConfig(void)
{
    batteryConfig_t *config = batteryConfigMutable();

    if (config->smartfuel_voltage_fall_rate > SMARTFUEL_VOLTAGE_FALL_RATE_MAX) {
        config->smartfuel_voltage_fall_rate = SMARTFUEL_VOLTAGE_FALL_RATE_DEFAULT;
    }
    if (config->smartfuel_charge_drop_rate > SMARTFUEL_CHARGE_DROP_RATE_MAX) {
        config->smartfuel_charge_drop_rate = SMARTFUEL_CHARGE_DROP_RATE_DEFAULT;
    }
    if (config->smartfuel_sag_multiplier > SMARTFUEL_SAG_MULTIPLIER_MAX) {
        config->smartfuel_sag_multiplier = SMARTFUEL_SAG_MULTIPLIER_DEFAULT;
    }
}

void INIT_CODE smartFuelInit(void)
{
    validateAndFixSmartFuelConfig();

    memset(&smartFuel, 0, sizeof(smartFuel));

    const float dT = 1.0f / batteryConfig()->vbatUpdateHz;

    smartFuel.config.enabled = batteryConfig()->smartfuel;

    smartFuel.config.vbatMin = batteryConfig()->vbatmincellvoltage / 100.0f;
    smartFuel.config.vbatFull = batteryConfig()->vbatfullcellvoltage / 100.0f;

    smartFuel.config.voltageFallRatePerSample = batteryConfig()->smartfuel_voltage_fall_rate / 100.0f * dT;
    smartFuel.config.chargeDropPerSample = batteryConfig()->smartfuel_charge_drop_rate / 10.0f * dT;

    const float sagMultiplier = batteryConfig()->smartfuel_sag_multiplier / 100.0f;
    smartFuel.config.sagCompensation = pow1_5(sagMultiplier) * SMARTFUEL_MAX_SAG_COMP_V;
}

static float smartFuelPercentFromVoltage(float voltage, uint8_t cellCount)
{
    const float fullCellVoltage = smartFuel.config.vbatFull;
    const float minCellVoltage = smartFuel.config.vbatMin;
    const float voltagePerCell = voltage / cellCount;

    if (voltagePerCell >= fullCellVoltage)
        return 100.0f;
    else if (voltagePerCell <= minCellVoltage)
        return 0.0f;

    const float scaledVoltage = constrainf(
        3.0f + ((voltagePerCell - minCellVoltage) / (fullCellVoltage - minCellVoltage)) * 1.2f,
        3.0f, 4.2f);

    return constrainf(100.0f / (1.0f + exp_approx(-12.0f * (scaledVoltage - 3.7f))), 0.0f, 100.0f);
}

static float smartFuelApplySagCompensation(float voltage)
{
    const float cyclic = getCyclicDeflection();
    const float collective = getCollectiveDeflectionAbs();
    const float stickLoad = constrainf(cyclic + collective * 1.2f, 0.0f, 1.0f);

    const float headSpeed = getHeadSpeedf();

    if (ARMING_FLAG(ARMED) && headSpeed > 100.0f) {
        float rpmDrop = 0.0f;
        if (smartFuel.lastHeadspeed > 100.0f) {
            rpmDrop = fmaxf((smartFuel.lastHeadspeed - headSpeed) / smartFuel.lastHeadspeed, 0.0f);
        }
        smartFuel.lastHeadspeed = headSpeed;
        voltage += smartFuel.config.sagCompensation * fmaxf(stickLoad, rpmDrop);
    }
    else {
        smartFuel.lastHeadspeed = 0.0f;
    }

    return voltage;
}

void smartFuelUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!smartFuelIsEnabled()) {
        return;
    }

    if (!isBatteryVoltageConfigured() || getBatteryCellCount() == 0) {
        smartFuelResetState();
        return;
    }

    float voltage = getBatteryVoltage() / 100.0f;

    if (smartFuel.lastVoltage > 0.0f && voltage < smartFuel.lastVoltage)
        voltage = fmaxf(voltage, smartFuel.lastVoltage - smartFuel.config.voltageFallRatePerSample);

    smartFuel.lastVoltage = voltage;

    const float compensatedVoltage = smartFuelApplySagCompensation(voltage);
    const float estimation = smartFuelPercentFromVoltage(compensatedVoltage, getBatteryCellCount());

    if ((ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) && (estimation < smartFuel.levelPercent)) {
        smartFuel.levelPercent = fmaxf(estimation, smartFuel.levelPercent - smartFuel.config.chargeDropPerSample);
    }
    else {
        smartFuel.levelPercent = estimation;
    }

    smartFuel.levelPercent = constrainf(smartFuel.levelPercent, 0.0f, 100.0f);
}

#endif
