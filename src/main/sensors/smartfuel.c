#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SMARTFUEL

#include "common/maths.h"
#include "pg/battery.h"
#include "sensors/battery.h"
#include "sensors/smartfuel.h"

typedef struct
{
    float chargeLevel;
    float initialChargeLevel;
    float lastCellVoltage;
    float voltageDropPerSample;
    float chargeDropPerSample;
    smartFuelMode_e mode;
} smartFuelState_t;

static smartFuelState_t smartFuel;

bool smartFuelIsEnabled(void)
{
    return smartFuel.mode != SMARTFUEL_MODE_OFF;
}

uint8_t smartFuelChargeLevel(void)
{
    return (uint8_t)lrintf(constrainf(smartFuel.chargeLevel, 0.0f, 1.0f) * 100.0f);
}

void validateAndFixSmartFuelConfig(void)
{
    batteryConfig_t *config = batteryConfigMutable();
    if (!isBatteryVoltageConfigured() || config->smartfuel_mode >= SMARTFUEL_MODE_COUNT)
    {
        config->smartfuel_mode = SMARTFUEL_MODE_OFF;
    }
    if (config->smartfuel_voltage_drop_rate > SMARTFUEL_VOLTAGE_DROP_RATE_MAX)
    {
        config->smartfuel_voltage_drop_rate = SMARTFUEL_VOLTAGE_DROP_RATE_DEFAULT;
    }
    if (config->smartfuel_charge_drop_rate > SMARTFUEL_CHARGE_DROP_RATE_MAX)
    {
        config->smartfuel_charge_drop_rate = SMARTFUEL_CHARGE_DROP_RATE_DEFAULT;
    }
    if (config->smartfuel_sag_gain > SMARTFUEL_SAG_GAIN_MAX)
    {
        config->smartfuel_sag_gain = SMARTFUEL_SAG_GAIN_DEFAULT;
    }
}

void smartFuelInit(void)
{
    memset(&smartFuel, 0, sizeof(smartFuel));
    validateAndFixSmartFuelConfig();
    smartFuel.mode = (smartFuelMode_e)batteryConfig()->smartfuel_mode;
    const float updateHz = MAX(1.0f, batteryConfig()->vbatUpdateHz);
    smartFuel.voltageDropPerSample = batteryConfig()->smartfuel_voltage_drop_rate / 1000.0f / updateHz;
    smartFuel.chargeDropPerSample = batteryConfig()->smartfuel_charge_drop_rate / 10000.0f / updateHz;
}

void smartFuelUpdate(void)
{
    if (!smartFuelIsEnabled() || getBatteryCellCount() == 0 || getVoltageState() == BATTERY_NOT_PRESENT)
    {
        return;
    }

    const float cellVoltage = getBatteryVoltage() / 100.0f / getBatteryCellCount();
    const float minVoltage = batteryConfig()->vbatmincellvoltage / 100.0f;
    const float fullVoltage = batteryConfig()->vbatfullcellvoltage / 100.0f;
    const float estimate = constrainf((cellVoltage - minVoltage) / (fullVoltage - minVoltage), 0.0f, 1.0f);

    if (smartFuel.initialChargeLevel == 0.0f)
    {
        smartFuel.initialChargeLevel = estimate;
        smartFuel.chargeLevel = estimate;
    }

    float next = fminf(smartFuel.chargeLevel, estimate);
    if (smartFuel.mode == SMARTFUEL_MODE_CURRENT || smartFuel.mode == SMARTFUEL_MODE_COMBINED)
    {
        const uint16_t capacity = getBatteryCapacity();
        if (capacity > 0)
        {
            next = fminf(next, smartFuel.initialChargeLevel - (float)getBatteryCapacityUsed() / capacity);
        }
    }
    smartFuel.chargeLevel = constrainf(next, 0.0f, 1.0f);
}

#endif
