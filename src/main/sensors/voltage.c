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

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/adcinternal.h"

#include "voltage.h"


/** Voltage Sensors **/

#ifndef VOLTAGE_SCALE_DEFAULT
#define VOLTAGE_SCALE_DEFAULT 110
#endif

#ifndef VOLTAGE_DIVIDER_DEFAULT
#define VOLTAGE_DIVIDER_DEFAULT 10
#endif

#ifndef VOLTAGE_MULTIPLIER_DEFAULT
#define VOLTAGE_MULTIPLIER_DEFAULT 1
#endif

#ifndef VOLTAGE_CUTOFF_DEFAULT
#define VOLTAGE_CUTOFF_DEFAULT 25
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);

void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .scale = VOLTAGE_SCALE_DEFAULT,
            .divider = VOLTAGE_DIVIDER_DEFAULT,
            .divmul = VOLTAGE_MULTIPLIER_DEFAULT,
            .cutoff = VOLTAGE_CUTOFF_DEFAULT,
        );
    }
}


/** Internal state **/

typedef struct {
    bool        enabled;
    uint32_t    sample;     // mV
    uint32_t    voltage;    // mV
    filter_t    filter;
} voltageSensorState_t;


/** Voltage ADC Sensors **/

#ifdef USE_ADC

static voltageSensorState_t voltageADCSensors[MAX_VOLTAGE_SENSOR_ADC];

// Voltage sensor ID to ADC Channel map
static const uint8_t voltageSensorAdcChannelMap[MAX_VOLTAGE_SENSOR_ADC] = {
    [VOLTAGE_SENSOR_ADC_BAT] = ADC_BATTERY,
    [VOLTAGE_SENSOR_ADC_BEC] = ADC_VBEC,
    [VOLTAGE_SENSOR_ADC_BUS] = ADC_VBUS,
    [VOLTAGE_SENSOR_ADC_EXT] = ADC_VEXT,
};

static float voltageSensorADCtoVoltage(int sensor, const uint16_t adc)
{
    const voltageSensorADCConfig_t *config = voltageSensorADCConfig(sensor);

    const float voltage = config->scale * getVrefMv();
    const float divisor = fmaxf(config->divider * config->divmul, 1) * 4095;

    return adc * voltage / divisor;
}
#endif

void voltageSensorADCRefresh(void)
{
#ifdef USE_ADC
    for (unsigned i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        voltageSensorState_t *state = &voltageADCSensors[i];

        const uint8_t channel = voltageSensorAdcChannelMap[i];

        if (state->enabled) {
            const uint16_t sample = adcGetChannel(channel);
            const float voltage = voltageSensorADCtoVoltage(i, sample);

            state->sample = voltage;
            state->voltage = filterApply(&state->filter, voltage);
        }
        else
        {
            state->sample = 0;
            state->voltage = 0;
        }
    }
#endif
}

bool voltageSensorADCRead(voltageSensorADC_e sensor, voltageMeter_t *meter)
{
#ifdef USE_ADC
    voltageSensorState_t *state = &voltageADCSensors[sensor];

    if (state->enabled) {
        meter->voltage = state->voltage;
        meter->sample = state->sample;
        return true;
    }
#else
    UNUSED(sensor);
#endif

    voltageMeterReset(meter);

    return false;
}

void voltageSensorADCInit(void)
{
#ifdef USE_ADC
    memset(voltageADCSensors, 0, sizeof(voltageADCSensors));

    for (unsigned i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        voltageADCSensors[i].enabled = adcIsEnabled(voltageSensorAdcChannelMap[i]);

        lowpassFilterInit(&voltageADCSensors[i].filter, LPF_BESSEL,
            voltageSensorADCConfig(i)->cutoff,
            batteryConfig()->vbatUpdateHz, 0);
    }
#endif
}


/** Voltage ESC Sensors **/

#ifdef USE_ESC_SENSOR
static voltageSensorState_t voltageESCSensor;
#endif

bool voltageSensorESCReadMotor(uint8_t motorNumber, voltageMeter_t *meter)
{
    UNUSED(motorNumber);

#ifdef USE_ESC_SENSOR
    const escSensorData_t *escData = getEscSensorData(motorNumber);

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        meter->voltage = meter->sample = escData->voltage;
        return true;
    }
    else
#endif
    {
        voltageMeterReset(meter);
    }

    return false;
}

bool voltageSensorESCReadTotal(voltageMeter_t *meter)
{
#ifdef USE_ESC_SENSOR
    const voltageSensorState_t * state = &voltageESCSensor;

    meter->sample = state->sample;
    meter->voltage = state->voltage;
#else
    voltageMeterReset(meter);
#endif

    return state->enabled;
}

void voltageSensorESCRefresh(void)
{
    voltageSensorState_t * state = &voltageESCSensor;

#ifdef USE_ESC_SENSOR
    const escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        uint32_t voltage = escData->voltage;
        state->sample = voltage;
        state->voltage = filterApply(&state->filter, voltage);
        state->enabled = true;
    }
    else
#endif
    {
        state->sample = 0;
        state->voltage = 0;
    }
}

void voltageSensorESCInit(void)
{
#ifdef USE_ESC_SENSOR
    memset(&voltageESCSensor, 0, sizeof(voltageESCSensor));
    lowpassFilterInit(&voltageESCSensor.filter, LPF_BESSEL,
        escSensorConfig()->filter_cutoff,
        batteryConfig()->vbatUpdateHz, 0);
#endif
}


/** Voltage MCU Sensors **/

bool voltageSensorMCURead(voltageMeter_t *meter)
{
#ifdef USE_ADC_INTERNAL
    meter->sample = meter->voltage = getVrefMv();
    return true;
#else
    voltageMeterReset(meter);
    return false;
#endif
}


//
// API for using voltage meters using IDs
//
// This API is used by MSP, for configuration/status.
//

const uint8_t voltageMeterIds[] = {
    VOLTAGE_METER_ID_BATTERY,
    VOLTAGE_METER_ID_BEC,
    VOLTAGE_METER_ID_BUS,
    VOLTAGE_METER_ID_EXT,
    VOLTAGE_METER_ID_MCU,
#ifdef USE_ESC_SENSOR
    VOLTAGE_METER_ID_ESC_COMBINED,
    VOLTAGE_METER_ID_ESC_1,
    VOLTAGE_METER_ID_ESC_2,
    VOLTAGE_METER_ID_ESC_3,
    VOLTAGE_METER_ID_ESC_4,
#endif
};

const uint8_t voltageMeterCount = ARRAYLEN(voltageMeterIds);

const uint8_t voltageSensorToMeterMap[MAX_VOLTAGE_SENSOR_ADC] = {
    [VOLTAGE_SENSOR_ADC_BAT] = VOLTAGE_METER_ID_BATTERY,
    [VOLTAGE_SENSOR_ADC_BEC] = VOLTAGE_METER_ID_BEC,
    [VOLTAGE_SENSOR_ADC_BUS] = VOLTAGE_METER_ID_BUS,
    [VOLTAGE_SENSOR_ADC_EXT] = VOLTAGE_METER_ID_EXT,
};

bool voltageMeterRead(voltageMeterId_e id, voltageMeter_t *meter)
{
    switch (id) {
        case VOLTAGE_METER_ID_BATTERY:
            return voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, meter);
        case VOLTAGE_METER_ID_BEC:
            return voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BEC, meter);
        case VOLTAGE_METER_ID_BUS:
            return voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BUS, meter);
        case VOLTAGE_METER_ID_EXT:
            return voltageSensorADCRead(VOLTAGE_SENSOR_ADC_EXT, meter);
        case VOLTAGE_METER_ID_MCU:
            return voltageSensorMCURead(meter);
#ifdef USE_ESC_SENSOR
        case VOLTAGE_METER_ID_ESC_COMBINED:
            return voltageSensorESCReadTotal(meter);
        case VOLTAGE_METER_ID_ESC_1:
        case VOLTAGE_METER_ID_ESC_2:
        case VOLTAGE_METER_ID_ESC_3:
        case VOLTAGE_METER_ID_ESC_4:
            return voltageSensorESCReadMotor(id - VOLTAGE_METER_ID_ESC_1, meter);
#endif
        default:
           voltageMeterReset(meter);
    }

    return false;
}

void voltageMeterReset(voltageMeter_t *meter)
{
    memset(meter, 0, sizeof(voltageMeter_t));
}
