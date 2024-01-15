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
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "current.h"


/** Current Sensors **/

#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400
#endif

#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

#ifndef CURRENT_METER_CUTOFF_DEFAULT
#define CURRENT_METER_CUTOFF_DEFAULT 25
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(currentSensorADCConfig_t, MAX_CURRENT_SENSOR_ADC, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

void pgResetFn_currentSensorADCConfig(currentSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_CURRENT_SENSOR_ADC; i++) {
        RESET_CONFIG(currentSensorADCConfig_t, &instance[i],
            .scale = CURRENT_METER_SCALE_DEFAULT,
            .offset = CURRENT_METER_OFFSET_DEFAULT,
            .cutoff = CURRENT_METER_CUTOFF_DEFAULT,
        );
    }
}


/** Internal state **/

typedef struct {
    bool enabled;
    uint32_t sample;        // mA
    uint32_t current;       // mA
    uint64_t capacity;      // mAus
    filter_t filter;
} currentSensorState_t;


/** Current ADC Sensors **/

#ifdef USE_ADC

static currentSensorState_t currentADCSensors[MAX_CURRENT_SENSOR_ADC];

// Current sensor ID to ADC Channel map
static const uint8_t currentSensorAdcChannelMap[MAX_CURRENT_SENSOR_ADC] = {
    [CURRENT_SENSOR_ADC_BAT] = ADC_CURRENT,
};

static float currentSensorADCToCurrent(int sensor, const uint16_t src)
{
    const currentSensorADCConfig_t *config = currentSensorADCConfig(sensor);

    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    float voltage = src * getVrefMv() / 0.4095f;
    float current = config->scale ? (voltage / config->scale + config->offset) : 0;

    return fmaxf(current, 0);
}
#endif

void currentSensorADCRefresh(timeUs_t currentTimeUs)
{
#ifdef USE_ADC
    static timeUs_t lastServiced = 0;

    const timeDelta_t udpateDelta = cmp32(currentTimeUs, lastServiced);
    lastServiced = currentTimeUs;

    for (unsigned i = 0; i < MAX_CURRENT_SENSOR_ADC; i++) {
        currentSensorState_t * state = &currentADCSensors[i];

        const uint8_t channel = currentSensorAdcChannelMap[i];
        state->enabled = adcIsEnabled(channel);

        if (state->enabled) {
            const uint16_t sample = adcGetChannel(channel);
            const float current = currentSensorADCToCurrent(i, sample);

            state->sample = current;
            state->current = filterApply(&state->filter, current);
            state->capacity += current * udpateDelta;

            DEBUG_AXIS(CURRENT_SENSOR, i, 0, sample);
            DEBUG_AXIS(CURRENT_SENSOR, i, 1, current);
            DEBUG_AXIS(CURRENT_SENSOR, i, 2, state->current);
            DEBUG_AXIS(CURRENT_SENSOR, i, 3, state->capacity / 3600000000u);
        }
        else {
            state->sample = 0;
            state->current = 0;
        }
    }
#else
    UNUSED(currentTimeUs);
#endif
}

bool currentSensorADCRead(currentSensorADC_e sensor, currentMeter_t *meter)
{
#ifdef USE_ADC
    currentSensorState_t * state = &currentADCSensors[sensor];

    if (state->enabled) {
        meter->sample = state->sample;
        meter->current = state->current;
        meter->capacity = state->capacity / 3600000000u;
    }

    return state->enabled;
#else
    UNUSED(sensor);
    currentMeterReset(meter);
    return false;
#endif
}

void currentSensorADCInit(void)
{
#ifdef USE_ADC
    memset(&currentADCSensors, 0, sizeof(currentADCSensors));

    for (unsigned i = 0; i < MAX_CURRENT_SENSOR_ADC; i++) {
        lowpassFilterInit(&currentADCSensors[i].filter, LPF_BESSEL,
            currentSensorADCConfig(i)->cutoff,
            batteryConfig()->ibatUpdateHz, 0);
    }
#endif
}


/** Current ESC Sensors **/

#ifdef USE_ESC_SENSOR

currentSensorState_t currentESCSensor;

bool currentSensorESCReadMotor(uint8_t motorNumber, currentMeter_t *meter)
{
    escSensorData_t *escData = getEscSensorData(motorNumber);

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        meter->current = meter->sample = escData->current;
        meter->capacity = escData->consumption;
        return true;
    } else {
        currentMeterReset(meter);
    }

    return false;
}

bool currentSensorESCReadTotal(currentMeter_t *meter)
{
    currentSensorState_t * state = &currentESCSensor;

    meter->sample = state->sample;
    meter->current = state->current;
    meter->capacity = state->capacity;

    return state->enabled;
}

void currentSensorESCRefresh(void)
{
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    currentSensorState_t * state = &currentESCSensor;

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        const uint32_t current = escData->current + escSensorConfig()->current_offset;
        state->sample = current;
        state->current = filterApply(&state->filter, current);
        state->capacity = escData->consumption + (escSensorConfig()->current_offset * millis() / 3600000u);
        state->enabled = true;
    }
    else {
        state->sample = 0;
        state->current = 0;
    }
}

void currentSensorESCInit(void)
{
    memset(&currentESCSensor, 0, sizeof(currentESCSensor));

    lowpassFilterInit(&currentESCSensor.filter, LPF_BESSEL,
        escSensorConfig()->filter_cutoff,
        batteryConfig()->ibatUpdateHz, 0);
}

#endif


//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//

const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY,
#ifdef USE_ESC_SENSOR
    CURRENT_METER_ID_ESC_COMBINED,
    CURRENT_METER_ID_ESC_1,
    CURRENT_METER_ID_ESC_2,
    CURRENT_METER_ID_ESC_3,
    CURRENT_METER_ID_ESC_4,
#endif
};

const uint8_t currentMeterCount = ARRAYLEN(currentMeterIds);

const uint8_t currentSensorToMeterMap[MAX_CURRENT_SENSOR_ADC] = {
    [CURRENT_SENSOR_ADC_BAT] = CURRENT_METER_ID_BATTERY,
};

bool currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    switch (id) {
        case CURRENT_METER_ID_BATTERY:
            return currentSensorADCRead(CURRENT_SENSOR_ADC_BAT, meter);
#ifdef USE_ESC_SENSOR
        case CURRENT_METER_ID_ESC_COMBINED:
            return currentSensorESCReadTotal(meter);
        case CURRENT_METER_ID_ESC_1:
        case CURRENT_METER_ID_ESC_2:
        case CURRENT_METER_ID_ESC_3:
        case CURRENT_METER_ID_ESC_4:
            return currentSensorESCReadMotor(id - CURRENT_METER_ID_ESC_1, meter);
#endif
        default:
            currentMeterReset(meter);
    }

    return false;
}

void currentMeterReset(currentMeter_t *meter)
{
    memset(meter, 0, sizeof(currentMeter_t));
}

