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

#pragma once

#include "types.h"
#include "platform.h"

#include "pg/pg.h"


#define VOLTAGE_SCALE_MIN 0
#define VOLTAGE_SCALE_MAX 65535

#define VOLTAGE_DIVIDER_MIN 1
#define VOLTAGE_DIVIDER_MAX 65535

#define VOLTAGE_MULTIPLIER_MIN 1
#define VOLTAGE_MULTIPLIER_MAX 255


typedef enum {
    VOLTAGE_SENSOR_TYPE_NONE = 0,
    VOLTAGE_SENSOR_TYPE_ADC,
    VOLTAGE_SENSOR_TYPE_ESC,
} voltageSensorType_e;

typedef enum {
    VOLTAGE_SENSOR_ADC_BAT = 0,
    VOLTAGE_SENSOR_ADC_BEC = 1,
    VOLTAGE_SENSOR_ADC_BUS = 2,
    VOLTAGE_SENSOR_ADC_EXT = 3,
    MAX_VOLTAGE_SENSOR_ADC
} voltageSensorADC_e;


typedef struct {
    uint16_t    scale;          // adjust scale and divider to match voltage to measured value
    uint16_t    divider;
    uint8_t     divmul;         // extra multiplier for divider (backwards compatibility)
    uint8_t     cutoff;         // filter cutoff in Hz
} voltageSensorADCConfig_t;

PG_DECLARE_ARRAY(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);
