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


typedef enum {
    CURRENT_SENSOR_TYPE_NONE = 0,
    CURRENT_SENSOR_TYPE_ADC,
    CURRENT_SENSOR_TYPE_ESC,
} currentSensorType_e;

typedef enum {
    CURRENT_SENSOR_ADC_BAT = 0,
    MAX_CURRENT_SENSOR_ADC
} currentSensorADC_e;


typedef struct {
    int16_t     scale;          // scale the current sensor output voltage to milliamps. Value in mV/10A
    int16_t     offset;         // offset of the current sensor in mA
    uint8_t     cutoff;         // filter cutoff frequency in Hz
} currentSensorADCConfig_t;

PG_DECLARE_ARRAY(currentSensorADCConfig_t, MAX_CURRENT_SENSOR_ADC, currentSensorADCConfig);
