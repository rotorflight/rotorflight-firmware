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

#include "platform.h"

#include "pg/current.h"

#include "common/time.h"
#include "current_ids.h"


typedef struct {
    uint32_t sample;
    uint32_t current;
    uint32_t capacity;
} currentMeter_t;


//
// Current Sensor API
//

void currentSensorADCInit(void);
void currentSensorADCRefresh(timeUs_t currentTimeUs);
bool currentSensorADCRead(currentSensorADC_e sensor, currentMeter_t *meter);

void currentSensorESCInit(void);
void currentSensorESCRefresh(void);
bool currentSensorESCReadTotal(currentMeter_t *meter);
bool currentSensorESCReadMotor(uint8_t motorNumber, currentMeter_t *meter);


//
// Current Meter API
//

extern const uint8_t currentSensorToMeterMap[MAX_CURRENT_SENSOR_ADC];
extern const uint8_t currentMeterIds[];
extern const uint8_t currentMeterCount;

bool currentMeterRead(currentMeterId_e id, currentMeter_t *currentMeter);
void currentMeterReset(currentMeter_t *meter);

