/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "pg/voltage.h"

#include "voltage_ids.h"


typedef struct voltageMeter_s {
    uint32_t sample;
    uint32_t voltage;
} voltageMeter_t;


//
// Voltage Sensor API
//

void voltageSensorADCInit(void);
void voltageSensorADCRefresh(void);
bool voltageSensorADCRead(voltageSensorADC_e sensor, voltageMeter_t *voltageMeter);

void voltageSensorESCInit(void);
void voltageSensorESCRefresh(void);
bool voltageSensorESCReadTotal(voltageMeter_t *voltageMeter);
bool voltageSensorESCReadMotor(uint8_t motor, voltageMeter_t *voltageMeter);


//
// Voltage Meter API
//

extern const uint8_t voltageSensorToMeterMap[MAX_VOLTAGE_SENSOR_ADC];
extern const uint8_t voltageMeterIds[];
extern const uint8_t voltageMeterCount;

bool voltageMeterRead(voltageMeterId_e id, voltageMeter_t *voltageMeter);
void voltageMeterReset(voltageMeter_t *voltageMeter);
