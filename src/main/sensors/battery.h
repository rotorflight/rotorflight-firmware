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

#include "common/filter.h"
#include "common/time.h"

#include "sensors/current.h"
#include "sensors/voltage.h"

#include "pg/battery.h"


#define VBAT_CELL_VOTAGE_RANGE_MIN          100
#define VBAT_CELL_VOTAGE_RANGE_MAX          500

#define GET_BATTERY_LPF_FREQUENCY(period) (10.0f / period)


typedef struct lowVoltageCutoff_s {
    bool enabled;
    uint8_t percentage;
    timeUs_t startTime;
} lowVoltageCutoff_t;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
} batteryState_e;

void batteryInit(void);

void taskBatteryVoltageUpdate(timeUs_t currentTimeUs);
void taskBatteryCurrentUpdate(timeUs_t currentTimeUs);
void taskBatteryAlerts(timeUs_t currentTimeUs);

batteryState_e getBatteryState(void);
batteryState_e getVoltageState(void);
batteryState_e getConsumptionState(void);

const voltageMeter_t * getBatteryVoltageMeter();
const currentMeter_t * getBatteryCurrentMeter();

const char * getBatteryStateString(void);

bool isBatteryVoltageConfigured(void);
uint8_t getBatteryVoltageSource(void);

uint8_t getBatteryCellCount(void);
uint16_t getBatteryCellVoltage(uint8_t cell);
uint16_t getBatteryVoltage(void);
uint16_t getBatteryVoltageSample(void);
uint16_t getLegacyBatteryVoltage(void);
uint16_t getBatteryAverageCellVoltage(void);

bool isBatteryCurrentConfigured(void);
uint16_t getBatteryCurrent(void);
uint16_t getBatteryCurrentSample(void);
uint16_t getLegacyBatteryCurrent(void);
uint32_t getBatteryCapacityUsed(void);

uint8_t calculateBatteryPercentageRemaining(void);

const lowVoltageCutoff_t *getLowVoltageCutoff(void);

extern const char * const batteryVoltageSourceNames[VOLTAGE_METER_COUNT];
extern const char * const batteryCurrentSourceNames[CURRENT_METER_COUNT];
