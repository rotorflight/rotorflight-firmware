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

#include "pg/pg.h"

#include "common/filter.h"
#include "common/time.h"

#include "sensors/current.h"
#include "sensors/voltage.h"

#define VBAT_CELL_VOTAGE_RANGE_MIN      100
#define VBAT_CELL_VOTAGE_RANGE_MAX      500
#define VBAT_CELL_VOLTAGE_DEFAULT_MIN   330
#define VBAT_CELL_VOLTAGE_DEFAULT_MAX   430
#define VBAT_CELL_VOLTAGE_DEFAULT_FULL  410

#define GET_BATTERY_LPF_FREQUENCY(period) (10.0f / period)

typedef enum {
    VOLTAGE_METER_NONE = 0,
    VOLTAGE_METER_ADC,
    VOLTAGE_METER_ESC,
    VOLTAGE_METER_COUNT
} voltageMeterSource_e;

typedef enum {
    CURRENT_METER_NONE = 0,
    CURRENT_METER_ADC,
    CURRENT_METER_ESC,
    CURRENT_METER_COUNT
} currentMeterSource_e;

typedef struct batteryConfig_s {

    // battery size
    uint8_t forceBatteryCellCount;          // Number of cells in battery, used for overwriting auto-detected cell count if someone has issues with it.

    // sources
    uint8_t currentMeterSource;             // source of battery current meter used
    uint8_t voltageMeterSource;             // source of battery voltage meter used

    // voltages
    uint16_t vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 430 (4.30V)
    uint16_t vbatmincellvoltage;            // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.30V)
    uint16_t vbatfullcellvoltage;           // Cell voltage at which the battery is deemed to be "full" 0.01V units, default is 410 (4.1V)
    uint16_t vbatwarningcellvoltage;        // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.50V)
    uint16_t vbatnotpresentcellvoltage;     // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    uint8_t vbathysteresis;                 // hysteresis for alarm in 0.01V units, default 1 = 0.01V
    uint8_t lvcPercentage;                  // Percentage of throttle when lvc is triggered

    // current & capacity
    uint16_t batteryCapacity;               // mAh
    uint8_t consumptionWarningPercentage;   // Percentage of remaining capacity that should trigger a battery warning

    // warnings / alerts
    bool useVoltageAlerts;                  // Issue alerts based on VBat readings
    bool useConsumptionAlerts;              // Issue alerts based on total power consumption

    uint8_t vbatDurationForWarning;         // Period voltage has to sustain before the battery state is set to BATTERY_WARNING (in 0.1 s)
    uint8_t vbatDurationForCritical;        // Period voltage has to sustain before the battery state is set to BATTERY_CRIT (in 0.1 s)

    // Filters
    uint8_t vbatLpfPeriod;                  // Period of the cutoff frequency for the Vbat filter for display and startup (in 0.1 s)
    uint8_t ibatLpfPeriod;                  // Period of the cutoff frequency for the Ibat filter (in 0.1 s)

    uint16_t vbatUpdateHz;                  // Update rate for voltage task
    uint16_t ibatUpdateHz;                  // Update rate for current task

} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);


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
uint8_t getBatteryCellCount(void);
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
