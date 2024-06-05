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

#define VBAT_CELL_VOLTAGE_DEFAULT_MIN       330
#define VBAT_CELL_VOLTAGE_DEFAULT_MAX       430
#define VBAT_CELL_VOLTAGE_DEFAULT_FULL      410
#define VBAT_CELL_VOLTAGE_DEFAULT_WARN      350
#define VBAT_CELL_VOLTAGE_DEFAULT_ABSENT    300

#ifndef VOLTAGE_TASK_FREQ_HZ
#define VOLTAGE_TASK_FREQ_HZ    50
#endif

#ifndef CURRENT_TASK_FREQ_HZ
#define CURRENT_TASK_FREQ_HZ    50
#endif


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

typedef struct {

    // battery size
    uint16_t    batteryCapacity;            // mAh
    uint8_t     batteryCellCount;           // Number of cells in battery, zero for autodetection

    // sources
    uint8_t     currentMeterSource;         // source of battery current meter used
    uint8_t     voltageMeterSource;         // source of battery voltage meter used

    // voltages
    uint16_t    vbatmaxcellvoltage;         // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 430 (4.30V)
    uint16_t    vbatmincellvoltage;         // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.30V)
    uint16_t    vbatfullcellvoltage;        // Cell voltage at which the battery is deemed to be "full" 0.01V units, default is 410 (4.1V)
    uint16_t    vbatwarningcellvoltage;     // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.50V)
    uint16_t    vbatnotpresentcellvoltage;  // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    uint8_t     vbathysteresis;             // hysteresis for alarm in 0.01V units, default 1 = 0.01V
    uint8_t     lvcPercentage;              // Percentage of throttle when lvc is triggered
    uint8_t     consumptionWarningPercentage; // Percentage of remaining capacity that should trigger a battery warning

    // warnings / alerts
    bool        useVoltageAlerts;           // Issue alerts based on VBat readings
    bool        useConsumptionAlerts;       // Issue alerts based on total power consumption

    uint8_t     vbatDurationForWarning;     // Period voltage has to sustain before the battery state is set to BATTERY_WARNING (in 0.1 s)
    uint8_t     vbatDurationForCritical;    // Period voltage has to sustain before the battery state is set to BATTERY_CRIT (in 0.1 s)

    // Filters
    uint8_t     vbatLpfPeriod;              // Period of the cutoff frequency for the Vbat filter for display and startup (in 0.1 s)
    uint8_t     ibatLpfPeriod;              // Period of the cutoff frequency for the Ibat filter (in 0.1 s)

    uint16_t    vbatUpdateHz;               // Update rate for voltage task
    uint16_t    ibatUpdateHz;               // Update rate for current task

} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);
