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

#include "types.h"
#include "platform.h"

#include "config/config.h"

#include "pg/pg_ids.h"
#include "pg/battery.h"

#ifndef DEFAULT_CURRENT_METER_SOURCE
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_NONE
#endif

#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_NONE
#endif


PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    .batteryCellCount = 0,
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,
    .vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX,
    .vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN,
    .vbatfullcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_FULL,
    .vbatwarningcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_WARN,
    .vbatnotpresentcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_ABSENT,
    .vbathysteresis = 1,
    .lvcPercentage = 100, // Off by default at 100%
    .batteryCapacity = 0,
    .consumptionWarningPercentage = 10,
    .useVoltageAlerts = true,
    .useConsumptionAlerts = false,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
    .vbatLpfHz = 10,
    .ibatLpfHz = 10,
    .vbatUpdateHz = VOLTAGE_TASK_FREQ_HZ,
    .ibatUpdateHz = CURRENT_TASK_FREQ_HZ,
);

