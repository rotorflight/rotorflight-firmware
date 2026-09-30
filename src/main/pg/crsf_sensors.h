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

#include <stdint.h>

#include "pg/pg.h"

// Which decoded source feeds battery_meter/current_meter = CRSF:
//   AUTO    - prefer the aggregate Battery Sensor frame (0x08, has voltage,
//             current, capacity and remaining%); fall back to summed Cells
//             (0x0E, voltage only) if no 0x08 frame is present.
//   CURRENT - always use the Battery Sensor frame (0x08) - named for the
//             one field only it provides, since Cells has no current data.
//   VOLTAGE - always use summed Cells (0x0E), even if a Battery Sensor
//             frame is also present.
typedef enum {
    CRSF_SENSORS_BATTERY_SOURCE_AUTO = 0,
    CRSF_SENSORS_BATTERY_SOURCE_CURRENT,
    CRSF_SENSORS_BATTERY_SOURCE_VOLTAGE,
} crsfSensorsBatterySource_e;

typedef struct crsfSensorsConfig_s {
    uint16_t sensorTimeoutMs;
    uint8_t useBaroAltitude;
    uint8_t pinSwap;
    uint8_t batterySource;
    uint8_t useRpm;
} crsfSensorsConfig_t;

PG_DECLARE(crsfSensorsConfig_t, crsfSensorsConfig);
