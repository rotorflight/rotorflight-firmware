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

#include "platform.h"

#include "drivers/crsf_sensors.h"
#include "pg/crsf_sensors.h"
#include "pg/pg_ids.h"

#ifdef USE_CRSF_SENSORS

PG_REGISTER_WITH_RESET_FN(crsfSensorsConfig_t, crsfSensorsConfig, PG_DRIVER_CRSF_SENSORS_CONFIG, 0);

void pgResetFn_crsfSensorsConfig(crsfSensorsConfig_t *config)
{
    config->sensorTimeoutMs = CRSF_SENSORS_TIMEOUT_MS_DEFAULT;
    config->useBaroAltitude = 0;
    // On, by default: real-world CRSF sensor accessories commonly wire up
    // the opposite way round from this port's un-swapped default, per the
    // wiring investigation that added this option.
    config->pinSwap = 1;
    config->batterySource = CRSF_SENSORS_BATTERY_SOURCE_AUTO;
    config->useRpm = 0;
}

#endif
