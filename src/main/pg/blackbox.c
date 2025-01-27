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

#ifdef USE_BLACKBOX

#include "pg/pg_ids.h"
#include "pg/blackbox.h"

#include "blackbox/blackbox_fielddefs.h"


#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif


PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 2);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .mode = BLACKBOX_MODE_NORMAL,
    .denom = 8,
    .fields = BIT(FLIGHT_LOG_FIELD_SELECT_COMMAND) |
              BIT(FLIGHT_LOG_FIELD_SELECT_SETPOINT) |
              BIT(FLIGHT_LOG_FIELD_SELECT_MIXER) |
              BIT(FLIGHT_LOG_FIELD_SELECT_PID) |
              BIT(FLIGHT_LOG_FIELD_SELECT_ATTITUDE) |
              BIT(FLIGHT_LOG_FIELD_SELECT_GYRAW) |
              BIT(FLIGHT_LOG_FIELD_SELECT_GYRO) |
              BIT(FLIGHT_LOG_FIELD_SELECT_ALT) |
              BIT(FLIGHT_LOG_FIELD_SELECT_BATTERY) |
              BIT(FLIGHT_LOG_FIELD_SELECT_RSSI) |
              BIT(FLIGHT_LOG_FIELD_SELECT_RPM) |
              BIT(FLIGHT_LOG_FIELD_SELECT_MOTOR) |
              BIT(FLIGHT_LOG_FIELD_SELECT_SERVO) |
              BIT(FLIGHT_LOG_FIELD_SELECT_VBEC) |
              BIT(FLIGHT_LOG_FIELD_SELECT_VBUS) |
              BIT(FLIGHT_LOG_FIELD_SELECT_VBUS) |
              BIT(FLIGHT_LOG_FIELD_SELECT_TEMP) |
              0,
    .initialEraseFreeSpaceKiB = 0,
    .rollingErase = 0,
);

#endif
