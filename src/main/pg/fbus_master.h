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

#include "common/utils.h"
#include "pg/pg.h"
#include "pg/bus_servo.h"

#define FBUS_MASTER_CHANNELS 16

#define FBUS_MIN 192
#define FBUS_MAX 1792

// Backward compatibility aliases
typedef busServoSourceType_e fbusMasterSourceType_e;

#define FBUS_MASTER_MAX_FORWARDED_SENSORS 8

#define FBUS_MASTER_TELEMETRY_RATE_MIN_HZ 25
#define FBUS_MASTER_TELEMETRY_RATE_MAX_HZ 550
#define FBUS_MASTER_DISCOVERY_TIME_MIN_MS 100
#define FBUS_MASTER_DISCOVERY_TIME_MAX_MS 10000

typedef struct fbusMasterConfig_s {
    uint16_t frameRate;
    uint8_t pinSwap;

    // When ON, the UART output is electrically inverted (F.Bus signal uses
    // inverted logic). When OFF, the output is non-inverted.
    uint8_t inverted;

    uint16_t telemetryRate;
    uint16_t sensorDiscoveryTimeMs;
    // List of sensor physical IDs to forward to the receiver (max 8)
    // 0 = disabled slot
    uint8_t forwardedSensors[FBUS_MASTER_MAX_FORWARDED_SENSORS];

} fbusMasterConfig_t;

PG_DECLARE(fbusMasterConfig_t, fbusMasterConfig);
