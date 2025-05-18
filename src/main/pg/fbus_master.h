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

#define FBUS_MASTER_CHANNELS 16

typedef enum {
    FBUS_MASTER_SOURCE_NONE = 0,
    FBUS_MASTER_SOURCE_RX = 1,
    FBUS_MASTER_SOURCE_MIXER = 2,
    FBUS_MASTER_SOURCE_SERVO = 3,
    FBUS_MASTER_SOURCE_MOTOR = 4
} fbusMasterSourceType_e;

typedef struct fbusMasterConfig_s {
    uint8_t sourceType[FBUS_MASTER_CHANNELS];

    // channel index, rule index or servo index.
    uint8_t sourceIndex[FBUS_MASTER_CHANNELS];

    // The min max define how source values map to the sbus valuex for each
    // channel. The module maps source value [min, max] to sbus value [192,
    // 1792] (full-scale channels) or [0, 1] (on-off channels). Typically:
    //   * min = 1000, max = 2000 or min = 988, max = 2012 for receiver channels.
    //   * min = -1000, max = +1000 for mixer rule (treat as -1.0f and +1.0f).
    //   * min = 1000, max = 2000 for wideband servo, or min = 500, max = 1000
    //   * min = 0, max = 1000 for motor (treat as 0 and +1.0f)
    //   for narrowband servo.
    int16_t sourceRangeLow[FBUS_MASTER_CHANNELS];
    int16_t sourceRangeHigh[FBUS_MASTER_CHANNELS];

    // SBus output frame rate in Hz, typically 144Hz. Your receiver may support
    // faster updates.
    uint8_t frameRate;

    uint8_t pinSwap;

} fbusMasterConfig_t;

PG_DECLARE(fbusMasterConfig_t, fbusMasterConfig);
