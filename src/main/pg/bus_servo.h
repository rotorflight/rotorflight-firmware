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

// SBUS has 18 channels, FBUS has 16 channels

// Bus servo defaults (S9-S26) - constrained to BUS_SERVO_MIN/MAX range
#define DEFAULT_BUS_SERVO_MIN     -500
#define DEFAULT_BUS_SERVO_MAX      500
#define DEFAULT_BUS_SERVO_SCALE    1000
#define BUS_SERVO_MAX_SIGNAL       2000
#define BUS_SERVO_MIN_SIGNAL       1000

// S1-S8 (indices 0-7) are PWM servos
// S9-S26 (indices 8-25) are BUS servos for SBUS/FBUS
#define BUS_SERVO_OFFSET 8

typedef enum {
    BUS_SERVO_SOURCE_MIXER = 0,
    BUS_SERVO_SOURCE_RX = 1
} busServoSourceType_e;

typedef struct busServoConfig_s {
    uint8_t sourceType[BUS_SERVO_CHANNELS];
} busServoConfig_t;

PG_DECLARE(busServoConfig_t, busServoConfig);

// Bus servo output functions
void setBusServoOutput(uint8_t channel, float value);
uint16_t getBusServoOutput(uint8_t channel);

// Bus servo configuration helpers
bool hasBusServosConfigured(void);
