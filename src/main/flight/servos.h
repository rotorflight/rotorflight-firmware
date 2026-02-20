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

#include "config/config.h"

#include "pg/servos.h"

#define DEFAULT_SERVO_FLAGS      0
#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_MIN     -700
#define DEFAULT_SERVO_MAX      700
#define DEFAULT_SERVO_SCALE    500
#define DEFAULT_SERVO_RATE     333
#define DEFAULT_SERVO_SPEED      0

#define SERVO_LIMIT_MIN      -1000
#define SERVO_LIMIT_MAX       1000
#define SERVO_SCALE_MIN        100
#define SERVO_SCALE_MAX       1000
#define SERVO_RATE_MIN          25
#define SERVO_RATE_MAX        5000
#define SERVO_SPEED_MIN          0
#define SERVO_SPEED_MAX      60000
#define SERVO_OVERRIDE_MIN   -2000
#define SERVO_OVERRIDE_MAX    2000
#define SERVO_OVERRIDE_OFF   (SERVO_OVERRIDE_MAX + 1)

enum {
    SERVO_FLAG_REVERSED     = BIT(0),
    SERVO_FLAG_GEO_CORR     = BIT(1),
    SERVO_FLAGS_ALL         = BIT(2) - 1,
};

void servoInit(void);
void servoUpdate(void);
void servoShutdown(void);

void validateAndFixServoConfig(void);

uint8_t getServoCount(void);
uint16_t getServoOutput(uint8_t servo);

int16_t getServoOverride(uint8_t servo);
int16_t setServoOverride(uint8_t servo, int16_t val);
bool    hasServoOverride(uint8_t servo);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
float geometryCorrection(float pos);
#endif

