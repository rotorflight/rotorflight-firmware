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

#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_MIN     -500
#define DEFAULT_SERVO_MAX      500
#define DEFAULT_SERVO_RATE     500
#define DEFAULT_SERVO_TRIM       0
#define DEFAULT_SERVO_SPEED      0
#define DEFAULT_SERVO_UPDATE   333
#define DEFAULT_SERVO_FLAGS      0

#define SERVO_RANGE_MIN      -1000
#define SERVO_RANGE_MAX       1000
#define SERVO_RATE_MIN       -2500
#define SERVO_RATE_MAX        2500
#define SERVO_TRIM_MIN        -250
#define SERVO_TRIM_MAX         250
#define SERVO_SPEED_MIN          0
#define SERVO_SPEED_MAX      10000
#define SERVO_OVERRIDE_MIN   -2000
#define SERVO_OVERRIDE_MAX    2000
#define SERVO_OVERRIDE_OFF   (SERVO_OVERRIDE_MAX + 1)

enum {
    SERVO_FLAG_GEOMETRY_CORRECTION = BIT(0),
    SERVO_FLAG_CURVE_CORRECTION = BIT(1),
    SERVO_FLAG_ALL = BIT(2) - 1,
};

typedef struct servoParam_s {
    uint16_t    mid;        // center point
    int16_t     min;        // movement lower limit in us
    int16_t     max;        // movement upper limit in us
    int16_t     rate;       // scaling in us - sign indicates direction
    int16_t     trim;       // link trim in 0.1% steps
    uint16_t    speed;      // speed limit (ms/50deg) ; 0 = disabled
    uint16_t    flags;      // feature flags
    int8_t      curve[4];   // correction curve
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    servoDevConfig_t dev;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

void servoInit(void);
void servoUpdate(void);

void servoInitCurve(uint8_t servo);

uint8_t getServoCount(void);
int16_t getServoOutput(uint8_t servo);

int16_t getServoOverride(uint8_t servo);
int16_t setServoOverride(uint8_t servo, int16_t val);
bool    hasServoOverride(uint8_t servo);

