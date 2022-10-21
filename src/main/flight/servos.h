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

#define DEFAULT_SERVO_FLAGS      0
#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_MIN     1000
#define DEFAULT_SERVO_MAX     2000
#define DEFAULT_SERVO_RANGE    500
#define DEFAULT_SERVO_RATE     333

#define SERVO_LIMIT_MIN        100
#define SERVO_LIMIT_MAX       2500
#define SERVO_RANGE_MIN        100
#define SERVO_RANGE_MAX       1000
#define SERVO_RATE_MIN          50
#define SERVO_RATE_MAX        1000
#define SERVO_OVERRIDE_MIN   -2000
#define SERVO_OVERRIDE_MAX    2000
#define SERVO_OVERRIDE_OFF   (SERVO_OVERRIDE_MAX + 1)

enum {
    SERVO_FLAG_REVERSED     = BIT(0),
    SERVO_FLAG_GEO_CORR     = BIT(1),
    SERVO_FLAGS_ALL         = BIT(2) - 1,
};

typedef struct servoParam_s {
    uint16_t    mid;     // center (mid) point
    uint16_t    min;     // lower limit in us
    uint16_t    max;     // upper limit in us
    uint16_t    rneg;    // negative range in us
    uint16_t    rpos;    // positive range in us
    uint16_t    rate;    // servo update rate Hz
    uint16_t    flags;   // feature flags
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    ioTag_t  ioTags[MAX_SUPPORTED_SERVOS];
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

void servoInit(void);
void servoUpdate(void);

void validateAndFixServoConfig(void);

uint8_t getServoCount(void);
uint16_t getServoOutput(uint8_t servo);

int16_t getServoOverride(uint8_t servo);
int16_t setServoOverride(uint8_t servo, int16_t val);
bool    hasServoOverride(uint8_t servo);

