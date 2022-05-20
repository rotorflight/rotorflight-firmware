/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#define QUAD_MOTOR_COUNT 4

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_s {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct mixerConfig_s {
    uint8_t unused;
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];
struct rxConfig_s;

uint8_t getMotorCount(void);
float getMotorMixRange(void);
bool areMotorsRunning(void);

void initEscEndpoints(void);
void mixerInit(void);
void mixerInitProfile(void);

void mixerResetDisarmedMotors(void);
void mixTable(timeUs_t currentTimeUs);
void stopMotors(void);
void writeMotors(void);

float mixerGetThrottle(void);

float getMotorOutputLow(void);
float getMotorOutputHigh(void);
