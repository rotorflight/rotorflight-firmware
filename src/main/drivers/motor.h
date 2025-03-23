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
 *
 * Author: jflyper
 */

#pragma once

#include "common/time.h"

#include "pg/motor.h"

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_RESERVED,  // BRUSHED
    PWM_TYPE_DSHOT150,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_PROSHOT1000,
    PWM_TYPE_CASTLE_LINK,
    PWM_TYPE_DISABLED,
    PWM_TYPE_MAX
} motorPwmProtocolTypes_e;

typedef enum {
    MOTOR_CONTROL_UNIDIR = 0,
    MOTOR_CONTROL_BIDIR,
} motorControlMode_e;

typedef struct motorVTable_s {
    void (*postInit)(void);
    bool (*enable)(void);
    void (*disable)(void);
    void (*shutdown)(void);
    bool (*updateStart)(void);
    void (*updateComplete)(void);
    void (*write)(uint8_t index, uint8_t mode, float value);
    void (*writeInt)(uint8_t index, uint16_t value);
    bool (*isMotorEnabled)(uint8_t index);
} motorVTable_t;

typedef struct motorDevice_s {
    motorVTable_t vTable;
    uint8_t       count;
    bool          initialized;
    bool          enabled;
    timeMs_t      motorEnableTimeMs;
    uint8_t       motorControlMode[MAX_SUPPORTED_MOTORS];
} motorDevice_t;


void motorPostInitNull();
void motorWriteNull(uint8_t index, uint8_t mode, float value);
bool motorUpdateStartNull(void);
void motorUpdateCompleteNull(void);

void motorPostInit();
void motorWriteAll(float *values);

void motorDevInit(const struct motorDevConfig_s *motorConfig, uint8_t motorCount);

unsigned motorDeviceCount(void);

motorVTable_t motorGetVTable(void);

bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig);
bool checkMotorProtocolDshot(const motorDevConfig_t *motorDevConfig);

bool isMotorProtocolDshot(void);
bool isMotorProtocolEnabled(void);
bool isMotorProtocolCastlePWM(void);

void motorDisable(void);
void motorEnable(void);
bool motorIsEnabled(void);
bool motorIsMotorEnabled(uint8_t index);
timeMs_t motorGetMotorEnableTimeMs(void);
void motorShutdown(void);

#ifdef USE_DSHOT_BITBANG
bool isDshotBitbangActive(const motorDevConfig_t *motorConfig);
#endif
