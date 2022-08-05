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

#include <stdbool.h>

#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pid.h"


#define PID_CONTROLLER_BETAFLIGHT   1

#define PID_PROCESS_DENOM_DEFAULT   8
#define MAX_PID_PROCESS_DENOM       16

#define PID_GAIN_MAX                2500

#define PID_ROLL_DEFAULT            { .P = 50, .I = 100, .D = 0, .F = 100, }
#define PID_PITCH_DEFAULT           { .P = 50, .I = 100, .D = 0, .F = 100, }
#define PID_YAW_DEFAULT             { .P = 50, .I =  50, .D = 0, .F =   0, }

#define PID_NAMES                   "ROLL;PITCH;YAW;"


typedef struct {
    float P;
    float I;
    float D;
    float F;
    float pidSum;
    float setPoint;
    float gyroRate;
    float gyroDterm;
} pidAxisData_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidAxisConfig_t;

typedef struct pid_s {
    float dT;
    float freq;

    pidAxisData_t data[XYZ_AXIS_COUNT];
    pidAxisConfig_t conf[XYZ_AXIS_COUNT];

} pid_t;


void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

void resetPidProfile(pidProfile_t *profile);

void pidResetIterm(int axis);

void pidInit(const pidProfile_t *pidProfile);
void pidInitProfile(const pidProfile_t *pidProfile);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);

float pidGetDT();
float pidGetPidFrequency();

float pidGetSetpoint(int axis);
float pidGetPidSum(int axis);

const pidAxisData_t * pidGetAxisData(void);

