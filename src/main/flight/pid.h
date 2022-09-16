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


#define PID_PROCESS_DENOM_DEFAULT   8
#define MAX_PID_PROCESS_DENOM       16

#define PID_GAIN_MAX                2500

#define ROLL_P_TERM_SCALE           (0.0033333333f / 500)
#define ROLL_I_TERM_SCALE           (0.0500000000f / 500)
#define ROLL_D_TERM_SCALE           (0.0000500000f / 500)
#define ROLL_F_TERM_SCALE           (0.0125000000f / 500)

#define PITCH_P_TERM_SCALE          (0.0033333333f / 500)
#define PITCH_I_TERM_SCALE          (0.0500000000f / 500)
#define PITCH_D_TERM_SCALE          (0.0000500000f / 500)
#define PITCH_F_TERM_SCALE          (0.0125000000f / 500)

#define YAW_P_TERM_SCALE            (0.0333333333f / 500)
#define YAW_I_TERM_SCALE            (0.2500000000f / 500)
#define YAW_D_TERM_SCALE            (0.0005000000f / 500)
#define YAW_F_TERM_SCALE            (0.0125000000f / 500)

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
    float prevError;
    float axisError;
} pidAxisData_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidAxisCoef_t;

typedef struct {

    float collectiveDeflectionLPF;

    float collectiveImpulseFilterGain;
    float pitchCollectiveFFGain;
    float pitchCollectiveImpulseFFGain;

    float yawCyclicFFGain;
    float yawCollectiveFFGain;
    float yawCollectiveImpulseFFGain;
} pidPrecomp_t;

typedef struct pid_s {
    float dT;
    float freq;

    uint8_t pidMode;

    uint8_t itermRelaxType;
    uint8_t itermRelaxLevel[PID_AXIS_COUNT];

    uint8_t errorRotation;

    float errorDecay;
    float errorLimit[PID_AXIS_COUNT];

    float yawCWStopGain;
    float yawCCWStopGain;

    float collective;

    pidPrecomp_t precomp;

    pidAxisCoef_t coef[PID_ITEM_COUNT];
    pidAxisData_t data[PID_AXIS_COUNT];

    pt1Filter_t gyrorFilter[PID_AXIS_COUNT];
    pt1Filter_t errorFilter[PID_AXIS_COUNT];
    pt1Filter_t dtermFilter[PID_AXIS_COUNT];
    pt1Filter_t ftermFilter[PID_AXIS_COUNT];
    pt1Filter_t relaxFilter[PID_AXIS_COUNT];

} pid_t;


void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

void resetPidProfile(pidProfile_t *profile);

void pidResetAxisErrors(void);
void pidResetAxisError(int axis);

void pidInit(const pidProfile_t *pidProfile);
void pidInitProfile(const pidProfile_t *pidProfile);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);

float pidGetDT();
float pidGetPidFrequency();

float pidGetSetpoint(int axis);
float pidGetOutput(int axis);

float pidGetCollective();

const pidAxisData_t * pidGetAxisData(void);

