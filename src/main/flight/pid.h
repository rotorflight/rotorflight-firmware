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

#define FILTER_PROCESS_DENOM_DEFAULT 0

#define PID_GAIN_MAX                1000

#define ROLL_P_TERM_SCALE           0.00000666666f
#define ROLL_I_TERM_SCALE           0.0002f
#define ROLL_D_TERM_SCALE           0.1e-6f
#define ROLL_F_TERM_SCALE           0.000025f
#define ROLL_B_TERM_SCALE           0.1e-6f

#define PITCH_P_TERM_SCALE          0.00000666666f
#define PITCH_I_TERM_SCALE          0.0002f
#define PITCH_D_TERM_SCALE          1.0e-6f
#define PITCH_F_TERM_SCALE          0.000025f
#define PITCH_B_TERM_SCALE          0.1e-6f

#define YAW_P_TERM_SCALE            0.00006666666f
#define YAW_I_TERM_SCALE            0.0005f
#define YAW_D_TERM_SCALE            1.0e-6f
#define YAW_F_TERM_SCALE            0.000025f
#define YAW_B_TERM_SCALE            1.0e-6f

#define CROSS_COUPLING_SCALE        10.0e-6f

typedef struct {
    float P;
    float I;
    float D;
    float F;
    float B;
    float O;
    float pidSum;
    float setPoint;
    float gyroRate;
    float axisError;
    float axisOffset;
} pidAxisData_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Kb;
    float Ko;
    float Kc;
} pidAxisCoef_t;

typedef struct {

    filter_t yawPrecompFilter;
    filter_t headspeedFilter;
    difFilter_t yawInertiaFilter;

    float yawCollectiveFFGain;
    float yawCyclicFFGain;
    float yawInertiaGain;

    float pitchCollectiveFFGain;

} pidPrecomp_t;

typedef struct pid_s {
    float dT;
    float freq;

    uint8_t pidMode;

    uint8_t itermRelaxType;
    uint8_t itermRelaxLevel[PID_AXIS_COUNT];

    uint8_t errorRotation;

    float errorDecayRateGround;
    float errorDecayRateCyclic;
    float errorDecayLimitCyclic;
    float errorDecayRateYaw;
    float errorDecayLimitYaw;

    float offsetFloodRelaxLevel;

    float offsetLimit[XY_AXIS_COUNT];
    float errorLimit[PID_AXIS_COUNT];

    float yawCWStopGain;
    float yawCCWStopGain;

    float cyclicCrossCouplingGain[XY_AXIS_COUNT];

    float collective;

    pidPrecomp_t precomp;

    pidAxisCoef_t coef[PID_ITEM_COUNT];
    pidAxisData_t data[PID_AXIS_COUNT];

    filter_t gyrorFilter[PID_AXIS_COUNT];

    pt1Filter_t relaxFilter[PID_AXIS_COUNT];

    difFilter_t dtermFilter[PID_AXIS_COUNT];
    difFilter_t btermFilter[PID_AXIS_COUNT];

    order1Filter_t crossCouplingFilter[XY_AXIS_COUNT];

    pt1Filter_t offsetFloodRelaxFilter;

} pidData_t;


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

