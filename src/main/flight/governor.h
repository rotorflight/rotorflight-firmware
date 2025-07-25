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

#include "platform.h"

#include "pg/governor.h"

#include "flight/pid.h"

typedef enum {
    GOV_MODE_NONE = 0,
    GOV_MODE_EXTERNAL,
    GOV_MODE_ELECTRIC,
    GOV_MODE_NITRO,
} govMode_e;

typedef enum {
    GOV_STATE_THROTTLE_OFF,
    GOV_STATE_THROTTLE_IDLE,
    GOV_STATE_SPOOLUP,
    GOV_STATE_RECOVERY,
    GOV_STATE_ACTIVE,
    GOV_STATE_THROTTLE_HOLD,
    GOV_STATE_FALLBACK,
    GOV_STATE_AUTOROTATION,
    GOV_STATE_BAILOUT,
    GOV_STATE_DISABLED,
} govState_e;

typedef enum {
    GOV_FLAG_3POS_THROTTLE,
    GOV_FLAG_TX_THROTTLE_CURVE,
    GOV_FLAG_FALLBACK_PRECOMP,
    GOV_FLAG_VOLTAGE_COMP,
    GOV_FLAG_PID_SPOOLUP,
    GOV_FLAG_HS_ADJUSTMENT,
    GOV_FLAG_DYN_MIN_THROTTLE,
    GOV_FLAG_AUTOROTATION,
    GOV_FLAG_SUSPEND,
    GOV_FLAG_BYPASS,
} govFlags_e;

typedef struct {
    int32_t    pidTerms[4];
    int32_t    pidSum;
    uint16_t   targetHS;
    uint16_t   requestHS;
} govLogData_t;

void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate(void);

bool getGovernerEnabled(void);
void setGovernorEnabled(bool enabled);

int getGovernorState(void);

float getGovernorOutput(void);

void getGovernorLogData(govLogData_t *data);

float getFullHeadSpeedRatio(void);
float getSpoolUpRatio(void);


float getTTAIncrease(void);

bool isSpooledUp(void);

