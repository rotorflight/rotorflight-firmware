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
#include "pg/adjustments.h"

#include "flight/pid.h"

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

ADJFUN_DECLARE(GOV_GAIN)
ADJFUN_DECLARE(GOV_P_GAIN)
ADJFUN_DECLARE(GOV_I_GAIN)
ADJFUN_DECLARE(GOV_D_GAIN)
ADJFUN_DECLARE(GOV_F_GAIN)
ADJFUN_DECLARE(GOV_TTA_GAIN)
ADJFUN_DECLARE(GOV_CYCLIC_FF)
ADJFUN_DECLARE(GOV_COLLECTIVE_FF)
ADJFUN_DECLARE(GOV_IDLE_THROTTLE)
ADJFUN_DECLARE(GOV_AUTO_THROTTLE)
