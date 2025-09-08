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
    GM_OFF = 0,
    GM_PASSTHROUGH,
    GM_STANDARD,
    GM_MODE1,
    GM_MODE2,
} govMode_e;

typedef enum {
    GS_THROTTLE_OFF,
    GS_THROTTLE_IDLE,
    GS_SPOOLING_UP,
    GS_RECOVERY,
    GS_ACTIVE,
    GS_ZERO_THROTTLE,
    GS_LOST_HEADSPEED,
    GS_AUTOROTATION,
    GS_AUTOROTATION_BAILOUT,
} govState_e;


void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate(void);

int getGovernorState(void);

float getGovernorOutput(void);

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
