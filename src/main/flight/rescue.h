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

#include "pg/pg.h"

#include "flight/pid.h"

typedef enum {
    RESCUE_MODE_OFF = 0,
    RESCUE_MODE_CLIMB,
    RESCUE_MODE_ALT_HOLD,
} rescueMode_e;

typedef enum {
    RESCUE_STATE_OFF = 0,
    RESCUE_STATE_PULLUP,
    RESCUE_STATE_FLIP,
    RESCUE_STATE_CLIMB,
    RESCUE_STATE_HOVER,
    RESCUE_STATE_EXIT,
} rescueState_e;


int getRescueState(void);

void rescueUpdate();
float rescueApply(uint8_t axis, float setpoint);
void rescueInitProfile(const pidProfile_t *pidProfile);

ADJFUN_DECLARE(RESCUE_CLIMB_COLLECTIVE)
ADJFUN_DECLARE(RESCUE_HOVER_COLLECTIVE)
ADJFUN_DECLARE(RESCUE_HOVER_ALTITUDE)
ADJFUN_DECLARE(RESCUE_ALT_P_GAIN)
ADJFUN_DECLARE(RESCUE_ALT_I_GAIN)
ADJFUN_DECLARE(RESCUE_ALT_D_GAIN)
