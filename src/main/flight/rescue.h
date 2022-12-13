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

enum {
    RESCUE_MODE_OFF = 0,
    RESCUE_MODE_CLIMB,
    RESCUE_MODE_ALT_HOLD,
};

uint8_t getRescueState(void);

void rescueUpdate();
float rescueApply(uint8_t axis, float setpoint);
void rescueInitProfile(const pidProfile_t *pidProfile);
