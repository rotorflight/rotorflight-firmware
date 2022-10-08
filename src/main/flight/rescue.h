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
    RESCUE_OFF,
    RESCUE_INIT_CLIMB,
    RESCUE_FLIP_OVER,
    RESCUE_FINAL_CLIMB,
    RESCUE_STABILISED,
    RESCUE_ALT_HOLD,
} rescueState_e;

typedef struct rescueConfig_s {
    uint8_t  rescue_mode;
    uint16_t rescue_climb_time;
    uint16_t recuue_climb_collective;
} rescueConfig_t;

//PG_DECLARE(rescueConfig_t, rescueConfig);


void rescueInit(const pidProfile_t *pidProfile);
void rescueInitProfile(const pidProfile_t *pidProfile);

void rescueUpdate();

