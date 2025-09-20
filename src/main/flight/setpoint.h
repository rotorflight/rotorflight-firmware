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
#include <stdint.h>
#include <math.h>

#include "pg/rates.h"
#include "pg/adjustments.h"

float getSetpoint(int axis);
float getDeflection(int axis);

void setpointInit(void);
void setpointInitProfile(void);

void setpointUpdateTiming(float frameTimeUs);

void setpointUpdate(void);

bool isHandsOn(void);
bool isAirborne(void);

ADJFUN_DECLARE(PITCH_SP_BOOST_GAIN)
ADJFUN_DECLARE(ROLL_SP_BOOST_GAIN)
ADJFUN_DECLARE(YAW_SP_BOOST_GAIN)
ADJFUN_DECLARE(COLL_SP_BOOST_GAIN)
ADJFUN_DECLARE(YAW_DYN_CEILING_GAIN)
ADJFUN_DECLARE(YAW_DYN_DEADBAND_GAIN)
ADJFUN_DECLARE(YAW_DYN_DEADBAND_FILTER)
