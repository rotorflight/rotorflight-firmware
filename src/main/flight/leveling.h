/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/axis.h"

#include "pg/pid.h"
#include "pg/adjustments.h"

void levelingInit(const pidProfile_t *pidProfile);

float angleModeApply(int axis, float pidSetpoint);
float horizonModeApply(int axis, float pidSetpoint);

ADJFUN_DECLARE(ANGLE_LEVEL_GAIN)
ADJFUN_DECLARE(HORIZON_LEVEL_GAIN)
