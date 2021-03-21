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

#include <stdint.h>
#include <stdbool.h>

#include "flight/pid.h"

#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT  500.0f
#define ACRO_TRAINER_SETPOINT_LIMIT        1000.0f


void acroTrainerInit(const pidProfile_t *pidProfile);
void acroTrainerReset(void);
void acroTrainerSetState(bool state);

float acroTrainerApply(int axis, float setPoint);

