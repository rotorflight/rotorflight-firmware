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


bool isRpmSourceActive(void);

int getMotorRPM(uint8_t motor);
float getMotorRPMf(uint8_t motor);

int getMotorRawRPM(uint8_t motor);
float getMotorRawRPMf(uint8_t motor);

int calcMotorRPM(uint8_t motor, int erpm);
float calcMotorRPMf(uint8_t motor, int erpm);

void rpmSourceInit(void);
void rpmSourceUpdate(void);

static inline float getHeadSpeed(void) { return getMotorRPMf(0); } // TODO

