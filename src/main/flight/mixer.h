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

#include "platform.h"

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/motors.h"
#include "flight/servos.h"

#include "flight/mixer_init.h"

typedef struct mixerConfig_s {
    uint8_t unused;
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

extern float motor[MAX_SUPPORTED_MOTORS];

void mixerInit(void);
void mixerInitProfile(void);

void mixerUpdate(void);

float mixerGetMotorOutput(uint8_t motor);

float mixerGetThrottle(void);
