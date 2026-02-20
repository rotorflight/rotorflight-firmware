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

#include "types.h"
#include "platform.h"

#ifdef USE_SERVOS

#include "pg/pg_ids.h"
#include "pg/servos.h"
#include "bus_servo.h"

#include "config/config_reset.h"

#include "drivers/timer.h"

#include "flight/servos.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 1);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 1);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .mid   = DEFAULT_SERVO_CENTER,
                     .min   = DEFAULT_SERVO_MIN,
                     .max   = DEFAULT_SERVO_MAX,
                     .rneg  = DEFAULT_SERVO_SCALE,
                     .rpos  = DEFAULT_SERVO_SCALE,
                     .rate  = DEFAULT_SERVO_RATE,
                     .speed = DEFAULT_SERVO_SPEED,
                     .flags = DEFAULT_SERVO_FLAGS,
        );
        
        // S1-S8 (indices 0-7) are PWM servos with wider range
        // S9-S26 (indices 8-25) are BUS servos with constrained range
        if (i > 7) {
            instance[i].min = DEFAULT_BUS_SERVO_MIN;
            instance[i].max = DEFAULT_BUS_SERVO_MAX;
            instance[i].rneg = DEFAULT_BUS_SERVO_SCALE;
            instance[i].rpos = DEFAULT_BUS_SERVO_SCALE;
        }
    }
}

#endif
