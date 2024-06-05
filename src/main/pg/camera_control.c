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

#ifdef USE_CAMERA_CONTROL

#include "config/config_reset.h"

#include "drivers/timer.h"

#include "pg/pg_ids.h"
#include "pg/camera_control.h"

PG_REGISTER_WITH_RESET_FN(cameraControlConfig_t, cameraControlConfig, PG_CAMERA_CONTROL_CONFIG, 0);

void pgResetFn_cameraControlConfig(cameraControlConfig_t *cameraControlConfig)
{
    cameraControlConfig->mode = CAMERA_CONTROL_MODE_HARDWARE_PWM;
    cameraControlConfig->refVoltage = 330;
    cameraControlConfig->keyDelayMs = 180;
    cameraControlConfig->internalResistance = 470;
    cameraControlConfig->ioTag = timerioTagGetByUsage(TIM_USE_CAMERA_CONTROL, 0);
    cameraControlConfig->inverted = 0;   // Output is inverted externally
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_ENTER] = 450;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_LEFT]  = 270;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_UP]    = 150;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_RIGHT] = 68;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_DOWN]  = 0;
}

#endif
