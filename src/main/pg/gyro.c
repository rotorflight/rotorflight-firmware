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

#include "config/config_reset.h"

#include "common/filter.h"

#include "drivers/accgyro/accgyro.h"

#include "pg/pg_ids.h"
#include "pg/gyro.h"

#define GYRO_LPF1_TYPE_DEFAULT          LPF_1ST_ORDER
#define GYRO_LPF1_HZ_DEFAULT            100
#define GYRO_LPF1_DYN_MIN_HZ_DEFAULT    0
#define GYRO_LPF1_DYN_MAX_HZ_DEFAULT    0

#define GYRO_LPF2_TYPE_DEFAULT          LPF_NONE
#define GYRO_LPF2_HZ_DEFAULT            50

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif


PG_REGISTER_WITH_RESET_FN(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 9);

void pgResetFn_gyroConfig(gyroConfig_t *gyroConfig)
{
    gyroConfig->gyroCalibrationDuration = 125;        // 1.25 seconds
    gyroConfig->gyroMovementCalibrationThreshold = 48;
    gyroConfig->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    gyroConfig->gyro_decimation_hz = 500;
    gyroConfig->gyro_lpf1_type = GYRO_LPF1_TYPE_DEFAULT;
    gyroConfig->gyro_lpf1_static_hz = GYRO_LPF1_HZ_DEFAULT;
    gyroConfig->gyro_lpf2_type = GYRO_LPF2_TYPE_DEFAULT;
    gyroConfig->gyro_lpf2_static_hz = GYRO_LPF2_HZ_DEFAULT;
    gyroConfig->gyro_lpf1_dyn_min_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
    gyroConfig->gyro_lpf1_dyn_max_hz = GYRO_LPF1_DYN_MAX_HZ_DEFAULT;
    gyroConfig->gyro_high_fsr = false;
    gyroConfig->gyro_rate_sync = true;
    gyroConfig->gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT;
    gyroConfig->gyro_soft_notch_hz_1 = 0;
    gyroConfig->gyro_soft_notch_cutoff_1 = 0;
    gyroConfig->gyro_soft_notch_hz_2 = 0;
    gyroConfig->gyro_soft_notch_cutoff_2 = 0;
    gyroConfig->checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;
}

