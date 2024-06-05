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

#include "types.h"
#include "platform.h"

#include "pg/pg.h"


enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
};

enum {
    GYRO_CONFIG_USE_GYRO_1 = 0,
    GYRO_CONFIG_USE_GYRO_2 = 1,
    GYRO_CONFIG_USE_GYRO_BOTH = 2,
};


typedef struct {

    uint8_t     gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t     gyro_hardware_lpf;                // gyro DLPF setting
    uint8_t     gyro_high_fsr;
    uint8_t     gyro_rate_sync;
    uint8_t     gyro_to_use;

    uint16_t    gyro_decimation_hz;

    uint16_t    gyro_lpf1_static_hz;
    uint16_t    gyro_lpf2_static_hz;

    uint16_t    gyro_soft_notch_hz_1;
    uint16_t    gyro_soft_notch_cutoff_1;
    uint16_t    gyro_soft_notch_hz_2;
    uint16_t    gyro_soft_notch_cutoff_2;
    int16_t     gyro_offset_yaw;
    uint8_t     checkOverflow;

    uint8_t     gyro_lpf1_type;
    uint8_t     gyro_lpf2_type;

    uint16_t    gyroCalibrationDuration;        // Gyro calibration duration in 1/100 second

    uint16_t    gyro_lpf1_dyn_min_hz;
    uint16_t    gyro_lpf1_dyn_max_hz;

    uint8_t     gyrosDetected;                  // Automatically set on first startup

} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);
