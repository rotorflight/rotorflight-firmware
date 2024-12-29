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

#include "common/sensor_alignment.h"
#include "sensors/sensors.h"

#include "pg/pg.h"


typedef enum {
    ACC_DEFAULT,
    ACC_NONE,
    ACC_ADXL345,
    ACC_MPU6050,
    ACC_MMA8452,
    ACC_BMA280,
    ACC_LSM303DLHC,
    ACC_MPU6000,
    ACC_MPU6500,
    ACC_MPU9250,
    ACC_ICM20601,
    ACC_ICM20602,
    ACC_ICM20608G,
    ACC_ICM20649,
    ACC_ICM20689,
    ACC_ICM42605,
    ACC_ICM42688P,
    ACC_BMI160,
    ACC_BMI270,
    ACC_LSM6DSO,
    ACC_BMI088,
    ACC_FAKE
} accelerationSensor_e;

typedef union {
    int16_t raw[2];
    struct {
        int16_t roll;
        int16_t pitch;
    } values;
} rollAndPitchTrims_t;

typedef struct {
    uint8_t                 acc_hardware;       // Which acc hardware to use on boards with more than one device
    uint8_t                 acc_high_fsr;
    uint16_t                acc_lpf_hz;         // cutoff frequency for the low pass filter used on the acc z-axis
    flightDynamicsTrims_t   accZero;
    rollAndPitchTrims_t     accelerometerTrims;
} accelerometerConfig_t;

PG_DECLARE(accelerometerConfig_t, accelerometerConfig);


void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
