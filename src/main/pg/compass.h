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

#include "common/time.h"
#include "common/sensor_alignment.h"

#include "drivers/io_types.h"
#include "drivers/sensor.h"

#include "sensors/sensors.h"

#include "pg/pg.h"


typedef enum {
    MAG_DEFAULT         = 0,
    MAG_NONE            = 1,
    MAG_HMC5883         = 2,
    MAG_AK8975          = 3,
    MAG_AK8963          = 4,
    MAG_QMC5883         = 5,
    MAG_LIS3MDL         = 6,
    MAG_MPU925X_AK8963  = 7
} magSensor_e;


typedef struct {
    uint8_t     mag_alignment;                  // mag alignment
    uint8_t     mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t     mag_busType;
    uint8_t     mag_i2c_device;
    uint8_t     mag_i2c_address;
    uint8_t     mag_spi_device;
    ioTag_t     mag_spi_csn;
    ioTag_t     interruptTag;
    flightDynamicsTrims_t   magZero;
    sensorAlignment_t       mag_customAlignment;
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);
