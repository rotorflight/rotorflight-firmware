/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"

// WHO_AM_I value of the LSM6DSK320X. The pin compatible LSM6DSV16X (0x70) and LSM6DSV320X
// (0x73) are different parts and are not driven by this driver.
#define LSM6DSK320X_WHO_AM_I_CONST          (0x75)

uint8_t lsm6dsk320xSpiDetect(const extDevice_t *dev);
bool lsm6dsk320xSpiAccDetect(accDev_t *acc);
bool lsm6dsk320xSpiGyroDetect(gyroDev_t *gyro);
