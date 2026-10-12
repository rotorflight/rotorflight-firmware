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

#include "drivers/bus.h"

/* -------------------------------------------------------------------------
 * Register map, Bank 0 (default bank on power-up / after reset)
 * Source: ICM-40608 datasheet, Section 13.1 "User Bank 0 Register Map"
 * and Section 14 "User Bank 0 Register Map - Descriptions"
 * ---------------------------------------------------------------------- */

#define ICM40608_RA_DEVICE_CONFIG 0x11 // Sec 14.1
#define ICM40608_BIT_SOFT_RESET (1 << 0)

#define ICM40608_RA_INT_CONFIG 0x14 // Sec 14.3 (not used yet, EXTI not wired)

#define ICM40608_RA_TEMP_DATA1 0x1D // Sec 14.5, upper byte of temp

#define ICM40608_RA_ACCEL_DATA_X1 0x1F // Sec 14.7, start of 6-byte accel burst
#define ICM40608_RA_GYRO_DATA_X1 0x25  // Sec 14.13, start of 6-byte gyro burst

#define ICM40608_RA_INT_STATUS 0x2D // Sec 14.21

#define ICM40608_RA_SIGNAL_PATH_RESET 0x4B // Sec 14.32
#define ICM40608_BIT_DMP_INIT_EN (1 << 6)

#define ICM40608_RA_INTF_CONFIG0 0x4C      // Sec 14.33
#define ICM40608_RA_INTF_CONFIG1 0x4D      // Sec 14.34
#define ICM40608_BIT_CLKSEL_PLL_OR_RC 0x01 // 01: select PLL when available, else RC

#define ICM40608_RA_PWR_MGMT0 0x4E // Sec 14.35
// GYRO_MODE, bits 3:2
#define ICM40608_GYRO_MODE_OFF (0 << 2)
#define ICM40608_GYRO_MODE_STANDBY (1 << 2)
#define ICM40608_GYRO_MODE_LOW_NOISE (3 << 2)
// ACCEL_MODE, bits 1:0
#define ICM40608_ACCEL_MODE_OFF (0 << 0)
#define ICM40608_ACCEL_MODE_LOW_POWER (2 << 0)
#define ICM40608_ACCEL_MODE_LOW_NOISE (3 << 0)

#define ICM40608_RA_GYRO_CONFIG0 0x4F // Sec 14.36
// GYRO_FS_SEL, bits 7:5 (see enum below)
// GYRO_ODR,    bits 3:0 (see enum below)

#define ICM40608_RA_ACCEL_CONFIG0 0x50 // Sec 14.37
// ACCEL_FS_SEL, bits 7:5 (see enum below)
// ACCEL_ODR,    bits 3:0 (see enum below)

#define ICM40608_RA_GYRO_CONFIG1 0x51       // Sec 14.38 (UI filter order etc, optional)
#define ICM40608_RA_GYRO_ACCEL_CONFIG0 0x52 // Sec 14.39 (UI filter bandwidth, optional)
#define ICM40608_RA_ACCEL_CONFIG1 0x53      // Sec 14.40 (optional)

#define ICM40608_RA_WHO_AM_I 0x75     // Sec 14.57
#define ICM40608_RA_REG_BANK_SEL 0x76 // Sec 14.58

/* -------------------------------------------------------------------------
 * Gyroscope full-scale select, GYRO_CONFIG0[7:5] -- Sec 14.36 table
 * ---------------------------------------------------------------------- */
enum icm40608_gyro_fs_e
{
    ICM40608_GYRO_FS_2000DPS = 0,
    ICM40608_GYRO_FS_1000DPS,
    ICM40608_GYRO_FS_500DPS,
    ICM40608_GYRO_FS_250DPS,
    ICM40608_GYRO_FS_125DPS,
    ICM40608_GYRO_FS_62_5DPS,
    ICM40608_GYRO_FS_31_25DPS,
    ICM40608_GYRO_FS_15_625DPS,
};

/* -------------------------------------------------------------------------
 * Accelerometer full-scale select, ACCEL_CONFIG0[7:5] -- Sec 14.37 table
 * ---------------------------------------------------------------------- */
enum icm40608_accel_fs_e
{
    ICM40608_ACCEL_FS_16G = 0,
    ICM40608_ACCEL_FS_8G,
    ICM40608_ACCEL_FS_4G,
    ICM40608_ACCEL_FS_2G,
};

/* -------------------------------------------------------------------------
 * ODR select field, shared encoding for GYRO_CONFIG0[3:0] and
 * ACCEL_CONFIG0[3:0] -- Sec 14.36 / 14.37 tables
 * ---------------------------------------------------------------------- */
enum icm40608_odr_e
{
    ICM40608_ODR_8K = 0x03,
    ICM40608_ODR_4K = 0x04,
    ICM40608_ODR_2K = 0x05,
    ICM40608_ODR_1K = 0x06, // default
    ICM40608_ODR_200HZ = 0x07,
    ICM40608_ODR_100HZ = 0x08,
    ICM40608_ODR_50HZ = 0x09,
    ICM40608_ODR_25HZ = 0x0A,
    ICM40608_ODR_12_5HZ = 0x0B,
    ICM40608_ODR_500HZ = 0x0F,
};

void icm40608AccInit(accDev_t *acc);
void icm40608GyroInit(gyroDev_t *gyro);

uint8_t icm40608SpiDetect(const extDevice_t *dev);

bool icm40608SpiAccDetect(accDev_t *acc);
bool icm40608SpiGyroDetect(gyroDev_t *gyro);

bool icm40608GyroReadSPI(gyroDev_t *gyro);
bool icm40608AccRead(accDev_t *acc);