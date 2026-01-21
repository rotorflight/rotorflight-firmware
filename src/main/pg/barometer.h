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

#include "drivers/barometer/barometer.h"

#include "pg/pg.h"


typedef enum {
    BARO_DEFAULT    = 0,
    BARO_NONE       = 1,
    BARO_BMP085     = 2,
    BARO_MS5611     = 3,
    BARO_BMP280     = 4,
    BARO_LPS        = 5,
    BARO_QMP6988    = 6,
    BARO_BMP388     = 7,
    BARO_DPS310     = 8,
    BARO_BMP581     = 9,
} baroSensor_e;


typedef struct {
    uint8_t baro_busType;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    ioTag_t baro_eoc_tag;
    ioTag_t baro_xclr_tag;
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);
