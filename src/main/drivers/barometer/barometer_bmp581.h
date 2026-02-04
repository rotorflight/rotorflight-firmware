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
 *
 * BMP581 Driver
 *
 * References:
 * BMP581 datasheet - https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp581/
 */

#pragma once

typedef struct bmp581Config_s {
    ioTag_t eocTag;
} bmp581Config_t;

bool bmp581Detect(const bmp581Config_t *config, baroDev_t *baro);
