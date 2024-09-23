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

#include "pg/pg.h"

#define RPM_FILTER_BANK_COUNT 16

typedef struct rpmFilteConfig_s
{
    uint8_t  filter_bank_rpm_source[3*RPM_FILTER_BANK_COUNT];     // RPM source
    uint16_t filter_bank_rpm_ratio[3*RPM_FILTER_BANK_COUNT];      // RPM ratio *1000
    uint16_t filter_bank_rpm_limit[3*RPM_FILTER_BANK_COUNT];      // RPM minimum limit
    uint8_t  filter_bank_notch_q[3*RPM_FILTER_BANK_COUNT];        // Notch Q *10

} rpmFilterConfig_t;


PG_DECLARE(rpmFilterConfig_t, rpmFilterConfig);

