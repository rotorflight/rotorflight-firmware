/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "pg/pg.h"

#define RPM_FILTER_BANK_COUNT 16

typedef struct rpmFilteConfig_s
{
    uint8_t  filter_bank_motor_index[RPM_FILTER_BANK_COUNT];    // Motor index
    uint16_t filter_bank_gear_ratio[RPM_FILTER_BANK_COUNT];     // Motor/rotor speed ratio *1000
    uint16_t filter_bank_notch_q[RPM_FILTER_BANK_COUNT];        // Filter Q * 100
    uint16_t filter_bank_min_hz[RPM_FILTER_BANK_COUNT];         // Filter minimum frequency
    uint16_t filter_bank_max_hz[RPM_FILTER_BANK_COUNT];         // Filter maximum frequency

} rpmFilterConfig_t;


PG_DECLARE(rpmFilterConfig_t, rpmFilterConfig);

void  rpmFilterInit(const rpmFilterConfig_t *config);
float rpmFilterGyro(int axis, float values);
void  rpmFilterUpdate();
