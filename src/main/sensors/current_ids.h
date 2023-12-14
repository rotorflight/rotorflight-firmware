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

typedef enum {
    CURRENT_METER_ID_NONE = 0,

    CURRENT_METER_ID_BATTERY = 10,          // 10-19 for battery meters
    CURRENT_METER_ID_BEC = 20,              // 20-29 for BEC / servo bus meters
    CURRENT_METER_ID_BUS = 30,              // 30-39 for BUS / 5V bus meters
    CURRENT_METER_ID_EXT = 40,              // 40-49 for EXT meters
    CURRENT_METER_ID_MCU = 50,              // 50-59 for MCU / 3.3V meters
    CURRENT_METER_ID_ESC_COMBINED = 100,    // 100 for ESC combined
    CURRENT_METER_ID_ESC_1,                 // 101- for ESC currents
    CURRENT_METER_ID_ESC_2,
    CURRENT_METER_ID_ESC_3,
    CURRENT_METER_ID_ESC_4,

} currentMeterId_e;
