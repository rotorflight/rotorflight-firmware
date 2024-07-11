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
    WIGGLE_OFF = 0,
    WIGGLE_READY,
    WIGGLE_ARMED,
    WIGGLE_ERROR,
    WIGGLE_FATAL,
};

typedef struct
{
    uint8_t   gyro_cal_on_first_arm;        // allow disarm/arm on throttle down + roll left/right
    uint8_t   auto_disarm_delay;            // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0

    uint8_t   power_on_arming_grace_time;   // in seconds

    uint8_t   enable_stick_arming;          // boolean that determines whether stick arming can be used
    uint8_t   enable_stick_commands;        // boolean that determines whether stick commands can be used

    uint8_t   wiggle_frequency;             // Swashplate indication frequency
    uint8_t   wiggle_strength;              // Swashplate indication amplitude
    uint32_t  wiggle_flags;                 // Wiggle enable flags

} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

