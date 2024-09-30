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

#ifndef ESC_SENSOR_TASK_FREQ_HZ
#define ESC_SENSOR_TASK_FREQ_HZ 50
#endif

enum {
    ESC_SENSOR_PROTO_NONE = 0,
    ESC_SENSOR_PROTO_BLHELI32,
    ESC_SENSOR_PROTO_HW4,
    ESC_SENSOR_PROTO_HW5,
    ESC_SENSOR_PROTO_SCORPION,
    ESC_SENSOR_PROTO_KONTRONIK,
    ESC_SENSOR_PROTO_OMPHOBBY,
    ESC_SENSOR_PROTO_ZTW,
    ESC_SENSOR_PROTO_APD,
    ESC_SENSOR_PROTO_OPENYGE,
    ESC_SENSOR_PROTO_FLY,
    ESC_SENSOR_PROTO_GRAUPNER,
    ESC_SENSOR_PROTO_RECORD,
};

typedef struct {
    uint8_t     protocol;               // ESC telemetry protocol
    uint8_t     halfDuplex;             // Set to false to listen on the TX pin for telemetry data
    uint8_t     pinSwap;                // Swap rx and tx pins around compared to the resource settings
    uint16_t    update_hz;              // Update frequency
    uint16_t    current_offset;         // Offset (extra current) consumed by the VTX / cam (mA)
    uint16_t    hw4_current_offset;     // HobbyWing V4 raw current offset
    uint8_t     hw4_current_gain;       // HobbyWing V4 current gain
    uint8_t     hw4_voltage_gain;       // HobbyWing V4 voltage gain
    uint8_t     filter_cutoff;          // Frequency cutoff in Hz
} escSensorConfig_t;

PG_DECLARE(escSensorConfig_t, escSensorConfig);
