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

#include "platform.h"

#include "common/time.h"

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
    ESC_SENSOR_PROTO_RECORD,
};

typedef struct escSensorConfig_s {
    uint8_t protocol;               // ESC telemetry protocol
    uint8_t halfDuplex;             // Set to false to listen on the TX pin for telemetry data
    uint16_t update_hz;             // Update frequency
    uint16_t current_offset;        // Offset (extra current) consumed by the VTX / cam (mA)
    uint16_t hw4_current_offset;    // HobbyWing V4 raw current offset
    uint8_t hw4_current_gain;       // HobbyWing V4 current gain
    uint8_t hw4_voltage_gain;       // HobbyWing V4 voltage gain
    uint8_t filter_cutoff;          // Frequency cutoff in Hz
} escSensorConfig_t;

PG_DECLARE(escSensorConfig_t, escSensorConfig);

typedef struct {
    uint8_t   age;              // Data age
    uint16_t  pwm;              // Output duty cycle 0.1%
    uint32_t  erpm;             // eRPM
    uint32_t  voltage;          // mV
    uint32_t  current;          // mA
    uint32_t  consumption;      // mAh
    int16_t   temperature;      // 0.1°C
    int16_t   temperature2;     // 0.1°C
    int16_t   temperature3;     // 0.1°C
    uint32_t  bec_voltage;      // mV
    uint32_t  bec_current;      // mA
} escSensorData_t;

#define ESC_DATA_INVALID 255

#define ESC_BATTERY_AGE_MAX 10

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

#define ESC_SENSOR_COMBINED 255

bool isEscSensorActive(void);

uint32_t getEscSensorRPM(uint8_t motorNumber);
escSensorData_t *getEscSensorData(uint8_t motorNumber);
