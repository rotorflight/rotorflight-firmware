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
    ESC_SENSOR_PROTO_KISS,
    ESC_SENSOR_PROTO_HW4,
    ESC_SENSOR_PROTO_KONTRONIK,
};

typedef struct escSensorConfig_s {
    uint8_t protocol;               // ESC telemetry protocol
    uint8_t halfDuplex;             // Set to false to listen on the TX pin for telemetry data
    uint16_t update_hz;             // Update frequency
    uint16_t current_offset;        // Offset (extra current) consumed by the VTX / cam (mA)
    uint16_t hw4_current_offset;    // HobbyWing V4 raw current offset
    uint8_t hw4_current_gain;       // HobbyWing V4 current gain
    uint8_t hw4_voltage_gain;       // HobbyWing V4 voltage gain
} escSensorConfig_t;

PG_DECLARE(escSensorConfig_t, escSensorConfig);

typedef struct {
    uint8_t dataAge;
    int8_t temperature;  // C degrees
    int16_t voltage;     // x0.01V
    int32_t current;     // x0.01A
    int32_t consumption; // mAh
    int16_t rpm;         // x100erpm
} escSensorData_t;

#define ESC_DATA_INVALID 255

#define ESC_BATTERY_AGE_MAX 10

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

#define ESC_SENSOR_COMBINED 255

bool isEscSensorActive(void);

uint16_t getEscSensorRPM(uint8_t motorNumber);
escSensorData_t *getEscSensorData(uint8_t motorNumber);

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength);
uint8_t getNumberEscBytesRead(void);

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);

