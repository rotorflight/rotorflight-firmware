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

#include "pg/esc_sensor.h"

typedef struct {
    uint8_t   age;              // Data age
    uint16_t  pwm;              // Output duty cycle 0.1%
    uint16_t  throttle;         // Input setpoint 0.1%
    uint32_t  erpm;             // eRPM
    uint32_t  voltage;          // mV
    uint32_t  current;          // mA
    uint32_t  consumption;      // mAh
    int16_t   temperature;      // 0.1°C
    int16_t   temperature2;     // 0.1°C
    uint32_t  bec_voltage;      // mV
    uint32_t  bec_current;      // mA
    uint32_t  status;           // status / fault codes
    uint8_t   id;               // ESC id / flags
} escSensorData_t;

#define ESC_DATA_INVALID 255

#define ESC_BATTERY_AGE_MAX 10

#define ESC_SENSOR_COMBINED 255

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

void validateAndFixEscSensorConfig(void);

bool isEscSensorActive(void);

uint32_t getEscSensorRPM(uint8_t motorNumber);
escSensorData_t *getEscSensorData(uint8_t motorNumber);

/* Inject ESC sensor data for a specific motor (used by SMART ESC receiver driver). */
void escSensorInject(uint8_t motorNumber, const escSensorData_t *data);

uint8_t escGetParamBufferLength(void);
uint8_t *escGetParamBuffer(void);
uint8_t *escGetParamUpdBuffer(void);
bool escCommitParameters(void);
