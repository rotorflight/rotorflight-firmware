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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_AUTO_ALIGN_IDLE = 0,
    BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT = 1,
    BOARD_AUTO_ALIGN_SUCCESS = 2,
    BOARD_AUTO_ALIGN_REJECTED_ARMED = 3,
    BOARD_AUTO_ALIGN_TIMEOUT = 4,
    BOARD_AUTO_ALIGN_NO_MATCH = 5,
    BOARD_AUTO_ALIGN_REJECTED_UNCALIBRATED = 6,
} boardAutoAlignState_e;

typedef struct boardAutoAlignStatus_s {
    uint8_t state;
    int16_t rollDegrees;
    int16_t pitchDegrees;
    int16_t yawDegrees;
    uint8_t matchedSamples;
} boardAutoAlignStatus_t;

void boardAutoAlignProcessSample(const float *sensorVector);
bool boardAutoAlignStart(void);
boardAutoAlignStatus_t boardAutoAlignGetStatus(void);
