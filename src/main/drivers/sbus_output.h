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

#include "platform.h"

#include "common/utils.h"
#include "common/time.h"

#define SBUS_OUT_CHANNELS   18

typedef enum {
    SBUS_OUT_SOURCE_RX = 0,
    SBUS_OUT_SOURCE_MIXER = 1,
    SBUS_OUT_SOURCE_SERVO = 2
} sbusOutSourceType_e;

typedef struct sbusOutConfigChannel_s{
    sbusOutSourceType_e sourceType:4;
    uint8_t sourceIndex:6;  // channel index, rule index or servo index.
    uint16_t min:11;  // 1000us maps to value x
    uint16_t max:11;  // 2000us maps to value y
} sbusOutConfigChannel_t;

// Define the sbus frame struct. Let's not reuse the one in the rx/ so we
// don't have to link them together in the unit test.
typedef struct {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    uint8_t endByte;
} __attribute__((__packed__)) sbusOutFrame_t;

STATIC_ASSERT(sizeof(sbusOutFrame_t) == 25,
              sbus_output_sbus_frame_size_mismatch);

// Routine function called by the scheduler or timer
void sbusOutUpdate(timeUs_t currentTimeUs);

// Init function
void sbusOutInit(void);