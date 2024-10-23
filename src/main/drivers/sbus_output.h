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

// sbusOutChannel_t is the handle hold by the callers (e.g., servo, mixer, rx).
// It is now only a uint8_t (1-based sbus channel index) but can be extended in
// the future.
typedef uint8_t sbusOutChannel_t;

// Register a sbusOutChannel_t with a channel index (1-based).
void sbusOutConfig(sbusOutChannel_t *channel, uint8_t index);

// Set output of a sbusOutChannel_t.
// This should be a 11bit value (or 1bit for digital channels).
void sbusOutSetOutput(sbusOutChannel_t *channel, uint16_t value);

// Routine function called by the scheduler or timer
void sbusOutUpdate(timeUs_t currentTimeUs);

// Init function
void sbusOutInit(void);

// Convert a PWM us value [1000, 2000] to SBus value [192, 1792] (or [0, 1] for
// digital channels).
uint16_t sbusOutPwmToSbus(sbusOutChannel_t *channel, float pwm);
