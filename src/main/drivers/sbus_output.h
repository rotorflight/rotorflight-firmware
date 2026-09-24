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

#include "common/time.h"
#include "common/utils.h"
#include "pg/sbus_output.h"

// Define the sbus frame struct
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

bool sbusOutIsEnabled(void);

// Init function
void sbusOutInit(void);

// Speed-limit state for one bus output (SBUS or F.Bus). Each output keeps its
// own, so the limit steps once per that output's frame, by the real time
// since its previous frame.
typedef struct {
    float pos[SBUS_OUT_CHANNELS];   // last speed-limited position per channel
    float cyclicRatio;              // swashplate ratio limit for this frame
    float dt;                       // seconds since this output's previous frame
    timeUs_t lastFrameUs;
} sbusOutSpeedState_t;

// Start an output frame: works out dt and the cyclic ratio for this output.
// frameRateHz is the output's configured rate, used for the first frame.
void sbusOutBeginFrame(sbusOutSpeedState_t *state, timeUs_t currentTimeUs, float frameRateHz);

// Channel value getters
float sbusOutGetRX(uint8_t channel);
float sbusOutGetValueMixer(uint8_t channel, sbusOutSpeedState_t *state);

// Process all mixer channels (called internally by sbusOutUpdate, but can be called externally)
void sbusOutProcessMixerChannels(float output[SBUS_OUT_CHANNELS], sbusOutSpeedState_t *state);
