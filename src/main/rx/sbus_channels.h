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

#include <stdint.h>

#define SBUS_MAX_CHANNEL 18

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

typedef struct sbusChannels_s {
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
} __attribute__((__packed__)) sbusChannels_t;

#define SBUS_CHANNEL_DATA_LENGTH sizeof(sbusChannels_t)

uint8_t sbusChannelsDecode(rxRuntimeState_t *rxRuntimeState, const sbusChannels_t *channels);

void sbusChannelsInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);

// FrSky variants of sbus frame: 8ch frame and 24ch frame
typedef struct sbusChannels8ch_s {
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbusChannels8ch_t;

STATIC_ASSERT(sizeof(sbusChannels8ch_t) == 12, sizeof(subsChannels8ch_t) incorrect);

uint8_t sbusChannelsDecode8ch(rxRuntimeState_t *rxRuntimeState, const sbusChannels8ch_t *channels);

typedef struct sbusChannels24ch_s {
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
    unsigned int chan16 : 11;
    unsigned int chan17 : 11;
    unsigned int chan18 : 11;
    unsigned int chan19 : 11;
    unsigned int chan20 : 11;
    unsigned int chan21 : 11;
    unsigned int chan22 : 11;
    unsigned int chan23 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbusChannels24ch_t;

STATIC_ASSERT(sizeof(sbusChannels24ch_t) == 34, sizeof(subsChannels24ch_t) incorrect);

uint8_t sbusChannelsDecode24ch(rxRuntimeState_t *rxRuntimeState, const sbusChannels24ch_t *channels);