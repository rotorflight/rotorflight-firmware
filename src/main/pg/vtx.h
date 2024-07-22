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
#include "pg/modes.h"

typedef struct {
    uint8_t     band;           // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t     channel;        // 1-8
    uint8_t     power;          // 0 = lowest
    uint16_t    freq;           // sets freq in MHz if band=0
    uint16_t    pitModeFreq;    // sets out-of-range pitmode frequency
    uint8_t     lowPowerDisarm; // min power while disarmed, from vtxLowerPowerDisarm_e
    uint8_t     softserialAlt;  // pull line low before sending frame even with SOFTSERIAL
} vtxSettingsConfig_t;

PG_DECLARE(vtxSettingsConfig_t, vtxSettingsConfig);


typedef struct {
    uint8_t     auxChannelIndex;
    uint8_t     band;
    uint8_t     channel;
    uint8_t     power;
    channelRange_t range;
} vtxChannelActivationCondition_t;

#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT  10

typedef struct {
    vtxChannelActivationCondition_t vtxChannelActivationConditions[MAX_CHANNEL_ACTIVATION_CONDITION_COUNT];
    uint16_t serial_options;
} vtxConfig_t;

PG_DECLARE(vtxConfig_t, vtxConfig);

