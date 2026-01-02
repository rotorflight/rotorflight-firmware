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
#include "rx/rx.h"
#include "rx/sbus_channels.h"

#include "common/time.h"
#include "common/utils.h"
#include "drivers/fbus_sensor.h"


#define FMUS_MASTER_CONTROL_EXTRA_FIELDS 4
#define FBUS_MASTER_NUM_CHANNELS 16
#define SBUS_BITS_PER_CHANNEL 11
#define FBUS_MASTER_CONTROL_FRAME_SIZE (FBUS_MASTER_NUM_CHANNELS * SBUS_BITS_PER_CHANNEL / 8 + 1 + FMUS_MASTER_CONTROL_EXTRA_FIELDS)
#define FBUS_MASTER_CONTROL_FRAME_PAYLOAD_SIZE (FBUS_MASTER_CONTROL_FRAME_SIZE - FMUS_MASTER_CONTROL_EXTRA_FIELDS + 2)
#define FBUS_CONTROL16_LENGTH 0x18
#define FBUS_CONTROL16_TYPE 0xFF

#define FBUS_DOWNLINK_PAYLOAD_SIZE 0x08
#define FBUS_DOWNLINK_LENGTH 0x0A


typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl16_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels8ch_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl8_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels24ch_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl24_t;

typedef struct {
    uint8_t length;
    uint8_t phyID;
    uint8_t prim;
    uint16_t appId ;
    uint8_t data[4];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterDownlink_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[24];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaStart_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[32];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaData_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[24];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaEnd_t;

typedef struct {
    fbusMasterControl16_t c16;
    fbusMasterDownlink_t downlink;
} __attribute__((__packed__)) fbusMasterFrame_t;

STATIC_ASSERT(sizeof(fbusMasterControl16_t) == FBUS_MASTER_CONTROL_FRAME_SIZE,
              fbus_master_frame_size_mismatch);

// Routine function called by the scheduler or timer
void fbusMasterUpdate(timeUs_t currentTimeUs);

bool fbusMasterIsEnabled();

// Init function
void fbusMasterInit(void);
