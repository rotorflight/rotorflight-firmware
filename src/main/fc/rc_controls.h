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

#include <stdbool.h>

#include "common/filter.h"
#include "pg/pg.h"

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    COLLECTIVE,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

#define PRIMARY_CHANNEL_COUNT 5

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define ROL_LO    (1 << 0)
#define ROL_HI    (2 << 0)
#define ROL_CE    (3 << 0)
#define PIT_LO    (1 << 2)
#define PIT_HI    (2 << 2)
#define PIT_CE    (3 << 2)
#define YAW_LO    (1 << 4)
#define YAW_HI    (2 << 4)
#define YAW_CE    (3 << 4)
#define THR_LO    (1 << 6)
#define THR_HI    (2 << 6)
#define THR_CE    (3 << 6)
#define THR_MASK  (3 << 6)


extern float rcCommand[5];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // allow disarm/arm on throttle down + roll left/right
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions();

bool isUsingSticksForArming(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
void rcControlsInit(void);
