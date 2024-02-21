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

#include "pg/pg.h"

#include "flight/pid.h"

typedef enum {
    GM_OFF = 0,
    GM_PASSTHROUGH,
    GM_STANDARD,
    GM_MODE1,
    GM_MODE2,
} govMode_e;

typedef enum {
    GS_THROTTLE_OFF,
    GS_THROTTLE_IDLE,
    GS_SPOOLING_UP,
    GS_RECOVERY,
    GS_ACTIVE,
    GS_ZERO_THROTTLE,
    GS_LOST_HEADSPEED,
    GS_AUTOROTATION,
    GS_AUTOROTATION_BAILOUT,
} govState_e;

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint16_t gov_startup_time;
    uint16_t gov_spoolup_time;
    uint16_t gov_tracking_time;
    uint16_t gov_recovery_time;
    uint16_t gov_zero_throttle_timeout;
    uint16_t gov_lost_headspeed_timeout;
    uint16_t gov_autorotation_timeout;
    uint16_t gov_autorotation_bailout_time;
    uint16_t gov_autorotation_min_entry_time;
    uint8_t  gov_handover_throttle;
    uint8_t  gov_pwr_filter;
    uint8_t  gov_rpm_filter;
    uint8_t  gov_tta_filter;
    uint8_t  gov_ff_filter;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate();

uint8_t getGovernorState();

float getGovernorOutput(void);

float getFullHeadSpeedRatio(void);
float getSpoolUpRatio(void);

float getTTAIncrease(void);

bool isSpooledUp(void);

