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
    GS_LOST_THROTTLE,
    GS_LOST_HEADSPEED,
    GS_AUTOROTATION,
    GS_AUTOROTATION_BAILOUT,
} govState_e;

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint16_t gov_max_headspeed;
    uint16_t gov_gear_ratio;
    uint16_t gov_spoolup_time;
    uint16_t gov_tracking_time;
    uint16_t gov_recovery_time;
    uint16_t gov_lost_throttle_timeout;
    uint16_t gov_lost_headspeed_timeout;
    uint16_t gov_autorotation_timeout;
    uint16_t gov_autorotation_bailout_time;
    uint16_t gov_autorotation_min_entry_time;
    uint16_t gov_pwr_filter;
    uint16_t gov_rpm_filter;
    uint16_t gov_gain;
    uint16_t gov_p_gain;
    uint16_t gov_i_gain;
    uint16_t gov_d_gain;
    uint16_t gov_f_gain;
    uint16_t gov_cyclic_ff_weight;
    uint16_t gov_collective_ff_weight;
    uint16_t gov_ff_exponent;
    uint16_t gov_vbat_offset;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


void governorInit();
void governorUpdate();
void governorUpdateGains(void);

uint8_t getGovernorState();

float getGovernorOutput(void);

float getHeadSpeed(void);
float getHeadSpeedRatio(void);

bool isSpooledUp(void);

