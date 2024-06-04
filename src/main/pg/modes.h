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
#include "pg/pg_ids.h"


typedef struct {
    int8_t startStep;
    int8_t endStep;
} channelRange_t;

typedef struct {
    uint8_t modeId;
    uint8_t linkedTo;
    uint8_t modeLogic;
    uint8_t auxChannelIndex;
    channelRange_t range;
} modeActivationCondition_t;

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 20

PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);


#if defined(USE_CUSTOM_BOX_NAMES)

#define MAX_BOX_USER_NAME_LENGTH 16

typedef struct {
    char box_user_1_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_2_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_3_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_4_name[MAX_BOX_USER_NAME_LENGTH];
} modeActivationConfig_t;

PG_DECLARE(modeActivationConfig_t, modeActivationConfig);

#endif /* USE_CUSTOM_BOX_NAMES */
