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

#define MAX_NAME_LENGTH 16U

typedef struct {
    char    name[MAX_NAME_LENGTH + 1];
    char    displayName[MAX_NAME_LENGTH + 1];
    uint8_t modelId;
    uint8_t modelParam1Type;
    int16_t modelParam1Value;
    uint8_t modelParam2Type;
    int16_t modelParam2Value;
    uint8_t modelParam3Type;
    int16_t modelParam3Value;
} pilotConfig_t;

PG_DECLARE(pilotConfig_t, pilotConfig);
