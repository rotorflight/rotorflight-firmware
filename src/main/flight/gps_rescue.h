/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"

#include "pg/gps_rescue.h"

#ifdef USE_MAG
#define GPS_RESCUE_USE_MAG              true
#else
#define GPS_RESCUE_USE_MAG              false
#endif

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY
} gpsRescueSanity_e;

typedef enum {
    MAX_ALT,
    FIXED_ALT,
    CURRENT_ALT
} altitudeMode_e;


extern int32_t gpsRescueAngle[ANGLE_INDEX_COUNT]; //NOTE: ANGLES ARE IN CENTIDEGREES

void updateGPSRescueState(void);
void rescueNewGpsData(void);

float gpsRescueGetYawRate(void);
float gpsRescueGetThrottle(void);
bool gpsRescueIsConfigured(void);
bool gpsRescueIsAvailable(void);
bool gpsRescueIsDisabled(void);
bool gpsRescueDisableMag(void);
