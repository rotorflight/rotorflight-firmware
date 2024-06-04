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

#include "types.h"
#include "platform.h"

#include "pg/pg_ids.h"
#include "pg/gps_rescue.h"

#include "flight/gps_rescue.h"


PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 1);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .angle = 32,
    .initialAltitudeM = 50,
    .descentDistanceM = 200,
    .rescueGroundspeed = 2000,
    .throttleP = 150,
    .throttleI = 20,
    .throttleD = 50,
    .velP = 80,
    .velI = 20,
    .velD = 15,
    .yawP = 40,
    .throttleMin = 1100,
    .throttleMax = 1600,
    .throttleHover = 1280,
    .sanityChecks = RESCUE_SANITY_ON,
    .minSats = 8,
    .minRescueDth = 100,
    .allowArmingWithoutFix = false,
    .useMag = GPS_RESCUE_USE_MAG,
    .targetLandingAltitudeM = 5,
    .targetLandingDistanceM = 10,
    .altitudeMode = MAX_ALT,
    .ascendRate = 500,
    .descendRate = 150,
    .rescueAltitudeBufferM = 15,
);

