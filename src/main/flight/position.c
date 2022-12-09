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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/position.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"


typedef struct {

    bool        wasARMED;

    bool        haveBaroAlt;
    bool        haveBaroOffset;

    float       baroAlt;
    float       baroAltOffset;

    bool        haveGpsAlt;
    bool        haveGpsOffset;

    float       gpsAlt;
    float       gpsAltOffset;

    float       estimatedAltitude;
    float       estimatedVario;

    float       varioAltPrev;

    pt2Filter_t gpsFilter;
    pt2Filter_t baroFilter;
    pt2Filter_t varioFilter;

    pt3Filter_t gpsOffsetFilter;
    pt3Filter_t baroOffsetFilter;

    pt2Filter_t baroDriftFilter;

} altState_t;

static FAST_DATA altState_t alt;


bool hasAltitudeOffset(void)
{
    return alt.haveBaroOffset || alt.haveGpsOffset;
}

float getAltitude(void)
{
    return alt.estimatedAltitude;
}

float getVario(void)
{
    return alt.estimatedVario;
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(alt.estimatedAltitude);
}

int16_t getEstimatedVario(void)
{
    return lrintf(alt.estimatedVario);
}


static float calculateVario(float altitude)
{
    float vario = (altitude - alt.varioAltPrev) * pidGetPidFrequency();
    vario = pt2FilterApply(&alt.varioFilter, vario);

    alt.varioAltPrev = altitude;

    return vario;
}

static void calculateAltitude(void)
{
    float gpsGround = 0;
    float baroGround = 0;
    float baroDrift = 0;

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (baroIsReady()) {
            alt.baroAlt = pt2FilterApply(&alt.baroFilter, baro.baroAltitude);
            baroGround = pt3FilterApply(&alt.baroOffsetFilter, alt.baroAlt);
            alt.haveBaroAlt = true;
        }
        else {
            alt.haveBaroAlt = false;
        }
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        if (STATE(GPS_FIX) && gpsSol.numSat >= positionConfig()->gps_min_sats) {
            alt.gpsAlt = pt2FilterApply(&alt.gpsFilter, gpsSol.llh.altCm);
            gpsGround = pt3FilterApply(&alt.gpsOffsetFilter, alt.gpsAlt);
            alt.haveGpsAlt = true;
        }
        else {
            alt.haveGpsAlt = false;
        }
    }
#endif

    if (ARMING_FLAG(ARMED)) {
        if (!alt.wasARMED) {
            if (alt.wasARMED) {
                alt.haveBaroOffset = true;
                alt.baroAltOffset = baroGround;
            }
            if (alt.haveGpsAlt) {
                alt.haveGpsOffset = true;
                alt.gpsAltOffset = gpsGround;
            }
            alt.wasARMED = true;
        }
    }
    else {
        if (alt.wasARMED) {
            alt.haveBaroOffset = false;
            alt.haveGpsOffset = false;
            alt.wasARMED = false;
        }
    }

    if (alt.haveBaroAlt && alt.haveBaroOffset)
        alt.baroAlt -= alt.baroAltOffset;

    if (alt.haveGpsAlt && alt.haveGpsOffset)
        alt.gpsAlt -= alt.gpsAltOffset;

    if (alt.haveBaroAlt) {
        alt.estimatedAltitude = alt.baroAlt;
        alt.estimatedVario = calculateVario(alt.baroAlt);
        if (alt.haveBaroOffset && alt.haveGpsAlt && alt.haveGpsOffset) {
            baroDrift = pt2FilterApply(&alt.baroDriftFilter, alt.baroAlt - alt.gpsAlt);
            alt.estimatedAltitude -= baroDrift;
        }
    }
    else if (alt.haveGpsAlt) {
        alt.estimatedAltitude = alt.gpsAlt;
        alt.estimatedVario = calculateVario(alt.gpsAlt);
    }
    else {
        alt.estimatedAltitude = 0;
        alt.estimatedVario = 0;
    }

    DEBUG(ALTITUDE, 0, alt.estimatedAltitude);
    DEBUG(ALTITUDE, 1, alt.estimatedVario);
    DEBUG(ALTITUDE, 2, alt.baroAlt);
    DEBUG(ALTITUDE, 3, baroGround);
    DEBUG(ALTITUDE, 4, baroDrift);
    DEBUG(ALTITUDE, 5, alt.gpsAlt);
    DEBUG(ALTITUDE, 6, gpsGround);
}

void positionUpdate(void)
{
    calculateAltitude();
}

void positionInit(void)
{
    pt2FilterInit(&alt.gpsFilter, pt2FilterGain(positionConfig()->gps_alt_lpf / 100.0f, pidGetDT()));
    pt2FilterInit(&alt.baroFilter, pt2FilterGain(positionConfig()->baro_alt_lpf / 100.0f, pidGetDT()));
    pt2FilterInit(&alt.varioFilter, pt2FilterGain(positionConfig()->vario_lpf / 100.0f, pidGetDT()));

    pt3FilterInit(&alt.gpsOffsetFilter, pt3FilterGain(positionConfig()->gps_offset_lpf / 1000.0f, pidGetDT()));
    pt3FilterInit(&alt.baroOffsetFilter, pt3FilterGain(positionConfig()->baro_offset_lpf / 1000.0f, pidGetDT()));

    pt2FilterInit(&alt.baroDriftFilter, pt2FilterGain(positionConfig()->baro_drift_lpf / 1000.0f, pidGetDT()));
}
