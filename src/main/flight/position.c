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

    uint8_t     source;

    float       altitude;
    float       variometer;

    bool        haveBaroAlt;
    bool        haveGpsAlt;

    float       baroAlt;
    float       baroAltOffset;

    float       gpsAlt;
    float       gpsAltOffset;

    difFilter_t varioFilter;

    filter_t    gpsFilter;
    filter_t    baroFilter;

    filter_t    gpsOffsetFilter;
    filter_t    baroOffsetFilter;

} altState_t;

static FAST_DATA altState_t alt;


float getAltitude(void)
{
    return alt.altitude;
}

float getVario(void)
{
    return alt.variometer;
}

int getEstimatedAltitudeCm(void)
{
    return lrintf(alt.altitude * 100);
}

int getEstimatedVarioCms(void)
{
    return lrintf(alt.variometer * 100);
}


static float calculateVario(float altitude)
{
    return difFilterApply(&alt.varioFilter, altitude);
}

void positionUpdate(void)
{
#ifdef USE_BARO
    if (alt.source == ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_BARO_ONLY) {
        if (sensors(SENSOR_BARO) && baroIsReady()) {
            if (baro.baroAltitude < 15000 && baro.baroAltitude > -2500) {
                alt.baroAlt = filterApply(&alt.baroFilter, baro.baroAltitude / 100.0f);
                alt.haveBaroAlt = true;
            }
        }
        else {
            alt.haveBaroAlt = false;
        }
    }
#endif

#ifdef USE_GPS
    if (alt.source & ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_GPS_ONLY) {
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= positionConfig()->gps_min_sats) {
            alt.gpsAlt = filterApply(&alt.gpsFilter, gpsSol.llh.altCm / 100.0f);
            alt.haveGpsAlt = true;
        }
        else {
            alt.haveGpsAlt = false;
        }
    }
#endif

    if (!ARMING_FLAG(ARMED)) {
        if (alt.haveBaroAlt) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter, alt.baroAlt);
        }
        if (alt.haveGpsAlt) {
            alt.gpsAltOffset = filterApply(&alt.gpsOffsetFilter, alt.gpsAlt);
        }
    }
    else {
        if (alt.haveBaroAlt && alt.haveGpsAlt && alt.gpsAltOffset) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter,
                alt.baroAlt - (alt.gpsAlt - alt.gpsAltOffset));
        }
    }

    if (alt.haveBaroAlt && alt.baroAltOffset) {
        alt.altitude = alt.baroAlt - alt.baroAltOffset;
        alt.variometer = calculateVario(alt.baroAlt);
    }
    else if (alt.haveGpsAlt && alt.gpsAltOffset) {
        alt.altitude = alt.gpsAlt - alt.gpsAltOffset;
        alt.variometer = calculateVario(alt.gpsAlt);
    }
    else {
        alt.altitude = 0;
        alt.variometer = 0;
    }

    DEBUG(ALTITUDE, 0, alt.altitude * 100);
    DEBUG(ALTITUDE, 1, alt.variometer * 100);
    DEBUG(ALTITUDE, 2, alt.baroAlt * 100);
    DEBUG(ALTITUDE, 3, alt.baroAltOffset * 100);
    DEBUG(ALTITUDE, 4, alt.gpsAlt * 100);
    DEBUG(ALTITUDE, 5, alt.gpsAltOffset * 100);
#ifdef USE_GPS
    DEBUG(ALTITUDE, 6, gpsSol.llh.altCm);
    DEBUG(ALTITUDE, 7, gpsSol.numSat);
#endif
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    difFilterInit(&alt.varioFilter, positionConfig()->vario_lpf / 100.0f, pidGetPidFrequency());

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
}
