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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"

#include "fc/rc.h"

#include "setpoint.h"


#define SP_SMOOTHING_FILTER_MIN_HZ             5
#define SP_SMOOTHING_FILTER_MAX_HZ           250


typedef struct
{
    float accelLimit[4];
    float limitedSp[4];
    float setpoint[4];

    pt3Filter_t filter[4];

    uint16_t smoothCutoff;
    uint16_t activeCutoff;

} setpointFilter_t;

static FAST_DATA_ZERO_INIT setpointFilter_t spFilter;


float getSetpoint(int axis)
{
    return spFilter.setpoint[axis];
}

uint16_t setpointFilterGetCutoffFreq(void)
{
    return spFilter.activeCutoff;
}


static inline float setpointAutoSmoothingCutoff(float frameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        float factor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        cutoff = factor * (1.0f / (frameTimeUs * 1e-6f));
    }

    return cutoff;
}

FAST_CODE void setpointFilterUpdate(float frameTimeUs)
{
    float cutoff = setpointAutoSmoothingCutoff(frameTimeUs, rxConfig()->rx_smoothness);

    DEBUG(SETPOINT, 4, cutoff);

    cutoff = MIN(spFilter.smoothCutoff, cutoff);
    cutoff = constrain(cutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);

    DEBUG(SETPOINT, 5, cutoff);

    if (spFilter.activeCutoff != cutoff) {
        const float gain = pt3FilterGain(cutoff, pidGetDT());
        for (int i = 0; i < 4; i++) {
            pt3FilterUpdateCutoff(&spFilter.filter[i], gain);
        }
        spFilter.activeCutoff = cutoff;
    }
}

INIT_CODE void setpointFilterInitProfile(void)
{
    for (int i = 0; i < 4; i++) {
        spFilter.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
    }

    spFilter.smoothCutoff  = 1000.0f / constrain(currentControlRateProfile->rates_smoothness, 1, 250);
    spFilter.activeCutoff = constrain(spFilter.smoothCutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
}

INIT_CODE void setpointFilterInit(void)
{
    setpointFilterInitProfile();

    const float gain = pt3FilterGain(spFilter.activeCutoff, pidGetDT());
    for (int i = 0; i < 4; i++) {
        pt3FilterInit(&spFilter.filter[i], gain);
    }
}


FAST_CODE void setpointUpdate(void)
{
    for (int axis = 0; axis < 4; axis++) {
        float setpoint = getRawSetpoint(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, setpoint);

        setpoint = spFilter.limitedSp[axis] = slewLimit(spFilter.limitedSp[axis], setpoint, spFilter.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 1, setpoint);

        setpoint = pt3FilterApply(&spFilter.filter[axis], setpoint);
        DEBUG_AXIS(SETPOINT, axis, 2, setpoint);

        spFilter.setpoint[axis] = setpoint;
    }
}

