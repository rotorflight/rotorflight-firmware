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
#define SP_SMOOTHING_FILTER_MAX_HZ           500


typedef struct
{
    float setpoint[4];
    float limitedSp[4];

    float accelLimit[4];
    float ringLimit[2];

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

void setpointFilterUpdate(float frameTimeUs)
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
    const float cyclicLimit = 1.4142135623f - currentControlRateProfile->cyclic_ring * 0.004142135623f;

    for (int i = 0; i < 4; i++) {
        spFilter.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
    }
    for (int i = 0; i < 2; i++) {
        spFilter.ringLimit[i] = 1.0f / (currentControlRateProfile->rate_limit[i] * cyclicLimit);
    }

    spFilter.smoothCutoff = 1000.0f / constrain(currentControlRateProfile->rates_smoothness, 1, 250);
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


void setpointUpdate(void)
{
    for (int axis = 0; axis < 4; axis++) {
        spFilter.setpoint[axis] = getRawSetpoint(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, spFilter.setpoint[axis]);
    }

    const float R = spFilter.setpoint[FD_ROLL]  * spFilter.ringLimit[FD_ROLL];
    const float P = spFilter.setpoint[FD_PITCH] * spFilter.ringLimit[FD_PITCH];
    const float C = sqrtf(sq(R) + sq(P));

    if (C > 1.0f) {
        spFilter.setpoint[FD_ROLL]  /= C;
        spFilter.setpoint[FD_PITCH] /= C;
    }

    for (int axis = 0; axis < 4; axis++) {
        spFilter.setpoint[axis] = spFilter.limitedSp[axis] = slewLimit(spFilter.limitedSp[axis], spFilter.setpoint[axis], spFilter.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 1, spFilter.setpoint[axis]);

        spFilter.setpoint[axis] = pt3FilterApply(&spFilter.filter[axis], spFilter.setpoint[axis]);
        DEBUG_AXIS(SETPOINT, axis, 2, spFilter.setpoint[axis]);
    }
}

