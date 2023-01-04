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
    float deflection[4];
    float setpoint[4];
    float limited[4];

    float accelLimit[4];
    float ringLimit;

    pt3Filter_t filter[4];

    uint16_t smoothCutoff;
    uint16_t activeCutoff;

} setpointData_t;

static FAST_DATA_ZERO_INIT setpointData_t sp;


float getSetpoint(int axis)
{
    return sp.setpoint[axis];
}

float getDeflection(int axis)
{
    return sp.deflection[axis];
}

uint16_t setpointFilterGetCutoffFreq(void)
{
    return sp.activeCutoff;
}


static float setpointAutoSmoothingCutoff(float frameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        float factor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        cutoff = factor * (1.0f / (frameTimeUs * 1e-6f));
    }

    return cutoff;
}

void setpointUpdateTiming(float frameTimeUs)
{
    float cutoff = setpointAutoSmoothingCutoff(frameTimeUs, rxConfig()->rx_smoothness);

    DEBUG(SETPOINT, 5, cutoff);

    cutoff = MIN(sp.smoothCutoff, cutoff);
    cutoff = constrain(cutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);

    DEBUG(SETPOINT, 6, cutoff);

    if (sp.activeCutoff != cutoff) {
        const float gain = pt3FilterGain(cutoff, pidGetDT());
        for (int i = 0; i < 4; i++) {
            pt3FilterUpdateCutoff(&sp.filter[i], gain);
        }
        sp.activeCutoff = cutoff;
    }

    DEBUG(SETPOINT, 7, frameTimeUs);
}

INIT_CODE void setpointInitProfile(void)
{
    sp.ringLimit = 1.0f / (1.4142135623f - currentControlRateProfile->cyclic_ring * 0.004142135623f);

    for (int i = 0; i < 4; i++) {
        sp.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
    }

    sp.smoothCutoff = 1000.0f / constrain(currentControlRateProfile->rates_smoothness, 1, 250);
    sp.activeCutoff = constrain(sp.smoothCutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
}

INIT_CODE void setpointInit(void)
{
    setpointInitProfile();

    const float gain = pt3FilterGain(sp.activeCutoff, pidGetDT());
    for (int i = 0; i < 4; i++) {
        pt3FilterInit(&sp.filter[i], gain);
    }
}

void setpointUpdate(void)
{
    for (int axis = 0; axis < 4; axis++) {
        sp.deflection[axis] = getRcDeflection(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, sp.deflection[axis] * 1000);
    }

    const float R = sp.deflection[FD_ROLL]  * sp.ringLimit;
    const float P = sp.deflection[FD_PITCH] * sp.ringLimit;
    const float C = sqrtf(sq(R) + sq(P));

    if (C > 1.0f) {
        sp.deflection[FD_ROLL]  /= C;
        sp.deflection[FD_PITCH] /= C;
    }

    DEBUG_AXIS(SETPOINT, FD_ROLL, 1, sp.deflection[FD_ROLL] * 1000);
    DEBUG_AXIS(SETPOINT, FD_PITCH, 1, sp.deflection[FD_PITCH] * 1000);

    for (int axis = 0; axis < 4; axis++) {
        float SP = sp.limited[axis] = slewLimit(sp.limited[axis], sp.deflection[axis], sp.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 2, SP * 1000);

        SP = sp.deflection[axis] = pt3FilterApply(&sp.filter[axis], SP);
        DEBUG_AXIS(SETPOINT, axis, 3, SP * 1000);

        SP = sp.setpoint[axis] = applyRatesCurve(axis, SP);
        DEBUG_AXIS(SETPOINT, axis, 4, SP);
    }
}
