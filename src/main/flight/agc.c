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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/gyro.h"

#include "flight/pid.h"
#include "flight/agc.h"
#include "flight/mixer.h"

#ifdef USE_AUTOMATIC_GAIN_CONTROL__DISABLED__

static FAST_DATA_ZERO_INIT float updateRate;

static FAST_DATA_ZERO_INIT float ctrlGain[3];

static FAST_DATA_ZERO_INIT uint16_t ctrlDelay;

static FAST_DATA_ZERO_INIT biquadFilter_t inputLpFilter[4];
static FAST_DATA_ZERO_INIT biquadFilter_t inputHpFilter[4];

static FAST_DATA_ZERO_INIT biquadFilter_t outputLpFilter[4];
static FAST_DATA_ZERO_INIT biquadFilter_t outputHpFilter[4];


void agcUpdateControlGain(uint8_t axis)
{
    float ctrl = biquadFilterApply(&inputLpFilter[axis], mixerGetInputHistory(axis, ctrlDelay) / 2.0f);
    float rate = biquadFilterApply(&outputLpFilter[axis], gyro.gyroADCf[axis] / 360.0f);

    ctrl -= biquadFilterApply(&inputHpFilter[axis], ctrl);
    rate -= biquadFilterApply(&outputHpFilter[axis], rate);

    if (fabsf(ctrl) > 0.1f && fabsf(rate) > 0.1f) {
        float ratio = rate / ctrl;
        float cgain = fabsf(ctrl * rate);
        ctrlGain[axis] += (ratio - ctrlGain[axis]) * updateRate * cgain;
        DEBUG_AXIS(GYRO_AGC, axis, 2, ratio * 1000);
        DEBUG_AXIS(GYRO_AGC, axis, 3, cgain * 1000);
    }
    else {
        DEBUG_AXIS(GYRO_AGC, axis, 2, 0);
        DEBUG_AXIS(GYRO_AGC, axis, 3, 0);
    }

    DEBUG_AXIS(GYRO_AGC, axis, 0, ctrl * 1000);
    DEBUG_AXIS(GYRO_AGC, axis, 1, rate * 1000);
    DEBUG_AXIS(GYRO_AGC, axis, 4, ctrlGain[axis] * 1000);
}

void agcInit(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ctrlGain[axis] = 1.0f;
    }
}

void agcInitProfile(const pidProfile_t *pidProfile)
{
    ctrlDelay = constrain(lrintf((pidProfile->agc.delay / 1000.0f) / pidGetDT()), 0, 1000);

    updateRate = 0.1f / constrainf(pidProfile->agc.rate, 1, 50000);

    float hpf = constrainf(pidProfile->agc.hpf, 1, 1000) / 10.0f;
    float lpf = constrainf(pidProfile->agc.lpf, 1, 1000) / 10.0f;

    for (int axis = 0; axis < 4; axis++) {
        biquadFilterInitLPF(&inputHpFilter[axis], hpf, gyro.targetLooptime);
        biquadFilterInitLPF(&inputLpFilter[axis], lpf, gyro.targetLooptime);
        biquadFilterInitLPF(&outputHpFilter[axis], hpf, gyro.targetLooptime);
        biquadFilterInitLPF(&outputLpFilter[axis], lpf, gyro.targetLooptime);
    }
}

void agcUpdateGains(void)
{
    agcUpdateControlGain(FD_PITCH);
    agcUpdateControlGain(FD_ROLL);
    agcUpdateControlGain(FD_YAW);
}


#else

void agcInit(void)
{

}

void agcInitProfile(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);
}

void agcUpdateGains(void)
{

}

#endif
