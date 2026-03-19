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
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/governor.h"
#include "flight/pid.h"
#include "flight/ff_estimate.h"

#include "pg/mixer.h"
#include "pg/pid.h"


typedef struct {
    bool            active;
    bool            was_active;
    float           min_setpoint;
    float           convergence_ratio;
    float           accum_rate;
    float           min_kf[2];
    float           max_kf[2];
    float           base_kf[2];
    float           kf_correction[2];
    pt1Filter_t     ff_error_filter[2];
} ffEstimateState_t;

static FAST_DATA_ZERO_INIT ffEstimateState_t ffest;


void INIT_CODE ffEstimateInit(void)
{
    ffest.active = false;
    ffest.was_active = false;
    ffest.kf_correction[0] = 0.0f;
    ffest.kf_correction[1] = 0.0f;

    const uint8_t gain = mixerConfig()->ff_estimate_gain;

    if (gain == 0 || mixerConfig()->ff_estimate_min_f > mixerConfig()->ff_estimate_max_f) {
        return;
    }

    ffest.min_setpoint = mixerConfig()->ff_estimate_min_setpoint;
    ffest.convergence_ratio = mixerConfig()->ff_estimate_convergence / 100.0f;
    ffest.accum_rate = gain / 100.0f;
    
    const float f_term_scale[2] = { ROLL_F_TERM_SCALE, PITCH_F_TERM_SCALE };

    for (int i = 0; i < 2; i++) {
        ffest.min_kf[i] = mixerConfig()->ff_estimate_min_f * f_term_scale[i];
        ffest.max_kf[i] = mixerConfig()->ff_estimate_max_f * f_term_scale[i];
    }

    const float cutoff_freq = 0.5f;
    const float pid_freq = pidGetPidFrequency();

    for (int i = 0; i < 2; i++) {
        pt1FilterInit(&ffest.ff_error_filter[i], cutoff_freq, pid_freq);
    }
}

static bool ffEstimateCheckAxisPreconditions(int axis)
{
    const pidAxisData_t *pid_data = pidGetAxisData();
    const float setpoint = pid_data[axis].setPoint;
    const float gyroRate = pid_data[axis].gyroRate;

    if (fabsf(setpoint) < ffest.min_setpoint)
        return false;

    const float errorRate = fabsf(gyroRate - setpoint);

    if (errorRate > ffest.convergence_ratio * fabsf(setpoint))
        return false;

    return true;
}

static bool ffEstimateCheckPreconditions(void)
{
    if (!ARMING_FLAG(ARMED))
        return false;

    if (!isSpooledUp())
        return false;

    if (!isAirborne())
        return false;

    if (!IS_RC_MODE_ACTIVE(BOXFFESTIMATE))
        return false;

    if (FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE | ANGLE_MODE | HORIZON_MODE))
        return false;

    for (int i = 0; i < 2; i++) {
        const uint16_t f = currentPidProfile->pid[i].F;
        
        if (f < mixerConfig()->ff_estimate_min_f || f > mixerConfig()->ff_estimate_max_f)
            return false;
    }

    return true;
}

void ffEstimateUpdate(void)
{
    if (mixerConfig()->ff_estimate_gain == 0) {
        return;
    }

    float filtered_ff_error[2] = { 0.0f, 0.0f };

    const bool mode_active = ffEstimateCheckPreconditions();

    ffest.active = mode_active;

    if (ffest.active && !ffest.was_active) {
        const float f_term_scale[2] = { ROLL_F_TERM_SCALE, PITCH_F_TERM_SCALE };

        for (int i = 0; i < 2; i++) {
            ffest.ff_error_filter[i].y1 = 0.0f;
            ffest.base_kf[i] = f_term_scale[i] * currentPidProfile->pid[i].F;
            ffest.kf_correction[i] = 0.0f;
        }
    }

    if (ffest.active) {
        const float dT = pidGetDT();
        const pidAxisData_t *pid_data = pidGetAxisData();

        for (int i = 0; i < 2; i++) {
            if (ffEstimateCheckAxisPreconditions(i)) {
                const float setpoint = pid_data[i].setPoint;
                const float ff_error = pid_data[i].I / setpoint;

                filtered_ff_error[i] = pt1FilterApply(&ffest.ff_error_filter[i], ff_error);
                ffest.kf_correction[i] += filtered_ff_error[i] * ffest.accum_rate * dT;

                const float total_kf = ffest.base_kf[i] + ffest.kf_correction[i];
                const float clamped_kf = constrainf(total_kf, ffest.min_kf[i], ffest.max_kf[i]);
                ffest.kf_correction[i] = clamped_kf - ffest.base_kf[i];
            }
        }
    }

    if (ffest.was_active && !ffest.active) {
        if (ARMING_FLAG(ARMED)) {
            const float f_term_scale[2] = { ROLL_F_TERM_SCALE, PITCH_F_TERM_SCALE };

            for (int i = 0; i < 2; i++) {
                const int f_correction = lrintf(ffest.kf_correction[i] / f_term_scale[i]);
                const uint16_t new_f = constrain(
                    (int)currentPidProfile->pid[i].F + f_correction,
                    mixerConfig()->ff_estimate_min_f,
                    mixerConfig()->ff_estimate_max_f);

                if (i == 0)
                    set_ADJUSTMENT_ROLL_F_GAIN(new_f);
                else
                    set_ADJUSTMENT_PITCH_F_GAIN(new_f);
            }

            setConfigDirty();
        }

        ffest.kf_correction[0] = 0.0f;
        ffest.kf_correction[1] = 0.0f;
    }

    ffest.was_active = ffest.active;

    DEBUG(FF_ESTIMATE, 0, lrintf(ffest.kf_correction[0] / ROLL_F_TERM_SCALE));
    DEBUG(FF_ESTIMATE, 1, lrintf(ffest.kf_correction[1] / PITCH_F_TERM_SCALE));
    DEBUG(FF_ESTIMATE, 2, lrintf(filtered_ff_error[0] * 1000));
    DEBUG(FF_ESTIMATE, 3, lrintf(filtered_ff_error[1] * 1000));
    DEBUG(FF_ESTIMATE, 4, ffest.active);
    DEBUG(FF_ESTIMATE, 5, currentPidProfile->pid[0].F);
    DEBUG(FF_ESTIMATE, 6, currentPidProfile->pid[1].F);
}

bool isFFEstimateActive(void)
{
    return ffest.active;
}

float ffEstimateGetKfCorrection(int axis)
{
    if (ffest.active && axis < 2)
        return ffest.kf_correction[axis];
        
    return 0.0f;
}
