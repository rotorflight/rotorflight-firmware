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

#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/governor.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/trim_flight.h"

#include "pg/mixer.h"


typedef struct {
    bool            active;
    bool            was_active;
    float           stick_threshold;
    float           max_trim;
    float           accum_rate;
    float           accumulator[2];
    float           input_rate_scale[2];
    pt1Filter_t     iterm_filter[2];
} trimFlightState_t;

static FAST_DATA_ZERO_INIT trimFlightState_t trim;

static int trim_flight_reset_state = 0;


void INIT_CODE trimFlightInit(void)
{
    trim.active = false;
    trim.was_active = false;
    trim.accumulator[0] = 0.0f;
    trim.accumulator[1] = 0.0f;

    const uint8_t gain = mixerConfig()->trim_flight_gain;

    if (gain == 0) {
        return;
    }

    trim.stick_threshold = mixerConfig()->trim_flight_stick_threshold / 1000.0f;
    trim.max_trim = mixerConfig()->trim_flight_max_trim / 1000.0f;
    trim.accum_rate = gain / 100.0f;

    trim.input_rate_scale[0] = mixerInputs(MIXER_IN_STABILIZED_ROLL)->rate / 1000.0f;
    trim.input_rate_scale[1] = mixerInputs(MIXER_IN_STABILIZED_PITCH)->rate / 1000.0f;

    const float cutoff_freq = 0.5f;
    const float pid_freq = pidGetPidFrequency();

    for (int i = 0; i < 2; i++) {
        pt1FilterInit(&trim.iterm_filter[i], cutoff_freq, pid_freq);
    }
}

static bool trimFlightCheckPreconditions(void)
{
    if (!ARMING_FLAG(ARMED))
        return false;

    if (!isSpooledUp())
        return false;

    if (!isAirborne())
        return false;

    if (getCosTiltAngle() < 0.995f)
        return false;

    if (fabsf(getRcDeflection(ROLL)) > trim.stick_threshold)
        return false;

    if (fabsf(getRcDeflection(PITCH)) > trim.stick_threshold)
        return false;

    if (!IS_RC_MODE_ACTIVE(BOXTRIMFLIGHT))
        return false;

    if (FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE | ANGLE_MODE | HORIZON_MODE))
        return false;

    return true;
}

void trimFlightUpdate(void)
{
    if (mixerConfig()->trim_flight_gain == 0) {
        return;
    }

    float filtered_iterm[2] = { 0.0f, 0.0f };

    trim.active = trimFlightCheckPreconditions();

    if (trim.active && !trim.was_active) {
        for (int i = 0; i < 2; i++) {
            trim.iterm_filter[i].y1 = 0.0f;
        }
    }

    if (trim.active) {
        const float dT = pidGetDT();
        const pidAxisData_t *pid_data = pidGetAxisData();

        for (int i = 0; i < 2; i++) {
            const float iterm_swash = pid_data[i].I * trim.input_rate_scale[i];
            filtered_iterm[i] = pt1FilterApply(&trim.iterm_filter[i], iterm_swash);

            const float delta = filtered_iterm[i] * trim.accum_rate * dT;
            const float current_trim = mixerConfig()->trim_flight_trim[i] / 1000.0f;
            const float total_trim = current_trim + trim.accumulator[i] + delta;
            const float clamped_total = constrainf(total_trim, -trim.max_trim, trim.max_trim);
            const float clamped_delta = clamped_total - current_trim - trim.accumulator[i];

            trim.accumulator[i] += clamped_delta;
        }
    }

    if (trim.was_active && !trim.active) {
        if (ARMING_FLAG(ARMED)) {
            for (int i = 0; i < 2; i++) {
                const int16_t new_trim = constrain(
                    mixerConfig()->trim_flight_trim[i] + lrintf(trim.accumulator[i] * 1000.0f),
                    -1000, 1000);

                mixerConfigMutable()->trim_flight_trim[i] = new_trim;
            }

            setConfigDirty();
        }

        trim.accumulator[0] = 0.0f;
        trim.accumulator[1] = 0.0f;
    }

    trim.was_active = trim.active;

    DEBUG(TRIM_FLIGHT, 0, lrintf(trim.accumulator[0] * 1000));
    DEBUG(TRIM_FLIGHT, 1, lrintf(trim.accumulator[1] * 1000));
    DEBUG(TRIM_FLIGHT, 2, lrintf(filtered_iterm[0] * 1000));
    DEBUG(TRIM_FLIGHT, 3, lrintf(filtered_iterm[1] * 1000));
    DEBUG(TRIM_FLIGHT, 4, trim.active);
    DEBUG(TRIM_FLIGHT, 5, lrintf((mixerConfig()->trim_flight_trim[0] / 1000.0f + trim.accumulator[0]) * 1000));
    DEBUG(TRIM_FLIGHT, 6, lrintf((mixerConfig()->trim_flight_trim[1] / 1000.0f + trim.accumulator[1]) * 1000));
}

bool isTrimFlightActive(void)
{
    return trim.active;
}

float trimFlightGetTrim(int axis)
{
    return mixerConfig()->trim_flight_trim[axis] / 1000.0f + trim.accumulator[axis];
}

int get_ADJUSTMENT_TRIM_FLIGHT_RESET(void)
{
    return trim_flight_reset_state;
}

void set_ADJUSTMENT_TRIM_FLIGHT_RESET(int value)
{
    trim_flight_reset_state = value;

    if (value != 0) {
        mixerConfigMutable()->trim_flight_trim[0] = 0;
        mixerConfigMutable()->trim_flight_trim[1] = 0;
        trim.accumulator[0] = 0.0f;
        trim.accumulator[1] = 0.0f;
    }
}
