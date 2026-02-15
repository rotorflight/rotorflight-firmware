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

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "airborne.h"

#define FILTER_CUTOFF                       5.0f

#define PEAK_UP_CUTOFF                     20.0f
#define PEAK_DN_CUTOFF                      0.5f

#define LIFTOFF_COS_ANGLE_THRESHOLD        0.80f
#define LANDING_COS_ANGLE_THRESHOLD        0.90f

typedef enum {
    AIRBORNE_STATE_INIT = 0,
    AIRBORNE_STATE_LANDED,
    AIRBORNE_STATE_AIRBORNE,
} airborneState_e;

typedef struct
{
    airborneState_e state;

    float liftoffThreshold[4];
    float landingThreshold[4];

    pt1Filter_t filter[4];
    peakFilter_t stickDeflection[4];

} airborneData_t;

static FAST_DATA_ZERO_INIT airborneData_t airborne;


INIT_CODE void airborneInit(void)
{
    airborne.state = AIRBORNE_STATE_LANDED;

    for (int axis = 0; axis < 4; axis++) {
        pt1FilterInit(&airborne.filter[axis], FILTER_CUTOFF, pidGetPidFrequency());
        peakFilterInit(&airborne.stickDeflection[axis], PEAK_UP_CUTOFF, PEAK_DN_CUTOFF, pidGetPidFrequency());
        airborne.liftoffThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1000.0f;
        airborne.landingThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1500.0f;
    }
}

static bool isOverThreshold(const float *threshold)
{
    return (
        peakFilterOutput(&airborne.stickDeflection[FD_ROLL]) > threshold[FD_ROLL] ||
        peakFilterOutput(&airborne.stickDeflection[FD_PITCH]) > threshold[FD_PITCH] ||
        peakFilterOutput(&airborne.stickDeflection[FD_YAW]) > threshold[FD_YAW] ||
        peakFilterOutput(&airborne.stickDeflection[FD_COLL]) > threshold[FD_COLL]
    );
}

static bool liftoff(void)
{
    return (
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            isOverThreshold(airborne.liftoffThreshold) ||
            getCosTiltAngle() < LIFTOFF_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}

static bool touchdown(void)
{
    return !(
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            isOverThreshold(airborne.landingThreshold) ||
            getCosTiltAngle() < LANDING_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}

static void updateStickDeflection(const float SP[4])
{
    for (int axis = 0; axis < 4; axis++) {
        float stick = pt1FilterApply(&airborne.filter[axis], SP[axis]);
        if (axis == FD_COLL)
            stick = fmaxf(stick, 0);
        peakFilterApply(&airborne.stickDeflection[axis], fabsf(stick));
    }
}

void airborneUpdate(const float SP[4])
{
    updateStickDeflection(SP);

    switch (airborne.state) {
        case AIRBORNE_STATE_INIT:
            break;
        case AIRBORNE_STATE_LANDED:
            if (liftoff()) {
                airborne.state = AIRBORNE_STATE_AIRBORNE;
            }
            break;
        case AIRBORNE_STATE_AIRBORNE:
            if (touchdown()) {
                airborne.state = AIRBORNE_STATE_LANDED;
            }
            break;
    }

    DEBUG(AIRBORNE, 0, peakFilterOutput(&airborne.stickDeflection[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, peakFilterOutput(&airborne.stickDeflection[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, peakFilterOutput(&airborne.stickDeflection[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, peakFilterOutput(&airborne.stickDeflection[FD_COLL]) * 1000);
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, liftoff());
    DEBUG(AIRBORNE, 6, touchdown());
    DEBUG(AIRBORNE, 7, airborne.state);
}

bool isAirborne(void)
{
    return (airborne.state == AIRBORNE_STATE_AIRBORNE);
}

bool isHandsOn(void)
{
    return isOverThreshold(airborne.liftoffThreshold);
}