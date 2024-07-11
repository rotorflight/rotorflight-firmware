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

#include "common/maths.h"
#include "common/axis.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/config.h"

#include "pg/arming.h"

#include "flight/pid.h"
#include "flight/wiggle.h"


typedef struct {
    bitmap_t    flags;
    float       strength;
    uint        period;
    int         action;
    int         param;
    int         state;
    timeUs_t    start;
    float       axis[4];
} wiggleState_t;

static wiggleState_t wgl = INIT_ZERO;


float wiggleGetAxis(int axis)
{
    return wgl.axis[axis];
}

bool wiggleActive(void)
{
    return (wgl.action != WIGGLE_OFF);
}

bool wiggleEnabled(int wiggle)
{
    return (wgl.flags & BIT(wiggle));
}

void wiggleTrigger(int wiggle, int param)
{
    if (wiggleEnabled(wiggle)) {
        wgl.action = wiggle;
        wgl.param = param;
        wgl.state = 0;
    }
}

static inline void wiggleResetAxis(void)
{
    wgl.axis[0] = wgl.axis[1] = wgl.axis[2] = wgl.axis[3] = 0;
}

static void wiggleSetState(timeUs_t currentTimeUs, uint8_t state)
{
    wgl.state = state;
    wgl.start = currentTimeUs;
}

static timeDelta_t wiggleStateTime(timeUs_t currentTimeUs)
{
    return cmpTimeUs(currentTimeUs, wgl.start) / 1000;
}

static void wiggleNextStateAfter(timeUs_t currentTimeUs, timeDelta_t period)
{
    if (wiggleStateTime(currentTimeUs) > period) {
        wiggleSetState(currentTimeUs, wgl.state + 1);
    }
}

static void wiggleStopAction(timeUs_t __unused currentTimeUs)
{
    wiggleResetAxis();
    wgl.action = 0;
    wgl.state = 0;
    wgl.start = 0;
}


static void wiggleActionReady(timeUs_t currentTimeUs)
{
    const timeDelta_t circle_period = 1200;
    const timeDelta_t exit_period = 750;
    const timeDelta_t jump_period = 100;

    const float level = pidGetCollective();

    switch (wgl.state)
    {
        case 0:
        {
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;
        }
        case 1:
        {
            const float angle = wiggleStateTime(currentTimeUs) * M_2PIf / circle_period;
            wgl.axis[FD_ROLL] = sin_approx(angle) * wgl.strength;
            wgl.axis[FD_PITCH] = cos_approx(angle) * wgl.strength;
            wgl.axis[FD_COLL] = level;
            wiggleNextStateAfter(currentTimeUs, circle_period);
            break;
        }
        case 2:
        {
            wgl.axis[FD_ROLL] = 0;
            wgl.axis[FD_PITCH] = 0;
            wgl.axis[FD_COLL] = level;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        case 3:
        {
            wgl.axis[FD_ROLL] = 0;
            wgl.axis[FD_PITCH] = 0;
            wgl.axis[FD_COLL] = level + wgl.strength;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        case 4:
        {
            wgl.axis[FD_ROLL] = 0;
            wgl.axis[FD_PITCH] = 0;
            wgl.axis[FD_COLL] = level - wgl.strength;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        case 5:
        {
            const float time = wiggleStateTime(currentTimeUs);
            wgl.axis[FD_ROLL] = 0;
            wgl.axis[FD_PITCH] = 0;
            wgl.axis[FD_COLL] = level - (wgl.strength * (exit_period - time) / exit_period);
            wiggleNextStateAfter(currentTimeUs, exit_period);
            break;
        }
        default:
        {
            wiggleStopAction(currentTimeUs);
            break;
        }
    }
}

static void wiggleActionArmed(timeUs_t currentTimeUs)
{
    const timeDelta_t jump_period = 50;

    const float level = pidGetCollective();

    switch (wgl.state)
    {
        case 0:
        {
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;
        }
        case 1:
        {
            wgl.axis[FD_COLL] = level - wgl.strength;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        case 2:
        {
            wgl.axis[FD_COLL] = level;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        case 3:
        {
            wgl.axis[FD_COLL] = level + wgl.strength;
            wiggleNextStateAfter(currentTimeUs, jump_period);
            break;
        }
        default:
        {
            wiggleStopAction(currentTimeUs);
            break;
        }
    }
}

static void wiggleActionError(timeUs_t currentTimeUs)
{
    const float level = pidGetCollective();

    switch (wgl.state)
    {
        case 0:
        {
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;
        }
        case 1:
        {
            const int cycle = wiggleStateTime(currentTimeUs) / wgl.period;
            wgl.axis[FD_COLL] = (cycle & 1) ? level - wgl.strength : level + wgl.strength;
            wiggleNextStateAfter(currentTimeUs, wgl.param);
            break;
        }
        default:
        {
            wiggleStopAction(currentTimeUs);
            break;
        }
    }
}

static void wiggleActionFatal(timeUs_t currentTimeUs)
{
    const float level = pidGetCollective();

    const int period = 500;

    switch (wgl.state)
    {
        case 0:
        {
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;
        }
        case 1:
        {
            const timeDelta_t time = wiggleStateTime(currentTimeUs);
            const int freq = (time % period) / wgl.period;
            const int step = time / period;
            wgl.axis[FD_COLL] = (step & 1) ? 0 : ((freq & 1) ? level - wgl.strength : level + wgl.strength);
            wiggleNextStateAfter(currentTimeUs, wgl.param);
            break;
        }
        default:
        {
            wiggleStopAction(currentTimeUs);
            break;
        }
    }
}


void wiggleUpdate(timeUs_t currentTimeUs)
{
    if (wgl.action) {
        switch (wgl.action)
        {
            case WIGGLE_READY:
                wiggleActionReady(currentTimeUs);
                break;
            case WIGGLE_ARMED:
                wiggleActionArmed(currentTimeUs);
                break;
            case WIGGLE_ERROR:
                wiggleActionError(currentTimeUs);
                break;
            case WIGGLE_FATAL:
                wiggleActionFatal(currentTimeUs);
                break;
            default:
                wiggleStopAction(currentTimeUs);
                break;
        }
    }
}

void INIT_CODE wiggleInit(void)
{
    wgl.strength = constrain(armingConfig()->wiggle_strength, 0, 100) / 100.0f;
    wgl.period   = 1000 / constrain(armingConfig()->wiggle_frequency, 2, 50);
    wgl.flags    = armingConfig()->wiggle_flags;
}
