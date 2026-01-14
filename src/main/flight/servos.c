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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/time.h"
#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/servos.h"


static FAST_DATA_ZERO_INIT uint8_t      servoCount;

static FAST_DATA_ZERO_INIT float        servoInput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoResolution[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT int16_t      servoOverride[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT timerChannel_t servoChannel[MAX_SUPPORTED_SERVOS];


uint8_t getServoCount(void)
{
    return servoCount;
}

uint16_t getServoOutput(uint8_t servo)
{
    return lrintf(servoOutput[servo]);
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t val)
{
    return servoOverride[servo] = val;
}

void validateAndFixServoConfig(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
#ifndef USE_SERVO_GEOMETRY_CORRECTION
        servoParamsMutable(i)->flags &= ~SERVO_FLAG_GEO_CORR;
#endif
    }
}

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->ioTags;
    const timerHardware_t *timer[MAX_SUPPORTED_SERVOS];
    uint32_t rates[MAX_SUPPORTED_SERVOS];
    uint8_t index, jndex;

    for (index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        servoOutput[index] = servoParams(index)->mid;
        servoOverride[index] = SERVO_OVERRIDE_OFF;
    }

    for (index = 0; index < MAX_SUPPORTED_SERVOS && ioTags[index]; index++)
    {
        const ioTag_t tag = ioTags[index];
        const IO_t io = IOGetByTag(tag);

        timer[index] = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(index));

        if (!timer[index])
            break;

        IOInit(io, OWNER_SERVO, RESOURCE_INDEX(index));
        IOConfigGPIOAF(io, IOCFG_AF_PP, timer[index]->alternateFunction);
    }

    servoCount = index;

    for (index = 0; index < servoCount; index++)
    {
        uint32_t update_rate = servoParams(index)->rate;

        for (jndex = 0; jndex < servoCount; jndex++) {
            if (timer[index]->tim == timer[jndex]->tim) {
                uint32_t maxpulse = servoParams(jndex)->mid + servoParams(jndex)->max;
                uint32_t maxrate = MIN(servoParams(jndex)->rate, 950000 / maxpulse);  // 1000000 / (maxpulse +5%)
                if (maxrate < update_rate)
                    update_rate = maxrate;
            }
        }

        rates[index] = constrain(update_rate, SERVO_RATE_MIN, SERVO_RATE_MAX);
    }

    for (index = 0; index < servoCount; index++)
    {
        const uint32_t timer_clock = timerClock(timer[index]->tim);
        const uint32_t update_rate = rates[index];
        uint32_t timebase;

        servoParamsMutable(index)->rate = update_rate;

        if (update_rate > 500) {
            const uint32_t timer_rate = update_rate * 64000;
            const uint32_t timer_div = (timer_clock + timer_rate - 1) / timer_rate;
            timebase = timer_clock / timer_div;
        }
        else if (update_rate > 154 && timer_clock % 10000000 == 0) {
            timebase = 10000000;
        }
        else if (update_rate > 124 && timer_clock % 8000000 == 0) {
            timebase = 8000000;
        }
        else if (update_rate > 77 && timer_clock % 5000000 == 0) {
            timebase = 5000000;
        }
        else if (update_rate > 62 && timer_clock % 4000000 == 0) {
            timebase = 4000000;
        }
        else if (update_rate > 31 && timer_clock % 2000000 == 0) {
            timebase = 2000000;
        }
        else {
            timebase = 1000000;
        }

        servoResolution[index] = timebase * 1e-6f;

        pwmOutConfig(&servoChannel[index], timer[index], timebase, timebase / update_rate, 0, 0);
    }
}

void servoShutdown(void)
{
    for (int index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        if (servoChannel[index].ccr) {
            *servoChannel[index].ccr = 0;
            servoChannel[index].ccr = NULL;
        }
    }

    delay(100);
}

static inline void servoSetOutput(uint8_t index, float pos)
{
    servoOutput[index] = pos;

    if (servoChannel[index].ccr)
        *servoChannel[index].ccr = lrintf(pos * servoResolution[index]);
}

static inline float limitTravel(uint8_t servo, float pos, float min, float max)
{
    if (pos > max) {
        mixerSaturateServoOutput(servo);
        return max;
    } else if (pos < min) {
        mixerSaturateServoOutput(servo);
        return min;
    }
    return pos;
}

static inline float limitSpeed(float old, float new, float speed)
{
    float rate = 1200 * pidGetDT() / speed;
    float diff = new - old;

    if (diff > rate)
        new = old + rate;
    else if (diff < -rate)
        new = old - rate;

    return new;
 }

 static inline float limitRatio(float old, float new, float ratio)
 {
    return old + (new - old) * ratio;
 }

#ifdef USE_SERVO_GEOMETRY_CORRECTION
static float geometryCorrection(float pos)
{
    // 1.0 == 50° without correction
    float height = constrainf(pos * 0.7660444431f, -1, 1);

    // Scale 50° in rad => 1.0
    float rotation = asin_approx(height) * 1.14591559026f;

    return rotation;
}
#endif

void servoUpdate(void)
{
    float input[MAX_SUPPORTED_SERVOS];
    float cyclic_ratio = 1;

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            input[i] = servoOverride[i] / 1000.0f;
        else
            input[i] = mixerGetServoOutput(i);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (servo->flags & SERVO_FLAG_GEO_CORR)
            input[i] = geometryCorrection(input[i]);
#endif

        if (servo->speed && mixerIsCyclicServo(i)) {
            const float limit = 1200 * pidGetDT() / servo->speed;
            const float speed = fabsf(input[i] - servoInput[i]);
            if (speed > limit)
                cyclic_ratio = fminf(cyclic_ratio, limit / speed);
        }
    }

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);
        float pos = input[i];

        if (servo->speed > 0) {
            if (mixerIsCyclicServo(i))
                pos = limitRatio(servoInput[i], pos, cyclic_ratio);
            else
                pos = limitSpeed(servoInput[i], pos, servo->speed);
        }

        servoInput[i] = pos;

        if (servo->flags & SERVO_FLAG_REVERSED)
            pos = -pos;

        float scale = (pos > 0) ? servo->rpos : servo->rneg;

        pos = limitTravel(i, scale * pos, servo->min, servo->max);
        pos = servo->mid + pos;

        servoSetOutput(i, pos);
    }
}

#endif
