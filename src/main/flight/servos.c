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

#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

static FAST_DATA_ZERO_INIT uint8_t  servoCount;

static FAST_DATA_ZERO_INIT float    servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT int16_t  servoOverride[MAX_SUPPORTED_SERVOS];

#ifdef USE_SERVO_CORRECTION_CURVE

typedef struct {
    float y[5];
    float u[5];
} servoSpline_t;

static FAST_DATA_ZERO_INIT servoSpline_t servoSpline[MAX_SUPPORTED_SERVOS];

#endif

PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoPwmRate = DEFAULT_SERVO_UPDATE;

    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->dev.ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .mid   = DEFAULT_SERVO_CENTER,
                     .min   = DEFAULT_SERVO_MIN,
                     .max   = DEFAULT_SERVO_MAX,
                     .rate  = DEFAULT_SERVO_RATE,
                     .trim  = DEFAULT_SERVO_TRIM,
                     .speed = DEFAULT_SERVO_SPEED,
                     .flags = DEFAULT_SERVO_FLAGS,
                     .curve = { 0, },
        );
    }
}


uint8_t getServoCount(void)
{
    return servoCount;
}

int16_t getServoOutput(uint8_t servo)
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

#ifdef USE_SERVO_CORRECTION_CURVE

static const float spx[5] = { -1.0f, -0.5f, 0, 0.5f, 1.0f };

void servoInitCurve(uint8_t servo)
{
    const servoParam_t *param = servoParams(servo);
    servoSpline_t *sp = &servoSpline[servo];

    float *y = sp->y;
    float *u = sp->u;

    y[0] = -1.0f + param->curve[0] / 1000.0f;
    y[1] = -0.5f + param->curve[1] / 1000.0f;
    y[2] =  0.0f;
    y[3] =  0.5f + param->curve[2] / 1000.0f;
    y[4] =  1.0f + param->curve[3] / 1000.0f;

    u[0] = 0;
    u[1] = ((y[2] - 2*y[1] + y[0])        ) / 4.0f;
    u[2] = ((y[3] - 2*y[2] + y[1]) - u[1] ) / 3.75f;
    u[3] = ((y[4] - 2*y[3] + y[2]) - u[2] ) / 3.7333333333f;
    u[4] = 0;

    u[2] = -0.2666666666f * u[3] + u[2];
    u[1] = -0.25f * u[2] + u[1];
}

static float servoEvalSpline(servoSpline_t * sp, float x, uint8_t index)
{
    const float a = 2 * (spx[index] - x);
    const float b = 1.0f - a;

    return a * sp->y[index - 1] + b * sp->y[index] +
        (a*a*a - a) * sp->u[index - 1] + (b*b*b - b) * sp->u[index];
}

static float servoCorrection(uint8_t servo, float x)
{
    servoSpline_t *sp = &servoSpline[servo];

    for (int i = 1; i < 4; i++) {
        if (x < spx[i])
            return servoEvalSpline(sp, x, i);
    }

    return servoEvalSpline(sp, x, 4);
}
#else
void servoInitCurve(uint8_t servo)
{
    UNUSED(servo);
}
#endif

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->dev.ioTags;

    for (servoCount = 0;
         servoCount < MAX_SUPPORTED_SERVOS && ioTags[servoCount] != IO_TAG_NONE;
         servoCount++);

    servoDevInit(&servoConfig()->dev, servoCount);

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoOutput[i] = servoParams(i)->mid;
        servoOverride[i] = SERVO_OVERRIDE_OFF;
        servoInitCurve(i);
    }
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

static inline float limitSpeed(float rate, float speed, float old, float new)
{
    float diff = new - old;

    rate = fabsf(rate * gyro.targetLooptime) / (speed * 1000);
    if (diff > rate)
        return old + rate;
    else if (diff < -rate)
        return old - rate;

    return new;
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
    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);
        float pos = mixerGetServoOutput(i);

        pos += servo->trim / 1000.0f;

#ifdef USE_SERVO_CORRECTION_CURVE
        if (servo->flags & SERVO_FLAG_CURVE_CORRECTION)
            pos = servoCorrection(i, pos);
#endif
#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (servo->flags & SERVO_FLAG_GEOMETRY_CORRECTION)
            pos = geometryCorrection(pos);
#endif

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            pos = servoOverride[i] / 1000.0f;

        pos = limitTravel(i, servo->rate * pos, servo->min, servo->max);
        pos = servo->mid + pos;

        if (servo->speed > 0)
            pos = limitSpeed(servo->rate, servo->speed, servoOutput[i], pos);

        servoOutput[i] = pos;

        pwmWriteServo(i, servoOutput[i]);
    }
}

#endif
