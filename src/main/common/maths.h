/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "types.h"

/*
 * Floating point constants
 */

#define M_PIf           3.14159265358979323846f     //  π
#define M_PI2f          1.57079632679489661923f     //  π/2
#define M_PI_2f         1.57079632679489661923f     //  π/2
#define M_PI_4f         0.78539816339744830962f     //  π/4
#define M_PI_8f         0.39269908169872415481f     //  π/8
#define M_2PIf          6.28318530717958647693f     //  2π
#define M_1_2PIf        0.15915494309189533577f     //  1/2π
#define M_2_PIf         0.63661977236758134308f     //  2/π

#define M_RADf          0.01745329251994329577f     //  π/180
#define RAD             M_RADf                      //  π/180

/* Type for sin(x) and cos(x) return value */
typedef struct {
    float sin;
    float cos;
} sincosf_t;

// sincosf() is already a GCC extension
static inline sincosf_t sinfcosf(float x) { return (sincosf_t){ sinf(x), cosf(x) }; }

/*
 * Fast math routines
 */

#ifndef USE_STANDARD_MATH

float sin_fast(float x);
float cos_fast(float x);
float tan_fast(float x);
sincosf_t sincos_fast(float x);

float sin_approx(float x);
float cos_approx(float x);
float tan_approx(float x);
float tan_approx2(float x);
sincosf_t sincos_approx(float x);

float asin_approx(float x);
float acos_approx(float x);
float atan2_approx(float y, float x);

float exp_approx(float val);
float log_approx(float val);
float pow_approx(float a, float b);

#else /* USE_STANDARD_MATH */

#define sin_fast(x)         sinf(x)
#define cos_fast(x)         cosf(x)
#define tan_fast(x)         tanf(x)
#define sincos_fast(x)      sinfcosf(x)

#define sin_approx(x)       sinf(x)
#define cos_approx(x)       cosf(x)
#define tan_approx(x)       tanf(x)
#define sincos_approx(x)    sinfcosf(x)

#define asin_approx(x)      asinf(x)
#define acos_approx(x)      acosf(x)
#define atan2_approx(y,x)   atan2f(y,x)

#define exp_approx(x)       expf(x)
#define log_approx(x)       logf(x)
#define pow_approx(a, b)    powf(a, b)

#endif /* USE_STANDARD_MATH */


/*
 * Unit conversion macros
 */

#define DEGREES_TO_DECIDEGREES(angle)       ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle)       ((angle) / 10)
#define DECIDEGREES_TO_RADIANS(angle)       ((angle) / 10 * M_RADf)
#define DEGREES_TO_RADIANS(angle)           ((angle) * M_RADf)

#define CM_S_TO_KM_H(cmps)                  ((cmps) * 9 / 250)
#define CM_S_TO_MPH(cmps)                   ((cmps) * 125 / 5588)

#define HZ_TO_INTERVAL(x)                   (1.0f / (x))
#define HZ_TO_INTERVAL_US(x)                (1000000 / (x))


/*
 * Basic math macros
 */

#ifndef sq
#define sq(x) POWER2(x)
#endif

#ifndef sqf
#define sqf(x) POWER2((float)(x))
#endif

#define POWER2(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x * _x; })

#define POWER3(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x * _x * _x; })

#define POWER4(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x * _x * _x * _x; })

#define POWER5(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x * _x * _x * _x * _x; })

#define POWER6(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x * _x * _x * _x * _x * _x; })

#define MIN(a,b) \
  __extension__ ({ \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

#define MAX(a,b) \
  __extension__ ({ \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define ABS(x) \
  __extension__ ({ \
    __typeof__ (x) _x = (x); \
    _x > 0 ? _x : -_x; })

#define SIGN(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  (_x > 0) - (_x < 0); })

/*
 * Basic math operations
 */

static inline int constrain(int value, int low, int high)
{
    if (value < low)
        return low;
    else if (value > high)
        return high;
    else
        return value;
}

static inline float constrainf(float value, float low, float high)
{
    if (value < low)
        return low;
    else if (value > high)
        return high;
    else
        return value;
}

static inline int limit(int value, int limit)
{
    if (value < -limit)
        return -limit;
    else if (value > limit)
        return limit;
    else
        return value;
}

static inline float limitf(float value, float limit)
{
    if (value < -limit)
        return -limit;
    else if (value > limit)
        return limit;
    else
        return value;
}

static inline int scaleRange(int src, int srcFrom, int srcTo, int dstFrom, int dstTo)
{
    const int srcRange = srcTo - srcFrom;
    const int dstRange = dstTo - dstFrom;
    return ((src - srcFrom) * dstRange) / srcRange + dstFrom;
}

static inline float scaleRangef(float src, float srcFrom, float srcTo, float dstFrom, float dstTo)
{
    const float srcRange = srcTo - srcFrom;
    const float dstRange = dstTo - dstFrom;
    return ((src - srcFrom) * dstRange) / srcRange + dstFrom;
}

static inline float slewLimit(float current, float target, float rate)
{
    if (rate > 0) {
        if (target > current + rate)
            return current + rate;
        if (target < current - rate)
            return current - rate;
    }
    return target;
}

static inline float slewUpLimit(float current, float target, float rate)
{
    if (rate > 0) {
        if (target > current + rate)
            return current + rate;
    }
    return target;
}

static inline float slewDownLimit(float current, float target, float rate)
{
    if (rate > 0) {
        if (target < current - rate)
            return current - rate;
    }
    return target;
}

static inline float slewUpDownLimit(float current, float target, float uprate, float downrate)
{
    if (uprate > 0 && target > current + uprate) {
        return current + uprate;
    }
    if (downrate > 0 && target < current - downrate) {
        return current - downrate;
    }
    return target;
}

static inline int32_t applyDeadband(const int32_t value, const int32_t deadband)
{
    if (value > deadband)
        return value - deadband;
    else if (value < -deadband)
        return value + deadband;
    return 0;
}

static inline float fapplyDeadband(const float value, const float deadband)
{
    if (value > deadband)
        return value - deadband;
    else if (value < -deadband)
        return value + deadband;
    return 0;
}

static inline float transition(const float src, const float srcMin, const float srcMax,
                               const float dstMin, const float dstMax)
{
    if (src > srcMax)
        return dstMax;
    else if (src < srcMin)
        return dstMin;

    return scaleRangef(src, srcMin, srcMax, dstMin, dstMax);
}

static inline float degreesToRadians(int16_t degrees)
{
    return DEGREES_TO_RADIANS(degrees);
}


/*
 * Standard deviation calculus
 */

typedef struct stdev_s
{
    float m_oldM;
    float m_newM;
    float m_oldS;
    float m_newS;
    int m_n;
} stdev_t;

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);

/*
 * 3D calculus
 */

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union u_fp_vector {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

typedef struct fp_rotationMatrix_s {
    float m[3][3];
} fp_rotationMatrix_t;

void buildRotationMatrix(fp_angles_t *delta, fp_rotationMatrix_t *rotation);
void applyMatrixRotation(float *v, fp_rotationMatrix_t *rotationMatrix);


