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

#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "axis.h"
#include "maths.h"

#ifndef MATH_CODE
# define MATH_CODE
#endif

#ifndef USE_STANDARD_MATH

/*
 * libc math functions: sinf()/cosf():
 *  - ~140 cycles on M7
 *  - 24 bits precision
 *
 * sin_approx()/cos_approx():
 *  - ~40 cycles on M7
 *  - >20 bits precision
 *
 * sin_fast()/cos_fast():
 *  - ~35 cycles on M7
 *  - 24 bits precision in [-π/4, π/4]
 */

// Fast sin approximation for x ∈ [-π/4, π/4].  Taylor series degree-9 odd polynomial.
MATH_CODE float sin_fast(float x)
{
    const float c1 =  1.0f;                         //  1
    const float c3 = -0.16666666666666667f;         // -1/3!
    const float c5 =  0.0083333333333333332f;       //  1/5!
    const float c7 = -0.00019841269841269841f;      // -1/7!
    const float c9 =  0.0000027557319223985893f;    //  1/9!
    const float x2 = x * x;
    return x * (c1 + x2 * (c3 + x2 * (c5 + x2 * (c7 + x2 * c9))));
}

// Fast cos approximation for x ∈ [-π/4, π/4].  Taylor series degree-8 even polynomial.
MATH_CODE float cos_fast(float x)
{
    const float c0 =  1.0f;                         //  1
    const float c2 = -0.5f;                         // -1/2
    const float c4 =  0.041666666666666664f;        //  1/24
    const float c6 = -0.001388888888888889f;        // -1/720
    const float c8 =  0.00002480158730158730f;      //  1/40320
    const float x2 = x * x;
    return (c0 + x2 * (c2 + x2 * (c4 + x2 * (c6 + x2 * c8))));
}

// Fast sin+cos approximation for x ∈ [-π/4, π/4].
MATH_CODE sincosf_t sincos_fast(float x)
{
    return (sincosf_t){ sin_fast(x), cos_fast(x) };
}

// Fast tan approximation for x ∈ [-π/4, π/4].
MATH_CODE float tan_fast(float x)
{
    return sin_fast(x) / cos_fast(x);
}

#if 0
// Taylor series degree-9 odd polynomial.
static inline float sin_poly(float r)
{
    const float c1 =  1.5707963267948966f;          //  (pi/2)^1 / 1!
    const float c3 = -0.64596409750624617f;         // -(pi/2)^3 / 3!
    const float c5 =  0.079692626246167033f;        //  (pi/2)^5 / 5!
    const float c7 = -0.0046817541353186866f;       // -(pi/2)^7 / 7!
    const float c9 =  0.00016044118478735976f;      //  (pi/2)^9 / 9!
    const float x2 = r * r;
    return r * (c1 + x2 * (c3 + x2 * (c5 + x2 * (c7 + x2 * c9))));
}

// Taylor series degree-8 even polynomial.
static inline float cos_poly(float r)
{
    const float c0 =  1.0f;                         //  1
    const float c2 = -1.2337005501361697f;          // -(pi/2)^2 / 2!
    const float c4 =  0.25366950790104797f;         //  (pi/2)^4 / 4!
    const float c6 = -0.020863480763352957f;        // -(pi/2)^6 / 6!
    const float c8 =  0.00091926027483942626f;      //  (pi/2)^8 / 8!
    const float x2 = r * r;
    return (c0 + x2 * (c2 + x2 * (c4 + x2 * (c6 + x2 * c8))));
}
#else
// Minimum degree-5 odd polynomial.
static inline float sin_poly(float r)
{
    const float c1 =  1.570788468983057f;
    const float c3 = -0.645711990181946f;
    const float c5 =  0.077667393626301f;
    const float r2 = r * r;
    return r * (c1 + r2 * (c3 + r2 * c5));
}

// Minimum degree-6 even polynomial.
static inline float cos_poly(float r)
{
    const float c2 = -1.233697953970536f;
    const float c4 =  0.253606361920527f;
    const float c6 = -0.020426250304794f;
    const float r2 = r * r;
    return 1.0f + r2 * (c2 + r2 * (c4 + r2 * c6));
}
#endif

MATH_CODE float sin_approx(float rad)
{
    float x = rad * M_2_PIf;
    float q = roundf(x);
    float r = x - q;
    int32_t i = q;
    float y = (i & 1) ? cos_poly(r) : sin_poly(r);
    return (i & 2) ? -y : y;
}

MATH_CODE float cos_approx(float rad)
{
    float x = rad * M_2_PIf;
    float q = roundf(x);
    float r = x - q;
    int32_t i = q;
    float y = (i & 1) ? -sin_poly(r) : cos_poly(r);
    return (i & 2) ? -y : y;
}

MATH_CODE sincosf_t sincos_approx(float rad)
{
    float x = rad * M_2_PIf;
    float q = roundf(x);
    float r = x - q;
    int32_t i = q;
    float s = sin_poly(r);
    float c = cos_poly(r);
    float sin = (i & 1) ?  c : s;
    float cos = (i & 1) ? -s : c;
    return (i & 2) ?
        (sincosf_t){ -sin, -cos } :
        (sincosf_t){  sin,  cos };
}

MATH_CODE float tan_approx(float rad)
{
    float x = rad * M_2_PIf;
    float q = roundf(x);
    float r = x - q;
    int32_t i = q;
    float s = sin_poly(r);
    float c = cos_poly(r);
    return (i & 1) ? -c/s : s/c;
}

MATH_CODE float tan_approx2(float rad)
{
    const float p0 = 1.29192793f;
    const float p1 = 1.27515733f;
    const float p2 = 1.26977468f;
    const float p3 = 1.33686483f;
    const float p4 = 0.791981876f;
    const float p5 = 2.81889224f;

    float x = rad * M_2_PIf;
    float q = roundf(x);
    float r = x - q;
    int32_t i = q;

    const float w = r * r;
    const float p = (((((p5 * w + p4) * w + p3) * w + p2) * w + p1) * w + p0);
    const float y = r * (M_PI2f + w * p);

    return (i & 1) ? (-1.0f / y) : y;
}

MATH_CODE float asin_approx(float x)
{
    return M_PI2f - acos_approx(x);
}

MATH_CODE float acos_approx(float x)
{
    const float a0 =  1.5707288f;
    const float a1 = -0.2121144f;
    const float a2 =  0.0742610f;
    const float a3 = -0.0187293f;

    const float z = fabsf(x);
    const float y = sqrtf(fmaxf(0.0f, 1.0f - z)) * (a0 + z * (a1 + z * (a2 + z * a3)));

    return (x < 0) ? M_PIf - y : y;
}


MATH_CODE float atan2_approx(float y, float x)
{
    const float n1 = 3.14551665884836e-7f;
    const float n2 = 0.99997356613987f;
    const float n3 = 0.14744007058297684f;
    const float n4 = 0.3099814292351353f;
    const float n5 = 0.05030176425872175f;
    const float d1 = 0.1471039133652469f;
    const float d2 = 0.6444640676891548f;

    float absX = fabsf(x);
    float absY = fabsf(y);

    float z = fmaxf(absX, absY);
    float r = (z) ? fminf(absX, absY) / z : 0.0f;

    float s = -((((n5 * r - n4) * r - n3) * r - n2) * r - n1) /
                ((d2 * r + d1) * r + 1.0f);

    if (absY > absX)
        s = M_PI2f - s;
    if (x < 0)
        s = M_PIf - s;
    if (y < 0)
        s = -s;

    return s;
}

MATH_CODE float exp_approx(float x)
{
    const float c1 = 12102203.1615614f;
    const float c2 = 1065353216.0f;
    const float c3 = 2139095040.0f;

    union { int32_t i; float f; } xu, xv;

    float a = c1 * x + c2;
    float b = (a < c3) ? a : c3;
    int32_t c = (b > 0) ? b : 0;

    xu.i = (c & 0x7F800000);
    xv.i = (c & 0x007FFFFF) | 0x3F800000;

    float y = xu.f * (0.509871020343597804469416f +
              xv.f * (0.312146713032169896138863f +
              xv.f * (0.166617139319965966118107f +
              xv.f * (-2.19061993049215080032874e-3f +
              xv.f * (1.3555747234758484073940937e-2f)))));

    return y;
}

MATH_CODE float log_approx(float x)
{
    /* 89.970756366f = 127 * log(2) - constant term of polynomial */
    const float c1 = -89.970756366f;

    union { float f; int32_t i; } xu = { .f = x };

    float cst = (x > 0) ? c1 : -(float)INFINITY;
    float exp = xu.i >> 23;
    xu.i = (xu.i & 0x007FFFFF) | 0x3F800000;

    float y = xu.f * (3.529304993f +
              xu.f * (-2.461222105f +
              xu.f * (1.130626167f +
              xu.f * (-0.288739945f +
              xu.f * (3.110401639e-2f))))) +
              (cst + 0.69314718055995f * exp);

    return y;
}

MATH_CODE float pow_approx(float a, float b)
{
    return exp_approx(b * log_approx(a));
}

#endif /* USE_STANDARD_MATH */


void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


void buildRotationMatrix(fp_angles_t *delta, fp_rotationMatrix_t *rotation)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta->angles.roll);
    sinx = sinf(delta->angles.roll);
    cosy = cosf(delta->angles.pitch);
    siny = sinf(delta->angles.pitch);
    cosz = cosf(delta->angles.yaw);
    sinz = sinf(delta->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    rotation->m[0][X] = cosz * cosy;
    rotation->m[0][Y] = -cosy * sinz;
    rotation->m[0][Z] = siny;
    rotation->m[1][X] = sinzcosx + (coszsinx * siny);
    rotation->m[1][Y] = coszcosx - (sinzsinx * siny);
    rotation->m[1][Z] = -sinx * cosy;
    rotation->m[2][X] = (sinzsinx) - (coszcosx * siny);
    rotation->m[2][Y] = (coszsinx) + (sinzcosx * siny);
    rotation->m[2][Z] = cosy * cosx;
}

void applyMatrixRotation(float *v, fp_rotationMatrix_t *rotationMatrix)
{
    struct fp_vector *vDest = (struct fp_vector *)v;
    struct fp_vector vTmp = *vDest;

    vDest->X = (rotationMatrix->m[0][X] * vTmp.X + rotationMatrix->m[1][X] * vTmp.Y + rotationMatrix->m[2][X] * vTmp.Z);
    vDest->Y = (rotationMatrix->m[0][Y] * vTmp.X + rotationMatrix->m[1][Y] * vTmp.Y + rotationMatrix->m[2][Y] * vTmp.Z);
    vDest->Z = (rotationMatrix->m[0][Z] * vTmp.X + rotationMatrix->m[1][Z] * vTmp.Y + rotationMatrix->m[2][Z] * vTmp.Z);
}
