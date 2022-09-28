/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "axis.h"
#include "maths.h"

#ifndef USE_STANDARD_MATH

#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f

FAST_CODE float sin_approx(float x)
{
    int32_t xint = x;

    if (xint < -32 || xint > 32)
        return 0;

    while (x >  M_PIf)
        x -= M_2PIf;
    while (x < -M_PIf)
        x += M_2PIf;

    if (x >  M_PI2f)
        x =  M_PIf - x;
    else if (x < -M_PI2f)
        x = -M_PIf - x;

    const float x2 = x * x;

    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * sinPolyCoef7));
}

FAST_CODE float cos_approx(float x)
{
    return sin_approx(x + M_PI2f);
}

FAST_CODE float asin_approx(float x)
{
    return M_PI2f - acos_approx(x);
}

FAST_CODE float acos_approx(float x)
{
    const float xa = fabsf(x);

    float result = sqrtf(1.0f - xa) *
        (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));

    if (x < 0)
        result = M_PIf - result;

    return result;
}

#define atanPolyCoef1  3.14551665884836e-07f
#define atanPolyCoef2  0.99997356613987f
#define atanPolyCoef3  0.14744007058297684f
#define atanPolyCoef4  0.3099814292351353f
#define atanPolyCoef5  0.05030176425872175f
#define atanPolyCoef6  0.1471039133652469f
#define atanPolyCoef7  0.6444640676891548f

FAST_CODE float atan2_approx(float y, float x)
{
    float absX = fabsf(x);
    float absY = fabsf(y);
    float res  = MAX(absX, absY);

    if (res)
        res = MIN(absX, absY) / res;
    else
        res = 0.0f;

    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);

    if (absY > absX)
        res = M_PI2f - res;

    if (x < 0)
        res = M_PIf - res;
    if (y < 0)
        res = -res;

    return res;
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

    cosx = cos_approx(delta->angles.roll);
    sinx = sin_approx(delta->angles.roll);
    cosy = cos_approx(delta->angles.pitch);
    siny = sin_approx(delta->angles.pitch);
    cosz = cos_approx(delta->angles.yaw);
    sinz = sin_approx(delta->angles.yaw);

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

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v)
{
    int32_t p[7];
    QMF_COPY(p, v, 7);

    QMF_SORT(p[0], p[5]); QMF_SORT(p[0], p[3]); QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]); QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]); QMF_SORT(p[2], p[3]); QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]); QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v)
{
    int32_t p[9];
    QMF_COPY(p, v, 9);

    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]); QMF_SORT(p[5], p[8]); QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]); QMF_SORT(p[1], p[4]); QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]); QMF_SORT(p[4], p[2]); QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

float quickMedianFilter3f(float * v)
{
    float p[3];
    QMF_COPY(p, v, 3);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[0], p[1]) ;
    return p[1];
}

float quickMedianFilter5f(float * v)
{
    float p[5];
    QMF_COPY(p, v, 5);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[2], p[3]);
    QMF_SORTF(p[1], p[2]);
    return p[2];
}

float quickMedianFilter7f(float * v)
{
    float p[7];
    QMF_COPY(p, v, 7);

    QMF_SORTF(p[0], p[5]); QMF_SORTF(p[0], p[3]); QMF_SORTF(p[1], p[6]);
    QMF_SORTF(p[2], p[4]); QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[5]);
    QMF_SORTF(p[2], p[6]); QMF_SORTF(p[2], p[3]); QMF_SORTF(p[3], p[6]);
    QMF_SORTF(p[4], p[5]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[3]);
    QMF_SORTF(p[3], p[4]);
    return p[3];
}

float quickMedianFilter9f(float * v)
{
    float p[9];
    QMF_COPY(p, v, 9);

    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[6], p[7]);
    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[3]); QMF_SORTF(p[5], p[8]); QMF_SORTF(p[4], p[7]);
    QMF_SORTF(p[3], p[6]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[2], p[5]);
    QMF_SORTF(p[4], p[7]); QMF_SORTF(p[4], p[2]); QMF_SORTF(p[6], p[4]);
    QMF_SORTF(p[4], p[2]);
    return p[4];
}
