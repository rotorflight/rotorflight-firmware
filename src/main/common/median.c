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

#include "common/median.h"

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a, b) { if ((a) > (b)) QMF_SWAP((a), (b)); }
#define QMF_SWAP(a, b) { int32_t temp = (a); (a) = (b); (b) = temp; }
#define QMF_COPY(p, v, n) { int32_t i; for (i = 0; i < (n); i++) (p)[i] = (v)[i]; }
#define QMF_SORTF(a, b) { if ((a) > (b)) QMF_SWAPF((a), (b)); }
#define QMF_SWAPF(a, b) { float temp = (a); (a) = (b); (b) = temp; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]);
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

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[0], p[1]);
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
