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

#pragma once

#include <stdbool.h>
#include "common/utils.h"


#define BUTTER_Q        0.707106781f     /* 2nd order Butterworth: 1/sqrt(2) */
#define BESSEL_Q        0.577350269f     /* 2nd order Bessel: 1/sqrt(3) */
#define DAMPED_Q        0.5f             /* 2nd order Critically damped: 1/sqrt(4) */

#define BUTTER_C        1.0f
#define BESSEL_C        1.272019649f
#define DAMPED_C        1.553773974f

#define BUTTER_4A_Q     0.541196100f     /* 4nd order Butterworth 1st section */
#define BUTTER_4B_Q     1.306562965f     /* 4nd order Butterworth 2nd section */

#define BUTTER_4A_C     1.0f
#define BUTTER_4B_C     1.0f

#define BESSEL_4A_Q     0.805538282f
#define BESSEL_4B_Q     0.521934582f

#define BESSEL_4A_C     1.603357516f
#define BESSEL_4B_C     1.430171560f


enum {
    LPF_NONE = 0,
    LPF_1ST_ORDER,      /* Default first order filter type */
    LPF_2ND_ORDER,      /* Default second order filter type */
    LPF_PT1,
    LPF_PT2,
    LPF_PT3,
    LPF_ORDER1,
    LPF_BUTTER,
    LPF_BESSEL,
    LPF_DAMPED,
};

enum {
    BIQUAD_NULL = 0,
    BIQUAD_LPF,
    BIQUAD_HPF,
    BIQUAD_BPF,
    BIQUAD_NOTCH,
};

typedef struct {
    float y1;
} nilFilter_t;

typedef struct {
    float y1;
    float gain;
} pt1Filter_t;

typedef struct {
    float y1;
    float y2;
    float gain;
} pt2Filter_t;

typedef struct {
    float y1;
    float y2;
    float y3;
    float gain;
} pt3Filter_t;

typedef struct {
    float y1;
    float W;
    uint32_t N;
} ewma1Filter_t;

typedef struct {
    float y1;
    float y2;
    float W;
    uint32_t N;
} ewma2Filter_t;

typedef struct {
    float y1;
    float y2;
    float y3;
    float W;
    uint32_t N;
} ewma3Filter_t;

typedef struct {
    float y1;
    float x1;
    float a;
    float b;
} difFilter_t;

typedef struct {
    float y1;
    float x1;
    float min;
    float max;
    float gain;
} intFilter_t;

typedef struct {
    float y1;
    float x1;
    float b0;
    float b1;
    float a1;
} order1Filter_t;

typedef struct {
    float y1;
    float y2;
    float x1;
    float x2;
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} biquadFilter_t;

typedef struct {
    float y1;
    float alpha;
    float beta;
} peakFilter_t;

typedef union {
    nilFilter_t     nil;
    pt1Filter_t     pt1;
    pt2Filter_t     pt2;
    pt3Filter_t     pt3;
    ewma1Filter_t   ew1;
    ewma2Filter_t   ew2;
    ewma3Filter_t   ew3;
    order1Filter_t  fos;
    biquadFilter_t  sos;
} filterData_t;

typedef struct filter_s filter_t;

typedef void  (*filterInitFn)(filterData_t *filter, float cutoff, float sampleRate);
typedef void  (*filterUpdateFn)(filterData_t *filter, float cutoff, float sampleRate);
typedef float (*filterApplyFn)(filterData_t *filter, float input);

typedef struct filter_s {
    filterInitFn    init;
    filterApplyFn   apply;
    filterUpdateFn  update;
    filterData_t    data;
} filter_t;

enum {
    LPF_UPDATE  = BIT(0),
    LPF_EWMA    = BIT(1),
};


static inline void filterInit(filter_t *filter, float cutoff, float sampleRate)
{
    if (filter->init)
        filter->init(&filter->data, cutoff, sampleRate);
}

static inline void filterUpdate(filter_t *filter, float cutoff, float sampleRate)
{
    if (filter->update)
        filter->update(&filter->data, cutoff, sampleRate);
}

static inline float filterApply(filter_t *filter, float input)
{
    if (filter->apply)
        return filter->apply(&filter->data, input);
    else
        return input;
}

static inline float filterOutput(filter_t *filter)
{
    return filter->data.nil.y1;
}

void nilFilterInit(nilFilter_t *filter, float cutoff, float sampleRate);
void nilFilterUpdate(nilFilter_t *filter, float cutoff, float sampleRate);
float nilFilterApply(nilFilter_t *filter, float input);

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float sampleRate);
void pt1FilterUpdate(pt1Filter_t *filter, float cutoff, float sampleRate);
void pt1FilterInitGain(pt1Filter_t *filter, float gain);
void pt1FilterUpdateGain(pt1Filter_t *filter, float gain);
float pt1FilterGain(float cutoff, float sampleRate);
float pt1FilterApply(pt1Filter_t *filter, float input);
static inline float pt1FilterOutput(pt1Filter_t *filter) { return filter->y1; }

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float sampleRate);
void pt2FilterUpdate(pt2Filter_t *filter, float cutoff, float sampleRate);
void pt2FilterInitGain(pt2Filter_t *filter, float gain);
void pt2FilterUpdateGain(pt2Filter_t *filter, float gain);
float pt2FilterGain(float cutoff, float sampleRate);
float pt2FilterApply(pt2Filter_t *filter, float input);
static inline float pt2FilterOutput(pt2Filter_t *filter) { return filter->y1; }

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float sampleRate);
void pt3FilterUpdate(pt3Filter_t *filter, float cutoff, float sampleRate);
void pt3FilterInitGain(pt3Filter_t *filter, float gain);
void pt3FilterUpdateGain(pt3Filter_t *filter, float gain);
float pt3FilterGain(float cutoff, float sampleRate);
float pt3FilterApply(pt3Filter_t *filter, float input);
static inline float pt3FilterOutput(pt3Filter_t *filter) { return filter->y1; }

void ewma1FilterInit(ewma1Filter_t *filter, float cutoff, float sampleRate);
void ewma1FilterUpdate(ewma1Filter_t *filter, float cutoff, float sampleRate);
void ewma1FilterInitWeight(ewma1Filter_t *filter, float weight);
void ewma1FilterUpdateWeight(ewma1Filter_t *filter, float weight);
float ewma1FilterWeight(float cutoff, float sampleRate);
float ewma1FilterApply(ewma1Filter_t *filter, float input);
static inline float ewma1FilterOutput(ewma1Filter_t *filter) { return filter->y1; }

void ewma2FilterInit(ewma2Filter_t *filter, float cutoff, float sampleRate);
void ewma2FilterUpdate(ewma2Filter_t *filter, float cutoff, float sampleRate);
void ewma2FilterInitWeight(ewma2Filter_t *filter, float weight);
void ewma2FilterUpdateWeight(ewma2Filter_t *filter, float weight);
float ewma2FilterWeight(float cutoff, float sampleRate);
float ewma2FilterApply(ewma2Filter_t *filter, float input);
static inline float ewma2FilterOutput(ewma2Filter_t *filter) { return filter->y1; }

void ewma3FilterInit(ewma3Filter_t *filter, float cutoff, float sampleRate);
void ewma3FilterUpdate(ewma3Filter_t *filter, float cutoff, float sampleRate);
void ewma3FilterInitWeight(ewma3Filter_t *filter, float weight);
void ewma3FilterUpdateWeight(ewma3Filter_t *filter, float weight);
float ewma3FilterWeight(float cutoff, float sampleRate);
float ewma3FilterApply(ewma3Filter_t *filter, float input);
static inline float ewma3FilterOutput(ewma3Filter_t *filter) { return filter->y1; }

void difFilterInit(difFilter_t *filter, float cutoff, float sampleRate);
void difFilterUpdate(difFilter_t *filter, float cutoff, float sampleRate);
float difFilterApply(difFilter_t *filter, float input);
static inline float difFilterOutput(difFilter_t *filter) { return filter->y1; }

void intFilterInit(intFilter_t *filter, float sampleRate, float min, float max);
void intFilterReset(intFilter_t *filter);
void intFilterUpdate(intFilter_t *filter, float sampleRate, float min, float max);
float intFilterApply(intFilter_t *filter, float input);
static inline float intFilterOutput(intFilter_t *filter) { return filter->y1; }

void biquadBesselInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadBesselUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadButterInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadButterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadDampedInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadDampedUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadFilterInit(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType);
void biquadFilterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType);

float biquadFilterApply(biquadFilter_t *filter, float input);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApplyDF2(biquadFilter_t *filter, float input);

static inline float biquadFilterOutput(biquadFilter_t *filter) { return filter->y1; }

void firstOrderLPFInit(order1Filter_t *filter, float cutoff, float sampleRate);
void firstOrderLPFUpdate(order1Filter_t *filter, float cutoff, float sampleRate);
void firstOrderHPFInit(order1Filter_t *filter, float cutoff, float sampleRate);
void firstOrderHPFUpdate(order1Filter_t *filter, float cutoff, float sampleRate);
float firstOrderFilterApply(order1Filter_t *filter, float input);
static inline float firstOrderFilterOutput(order1Filter_t *filter) { return filter->y1; }

float filterStackApply(biquadFilter_t *filter, float input, int count);

void lowpassFilterInit(filter_t *filter, uint8_t type, float cutoff, float sampleRate, uint32_t flags);

void notchFilterInit(filter_t *filter, float cutoff, float Q, float sampleRate, uint32_t flags);
void notchFilterUpdate(filter_t *filter, float cutoff, float Q, float sampleRate);
float notchFilterGetQ(float centerFreq, float cutoffFreq);

void peakFilterInit(peakFilter_t *filter, float cutoff_up, float cutoff_down, float sampleRate);
float peakFilterApply(peakFilter_t *filter, float input);
static inline float peakFilterOutput(peakFilter_t *filter) { return filter->y1; }

typedef struct simpleLowpassFilter_s {
    int32_t fp;
    int32_t beta;
    int32_t fpShift;
} simpleLowpassFilter_t;

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal);
void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift);
