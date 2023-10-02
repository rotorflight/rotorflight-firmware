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

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"


static inline float limitCutoff(float cutoff, float sampleRate)
{
    // 95% of Nyquist
    return fminf(cutoff, 0.475f * sampleRate);
}


// NIL filter

FAST_CODE float nilFilterApply(nilFilter_t *filter, float input)
{
    return filter->y1 = input;
}

void nilFilterUpdate(nilFilter_t *filter, float cutoff, float sampleRate)
{
    UNUSED(filter);
    UNUSED(cutoff);
    UNUSED(sampleRate);
}

void nilFilterInit(nilFilter_t *filter, float cutoff, float sampleRate)
{
    UNUSED(filter);
    UNUSED(cutoff);
    UNUSED(sampleRate);
}


/*
 * PT1 Low Pass filter
 *
 * It is calculated like this:
 *
 *   yₙ = yₙ₋₁ + α(xₙ - yₙ₋₁)
 *
 * where
 *
 *   Fs = Sampling frequency
 *   Fc = Cutoff frequency
 *
 *            Fc
 *    ω = 2π⋅――――
 *            Fs
 *
 *            1          ω           Fc
 *    α = ――――――――― = ―――――――― = ――――――――――――
 *         1/ω + 1     1 + ω      Fc + Fs/2π
 *
 *
 * The transfer function is:
 *
 *                  α
 *   H(z) = ―――――――――――――――――
 *           1 - (1 - α)⋅z⁻¹
 *
 *
 * This is a first order low-pass filter, which is transformed
 * from s-domain to z-domain with the backwards difference method:
 *
 *         1     z - 1
 *    s ← ――― ⋅ ―――――――
 *         T       z
 *
 * This is also known as rectangular integration.
 *
 * Like the bilinear transform, it is warping frequencies, and
 * the actual -3dB cutoff frequency can be calculated with
 *
 *          Fs         ⎡         α²    ⎤
 *    Fg = ―――― ⋅ acos ⎢1 - ―――――――――――⎥
 *          2π         ⎣     2⋅(1 - α) ⎦
 *
 *
 * Or, α can be calculated from the required cutoff frequency:
 *
 *     α = cosω - 1 + √(cos²ω - 4⋅cosω + 3)
 *
 *
 * This could be used for correcting the frequency warping, like it is
 * done with the bilinear transform.
 *
 * HOWEVER!
 *
 * It is NOT done here, because:
 *
 *   - PTx filters have poor performance when cutoff frequency Fc is higher than Fs/10
 *   - Pre-warping would limit the maximum cutoff to around 83% of Nyquist
 *   - Nobody is using pre-warping with PT filters
 *   - It would be surprising for developers
 *
 */

float pt1FilterGain(float cutoff, float sampleRate)
{
    cutoff = limitCutoff(cutoff, sampleRate);

    float gamma = M_1_2PIf * sampleRate;
    float alpha = cutoff / (cutoff + gamma);

    return fminf(alpha, 1.0f);
}

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->gain = pt1FilterGain(cutoff, sampleRate);
}

void pt1FilterInitGain(pt1Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->gain = gain;
}

void pt1FilterUpdate(pt1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt1FilterGain(cutoff, sampleRate);
}

void pt1FilterUpdateGain(pt1Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->y1 += (input - filter->y1) * filter->gain;
    return filter->y1;
}


// PT2 Low Pass filter

float pt2FilterGain(float cutoff, float sampleRate)
{
    // order=2: 1 / sqrt( (2^(1 / order) - 1)) = 1.553773974
    return pt1FilterGain(cutoff * 1.553773974f, sampleRate);
}

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->gain = pt2FilterGain(cutoff, sampleRate);
}

void pt2FilterInitGain(pt2Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->gain = gain;
}

void pt2FilterUpdate(pt2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt2FilterGain(cutoff, sampleRate);
}

void pt2FilterUpdateGain(pt2Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->y2 += (input      - filter->y2) * filter->gain;
    filter->y1 += (filter->y2 - filter->y1) * filter->gain;
    return filter->y1;
}


// PT3 Low Pass filter

float pt3FilterGain(float cutoff, float sampleRate)
{
    // order=3: 1 / sqrt( (2^(1 / order) - 1)) = 1.961459177
    return pt1FilterGain(cutoff * 1.961459177f, sampleRate);
}

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->gain = pt3FilterGain(cutoff, sampleRate);
}

void pt3FilterInitGain(pt3Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->gain = gain;
}

void pt3FilterUpdate(pt3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt3FilterGain(cutoff, sampleRate);
}

void pt3FilterUpdateGain(pt3Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt3FilterApply(pt3Filter_t *filter, float input)
{
    filter->y3 += (input      - filter->y3) * filter->gain;
    filter->y2 += (filter->y3 - filter->y2) * filter->gain;
    filter->y1 += (filter->y2 - filter->y1) * filter->gain;
    return filter->y1;
}


// EWMA1 Low Pass filter

float ewma1FilterWeight(float cutoff, float sampleRate)
{
    cutoff = limitCutoff(cutoff, sampleRate);

    float gamma = M_1_2PIf * sampleRate;
    float weight = (cutoff + gamma) / cutoff;

    return fmaxf(weight, 1.0f);
}

void ewma1FilterInit(ewma1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->N = 0;
    filter->W = ewma1FilterWeight(cutoff, sampleRate);
}

void ewma1FilterInitWeight(ewma1Filter_t *filter, float weight)
{
    filter->y1 = 0;
    filter->N = 0;
    filter->W = weight;
}

void ewma1FilterUpdate(ewma1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->W = ewma1FilterWeight(cutoff, sampleRate);
    filter->N = MIN(filter->N, filter->W);
}

void ewma1FilterUpdateWeight(ewma1Filter_t *filter, float weight)
{
    filter->W = weight;
    filter->N = MIN(filter->N, filter->W);
}

FAST_CODE float ewma1FilterApply(ewma1Filter_t *filter, float input)
{
    uint32_t count = filter->N + 1;
    float weight = filter->W;

    if (count < weight)
        weight = filter->N = count;

    filter->y1 += (input - filter->y1) / weight;

    return filter->y1;
}


// EWMA2 Low Pass filter

float ewma2FilterWeight(float cutoff, float sampleRate)
{
    // order=2: 1 / sqrt( (2^(1 / order) - 1)) = 1.553773974
    return ewma1FilterWeight(cutoff * 1.553773974f, sampleRate);
}

void ewma2FilterInit(ewma2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->N = 0;
    filter->W = ewma2FilterWeight(cutoff, sampleRate);
}

void ewma2FilterInitWeight(ewma2Filter_t *filter, float weight)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->N = 0;
    filter->W = weight;
}

void ewma2FilterUpdate(ewma2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->W = ewma2FilterWeight(cutoff, sampleRate);
    filter->N = MIN(filter->N, filter->W);
}

void ewma2FilterUpdateWeight(ewma2Filter_t *filter, float weight)
{
    filter->W = weight;
    filter->N = MIN(filter->N, filter->W);
}

FAST_CODE float ewma2FilterApply(ewma2Filter_t *filter, float input)
{
    uint32_t count = filter->N + 1;
    float weight = filter->W;

    if (count < weight)
        weight = filter->N = count;

    filter->y2 += (input      - filter->y2) / weight;
    filter->y1 += (filter->y2 - filter->y1) / weight;

    return filter->y1;
}


// EWMA3 Low Pass filter

float ewma3FilterWeight(float cutoff, float sampleRate)
{
    // order=3: 1 / sqrt( (2^(1 / order) - 1)) = 1.961459177
    return ewma1FilterWeight(cutoff * 1.961459177f, sampleRate);
}

void ewma3FilterInit(ewma3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->N = 0;
    filter->W = ewma3FilterWeight(cutoff, sampleRate);
}

void ewma3FilterInitWeight(ewma3Filter_t *filter, float weight)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->N = 0;
    filter->W = weight;
}

void ewma3FilterUpdate(ewma3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->W = ewma3FilterWeight(cutoff, sampleRate);
    filter->N = MIN(filter->N, filter->W);
}

void ewma3FilterUpdateWeight(ewma3Filter_t *filter, float weight)
{
    filter->W = weight;
    filter->N = MIN(filter->N, filter->W);
}

FAST_CODE float ewma3FilterApply(ewma3Filter_t *filter, float input)
{
    uint32_t count = filter->N + 1;
    float weight = filter->W;

    if (count < weight)
        weight = filter->N = count;

    filter->y3 += (input      - filter->y3) / weight;
    filter->y2 += (filter->y3 - filter->y2) / weight;
    filter->y1 += (filter->y2 - filter->y1) / weight;

    return filter->y1;
}


/*
 * Differentiator with bandwidth limit
 *
 *   Fc = Cutoff frequency
 *   Fs = Sampling frequency
 *
 *   Wc = 2⋅π⋅Fc
 *
 *                Wc          Wc
 *  H(s) = s ⋅ ―――――――― = ――――――――――――
 *              s + Wc     1 + Wc⋅s⁻¹
 *
 *
 * Apply bilinear transform:
 *
 *          b₀ + b₁⋅z⁻¹
 *  H(z) = ―――――――――――――
 *          a₀ + a₁⋅z⁻¹
 *
 * Where
 *      b₀ = Wc / (1 + K)
 *      b₁ = -b₀
 *      a₀ = 1
 *      a₁ = (1 - K) / (1 + K)
 *
 * And
 *       K = tan(π⋅Fc/Fs)
 *
 */

void difFilterInit(difFilter_t *filter, float cutoff, float sampleRate)
{
    filter->x1 = 0;
    filter->y1 = 0;

    difFilterUpdate(filter, cutoff, sampleRate);
}

void difFilterUpdate(difFilter_t *filter, float cutoff, float sampleRate)
{
    cutoff = limitCutoff(cutoff, sampleRate);

    const float K = tan_approx(M_PIf * cutoff / sampleRate);
    const float Wc = M_2PIf * cutoff;

    filter->a = (K - 1) / (K + 1);
    filter->b = Wc / (K + 1);
}

FAST_CODE float difFilterApply(difFilter_t *filter, float input)
{
    const float output =
        filter->b * input -
        filter->b * filter->x1 -
        filter->a * filter->y1;

    filter->y1 = output;
    filter->x1 = input;

    return output;
}

/*
 * Bilinear (trapezoidal) Integrator
 *
 *  Fs = Sampling frequency
 *
 *          1
 *  H(s) = ―――
 *          s
 *
 * Apply bilinear transform:
 *
 *          b₀ + b₁⋅z⁻¹        1       1 + z⁻¹
 *  H(z) = ―――――――――――――  =  ―――――― ⋅ ―――――――――
 *          a₀ + a₁⋅z⁻¹       2⋅Fs     1 - z⁻¹
 *
 * Where
 *      b₀ = 1/(2⋅Fs)
 *      b₁ = b₀
 *      a₀ = 1
 *      a₁ = 1
 */

void intFilterInit(intFilter_t *filter, float sampleRate, float min, float max)
{
    intFilterReset(filter);
    intFilterUpdate(filter, sampleRate, min, max);
}

void intFilterReset(intFilter_t *filter)
{
    filter->x1 = 0;
    filter->y1 = 0;
}

void intFilterUpdate(intFilter_t *filter, float sampleRate, float min, float max)
{
    filter->min = min;
    filter->max = max;
    filter->gain = 1.0f / (2 * sampleRate);
}

FAST_CODE float intFilterApply(intFilter_t *filter, float input)
{
    float output = filter->y1 + (input + filter->x1) * filter->gain;

    output = constrainf(output, filter->min, filter->max);

    filter->x1 = input;
    filter->y1 = output;

    return output;
}


// BiQuad filter a.k.a. Second-Order-Section

void biquadFilterInit(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType)
{
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;

    biquadFilterUpdate(filter, cutoff, sampleRate, Q, filterType);
}

FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType)
{
    cutoff = limitCutoff(cutoff, sampleRate);

    const float omega = M_2PIf * cutoff / sampleRate;
    const float sinom = sin_approx(omega);
    const float cosom = cos_approx(omega);
    const float alpha = sinom / (2 * Q);

    switch (filterType) {
        case BIQUAD_LPF:
            filter->b1 = 1 - cosom;
            filter->b0 = filter->b1 / 2;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_HPF:
            filter->b0 = (1 + cosom) / 2;
            filter->b1 = -1 - cosom;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_BPF:
            filter->b0 = alpha;
            filter->b1 = 0;
            filter->b2 = -alpha;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_NOTCH:
            filter->b0 = 1;
            filter->b1 = -2 * cosom;
            filter->b2 = 1;
            filter->a1 = filter->b1;
            filter->a2 = 1 - alpha;
            break;
    }

    const float a0 = 1 + alpha;

    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;
}


FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    const float output =
        filter->b0 * input +
        filter->b1 * filter->x1 +
        filter->b2 * filter->x2 -
        filter->a1 * filter->y1 -
        filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

FAST_CODE float biquadFilterApplyTF2(biquadFilter_t *filter, float input)
{
    const float output = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * output + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * output;

    filter->y1 = output;

    return output;
}

FAST_CODE float filterStackApply(biquadFilter_t *filter, float input, int count)
{
    for (int i = 0; i < count; i++, filter++) {
        const float output = filter->b0 * input + filter->x1;

        filter->x1 = filter->b1 * input - filter->a1 * output + filter->x2;
        filter->x2 = filter->b2 * input - filter->a2 * output;

        input = output;
    }

    return input;
}


// First order filter

void firstOrderFilterInit(order1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->x1 = 0;
    filter->y1 = 0;

    firstOrderFilterUpdate(filter, cutoff, sampleRate);
}

FAST_CODE void firstOrderFilterUpdate(order1Filter_t *filter, float cutoff, float sampleRate)
{
    cutoff = limitCutoff(cutoff, sampleRate);

    const float W = tan_approx(M_PIf * cutoff / sampleRate);

    filter->a1 = (W - 1) / (W + 1);
    filter->b1 = filter->b0 = W / (W + 1);
}

FAST_CODE float firstOrderFilterApplyDF1(order1Filter_t *filter, float input)
{
    const float output =
        filter->b0 * input +
        filter->b1 * filter->x1 -
        filter->a1 * filter->y1;

    filter->x1 = input;
    filter->y1 = output;

    return output;
}

FAST_CODE float firstOrderFilterApplyTF2(order1Filter_t *filter, float input)
{
    const float output = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * output;
    filter->y1 = output;

    return output;
}


// Generic Low-Pass Filter (LPF)

static void biquadButterLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, BUTTER_C * cutoff, sampleRate, BUTTER_Q, BIQUAD_LPF);
}

static void biquadBesselLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, BESSEL_C * cutoff, sampleRate, BESSEL_Q, BIQUAD_LPF);
}

static void biquadDampedLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, DAMPED_C * cutoff, sampleRate, DAMPED_Q, BIQUAD_LPF);
}

static void biquadButterLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, BUTTER_C * cutoff, sampleRate, BUTTER_Q, BIQUAD_LPF);
}

static void biquadBesselLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, BESSEL_C * cutoff, sampleRate, BESSEL_Q, BIQUAD_LPF);
}

static void biquadDampedLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, DAMPED_C * cutoff, sampleRate, DAMPED_Q, BIQUAD_LPF);
}

void lowpassFilterInit(filter_t *filter, uint8_t type, float cutoff, float sampleRate, uint32_t flags)
{
    if (cutoff == 0 || sampleRate == 0)
        type = LPF_NONE;

    switch (type) {
        case LPF_PT1:
            if (flags & LPF_EWMA) {
                filter->init   = (filterInitFn)ewma1FilterInit;
                filter->apply  = (filterApplyFn)ewma1FilterApply;
                filter->update = (filterUpdateFn)ewma1FilterUpdate;
            } else {
                filter->init   = (filterInitFn)pt1FilterInit;
                filter->apply  = (filterApplyFn)pt1FilterApply;
                filter->update = (filterUpdateFn)pt1FilterUpdate;
            }
            break;

        case LPF_PT2:
            if (flags & LPF_EWMA) {
                filter->init   = (filterInitFn)ewma2FilterInit;
                filter->apply  = (filterApplyFn)ewma2FilterApply;
                filter->update = (filterUpdateFn)ewma2FilterUpdate;
            } else {
                filter->init   = (filterInitFn)pt2FilterInit;
                filter->apply  = (filterApplyFn)pt2FilterApply;
                filter->update = (filterUpdateFn)pt2FilterUpdate;
            }
            break;

        case LPF_PT3:
            if (flags & LPF_EWMA) {
                filter->init   = (filterInitFn)ewma3FilterInit;
                filter->apply  = (filterApplyFn)ewma3FilterApply;
                filter->update = (filterUpdateFn)ewma3FilterUpdate;
            } else {
                filter->init   = (filterInitFn)pt3FilterInit;
                filter->apply  = (filterApplyFn)pt3FilterApply;
                filter->update = (filterUpdateFn)pt3FilterUpdate;
            }
            break;

        case LPF_1ST_ORDER:
        case LPF_ORDER1:
            filter->init   = (filterInitFn)firstOrderFilterInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)firstOrderFilterApplyDF1;
                filter->update = (filterUpdateFn)firstOrderFilterUpdate;
            }
            else {
                filter->apply = (filterApplyFn)firstOrderFilterApplyTF2;
                filter->update = (filterUpdateFn)nilFilterUpdate;
            }
            break;

        case LPF_BUTTER:
            filter->init   = (filterInitFn)biquadButterLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)biquadButterLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyTF2;
                filter->update = (filterUpdateFn)nilFilterUpdate;
            }
            break;

        case LPF_2ND_ORDER:
        case LPF_BESSEL:
            filter->init   = (filterInitFn)biquadBesselLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)biquadBesselLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyTF2;
                filter->update = (filterUpdateFn)nilFilterUpdate;
            }
            break;

        case LPF_DAMPED:
            filter->init   = (filterInitFn)biquadDampedLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)biquadDampedLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyTF2;
                filter->update = (filterUpdateFn)nilFilterUpdate;
            }
            break;

        default:
            filter->init   = (filterInitFn)nilFilterInit;
            filter->apply  = (filterApplyFn)nilFilterApply;
            filter->update = (filterUpdateFn)nilFilterUpdate;
            break;
    }

    filterInit(filter, cutoff, sampleRate);
}


void notchFilterInit(filter_t *filter, float cutoff, float Q, float sampleRate, uint32_t flags)
{
    filter->init   = (filterInitFn)nilFilterInit;
    filter->update = (filterUpdateFn)nilFilterUpdate;
    filter->apply  = (filterApplyFn)nilFilterApply;

    if (cutoff > 0 && Q > 0) {
        if (flags & LPF_UPDATE)
            filter->apply = (filterApplyFn)biquadFilterApplyDF1;
        else
            filter->apply = (filterApplyFn)biquadFilterApplyTF2;

        biquadFilterInit(&filter->data.sos, cutoff, sampleRate, Q, BIQUAD_NOTCH);
    }
}

void notchFilterUpdate(filter_t *filter, float cutoff, float Q, float sampleRate)
{
    if (cutoff > 0 && Q > 0)
        biquadFilterUpdate(&filter->data.sos, cutoff, sampleRate, Q, BIQUAD_NOTCH);
}

// Get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float notchFilterGetQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}


// Simple fixed-point lowpass filter based on integer math

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal)
{
    filter->fp = (filter->fp << filter->beta) - filter->fp;
    filter->fp += newVal << filter->fpShift;
    filter->fp >>= filter->beta;
    return filter->fp >> filter->fpShift;
}

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift)
{
    filter->fp = 0;
    filter->beta = beta;
    filter->fpShift = fpShift;
}
