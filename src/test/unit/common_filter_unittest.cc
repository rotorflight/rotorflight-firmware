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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <functional>

extern "C" {
#include "common/filter.h"
#include "common/maths.h"
}

#include "gtest/gtest.h"

static constexpr float FS = 1000.0f;     // sample rate used throughout
static constexpr float FC = 20.0f;       // lowpass cutoff used throughout

using FilterFn = std::function<float(float)>;

// Steady-state gain of a filter for a sine at freq. The amplitude is taken by
// correlating the output with sin/cos over whole periods, since the sample
// peak misses the crest when there are only a few samples per period.
static float sineGain(const FilterFn &apply, float freq, float fs = FS)
{
    const int settle = 4000;
    const int periods = (int)ceilf(freq);              // 1 s worth of periods...
    const int measure = (int)lroundf(periods * fs / freq);   // ...in whole samples
    double si = 0, co = 0;
    for (int i = 0; i < settle + measure; i++) {
        const double ph = 2 * M_PI * freq * i / fs;
        const float y = apply((float)sin(ph));
        if (i >= settle) {
            si += y * sin(ph);
            co += y * cos(ph);
        }
    }
    return (float)(2 * sqrt(si * si + co * co) / measure);
}

// Output after a long run of constant input
static float dcGain(const FilterFn &apply)
{
    float y = 0;
    for (int i = 0; i < 5000; i++)
        y = apply(1.0f);
    return y;
}

static FilterFn wrap(filter_t *f)
{
    return [f](float x) { return filterApply(f, x); };
}

/** Gain helpers **/

TEST(FilterGain, Pt1)
{
    // alpha = fc / (fc + fs / 2pi)
    EXPECT_NEAR(FC / (FC + FS / (2 * M_PIf)), pt1FilterGain(FC, FS), 1e-6f);

    // Disabled filter passes through
    EXPECT_EQ(1.0f, pt1FilterGain(0, FS));
    EXPECT_EQ(1.0f, pt1FilterGain(FC, 0));

    // Cutoff is limited to 95% of Nyquist
    EXPECT_EQ(pt1FilterGain(0.475f * FS, FS), pt1FilterGain(10 * FS, FS));
}

TEST(FilterGain, Pt2AndPt3AreCutoffCorrected)
{
    EXPECT_FLOAT_EQ(pt1FilterGain(FC * 1.553773974f, FS), pt2FilterGain(FC, FS));
    EXPECT_GT(pt3FilterGain(FC, FS), pt2FilterGain(FC, FS));
}

TEST(FilterGain, NotchQ)
{
    // Q = f0 * f1 / (f0^2 - f1^2)
    EXPECT_NEAR(200.0f * 160 / (200 * 200 - 160 * 160), notchFilterGetQ(200, 160), 1e-6f);

    EXPECT_EQ(0, notchFilterGetQ(0, 160));
    EXPECT_EQ(0, notchFilterGetQ(200, 0));
    EXPECT_EQ(0, notchFilterGetQ(200, 200));
    EXPECT_EQ(0, notchFilterGetQ(200, 250));
}

/** Lowpass filters through the generic interface **/

struct LowpassCase {
    const char *name;
    uint8_t type;
    uint32_t flags;
    int order;
    float cutoffTol;    // allowed deviation from -3 dB at the cutoff
    bool updatable;
};

class LowpassTest : public ::testing::TestWithParam<LowpassCase> {
  protected:
    void SetUp() override
    {
        lowpassFilterInit(&filter, GetParam().type, FC, FS, GetParam().flags);
    }
    filter_t filter;
};

TEST_P(LowpassTest, UnityDcGain)
{
    EXPECT_NEAR(1.0f, dcGain(wrap(&filter)), 1e-3f);
}

TEST_P(LowpassTest, PassesLowFrequencies)
{
    EXPECT_NEAR(1.0f, sineGain(wrap(&filter), FC / 10), 0.03f);
}

TEST_P(LowpassTest, ThreeDbAtCutoff)
{
    EXPECT_NEAR(M_SQRT1_2, sineGain(wrap(&filter), FC), GetParam().cutoffTol);
}

TEST_P(LowpassTest, AttenuatesAboveCutoff)
{
    // At 10x the cutoff
    static const float limit[] = { 0, 0.15f, 0.03f, 0.01f };
    EXPECT_LT(sineGain(wrap(&filter), 10 * FC), limit[GetParam().order]);
}

TEST_P(LowpassTest, CutoffUpdate)
{
    filter_t fresh;
    lowpassFilterInit(&fresh, GetParam().type, GetParam().updatable ? 4 * FC : FC, FS, GetParam().flags);

    // Updatable filters take the new cutoff, static ones keep the old one
    filterUpdate(&filter, 4 * FC, FS);
    for (int i = 0; i < 200; i++) {
        const float x = sinf(i * 0.37f) + ((i / 40) % 2);
        EXPECT_FLOAT_EQ(filterApply(&fresh, x), filterApply(&filter, x)) << "sample " << i;
    }
}

// The PT types derive their gain from the continuous RC formula, so a cascade
// lands a little below -3 dB at the cutoff (at fc/fs = 0.02: PT1 -3.3 dB, PT3 -3.7 dB).
INSTANTIATE_TEST_SUITE_P(
    Filters, LowpassTest,
    ::testing::Values(
        LowpassCase{ "PT1",            LPF_PT1,       0,          1, 0.03f, true  },
        LowpassCase{ "PT1_EWMA",       LPF_PT1,       LPF_EWMA,   1, 0.03f, true  },
        LowpassCase{ "PT2",            LPF_PT2,       0,          2, 0.05f, true  },
        LowpassCase{ "PT2_EWMA",       LPF_PT2,       LPF_EWMA,   2, 0.05f, true  },
        LowpassCase{ "PT3",            LPF_PT3,       0,          3, 0.06f, true  },
        LowpassCase{ "PT3_EWMA",       LPF_PT3,       LPF_EWMA,   3, 0.06f, true  },
        LowpassCase{ "ORDER1",         LPF_ORDER1,    0,          1, 0.02f, false },
        LowpassCase{ "ORDER1_UPDATE",  LPF_ORDER1,    LPF_UPDATE, 1, 0.02f, true  },
        LowpassCase{ "BUTTER",         LPF_BUTTER,    0,          2, 0.02f, false },
        LowpassCase{ "BUTTER_UPDATE",  LPF_BUTTER,    LPF_UPDATE, 2, 0.02f, true  },
        LowpassCase{ "BESSEL",         LPF_BESSEL,    0,          2, 0.02f, false },
        LowpassCase{ "BESSEL_UPDATE",  LPF_BESSEL,    LPF_UPDATE, 2, 0.02f, true  },
        LowpassCase{ "DAMPED",         LPF_DAMPED,    0,          2, 0.02f, false },
        LowpassCase{ "DAMPED_UPDATE",  LPF_DAMPED,    LPF_UPDATE, 2, 0.02f, true  }
    ),
    [](const ::testing::TestParamInfo<LowpassCase> &info) { return std::string(info.param.name); });

TEST(Lowpass, NoneIsPassthrough)
{
    filter_t filter;
    lowpassFilterInit(&filter, LPF_NONE, FC, FS, 0);

    EXPECT_EQ(0.3f, filterApply(&filter, 0.3f));
    EXPECT_EQ(-2.0f, filterApply(&filter, -2.0f));
    EXPECT_EQ(-2.0f, filterOutput(&filter));
}

TEST(Lowpass, DefaultTypesMapToPrimaryFilters)
{
    filter_t first, order1, second, bessel;
    lowpassFilterInit(&first,  LPF_1ST_ORDER, FC, FS, 0);
    lowpassFilterInit(&order1, LPF_ORDER1,    FC, FS, 0);
    lowpassFilterInit(&second, LPF_2ND_ORDER, FC, FS, 0);
    lowpassFilterInit(&bessel, LPF_BESSEL,    FC, FS, 0);

    for (int i = 0; i < 100; i++) {
        const float x = sinf(i * 0.3f) + (i % 7) * 0.1f;
        EXPECT_EQ(filterApply(&order1, x), filterApply(&first, x));
        EXPECT_EQ(filterApply(&bessel, x), filterApply(&second, x));
    }
}

TEST(Lowpass, UninitialisedFilterIsPassthrough)
{
    filter_t filter = {};
    EXPECT_EQ(1.5f, filterApply(&filter, 1.5f));
}

TEST(Lowpass, Pt1StepResponse)
{
    // y[n] = 1 - (1 - alpha)^n for a unit step
    pt1Filter_t filter;
    pt1FilterInit(&filter, FC, FS);
    const float alpha = pt1FilterGain(FC, FS);

    float y = 0;
    for (int n = 1; n <= 50; n++) {
        y = pt1FilterApply(&filter, 1.0f);
        EXPECT_NEAR(1 - powf(1 - alpha, n), y, 1e-5f) << "sample " << n;
    }
    EXPECT_EQ(y, pt1FilterOutput(&filter));
}

/** Highpass **/

TEST(Highpass, FirstOrderBlocksDc)
{
    order1Filter_t filter;
    firstOrderHPFInit(&filter, FC, FS);
    auto apply = [&](float x) { return firstOrderFilterApply(&filter, x); };

    EXPECT_NEAR(0, dcGain(apply), 1e-3f);
    EXPECT_NEAR(1.0f, sineGain(apply, 10 * FC), 0.03f);
    EXPECT_NEAR(M_SQRT1_2, sineGain(apply, FC), 0.05f);
}

/** Notch **/

TEST(Notch, RejectsCenterPassesOthers)
{
    const float center = 100, Q = notchFilterGetQ(100, 80);

    for (uint32_t flags : { 0u, (uint32_t)LPF_UPDATE }) {
        filter_t filter;
        notchFilterInit(&filter, center, Q, FS, flags);

        EXPECT_LT(sineGain(wrap(&filter), center), 0.02f) << "flags " << flags;
        EXPECT_NEAR(1.0f, sineGain(wrap(&filter), center / 10), 0.02f) << "flags " << flags;
        EXPECT_NEAR(1.0f, sineGain(wrap(&filter), center * 4), 0.05f) << "flags " << flags;
        EXPECT_NEAR(1.0f, dcGain(wrap(&filter)), 1e-3f) << "flags " << flags;
    }
}

TEST(Notch, Update)
{
    filter_t filter;
    notchFilterInit(&filter, 100, 3, FS, LPF_UPDATE);
    notchFilterUpdate(&filter, 150, 3, FS);

    EXPECT_LT(sineGain(wrap(&filter), 150), 0.02f);
    EXPECT_GT(sineGain(wrap(&filter), 100), 0.5f);
}

/** Biquad sections **/

TEST(Biquad, BandpassPeaksAtCenter)
{
    biquadFilter_t filter;
    biquadFilterInit(&filter, 100, FS, 2, BIQUAD_BPF);
    auto apply = [&](float x) { return biquadFilterApplyDF1(&filter, x); };

    EXPECT_NEAR(1.0f, sineGain(apply, 100), 0.03f);
    EXPECT_LT(sineGain(apply, 10), 0.1f);
    EXPECT_NEAR(0, dcGain(apply), 1e-3f);
}

TEST(Biquad, HighpassBlocksDc)
{
    biquadFilter_t filter;
    biquadFilterInit(&filter, FC, FS, BUTTER_Q, BIQUAD_HPF);
    auto apply = [&](float x) { return biquadFilterApplyDF1(&filter, x); };

    EXPECT_NEAR(0, dcGain(apply), 1e-3f);
    EXPECT_NEAR(1.0f, sineGain(apply, 10 * FC), 0.03f);
}

TEST(Biquad, DirectFormsAgree)
{
    biquadFilter_t df1, df2;
    biquadFilterInit(&df1, FC, FS, BUTTER_Q, BIQUAD_LPF);
    biquadFilterInit(&df2, FC, FS, BUTTER_Q, BIQUAD_LPF);

    for (int i = 0; i < 500; i++) {
        const float x = sinf(i * 0.1f) + ((i / 50) % 2);
        EXPECT_NEAR(biquadFilterApplyDF1(&df1, x), biquadFilterApplyTF2(&df2, x), 1e-4f) << "sample " << i;
    }
}

TEST(Biquad, StackAppliesInSeries)
{
    biquadFilter_t stack[2], a, b;
    biquadFilterInit(&stack[0], FC, FS, BUTTER_4A_Q, BIQUAD_LPF);
    biquadFilterInit(&stack[1], FC, FS, BUTTER_4B_Q, BIQUAD_LPF);
    biquadFilterInit(&a, FC, FS, BUTTER_4A_Q, BIQUAD_LPF);
    biquadFilterInit(&b, FC, FS, BUTTER_4B_Q, BIQUAD_LPF);

    for (int i = 0; i < 200; i++) {
        const float x = sinf(i * 0.2f);
        const float expected = biquadFilterApplyTF2(&b, biquadFilterApplyTF2(&a, x));
        EXPECT_NEAR(expected, filterStackApply(stack, x, 2), 1e-5f);
    }
}

/** Differentiator and integrator **/

TEST(Differentiator, RampSlope)
{
    difFilter_t filter;
    difFilterInit(&filter, 50, FS);

    // Ramp of 100 units/s
    float y = 0;
    for (int i = 0; i < 2000; i++)
        y = difFilterApply(&filter, 100.0f * i / FS);
    EXPECT_NEAR(100.0f, y, 0.5f);
    EXPECT_EQ(y, difFilterOutput(&filter));

    // Constant input settles to zero
    for (int i = 0; i < 2000; i++)
        y = difFilterApply(&filter, 3.0f);
    EXPECT_NEAR(0, y, 1e-3f);
}

TEST(Differentiator, Disabled)
{
    difFilter_t filter;
    difFilterInit(&filter, 0, FS);
    EXPECT_EQ(0, difFilterApply(&filter, 1.0f));
    EXPECT_EQ(0, difFilterApply(&filter, 5.0f));
}

TEST(Integrator, TrapezoidalSum)
{
    intFilter_t filter;
    intFilterInit(&filter, FS, -10, 10);

    // y[n] = y[n-1] + (x[n] + x[n-1]) / (2 Fs)
    float y = 0;
    for (int i = 0; i < 1000; i++)
        y = intFilterApply(&filter, 1.0f);
    EXPECT_NEAR(1999.0f / 2000.0f, y, 1e-4f);
    EXPECT_EQ(y, intFilterOutput(&filter));

    intFilterReset(&filter);
    EXPECT_EQ(0, intFilterOutput(&filter));
}

TEST(Integrator, Limits)
{
    intFilter_t filter;
    intFilterInit(&filter, FS, -0.5f, 0.25f);

    float y = 0;
    for (int i = 0; i < 1000; i++)
        y = intFilterApply(&filter, 1.0f);
    EXPECT_EQ(0.25f, y);

    for (int i = 0; i < 2000; i++)
        y = intFilterApply(&filter, -1.0f);
    EXPECT_EQ(-0.5f, y);
}

/** Peak filter **/

TEST(Peak, FastAttackSlowRelease)
{
    peakFilter_t filter;
    peakFilterInit(&filter, 100, 1, FS);

    float y = 0;
    for (int i = 0; i < 50; i++)
        y = peakFilterApply(&filter, 1.0f);
    EXPECT_GT(y, 0.95f);

    for (int i = 0; i < 50; i++)
        y = peakFilterApply(&filter, 0.0f);
    EXPECT_GT(y, 0.7f);
    EXPECT_EQ(y, peakFilterOutput(&filter));
}

/** Fixed-point lowpass **/

TEST(SimpleLowpass, ConvergesToInput)
{
    simpleLowpassFilter_t filter;
    simpleLPFilterInit(&filter, 3, 8);

    int32_t y = 0;
    for (int i = 0; i < 200; i++)
        y = simpleLPFilterUpdate(&filter, 1000);
    EXPECT_NEAR(1000, y, 1);

    for (int i = 0; i < 200; i++)
        y = simpleLPFilterUpdate(&filter, -500);
    EXPECT_NEAR(-500, y, 1);
}
