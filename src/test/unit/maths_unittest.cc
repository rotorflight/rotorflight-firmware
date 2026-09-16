/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#include <math.h>

#include <cstdio>

#define USE_BARO

extern "C" {
    #include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(MathsUnittest, TestScaleRange)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, 0, 10, 0, 100), 0);
    EXPECT_EQ(scaleRange(10, 0, 10, 0, 100), 100);
    EXPECT_EQ(scaleRange(0, 0, 100, 0, 10), 0);
    EXPECT_EQ(scaleRange(100, 0, 100, 0, 10), 10);

    // Scale up
    EXPECT_EQ(scaleRange(1, 0, 10, 0, 100), 10);
    EXPECT_EQ(scaleRange(2, 0, 10, 0, 100), 20);
    EXPECT_EQ(scaleRange(5, 0, 10, 0, 100), 50);

    // Scale down
    EXPECT_EQ(scaleRange(10, 0, 100, 0, 10), 1);
    EXPECT_EQ(scaleRange(20, 0, 100, 0, 10), 2);
    EXPECT_EQ(scaleRange(50, 0, 100, 0, 10), 5);
}

TEST(MathsUnittest, TestScaleRangeNegatives)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, -10, 0, -100, 0), 0);
    EXPECT_EQ(scaleRange(-10, -10, 0, -100, 0), -100);
    EXPECT_EQ(scaleRange(0, -100, 0, -10, 0), 0);
    EXPECT_EQ(scaleRange(-100, -100, 0, -10, 0), -10);

    // Scale up
    EXPECT_EQ(scaleRange(-1, -10, 0, -100, 0), -10);
    EXPECT_EQ(scaleRange(-2, -10, 0, -100, 0), -20);
    EXPECT_EQ(scaleRange(-5, -10, 0, -100, 0), -50);

    // Scale down
    EXPECT_EQ(scaleRange(-10, -100, 0, -10, 0), -1);
    EXPECT_EQ(scaleRange(-20, -100, 0, -10, 0), -2);
    EXPECT_EQ(scaleRange(-50, -100, 0, -10, 0), -5);
}

TEST(MathsUnittest, TestScaleRangeNegativePositive)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, -10, 0, 0, 100), 100);
    EXPECT_EQ(scaleRange(-10, -10, 0, 0, 100), 0);
    EXPECT_EQ(scaleRange(0, -100, 0, 0, 10), 10);
    EXPECT_EQ(scaleRange(-100, -100, 0, 0, 10), 0);

    // Scale up
    EXPECT_EQ(scaleRange(-1, -10, 0, 0, 100), 90);
    EXPECT_EQ(scaleRange(-2, -10, 0, 0, 100), 80);
    EXPECT_EQ(scaleRange(-5, -10, 0, 0, 100), 50);

    // Scale down
    EXPECT_EQ(scaleRange(-10, -100, 0, 0, 10), 9);
    EXPECT_EQ(scaleRange(-20, -100, 0, 0, 10), 8);
    EXPECT_EQ(scaleRange(-50, -100, 0, 0, 10), 5);
}

TEST(MathsUnittest, TestScaleRangeReverse)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, 0, 10, 100, 0), 100);
    EXPECT_EQ(scaleRange(10, 0, 10, 100, 0), 0);
    EXPECT_EQ(scaleRange(0, 0, 100, 10, 0), 10);
    EXPECT_EQ(scaleRange(100, 0, 100, 10, 0), 0);

    // Scale up
    EXPECT_EQ(scaleRange(1, 0, 10, 100, 0), 90);
    EXPECT_EQ(scaleRange(2, 0, 10, 100, 0), 80);
    EXPECT_EQ(scaleRange(5, 0, 10, 100, 0), 50);

    // Scale down
    EXPECT_EQ(scaleRange(10, 0, 100, 10, 0), 9);
    EXPECT_EQ(scaleRange(20, 0, 100, 10, 0), 8);
    EXPECT_EQ(scaleRange(50, 0, 100, 10, 0), 5);
}

TEST(MathsUnittest, TestConstrain)
{
    // Within bounds
    EXPECT_EQ(constrain(0, 0, 0), 0);
    EXPECT_EQ(constrain(1, 1, 1), 1);
    EXPECT_EQ(constrain(1, 0, 2), 1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(1, 1, 2), 1);
    // Equal to top bound.
    EXPECT_EQ(constrain(1, 0, 1), 1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(1, 1, 1), 1);

    // Above top bound.
    EXPECT_EQ(constrain(2, 0, 1), 1);
    // Below bottom bound.
    EXPECT_EQ(constrain(0, 1, 2), 1);
}

TEST(MathsUnittest, TestConstrainNegatives)
{
    // Within bounds.
    EXPECT_EQ(constrain(-1, -1, -1), -1);
    EXPECT_EQ(constrain(-1, -2, 0), -1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(-1, -1, 0), -1);
    // Equal to top bound.
    EXPECT_EQ(constrain(-1, -2, -1), -1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(-1, -1, -1), -1);

    // Above top bound.
    EXPECT_EQ(constrain(-1, -3, -2), -2);
    // Below bottom bound.
    EXPECT_EQ(constrain(-3, -2, -1), -2);
}

TEST(MathsUnittest, TestConstrainf)
{
    // Within bounds.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 2.0f), 1.0f);

    // Equal to bottom bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 2.0f), 1.0f);
    // Equal to top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 1.0f), 1.0f);

    // Equal to both bottom and top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 1.0f), 1.0f);

    // Above top bound.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bottom bound.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);

    // Above bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);
}

TEST(MathsUnittest, TestDegreesToRadians)
{
    EXPECT_FLOAT_EQ(degreesToRadians(0), 0.0f);
    EXPECT_FLOAT_EQ(degreesToRadians(90), 0.5f * M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(180), M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(-180), - M_PIf);
}

TEST(MathsUnittest, TestApplyDeadband)
{
    EXPECT_EQ(applyDeadband(0, 0), 0);
    EXPECT_EQ(applyDeadband(1, 0), 1);
    EXPECT_EQ(applyDeadband(-1, 0), -1);

    EXPECT_EQ(applyDeadband(0, 10), 0);
    EXPECT_EQ(applyDeadband(1, 10), 0);
    EXPECT_EQ(applyDeadband(10, 10), 0);

    EXPECT_EQ(applyDeadband(11, 10), 1);
    EXPECT_EQ(applyDeadband(-11, 10), -1);
}

void expectVectorsAreEqual(struct fp_vector *a, struct fp_vector *b, float absTol)
{
    EXPECT_NEAR(a->X, b->X, absTol);
    EXPECT_NEAR(a->Y, b->Y, absTol);
    EXPECT_NEAR(a->Z, b->Z, absTol);
}

#ifndef USE_STANDARD_MATH

namespace {

void updateMaxAbsError(double *maxErr, float approx, double ref)
{
    const double err = fabs((double)approx - ref);
    if (err > *maxErr) {
        *maxErr = err;
    }
}

// Max relative error vs max(|ref|, 1): stable when ref is near 0 (e.g. log(1)).
void updateMaxRelativeError(double *maxRelErr, float approx, double ref)
{
    if (!isfinite(ref) || !isfinite((double)approx)) {
        return;
    }
    const double err = fabs((double)approx - ref);
    const double denom = fmax(fabs(ref), 1.0);
    const double rel = err / denom;
    if (rel > *maxRelErr) {
        *maxRelErr = rel;
    }
}

double meaningfulBitsFromMaxRelativeError(double maxRelErr)
{
    if (maxRelErr <= 0.0 || !isfinite(maxRelErr)) {
        return 100.0;
    }
    return -log2(maxRelErr);
}

} // namespace

TEST(MathsUnittest, TestFastSinCos)
{
    // sin_fast / cos_fast are degree-9/8 Taylor series; accurate well beyond [-π/8, π/8]
    const float range = (float)M_PI / 4.0f;
    double sinError = 0;
    double cosError = 0;
    for (float x = -range; x <= range; x += 1e-4f) {
        updateMaxAbsError(&sinError, sin_fast(x), sin((double)x));
        updateMaxAbsError(&cosError, cos_fast(x), cos((double)x));
    }
    printf("sin_fast maximum absolute error (pi/4) = %e\n", sinError);
    printf("cos_fast maximum absolute error (pi/4) = %e\n", cosError);
    EXPECT_LE(sinError, 1e-7);
    EXPECT_LE(cosError, 1e-7);
}

TEST(MathsUnittest, TestFastSinCosCombined)
{
    const float range = (float)M_PI / 4.0f;
    double sinErr = 0;
    double cosErr = 0;
    for (float x = -range; x <= range; x += 1e-4f) {
        const sincosf_t sc = sincos_fast(x);
        updateMaxAbsError(&sinErr, sc.sin, sin((double)x));
        updateMaxAbsError(&cosErr, sc.cos, cos((double)x));
    }
    EXPECT_LE(sinErr, 1e-7);
    EXPECT_LE(cosErr, 1e-7);
}

TEST(MathsUnittest, TestFastSinCosConsistent)
{
    const float range = (float)M_PI / 4.0f;
    for (float x = -range; x <= range; x += 1e-4f) {
        const sincosf_t sc = sincos_fast(x);
        EXPECT_FLOAT_EQ(sc.sin, sin_fast(x));
        EXPECT_FLOAT_EQ(sc.cos, cos_fast(x));
    }
}

TEST(MathsUnittest, TestFastSinCosKeyValues)
{
    // sin_fast(0) == 0, cos_fast(0) == 1
    EXPECT_FLOAT_EQ(sin_fast(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(cos_fast(0.0f), 1.0f);

    // Small angle: sin(x) ≈ x, cos(x) ≈ 1
    EXPECT_NEAR(sin_fast(1e-5f), 1e-5f, 1e-12f);
    EXPECT_NEAR(cos_fast(1e-5f), 1.0f, 1e-9f);
}

TEST(MathsUnittest, TestFastTan)
{
    // tan_fast valid in [-π/4, π/4]; cos(x) >= 0.707 so no division-by-zero risk
    const float range = (float)M_PI / 4.0f;
    double tanError = 0;
    for (float x = -range; x <= range; x += 1e-4f) {
        updateMaxAbsError(&tanError, tan_fast(x), tan((double)x));
    }
    printf("tan_fast maximum absolute error (|x| <= pi/4) = %e\n", tanError);
    EXPECT_LE(tanError, 2e-7);
}


TEST(MathsUnittest, TestFastTrigonometrySinCos)
{
    double sinError = 0;
    for (float x = -10.0f * (float)M_PI; x < 10.0f * (float)M_PI; x += (float)M_PI / 300.0f) {
        updateMaxAbsError(&sinError, sin_approx(x), sin((double)x));
    }
    printf("sin_approx maximum absolute error = %e\n", sinError);
    EXPECT_LE(sinError, 3.5e-6);

    double cosError = 0;
    for (float x = -10.0f * (float)M_PI; x < 10.0f * (float)M_PI; x += (float)M_PI / 300.0f) {
        updateMaxAbsError(&cosError, cos_approx(x), cos((double)x));
    }
    printf("cos_approx maximum absolute error = %e\n", cosError);
    EXPECT_LE(cosError, 3.5e-6);
}

TEST(MathsUnittest, TestFastTrigonometrySinCosCombined)
{
    double sinErr = 0;
    double cosErr = 0;
    for (float x = -10.0f * (float)M_PI; x < 10.0f * (float)M_PI; x += (float)M_PI / 300.0f) {
        const sincosf_t sc = sincos_approx(x);
        updateMaxAbsError(&sinErr, sc.sin, sin((double)x));
        updateMaxAbsError(&cosErr, sc.cos, cos((double)x));
    }
    EXPECT_LE(sinErr, 3.5e-6);
    EXPECT_LE(cosErr, 3.5e-6);
}

TEST(MathsUnittest, TestFastTrigonometrySinCosConsistent)
{
    for (float x = -10.0f * (float)M_PI; x < 10.0f * (float)M_PI; x += (float)M_PI / 300.0f) {
        const sincosf_t sc = sincos_approx(x);
        EXPECT_FLOAT_EQ(sc.sin, sin_approx(x));
        EXPECT_FLOAT_EQ(sc.cos, cos_approx(x));
    }
}

TEST(MathsUnittest, TestFastTrigonometryTanCore)
{
    // Single quarter-turn strip (no 1/y branch): >20 bits vs libm on this interval.
    const float tol = 1.0 / (double)(1u << 20);
    double maxErr = 0;
    for (float x = -0.78f; x <= 0.78f; x += 1e-4f) {
        updateMaxAbsError(&maxErr, tan_approx(x), tan((double)x));
    }
    printf("tan_approx core (|x|<=0.78 rad) max abs error = %e (tol 2^-20 = %e)\n", maxErr, tol);
    EXPECT_LE(maxErr, tol);
}

TEST(MathsUnittest, TestFastTrigonometryTanFolded)
{
    double tanError = 0;
    const float limit = 1.4f;
    for (float x = -limit; x <= limit; x += (float)M_PI / 800.0f) {
        const float ref = tanf(x);
        if (!isfinite(ref) || fabsf(ref) > 1e3f) {
            continue;
        }
        updateMaxAbsError(&tanError, tan_approx(x), (double)ref);
    }
    printf("tan_approx folded range max abs error = %e\n", tanError);
    EXPECT_LE(tanError, 2e-5);
}

TEST(MathsUnittest, TestFastTrigonometryATan2)
{
    double error = 0;
    for (float x = -1.0f; x <= 1.0f; x += 0.02f) {
        for (float y = -1.0f; y <= 1.0f; y += 0.02f) {
            if (x == 0.0f && y == 0.0f) {
                continue;
            }
            updateMaxAbsError(&error, atan2_approx(y, x), atan2((double)y, (double)x));
        }
    }
    printf("atan2_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0);
    EXPECT_LE(error, 1e-6);
}

TEST(MathsUnittest, TestFastTrigonometryACos)
{
    double error = 0;
    for (float x = -1.0f; x <= 1.0f; x += 0.001f) {
        updateMaxAbsError(&error, acos_approx(x), acos((double)x));
    }
    printf("acos_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0);
    EXPECT_LE(error, 1e-4);
}

TEST(MathsUnittest, TestFastTrigonometryASin)
{
    double error = 0;
    for (float x = -1.0f; x <= 1.0f; x += 0.001f) {
        updateMaxAbsError(&error, asin_approx(x), asin((double)x));
    }
    printf("asin_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0);
    EXPECT_LE(error, 1e-4);
}

TEST(MathsUnittest, TestAsinAcosApproxKeyValues)
{
    const float tol = 2.5e-4f;
    const float xs[] = { -1.0f, -0.8660254f, -0.70710678f, -0.5f, -0.258819f, 0.0f,
                         0.258819f, 0.5f, 0.70710678f, 0.8660254f, 1.0f };
    for (float x : xs) {
        EXPECT_NEAR(asin_approx(x), (float)asin((double)x), tol);
        EXPECT_NEAR(acos_approx(x), (float)acos((double)x), tol);
    }
}

TEST(MathsUnittest, TestAsinAcosApproxComplementarySum)
{
    for (float x = -1.0f; x <= 1.0f; x += 0.03125f) {
        EXPECT_NEAR(asin_approx(x) + acos_approx(x), M_PI2f, 1e-6f);
    }
}

TEST(MathsUnittest, TestAsinAcosApproxSymmetryIdentities)
{
    const float tol = 2.5e-4f;
    for (float x = 1.0f / 64.0f; x <= 1.0f; x += 1.0f / 64.0f) {
        EXPECT_NEAR(asin_approx(-x), -asin_approx(x), tol);
        EXPECT_NEAR(acos_approx(-x), M_PIf - acos_approx(x), tol);
    }
}

TEST(MathsUnittest, TestAsinAcosApproxMonotonic)
{
    float prev_asin = asin_approx(-1.0f);
    float prev_acos = acos_approx(-1.0f);
    for (float x = -1.0f + 1.0f / 128.0f; x <= 1.0f; x += 1.0f / 128.0f) {
        const float a = asin_approx(x);
        const float c = acos_approx(x);
        EXPECT_GE(a, prev_asin);
        EXPECT_LE(c, prev_acos);
        prev_asin = a;
        prev_acos = c;
    }
}

TEST(MathsUnittest, TestLogApproxVsLibm)
{
    const double minMeaningfulBits = 15.0;
    double maxRelErr = 0;
    for (float x = 1e-3f; x <= 1000.0f; x *= 1.02f) {
        updateMaxRelativeError(&maxRelErr, log_approx(x), log((double)x));
    }
    const double bits = meaningfulBitsFromMaxRelativeError(maxRelErr);
    printf("log_approx (1e-3..1000) max rel err vs max(|ref|,1) = %e  (~%.2f meaningful bits)\n", maxRelErr, bits);
    EXPECT_GE(bits, minMeaningfulBits);
}

TEST(MathsUnittest, TestLogApproxUnity)
{
    const double minMeaningfulBits = 15.0;
    const double ref = 0.0;
    const double maxRelErr = fabs((double)log_approx(1.0f) - ref) / fmax(fabs(ref), 1.0);
    const double bits = meaningfulBitsFromMaxRelativeError(maxRelErr);
    printf("log_approx(1) max rel err vs max(|ref|,1) = %e  (~%.2f meaningful bits)\n", maxRelErr, bits);
    EXPECT_GE(bits, minMeaningfulBits);
}

TEST(MathsUnittest, TestExpApproxVsLibm)
{
    const double minMeaningfulBits = 14.5;
    double maxRelErr = 0;
    // exp_approx tracks libm closely in a central range; error grows toward float overflow.
    for (float x = -6.0f; x <= 6.0f; x += 0.04f) {
        updateMaxRelativeError(&maxRelErr, exp_approx(x), exp((double)x));
    }
    const double bits = meaningfulBitsFromMaxRelativeError(maxRelErr);
    printf("exp_approx ([-6,6]) max rel err vs max(|ref|,1) = %e  (~%.2f meaningful bits)\n", maxRelErr, bits);
    EXPECT_GE(bits, minMeaningfulBits);
}

TEST(MathsUnittest, TestExpApproxIdentity)
{
    const double minBitsZero = 14.0;
    const double minBitsOne = 14.0;
    const double rel0 = fabs((double)exp_approx(0.0f) - 1.0) / 1.0;
    const double rel1 = fabs((double)exp_approx(1.0f) - exp(1.0)) / fmax(fabs(exp(1.0)), 1.0);
    EXPECT_GE(meaningfulBitsFromMaxRelativeError(rel0), minBitsZero);
    EXPECT_GE(meaningfulBitsFromMaxRelativeError(rel1), minBitsOne);
}

TEST(MathsUnittest, TestPowApproxVsLibm)
{
    const double minMeaningfulBits = 14.0;
    double maxRelErr = 0;
    // pow_approx composes log_approx + exp_approx; keep a,b in a moderate range for tight error.
    for (float a = 0.25f; a <= 8.0f; a *= 1.06f) {
        for (float b = -2.5f; b <= 2.5f; b += 0.15f) {
            const double ref = pow((double)a, (double)b);
            if (!isfinite(ref) || fabs(ref) > 1e4) {
                continue;
            }
            updateMaxRelativeError(&maxRelErr, pow_approx(a, b), ref);
        }
    }
    const double bits = meaningfulBitsFromMaxRelativeError(maxRelErr);
    printf("pow_approx (filtered grid) max rel err vs max(|ref|,1) = %e  (~%.2f meaningful bits)\n", maxRelErr, bits);
    EXPECT_GE(bits, minMeaningfulBits);
}

TEST(MathsUnittest, TestPowApproxIntegerSpots)
{
    const double minMeaningfulBits = 13.0;
    struct {
        float a;
        float b;
        double ref;
    } cases[] = {
        { 2.0f, 3.0f, 8.0 },
        { 10.0f, 2.0f, 100.0 },
        { 9.0f, 0.5f, 3.0 },
    };
    for (const auto &t : cases) {
        const double maxRelErr = fabs((double)pow_approx(t.a, t.b) - t.ref) / fmax(fabs(t.ref), 1.0);
        EXPECT_GE(meaningfulBitsFromMaxRelativeError(maxRelErr), minMeaningfulBits)
            << " a=" << t.a << " b=" << t.b << " maxRelErr=" << maxRelErr;
    }
}

#endif /* !USE_STANDARD_MATH */
