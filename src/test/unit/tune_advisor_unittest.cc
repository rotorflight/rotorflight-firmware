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

#include <cmath>
#include <deque>

#include "gtest/gtest.h"

extern "C" {
#include "flight/tune_advisor.h"
}

// Simulated airframe on one axis: rate = gain x setpoint, delay samples later, capped at maxRate.
// The other axes stay at zero.
class TuneAdvisorTest : public ::testing::Test {
protected:
    float gain = 1.0f;
    int delay = 0;
    float maxRate = 10000.0f;
    std::deque<float> pipe;

    void SetUp() override
    {
        tuneAdvisorReset();
        pipe.clear();
    }

    tuneAdvisorSample_t sample(float setpoint, float collective = 0.5f, bool valid = true)
    {
        pipe.push_back(setpoint);
        float delayed = 0;
        if ((int)pipe.size() > delay) {
            delayed = pipe.front();
            pipe.pop_front();
        }
        float rate = gain * delayed;
        const bool saturated = fabsf(rate) > maxRate;
        if (saturated)
            rate = copysignf(maxRate, rate);

        tuneAdvisorSample_t s = {};
        s.valid = valid;
        s.collective = collective;
        s.setpoint[FD_ROLL] = setpoint;
        s.gyro[FD_ROLL] = rate;
        s.deflection[FD_ROLL] = setpoint / 400.0f;
        s.saturated[FD_ROLL] = saturated;
        return s;
    }

    // Stick doublets and holds covering 40..400 deg/s, `seconds` long
    void flyRolls(float seconds, float collective = 0.5f, bool valid = true)
    {
        const int n = seconds * TA_SAMPLE_HZ;
        for (int i = 0; i < n; i++) {
            const float t = (float)i / TA_SAMPLE_HZ;
            const float sp = 60.0f * sinf(2.0f * (float)M_PI * 0.7f * t) + 70.0f * sinf(2.0f * (float)M_PI * 0.23f * t);
            tuneAdvisorSample_t s = sample(sp, collective, valid);
            tuneAdvisorProcessSample(&s);
        }
    }

    void feed(float setpoint, float gyro, int samples, float feedback = 0, float iterm = 0)
    {
        for (int i = 0; i < samples; i++) {
            tuneAdvisorSample_t s = {};
            s.valid = true;
            s.collective = 0.3f;
            s.setpoint[FD_ROLL] = setpoint;
            s.gyro[FD_ROLL] = gyro;
            s.feedback[FD_ROLL] = feedback;
            s.iterm[FD_ROLL] = iterm;
            s.deflection[FD_ROLL] = setpoint / 400.0f;
            tuneAdvisorProcessSample(&s);
        }
    }

    tuneAdvisorAxis_t roll()
    {
        tuneAdvisorAxis_t a;
        tuneAdvisorGetAxis(FD_ROLL, &a);
        return a;
    }
};

TEST_F(TuneAdvisorTest, MatchedFeedforwardReadsOne)
{
    flyRolls(60);

    const tuneAdvisorAxis_t a = roll();
    EXPECT_GT(a.ffCount, 1000u);
    EXPECT_NEAR(1.0f, a.ffGain, 0.01f);
    EXPECT_NEAR(1.0f, a.ffCorr, 0.01f);
    EXPECT_EQ(0, a.ffLagMs);
}

TEST_F(TuneAdvisorTest, HotFeedforwardAndLagAreFound)
{
    gain = 1.3f;
    delay = 11;     // 110 ms, the first log's roll delay

    flyRolls(60);

    const tuneAdvisorAxis_t a = roll();
    EXPECT_NEAR(1.3f, a.ffGain, 0.02f);
    EXPECT_GT(a.ffCorr, 0.99f);
    EXPECT_EQ(110, a.ffLagMs);
    EXPECT_NEAR(1.3f, a.spBand[TA_SP_BAND_LOW].gain, 0.05f);
    EXPECT_NEAR(1.3f, a.spBand[TA_SP_BAND_MID].gain, 0.05f);
}

TEST_F(TuneAdvisorTest, SaturatedSamplesDoNotPullTheGainDown)
{
    maxRate = 120.0f;   // capped by surface travel above 120 deg/s

    flyRolls(60);

    const tuneAdvisorAxis_t a = roll();
    EXPECT_NEAR(1.0f, a.ffGain, 0.02f);
}

TEST_F(TuneAdvisorTest, CollectiveBandsAreSeparate)
{
    gain = 1.2f;
    flyRolls(30, 0.1f);
    gain = 1.7f;
    flyRolls(30, 0.8f);

    const tuneAdvisorAxis_t a = roll();
    EXPECT_NEAR(1.2f, a.collBand[TA_COLL_BAND_LOW].gain, 0.05f);
    EXPECT_NEAR(1.7f, a.collBand[TA_COLL_BAND_HIGH].gain, 0.05f);
    EXPECT_EQ(0u, a.collBand[TA_COLL_BAND_MID].count);
}

TEST_F(TuneAdvisorTest, InvalidFlightIsIgnored)
{
    flyRolls(30, 0.5f, false);

    EXPECT_EQ(0u, roll().ffCount);
    EXPECT_EQ(0u, tuneAdvisorGetValidSamples());
}

TEST_F(TuneAdvisorTest, GroundStickChecksAreIgnored)
{
    // Armed and "valid", stick moving, but the aircraft is not rotating
    gain = 0.0f;
    flyRolls(30);

    EXPECT_LT(roll().ffCount, (uint32_t)(TA_MOTION_HOLD + 50));
    EXPECT_LT(tuneAdvisorGetValidSamples(), (uint32_t)(TA_MOTION_HOLD + 50));
}

TEST_F(TuneAdvisorTest, ReleaseReboundIsMeasured)
{
    feed(0, 50, 20);                    // flying
    feed(120, 180, 40);                 // roll right, 1.5x overshoot
    feed(0, 60, 5, -0.02f, -0.01f);     // released, still rolling; controller brakes a little
    feed(0, -45, 10, -0.02f);           // rebound of 25% of the peak
    feed(0, 0, 80);                     // settled

    const tuneAdvisorAxis_t a = roll();
    EXPECT_EQ(1, a.releases);
    EXPECT_EQ(1, a.bigRebounds);
    EXPECT_NEAR(0.25f, a.meanRebound, 0.01f);
    EXPECT_NEAR(1.5f, a.meanOvershoot, 0.01f);
    EXPECT_NEAR(0.02f, a.meanCounter, 0.001f);
    EXPECT_NEAR(0.01f, a.meanIterm, 0.001f);
}

TEST_F(TuneAdvisorTest, LeftRollReboundHasTheSameSign)
{
    feed(0, 50, 20);
    feed(-120, -150, 40);
    feed(0, 15, 100);                   // 10% rebound to the right

    const tuneAdvisorAxis_t a = roll();
    EXPECT_EQ(1, a.releases);
    EXPECT_EQ(0, a.bigRebounds);
    EXPECT_NEAR(0.10f, a.meanRebound, 0.01f);
}

TEST_F(TuneAdvisorTest, QuickReversalIsNotARelease)
{
    feed(0, 50, 20);
    feed(120, 180, 40);
    feed(0, 60, 5);                     // centred for only 50 ms
    feed(-120, -180, 40);

    EXPECT_EQ(0, roll().releases);
}

TEST_F(TuneAdvisorTest, SmallInputsAreNotReleases)
{
    feed(0, 50, 20);
    feed(40, 60, 40);
    feed(0, -20, 80);

    EXPECT_EQ(0, roll().releases);
}

// Stick centred, gyro oscillating at freq Hz with this amplitude, plus a slow drift
static void feedShake(float amplitude, float freq, int samples)
{
    for (int i = 0; i < samples; i++) {
        const float t = (float)i / TA_SAMPLE_HZ;
        tuneAdvisorSample_t s = {};
        s.valid = true;
        s.collective = 0.3f;
        s.gyro[FD_ROLL] = amplitude * sinf(2.0f * (float)M_PI * freq * t) + 50.0f * sinf(2.0f * (float)M_PI * 0.1f * t);
        tuneAdvisorProcessSample(&s);
    }
}

TEST_F(TuneAdvisorTest, CentredShakeIsMeasured)
{
    feedShake(50, 15, 1000);            // the issue log's roll: P oscillating at 15 Hz

    const tuneAdvisorAxis_t a = roll();
    EXPECT_EQ(1000u - 1 - TA_SHAKE_SETTLE, a.shakeCount);       // the first sample is not rotating yet
    EXPECT_NEAR(0.93f * 50.0f / sqrtf(2), a.shakeRms, 1.0f);    // the high-pass keeps 93% at 15 Hz
}

TEST_F(TuneAdvisorTest, SlowDriftIsNotShake)
{
    feedShake(0, 0, 1000);              // only the 0.1 Hz drift

    EXPECT_GT(roll().shakeCount, 0u);
    EXPECT_LT(roll().shakeRms, 3.0f);
}

TEST_F(TuneAdvisorTest, StopIsNotShake)
{
    feed(0, 50, 20);
    feed(120, 180, 40);
    feed(0, -45, 10);                   // big rebound, inside the settle time
    feed(0, 50, 40);                    // the heli still rotating keeps the flying check on
    feed(0, 50, 100);                   // steady rate, no shake

    const tuneAdvisorAxis_t a = roll();
    EXPECT_GT(a.shakeCount, 0u);
    EXPECT_LT(a.shakeRms, 5.0f);
}

// A stop with a slow second-order settle and no sustained oscillation: `stops` rolls held at
// `rate` for 1 s, then the stick centred for `centredMs` while the rate settles with 15%
// overshoot, the opposite-sign peak `peakMs` after the release.
static void feedSlowStops(float rate, int peakMs, int centredMs, int stops)
{
    const float zeta = 0.517f;          // 15% overshoot
    const float wd = (float)M_PI / (peakMs / 1000.0f);
    const float wn = wd / sqrtf(1 - zeta * zeta);
    for (int stop = 0; stop < stops; stop++) {
        for (int i = 0; i < TA_SAMPLE_HZ; i++) {
            tuneAdvisorSample_t s = {};
            s.valid = true;
            s.collective = 0.3f;
            s.setpoint[FD_ROLL] = rate;
            s.gyro[FD_ROLL] = rate;
            s.deflection[FD_ROLL] = rate / 400.0f;
            tuneAdvisorProcessSample(&s);
        }
        for (int i = 0; i < centredMs * TA_SAMPLE_HZ / 1000; i++) {
            const float t = (float)i / TA_SAMPLE_HZ;
            tuneAdvisorSample_t s = {};
            s.valid = true;
            s.collective = 0.3f;
            s.gyro[FD_ROLL] = rate * expf(-zeta * wn * t) *
                (cosf(wd * t) + zeta / sqrtf(1 - zeta * zeta) * sinf(wd * t));
            tuneAdvisorProcessSample(&s);
        }
    }
}

TEST_F(TuneAdvisorTest, SlowReboundIsNotShake)
{
    // Rebounds peaking 300 and 400 ms after the stop read 15-40 deg/s with a 0.3 s settle,
    // above the 15 deg/s the clients take for shaking
    const float rates[] = { 360, 500 };
    const int peaks[] = { 300, 400 };
    for (float rate : rates) {
        for (int peakMs : peaks) {
            tuneAdvisorReset();
            feedSlowStops(rate, peakMs, 1500, 12);

            const tuneAdvisorAxis_t a = roll();
            EXPECT_GT(a.shakeCount, 0u) << rate << " deg/s, peak at " << peakMs << " ms";
            EXPECT_LT(a.shakeRms, 5.0f) << rate << " deg/s, peak at " << peakMs << " ms";
            EXPECT_EQ(12, a.releases) << rate << " deg/s, peak at " << peakMs << " ms";
        }
    }
}

TEST_F(TuneAdvisorTest, CentredSpellShorterThanTheSettleAddsNothing)
{
    feedSlowStops(500, 400, 800, 12);

    EXPECT_EQ(0u, roll().shakeCount);
}

TEST_F(TuneAdvisorTest, CrossCouplingIsNotShake)
{
    // A flip: pitch commanded, roll stick centred, roll moving with the pitch input
    for (int i = 0; i < 500; i++) {
        const float t = (float)i / TA_SAMPLE_HZ;
        tuneAdvisorSample_t s = {};
        s.valid = true;
        s.collective = 0.3f;
        s.setpoint[FD_PITCH] = 300;
        s.gyro[FD_PITCH] = 300;
        s.gyro[FD_ROLL] = 40.0f * sinf(2.0f * (float)M_PI * 6.0f * t);
        tuneAdvisorProcessSample(&s);
    }

    EXPECT_EQ(0u, roll().shakeCount);
}

TEST_F(TuneAdvisorTest, FullStickReportsTheRateReached)
{
    gain = 1.0f;
    maxRate = 200.0f;   // airframe tops out at 200 deg/s

    feed(0, 50, 20);
    for (int i = 0; i < 200; i++) {
        tuneAdvisorSample_t s = sample(400);
        tuneAdvisorProcessSample(&s);
    }

    const tuneAdvisorAxis_t a = roll();
    EXPECT_GT(a.fullCount, 150u);
    EXPECT_EQ(a.fullCount, a.fullSatCount);
    EXPECT_NEAR(0.5f, a.fullRatio, 0.01f);
    EXPECT_NEAR(200.0f, a.fullMaxRate, 0.1f);
}

TEST_F(TuneAdvisorTest, FullStickDirectionFollowsTheSetpoint)
{
    // Yaw: the setpoint is the negated stick (setpoint.c)
    feed(0, 50, 20);
    for (int i = 0; i < 200; i++) {
        tuneAdvisorSample_t s = sample(300);
        s.deflection[FD_ROLL] = -1.0f;
        tuneAdvisorProcessSample(&s);
    }

    const tuneAdvisorAxis_t a = roll();
    EXPECT_GT(a.fullCount, 150u);
    EXPECT_NEAR(1.0f, a.fullRatio, 0.01f);
    EXPECT_NEAR(300.0f, a.fullMaxRate, 0.1f);
}

TEST_F(TuneAdvisorTest, PirouetteCountsAsFlying)
{
    // Yaw only: roll and pitch stay still, the tail answers 1.2x the stick
    for (int i = 0; i < 3000; i++) {
        const float t = (float)i / TA_SAMPLE_HZ;
        tuneAdvisorSample_t s = {};
        s.valid = true;
        s.collective = 0.3f;
        s.setpoint[FD_YAW] = 150.0f * sinf(2.0f * (float)M_PI * 0.5f * t);
        s.gyro[FD_YAW] = 1.2f * s.setpoint[FD_YAW];
        tuneAdvisorProcessSample(&s);
    }

    tuneAdvisorAxis_t a;
    tuneAdvisorGetAxis(FD_YAW, &a);
    EXPECT_GT(a.ffCount, 1000u);
    EXPECT_NEAR(1.2f, a.ffGain, 0.02f);
}

TEST_F(TuneAdvisorTest, ResetClearsEverything)
{
    flyRolls(10);
    feedShake(50, 15, 100);
    tuneAdvisorReset();

    EXPECT_EQ(0u, roll().ffCount);
    EXPECT_EQ(0u, roll().shakeCount);
    EXPECT_EQ(0u, tuneAdvisorGetValidSamples());
}
