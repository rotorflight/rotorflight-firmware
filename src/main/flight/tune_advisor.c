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

// Tune advisor: in-flight statistics on how the helicopter answers the rate loop, read over
// MSP (MSP2_GET_TUNE_ADVISOR) by the radio/Configurator, which turn them into per-axis advice.
// Ported from Wingflight. Heli differences: the flying check (spooled up), yaw in the motion
// check, |collective| instead of throttle bands, and a counter-output without F, so the tail and
// collective-to-pitch precomp do not count as braking.
// The firmware only measures; the advice rules live in the clients so they can change without
// a flash.
//
// Only plain rate flight counts: spooled up, airborne, the body actually rotating recently, and
// no leveling/trainer/rescue layer changing the setpoint.
//
// Feed-forward match. F sends the setpoint straight to the surfaces, so gyro / setpoint says how
// well F fits the airframe: 1.0 = the aircraft flies the rate it is asked for, above 1 = F is hot.
// The aircraft answers late (servo, aero), so the setpoint is compared delayed by 0..250 ms and
// the delay with the best correlation wins. Samples with a saturated surface are left out: there
// the rate is capped by travel, not by F. The ratio is also split by stick size (large requests
// run out of authority) and by |collective| (rotor load changes cyclic and tail authority).
//
// Stick releases. After a roll/pitch/yaw input of at least TA_RELEASE_SP_MIN is released and
// the stick stays centred, the opposite-sign rate peak within TA_RELEASE_WINDOW is the rebound.
// The controller's counter-output and I at release are kept next to it, so a client can tell
// airframe rebound (small counter-output, I near 0: needs damping) from I-term push-back.
//
// Everything accumulates across flights, so the numbers firm up, and is cleared when the tune
// changes (checked when arming) or on MSP2_CLEAR_TUNE_ADVISOR.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TUNE_ADVISOR

#include "common/maths.h"

#ifndef UNIT_TEST
#include "config/config.h"

#include "fc/rc.h"
#include "fc/rc_rates.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pid.h"
#endif

#include "tune_advisor.h"

typedef struct {
    float ab;                   // sum setpoint * gyro
    float aa;                   // sum setpoint^2
    float bb;                   // sum gyro^2
    uint32_t count;
} taSums_t;

typedef enum {
    TA_REL_IDLE = 0,            // stick centred
    TA_REL_ACTIVE,              // stick out, tracking peaks
    TA_REL_RELEASED,            // stick back, watching for the rebound
} taReleaseState_e;

typedef struct {
    taReleaseState_e state;
    float spMax, spMin;         // setpoint extremes while the stick was out
    float gyroMax, gyroMin;
    float sign;                 // direction of the input that was released
    float spPeak;               // |setpoint| peak of that input
    float gyroPeak;             // rate peak in that direction
    float rebound;              // opposite-direction rate peak after release
    float counter;              // opposite-direction surface peak after release
    float iterm;                // I at release, positive = opposing the input
    uint16_t since;             // samples since release
} taRelease_t;

typedef struct {
    float spHist[TA_LAG_MAX + 1];
    float deflHist[TA_LAG_MAX + 1];
    bool okHist[TA_LAG_MAX + 1];    // sample was usable for the FF match

    taSums_t lag[TA_LAG_MAX + 1];
    uint8_t bestLag;
    taSums_t spBand[TA_SP_BAND_COUNT];
    taSums_t collBand[TA_COLL_BAND_COUNT];

    uint32_t fullCount;
    uint32_t fullSatCount;
    float fullRate;             // sum of rate in the stick direction
    float fullSetpoint;         // sum of |setpoint|
    float fullMaxRate;

    taRelease_t rel;
    uint16_t releases;
    uint16_t bigRebounds;
    float sumRebound;
    float sumOvershoot;
    float sumCounter;
    float sumIterm;
} taAxis_t;

typedef struct {
    taAxis_t axis[XYZ_AXIS_COUNT];
    uint8_t histPos;
    uint32_t sampleCount;       // samples processed since reset, for the history fill
    uint32_t validSamples;
    uint16_t sinceMotion;       // samples since the body last rotated at TA_MOTION_RATE
    bool collecting;

#ifndef UNIT_TEST
    // Block average of PID loops into one sample
    tuneAdvisorSample_t acc;
    uint16_t accCount;
    bool accValid;
    float accTime;
    uint32_t signature;
    bool wasArmed;
#endif
} tuneAdvisor_t;

static tuneAdvisor_t ta;

void tuneAdvisorReset(void)
{
#ifndef UNIT_TEST
    const uint32_t signature = ta.signature;
#endif
    memset(&ta, 0, sizeof(ta));
    ta.sinceMotion = UINT16_MAX;
#ifndef UNIT_TEST
    ta.signature = signature;
#endif
}

static void addSums(taSums_t *s, float a, float b)
{
    s->ab += a * b;
    s->aa += a * a;
    s->bb += b * b;
    s->count++;
}

static float sumsGain(const taSums_t *s)
{
    return (s->aa > 0) ? s->ab / s->aa : 0;
}

static float sumsCorr(const taSums_t *s)
{
    const float d = s->aa * s->bb;
    return (d > 0) ? s->ab / sqrtf(d) : 0;
}

static void updateBestLag(taAxis_t *ax)
{
    float best = -2;
    for (int lag = 0; lag <= TA_LAG_MAX; lag++) {
        if (ax->lag[lag].count > 0) {
            const float r = sumsCorr(&ax->lag[lag]);
            if (r > best) {
                best = r;
                ax->bestLag = lag;
            }
        }
    }
}

static bool ffUsable(float setpoint, bool saturated)
{
    const float a = fabsf(setpoint);
    return a >= TA_SP_MIN && !saturated;
}

static void processFeedforward(taAxis_t *ax, const tuneAdvisorSample_t *s, int axis, uint8_t pos)
{
    const float sp = s->setpoint[axis];
    const float gyro = s->gyro[axis];

    ax->spHist[pos] = sp;
    ax->deflHist[pos] = s->deflection[axis];
    ax->okHist[pos] = s->valid && ffUsable(sp, s->saturated[axis]);

    if (!s->valid || s->saturated[axis])
        return;

    const uint32_t filled = MIN(ta.sampleCount, (uint32_t)(TA_LAG_MAX + 1));

    for (uint32_t lag = 0; lag < filled; lag++) {
        const uint8_t i = (pos + TA_LAG_MAX + 1 - lag) % (TA_LAG_MAX + 1);
        const float a = ax->spHist[i];
        if (ax->okHist[i] && fabsf(a) < TA_SP_HIGH)
            addSums(&ax->lag[lag], a, gyro);
    }

    if (ax->bestLag >= filled)
        return;

    const uint8_t i = (pos + TA_LAG_MAX + 1 - ax->bestLag) % (TA_LAG_MAX + 1);
    const float a = ax->spHist[i];

    if (!ax->okHist[i])
        return;

    const float mag = fabsf(a);
    const int band = (mag < TA_SP_MID) ? TA_SP_BAND_LOW : (mag < TA_SP_HIGH) ? TA_SP_BAND_MID : TA_SP_BAND_HIGH;
    addSums(&ax->spBand[band], a, gyro);

    if (mag < TA_SP_HIGH) {
        const int coll = (s->collective < TA_COLL_LOW) ? TA_COLL_BAND_LOW : (s->collective < TA_COLL_HIGH) ? TA_COLL_BAND_MID : TA_COLL_BAND_HIGH;
        addSums(&ax->collBand[coll], a, gyro);
    }
}

// Rate now against the full-stick request one best-lag ago, as for the FF match. Direction comes
// from the setpoint, not the stick: yaw is negated between the two (setpoint.c).
static void processFullStick(taAxis_t *ax, const tuneAdvisorSample_t *s, int axis, uint8_t pos)
{
    const uint8_t lag = (ta.sampleCount > (uint32_t)ax->bestLag) ? ax->bestLag : 0;
    const uint8_t i = (pos + TA_LAG_MAX + 1 - lag) % (TA_LAG_MAX + 1);
    const float sp = ax->spHist[i];

    if (!s->valid || fabsf(ax->deflHist[i]) < TA_FULL_STICK)
        return;

    const float dir = (sp >= 0) ? 1.0f : -1.0f;
    const float rate = dir * s->gyro[axis];

    ax->fullCount++;
    if (s->saturated[axis])
        ax->fullSatCount++;
    ax->fullRate += rate;
    ax->fullSetpoint += fabsf(sp);
    ax->fullMaxRate = MAX(ax->fullMaxRate, rate);
}

static void releaseStart(taRelease_t *r, float sp, float gyro)
{
    r->state = TA_REL_ACTIVE;
    r->spMax = r->spMin = sp;
    r->gyroMax = r->gyroMin = gyro;
}

static void releaseFinish(taAxis_t *ax)
{
    taRelease_t *r = &ax->rel;

    if (r->spPeak >= TA_RELEASE_SP_MIN && r->gyroPeak >= TA_RELEASE_GYRO_MIN) {
        const float rebound = MAX(r->rebound, 0.0f) / r->gyroPeak;

        ax->releases++;
        if (rebound >= TA_REBOUND_BIG)
            ax->bigRebounds++;
        ax->sumRebound += rebound;
        ax->sumOvershoot += r->gyroPeak / r->spPeak;
        ax->sumCounter += MAX(r->counter, 0.0f);
        ax->sumIterm += r->iterm;
    }

    r->state = TA_REL_IDLE;
}

static void processRelease(taAxis_t *ax, const tuneAdvisorSample_t *s, int axis)
{
    taRelease_t *r = &ax->rel;
    const float sp = s->setpoint[axis];
    const float gyro = s->gyro[axis];
    const bool centred = fabsf(sp) < TA_STICK_CENTER;

    if (!s->valid || ax->releases == UINT16_MAX) {
        r->state = TA_REL_IDLE;
        return;
    }

    switch (r->state) {
        case TA_REL_IDLE:
            if (!centred)
                releaseStart(r, sp, gyro);
            break;

        case TA_REL_ACTIVE:
            if (!centred) {
                r->spMax = MAX(r->spMax, sp);
                r->spMin = MIN(r->spMin, sp);
                r->gyroMax = MAX(r->gyroMax, gyro);
                r->gyroMin = MIN(r->gyroMin, gyro);
            }
            else {
                r->sign = (r->spMax >= -r->spMin) ? 1.0f : -1.0f;
                r->spPeak = (r->sign > 0) ? r->spMax : -r->spMin;
                r->gyroPeak = (r->sign > 0) ? r->gyroMax : -r->gyroMin;
                r->rebound = -r->sign * gyro;
                r->counter = -r->sign * s->feedback[axis];
                r->iterm = -r->sign * s->iterm[axis];
                r->since = 0;
                r->state = TA_REL_RELEASED;
            }
            break;

        case TA_REL_RELEASED:
            if (!centred) {
                if (r->since >= TA_RELEASE_MIN_QUIET)
                    releaseFinish(ax);
                releaseStart(r, sp, gyro);
                break;
            }
            r->since++;
            if (r->since <= TA_RELEASE_PEAK_TAIL)
                r->gyroPeak = MAX(r->gyroPeak, r->sign * gyro);
            r->rebound = MAX(r->rebound, -r->sign * gyro);
            r->counter = MAX(r->counter, -r->sign * s->feedback[axis]);
            if (r->since >= TA_RELEASE_WINDOW)
                releaseFinish(ax);
            break;
    }
}

void tuneAdvisorProcessSample(const tuneAdvisorSample_t *s)
{
    // All three axes: a pirouette is flying. (Wingflight leaves yaw out because of taxi turns.)
    const float motion = fabsf(s->gyro[FD_ROLL]) + fabsf(s->gyro[FD_PITCH]) + fabsf(s->gyro[FD_YAW]);

    if (motion >= TA_MOTION_RATE)
        ta.sinceMotion = 0;
    else if (ta.sinceMotion < UINT16_MAX)
        ta.sinceMotion++;

    tuneAdvisorSample_t gated = *s;
    gated.valid = s->valid && ta.sinceMotion <= TA_MOTION_HOLD;
    ta.collecting = gated.valid;

    if (gated.valid)
        ta.validSamples++;

    ta.sampleCount++;
    ta.histPos = (ta.histPos + 1) % (TA_LAG_MAX + 1);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        taAxis_t *ax = &ta.axis[axis];

        processFeedforward(ax, &gated, axis, ta.histPos);
        processFullStick(ax, &gated, axis, ta.histPos);
        processRelease(ax, &gated, axis);

        if ((ta.sampleCount % TA_SAMPLE_HZ) == 0)
            updateBestLag(ax);
    }
}

void tuneAdvisorGetAxis(int axis, tuneAdvisorAxis_t *out)
{
    const taAxis_t *ax = &ta.axis[axis];
    const taSums_t *best = &ax->lag[ax->bestLag];

    memset(out, 0, sizeof(*out));

    out->ffCount = best->count;
    out->ffGain = sumsGain(best);
    out->ffCorr = sumsCorr(best);
    out->ffLagMs = ax->bestLag * (1000 / TA_SAMPLE_HZ);

    for (int i = 0; i < TA_SP_BAND_COUNT; i++) {
        out->spBand[i].gain = sumsGain(&ax->spBand[i]);
        out->spBand[i].count = ax->spBand[i].count;
    }
    for (int i = 0; i < TA_COLL_BAND_COUNT; i++) {
        out->collBand[i].gain = sumsGain(&ax->collBand[i]);
        out->collBand[i].count = ax->collBand[i].count;
    }

    out->fullCount = ax->fullCount;
    out->fullSatCount = ax->fullSatCount;
    out->fullRatio = (ax->fullSetpoint > 0) ? ax->fullRate / ax->fullSetpoint : 0;
    out->fullMaxRate = ax->fullMaxRate;

    out->releases = ax->releases;
    out->bigRebounds = ax->bigRebounds;
    if (ax->releases) {
        out->meanRebound = ax->sumRebound / ax->releases;
        out->meanOvershoot = ax->sumOvershoot / ax->releases;
        out->meanCounter = ax->sumCounter / ax->releases;
        out->meanIterm = ax->sumIterm / ax->releases;
    }
}

uint32_t tuneAdvisorGetValidSamples(void)
{
    return ta.validSamples;
}

bool tuneAdvisorIsCollecting(void)
{
    return ta.collecting;
}

#ifndef UNIT_TEST

static uint32_t hashBytes(uint32_t h, const void *data, size_t len)
{
    const uint8_t *p = data;
    for (size_t i = 0; i < len; i++) {
        h = (h ^ p[i]) * 16777619u;     // FNV-1a
    }
    return h;
}

// Anything that changes what the statistics describe. Checked when arming, so the numbers
// always belong to the tune being flown.
static uint32_t tuneSignature(void)
{
    uint32_t h = 2166136261u;
    const uint8_t profiles[2] = { getCurrentPidProfileIndex(), getCurrentControlRateProfileIndex() };

    h = hashBytes(h, profiles, sizeof(profiles));
    h = hashBytes(h, currentPidProfile->pid, sizeof(currentPidProfile->pid));
    h = hashBytes(h, currentPidProfile->iterm_relax_cutoff, sizeof(currentPidProfile->iterm_relax_cutoff));
    h = hashBytes(h, &currentPidProfile->pid_mode, sizeof(currentPidProfile->pid_mode));
    h = hashBytes(h, &currentControlRateProfile->rates_type, sizeof(currentControlRateProfile->rates_type));
    h = hashBytes(h, currentControlRateProfile->rcRates, sizeof(currentControlRateProfile->rcRates));
    h = hashBytes(h, currentControlRateProfile->rcExpo, sizeof(currentControlRateProfile->rcExpo));
    h = hashBytes(h, currentControlRateProfile->sRates, sizeof(currentControlRateProfile->sRates));

    return h;
}

static bool plainRateFlight(void)
{
    return ARMING_FLAG(ARMED) && isSpooledUp() && isAirborne() &&
        !FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | TRAINER_MODE | ALTHOLD_MODE | RESCUE_MODE |
                     GPS_RESCUE_MODE | FAILSAFE_MODE);
}

void tuneAdvisorUpdate(float dT)
{
    const bool armed = ARMING_FLAG(ARMED);

    if (armed && !ta.wasArmed) {
        const uint32_t signature = tuneSignature();
        if (signature != ta.signature) {
            tuneAdvisorReset();
            ta.signature = signature;
        }
    }
    ta.wasArmed = armed;

    const pidAxisData_t *pd = pidGetAxisData();
    tuneAdvisorSample_t *acc = &ta.acc;

    if (ta.accCount == 0) {
        memset(acc, 0, sizeof(*acc));
        ta.accValid = true;
    }

    ta.accValid = ta.accValid && plainRateFlight();
    acc->collective += getCollectiveDeflectionAbs();
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc->setpoint[axis] += pd[axis].setPoint;
        acc->gyro[axis] += pd[axis].gyroRate;
        acc->iterm[axis] += pd[axis].I;
        acc->feedback[axis] += pd[axis].pidSum - pd[axis].F - pd[axis].O;
        acc->deflection[axis] += getRcDeflection(axis);
        acc->saturated[axis] |= pidAxisSaturated(axis);
    }
    ta.accCount++;
    ta.accTime += dT;

    if (ta.accTime < 1.0f / TA_SAMPLE_HZ)
        return;

    const float k = 1.0f / ta.accCount;
    acc->valid = ta.accValid;
    acc->collective *= k;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc->setpoint[axis] *= k;
        acc->gyro[axis] *= k;
        acc->iterm[axis] *= k;
        acc->feedback[axis] *= k;
        acc->deflection[axis] *= k;
    }

    tuneAdvisorProcessSample(acc);

    ta.accCount = 0;
    ta.accTime -= 1.0f / TA_SAMPLE_HZ;
}

#endif // !UNIT_TEST

#endif // USE_TUNE_ADVISOR
