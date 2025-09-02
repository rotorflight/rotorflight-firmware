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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "pg/adjustments.h"
#include "pg/rates.h"

#include "fc/rc.h"
#include "fc/rc_rates.h"


const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] = {
    [RATES_TYPE_BETAFLIGHT] = { 255, 100, 100 },
    [RATES_TYPE_RACEFLIGHT] = { 200, 255, 100 },
    [RATES_TYPE_KISS]       = { 255,  99, 100 },
    [RATES_TYPE_ACTUAL]     = { 200, 200, 100 },
    [RATES_TYPE_QUICK]      = { 255, 200, 100 },
};

typedef float (*applyRatesCurveFn)(const int axis, float rcCommandf);

FAST_DATA_ZERO_INIT applyRatesCurveFn applyRatesFn;

FAST_DATA_ZERO_INIT controlRateConfig_t * currentControlRateProfile;


/*** Adjustment Functions ***/

int get_ADJUSTMENT_RATE_PROFILE(__unused int adjFunc)
{
    return getCurrentControlRateProfileIndex() + 1;
}

void set_ADJUSTMENT_RATE_PROFILE(__unused int adjFunc, int value)
{
    changeControlRateProfile(value - 1);
}

int get_ADJUSTMENT_PITCH_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rates[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rates[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rates[FD_YAW];
}

void set_ADJUSTMENT_YAW_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rates[FD_YAW] = value;
}

int get_ADJUSTMENT_PITCH_RC_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rcRates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcRates[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_RC_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rcRates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcRates[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_RC_RATE(__unused int adjFunc)
{
    return currentControlRateProfile->rcRates[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_RATE(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcRates[FD_YAW] = value;
}

int get_ADJUSTMENT_PITCH_RC_EXPO(__unused int adjFunc)
{
    return currentControlRateProfile->rcExpo[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_EXPO(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcExpo[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_RC_EXPO(__unused int adjFunc)
{
    return currentControlRateProfile->rcExpo[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_EXPO(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcExpo[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_RC_EXPO(__unused int adjFunc)
{
    return currentControlRateProfile->rcExpo[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_EXPO(__unused int adjFunc, int value)
{
    currentControlRateProfile->rcExpo[FD_YAW] = value;
}


/*** Rates Curve Functions ***/

static float applyNullRates(const int axis, float rcCommandf)
{
    UNUSED(axis);

    return 500.0f * rcCommandf;
}

static float applyBetaflightRates(const int axis, float rcCommandf)
{
    const float rcCommandfAbs = fabsf(rcCommandf);

    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * POWER3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += 14.54f * (rcRate - 2.0f);
    }

    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

static float applyRaceFlightRates(const int axis, float rcCommandf)
{
    const float rcCommandfAbs = fabsf(rcCommandf);

    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);

    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
    angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->rates[axis] * 0.01f);

    return angleRate;
}

static float applyKissRates(const int axis, float rcCommandf)
{
    const float rcCommandfAbs = fabsf(rcCommandf);
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (POWER3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = 2000.0f * kissRpyUseRates * kissRcCommandf;

    return kissAngle;
}

static float applyActualRates(const int axis, float rcCommandf)
{
    const float rcCommandfAbs = fabsf(rcCommandf);
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    float expof = rcCommandfAbs * (powf(rcCommandf, 5) * rcExpo + rcCommandf * (1 - rcExpo));

    const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
    const float stickMovement = MAX(0, currentControlRateProfile->rates[axis] * 10.0f - centerSensitivity);
    const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

static float applyQuickRates(const int axis, float rcCommandf)
{
    const float rcCommandfAbs = fabsf(rcCommandf);

    const uint16_t rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const uint16_t maxDPS = MAX(currentControlRateProfile->rates[axis] * 10, rcRate);
    const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float superFactorConfig = ((float)maxDPS / rcRate - 1) / ((float)maxDPS / rcRate);

    float curve;
    float superFactor;
    float angleRate;

    if (currentControlRateProfile->quickRatesRcExpo) {
        curve = POWER3(rcCommandf) * expof + rcCommandf * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superFactorConfig), 0.01f, 1.00f));
        angleRate = curve * rcRate * superFactor;
    } else {
        curve = POWER3(rcCommandfAbs) * expof + rcCommandfAbs * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
        angleRate = rcCommandf * rcRate * superFactor;
    }

    return angleRate;
}

float applyRatesCurve(const int axis, float rcCommandf)
{
    float rate = applyRatesFn(axis, rcCommandf);

    rate = constrainf(rate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

    // Collective is an angle - scale it here so that 480°/s => 12°
    if (axis == COLLECTIVE)
        rate *= 2.083333333f;

    return rate;
}


INIT_CODE void loadControlRateProfile(void)
{
    currentControlRateProfile = controlRateProfilesMutable(systemConfig()->activeRateProfile);

    switch (currentControlRateProfile->rates_type)
    {
        case RATES_TYPE_BETAFLIGHT:
            applyRatesFn = applyBetaflightRates;
            break;
        case RATES_TYPE_RACEFLIGHT:
            applyRatesFn = applyRaceFlightRates;
            break;
        case RATES_TYPE_KISS:
            applyRatesFn = applyKissRates;
            break;
        case RATES_TYPE_ACTUAL:
            applyRatesFn = applyActualRates;
            break;
        case RATES_TYPE_QUICK:
            applyRatesFn = applyQuickRates;
            break;
        default:
            applyRatesFn = applyNullRates;
            break;
    }

    setpointInitProfile();
}

INIT_CODE void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
}

INIT_CODE void copyControlRateProfile(uint8_t dstControlRateProfileIndex, uint8_t srcControlRateProfileIndex) {
    if (dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        dstControlRateProfileIndex != srcControlRateProfileIndex) {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
