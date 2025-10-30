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

#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "pg/adjustments.h"
#include "pg/rates.h"

#include "fc/rc.h"
#include "fc/rc_rates.h"


const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] =
{
    [RATES_TYPE_BETAFLIGHT] = { 255, 100, 100 },
    [RATES_TYPE_RACEFLIGHT] = { 200, 255, 100 },
    [RATES_TYPE_KISS]       = { 255,  99, 100 },
    [RATES_TYPE_ACTUAL]     = { 200, 200, 100 },
    [RATES_TYPE_QUICK]      = { 255, 200, 100 },
};


FAST_DATA_ZERO_INIT controlRateConfig_t * currentControlRateProfile;


/*** Adjustment Functions ***/

int get_ADJUSTMENT_RATE_PROFILE(void)
{
    return getCurrentControlRateProfileIndex() + 1;
}

void set_ADJUSTMENT_RATE_PROFILE(int value)
{
    changeControlRateProfile(value - 1);
}

int get_ADJUSTMENT_PITCH_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_YAW];
}

void set_ADJUSTMENT_YAW_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_YAW] = value;
}

int get_ADJUSTMENT_PITCH_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_YAW] = value;
}

int get_ADJUSTMENT_PITCH_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_PITCH] = value;
}

int get_ADJUSTMENT_ROLL_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_ROLL] = value;
}

int get_ADJUSTMENT_YAW_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_YAW] = value;
}


/*** Rates Curve Functions ***/

static float applyNullRates(const int axis, const float rcCommandAbs)
{
    UNUSED(axis);

    return 500.0f * rcCommandAbs;
}

/*
 * BETAFLIGHT
 *
 *  float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 *  {
 *      if (currentControlRateProfile->rcExpo[axis]) {
 *          const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
 *          rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
 *      }
 *
 *      float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
 *      if (rcRate > 2.0f) {
 *          rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
 *      }
 *      float angleRate = 200.0f * rcRate * rcCommandf;
 *      if (currentControlRateProfile->sRates[axis]) {
 *          const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->sRates[axis] / 100.0f)), 0.01f, 1.00f));
 *          angleRate *= rcSuperfactor;
 *      }
 *
 *      return angleRate;
 *   }
 */

static float applyBetaflightRates(const int axis, const float rcCommandAbs)
{
    float rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float sRate = currentControlRateProfile->sRates[axis] / 100.0f;

    if (rcRate > 400) {
        rcRate += 14.54f * (rcRate - 400);
    }

    // Fourth order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + POWER4(rcCommandAbs) * rcExpo;

    // Super rate factor
    float superFactor = 1.0f / constrainf(1.0f - rcCommandAbs * sRate, 0.01f, 1.00f);

    // Final angle rate
    float angleRate = rcRate * expof * superFactor;

    return angleRate;
}

/*
 * RACEFLIGHT
 *
 *  float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 *  {
 *      rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
 *      float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
 *      angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->sRates[axis] * 0.01f);
 *
 *      return angleRate;
 *  }
 */

static float applyRaceFlightRates(const int axis, const float rcCommandAbs)
{
    const float rcRate = currentControlRateProfile->rcRates[axis] * 10;
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float sRate = currentControlRateProfile->sRates[axis] / 100.0f;

    // Third order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + POWER3(rcCommandAbs) * rcExpo;

    // Fourth order component
    expof += expof * rcCommandAbs * sRate;

    // Final angle rate
    float angleRate = rcRate * expof;

    return angleRate;
}

/*
 * KISS
 *
 *  float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 *  {
 *      const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;
 *
 *      float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->sRates[axis] / 100.0f)), 0.01f, 1.00f));
 *      float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
 *      float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
 *
 *      return kissAngle;
 *  }
 */

static float applyKissRates(const int axis, const float rcCommandAbs)
{
    const float rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float sRate = currentControlRateProfile->sRates[axis] / 100.0f;

    // Third order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + POWER3(rcCommandAbs) * rcExpo;

    // Super rate factor
    float superFactor = 1.0f / constrainf(1.0f - rcCommandAbs * sRate, 0.01f, 1.00f);

    // Final angle rate
    float angleRate = rcRate * expof * superFactor;

    return angleRate;
}

/*
 * ACTUAL
 *
 *  float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 *  {
 *      float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
 *      expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));
 *
 *      const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
 *      const float stickMovement = MAX(0, currentControlRateProfile->sRates[axis] * 10.0f - centerSensitivity);
 *      const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;
 *
 *      return angleRate;
 *  }
 */

static float applyActualRates(const int axis, const float rcCommandAbs)
{
    const float rcRate = currentControlRateProfile->rcRates[axis] * 10;
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float sRate = currentControlRateProfile->sRates[axis] * 10;

    float aRate = fmaxf(0, sRate - rcRate);

    // Sixth order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + POWER6(rcCommandAbs) * rcExpo;

    // Final angle rate
    float angleRate = rcRate * rcCommandAbs + aRate * expof;

    return angleRate;
}

/*
 * QUICK
 *
 *  float applyQuickRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 *  {
 *      const uint16_t rcRate = currentControlRateProfile->rcRates[axis] * 2;
 *      const uint16_t maxDPS = MAX(currentControlRateProfile->sRates[axis] * 10, rcRate);
 *      const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
 *      const float superFactorConfig = ((float)maxDPS / rcRate - 1) / ((float)maxDPS / rcRate);
 *
 *      float curve, superFactor, angleRate;
 *
 *      if (currentControlRateProfile->quickRatesRcExpo) {
 *          curve = power3(rcCommandf) * expof + rcCommandf * (1 - expof);
 *          superFactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superFactorConfig), 0.01f, 1.00f));
 *          angleRate = constrainf(curve * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
 *      } else {
 *          curve = power3(rcCommandfAbs) * expof + rcCommandfAbs * (1 - expof);
 *          superFactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
 *          angleRate = constrainf(rcCommandf * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
 *      }
 *
 *      return angleRate;
 *  }
 */

static float applyQuickRates(const int axis, const float rcCommandAbs)
{
    const float rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float sRate = currentControlRateProfile->sRates[axis] * 10;

    // Third order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + POWER3(rcCommandAbs) * rcExpo;

    // Super rate
    float maxRate = fmaxf(sRate, rcRate);
    float superRate = 1.0f - (rcRate / maxRate);
    float superFactor = 1.0f / constrainf(1.0f - expof * superRate, 0.01f, 1.00f);

    // Final angle rate
    float angleRate = rcRate * rcCommandAbs * superFactor;

    return angleRate;
}


float applyRatesCurve(const int axis, const float rcCommandf)
{
    const float rcCommandAbs = fabsf(rcCommandf);
    float rate = 0;

    switch (currentControlRateProfile->rates_type)
    {
        case RATES_TYPE_BETAFLIGHT:
            rate = applyBetaflightRates(axis, rcCommandAbs);
            break;
        case RATES_TYPE_RACEFLIGHT:
            rate = applyRaceFlightRates(axis, rcCommandAbs);
            break;
        case RATES_TYPE_KISS:
            rate = applyKissRates(axis, rcCommandAbs);
            break;
        case RATES_TYPE_ACTUAL:
            rate = applyActualRates(axis, rcCommandAbs);
            break;
        case RATES_TYPE_QUICK:
            rate = applyQuickRates(axis, rcCommandAbs);
            break;
        default:
            rate = applyNullRates(axis, rcCommandAbs);
            break;
    }

    rate = fminf(rate, SETPOINT_RATE_LIMIT);
    rate = copysignf(rate, rcCommandf);

    // Collective is an angle - scale it here so that 480°/s => 12°
    if (axis == COLLECTIVE)
        rate *= 2.083333333f;

    return rate;
}


INIT_CODE void loadControlRateProfile(void)
{
    currentControlRateProfile = controlRateProfilesMutable(systemConfig()->activeRateProfile);

    setpointInitProfile();
}

INIT_CODE void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
}

INIT_CODE void copyControlRateProfile(uint8_t dstControlRateProfileIndex, uint8_t srcControlRateProfileIndex)
{
    if (dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        dstControlRateProfileIndex != srcControlRateProfileIndex)
    {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
