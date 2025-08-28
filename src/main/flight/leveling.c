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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#ifdef USE_ACC

#include "common/axis.h"
#include "common/filter.h"

#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/gps_rescue.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "leveling.h"

typedef struct {
    float Gain;
    float AngleLimit;
} level_t;

typedef struct {
    float Gain;
    float Transition;
    float CutoffDegrees;
    float FactorRatio;
    uint8_t TiltExpertMode;
} horizon_t;

static FAST_DATA_ZERO_INIT level_t level;
static FAST_DATA_ZERO_INIT horizon_t horizon;


INIT_CODE void levelingInit(const pidProfile_t *pidProfile)
{
    level.Gain = pidProfile->angle.level_strength / 10.0f;
    level.AngleLimit = pidProfile->angle.level_limit;

    horizon.Gain = pidProfile->horizon.level_strength / 10.0f;
    horizon.Transition = pidProfile->horizon.transition;
    horizon.TiltExpertMode = pidProfile->horizon.tilt_expert_mode;
    horizon.CutoffDegrees = (175 - pidProfile->horizon.tilt_effect) * 1.8f;
    horizon.FactorRatio = (100 - pidProfile->horizon.tilt_effect) * 0.01f;
}

int get_ADJUSTMENT_ANGLE_LEVEL_GAIN(__unused int adjFunc)
{
    return currentPidProfile->angle.level_strength;
}

void set_ADJUSTMENT_ANGLE_LEVEL_GAIN(__unused int adjFunc, int value)
{
    currentPidProfile->angle.level_strength = value;
    level.Gain = value / 10.0f;
}

int get_ADJUSTMENT_HORIZON_LEVEL_GAIN(__unused int adjFunc)
{
    return currentPidProfile->horizon.level_strength;
}

void set_ADJUSTMENT_HORIZON_LEVEL_GAIN(__unused int adjFunc, int value)
{
    currentPidProfile->horizon.level_strength = value;
    horizon.Gain = value / 10.0f;
}

// calculate the stick deflection while applying level mode expo
static inline float getLevelModeDeflection(uint8_t axis)
{
    float deflection = getDeflection(axis);

    if (axis < FD_YAW) {
        const float expof = currentControlRateProfile->levelExpo[axis] / 100.0f;
        deflection = POWER3(deflection) * expof + deflection * (1 - expof);
    }

    return deflection;
}

static float calcLevelErrorAngle(int axis)
{
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float angle = level.AngleLimit * getLevelModeDeflection(axis);

#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100.0f; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -level.AngleLimit, level.AngleLimit);

    float currentAngle = ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);

    if (isUpsidedown() && FLIGHT_MODE(HORIZON_MODE)) {
        switch (axis) {
            case FD_PITCH:
                currentAngle = -currentAngle;
                break;
            case FD_ROLL:
                currentAngle -= SIGN(currentAngle) * 180;
                break;
        }
    }

    float error = angle - currentAngle;

    return error;
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
static float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - fmaxf(fabsf(getLevelModeDeflection(FD_ROLL)), fabsf(getLevelModeDeflection(FD_PITCH)));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = fmaxf(abs(attitude.values.roll), abs(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (horizon.TiltExpertMode) {
        if (horizon.Transition > 0 && horizon.CutoffDegrees > 0) {
            // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((horizon.CutoffDegrees-currentInclination) / horizon.CutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  H_sensitivity value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / horizon.Transition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        }
        else  {
            // d_level=0 or horizon_tilt_effect>=175 means no leveling
            horizonLevelStrength = 0;
        }
    }
    else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (horizon.FactorRatio < 1.0f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180 - currentInclination) / 180 * (1.0f - horizon.FactorRatio) + horizon.FactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizon.Transition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizon.Transition;
        }
        // zero means no leveling
        if (sensitFact <= 0) {
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }

    return constrainf(horizonLevelStrength, 0, 1);
}

float angleModeApply(int axis, float pidSetpoint)
{
    if (axis == FD_ROLL || axis == FD_PITCH)
    {
        float errorAngle = calcLevelErrorAngle(axis);

        if (!isAirborne())
            errorAngle *= 0.25f;

        pidSetpoint = errorAngle * level.Gain;
    }

    return pidSetpoint;
}

float horizonModeApply(int axis, float pidSetpoint)
{
    if (axis == FD_ROLL || axis == FD_PITCH)
    {
        float errorAngle = calcLevelErrorAngle(axis);

        if (!isAirborne())
            errorAngle *= 0.25f;

        pidSetpoint += errorAngle * horizon.Gain * calcHorizonLevelStrength();
    }

    return pidSetpoint;
}

#endif // USE_ACC
