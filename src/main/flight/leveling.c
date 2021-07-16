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

#include "build/build_config.h"
#include "build/debug.h"

#ifdef USE_ACC

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_rescue.h"

#include "leveling.h"


static FAST_RAM_ZERO_INIT float levelGain;
static FAST_RAM_ZERO_INIT float levelAngleLimit;

static FAST_RAM_ZERO_INIT float horizonGain;
static FAST_RAM_ZERO_INIT float horizonTransition;
static FAST_RAM_ZERO_INIT float horizonCutoffDegrees;
static FAST_RAM_ZERO_INIT float horizonFactorRatio;

static FAST_RAM_ZERO_INIT uint8_t horizonTiltExpertMode;

static FAST_RAM_ZERO_INIT float rescueCollective;


void pidLevelInit(const pidProfile_t *pidProfile)
{
    levelGain = pidProfile->angle_level_strength / 10.0f;
    levelAngleLimit = pidProfile->angle_level_limit;

    horizonGain = pidProfile->horizon_level_strength / 10.0f;
    horizonTransition = pidProfile->horizon_transition;
    horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;

    rescueCollective = pidProfile->rescue_collective / 1000.0f;
}

float pidRescueCollective(void)
{
    float collective = 0;

    // attitude.values.roll/pitch = 0 when level, 1800 when fully inverted (decidegrees)
    const float absRoll = fabsf(attitude.values.roll / 900.0f);
    const float absPitch = fabsf(attitude.values.pitch / 900.0f);

    // Pitch is +90/-90 at straight down and straight up. Convert it so that level = 1.0
    const float pitchCurrentInclination = 1.0f - absPitch;

    // Roll is +90/-90 when sideways, and +180/-180 when inverted
    const float rollCurrentInclination = (absRoll < 1.0f) ?  1.0f - absRoll : -1.0f + absRoll;

    // Smaller of the two
    const float vertCurrentInclination = MIN(pitchCurrentInclination, rollCurrentInclination);

    // Add more pitch as the heli approaches level
    // vertCurrentInclination is between 0.0 (vertical) and 1.0 (level)
    if ( absRoll < 1.0f ) {
        // We're closer to upright. Use positive collective pitch.
        collective = rescueCollective * (vertCurrentInclination * vertCurrentInclination);
    } else {
        // We're closer to inverted. Use negative collective pitch.
        collective = -rescueCollective * (vertCurrentInclination * vertCurrentInclination);
    }

    return collective;
}

static float calcRescueErrorAngle(int axis)
{
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    const float roll = (attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) / 10.0f;
    const float angle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
    float error = 0;

    if (roll > 90) {
        // Rolled right closer to inverted, continue to roll right to inverted (+180 degrees)
        if (axis == FD_PITCH) {
            error = angle;
        } else if (axis == FD_ROLL) {
            error = 180.0f - angle;
        }
    } else if (roll < -90) {
        // Rolled left closer to inverted, continue to roll left to inverted (-180 degrees)
        if (axis == FD_PITCH) {
            error = angle;
        } else if (axis == FD_ROLL) {
            error = -180.0f - angle;
        }
    } else {
        // We're rolled left or right between -90 and 90, and thus are closer to up-right
        if (axis == FD_PITCH) {
            error = -angle;
        } else if (axis == FD_ROLL) {
            error = -angle;
        }
    }

    return error;
}

static float calcLevelErrorAngle(int axis)
{
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float angle = levelAngleLimit * getRcDeflection(axis);

#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100.0f; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -levelAngleLimit, levelAngleLimit);

    float error = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);

    return error;
}

static float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (horizonTiltExpertMode) {
        if (horizonTransition > 0 && horizonCutoffDegrees > 0) {
            // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((horizonCutoffDegrees-currentInclination) / horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  H_sensitivity value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  {
            // d_level=0 or horizon_tilt_effect>=175 means no leveling
            horizonLevelStrength = 0;
        }
    } else {
        float sensitFact;
        if (horizonFactorRatio < 1.01f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180-currentInclination)/180 * (1.0f-horizonFactorRatio) + horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonTransition;
        }
        if (sensitFact <= 0) {           // zero means no leveling
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

float pidLevelApply(int axis, float currentPidSetpoint)
{
    float errorAngle = 0;

    if (FLIGHT_MODE(RESCUE_MODE)) {
        errorAngle = calcRescueErrorAngle(axis);
    } else {
        errorAngle = calcLevelErrorAngle(axis);
    }

    if (FLIGHT_MODE(ANGLE_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        // Angle based control
        currentPidSetpoint = errorAngle * levelGain;
    }
    else if (FLIGHT_MODE(HORIZON_MODE)) {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        currentPidSetpoint += errorAngle * horizonGain * calcHorizonLevelStrength();
    }

    return currentPidSetpoint;
}

#endif // USE_ACC
