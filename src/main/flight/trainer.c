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

#ifdef USE_ACRO_TRAINER

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "trainer.h"


static FAST_RAM_ZERO_INIT bool acroTrainerActive;

static FAST_RAM_ZERO_INIT float acroTrainerGain;
static FAST_RAM_ZERO_INIT float acroTrainerAngleLimit;
static FAST_RAM_ZERO_INIT float acroTrainerLookaheadTime;

static FAST_RAM_ZERO_INIT int8_t acroTrainerAxisState[2];

static FAST_RAM_ZERO_INIT uint8_t acroTrainerDebugAxis;


void acroTrainerInit(const pidProfile_t *pidProfile)
{
    acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
    acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
}

void acroTrainerReset(void)
{
    acroTrainerAxisState[FD_ROLL] = 0;
    acroTrainerAxisState[FD_PITCH] = 0;
}

void acroTrainerSetState(bool state)
{
    if (acroTrainerActive != state) {
        acroTrainerReset();
        acroTrainerActive = state;
    }
}


static inline int acroTrainerSign(float x)
{
    return (x >= 0) ? 1 : -1;
}


// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
//
// There are three states:
//
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit
//    (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint
//

float acroTrainerApply(int axis, float setPoint)
{
    if (acroTrainerActive && axis != FD_YAW &&
        (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)))
    {
        const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);
        const int setpointSign = acroTrainerSign(setPoint);
        float projectedAngle = 0;

        // stick has reversed - stop limiting
        if (acroTrainerAxisState[axis] && acroTrainerAxisState[axis] != setpointSign) {
            acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if (!acroTrainerAxisState[axis] && fabsf(currentAngle) > acroTrainerAngleLimit) {
            if (angleSign == setpointSign) {
                acroTrainerAxisState[axis] = angleSign;
                pidData[axis].I = 0;
            }
        }

        if (acroTrainerAxisState[axis]) {
            setPoint = constrainf(((acroTrainerAngleLimit * angleSign) - currentAngle) * acroTrainerGain,
                                  -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        }
        else {
            // Not currently over the limit so project the angle based on current angle and
            // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
            // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f)
                * acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if (fabsf(projectedAngle) > acroTrainerAngleLimit && projectedAngleSign == setpointSign) {
                setPoint = ((acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * acroTrainerGain;
                pidData[axis].I = 0;
            }
        }

        if (axis == acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(setPoint));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return setPoint;
}

#endif
