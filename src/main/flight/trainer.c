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

#ifdef USE_ACRO_TRAINER

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/trainer.h"


#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f    // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f   // Limit the correcting setpoint

typedef int8_t sign_t;

typedef struct {
    bool        Active;
    float       Gain;
    float       AngleLimit;
    float       LookaheadTime;
    sign_t      AxisState[2];
} acroTrainer_t;

static FAST_DATA_ZERO_INIT acroTrainer_t acroTrainer;

int get_ADJUSTMENT_ACRO_TRAINER_GAIN(__unused int adjFunc)
{
    return currentPidProfile->trainer.gain;
}

void set_ADJUSTMENT_ACRO_TRAINER_GAIN(__unused int adjFunc, int value)
{
    currentPidProfile->trainer.gain = value;
    acroTrainer.Gain = value / 10.0f;
}

INIT_CODE void acroTrainerInit(const pidProfile_t *pidProfile)
{
    acroTrainer.Gain = pidProfile->trainer.gain / 10.0f;
    acroTrainer.AngleLimit = pidProfile->trainer.angle_limit;
    acroTrainer.LookaheadTime = pidProfile->trainer.lookahead_ms / 1000.0f;
}

void acroTrainerReset(void)
{
    acroTrainer.AxisState[FD_ROLL] = 0;
    acroTrainer.AxisState[FD_PITCH] = 0;
}

void acroTrainerSetState(bool state)
{
    if (acroTrainer.Active != state) {
        acroTrainerReset();
        acroTrainer.Active = state;
    }
}

static inline sign_t Sign(float x)
{
    return (x > 0) ? 1 : -1;
}

//
// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
//
// There are three states:
//
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint
//

float acroTrainerApply(int axis, float setPoint)
{
    if (acroTrainer.Active && (axis == FD_ROLL || axis == FD_PITCH))
    {
        const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const sign_t angleSign = Sign(currentAngle);
        const sign_t setpointSign = Sign(setPoint);
        float projectedAngle = 0;
        bool resetAxis = false;

        // stick has reversed - stop limiting
        if ((acroTrainer.AxisState[axis] != 0) && (acroTrainer.AxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            acroTrainer.AxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > acroTrainer.AngleLimit) && (acroTrainer.AxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                acroTrainer.AxisState[axis] = angleSign;
                resetAxis = true;
            }
        }

        if (acroTrainer.AxisState[axis] != 0) {
            setPoint = limitf((acroTrainer.AngleLimit * angleSign - currentAngle) * acroTrainer.Gain, ACRO_TRAINER_SETPOINT_LIMIT);
        }
        else {
            // Not currently over the limit so project the angle based on current angle and
            // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
            // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainer.LookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;

            const sign_t projectedAngleSign = Sign(projectedAngle);
            if ((fabsf(projectedAngle) > acroTrainer.AngleLimit) && (projectedAngleSign == setpointSign)) {
                setPoint = ((acroTrainer.AngleLimit * projectedAngleSign) - projectedAngle) * acroTrainer.Gain;
                resetAxis = true;
            }
        }

        if (resetAxis)
            pidResetAxisError(axis);

        DEBUG_AXIS(ACRO_TRAINER, axis, 0, currentAngle * 10);
        DEBUG_AXIS(ACRO_TRAINER, axis, 1, acroTrainer.AxisState[axis]);
        DEBUG_AXIS(ACRO_TRAINER, axis, 2, setPoint);
        DEBUG_AXIS(ACRO_TRAINER, axis, 3, projectedAngle * 10);
    }

    return setPoint;
}

#endif
