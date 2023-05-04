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

#include <math.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "scheduler/scheduler.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "drivers/dshot.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "pg/motor.h"

#include "rpm_filter.h"


typedef struct rpmFilterBank_s
{
    uint8_t  motorIndex;

    float    rpmRatio;
    float    minHz;
    float    maxHz;
    float    Q;

    biquadFilter_t notch[XYZ_AXIS_COUNT];

} rpmFilterBank_t;


FAST_DATA_ZERO_INIT static rpmFilterBank_t filterBank[RPM_FILTER_BANK_COUNT];

FAST_DATA_ZERO_INIT static uint8_t activeBankCount;
FAST_DATA_ZERO_INIT static uint8_t currentBank;


PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 4);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
        config->filter_bank_motor_index[i] = 0;
        config->filter_bank_gear_ratio[i]  = 0;
        config->filter_bank_notch_q[i]     = 250;
        config->filter_bank_min_hz[i]      = 20;
        config->filter_bank_max_hz[i]      = 4000;
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    const int mainMotorIndex = 1;
    const int tailMotorIndex = mixerMotorizedTail() ? 2 : 1;

    float mainGearRatio = getMainGearRatio();
    float tailGearRatio = getTailGearRatio();

    const bool enable1x = (getMotorCount() >= mainMotorIndex);
    const bool enable10 = (enable1x && mainGearRatio != 1.0f);
    const bool enable2x = (getMotorCount() >= tailMotorIndex);
    const bool enable20 = (enable2x && tailGearRatio != 1.0f);

    activeBankCount = 0;

    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++)
    {
        unsigned index = config->filter_bank_motor_index[bank];

        // For index=10..28 min_hz and max_hz are in RPM
        float minHz = config->filter_bank_min_hz[bank] / 60.0f;
        float maxHz = config->filter_bank_max_hz[bank] / 60.0f;

        rpmFilterBank_t *filt = &filterBank[activeBankCount];

        if (config->filter_bank_motor_index[bank] == 0 ||
            config->filter_bank_gear_ratio[bank] == 0 ||
            config->filter_bank_notch_q[bank] == 0)
            continue;

        // Manually configured filters
        if (index >= 1 && index <= getMotorCount()) {
            filt->motorIndex = index;
            filt->rpmRatio   = 1.0f / ((constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 1000) * 60);
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(config->filter_bank_min_hz[bank], 10, 1000);
            filt->maxHz      = constrainf(config->filter_bank_max_hz[bank], 100, 0.45e6f / gyro.filterLooptime);
            activeBankCount++;
        }
        // Motor#1 (main)
        else if (index == 10 && enable10) {
            filt->motorIndex = mainMotorIndex;
            filt->rpmRatio   = 1.0f / ((constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 10000) * 60);
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(minHz / mainGearRatio, 10, 1000);
            filt->maxHz      = constrainf(maxHz / mainGearRatio, 100, 0.45e6f / gyro.filterLooptime);
            activeBankCount++;
        }
        // Main rotor harmonics
        else if (index >= 11 && index <= 18 && enable1x) {
            unsigned harmonic = index - 10;
            filt->motorIndex = mainMotorIndex;
            filt->rpmRatio   = mainGearRatio * harmonic / ((constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 10000) * 60);
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(minHz * harmonic, 10, 1000);
            filt->maxHz      = constrainf(maxHz * harmonic, 100, 0.45e6f / gyro.filterLooptime);
            activeBankCount++;
        }
        // Motor#2 (tail)
        else if (index == 20 && enable20) {
            filt->motorIndex = tailMotorIndex;
            filt->rpmRatio   = 1.0f / ((constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 10000) * 60);
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(minHz / tailGearRatio, 10, 1000);
            filt->maxHz      = constrainf(maxHz / tailGearRatio, 100, 0.45e6f / gyro.filterLooptime);
            activeBankCount++;
        }
        // Tail rotor harmonics
        else if (index >= 21 && index <= 28 && enable2x) {
            unsigned harmonic = index - 20;
            filt->motorIndex = tailMotorIndex;
            filt->rpmRatio   = tailGearRatio * harmonic / ((constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 10000) * 60);
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(minHz * harmonic, 10, 1000);
            filt->maxHz      = constrainf(maxHz * harmonic, 100, 0.45e6f / gyro.filterLooptime);
            activeBankCount++;
        }
    }

    // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
    for (int bank = 0; bank < activeBankCount; bank++) {
        rpmFilterBank_t *filt = &filterBank[bank];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&filt->notch[axis], filt->minHz, gyro.filterRateHz, filt->Q, BIQUAD_NOTCH);
        }
    }
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    for (int bank=0; bank<activeBankCount; bank++) {
        value = biquadFilterApplyDF1(&filterBank[bank].notch[axis], value);
    }
    return value;
}

void rpmFilterUpdate()
{
    if (activeBankCount > 0) {

        // Adjust the loop rate to actual gyro speed
        float loopRate = gyro.filterRateHz * schedulerGetCycleTimeMultiplier();

        // Update one filter bank per cycle
        rpmFilterBank_t *filt = &filterBank[currentBank];

        // Calculate filter frequency
        float rpm  = getMotorRPM(filt->motorIndex - 1);
        float freq = constrainf(rpm * filt->rpmRatio, filt->minHz, filt->maxHz);

        // Notches for Roll,Pitch,Yaw
        biquadFilter_t *R = &filt->notch[0];
        biquadFilter_t *P = &filt->notch[1];
        biquadFilter_t *Y = &filt->notch[2];

        // Update the filter coefficients
        biquadFilterUpdate(R, freq, loopRate, filt->Q, BIQUAD_NOTCH);

        // Transfer the filter coefficients from Roll axis filter into Pitch and Yaw
        P->b0 = Y->b0 = R->b0;
        P->b1 = Y->b1 = R->b1;
        P->b2 = Y->b2 = R->b2;
        P->a1 = Y->a1 = R->a1;
        P->a2 = Y->a2 = R->a2;

        DEBUG(RPM_FILTER, 0, currentBank);
        DEBUG(RPM_FILTER, 1, filt->motorIndex);
        DEBUG(RPM_FILTER, 2, rpm);
        DEBUG(RPM_FILTER, 3, freq * 10);
        DEBUG(RPM_FILTER, 4, loopRate * 10);

        // Find next active bank - there must be at least one
        currentBank = (currentBank + 1) % activeBankCount;
    }
}

#endif

