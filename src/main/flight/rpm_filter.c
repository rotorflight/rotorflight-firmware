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

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "flight/mixer.h"

#include "rpm_filter.h"

// Number of banks to update in one cycle
#define RPM_UPDATE_BANK_COUNT 1

typedef struct rpmFilterBank_s
{
    uint8_t  motor;

    float    ratio;
    float    minHz;
    float    maxHz;
    float    notchQ;

    biquadFilter_t notch[XYZ_AXIS_COUNT];

} rpmFilterBank_t;


FAST_DATA_ZERO_INIT static rpmFilterBank_t filterBank[RPM_FILTER_BANK_COUNT];

FAST_DATA_ZERO_INIT static uint8_t activeBankCount;
FAST_DATA_ZERO_INIT static uint8_t updateBankNumber;


INIT_CODE void rpmFilterInit(void)
{
    const rpmFilterConfig_t *config = rpmFilterConfig();

    const int mainMotorIndex = 0;
    const int tailMotorIndex = mixerMotorizedTail() ? 1 : 0;

    const float mainGearRatio = getMainGearRatio();
    const float tailGearRatio = getTailGearRatio();

    const bool enable10 = mainGearRatio != 1.0f;
    const bool enable20 = tailGearRatio != 1.0f && mixerMotorizedTail();

    int bankNumber = 0;

    #define CHECK_SOURCE(motor) if (!isMotorFastRpmSourceActive(motor)) goto error

    for (int index = 0; index < RPM_FILTER_BANK_COUNT; index++)
    {
        if (config->filter_bank_rpm_source[index] == 0 ||
            config->filter_bank_rpm_ratio[index] == 0 ||
            config->filter_bank_notch_q[index] == 0)
            continue;

        /*
         * NOTE!  rpm_limit has different meaning depending on rpm_source
         *
         *    1-4     Minimum RPM of the MOTOR
         *   10-18    Minimum RPM of the Main ROTOR
         *   20-28    Minimum RPM of the Tail ROTOR
         */

        rpmFilterBank_t *bank = &filterBank[bankNumber];

        // RPM source for this bank
        const unsigned source = config->filter_bank_rpm_source[index];

        // Ratio converts RPM to Hz
        const float ratio = 10000.0f / (constrain(config->filter_bank_rpm_ratio[index], 1, 50000) * 60.0f);

        // Absolute limits
        const float minHzLimit = 0.40f * gyro.filterRateHz;
        const float maxHzLimit = 0.45f * gyro.filterRateHz;

        // Q value
        const float notchQ = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;

        // Motor RPM based notches
        if (source >= 1 && source <= getMotorCount()) {
            CHECK_SOURCE(source - 1);
            bank->motor  = source - 1;
            bank->ratio  = ratio;
            bank->minHz  = constrainf(config->filter_bank_rpm_limit[index] * ratio, 10, minHzLimit);
            bank->maxHz  = maxHzLimit;
            bank->notchQ = notchQ;
            bankNumber++;
        }
        // Main Motor (M1)
        else if (source == 10) {
            if (enable10) {
                CHECK_SOURCE(mainMotorIndex);
                bank->motor  = mainMotorIndex;
                bank->ratio  = ratio;
                bank->minHz  = constrainf((config->filter_bank_rpm_limit[index] / mainGearRatio) * ratio, 10, minHzLimit);
                bank->maxHz  = maxHzLimit;
                bank->notchQ = notchQ;
                bankNumber++;
            }
        }
        // Main Rotor harmonics
        else if (source >= 11 && source <= 18) {
            CHECK_SOURCE(mainMotorIndex);
            const int harmonic = source - 10;
            bank->motor  = mainMotorIndex;
            bank->ratio  = mainGearRatio * harmonic * ratio;
            bank->minHz  = constrainf((config->filter_bank_rpm_limit[index] * harmonic) * ratio, 10, minHzLimit);
            bank->maxHz  = maxHzLimit;
            bank->notchQ = notchQ;
            bankNumber++;
        }
        // Tail Motor (M2)
        else if (source == 20) {
            if (enable20) {
                CHECK_SOURCE(tailMotorIndex);
                bank->motor  = tailMotorIndex;
                bank->ratio  = ratio;
                bank->minHz  = constrainf((config->filter_bank_rpm_limit[index] / tailGearRatio) * ratio, 10, minHzLimit);
                bank->maxHz  = maxHzLimit;
                bank->notchQ = notchQ;
                bankNumber++;
            }
        }
        // Tail Rotor harmonics
        else if (source >= 21 && source <= 28) {
            CHECK_SOURCE(tailMotorIndex);
            const int harmonic = source - 20;
            bank->motor  = tailMotorIndex;
            bank->ratio  = tailGearRatio * harmonic * ratio;
            bank->minHz  = constrainf((config->filter_bank_rpm_limit[index] * harmonic) * ratio, 10, minHzLimit);
            bank->maxHz  = maxHzLimit;
            bank->notchQ = notchQ;
            bankNumber++;
        }
        else {
            goto error;
        }
    }

    // Set activeBankCount to the number of configured notches
    activeBankCount = bankNumber;

    // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
    for (int index = 0; index < activeBankCount; index++) {
        rpmFilterBank_t *bank = &filterBank[index];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&bank->notch[axis], bank->minHz, gyro.filterRateHz, bank->notchQ, BIQUAD_NOTCH);
        }
    }

    return;

error:
    activeBankCount = 0;

    setArmingDisabled(ARMING_DISABLED_RPMFILTER);
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    for (int index = 0; index < activeBankCount; index++) {
        value = biquadFilterApplyDF1(&filterBank[index].notch[axis], value);
    }
    return value;
}

void rpmFilterUpdate()
{
    if (activeBankCount > 0) {

        // Actual update rate - allow ±25% variation
        const float updateRate = gyro.filterRateHz * constrainf(schedulerGetCycleTimeMultiplier(), 0.75f, 1.25f);

        // Number of banks to update in one update cycle
        for (int i = 0; i < RPM_UPDATE_BANK_COUNT; i++) {

            // Current filter bank
            rpmFilterBank_t *bank = &filterBank[updateBankNumber];

            // Calculate notch filter center frequency
            const float rpm = getMotorRPMf(bank->motor);
            const float freq = rpm * bank->ratio;
            const float notch = constrainf(freq, bank->minHz, bank->maxHz);

            // Notch filters for Roll,Pitch,Yaw
            biquadFilter_t *R = &bank->notch[0];
            biquadFilter_t *P = &bank->notch[1];
            biquadFilter_t *Y = &bank->notch[2];

            // Update the filter coefficients
            biquadFilterUpdate(R, notch, updateRate, bank->notchQ, BIQUAD_NOTCH);

            // Transfer the filter coefficients from Roll axis filter into Pitch and Yaw
            P->b0 = Y->b0 = R->b0;
            P->b1 = Y->b1 = R->b1;
            P->b2 = Y->b2 = R->b2;
            P->a1 = Y->a1 = R->a1;
            P->a2 = Y->a2 = R->a2;

            // Set debug if bank number matches
            if (updateBankNumber == debugAxis) {
                DEBUG(RPM_FILTER, 0, rpm);
                DEBUG(RPM_FILTER, 1, freq * 10);
                DEBUG(RPM_FILTER, 2, notch * 10);
                DEBUG(RPM_FILTER, 3, updateRate * 10);
                DEBUG(RPM_FILTER, 4, bank->motor);
                DEBUG(RPM_FILTER, 5, bank->minHz * 10);
                DEBUG(RPM_FILTER, 6, bank->maxHz * 10);
                DEBUG(RPM_FILTER, 7, bank->notchQ * 10);
            }

            // Next bank
            updateBankNumber = (updateBankNumber + 1) % activeBankCount;
        }
    }
}

#endif

