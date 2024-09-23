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
#define RPM_UPDATE_BANK_COUNT 2

typedef struct rpmFilterBank_s
{
    uint8_t  motor;

    float    ratio;
    float    minHz;
    float    maxHz;
    float    notchQ;

    biquadFilter_t notch;

} rpmFilterBank_t;


FAST_DATA_ZERO_INIT static rpmFilterBank_t filterBank[XYZ_AXIS_COUNT][RPM_FILTER_BANK_COUNT];

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

    #define CHECK_SOURCE(motor) if (!isMotorFastRpmSourceActive(motor)) goto error

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++)
    {
        const int index = axis * RPM_FILTER_BANK_COUNT + bank;

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

        rpmFilterBank_t *filter = &filterBank[axis][bank];

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
            filter->motor  = source - 1;
            filter->ratio  = ratio;
            filter->minHz  = constrainf(config->filter_bank_rpm_limit[index] * ratio, 10, minHzLimit);
            filter->maxHz  = maxHzLimit;
            filter->notchQ = notchQ;
        }
        // Main Motor (M1)
        else if (source == 10) {
            if (enable10) {
                CHECK_SOURCE(mainMotorIndex);
                filter->motor  = mainMotorIndex;
                filter->ratio  = ratio;
                filter->minHz  = constrainf((config->filter_bank_rpm_limit[index] / mainGearRatio) * ratio, 10, minHzLimit);
                filter->maxHz  = maxHzLimit;
                filter->notchQ = notchQ;
            }
        }
        // Main Rotor harmonics
        else if (source >= 11 && source <= 18) {
            CHECK_SOURCE(mainMotorIndex);
            const int harmonic = source - 10;
            filter->motor  = mainMotorIndex;
            filter->ratio  = mainGearRatio * harmonic * ratio;
            filter->minHz  = constrainf((config->filter_bank_rpm_limit[index] * harmonic) * ratio, 10, minHzLimit);
            filter->maxHz  = maxHzLimit;
            filter->notchQ = notchQ;
        }
        // Tail Motor (M2)
        else if (source == 20) {
            if (enable20) {
                CHECK_SOURCE(tailMotorIndex);
                filter->motor  = tailMotorIndex;
                filter->ratio  = ratio;
                filter->minHz  = constrainf((config->filter_bank_rpm_limit[index] / tailGearRatio) * ratio, 10, minHzLimit);
                filter->maxHz  = maxHzLimit;
                filter->notchQ = notchQ;
            }
        }
        // Tail Rotor harmonics
        else if (source >= 21 && source <= 28) {
            CHECK_SOURCE(tailMotorIndex);
            const int harmonic = source - 20;
            filter->motor  = tailMotorIndex;
            filter->ratio  = tailGearRatio * harmonic * ratio;
            filter->minHz  = constrainf((config->filter_bank_rpm_limit[index] * harmonic) * ratio, 10, minHzLimit);
            filter->maxHz  = maxHzLimit;
            filter->notchQ = notchQ;
        }
        else {
            goto error;
        }
    }

    // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++) {
        rpmFilterBank_t *filter = &filterBank[axis][bank];
        if (filter->notchQ) {
            biquadFilterInit(&filter->notch, filter->minHz, gyro.filterRateHz, filter->notchQ, BIQUAD_NOTCH);
        }
    }

    return;

error:
    setArmingDisabled(ARMING_DISABLED_RPMFILTER);
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++) {
        rpmFilterBank_t *filter = &filterBank[axis][bank];
        if (filter->notchQ) {
            value = biquadFilterApplyDF1(&filter->notch, value);
        }
    }
    return value;
}

void rpmFilterUpdate()
{
    // Actual update rate - allow ±10% variation
    const float updateRate = gyro.filterRateHz * constrainf(schedulerGetCycleTimeMultiplier(), 0.9f, 1.1f);

    // Number of banks to update in one update cycle
    for (int i = 0; i < RPM_UPDATE_BANK_COUNT; i++)
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        // Current filter bank
        rpmFilterBank_t *filter = &filterBank[axis][updateBankNumber];

        if (filter->notchQ) {

            // Calculate notch filter center frequency
            const float rpm = getMotorRPMf(filter->motor);
            const float freq = rpm * filter->ratio;
            const float notch = constrainf(freq, filter->minHz, filter->maxHz);

            // Update the filter coefficients
            biquadFilterUpdate(&filter->notch, notch, updateRate, filter->notchQ, BIQUAD_NOTCH);

            // Set debug if bank number matches
            if (debugAxis == axis * RPM_FILTER_BANK_COUNT + updateBankNumber) {
                DEBUG(RPM_FILTER, 0, rpm);
                DEBUG(RPM_FILTER, 1, freq * 10);
                DEBUG(RPM_FILTER, 2, notch * 10);
                DEBUG(RPM_FILTER, 3, updateRate * 10);
                DEBUG(RPM_FILTER, 4, filter->motor);
                DEBUG(RPM_FILTER, 5, filter->minHz * 10);
                DEBUG(RPM_FILTER, 6, filter->maxHz * 10);
                DEBUG(RPM_FILTER, 7, filter->notchQ * 10);
            }
        }

        // Next bank
        updateBankNumber = (updateBankNumber + 1) % RPM_FILTER_BANK_COUNT;
    }
}

#endif

