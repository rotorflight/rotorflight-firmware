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
#define RPM_UPDATE_BANK_COUNT       5

// Number of predefined presets
#define RPM_FILTER_PRESET_COUNT     3

// RPM Filter Presets
static const rpmNotchConfig_t rpmFilterPreset[RPM_FILTER_PRESET_COUNT] =
{
    // Low vibration preset
    {
        .notch_source = {
            { 11, 12, 21, },
            { 11, 12, 21, },
            { 11, 12, 21, },
        },
        .notch_center = {
            { 0, 0, 0, },
            { 0, 0, 0, },
            { 0, 0, 0, },
        },
        .notch_q = {
            { 100,  50, 50, },
            { 100,  50, 50, },
            { 100,  50, 50, },
        },
    },
    // Typical vibration preset
    {
        .notch_source = {
            { 11, 12, 13, 14, 21, },
            { 11, 12, 13, 14, 21, },
            { 11, 12, 13, 14, 21, },
        },
        .notch_center = {
            { 0, 0, 0, 0, 0, },
            { 0, 0, 0, 0, 0, },
            { 0, 0, 0, 0, 0, },
        },
        .notch_q = {
            { 80, 50, 80, 80, 50, },
            { 80, 50, 80, 80, 50, },
            { 80, 50, 80, 80, 50, },
        },
    },
    // High vibration preset
    {
        .notch_source = {
            { 11, 12, 13, 14, 15, 21, 22, 10 },
            { 11, 12, 13, 14, 15, 21, 22, 10 },
            { 11, 12, 13, 14, 15, 21, 22, 10 },
        },
        .notch_center = {
            { 0, 0, 0, 0, 0, 0, 0, 0, },
            { 0, 0, 0, 0, 0, 0, 0, 0, },
            { 0, 0, 0, 0, 0, 0, 0, 0, },
        },
        .notch_q = {
            { 50, 30, 80, 80, 80, 50, 80, 80 },
            { 50, 30, 80, 80, 80, 50, 80, 80 },
            { 80, 50, 80, 80, 80, 50, 80, 80 },
        },
    },
};

// Internal data
typedef struct
{
    uint8_t  motor;

    float    fader;
    float    ratio;
    float    notchQ;

    biquadFilter_t notch;

} rpmFilterBank_t;

FAST_DATA_ZERO_INIT static rpmFilterBank_t filterBank[XYZ_AXIS_COUNT][RPM_FILTER_NOTCH_COUNT];

FAST_DATA_ZERO_INIT static uint8_t updateAxisNumber;
FAST_DATA_ZERO_INIT static uint8_t updateBankNumber;
FAST_DATA_ZERO_INIT static uint8_t updateBankCount;
FAST_DATA_ZERO_INIT static uint8_t totalBankCount;

FAST_DATA_ZERO_INIT static float notchMaxHz;
FAST_DATA_ZERO_INIT static float notchMinHz;
FAST_DATA_ZERO_INIT static float notchFadeHz;


INIT_CODE void validateAndFixRPMFilterConfig(void)
{
    rpmFilterConfig_t *config = rpmFilterConfigMutable();
    rpmNotchConfig_t *custom = &config->custom;

    if (config->preset == 0) {
        for (int axis = 0; axis < RPM_FILTER_AXIS_COUNT; axis++) {
            for (int bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                if (custom->notch_source[axis][bank] == 0 ||
                    custom->notch_q[axis][bank] == 0)
                {
                    custom->notch_source[axis][bank] = 0;
                    custom->notch_center[axis][bank] = 0;
                    custom->notch_q[axis][bank] = 0;
                }
            }
        }
    }
    else if (config->preset <= RPM_FILTER_PRESET_COUNT) {
        const rpmNotchConfig_t *preset = &rpmFilterPreset[config->preset - 1];
        for (int axis = 0; axis < RPM_FILTER_AXIS_COUNT; axis++) {
            for (int bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                custom->notch_source[axis][bank] = preset->notch_source[axis][bank];
                custom->notch_center[axis][bank] = preset->notch_center[axis][bank];
                custom->notch_q[axis][bank] = preset->notch_q[axis][bank];
            }
        }
    }
    else {
        PG_RESET(rpmFilterConfig);
    }
}

INIT_CODE void rpmFilterInit(void)
{
    if (featureIsEnabled(FEATURE_RPM_FILTER))
    {
        const rpmFilterConfig_t *config = rpmFilterConfig();
        const rpmNotchConfig_t *notch = &config->custom;

        const int mainMotorIndex = 0;
        const int tailMotorIndex = mixerMotorizedTail() ? 1 : 0;

        const float mainGearRatio = getMainGearRatio();
        const float tailGearRatio = getTailGearRatio();

        // Main motor fundamental used if not direct-drive
        const bool enable10 = mainGearRatio != 1.0f;

        // Tail motor fundamental used if geared motorised tail and RPM available
        const bool enable20 = tailGearRatio != 1.0f && mixerMotorizedTail() && isMotorFastRpmSourceActive(tailMotorIndex);

        // Tail harmonics are not used if tail motor RPM unavailable
        const bool enable2x = !mixerMotorizedTail() || isMotorFastRpmSourceActive(tailMotorIndex);

        notchMaxHz = 0.45f * gyro.filterRateHz;
        notchMinHz = constrain(config->min_hz, 5, 0.5f * notchMaxHz);
        notchFadeHz = 1.2f * notchMinHz;

        #define CHECK_SOURCE(motor) if (!isMotorFastRpmSourceActive(motor)) goto error

        for (int axis = 0; axis < RPM_FILTER_AXIS_COUNT; axis++) {
            for (int bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                if (notch->notch_source[axis][bank] && notch->notch_q[axis][bank])
                {
                    rpmFilterBank_t *filter = &filterBank[axis][bank];

                    // RPM source for this bank
                    const unsigned source = notch->notch_source[axis][bank];

                    // Convert RPM to Hz plus center adjustment
                    const float ratio = (1.0f + notch->notch_center[axis][bank] / 10000.0f) / 60.0f;

                    // Q value
                    const float notchQ = constrainf(notch->notch_q[axis][bank], 10, 250) / 10;

                    // Main Motor (M1)
                    if (source == 10) {
                        if (enable10) {
                            CHECK_SOURCE(mainMotorIndex);
                            filter->motor  = mainMotorIndex;
                            filter->ratio  = ratio;
                            filter->notchQ = notchQ;
                        }
                    }
                    // Main Rotor harmonics
                    else if (source >= 11 && source <= 18) {
                        CHECK_SOURCE(mainMotorIndex);
                        const int harmonic = source - 10;
                        filter->motor  = mainMotorIndex;
                        filter->ratio  = mainGearRatio * harmonic * ratio;
                        filter->notchQ = notchQ;
                    }
                    // Tail Motor (M2)
                    else if (source == 20) {
                        if (enable20) {
                            CHECK_SOURCE(tailMotorIndex);
                            filter->motor  = tailMotorIndex;
                            filter->ratio  = ratio;
                            filter->notchQ = notchQ;
                        }
                    }
                    // Tail Rotor harmonics
                    else if (source >= 21 && source <= 28) {
                        if (enable2x) {
                            CHECK_SOURCE(tailMotorIndex);
                            const int harmonic = source - 20;
                            filter->motor  = tailMotorIndex;
                            filter->ratio  = tailGearRatio * harmonic * ratio;
                            filter->notchQ = notchQ;
                        }
                    }
                    else {
                        goto error;
                    }
                }
            }
        }

        // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            for (int bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                rpmFilterBank_t *filter = &filterBank[axis][bank];
                if (filter->notchQ) {
                    biquadFilterInit(&filter->notch, notchMinHz, gyro.filterRateHz, filter->notchQ, BIQUAD_NOTCH);
                    totalBankCount++;
                }
            }
        }

        // Number of banks to update in one update cycle
        updateBankCount = MIN(totalBankCount, RPM_UPDATE_BANK_COUNT);
    }

    return;

error:
    setArmingDisabled(ARMING_DISABLED_RPMFILTER);
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    if (totalBankCount) {
        for (int bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
            rpmFilterBank_t *filter = &filterBank[axis][bank];
            if (filter->notchQ) {
                value += (biquadFilterApplyDF1(&filter->notch, value) - value) * filter->fader;
            }
        }
    }
    return value;
}

void rpmFilterUpdate()
{
    if (totalBankCount)
    {
        // Actual update rate - allow ±25% variation
        const float updateRate = gyro.filterRateHz * constrainf(schedulerGetCycleTimeMultiplier(), 0.75f, 1.25f);

        // Number of banks to update in one update cycle
        for (int count = 0; count < updateBankCount;) {

            // Current filter
            rpmFilterBank_t *filter = &filterBank[updateAxisNumber][updateBankNumber];

            if (filter->notchQ) {

                // Calculate notch filter center frequency
                const float rpm = getMotorRPMf(filter->motor);
                const float freq = rpm * filter->ratio;
                const float center = constrainf(freq, 1, notchMaxHz);

                // Calculate fading
                filter->fader = transition(freq, notchMinHz, notchFadeHz, 0, 1);

                // Update the filter coefficients
                biquadFilterUpdate(&filter->notch, center, updateRate, filter->notchQ, BIQUAD_NOTCH);

                // Set debug if bank number matches
                if (debugAxis == updateAxisNumber * RPM_FILTER_NOTCH_COUNT + updateBankNumber) {
                    DEBUG(RPM_FILTER, 0, rpm);
                    DEBUG(RPM_FILTER, 1, freq * 10);
                    DEBUG(RPM_FILTER, 2, center * 10);
                    DEBUG(RPM_FILTER, 3, updateRate * 10);
                    DEBUG(RPM_FILTER, 4, filter->motor);
                    DEBUG(RPM_FILTER, 6, filter->fader * 1000);
                    DEBUG(RPM_FILTER, 7, filter->notchQ * 10);
                }

                // Bank updated
                count++;
            }

            // Next bank
            updateBankNumber = (updateBankNumber + 1) % RPM_FILTER_NOTCH_COUNT;
            if (updateBankNumber == 0)
                updateAxisNumber = (updateAxisNumber + 1) % RPM_FILTER_AXIS_COUNT;
        }
    }
}

#endif
