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


FAST_RAM_ZERO_INIT static rpmFilterBank_t filterBank[RPM_FILTER_BANK_COUNT];

FAST_RAM_ZERO_INIT static uint8_t activeBankCount;
FAST_RAM_ZERO_INIT static uint8_t currentBank;


PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 4);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
        config->filter_bank_motor_index[i] = 0;
        config->filter_bank_gear_ratio[i]  = 1000;
        config->filter_bank_notch_q[i]     = 250;
        config->filter_bank_min_hz[i]      = 20;
        config->filter_bank_max_hz[i]      = 4000;
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++) {
        if (config->filter_bank_motor_index[bank] > 0 && config->filter_bank_motor_index[bank] <= getMotorCount()) {
            rpmFilterBank_t *filt = &filterBank[bank];

            // Force bank config into reasonable limits
            filt->motorIndex = config->filter_bank_motor_index[bank];
            filt->rpmRatio   = constrainf(config->filter_bank_gear_ratio[bank], 1, 50000) / 1000 * 60;
            filt->Q          = constrainf(config->filter_bank_notch_q[bank], 10, 10000) / 100;
            filt->minHz      = constrainf(config->filter_bank_min_hz[bank], 20, 1000);
            filt->maxHz      = constrainf(config->filter_bank_max_hz[bank], 100, 0.45e6 / gyro.targetLooptime);

            // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInit(&filt->notch[axis], filt->minHz, gyro.targetLooptime, filt->Q, FILTER_NOTCH);
            }

            currentBank = bank;
            activeBankCount++;
        }
    }
}

FAST_CODE_NOINLINE float rpmFilterGyro(int axis, float value)
{
    if (activeBankCount > 0) {
        for (int bank=0; bank<RPM_FILTER_BANK_COUNT; bank++) {
            if (filterBank[bank].motorIndex) {
                value = biquadFilterApplyDF1(&filterBank[bank].notch[axis], value);
            }
        }
    }
    return value;
}

void rpmFilterUpdate()
{
    if (activeBankCount > 0) {

        // Update one filter bank per cycle
        rpmFilterBank_t *filt = &filterBank[currentBank];

        // Calculate filter frequency
        float rpm  = getMotorRPM(filt->motorIndex - 1);
        float freq = constrainf(rpm / filt->rpmRatio, filt->minHz, filt->maxHz);

        // Notches for Roll,Pitch,Yaw
        biquadFilter_t *R = &filt->notch[0];
        biquadFilter_t *P = &filt->notch[1];
        biquadFilter_t *Y = &filt->notch[2];

        // Update the filter coefficients
        biquadFilterUpdate(R, freq, gyro.targetLooptime, filt->Q, FILTER_NOTCH);

        // Transfer the filter coefficients from Roll axis filter into Pitch and Yaw
        P->b0 = Y->b0 = R->b0;
        P->b1 = Y->b1 = R->b1;
        P->b2 = Y->b2 = R->b2;
        P->a1 = Y->a1 = R->a1;
        P->a2 = Y->a2 = R->a2;

        DEBUG_SET(DEBUG_RPM_FILTER, 0, currentBank);
        DEBUG_SET(DEBUG_RPM_FILTER, 1, filt->motorIndex);
        DEBUG_SET(DEBUG_RPM_FILTER, 2, rpm);
        DEBUG_SET(DEBUG_RPM_FILTER, 3, freq);

        // Find next active bank - there must be at least one
        do {
            currentBank = (currentBank + 1) % RPM_FILTER_BANK_COUNT;
        } while (filterBank[currentBank].motorIndex == 0);
    }
}

#endif

