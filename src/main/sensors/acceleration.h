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

#pragma once

#include "common/time.h"
#include "drivers/accgyro/accgyro.h"
#include "fc/rc_adjustments.h"
#include "sensors/sensors.h"
#include "pg/accel.h"

// Type of accelerometer used/detected
typedef struct acc_s {
    accDev_t dev;
    uint16_t sampleRateHz;
    float accADC[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

extern acc_t acc;

bool accInit(uint16_t accSampleRateHz);
bool accIsCalibrationComplete(void);
bool accHasBeenCalibrated(void);
void accStartCalibration(void);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims);
bool accGetAccumulationAverage(float *accumulation);
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse);
void accInitFilters(void);
void applyAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta);

ADJFUN_DECLARE(ACC_TRIM_PITCH)
ADJFUN_DECLARE(ACC_TRIM_ROLL)
