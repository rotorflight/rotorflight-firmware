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
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gyrodev.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#ifdef USE_DYN_NOTCH_FILTER
#include "flight/dyn_notch_filter.h"
#endif
#include "flight/rpm_filter.h"
#include "flight/governor.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#if ((TARGET_FLASH_SIZE > 128) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6500)))
#define USE_GYRO_SLEW_LIMITER
#endif

FAST_DATA_ZERO_INIT gyro_t gyro;

static FAST_DATA_ZERO_INIT bool overflowDetected;
#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_DATA_ZERO_INIT timeUs_t overflowTimeUs;
#endif

static FAST_DATA_ZERO_INIT int16_t gyroSensorTemperature;

FAST_DATA uint8_t activePidLoopDenom = 1;
FAST_DATA uint8_t activeFilterLoopDenom = 1;

static bool firstArmingCalibrationWasStarted = false;

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyro.gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyro.gyroSensor1.gyroDev;
#endif


#define DEBUG_GYRO_CALIBRATION 3

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

PG_REGISTER_WITH_RESET_FN(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 9);

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif

void pgResetFn_gyroConfig(gyroConfig_t *gyroConfig)
{
    gyroConfig->gyroCalibrationDuration = 125;        // 1.25 seconds
    gyroConfig->gyroMovementCalibrationThreshold = 48;
    gyroConfig->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    gyroConfig->gyro_decimation_hz = 250;
    gyroConfig->gyro_lpf1_type = GYRO_LPF1_TYPE_DEFAULT;
    gyroConfig->gyro_lpf1_static_hz = GYRO_LPF1_HZ_DEFAULT;
    gyroConfig->gyro_lpf2_type = GYRO_LPF2_TYPE_DEFAULT;
    gyroConfig->gyro_lpf2_static_hz = GYRO_LPF2_HZ_DEFAULT;
    gyroConfig->gyro_lpf1_dyn_min_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
    gyroConfig->gyro_lpf1_dyn_max_hz = GYRO_LPF1_DYN_MAX_HZ_DEFAULT;
    gyroConfig->gyro_high_fsr = false;
    gyroConfig->gyro_rate_sync = true;
    gyroConfig->gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT;
    gyroConfig->gyro_soft_notch_hz_1 = 0;
    gyroConfig->gyro_soft_notch_cutoff_1 = 0;
    gyroConfig->gyro_soft_notch_hz_2 = 0;
    gyroConfig->gyro_soft_notch_cutoff_2 = 0;
    gyroConfig->checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;
    gyroConfig->gyro_offset_yaw = 0;
}

static inline bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

bool gyroIsCalibrationComplete(void)
{
    switch (gyro.gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor1);
        }
#ifdef USE_MULTI_GYRO
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor1) && isGyroSensorCalibrationComplete(&gyro.gyroSensor2);
        }
#endif
    }
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.sampleLooptime;
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
#if defined(USE_FAKE_GYRO) && !defined(UNIT_TEST)
    if (gyroSensor->gyroDev.gyroHardware == GYRO_FAKE) {
        gyroSensor->calibration.cyclesRemaining = 0;
        return;
    }
#endif
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (isFirstArmingCalibration && firstArmingCalibrationWasStarted) {
        return;
    }

    gyroSetCalibrationCycles(&gyro.gyroSensor1);
#ifdef USE_MULTI_GYRO
    gyroSetCalibrationCycles(&gyro.gyroSensor2);
#endif

    if (isFirstArmingCalibration) {
        firstArmingCalibrationWasStarted = true;
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !gyroIsCalibrationComplete();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            }

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }

    --gyroSensor->calibration.cyclesRemaining;
}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (gyroConfig()->checkOverflow || gyro.gyroHasOverflowProtection) {
        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
        return ret;
    }
    if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_CODE_NOINLINE void handleOverflow(timeUs_t currentTimeUs)
{
    // This will need to be revised if we ever allow different sensor types to be
    // used simultaneously. In that case the scale might be different between sensors.
    // It's complicated by the fact that we're using filtered gyro data here which is
    // after both sensors are scaled and averaged.
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyro.scale;

    if ((fabsf(gyro.gyroADCf[X]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Y]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Z]) < gyroOverflowResetRate)) {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, overflowTimeUs) > 50000) {
            overflowDetected = false;
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        overflowTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForOverflow(timeUs_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (overflowDetected) {
        handleOverflow(currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;

        // This will need to be revised if we ever allow different sensor types to be
        // used simultaneously. In that case the scale might be different between sensors.
        // It's complicated by the fact that we're using filtered gyro data here which is
        // after both sensors are scaled and averaged.
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyro.scale;

        if (fabsf(gyro.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (fabsf(gyro.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (fabsf(gyro.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & gyro.overflowAxisMask) {
            overflowDetected = true;
            overflowTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_GYRO_OVERFLOW_CHECK

static FAST_CODE void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        if (gyroSensor->gyroDev.gyroAlign == ALIGN_CUSTOM) {
            alignSensorViaMatrix(gyroSensor->gyroDev.gyroADC, &gyroSensor->gyroDev.rotationMatrix);
        } else {
            alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
        }
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
    }
}

FAST_CODE void gyroUpdate(void)
{
    switch (gyro.gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyro.gyroSensor1);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor1)) {
            gyro.gyroADC[X] = gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale;
            gyro.gyroADC[Y] = gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale;
            gyro.gyroADC[Z] = gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale;
        }
        break;
#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyro.gyroSensor2);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor2)) {
            gyro.gyroADC[X] = gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale;
            gyro.gyroADC[Y] = gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale;
            gyro.gyroADC[Z] = gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale;
        }
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyro.gyroSensor1);
        gyroUpdateSensor(&gyro.gyroSensor2);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor1) && isGyroSensorCalibrationComplete(&gyro.gyroSensor2)) {
            gyro.gyroADC[X] = ((gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
            gyro.gyroADC[Y] = ((gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
            gyro.gyroADC[Z] = ((gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
        }
        break;
#endif
    }

    gyro.gyroADCd[X] = filterStackApply(gyro.decimator[X], gyro.gyroADC[X], 2);
    gyro.gyroADCd[Y] = filterStackApply(gyro.decimator[Y], gyro.gyroADC[Y], 2);
    gyro.gyroADCd[Z] = filterStackApply(gyro.decimator[Z], gyro.gyroADC[Z], 2);
}

#define GYRO_FILTER_FUNCTION_NAME filterGyro
#define GYRO_FILTER_DEBUG_SET(mode, index, value)
#define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value)
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET
#undef GYRO_FILTER_AXIS_DEBUG_SET

#define GYRO_FILTER_FUNCTION_NAME filterGyroDebug
#define GYRO_FILTER_DEBUG_SET(mode, index, value) DEBUG_SET((mode), (index), (value))
#define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value) DEBUG_AXIS_SET((mode), (axis), (index), (value))
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET
#undef GYRO_FILTER_AXIS_DEBUG_SET

FAST_CODE void gyroFiltering(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (gyro.gyroDebugMode == DEBUG_NONE) {
        filterGyro();
    } else {
        filterGyroDebug();
    }

#ifdef USE_MULTI_GYRO
    if (gyro.useDualGyroDebugging) {
        switch (gyro.gyroToUse) {
        case GYRO_CONFIG_USE_GYRO_1:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
            break;
        case GYRO_CONFIG_USE_GYRO_2:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
            break;
        case GYRO_CONFIG_USE_GYRO_BOTH:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale)));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale)));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale)));
            break;
        }
    }
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyro.gyroHasOverflowProtection) {
        checkForOverflow(currentTimeUs);
    }
#endif
}

int16_t gyroReadSensorTemperature(gyroSensor_t gyroSensor)
{
    if (gyroSensor.gyroDev.temperatureFn) {
        gyroSensor.gyroDev.temperatureFn(&gyroSensor.gyroDev, &gyroSensor.gyroDev.temperature);
    }
    return gyroSensor.gyroDev.temperature;
}

void gyroReadTemperature(void)
{
    switch (gyro.gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroSensorTemperature = gyroReadSensorTemperature(gyro.gyroSensor1);
        break;

#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroSensorTemperature = gyroReadSensorTemperature(gyro.gyroSensor2);
        break;

    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroSensorTemperature = MAX(gyroReadSensorTemperature(gyro.gyroSensor1), gyroReadSensorTemperature(gyro.gyroSensor2));
        break;
#endif // USE_MULTI_GYRO
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroSensorTemperature;
}

bool gyroOverflowDetected(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    return overflowDetected;
#else
    return false;
#endif // USE_GYRO_OVERFLOW_CHECK
}

uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}

#ifdef USE_DYN_LPF

void dynLpfUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t lastDynLpfUpdateUs = 0;

    if (gyro.dynLpfFilter) {
        const float ratio = getFullHeadSpeedRatio();
        if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_UPDATE_DELAY_US) {
            const float cutoffFreq = constrainf(ratio * gyro.dynLpfHz, gyro.dynLpfMin, gyro.dynLpfMax);
            DEBUG_SET(DEBUG_DYN_LPF, 2, lrintf(cutoffFreq));
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                filterUpdate(&gyro.lowpassFilter[axis], cutoffFreq, gyro.filterRateHz);
            }
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}

#endif // USE_DYN_LPF
