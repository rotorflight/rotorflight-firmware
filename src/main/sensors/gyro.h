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

#include "common/axis.h"
#include "common/filter.h"
#include "common/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#ifdef USE_GYRO_DATA_ANALYSE
#include "flight/gyroanalyse.h"
#endif

#include "flight/pid.h"

#include "pg/pg.h"

#define FILTER_FREQUENCY_MAX 4000 // maximum frequency for filter cutoffs (nyquist limit of 8K max sampling)

#define DYN_LPF_UPDATE_DELAY_US 5000

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
} gyroLowpassFilter_t;

typedef enum gyroDetectionFlags_e {
    GYRO_NONE_MASK = 0,
    GYRO_1_MASK = BIT(0),
#if defined(USE_MULTI_GYRO)
    GYRO_2_MASK = BIT(1),
    GYRO_ALL_MASK = (GYRO_1_MASK | GYRO_2_MASK),
    GYRO_IDENTICAL_MASK = BIT(7), // All gyros are of the same hardware type
#endif
} gyroDetectionFlags_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
} gyroSensor_t;

typedef struct gyro_s {
    uint16_t sampleRateHz;
    uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float scale;
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
    float gyroDtermADCf[XYZ_AXIS_COUNT]; // filtered gyro data for D-term
    uint8_t sampleCount;               // gyro sensor sample counter
    float sampleSum[XYZ_AXIS_COUNT];   // summed samples used for downsampling
    bool downsampleFilterEnabled;      // if true then downsample using gyro lowpass 2, otherwise use averaging

    gyroSensor_t gyroSensor1;
#ifdef USE_MULTI_GYRO
    gyroSensor_t gyroSensor2;
#endif

    gyroDev_t *rawSensorDev;           // pointer to the sensor providing the raw data for DEBUG_GYRO_RAW

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilterDynApplyFn;
    filterApplyFnPtr notchFilterDynApplyFn2;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];
    biquadFilter_t notchFilterDyn2[XYZ_AXIS_COUNT];

    // D-term filters
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];

    filterApplyFnPtr dtermLowpassApplyFn;
    gyroLowpassFilter_t dtermLowpassFilter[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    gyroLowpassFilter_t dtermLowpass2Filter[XYZ_AXIS_COUNT];

#ifdef USE_DYN_LPF
    uint8_t  dynLpfDtermFilter;
    uint16_t dynLpfDtermMin;
    uint16_t dynLpfDtermMax;
#endif

#ifdef USE_GYRO_DATA_ANALYSE
    gyroAnalyseState_t gyroAnalyseState;
#endif

    uint16_t accSampleRateHz;
    uint8_t gyroToUse;
    uint8_t gyroDebugMode;
    bool gyroHasOverflowProtection;
    bool useDualGyroDebugging;
    flight_dynamics_index_t gyroDebugAxis;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    uint8_t overflowAxisMask;
#endif

} gyro_t;

extern gyro_t gyro;
extern uint8_t activePidLoopDenom;

enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
};

enum {
    DYN_LPF_NONE = 0,
    DYN_LPF_PT1,
    DYN_LPF_BIQUAD
};

#define GYRO_CONFIG_USE_GYRO_1      0
#define GYRO_CONFIG_USE_GYRO_2      1
#define GYRO_CONFIG_USE_GYRO_BOTH   2

enum {
    FILTER_LOWPASS = 0,
    FILTER_LOWPASS2,
    FILTER_DTERM_LOWPASS,
    FILTER_DTERM_LOWPASS2,
};

typedef struct gyroConfig_s {

    uint8_t  gyrosDetected;                     // What gyros should detection be attempted for on startup. Automatically set on first startup.

    uint8_t  gyroMovementCalibrationThreshold;  // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint16_t gyroCalibrationDuration;           // Gyro calibration duration in 1/100 second

    uint8_t  gyro_hardware_lpf;
    uint8_t  gyro_high_fsr;
    uint8_t  gyro_to_use;
    uint8_t  checkOverflow;

    uint16_t gyro_lowpass_hz;
    uint16_t gyro_lowpass2_hz;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    int16_t  gyro_offset_yaw;

    // Lowpass primary/secondary
    uint8_t  gyro_lowpass_type;
    uint8_t  gyro_lowpass2_type;

    uint16_t dyn_lpf_gyro_min_hz;
    uint16_t dyn_lpf_gyro_max_hz;

    uint16_t dyn_notch_max_hz;
    uint8_t  dyn_notch_width_percent;
    uint16_t dyn_notch_q;
    uint16_t dyn_notch_min_hz;

    // D-term lowpass
    uint8_t   dterm_filter_type;              // Filter selection for dterm
    uint16_t  dterm_lowpass_hz;               // Delta Filter in hz
    uint8_t   dterm_filter2_type;             // Filter selection for 2nd dterm
    uint16_t  dterm_lowpass2_hz;              // Extra PT1 Filter on D in hz
    uint16_t  dterm_notch_hz;                 // Biquad dterm notch hz
    uint16_t  dterm_notch_cutoff;             // Biquad dterm notch low cutoff

    uint16_t  dyn_lpf_dterm_min_hz;           // Minimum freq for D-term LPF
    uint16_t  dyn_lpf_dterm_max_hz;           // Maximum freq for D-term LPF

    uint8_t  gyro_filter_debug_axis;

} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

void gyroUpdate(void);
void gyroFiltering(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulation);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool gyroIsCalibrationComplete(void);
void gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
bool gyroOverflowDetected(void);
uint16_t gyroAbsRateDps(int axis);
#ifdef USE_DYN_LPF
void dynLpfUpdate(timeUs_t currentTimeUs, float ratio);
#endif
#ifdef USE_GYRO_DATA_ANALYSE
bool isDynamicFilterActive(void);
#endif
