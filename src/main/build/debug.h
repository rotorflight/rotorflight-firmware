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

#include "platform.h"

#define DEBUG_VALUE_COUNT 8

extern uint8_t debugMode;
extern uint8_t debugAxis;

extern int32_t debug[DEBUG_VALUE_COUNT];

extern uint32_t __timing[DEBUG_VALUE_COUNT];

#define DEBUG_SET(mode, index, value)             do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_AXIS_SET(mode, axis, index, value)  do { if (debugAxis == (axis) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_COND_SET(mode, cond, index, value)  do { if ((cond) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)

#define DEBUG(mode, index, value)                 DEBUG_SET(DEBUG_ ## mode, index, value)
#define DEBUG_AXIS(mode, axis, index, value)      DEBUG_AXIS_SET(DEBUG_ ## mode, axis, index, value)
#define DEBUG_COND(mode, cond, index, value)      DEBUG_COND_SET(DEBUG_ ## mode, cond, index, value)

#define DEBUG_TIME_START(mode, index)             do { if (debugMode == (DEBUG_ ## mode)) { __timing[(index)] = micros(); } } while (0)
#define DEBUG_TIME_END(mode, index)               do { if (debugMode == (DEBUG_ ## mode)) { debug[(index)] = micros() - __timing[(index)]; } } while (0)


typedef enum {
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO_FILTERED,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_GYRO_SCALED,
    DEBUG_RC_INTERPOLATION,
    DEBUG_ANGLERATE,
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_RPM,
    DEBUG_ESC_SENSOR_TMP,
    DEBUG_ALTITUDE,
    DEBUG_FFT,
    DEBUG_FFT_TIME,
    DEBUG_FFT_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_RX_SFHSS_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_DUAL_GYRO_RAW,
    DEBUG_DUAL_GYRO_DIFF,
    DEBUG_MAX7456_SIGNAL,
    DEBUG_MAX7456_SPICLOCK,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_RANGEFINDER,
    DEBUG_RANGEFINDER_QUALITY,
    DEBUG_LIDAR_TF,
    DEBUG_ADC_INTERNAL,
    DEBUG_UNUSED_31,
    DEBUG_SDIO,
    DEBUG_CURRENT_SENSOR,
    DEBUG_USB,
    DEBUG_SMARTAUDIO,
    DEBUG_RTH,
    DEBUG_UNUSED_37,
    DEBUG_ACRO_TRAINER,
    DEBUG_UNUSED_39,
    DEBUG_RX_SIGNAL_LOSS,
    DEBUG_UNUSED_41,
    DEBUG_UNUSED_42,
    DEBUG_DYN_LPF,
    DEBUG_RX_SPEKTRUM_SPI,
    DEBUG_DSHOT_RPM_TELEMETRY,
    DEBUG_RPM_FILTER,
    DEBUG_RPM_SOURCE,
    DEBUG_UNUSED_48,
    DEBUG_UNUSED_49,
    DEBUG_DUAL_GYRO_SCALED,
    DEBUG_DSHOT_RPM_ERRORS,
    DEBUG_CRSF_LINK_STATISTICS_UPLINK,
    DEBUG_CRSF_LINK_STATISTICS_PWR,
    DEBUG_CRSF_LINK_STATISTICS_DOWN,
    DEBUG_BARO,
    DEBUG_GPS_RESCUE_THROTTLE_PID,
    DEBUG_FREQ_SENSOR,
    DEBUG_FEEDFORWARD_LIMIT,
    DEBUG_FEEDFORWARD,
    DEBUG_BLACKBOX_OUTPUT,
    DEBUG_GYRO_SAMPLE,
    DEBUG_RX_TIMING,
    DEBUG_D_LPF,
    DEBUG_VTX_TRAMP,
    DEBUG_GHST,
    DEBUG_SCHEDULER_DETERMINISM,
    DEBUG_TIMING_ACCURACY,
    DEBUG_RX_EXPRESSLRS_SPI,
    DEBUG_RX_EXPRESSLRS_PHASELOCK,
    DEBUG_RX_STATE_TIME,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];
