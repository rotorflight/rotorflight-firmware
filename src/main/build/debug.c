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

#include <stdint.h>

#include "debug.h"

FAST_RAM_ZERO_INIT uint8_t debugMode;
FAST_RAM_ZERO_INIT int16_t debug[DEBUG16_VALUE_COUNT];

#ifdef USE_DEBUG32
FAST_RAM_ZERO_INIT int32_t debug32[DEBUG32_VALUE_COUNT];
#endif

#ifdef DEBUG_SECTION_TIMES
uint32_t sectionTimes[2][4];
#endif

#define DEBUG_NAME(x)  [DEBUG_ ## x] = #x

// Please ensure that names listed here match the enum values defined in 'debug.h'
const char * const debugModeNames[DEBUG_COUNT] = {
    DEBUG_NAME(NONE),
    DEBUG_NAME(CYCLETIME),
    DEBUG_NAME(BATTERY),
    DEBUG_NAME(GYRO_FILTERED),
    DEBUG_NAME(ACCELEROMETER),
    DEBUG_NAME(PIDLOOP),
    DEBUG_NAME(GYRO_SCALED),
    DEBUG_NAME(RC_INTERPOLATION),
    DEBUG_NAME(ANGLERATE),
    DEBUG_NAME(ESC_SENSOR),
    DEBUG_NAME(SCHEDULER),
    DEBUG_NAME(STACK),
    DEBUG_NAME(ESC_SENSOR_RPM),
    DEBUG_NAME(ESC_SENSOR_TMP),
    DEBUG_NAME(ALTITUDE),
    DEBUG_NAME(FFT),
    DEBUG_NAME(FFT_TIME),
    DEBUG_NAME(FFT_FREQ),
    DEBUG_NAME(RX_FRSKY_SPI),
    DEBUG_NAME(RX_SFHSS_SPI),
    DEBUG_NAME(GYRO_RAW),
    DEBUG_NAME(DUAL_GYRO_RAW),
    DEBUG_NAME(DUAL_GYRO_DIFF),
    DEBUG_NAME(MAX7456_SIGNAL),
    DEBUG_NAME(MAX7456_SPICLOCK),
    DEBUG_NAME(SBUS),
    DEBUG_NAME(FPORT),
    DEBUG_NAME(RANGEFINDER),
    DEBUG_NAME(RANGEFINDER_QUALITY),
    DEBUG_NAME(LIDAR_TF),
    DEBUG_NAME(ADC_INTERNAL),
    DEBUG_NAME(GOVERNOR),
    DEBUG_NAME(SDIO),
    DEBUG_NAME(CURRENT_SENSOR),
    DEBUG_NAME(USB),
    DEBUG_NAME(SMARTAUDIO),
    DEBUG_NAME(RTH),
    DEBUG_NAME(ITERM_RELAX),
    DEBUG_NAME(ACRO_TRAINER),
    DEBUG_NAME(RC_SMOOTHING),
    DEBUG_NAME(RX_SIGNAL_LOSS),
    DEBUG_NAME(RC_SMOOTHING_RATE),
    DEBUG_NAME(UNUSED_42),
    DEBUG_NAME(DYN_LPF),
    DEBUG_NAME(RX_SPEKTRUM_SPI),
    DEBUG_NAME(DSHOT_RPM_TELEMETRY),
    DEBUG_NAME(RPM_FILTER),
    DEBUG_NAME(RPM_SOURCE),
    DEBUG_NAME(AC_CORRECTION),
    DEBUG_NAME(AC_ERROR),
    DEBUG_NAME(DUAL_GYRO_SCALED),
    DEBUG_NAME(DSHOT_RPM_ERRORS),
    DEBUG_NAME(CRSF_LINK_STATISTICS_UPLINK),
    DEBUG_NAME(CRSF_LINK_STATISTICS_PWR),
    DEBUG_NAME(CRSF_LINK_STATISTICS_DOWN),
    DEBUG_NAME(BARO),
    DEBUG_NAME(GPS_RESCUE_THROTTLE_PID),
    DEBUG_NAME(FREQ_SENSOR),
    DEBUG_NAME(FF_LIMIT),
    DEBUG_NAME(FF_INTERPOLATED),
    DEBUG_NAME(BLACKBOX_OUTPUT),
    DEBUG_NAME(GYRO_SAMPLE),
    DEBUG_NAME(RX_TIMING),
};
