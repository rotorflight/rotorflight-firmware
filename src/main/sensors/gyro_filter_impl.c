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

#include "platform.h"

static FAST_CODE void GYRO_FILTER_FUNCTION_NAME(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // DEBUG_GYRO_RAW records the raw value read from the sensor (not zero offset, not scaled)
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyro.rawSensorDev->gyroADCRaw[axis]);

        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        // If downsampling than the last value in the sample group will be output
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyro.gyroADC[axis]));

        // DEBUG_GYRO_SAMPLE(0) Record the pre-downsample value for the selected debug axis (same as DEBUG_GYRO_SCALED)
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 0, lrintf(gyro.gyroADC[axis]));

        // Downsampled (decimated) gyro signal
        float gyroADCf = gyro.gyroADCd[axis];

        // DEBUG_GYRO_SAMPLE(1) Record the post-downsample value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 1, lrintf(gyroADCf));

#ifdef USE_RPM_FILTER
        gyroADCf = rpmFilterGyro(axis, gyroADCf);
#endif

        // DEBUG_GYRO_SAMPLE(2) Record the post-RPM Filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 2, lrintf(gyroADCf));

        // apply static filters
        gyroADCf = filterApply(&gyro.lowpass2Filter[axis], gyroADCf);
        gyroADCf = filterApply(&gyro.lowpassFilter[axis], gyroADCf);

        // DEBUG_GYRO_SAMPLE(3) Record the post-LPF Filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 3, lrintf(gyroADCf));

        // apply notch filters
        gyroADCf = filterApply(&gyro.notchFilter2[axis], gyroADCf);
        gyroADCf = filterApply(&gyro.notchFilter1[axis], gyroADCf);

        // DEBUG_GYRO_SAMPLE(4) Record the post-Notch Filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 4, lrintf(gyroADCf));

#ifdef USE_DYN_NOTCH_FILTER
        if (isDynNotchActive()) {
            gyroADCf = dynNotchFilter(axis, gyroADCf);

            // DEBUG_GYRO_SAMPLE(5) Record the post-Dyn Notch Filter value for the selected debug axis
            GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 5, lrintf(gyroADCf));
        }
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

        gyro.gyroADCf[axis] = gyroADCf;
    }
}
