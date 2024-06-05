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

#include "pg/barometer.h"

#define BARO_SAMPLE_COUNT_MAX   48

typedef struct baro_s {
    baroDev_t dev;
    int32_t baroTemperature;
    int32_t baroPressure;
    int32_t baroAltitude;
} baro_t;

extern baro_t baro;

void baroPreInit(void);
bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse);
bool baroIsCalibrationComplete(void);
bool baroIsReady(void);
void baroStartCalibration(void);
void baroSetGroundLevel(void);
uint32_t baroUpdate(timeUs_t currentTimeUs);
