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

#include "platform.h"

#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/initialisation.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

// requestedSensors is not actually used
uint8_t requestedSensors[SENSOR_INDEX_COUNT] = {GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE};
uint8_t detectedSensors[SENSOR_INDEX_COUNT] = {GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE};

void sensorsPreInit(void)
{
    gyroPreInit();

#ifdef USE_MAG
    compassPreInit();
#endif

#ifdef USE_BARO
    baroPreInit();
#endif
}

bool sensorsAutodetect(void)
{
    // extern void ch32_diag_blink(uint8_t count);

    // gyro must be initialised before accelerometer

    bool gyroDetected = gyroInit();
    // ch32_diag_blink(1); /* 1: gyroInit done */

#ifdef USE_ACC
    if (gyroDetected)
    {
        accInit(gyro.accSampleRateHz);
    }
#endif
    // ch32_diag_blink(2); /* 2: accInit done */

#ifdef USE_MAG
    compassInit();
#endif
    // ch32_diag_blink(3); /* 3: compassInit done */

#ifdef USE_BARO
    baroDetect(&baro.dev, barometerConfig()->baro_hardware);
#endif
    // ch32_diag_blink(4); /* 4: baroDetect done */

#ifdef USE_RANGEFINDER
    rangefinderInit();
#endif

#ifdef USE_ADC_INTERNAL
    adcInternalInit();
#endif
    // ch32_diag_blink(5); /* 5: adcInternalInit done */

    return gyroDetected;
}
