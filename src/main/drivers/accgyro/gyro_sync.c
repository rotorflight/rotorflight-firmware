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

/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

void gyroSetSampleRate(gyroDev_t *gyro)
{
    uint16_t gyroRateKHz = 0;
    uint16_t gyroDivider = 1;
    uint16_t gyroSampleRateHz = 0;
    uint16_t accSampleRateHz = 0;

    switch (gyro->mpuDetectionResult.sensor) {
#ifdef USE_ACCGYRO_LSM6DSO
        case LSM6DSO_SPI:
            gyroRateKHz = GYRO_RATE_6664_Hz;
            gyroSampleRateHz = 6664;   // Rounds to 150us and 6.67KHz
            accSampleRateHz = 833;
            break;
#endif

        case BMI_160_SPI:
            gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
            gyroSampleRateHz = 3200;
            accSampleRateHz = 800;
            break;

        case BMI_088_SPI:
            gyro->gyroRateKHz = GYRO_RATE_2000_Hz;
            gyroSampleRateHz = 2000;
            accSampleRateHz = 800;
            break;

        case BMI_270_SPI:
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
                // 6.4KHz sampling used when DLPF is disabled
                gyroRateKHz = GYRO_RATE_6400_Hz;
                gyroSampleRateHz = 6400;
            } else
#endif
            {
                gyroRateKHz = GYRO_RATE_3200_Hz;
                gyroSampleRateHz = 3200;
            }
            accSampleRateHz = 800;
            break;

        case ICM_20649_SPI:
#if defined(STM32F411xE) || defined(STM32G4)
            gyroRateKHz = GYRO_RATE_1100_Hz;
            gyroSampleRateHz = 1125;
            accSampleRateHz = 1125;
#else
            gyroRateKHz = GYRO_RATE_9_kHz;
            gyroSampleRateHz = 9000;
            accSampleRateHz = 1125;
#endif
            break;

        case ICM_20689_SPI:
        case MPU_9250_SPI:
        case MPU_65xx_SPI:
#if defined(STM32F411xE) || defined(STM32G4)
            gyroRateKHz = GYRO_RATE_1_kHz;
            gyroSampleRateHz = 1000;
            accSampleRateHz = 1000;
#else
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
#endif
            break;

        case ICM_42688P_SPI:
        case MPU_60x0_SPI:
#if defined(STM32F411xE)
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 1000;
            accSampleRateHz = 1000;
            gyroDivider = 8;
#elif defined(STM32G4)
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 2000;
            accSampleRateHz = 1000;
            gyroDivider = 4;
#else
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
#endif
            break;

        default:
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
            break;

    }

    gyro->gyroRateKHz = gyroRateKHz;
    gyro->mpuDividerDrops = gyroDivider - 1;
    gyro->gyroSampleRateHz = gyroSampleRateHz;
    gyro->accSampleRateHz = accSampleRateHz;
}
