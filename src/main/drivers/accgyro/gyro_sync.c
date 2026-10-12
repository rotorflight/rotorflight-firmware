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

#include "sensors/gyro.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/pwm_output.h"
#include "drivers/io.h"

#include "pg/gyrodev.h"

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

        case BMI_088_SPI:
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
                gyroRateKHz = GYRO_RATE_1_kHz;
                gyroSampleRateHz = 1000;
            } else
#endif
            {
                gyro->gyroRateKHz = GYRO_RATE_2000_Hz;
                gyroSampleRateHz = 2000;
            }
            accSampleRateHz = 800;
            break;

        case ICM_20649_SPI:
#if defined(STM32H7)
            gyroRateKHz = GYRO_RATE_9_kHz;
            gyroSampleRateHz = 9000;
            accSampleRateHz = 1125;
#else
            gyroRateKHz = GYRO_RATE_1100_Hz;
            gyroSampleRateHz = 1125;
            accSampleRateHz = 1125;
#endif
            break;

        case MPU_65xx_SPI:
        case MPU_9250_SPI:
        case ICM_20689_SPI:
#if defined(STM32H7)
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
#else
            gyroRateKHz = GYRO_RATE_1_kHz;
            gyroSampleRateHz = 1000;
            accSampleRateHz = 1000;
#endif
            break;

        case MPU_60x0_SPI:
        case ICM_42688P_SPI:
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
#elif defined(STM32F4) || defined(STM32F7)
            gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 4000;
            accSampleRateHz = 1000;
            gyroDivider = 2;
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


#if defined(USE_GYRO_CLK)

static pwmOutputPort_t pwmGyroClk = INIT_ZERO;

bool gyroExternalClockInit(const extDevice_t *dev, uint32_t clockFreq)
{
    const int cfg = 0; // Only on 1st gyro

    if (&gyro.gyroSensor1.gyroDev.dev != dev) {
        return false;
    }

    const ioTag_t tag = gyroDeviceConfig(cfg)->clkInTag;
    const IO_t io = IOGetByTag(tag);
    if (pwmGyroClk.enabled) {
       // pwm is already taken, but test for shared clkIn pin
       return pwmGyroClk.io == io;
    }

    const timerHardware_t *timer = timerAllocate(tag, OWNER_GYRO_CLK, RESOURCE_INDEX(cfg));
    if (!timer) {
        return false;
    }

    pwmGyroClk.io = io;
    pwmGyroClk.enabled = true;

    IOInit(io, OWNER_GYRO_CLK, RESOURCE_INDEX(cfg));
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

    const uint32_t clock = timerClock(timer->tim);
    const uint16_t period = clock / clockFreq;

    // Calculate duty cycle value for 50%
    const uint16_t cycle = period / 2;

    // Configure PWM output
    pwmOutConfig(&pwmGyroClk.channel, timer, clock, period - 1, cycle - 1, 0);

    // Set CCR value
    *pwmGyroClk.channel.ccr = cycle - 1;

    return true;
}

#endif
