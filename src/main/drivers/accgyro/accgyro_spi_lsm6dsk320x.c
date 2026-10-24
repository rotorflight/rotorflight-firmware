/*
 * This file is part of Betaflight and Rotorflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// LSM6DSK320X port from Betaflight accgyro_spi_lsm6dsv16x.c.
// Upstream revision: 805313c231e479f226689bfa27e2609580543418.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_LSM6DSK320X

#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsk320x.h"
#include "drivers/bus_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#define LSM6DSK320X_MAX_SPI_CLK_HZ 10000000
#define LSM6DSK320X_RESET_TIMEOUT_MS 20
#define LSM6DSK320X_INIT_ATTEMPTS 3
#define GYRO_EXTI_DETECT_THRESHOLD 1000
#define LSM6DSV_ENCODE_BITS(val, mask, shift)   (((val) << (shift)) & (mask))
#define LSM6DSV_INT1_CTRL                   0x0D
#define LSM6DSV_INT1_CTRL_INT1_DRDY_G                   0x02
#define LSM6DSV_WHO_AM_I                    0x0F
#define LSM6DSV_CTRL1                       0x10
#define LSM6DSV_CTRL1_OP_MODE_XL_MASK                   0x70
#define LSM6DSV_CTRL1_OP_MODE_XL_SHIFT                  4
#define LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY          1
#define LSM6DSV_CTRL1_ODR_XL_MASK                       0x0f
#define LSM6DSV_CTRL1_ODR_XL_SHIFT                      0
#define LSM6DSV_CTRL1_ODR_XL_1000HZ                     9
#define LSM6DSV_CTRL2                       0x11
#define LSM6DSV_CTRL2_OP_MODE_G_MASK                    0x70
#define LSM6DSV_CTRL2_OP_MODE_G_SHIFT                   4
#define LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY           1
#define LSM6DSV_CTRL2_ODR_G_MASK                        0x0f
#define LSM6DSV_CTRL2_ODR_G_SHIFT                       0
#define LSM6DSV_CTRL2_ODR_G_8000HZ                      12
#define LSM6DSV_CTRL3                       0x12
#define LSM6DSV_CTRL3_BDU                               0x40
#define LSM6DSV_CTRL3_IF_INC                            0x04
#define LSM6DSV_CTRL3_SW_RESET                          0x01
#define LSM6DSV_CTRL4                       0x13
#define LSM6DSV_CTRL4_DRDY_PULSED                       0x02
#define LSM6DSV_CTRL6                       0x15
#define LSM6DSV_CTRL6_LPF1_G_BW_MASK                    0x70 // See table 64
#define LSM6DSV_CTRL6_LPF1_G_BW_SHIFT                   4
#define LSM6DSV_CTRL6_FS_G_BW_288HZ                     0
#define LSM6DSV_CTRL6_FS_G_BW_215HZ                     1
#define LSM6DSV_CTRL6_FS_G_BW_157HZ                     2
#define LSM6DSV_CTRL6_FS_G_BW_455HZ                     3
#define LSM6DSV_CTRL6_FS_G_MASK                         0x0f
#define LSM6DSV_CTRL6_FS_G_SHIFT                        0
#define LSM6DSV_CTRL6_FS_G_2000DPS                      0x04
#define LSM6DSK320X_CTRL6_FS_G_BIT3                     0x08 // FS_G is only bits [2:0] on this part; bit 3 must be 1
#define LSM6DSV_CTRL7                       0x16
#define LSM6DSV_CTRL7_LPF1_G_EN                         0x01
#define LSM6DSV_CTRL8                       0x17
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_MASK              0xe0 // See table 69
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_SHIFT             5
#define LSM6DSV_CTRL8_FS_XL_MASK                        0x03
#define LSM6DSV_CTRL8_FS_XL_SHIFT                       0
#define LSM6DSV_CTRL8_FS_XL_16G                         3
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_4                   0
#define LSM6DSV_CTRL9                       0x18
#define LSM6DSV_CTRL9_LPF2_XL_EN                        0x08
#define LSM6DSV_OUTX_L_G                    0x22
#define LSM6DSV_OUTX_L_A                    0x28
#define LSM6DSV_HAODR_CFG                   0x62
#define LSM6DSV_HAODR_CFG_HAODR_SEL_MASK                    0x03
#define LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT                   0
#define LSM6DSV_HAODR_MODE1                                 1

uint8_t lsm6dsk320xSpiDetect(const extDevice_t *dev)
{
    const uint8_t who_am_i = spiReadRegMsk(dev, LSM6DSV_WHO_AM_I);

    if (who_am_i != LSM6DSK320X_WHO_AM_I_CONST) {
        return MPU_NONE;
    }

    return LSM6DSK320X_SPI;
}

static void lsm6dsk320xAccInit(accDev_t *acc)
{
    // ±16G mode
    acc->acc_1G = 512 * 4;
}

static bool lsm6dsk320xAccReadSPI(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_A | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &acc->gyro->dev.rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        int16_t *acc_data = (int16_t *)acc->gyro->dev.rxBuf;

        acc->ADCRaw[X] = acc_data[1];
        acc->ADCRaw[Y] = acc_data[2];
        acc->ADCRaw[Z] = acc_data[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // This data was read from the gyro, which is the same SPI device as the acc
        int16_t *acc_data = (int16_t *)acc->gyro->dev.rxBuf;

        acc->ADCRaw[X] = acc_data[4];
        acc->ADCRaw[Y] = acc_data[5];
        acc->ADCRaw[Z] = acc_data[6];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

bool lsm6dsk320xSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSK320X_SPI) {
        return false;
    }

    acc->initFn = lsm6dsk320xAccInit;
    acc->readFn = lsm6dsk320xAccReadSPI;

    return true;
}

static bool lsm6dsk320xWriteRegVerified(const extDevice_t *dev, uint8_t reg, uint8_t value)
{
    spiWriteReg(dev, reg, value);
    return spiReadRegMsk(dev, reg) == value;
}

static bool lsm6dsk320xReset(const extDevice_t *dev)
{
    // An MCU-only reset may leave the IMU running. Power down both sensors before SW_RESET,
    // preserving the operating modes until both ODRs are zero (AN5763 sections 3.1 and 5.7).
    const uint8_t ctrl1 = spiReadRegMsk(dev, LSM6DSV_CTRL1) & ~LSM6DSV_CTRL1_ODR_XL_MASK;
    const uint8_t ctrl2 = spiReadRegMsk(dev, LSM6DSV_CTRL2) & ~LSM6DSV_CTRL2_ODR_G_MASK;
    // SW_RESET sets BDU back to 1. Clearing it first keeps a dropped SW_RESET write from
    // passing as a completed reset and leaving the previous firmware's CTRL8 in place.
    if (!lsm6dsk320xWriteRegVerified(dev, LSM6DSV_CTRL1, ctrl1) ||
        !lsm6dsk320xWriteRegVerified(dev, LSM6DSV_CTRL2, ctrl2) ||
        !lsm6dsk320xWriteRegVerified(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC)) {
        return false;
    }

    delayMicroseconds(500);
    spiWriteReg(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);

    // Retain the reset settling time, then bound the retries if SW_RESET never clears
    delay(10);
    for (unsigned elapsedMs = 10; elapsedMs <= LSM6DSK320X_RESET_TIMEOUT_MS; elapsedMs++) {
        const uint8_t ctrl3 = spiReadRegMsk(dev, LSM6DSV_CTRL3);
        if ((ctrl3 & (LSM6DSV_CTRL3_SW_RESET | LSM6DSV_CTRL3_BDU)) == LSM6DSV_CTRL3_BDU) {
            return spiReadRegMsk(dev, LSM6DSV_WHO_AM_I) == LSM6DSK320X_WHO_AM_I_CONST;
        }
        if (elapsedMs < LSM6DSK320X_RESET_TIMEOUT_MS) {
            delay(1);
        }
    }

    return false;
}

static bool lsm6dsk320xConfigure(const extDevice_t *dev, uint8_t hardware_lpf)
{
    // Set default LPF1 filter bandwidth to be as close as possible to MPU6000's 250Hz cutoff
    static const uint8_t lpf_bandwidth_options[GYRO_HARDWARE_LPF_COUNT] = {
            [GYRO_HARDWARE_LPF_NORMAL] = LSM6DSV_CTRL6_FS_G_BW_288HZ,
            [GYRO_HARDWARE_LPF_OPTION_1] = LSM6DSV_CTRL6_FS_G_BW_157HZ,
            [GYRO_HARDWARE_LPF_OPTION_2] = LSM6DSV_CTRL6_FS_G_BW_215HZ,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            [GYRO_HARDWARE_LPF_EXPERIMENTAL] = LSM6DSV_CTRL6_FS_G_BW_455HZ
#endif
    };

    if (!lsm6dsk320xReset(dev)) {
        return false;
    }

    const uint8_t accel_mode = LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY,
                                                   LSM6DSV_CTRL1_OP_MODE_XL_MASK,
                                                   LSM6DSV_CTRL1_OP_MODE_XL_SHIFT);
    const uint8_t gyro_mode = LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY,
                                                  LSM6DSV_CTRL2_OP_MODE_G_MASK,
                                                  LSM6DSV_CTRL2_OP_MODE_G_SHIFT);

    const struct {
        uint8_t reg;
        uint8_t value;
    } config[] = {
            // Autoincrement burst reads and latch each output word until both bytes are read
            { LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU },
            // Configure HAODR mode on both sensors while both remain powered down
            { LSM6DSV_HAODR_CFG, LSM6DSV_ENCODE_BITS(LSM6DSV_HAODR_MODE1,
                                                     LSM6DSV_HAODR_CFG_HAODR_SEL_MASK,
                                                     LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT) },
            // Set the operating modes while both sensors stay powered down
            { LSM6DSV_CTRL1, accel_mode },
            { LSM6DSV_CTRL2, gyro_mode },
            // +/-16G mode, accelerometer LPF2 bandwidth ODR/4
            { LSM6DSV_CTRL8, LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_HP_LPF2_XL_BW_4,
                                                 LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_MASK,
                                                 LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_SHIFT) |
                             LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_FS_XL_16G,
                                                 LSM6DSV_CTRL8_FS_XL_MASK,
                                                 LSM6DSV_CTRL8_FS_XL_SHIFT) },
            // Enable the accelerometer digital LPF2 filter
            { LSM6DSV_CTRL9, LSM6DSV_CTRL9_LPF2_XL_EN },
            // +/-2000 dps and the selected LPF1 filter setting. FS_G is only bits [2:0] on the
            // LSM6DSK320X, where bit 3 must be 1 (DS15060 Table 64, reset value 0x08).
            { LSM6DSV_CTRL6, LSM6DSK320X_CTRL6_FS_G_BIT3 |
                             LSM6DSV_ENCODE_BITS(lpf_bandwidth_options[hardware_lpf],
                                                 LSM6DSV_CTRL6_LPF1_G_BW_MASK,
                                                 LSM6DSV_CTRL6_LPF1_G_BW_SHIFT) |
                             LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_2000DPS,
                                                 LSM6DSV_CTRL6_FS_G_MASK,
                                                 LSM6DSV_CTRL6_FS_G_SHIFT) },
            // Enable the gyro digital LPF1 filter
            { LSM6DSV_CTRL7, LSM6DSV_CTRL7_LPF1_G_EN },
            // Generate a pulse on the interrupt line, not requiring a read to clear
            { LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED },
            // Interrupt on INT1 when new gyro data is ready
            { LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G },
    };

    for (unsigned index = 0; index < ARRAYLEN(config); index++) {
        if (!lsm6dsk320xWriteRegVerified(dev, config[index].reg, config[index].value)) {
            return false;
        }
    }

    // Allow the power-down state to settle, then start the gyro before the accelerometer.
    // Retaining the mode bits while setting the ODR is what keeps the rates at 8 kHz / 1 kHz:
    // programming mode and ODR in a single write leaves them at 7.68 kHz / 960 Hz.
    delayMicroseconds(500);
    return lsm6dsk320xWriteRegVerified(dev, LSM6DSV_CTRL2,
                gyro_mode | LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_ODR_G_8000HZ,
                                                LSM6DSV_CTRL2_ODR_G_MASK,
                                                LSM6DSV_CTRL2_ODR_G_SHIFT)) &&
           lsm6dsk320xWriteRegVerified(dev, LSM6DSV_CTRL1,
                accel_mode | LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_ODR_XL_1000HZ,
                                                 LSM6DSV_CTRL1_ODR_XL_MASK,
                                                 LSM6DSV_CTRL1_ODR_XL_SHIFT));
}

static void lsm6dsk320xGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSK320X_MAX_SPI_CLK_HZ));

    bool configured = false;
    for (unsigned attempt = 0; attempt < LSM6DSK320X_INIT_ATTEMPTS && !configured; attempt++) {
        configured = lsm6dsk320xConfigure(dev, gyro->hardware_lpf);
    }
    if (!configured) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
        return;
    }

    // From section 4.1, Mechanical characteristics, of the datasheet: G_So is 70 mdps/LSB
    // for FS = +/-2000 dps.
    gyro->scale = 0.070f;

    mpuGyroInit(gyro);
    gyro->accDataReg = LSM6DSV_OUTX_L_A;
    gyro->gyroDataReg = LSM6DSV_OUTX_L_G;
}

static bool lsm6dsk320xGyroReadSPI(gyroDev_t *gyro)
{
    int16_t *gyro_data = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0xff
        memset(gyro->dev.txBuf, 0xff, 16);

        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)(uintptr_t)gyro;
                gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_G | 0x80;
                // Read three gyro words immediately followed by three accelerometer words
                gyro->segments[0].len = sizeof(uint8_t) + 6 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntcallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        // This call configures acquisition; no sensor data has been read yet.
        return false;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_G | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = gyro_data[1];
        gyro->gyroADCRaw[Y] = gyro_data[2];
        gyro->gyroADCRaw[Z] = gyro_data[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyro_data[1];
        gyro->gyroADCRaw[Y] = gyro_data[2];
        gyro->gyroADCRaw[Z] = gyro_data[3];
        break;
    }

    default:
        break;
    }

    return true;
}

bool lsm6dsk320xSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSK320X_SPI) {
        return false;
    }

    gyro->initFn = lsm6dsk320xGyroInit;
    gyro->readFn = lsm6dsk320xGyroReadSPI;

    return true;
}

#endif // USE_ACCGYRO_LSM6DSK320X
