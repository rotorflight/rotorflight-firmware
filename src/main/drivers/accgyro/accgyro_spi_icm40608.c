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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_GYRO_SPI_ICM40608

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm40608.h"
#if defined(USE_ICM40608_AS_ICM40609D)
#include "drivers/accgyro/accgyro_spi_icm40609.h"
#endif
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// Datasheet Sec 3.5: SPI clock frequency, max 24 MHz
#define ICM40608_MAX_SPI_CLK_HZ 24000000

// Datasheet Sec 14.35 note: "do not issue any register writes for 200us"
// after any PWR_MGMT0 mode transition.
#define ICM40608_MODE_CHANGE_SETTLE_US 200

// Datasheet Sec 3.3.1 Table 3: "Start-up time for register read/write, from
// power-up: 1ms". Sec 4.1 DEVICE_CONFIG soft reset has no explicit delay
// spec in this doc, 1ms (as used by other Invensense parts, e.g. ICM20689's
// 100ms MPU6000-derived reset delay) is used conservatively here; datasheet
// gives no number, so we borrow the same margin used for the sister part.
#define ICM40608_RESET_DELAY_MS 1

/*
 * Datasheet Sec 3.2 Table 2 "Accelerometer Specifications":
 *   ACCEL_FS_SEL=0 -> +-16g -> 2048 LSB/g =>Full Scale - FS
 *   ACCEL_FS_SEL=1 -> +-8g  -> 4096 LSB/g
 *   ACCEL_FS_SEL=2 -> +-4g  -> 8192 LSB/g
 *   ACCEL_FS_SEL=3 -> +-2g  -> 16384 LSB/g
 * Rotorflight's acc->acc_1G expects LSB-per-1g at the selected range.
 * We run the accelerometer at +-16g (ACCEL_FS_SEL = 0), same choice as
 * the other high-g capable gyros in this codebase (e.g. icm20689 uses
 * a fixed 16g equivalent too), so acc_1G = 2048.
 */
#define ICM40608_ACCEL_1G_AT_16G 2048 // how many raw digital counts equal 1g
// here we've set 2048 so 1g=2048 counts, raw accel(g) = raw accel(counts)/2048

/**
 * @brief Detect ICM40608 on SPI bus and return the sensor type if found.
 * @param dev Pointer to the extDevice_t structure for the SPI device.
 * @return mpuSensor_e value indicating the detected sensor type, or MPU_NONE if not detected.
 */
uint8_t icm40608SpiDetect(const extDevice_t *dev)
{
    // Datasheet Sec 14.1 DEVICE_CONFIG: bit0 SOFT_RESET_CONFIG, "1: Enable reset"
    spiWriteReg(dev, ICM40608_RA_DEVICE_CONFIG, ICM40608_BIT_SOFT_RESET);
    delay(ICM40608_RESET_DELAY_MS);

    // Datasheet Sec 14.57 WHO_AM_I: reg 0x75, reset value 0x39
    const uint8_t whoAmI = spiReadRegMsk(dev, ICM40608_RA_WHO_AM_I);
    if (whoAmI != ICM40608_WHO_AM_I_CONST)
    {
        return MPU_NONE;
    }

    return ICM_40608_SPI;
}
/**
 * @brief Initialize/turn on the ICM40608 accelerometer and set its range.
 * @param acc Pointer to the accDev_t structure for the accelerometer.
 * @return None
 */
void icm40608AccInit(accDev_t *acc)
{
    // See ICM40608_ACCEL_1G_AT_16G note above (Sec 3.2 Table 2)
    acc->acc_1G = ICM40608_ACCEL_1G_AT_16G; // set the conversation factor for 1g = 2048
    // Datasheet Sec 14.35 PWR_MGMT0: bits 1:0 ACCEL_MODE = 11 (Low Noise mode)
    // Datasheet Sec 14.35 note: "When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs."
    spiWriteReg(&acc->dev, ICM40608_RA_PWR_MGMT0, ICM40608_GYRO_MODE_LOW_NOISE | ICM40608_ACCEL_MODE_LOW_NOISE);
    delayMicroseconds(ICM40608_MODE_CHANGE_SETTLE_US);
    // Datasheet Sec 14.37 ACCEL_CONFIG0: bits 7:5 = ACCEL_FS_SEL, bits 3:0 = ACCEL_ODR
    // +-16g (000) | ODR = 1kHz (0110), matches gyro_sync.c ICM_40608_SPI case
    // set the maximum acceleration the sensor can measure, +-16g = +-16*9.80..m/s^2
    // ODR->output data rate = how often the sensor produces a new measurement per second
    spiWriteReg(&acc->dev, ICM40608_RA_ACCEL_CONFIG0,
                (ICM40608_ACCEL_FS_16G << 5) | ICM40608_ODR_1K);
    delayMicroseconds(ICM40608_MODE_CHANGE_SETTLE_US);
}
/**
 * @brief connect accel init/read functions(icm40608AccInit or icm40608AccRead), only if accel was already detected
 * @concept Check if acc->mpuDetectionResult.sensor == ICM_40608_SPI
    1 .If not → return false
    2. If yes → set acc->initFn = icm40608AccInit
    3. Set acc->readFn = icm40608AccRead
    4. Return true
 */
bool icm40608SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ICM_40608_SPI)
    {
        return false;
    }

#if defined(USE_ICM40608_AS_ICM40609D)
    acc->initFn = icm40609AccInit;
    acc->readFn = mpuAccReadSPI;
#else
    acc->initFn = icm40608AccInit;
    acc->readFn = icm40608AccRead;
#endif

    return true;
}

/**
 * @brief turn gyro ON, set its range
 * @concept Call mpuGyroInit(gyro) (generic, shared setup)
    1. Set SPI speed to 24MHz max
    2. Write to INTF_CONFIG1 (0x4D) → set clock source
    3. Write to PWR_MGMT0 (0x4E) → turn gyro ON
    4. Wait 200µs
    5. Write to GYRO_CONFIG0 (0x4F) → set range ±2000dps + speed 1000Hz
    6. Wait 200µs
 */
void icm40608GyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    mpuGyroInit(gyro);
    spiSetClkDivisor(dev, spiCalculateDivider(ICM40608_MAX_SPI_CLK_HZ));

    // Datasheet Sec 14.34 INTF_CONFIG1, CLKSEL bits 1:0:
    // "01: Select PLL when available, else select RC oscillator (default)"
    // -- this is already the reset default, written explicitly for clarity.
    // set the clock source to PLL
    spiWriteReg(dev, ICM40608_RA_INTF_CONFIG1, ICM40608_BIT_CLKSEL_PLL_OR_RC);
    delayMicroseconds(ICM40608_MODE_CHANGE_SETTLE_US);

    // Datasheet Sec 14.35 PWR_MGMT0: bits 3:2 GYRO_MODE = 11 (Low Noise mode)
    // Datasheet Sec 14.35 note: "Gyroscope needs to be kept ON for a minimum
    // of 45ms" and "do not issue any register writes for 200us" right after.
    spiWriteReg(dev, ICM40608_RA_PWR_MGMT0, ICM40608_GYRO_MODE_LOW_NOISE | ICM40608_ACCEL_MODE_LOW_NOISE);
    delayMicroseconds(ICM40608_MODE_CHANGE_SETTLE_US);

    // Datasheet Sec 14.36 GYRO_CONFIG0: bits 7:5 = GYRO_FS_SEL, bits 3:0 = GYRO_ODR
    // +-2000dps (000) | ODR = 8kHz (0111)
    spiWriteReg(dev, ICM40608_RA_GYRO_CONFIG0,
                (ICM40608_GYRO_FS_2000DPS << 5) | ICM40608_ODR_8K);
    delayMicroseconds(ICM40608_MODE_CHANGE_SETTLE_US);
}
/**
 * @brief connect gyro init/read functions, only if gyro was already detected
 * @concept Check if gyro->mpuDetectionResult.sensor == ICM_40608_SPI
    1 .If not → return false
    2. If yes → set gyro->initFn = icm40608GyroInit
    3. Set gyro->readFn = icm40608GyroReadSPI
    4. Set gyro->scale = GYRO_SCALE_2000DPS
    5. Return true
 */
bool icm40608SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ICM_40608_SPI)
    {
        return false;
    }

#if defined(USE_ICM40608_AS_ICM40609D)
    gyro->initFn = icm40609GyroInit;
    gyro->readFn = mpuGyroReadSPI;
#else
    gyro->initFn = icm40608GyroInit;
    gyro->readFn = icm40608GyroReadSPI;
#endif

    // Datasheet Sec 3.1 Table 1: GYRO_FS_SEL=0 -> +-2000dps -> 16.4 LSB/(deg/s)
    // GYRO_SCALE_2000DPS already encodes this same 16.4 LSB/dps constant
    // elsewhere in the codebase (see accgyro_mpu6050.c, icm20689.c etc).
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
/**
 * @brief read current gyro values
 * @param gyro pointer to the gyro device
 * @return true if successful, false otherwise
 * @concept
 * 1. Send read request starting at GYRO_DATA_X1 (0x25), 6 bytes
 * 2. If fail → return false
 * 3. Combine byte1+byte2 → X value
 * 4. Combine byte3+byte4 → Y value
 * 5. Combine byte5+byte6 → Z value
 * 6. Return true
 */
bool icm40608GyroReadSPI(gyroDev_t *gyro)
{
    // Datasheet Sec 9.5 SPI Interface: read = address with R/W bit (bit7) set,
    // MSB first. Datasheet Sec 14.13-14.18: GYRO_DATA_X1 (0x25) through
    // GYRO_DATA_Z0 are 6 contiguous bytes, big-endian per axis (X1 then X0...)
    static uint8_t dataToSend[7] = {ICM40608_RA_GYRO_DATA_X1 | 0x80,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    const bool ack = spiReadWriteBufRB(&gyro->dev, dataToSend, data, 7);
    if (!ack)
    {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
}

bool icm40608AccRead(accDev_t *acc)
{
    // Datasheet Sec 14.7-14.12: ACCEL_DATA_X1 (0x1F) through ACCEL_DATA_Z0
    // are 6 contiguous bytes, same burst-read pattern as gyro data.
    uint8_t data[6];

    const bool ack = spiReadRegMskBufRB(&acc->dev, ICM40608_RA_ACCEL_DATA_X1, data, 6);
    if (!ack)
    {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}
#endif