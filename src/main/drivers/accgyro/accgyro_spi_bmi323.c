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
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_SPI_BMI323

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi323.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 10 MHz max SPI frequency
#define BMI323_MAX_SPI_CLK_HZ 10000000

#define BMI323_CHIP_ID 0x43

// BMI323 registers (from datasheet)
typedef enum {
    BMI323_REG_CHIP_ID = 0x00,
    BMI323_REG_ERR_REG = 0x01,
    BMI323_REG_STATUS = 0x02,
    BMI323_REG_ACC_DATA_X = 0x03,
    BMI323_REG_ACC_DATA_Y = 0x04,
    BMI323_REG_ACC_DATA_Z = 0x05,
    BMI323_REG_GYR_DATA_X = 0x06,
    BMI323_REG_GYR_DATA_Y = 0x07,
    BMI323_REG_GYR_DATA_Z = 0x08,
    BMI323_REG_TEMP_DATA = 0x09,
    BMI323_REG_INT_STATUS_INT1 = 0x0D,
    BMI323_REG_INT_STATUS_INT2 = 0x0E,
    BMI323_REG_INT_STATUS_IBI = 0x0F,
    BMI323_REG_FEATURE_IO0 = 0x10,
    BMI323_REG_FEATURE_IO1 = 0x11,
    BMI323_REG_FEATURE_IO2 = 0x12,
    BMI323_REG_FEATURE_IO_STATUS = 0x14,
    BMI323_REG_FIFO_FILL_LEVEL = 0x15,
    BMI323_REG_FIFO_DATA = 0x16,
    BMI323_REG_ACC_CONF = 0x20,
    BMI323_REG_GYR_CONF = 0x21,
    BMI323_REG_ALT_ACC_CONF = 0x28,
    BMI323_REG_ALT_GYR_CONF = 0x29,
    BMI323_REG_ALT_CONF = 0x2A,
    BMI323_REG_ALT_STATUS = 0x2B,
    BMI323_REG_FIFO_WTMK = 0x35,
    BMI323_REG_FIFO_CONF = 0x36,
    BMI323_REG_FIFO_CTRL = 0x37,
    BMI323_REG_IO_INT_CTRL = 0x38,
    BMI323_REG_INT_CONF = 0x39,
    BMI323_REG_INT_MAP1 = 0x3A,
    BMI323_REG_INT_MAP2 = 0x3B,
    BMI323_REG_FEATURE_CTRL = 0x40,
    BMI323_REG_FEATURE_DATA_ADDR = 0x41,
    BMI323_REG_FEATURE_DATA_TX = 0x42,
    BMI323_REG_FEATURE_DATA_STATUS = 0x43,
    BMI323_REG_FEATURE_ENGINE_STATUS = 0x45,
    BMI323_REG_FEATURE_EVENT_EXT = 0x47,
    BMI323_REG_IO_PDN_CTRL = 0x4F,
    BMI323_REG_IO_SPI_IF = 0x50,
    BMI323_REG_IO_PAD_STRENGTH = 0x51,
    BMI323_REG_CMD = 0x7E,
    BMI323_REG_CFG_RES = 0x7F,
} bmi323Register_e;

// BMI323 commands
typedef enum {
    BMI323_CMD_SOFTRESET = 0xDEAF,
    BMI323_CMD_FIFO_FLUSH = 0x00B0,
    BMI323_CMD_SELF_TEST_TRIGGER = 0x0100,
} bmi323Command_e;

// BMI323 ACC/GYR configuration values
// ODR (Output Data Rate) values
#define BMI323_ACC_ODR_0_78HZ     0x01
#define BMI323_ACC_ODR_1_56HZ     0x02
#define BMI323_ACC_ODR_3_125HZ    0x03
#define BMI323_ACC_ODR_6_25HZ     0x04
#define BMI323_ACC_ODR_12_5HZ     0x05
#define BMI323_ACC_ODR_25HZ       0x06
#define BMI323_ACC_ODR_50HZ       0x07
#define BMI323_ACC_ODR_100HZ      0x08
#define BMI323_ACC_ODR_200HZ      0x09
#define BMI323_ACC_ODR_400HZ      0x0A
#define BMI323_ACC_ODR_800HZ      0x0B
#define BMI323_ACC_ODR_1600HZ     0x0C
#define BMI323_ACC_ODR_3200HZ     0x0D
#define BMI323_ACC_ODR_6400HZ     0x0E

#define BMI323_GYR_ODR_0_78HZ     0x01
#define BMI323_GYR_ODR_1_56HZ     0x02
#define BMI323_GYR_ODR_3_125HZ    0x03
#define BMI323_GYR_ODR_6_25HZ     0x04
#define BMI323_GYR_ODR_12_5HZ     0x05
#define BMI323_GYR_ODR_25HZ       0x06
#define BMI323_GYR_ODR_50HZ       0x07
#define BMI323_GYR_ODR_100HZ      0x08
#define BMI323_GYR_ODR_200HZ      0x09
#define BMI323_GYR_ODR_400HZ      0x0A
#define BMI323_GYR_ODR_800HZ      0x0B
#define BMI323_GYR_ODR_1600HZ     0x0C
#define BMI323_GYR_ODR_3200HZ     0x0D
#define BMI323_GYR_ODR_6400HZ     0x0E

// ACC range values (bits 0-2 of ACC_CONF)
#define BMI323_ACC_RANGE_2G       0x00
#define BMI323_ACC_RANGE_4G       0x01
#define BMI323_ACC_RANGE_8G       0x02
#define BMI323_ACC_RANGE_16G      0x03

// GYR range values (bits 0-2 of GYR_CONF)
#define BMI323_GYR_RANGE_125DPS   0x00
#define BMI323_GYR_RANGE_250DPS   0x01
#define BMI323_GYR_RANGE_500DPS   0x02
#define BMI323_GYR_RANGE_1000DPS  0x03
#define BMI323_GYR_RANGE_2000DPS  0x04

// ACC/GYR bandwidth parameter (bits 7-4 of ACC_CONF/GYR_CONF)
#define BMI323_BW_ODR_HALF        0x00
#define BMI323_BW_ODR_QUARTER     0x01

// ACC/GYR mode (bits 14-12 of ACC_CONF/GYR_CONF)
#define BMI323_MODE_DISABLE       0x00
#define BMI323_MODE_DUTY_CYCLE    0x03
#define BMI323_MODE_CONTINUOUS    0x04
#define BMI323_MODE_HIGH_PERF     0x07

// Gyr INT1 config
#define BMI323_INT_INT1_ACTIVE_HIGH   0x00
#define BMI323_INT_INT1_ACTIVE_LOW    0x01
#define BMI323_INT_INT1_PUSH_PULL     0x00
#define BMI323_INT_INT1_OPEN_DRAIN    0x01
#define BMI323_INT_INT1_OUTPUT_ENABLE 0x01
#define BMI323_INT_INT1_OUTPUT_DISABLE 0x00
#define BMI323_INT_INT1_LATCHED       0x01
#define BMI323_INT_INT1_NON_LATCHED   0x00
#define BMI323_INT_INT1_LEVEL_HIGH  0x00
#define BMI323_INT_INT1_LEVEL_LOW   0x01
#define BMI323_INT_GYR_DATA_READY_INT1 0x01


// ACC/GYR averaging (bits 10-8 of ACC_CONF/GYR_CONF for high performance mode)
#define BMI323_AVG_1              0x00
#define BMI323_AVG_2              0x01
#define BMI323_AVG_4              0x02
#define BMI323_AVG_8              0x03
#define BMI323_AVG_16             0x04
#define BMI323_AVG_32             0x05
#define BMI323_AVG_64             0x06

// Need to see at least this many interrupts during initialization
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Static function declarations
static void bmi323SpiGyroInit(gyroDev_t *gyro);
static void bmi323SpiAccInit(accDev_t *acc);
static bool bmi323GyroRead(gyroDev_t *gyro);
static bool bmi323AccRead(accDev_t *acc);
static void bmi323ExtiHandler(extiCallbackRec_t *cb);

// DMA buffer for accelerometer reads
static DMA_DATA uint8_t accBuf[32];

// BMI323 uses 16-bit register access via SPI
static uint16_t bmi323RegisterRead(const extDevice_t *dev, bmi323Register_e registerId)
{
    uint8_t rxBuf[3] = {0, 0, 0};

    /*
     * BMI323 SPI read protocol:
     * Byte 0: dummy byte (ignored)
     * Byte 1: LSB of 16-bit register value
     * Byte 2: MSB of 16-bit register value
     */
    if (!spiReadRegMskBufRB(dev, registerId, rxBuf, 3)) {
        uint8_t txBuf[3] = {registerId | 0x80, 0, 0};
        uint8_t fallbackRx[3] = {0, 0, 0};
        spiReadWriteBuf(dev, txBuf, fallbackRx, 3);
        return (fallbackRx[2] << 8) | fallbackRx[1];
    }

    return (rxBuf[2] << 8) | rxBuf[1];
}

static void bmi323RegisterWrite(const extDevice_t *dev, bmi323Register_e registerId, uint16_t value, unsigned delayMs)
{
    uint8_t data[3];
    data[0] = registerId;
    data[1] = value & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    
    spiWriteRegBuf(dev, registerId, &data[1], 2);
    
    if (delayMs) {
        delay(delayMs);
    }
}

// Soft reset the BMI323
static void bmi323SoftReset(const extDevice_t *dev)
{
    //give the device time to settle
    bmi323RegisterWrite(dev, BMI323_REG_CMD, BMI323_CMD_SOFTRESET, 50);
}

uint8_t bmi323Detect(const extDevice_t *dev)
{
    spiSetClkDivisor(dev, spiCalculateDivider(BMI323_MAX_SPI_CLK_HZ));

    // dummy read for switching to SPI mode
    bmi323RegisterRead(dev, BMI323_REG_CHIP_ID);
    
    // Soft reset
    bmi323SoftReset(dev);    

    // switch to SPI mode again after reset
    bmi323RegisterRead(dev, BMI323_REG_CHIP_ID);

    // Read chip ID
    uint16_t chipId = bmi323RegisterRead(dev, BMI323_REG_CHIP_ID);

    //MSB of the reigster is reserved
    uint16_t chipID_LSB = chipId & 0xFF;
    
    if (chipID_LSB == BMI323_CHIP_ID) {
        uint16_t err_reg = bmi323RegisterRead(dev, BMI323_REG_ERR_REG);
        uint16_t status = bmi323RegisterRead(dev, BMI323_REG_STATUS);

        if (err_reg != 0 || (status & 0x01) == 0) {
            // Error register non-zero or sensor not ready
            return MPU_NONE;
        }

        return BMI_323_SPI;
    }
    
    return MPU_NONE;
}

bool bmi323SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_323_SPI) {
        return false;
    }
    
    gyro->initFn = bmi323SpiGyroInit;
    gyro->readFn = bmi323GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;
    
    return true;
}

bool bmi323SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_323_SPI) {
        return false;
    }
    
    // ACC part uses the same SPI bus as the gyro, so we can just use the gyro's spi instance
    acc->dev.txBuf = accBuf;
    acc->dev.rxBuf = &accBuf[32 / 2];
    
    acc->initFn = bmi323SpiAccInit;
    acc->readFn = bmi323AccRead;
    
    busDeviceRegister(&acc->dev);
    
    return true;
}

static void bmi323ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}

static void bmi323SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    
    spiSetClkDivisor(dev, spiCalculateDivider(BMI323_MAX_SPI_CLK_HZ));
    
    // Select ODR based on hardware LPF setting
    uint16_t gyrODR = BMI323_GYR_ODR_3200HZ; // Default: 3200Hz ODR (BW 1181Hz)
    
    switch (gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            // ODR: 3.2kHz, BW: 1181Hz
            gyrODR = BMI323_GYR_ODR_3200HZ;
            break;
        case GYRO_HARDWARE_LPF_OPTION_1:
            // ODR: 1.6kHz, BW: 674Hz
            gyrODR = BMI323_GYR_ODR_1600HZ;
            break;
        case GYRO_HARDWARE_LPF_OPTION_2:
            // ODR: 6.4kHz, BW: 1677Hz
            gyrODR = BMI323_GYR_ODR_6400HZ;
            break;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            // ODR: 3.2kHz, BW: 1181Hz
            gyrODR = BMI323_GYR_ODR_3200HZ;
            break;
#endif
        default:
            gyrODR = BMI323_GYR_ODR_3200HZ;
            break;
    }
    
    // Configure gyroscope
    // Set to high performance mode, 2000dps range
    uint16_t gyrConf = (BMI323_MODE_HIGH_PERF << 12) | (BMI323_AVG_1 << 8) 
                    | (BMI323_BW_ODR_HALF << 7) | (BMI323_GYR_RANGE_2000DPS << 4) | (gyrODR << 0);
    bmi323RegisterWrite(dev, BMI323_REG_GYR_CONF, gyrConf, 0);
    
    // Configure interrupt in non latched mode
    uint16_t intConf = BMI323_INT_INT1_NON_LATCHED;
    bmi323RegisterWrite(dev, BMI323_REG_INT_CONF, intConf, 0);
    
    uint16_t intMap2 = (BMI323_INT_GYR_DATA_READY_INT1 << 8); // Map2: gyro data ready to INT1
    bmi323RegisterWrite(dev, BMI323_REG_INT_MAP2, intMap2, 0);
    
    uint16_t ioIntCtrl = ((BMI323_INT_INT1_OUTPUT_ENABLE << 2 )| (BMI323_INT_INT1_PUSH_PULL << 1) | BMI323_INT_INT1_ACTIVE_HIGH); // INT1: 0000 0 ENABLE:1; PUSH-PULL_EN:0, 0 ACTIVE HIGH, 1 OUTPUT ENABLED
    bmi323RegisterWrite(dev, BMI323_REG_IO_INT_CTRL, ioIntCtrl, 0);
    
    // Setup EXTI interrupt
    if (gyro->mpuIntExtiTag != IO_TAG_NONE) {
        IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);
        
        IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
        EXTIHandlerInit(&gyro->exti, bmi323ExtiHandler);
        EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
        EXTIEnable(mpuIntIO);
    }
    
    gyro->gyroDataReg = BMI323_REG_GYR_DATA_X;
}

extiCallbackRec_t bmi323IntCallbackRec;

busStatus_e bmi323Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

static void bmi323SpiAccInit(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;
    
    spiSetClkDivisor(dev, spiCalculateDivider(BMI323_MAX_SPI_CLK_HZ));
    
    // Configure accelerometer
    // Set to high performance mode, 16G range, 800Hz ODR
    uint16_t accConf = (BMI323_MODE_HIGH_PERF << 12) | (BMI323_AVG_1 << 8) | (BMI323_BW_ODR_HALF << 7)
                       | (BMI323_ACC_RANGE_16G << 4) | (BMI323_ACC_ODR_800HZ << 0) ;
    bmi323RegisterWrite(dev, BMI323_REG_ACC_CONF, accConf, 0);
    
    // 16G range scale: 32768 / 16 = 2048 LSB/G
    acc->acc_1G = 2048;
}

static bool bmi323GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(dev->txBuf, 0x00, 8);

        // Check that minimum number of interrupts have been detected
        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uint32_t)gyro;
                dev->txBuf[1] = BMI323_REG_GYR_DATA_X | 0x80;
                gyro->segments[0].len = 8;
                gyro->segments[0].callback = bmi323Intcallback;
                gyro->segments[0].u.buffers.txData = dev->txBuf;
                gyro->segments[0].u.buffers.rxData = dev->rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = BMI323_REG_GYR_DATA_X | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);

        // Wait for completion
        spiWait(dev);

        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Data is 16-bit little endian starting at byte 1 (byte 0 is dummy)
        int16_t *gyroData = (int16_t *)&dev->rxBuf[2];
        gyro->gyroADCRaw[X] = gyroData[0];
        gyro->gyroADCRaw[Y] = gyroData[1];
        gyro->gyroADCRaw[Z] = gyroData[2];
        break;
    }

    default:
        break;
    }
    
    return true;
}

static bool bmi323AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        memset(dev->txBuf, 0x00, 8);

        dev->txBuf[0] = BMI323_REG_ACC_DATA_X | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);

        // Wait for completion
        spiWait(dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Data is 16-bit little endian: first byte = reg addr, second byte = dummy
        int16_t *accData = (int16_t *)&dev->rxBuf[2];
        acc->ADCRaw[X] = accData[0];
        acc->ADCRaw[Y] = accData[1];
        acc->ADCRaw[Z] = accData[2];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

uint8_t bmi323InterruptStatus(gyroDev_t *gyro)
{
    uint16_t intStatus = bmi323RegisterRead(&gyro->dev, BMI323_REG_INT_STATUS_INT1);
    return (uint8_t)(intStatus & 0xFF);
}

#endif // USE_ACCGYRO_SPI_BMI323
