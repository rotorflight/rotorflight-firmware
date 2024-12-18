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

#ifdef USE_ACCGYRO_SPI_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "accgyro.h"
#include "accgyro_spi_bmi088.h"

#define BMI088_MAX_SPI_CLK_HZ 10000000
#define GYRO_EXTI_DETECT_THRESHOLD 1000

#define BMI088_REG_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID 0x0F
#define BMI088_REG_GYRO_RANGE 0x0F
#define BMI088_REG_GYRO_BANDWIDTH 0x10
#define BMI088_REG_GYRO_RATE_DATA 0x02
#define BMI088_REG_GYRO_SOFTRESET 0x14
#define BMI088_REG_GYRO_INT_CTRL 0x15
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_REG_GYRO_SELF_TEST 0x3C
#define BMI088_EN_DRDY_INT 0x80

#define BMI088_REG_ACC_CHIP_ID 0x00
#define BMI088_ACC_CHIP_ID 0x1E
#define BMI088_REG_ACC_CONF 0x40
#define BMI088_REG_ACC_RANGE 0x41
#define BMI088_REG_ACC_PWR_CONF 0x7C
#define BMI088_REG_ACC_PWR_CTRL 0x7D
#define BMI088_REG_ACC_SOFTRESET 0x7E
#define BMI088_REG_ACC_DATA 0x12
#define BMI088_TRIGGER_SOFTRESET 0xB6

enum bmi088_accc_range{
    BMI088_A_RANGE_3G = 0x00,
    BMI088_A_RANGE_6G = 0x01,
    BMI088_A_RANGE_12G = 0x02,
    BMI088_A_RANGE_24G = 0x03,
};
enum bmi088_acc_pwr_save{
    BMI088_A_ACTIVE = 0x00,
    BMI088_A_SUSPEND = 0x03,
};
enum bmi088_acc_enable{
    BMI088_A_OFF = 0x00,
    BMI088_A_ON = 0x04,
};
enum bmi088_acc_bwp{
    BMI088_A_BWP_OSR4 = 0x00,
    BMI088_A_BWP_OSR2 = 0x01,
    BMI088_A_BWP_NORMAL = 0x02,
};
enum bmi088_acc_odr{
    BMI088_A_ODR_12_5 = 0x05,
    BMI088_A_ODR_25 = 0x06,
    BMI088_A_ODR_50 = 0x07,
    BMI088_A_ODR_100 = 0x08,
    BMI088_A_ODR_200 = 0x09,
    BMI088_A_ODR_400 = 0x0A,
    BMI088_A_ODR_800 = 0x0B,
    BMI088_A_ODR_1600 = 0x0C,
};
enum bmi088_gyro_bandwidth{
    BMI088_G_BANDWIDTH_532HZ = 0x00,
    BMI088_G_BANDWIDTH_230HZ = 0x01,
    BMI088_G_BANDWIDTH_116HZ = 0x02,
    BMI088_G_BANDWIDTH_47HZ = 0x03,
    BMI088_G_BANDWIDTH_23HZ = 0x04,
    BMI088_G_BANDWIDTH_12HZ = 0x05,
    BMI088_G_BANDWIDTH_64HZ = 0x06,
    BMI088_G_BANDWIDTH_32HZ = 0x07,
};
enum bmi088_gyro_range {
    BMI088_G_RANGE_125DPS = 0x04,
    BMI088_G_RANGE_250DPS = 0x03,
    BMI088_G_RANGE_500DPS = 0x02,
    BMI088_G_RANGE_1000DPS = 0x01,
    BMI088_G_RANGE_2000DPS = 0x00,
};

void bmi088ExtiHandler(extiCallbackRec_t *cb);
void bmi088SpiGyroInit(gyroDev_t *gyro);
bool bmi088GyroRead(gyroDev_t *gyro);
bool bmi088SpiGyroDetect(gyroDev_t *gyro);
bool bmi088AccRead(accDev_t *acc);
uint8_t bmi088spiBusReadRegisterAcc(const extDevice_t *dev, const uint8_t reg);
bool bmi088SpiAccDetect(accDev_t *acc);
void bmi088SpiAccInit(accDev_t *acc);
static volatile bool BMI088GyroDetected = false;
static volatile bool BMI088AccDetected = false;
static DMA_DATA uint8_t accBuf[32];

void bmi088ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}

void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
      // softreset
    spiWriteReg(dev, BMI088_REG_GYRO_SOFTRESET, BMI088_TRIGGER_SOFTRESET);
    delay(30);

    //config sensor

    spiWriteReg(dev, BMI088_REG_GYRO_RANGE, BMI088_G_RANGE_2000DPS);
    spiWriteReg(dev, BMI088_REG_GYRO_BANDWIDTH, BMI088_G_BANDWIDTH_230HZ);

    // enable dataready interrupt
    spiWriteReg(dev, BMI088_REG_GYRO_INT_CTRL, BMI088_EN_DRDY_INT);

    // INT3: push-pull, active high
    spiWriteReg(dev, BMI088_REG_GYRO_INT3_INT4_IO_CONF, 0x01);

    // DRDY int is mapped to INT3 pin
    spiWriteReg(dev, BMI088_REG_GYRO_INT3_INT4_IO_MAP, 0x01);

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi088ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING); // TODO - maybe pullup / pulldown ?
    EXTIEnable(mpuIntIO);
}

extiCallbackRec_t bmi088IntCallbackRec;

busStatus_e bmi088Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

bool bmi088GyroRead(gyroDev_t *gyro)
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
                dev->txBuf[1] = BMI088_REG_GYRO_RATE_DATA | 0x80;
                gyro->segments[0].len = 8;
                gyro->segments[0].callback = bmi088Intcallback;
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
        dev->txBuf[0] = BMI088_REG_GYRO_RATE_DATA | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
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
        int16_t *gyroData = (int16_t *)&dev->rxBuf[1];//first byte is the register address
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

uint8_t bmi088SpiDetect(const extDevice_t *dev)
{
	if (BMI088GyroDetected) {
        return BMI_088_SPI;
    }

	spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));

    //init ACC cs to avoid comminication conflicts
    /*acc_cs_pin = IOGetByTag(IO_TAG(BMI088_CS_A_PIN));
    IOInit(acc_cs_pin, OWNER_ACC_CS, 0);
    IOConfigGPIO(acc_cs_pin, IOCFG_OUT_PP);
    IOHi(acc_cs_pin);*/

	if (spiReadReg(dev, BMI088_REG_GYRO_CHIP_ID | 0x80) != BMI088_GYRO_CHIP_ID) {
		return MPU_NONE;
	}

	BMI088GyroDetected = true;

	return BMI_088_SPI;
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if(gyro->mpuDetectionResult.sensor != BMI_088_SPI){
		return false;
	}
    uint8_t resultBITE = spiReadReg(&gyro->dev, BMI088_REG_GYRO_SELF_TEST| 0x80);

    //trigger BITE
    spiWriteReg(&gyro->dev, BMI088_REG_GYRO_SELF_TEST, 0x01);
    uint8_t startBITETime = millis();
    do{
        resultBITE = spiReadReg(&gyro->dev, BMI088_REG_GYRO_SELF_TEST| 0x80);
    }while(startBITETime - millis() < 50 && (resultBITE & 0x02) != 0x02);
    
    
    if((resultBITE & 0x04) == 0x04){
        //BITE failed
        return false;
    }
    

    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}


bool bmi088AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        memset(dev->txBuf, 0x00, 8);

        dev->txBuf[0] = BMI088_REG_ACC_DATA | 0x80;

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
        int16_t *accData = (int16_t *)dev->rxBuf; //first byte = reg addr, second byte = dummy
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

uint8_t bmi088spiBusReadRegisterAcc(const extDevice_t *dev, const uint8_t reg)
{
    uint8_t data[2] = { 0 };

    if (spiReadRegMskBufRB(dev, reg, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

void bmi088SpiAccInit(accDev_t *acc)
{
    //softreset
    spiWriteReg(&acc->dev, BMI088_REG_ACC_SOFTRESET, BMI088_TRIGGER_SOFTRESET);
    delay(1);

    // dummy read
    bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID);

    // From datasheet page 12:
    // Power up the sensor
    // wait 1ms
    // enter normal mode by writing '4' to ACC_PWR_CTRL
    // wait for 50 ms
    spiWriteReg(&acc->dev, BMI088_REG_ACC_PWR_CTRL, BMI088_A_ON);
    delay(50);

    for(uint8_t i = 0 ; i<5;i++){
        if(bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_PWR_CTRL) == BMI088_A_ON){
          break;
        }
        delay(5);
    }

    uint8_t range = bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_RANGE);
    spiWriteReg(&acc->dev, BMI088_REG_ACC_RANGE, (range & 0xFC) | BMI088_A_RANGE_12G);

    spiWriteReg(&acc->dev, BMI088_REG_ACC_CONF,
        0x80 | (BMI088_A_BWP_NORMAL<<4) | BMI088_A_ODR_800);

    spiWriteReg(&acc->dev, BMI088_REG_ACC_PWR_CONF, BMI088_A_ACTIVE);
    delay(10);

    acc->acc_1G = 2731; // 32768 / 12G
}

bool bmi088SpiAccDetect(accDev_t *acc)
{
    if(acc->mpuDetectionResult.sensor != BMI_088_SPI){
      return false;
    }
    
    //ACC part uses the same SPI bus as the gyro, so we can just use the gyro's spi instance
    spiSetBusInstance(&acc->dev, SPI_DEV_TO_CFG(spiDeviceByInstance(acc->gyro->dev.bus->busType_u.spi.instance)));
    acc->dev.busType_u.spi.csnPin = acc->gyro->csnAccPin; //get the CS pin from the gyro device config
    acc->dev.txBuf = accBuf;
    acc->dev.rxBuf = &accBuf[32 / 2];

    spiSetClkDivisor(&acc->dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));

    // perform dummy-read to switch the accel to SPI-mode
    bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID);
    delay(5);

    if (bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID) {
      return false;
    }

    //TODO: check  ACC_ERR_REG

    BMI088AccDetected = true;

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = bmi088AccRead;

    busDeviceRegister(&acc->dev);

    return true;
}

#endif