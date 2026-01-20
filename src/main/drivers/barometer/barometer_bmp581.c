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
 *
 * BMP581 Driver
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_BARO) && (defined(USE_BARO_BMP581) || defined(USE_BARO_SPI_BMP581))

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "barometer_bmp581.h"

// 10 MHz max SPI frequency
#define BMP581_MAX_SPI_CLK_HZ 10000000

// I2C Addresses from BMP5 datasheet
#define BMP581_I2C_ADDR                                 0x46 // Primary I2C address
#define BMP581_I2C_ADDR_ALT                             0x47 // Secondary I2C address

// Register Addresses from BMP5 API
#define BMP581_CHIP_ID_REG                              0x01
#define BMP581_CHIP_ID_PRIM                             0x50 // Primary chip ID for BMP580/BMP581
#define BMP581_CHIP_ID_SEC                              0x51 // Secondary chip ID

#define BMP581_REV_ID_REG                               0x02
#define BMP581_CHIP_STATUS_REG                          0x11
#define BMP581_DRIVE_CONFIG_REG                         0x13
#define BMP581_INT_CONFIG_REG                           0x14
#define BMP581_INT_SOURCE_REG                           0x15
#define BMP581_FIFO_CONFIG_REG                          0x16
#define BMP581_FIFO_COUNT_REG                           0x17
#define BMP581_FIFO_SEL_REG                             0x18

#define BMP581_TEMP_DATA_XLSB_REG                       0x1D
#define BMP581_TEMP_DATA_LSB_REG                        0x1E
#define BMP581_TEMP_DATA_MSB_REG                        0x1F

#define BMP581_PRESS_DATA_XLSB_REG                      0x20
#define BMP581_PRESS_DATA_LSB_REG                       0x21
#define BMP581_PRESS_DATA_MSB_REG                       0x22

#define BMP581_INT_STATUS_REG                           0x27
#define BMP581_STATUS_REG                               0x28
#define BMP581_FIFO_DATA_REG                            0x29

#define BMP581_NVM_ADDR_REG                             0x2B
#define BMP581_NVM_DATA_LSB_REG                         0x2C
#define BMP581_NVM_DATA_MSB_REG                         0x2D

#define BMP581_DSP_CONFIG_REG                           0x30
#define BMP581_DSP_IIR_REG                              0x31
#define BMP581_OOR_THR_P_LSB_REG                        0x32
#define BMP581_OOR_THR_P_MSB_REG                        0x33
#define BMP581_OOR_RANGE_REG                            0x34
#define BMP581_OOR_CONFIG_REG                           0x35

#define BMP581_OSR_CONFIG_REG                           0x36
#define BMP581_ODR_CONFIG_REG                           0x37
#define BMP581_OSR_EFF_REG                              0x38

#define BMP581_CMD_REG                                  0x7E

#define BMP581_DATA_FRAME_SIZE                          6 // 3 bytes temperature + 3 bytes pressure

// Power modes from BMP5 API
#define BMP581_POWERMODE_STANDBY                        0x00
#define BMP581_POWERMODE_NORMAL                         0x01
#define BMP581_POWERMODE_FORCED                         0x02
#define BMP581_POWERMODE_CONTINOUS                      0x03
#define BMP581_POWERMODE_DEEP_STANDBY                   0x04

// Oversampling settings from BMP5 API
#define BMP581_OVERSAMPLING_1X                          0x00
#define BMP581_OVERSAMPLING_2X                          0x01
#define BMP581_OVERSAMPLING_4X                          0x02
#define BMP581_OVERSAMPLING_8X                          0x03
#define BMP581_OVERSAMPLING_16X                         0x04
#define BMP581_OVERSAMPLING_32X                         0x05
#define BMP581_OVERSAMPLING_64X                         0x06
#define BMP581_OVERSAMPLING_128X                        0x07

// ODR (Output Data Rate) settings from BMP5 API
#define BMP581_ODR_240_HZ                               0x00
#define BMP581_ODR_218_HZ                               0x01
#define BMP581_ODR_199_HZ                               0x02
#define BMP581_ODR_179_HZ                               0x03
#define BMP581_ODR_160_HZ                               0x04
#define BMP581_ODR_149_HZ                               0x05
#define BMP581_ODR_140_HZ                               0x06
#define BMP581_ODR_129_HZ                               0x07
#define BMP581_ODR_120_HZ                               0x08
#define BMP581_ODR_110_HZ                               0x09
#define BMP581_ODR_100_HZ                               0x0A
#define BMP581_ODR_89_HZ                                0x0B
#define BMP581_ODR_80_HZ                                0x0C
#define BMP581_ODR_70_HZ                                0x0D
#define BMP581_ODR_60_HZ                                0x0E
#define BMP581_ODR_50_HZ                                0x0F
#define BMP581_ODR_45_HZ                                0x10
#define BMP581_ODR_40_HZ                                0x11
#define BMP581_ODR_35_HZ                                0x12
#define BMP581_ODR_30_HZ                                0x13
#define BMP581_ODR_25_HZ                                0x14
#define BMP581_ODR_20_HZ                                0x15
#define BMP581_ODR_15_HZ                                0x16
#define BMP581_ODR_10_HZ                                0x17
#define BMP581_ODR_5_HZ                                 0x18
#define BMP581_ODR_4_HZ                                 0x19
#define BMP581_ODR_3_HZ                                 0x1A
#define BMP581_ODR_2_HZ                                 0x1B
#define BMP581_ODR_1_HZ                                 0x1C
#define BMP581_ODR_0_5_HZ                               0x1D
#define BMP581_ODR_0_25_HZ                              0x1E
#define BMP581_ODR_0_125_HZ                             0x1F

// Default configuration
#define BMP581_PRESSURE_OSR                             BMP581_OVERSAMPLING_32X
#define BMP581_TEMPERATURE_OSR                          BMP581_OVERSAMPLING_128X
#define BMP581_ODR                                      BMP581_ODR_20_HZ

// Register bit masks from BMP5 API
#define BMP581_TEMP_OS_MASK                             0x07
#define BMP581_PRESS_OS_MASK                            0x38
#define BMP581_PRESS_OS_POS                             3
#define BMP581_PRESS_EN_MASK                            0x40
#define BMP581_PRESS_EN_POS                             6

#define BMP581_POWERMODE_MASK                           0x03
#define BMP581_DEEP_DISABLE_MASK                        0x80
#define BMP581_DEEP_DISABLE_POS                         7

#define BMP581_ODR_MASK                                 0x7C
#define BMP581_ODR_POS                                  2

// Soft reset command
#define BMP581_SOFT_RESET_CMD                           0xB6

// Delays
#define BMP581_DELAY_US_SOFT_RESET                      2000
#define BMP581_DELAY_US_STANDBY                         2500

// Note: BMP581 provides compensated temperature and pressure values directly
// The sensor performs internal compensation, so no calibration coefficients
// need to be read or applied in software. The raw values from data registers
// are already compensated and can be converted directly:
// - Temperature: raw_data / 2^16 = temperature in °C
// - Pressure: raw_data / 2^6 = pressure in Pa

// Driver state
static uint8_t bmp581_chip_id = 0;
STATIC_UNIT_TESTED uint32_t bmp581_up = 0;  // Uncompensated pressure
STATIC_UNIT_TESTED int32_t bmp581_ut = 0;  // Uncompensated temperature
static DMA_DATA_ZERO_INIT uint8_t sensor_data[BMP581_DATA_FRAME_SIZE];

// Forward declarations
static void bmp581StartUT(baroDev_t *baro);
static bool bmp581GetUT(baroDev_t *baro);
static bool bmp581ReadUT(baroDev_t *baro);
static void bmp581StartUP(baroDev_t *baro);
static bool bmp581GetUP(baroDev_t *baro);
static bool bmp581ReadUP(baroDev_t *baro);
STATIC_UNIT_TESTED void bmp581Calculate(int32_t *pressure, int32_t *temperature);

// EXTI handler for data ready interrupt
static void bmp581_extiHandler(extiCallbackRec_t* cb)
{
#ifdef DEBUG_BUILD
    static uint32_t bmp581ExtiCallbackCounter = 0;
    bmp581ExtiCallbackCounter++;
#endif

    baroDev_t *baro = container_of(cb, baroDev_t, exti);

    // Read interrupt status register
    uint8_t intStatus = 0;
    busReadRegisterBuffer(&baro->dev, BMP581_STATUS_REG, &intStatus, 1);
}

// Bus initialization
static void bmp581BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP581
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        IOHi(dev->busType_u.spi.csnPin);
        spiSetClkDivisor(dev, spiCalculateDivider(BMP581_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

// Bus deinitialization
static void bmp581BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP581
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiPreinitByIO(dev->busType_u.spi.csnPin);
    }
#else
    UNUSED(dev);
#endif
}

// Configure sensor mode (NORMAL or FORCED)
static void bmp581SetMode(const extDevice_t *dev, uint8_t powermode)
{
    uint8_t reg_data;
    
    // Read current ODR config register
    busReadRegisterBuffer(dev, BMP581_ODR_CONFIG_REG, &reg_data, 1);
    
    // Set deep disable, ODR, and powermode
    // deep_dis = 1 (bit 7), odr (bits 6:2), powermode (bits 1:0)
    reg_data = (1 << BMP581_DEEP_DISABLE_POS) | 
               ((BMP581_ODR << BMP581_ODR_POS) & BMP581_ODR_MASK) | 
               (powermode & BMP581_POWERMODE_MASK);
    
    busWriteRegisterStart(dev, BMP581_ODR_CONFIG_REG, reg_data);
}

// Main detect function
bool bmp581Detect(const bmp581Config_t *config, baroDev_t *baro)
{
    // wait for sensor power-up
    delay(20);

    // Initialize EOC pin if configured
    IO_t baroIntIO = IOGetByTag(config->eocTag);
    if (baroIntIO) {
        IOInit(baroIntIO, OWNER_BARO_EOC, 0);
        EXTIHandlerInit(&baro->exti, bmp581_extiHandler);
        EXTIConfig(baroIntIO, &baro->exti, NVIC_PRIO_BARO_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
        EXTIEnable(baroIntIO);
    }

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    bmp581BusInit(dev);

    // Set default I2C address if not specified
    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = BMP581_I2C_ADDR;
        defaultAddressApplied = true;
    }

    // Read and verify chip ID
    busReadRegisterBuffer(dev, BMP581_CHIP_ID_REG, &bmp581_chip_id, 1);

    // Check if chip ID matches either primary (0x50) or secondary (0x51) ID
    if ((bmp581_chip_id != BMP581_CHIP_ID_PRIM) && (bmp581_chip_id != BMP581_CHIP_ID_SEC)) {
        bmp581BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Configure interrupt if EOC pin is available
    if (baroIntIO) {
        // Configure data ready interrupt: enable, push-pull, active high
        uint8_t intSourceValue = 0x01; // Enable DRDY interrupt
        busWriteRegister(dev, BMP581_INT_SOURCE_REG, intSourceValue);
        
        uint8_t intConfigValue = 0x0A; // int_en=1, int_od=0 (push-pull), int_pol=1 (active high)
        busWriteRegister(dev, BMP581_INT_CONFIG_REG, intConfigValue);
    }

    // Configure oversampling: temperature OSR, pressure OSR, and enable pressure
    uint8_t osrConfig = (BMP581_TEMPERATURE_OSR & BMP581_TEMP_OS_MASK) |
                        ((BMP581_PRESSURE_OSR << BMP581_PRESS_OS_POS) & BMP581_PRESS_OS_MASK) |
                        (1 << BMP581_PRESS_EN_POS); // Enable pressure measurement
    busWriteRegister(dev, BMP581_OSR_CONFIG_REG, osrConfig);

    // Note: BMP581 provides compensated data directly, no calibration read needed

    // Start sensor in NORMAL mode for continuous measurements at configured ODR
    bmp581SetMode(dev, BMP581_POWERMODE_NORMAL);

    // Configure barometer device structure
    baro->ut_delay = 0;
    baro->start_ut = bmp581StartUT;
    baro->get_ut = bmp581GetUT;
    baro->read_ut = bmp581ReadUT;

    baro->start_up = bmp581StartUP;
    baro->get_up = bmp581GetUP;
    baro->read_up = bmp581ReadUP;

    // In NORMAL mode, sensor continuously samples at ODR rate
    // No need to trigger measurements, just read when ready
    baro->up_delay = 50000; // 50ms delay for 20Hz ODR (1000ms / 20Hz = 50ms)

    baro->calculate = bmp581Calculate;

    // Wait for bus with timeout to prevent indefinite hang
    const uint32_t maxRetries = 100;
    uint32_t retries = 0;
    while (busBusy(&baro->dev, NULL)) {
        if (++retries >= maxRetries) {
            // Timeout - bus remained busy, detection failed
            return false;
        }
        delayMicroseconds(10);
    }

    return true;
}

// Temperature measurement start (dummy for combined read)
static void bmp581StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read together with pressure
}

// Temperature read start (dummy for combined read)
static bool bmp581ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read together with pressure
    return true;
}

// Temperature data get (dummy for combined read)
static bool bmp581GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read together with pressure
    return true;
}

// Pressure measurement start
static void bmp581StartUP(baroDev_t *baro)
{
    UNUSED(baro);
    // In NORMAL mode, sensor continuously measures at ODR rate
    // No need to trigger individual measurements
}

// Pressure read start
static bool bmp581ReadUP(baroDev_t *baro)
{
    // Read both pressure and temperature data
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    busReadRegisterBufferStart(&baro->dev, BMP581_TEMP_DATA_XLSB_REG, sensor_data, BMP581_DATA_FRAME_SIZE);

    return true;
}

// Pressure data get
static bool bmp581GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Assemble 24-bit values from sensor data
    // Pressure is unsigned 24-bit
    bmp581_up = (uint32_t)sensor_data[5] << 16 | (uint32_t)sensor_data[4] << 8 | sensor_data[3];
    
    // Temperature is signed 24-bit - assemble and sign-extend to 32-bit
    uint32_t temp_raw = (uint32_t)sensor_data[2] << 16 | (uint32_t)sensor_data[1] << 8 | sensor_data[0];
    // Sign extend from 24-bit to 32-bit: shift left 8 bits, then arithmetic shift right 8 bits
    int32_t signed_temp = (int32_t)(temp_raw << 8) >> 8;
    bmp581_ut = signed_temp;

    return true;
}

// Calculate compensated pressure and temperature
STATIC_UNIT_TESTED void bmp581Calculate(int32_t *pressure, int32_t *temperature)
{
    // BMP581 provides compensated values directly from the sensor
    // Temperature: raw_data is 24-bit signed (already sign-extended in bmp581GetUP), divide by 2^16 to get °C
    // Pressure: raw_data is 24-bit unsigned, divide by 2^6 to get Pa
    
    // Temperature in 0.01°C (fixed point, 2 decimal places)
    // Divide by 2^16 = right shift by 16, then multiply by 100
    // To avoid overflow: (bmp581_ut * 100) >> 16
    // This is safe because bmp581_ut is 24-bit signed (max ~8M), * 100 = ~800M, well within int32 range
    *temperature = (bmp581_ut * 100) >> 16;
    
    // Pressure in Pa (integer)
    // Divide by 2^6 = right shift by 6
    *pressure = (int32_t)(bmp581_up >> 6);
}

#endif /* USE_BARO && (USE_BARO_BMP581 || USE_BARO_SPI_BMP581) */