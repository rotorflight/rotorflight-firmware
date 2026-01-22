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

/*
 * STM32F405
 */

#if defined(STM32F405)

#define TARGET_BOARD_IDENTIFIER "S405"

#define USBD_PRODUCT_STRING     "Rotorflight STM32F405"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 6)

#define USE_INVERTER

#define USE_SDCARD_SDIO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define VOLTAGE_TASK_FREQ_HZ     100
#define CURRENT_TASK_FREQ_HZ     100
#define ESC_SENSOR_TASK_FREQ_HZ  100

#define DEFAULT_FEATURES         (FEATURE_DYN_NOTCH)


/*
 * STM32F411
 */

#elif defined(STM32F411)

#define TARGET_BOARD_IDENTIFIER "S411"

#define USBD_PRODUCT_STRING     "Rotorflight STM32F411"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_UART1
#define USE_UART2
#define USE_UART6

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 3)

#define USE_INVERTER

#undef  USE_MULTI_GYRO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define VOLTAGE_TASK_FREQ_HZ     100
#define CURRENT_TASK_FREQ_HZ     100
#define ESC_SENSOR_TASK_FREQ_HZ  100

#define DEFAULT_FEATURES         (0)


/*
 * STM32F7x2
 */

#elif defined(STM32F7X2)

#define TARGET_BOARD_IDENTIFIER "S7X2"

#define USBD_PRODUCT_STRING     "Rotorflight STM32F7x2"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 6)

#define USE_SDCARD_SDIO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define VOLTAGE_TASK_FREQ_HZ     200
#define CURRENT_TASK_FREQ_HZ     200
#define ESC_SENSOR_TASK_FREQ_HZ  200

#define DEFAULT_FEATURES         (FEATURE_DYN_NOTCH)


/*
 * STM32F745
 */

#elif defined(STM32F745)

#define TARGET_BOARD_IDENTIFIER "S745"

#define USBD_PRODUCT_STRING     "Rotorflight STM32F745"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 8)

#define USE_SDCARD_SDIO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define VOLTAGE_TASK_FREQ_HZ     200
#define CURRENT_TASK_FREQ_HZ     200
#define ESC_SENSOR_TASK_FREQ_HZ  200

#define DEFAULT_FEATURES         (FEATURE_DYN_NOTCH)


/*
 * STM32G47x
 */

#elif defined(STM32G47X)

#define TARGET_BOARD_IDENTIFIER "SG47"

#define USBD_PRODUCT_STRING     "Rotorflight STM32G47x"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_LPUART1

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 6)

#undef  USE_MULTI_GYRO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define VOLTAGE_TASK_FREQ_HZ     100
#define CURRENT_TASK_FREQ_HZ     100
#define ESC_SENSOR_TASK_FREQ_HZ  100

#define DEFAULT_FEATURES         (FEATURE_DYN_NOTCH)


/*
 * STM32H743
 */

#elif defined(STM32H743)

#define TARGET_BOARD_IDENTIFIER "SH74"

#define USBD_PRODUCT_STRING     "Rotorflight STM32H743"

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8
#define USE_LPUART1

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 9)

#define USE_SDCARD_SDIO

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define VOLTAGE_TASK_FREQ_HZ     250
#define CURRENT_TASK_FREQ_HZ     250
#define ESC_SENSOR_TASK_FREQ_HZ  250

#define USE_LEDSTRIP_CACHE_MGMT

#define DEFAULT_FEATURES         (FEATURE_DYN_NOTCH)


/*
 * UNKNOWN target
 */

#elif !defined(UNIT_TEST)
#error "No resources defined for this Unified Target."
#endif


// Treat the target as unified, and expect manufacturer id / board name
// to be supplied when the board is configured for the first time
#define USE_UNIFIED_TARGET

#define USE_CUSTOM_DEFAULTS

#define USE_BEEPER

#undef  USE_GYRO_DLPF_EXPERIMENTAL

#define USE_ACC
#define USE_GYRO

#define USE_ACC_MPU6050
#define USE_GYRO_MPU6050
#define USE_ACC_MPU6500
#define USE_GYRO_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define USE_ACCGYRO_LSM6DSO
#define USE_ACCGYRO_BMI160
#define USE_ACCGYRO_BMI270
#define USE_ACCGYRO_SPI_BMI323
#define USE_ACCGYRO_SPI_BMI088
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P

#define USE_MAG
#define USE_MAG_DATA_READY_SIGNAL
#define USE_MAG_HMC5883
#define USE_MAG_SPI_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8963
#define USE_MAG_MPU925X_AK8963
#define USE_MAG_SPI_AK8963
#define USE_MAG_AK8975

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define USE_BARO_BMP388
#define USE_BARO_SPI_BMP388
#define USE_BARO_LPS
#define USE_BARO_SPI_LPS
#define USE_BARO_QMP6988
#define USE_BARO_SPI_QMP6988
#define USE_BARO_DPS310
#define USE_BARO_SPI_DPS310
#define USE_BARO_BMP581
#define USE_BARO_SPI_BMP581

#define USE_SDCARD
#define USE_SDCARD_SPI

#define USE_FLASHFS
#define USE_FLASHFS_LOOP
#define USE_FLASH_TOOLS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G          // 1Gb NAND flash support
#define USE_FLASH_W25M             // Stacked die support
#define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
#define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
#define USE_FLASH_W25Q128FV        // 16MB Winbond 25Q128

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_VCP

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define UNIFIED_SERIAL_PORT_COUNT       3

#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC

#define USE_FREQ_SENSOR

#define USE_SERVO_GEOMETRY_CORRECTION

#define USE_CMS

#undef USE_CRSF_V3

#undef USE_OSD
#undef USE_MAX7456
#undef USE_RCDEVICE
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
#undef USE_CAMERA_CONTROL

#undef USE_RX_FRSKY_SPI_D
#undef USE_RX_FRSKY_SPI_X
#undef USE_RX_SFHSS_SPI
#undef USE_RX_REDPINE_SPI
#undef USE_RX_FRSKY_SPI_TELEMETRY
#undef USE_RX_CC2500_SPI_PA_LNA
#undef USE_RX_CC2500_SPI_DIVERSITY
#undef USE_RX_FLYSKY
#undef USE_RX_FLYSKY_SPI_LED
#undef USE_RX_SPEKTRUM
#undef USE_RX_SPEKTRUM_TELEMETRY
#undef USE_RX_EXPRESSLRS
#undef USE_RX_SX1280
#undef USE_RX_SX127X

#ifdef USE_VTX_CONTROL
#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#endif

#ifdef USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF
#endif
