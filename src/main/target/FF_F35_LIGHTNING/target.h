/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "FF35"

#define USBD_PRODUCT_STRING     "FURIOUS F35-LIGHTNING"

#define USE_TARGET_CONFIG
// #define USE_HARDWARE_REVISION_DETECTION

#define LED0_PIN                    PC10 // Blue LED
// #define LED1                    PC10 // Red LED
// #define LED2                    PC10 // Green LED

#define BEEPER_PIN                  PA1
#define BEEPER_INVERTED

#define USE_DSHOT

// MPU interrupt
#define USE_EXTI
#define GYRO_1_EXTI_PIN            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU9250
#define GYRO_1_CS_PIN          PC0
#define GYRO_1_SPI_INSTANCE         SPI3

#define USE_ACC
#define USE_ACC_SPI_MPU9250
#define GYRO_1_ALIGN       CW180_DEG

#define USE_MAG
#define USE_MAG_AK8963
// #define MAG_I2C_BUS            BUS_I2C1

#define USE_BARO
#define USE_BARO_BMP280
#define BMP280_CS_PIN           PC5
#define BMP280_SPI_BUS          BUS_SPI3

#define USE_MAX7456
#define MAX7456_CS_PIN          PA4
#define MAX7456_SPI_BUS         BUS_SPI1

#define USE_VCP
#define USE_USB_DETECT
// #define VBUS_SENSING_PIN        PA9

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PA8

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA3     // shared with UART2 RX
#define SOFTSERIAL_1_TX_PIN     PA2     // shared with UART2 TX

#define SERIAL_PORT_COUNT       8       //VCP, UART1, UART2, UART3, UART4, UART5, UART6

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            NONE
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            NONE
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
// #define AIRSPEED_ADC_PIN       PC3
#define CURRENT_METER_ADC_PIN   PC2
#define VBAT_ADC_PIN            PC1

// #define USE_PITOT_MS4525
// #define PITOT_I2C_BUS           BUS_I2C1

// #define TEMPERATURE_I2C_BUS     BUS_I2C1
// #define BNO055_I2C_BUS          BUS_I2C1

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_GPS | FEATURE_TELEMETRY| FEATURE_SOFTSERIAL)

#define CURRENT_METER_SCALE     250

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS       6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9