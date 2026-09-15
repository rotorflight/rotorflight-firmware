#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "CH417"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING "Rotorflight CH32H415"
#endif

#ifndef CH32H41x
#define CH32H41x
#endif

#define USE_TARGET_CONFIG
#define TARGET_VALIDATECONFIG
#define USE_EXTERN_1V2

#define USE_VCP
#define USE_USB_DETECT

#define SERIAL_PORT_COUNT 9
#define USABLE_TIMER_CHANNEL_COUNT 6

// UARTs (from schematic: U1–U8 mapped)
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

// UART pin mapping per FC-30_H415-v0.3 schematic
#define UART1_TX_PIN PB6  // U1TX
#define UART1_RX_PIN PB7  // U1RX
#define UART2_TX_PIN PA2  // U2TX
#define UART2_RX_PIN PA3  // U2RX
#define UART3_TX_PIN PA13 // U3TX
#define UART3_RX_PIN PA14 // U3RX
#define UART4_TX_PIN PC6  // U4TX
#define UART4_RX_PIN PC7  // U4RX
#define UART5_TX_PIN PE0  // U5TX
#define UART5_RX_PIN PF5  // U5RX
#define UART6_TX_PIN PA0  // U6TX
#define UART6_RX_PIN PA1  // U6RX
#define UART7_TX_PIN NONE // U7TX not routed on this board
#define UART7_RX_PIN PB12 // U7RX
#define UART8_TX_PIN PB4  // U8TX
#define UART8_RX_PIN PB3  // U8RX

#define USE_BARO
#define I2C_DEVICE (I2CDEV_2)
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_DPS310
// Board barometer is an SPA06-003 (U3) - register compatible with DPS310.
// DEFAULT_BARO_DPS310 selects I2C2 (BARO_I2C_INSTANCE) as the baro bus type,
// otherwise baro_busType defaults to BUS_TYPE_NONE and the baro is never probed.
#define DEFAULT_BARO_DPS310
#define BARO_I2C_INSTANCE (I2CDEV_2)

#define I2C1_SCL NONE
#define I2C1_SDA NONE
#define I2C2_SCL PC0 // schematic: I2C2SCL
#define I2C2_SDA PC1 // schematic: I2C2SDA
#define I2C3_SCL NONE
#define I2C3_SDA NONE
#define I2C4_SCL NONE
#define I2C4_SDA NONE

// SPI buses
// Gyro ICM42688P on SPI3 (CS=PF3, INT=PA15, CLKIN=PC9 - CLKIN needs a timer
// channel which is not mapped yet, so the gyro runs on its internal clock)
#define SPI3_SCK_PIN PC10  // SPI3CLK
#define SPI3_MISO_PIN PC11 // SPI3MISO
#define SPI3_MOSI_PIN PC12 // SPI3MOSI

#define GYRO_1_SPI_INSTANCE SPI3
#define GYRO_1_CS_PIN PF3    // IMUCS
#define GYRO_1_EXTI_PIN PA15 // IMUEXTI

// SD card slot on SPI2 (card detect not wired)
#define SPI2_SCK_PIN PB13  // SPI2CLK
#define SPI2_MISO_PIN PB14 // SPI2MISO
#define SPI2_MOSI_PIN PB15 // SPI2MOSI
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_CS_PIN PE11
#define SDCARD_SPI_INSTANCE SPI2

// Onboard SPI flash (U2) on SPI4
// for on board M25P16 flash
#define USE_FLASHFS
#define USE_FLASH_LOOP
#define USE_FLASHFS_LOOP

#define USE_FLASH
#define USE_FLASH_M25P16
#define FLASH_SPI_INSTANCE SPI4
#define SPI4_SCK_PIN PE12  // SPI4CLK
#define SPI4_MISO_PIN PE13 // SPI4MISO
#define SPI4_MOSI_PIN PE14 // SPI4MOSI
#define FLASH_CS_PIN PB10
// Flash CS = PB10 (FLASHCS) - enable USE_FLASH_CHIP when flash support is ported

// SPI1 not used
#define SPI1_SCK_PIN NONE
#define SPI1_MISO_PIN NONE
#define SPI1_MOSI_PIN NONE

// Status LED (net LED0 drives the blue LED2, active low)
#define LED0_PIN PC4

// Beeper via Q2 on PF4
#define BEEPER_PIN PF4

// Battery voltage (R39/R40 divider) and current sensor
#define VBAT_ADC_PIN PC3
#define CURRENT_METER_ADC_PIN PC2

// Target IO masks (keep defaults unless schematic differs)
#define TARGET_IO_PORTA 0xff1f
#define TARGET_IO_PORTB 0xfffc
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

// Peripheral enables
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DMA_ENABLE_EARLY

#define USE_EXTI
#define USE_GYRO
#define USE_ACC
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_ICM40608
#define USE_ACC_SPI_ICM40608
#define USE_ACCGYRO_ICM40609D
#define USE_ICM40608_AS_ICM40609D

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_PERSISTENT_MSC_RTC

#define USE_ADC
#define USE_ADC_INTERNAL
#define ADC_INSTANCE ADC1
#define USE_DMA_SPEC
#define USE_TIMER
#define USE_TIMER_DMA

#define ADC1_DMA_OPT 0
#define ADC2_DMA_OPT 0

#undef USE_TRANSPONDER
#undef USE_DSHOT_DMAR
#define USE_DSHOT_BITBANG

#define USE_BEEPER
#define USE_OSD
#define USE_OSD_HD
#define USE_MAG
#define USE_MAG_DATA_READY_SIGNAL
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE

#define USE_SERVOS
#define USE_SERVO_GEOMETRY

#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#define USE_ESCSERIAL

// External magnetic/optical RPM sensor (timer input capture).
// The configurator Motors tab "RPM sensor" switch maps to the FREQ_SENSOR
// feature (bit 28). Without USE_FREQ_SENSOR, validateAndFixConfig() calls
// featureDisableImmediate(FEATURE_FREQ_SENSOR) at every boot and every
// EEPROM write, so the toggle always reverts to disabled after Save & Reboot.
// NOTE: still needs a TIM_USE_FREQ timer channel in target.c (RPM pad) to
// actually measure anything; until then RPM comes from DShot bidir telemetry.
#define USE_FREQ_SENSOR

#define FLASH_PAGE_SIZE ((uint32_t)0x2000) // 8K sectors
