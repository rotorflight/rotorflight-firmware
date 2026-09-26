CH32H417_TARGETS += CH32H417

TARGET_MCU        := CH32H417
TARGET_MCU_FAMILY := CH32H4
MCU_FLASH_SIZE    := 960
DEVICE_FLAGS       = -DCH32H415 -DCH32H41x -DCore_V5F -DRISC_V
HSE_VALUE          = 25000000

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/barometer/barometer_dps310.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_spi_icm40608.c \
            drivers/accgyro/accgyro_spi_icm40609.c \
            
            
FEATURES += ONBOARDFLASH SDCARD_SPI