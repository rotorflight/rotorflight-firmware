F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_fake.c \
            drivers/max7456.c
