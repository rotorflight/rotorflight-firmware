SITL_TARGETS += $(TARGET)
FEATURES       += ONBOARDFLASH #SDCARD_SPI VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/flash_file.c \
            drivers/serial_tcp.c
