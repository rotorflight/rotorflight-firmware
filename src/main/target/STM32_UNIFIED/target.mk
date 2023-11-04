
ifneq ($(findstring STM32F411,$(TARGET)),)
F411_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SPI ONBOARDFLASH
endif

ifneq ($(findstring STM32F405,$(TARGET)),)
F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SPI SDCARD_SDIO ONBOARDFLASH
endif

ifneq ($(findstring STM32F7X2,$(TARGET)),)
F7X2RE_TARGETS  += $(TARGET)
FEATURES        += VCP SDCARD_SPI SDCARD_SDIO ONBOARDFLASH
endif

ifneq ($(findstring STM32F745,$(TARGET)),)
F7X5XG_TARGETS  += $(TARGET)
FEATURES        += VCP SDCARD_SPI SDCARD_SDIO ONBOARDFLASH
endif

ifneq ($(findstring STM32G47X,$(TARGET)),)
G47X_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SPI ONBOARDFLASH
endif

ifneq ($(findstring STM32H743,$(TARGET)),)
H743xI_TARGETS  += $(TARGET)
FEATURES        += VCP SDCARD_SPI SDCARD_SDIO ONBOARDFLASH
endif

TARGET_SRC = \
    $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
    $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
    $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
    $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
    rx/cc2500_common.c \
    rx/cc2500_frsky_shared.c \
    rx/cc2500_frsky_d.c \
    rx/cc2500_frsky_x.c \
    rx/cc2500_sfhss.c \
    rx/cc2500_redpine.c \
    rx/a7105_flysky.c \
    rx/cyrf6936_spektrum.c \
    drivers/rx/expresslrs_driver.c \
    rx/expresslrs.c \
    rx/expresslrs_common.c \
    rx/expresslrs_telemetry.c \
    drivers/rx/rx_cc2500.c \
    drivers/rx/rx_a7105.c \
    drivers/rx/rx_cyrf6936.c \
    drivers/rx/rx_sx127x.c \
    drivers/rx/rx_sx1280.c \

ifneq ($(findstring _OSD,$(TARGET)),)
VARIANT_SRC = \
    drivers/max7456.c \
    drivers/vtx_rtc6705.c \
    drivers/vtx_rtc6705_soft_spi.c
endif
