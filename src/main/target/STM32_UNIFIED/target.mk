
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
CUSTOM_DEFAULTS_EXTENDED = yes
endif

TARGET_SRC = \
    $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
    $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
    $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
    $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
