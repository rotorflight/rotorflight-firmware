#
# CH32H4 Make file include
#

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CH32H41x/Cmsis

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/CH32H41x/Peripheral

# USB Middleware
MIDDLEWARES_DIR = $(ROOT)/lib/main/CH32H41x/middlewares

STDPERIPH_SRC   = \
        ch32h417_adc.c \
        ch32h417_can.c \
        ch32h417_crc.c \
        ch32h417_dac.c \
        ch32h417_dbgmcu.c \
        ch32h417_dfsdm.c \
        ch32h417_dma.c \
        ch32h417_dvp.c \
        ch32h417_ecdc.c \
        ch32h417_eth.c \
        ch32h417_exti.c \
        ch32h417_flash.c \
        ch32h417_fmc.c \
        ch32h417_gpha.c \
        ch32h417_gpio.c \
        ch32h417_hsadc.c \
        ch32h417_hsem.c \
        ch32h417_i2c.c \
        ch32h417_i3c.c \
        ch32h417_ipc.c \
        ch32h417_iwdg.c \
        ch32h417_lptim.c \
        ch32h417_ltdc.c \
        ch32h417_opa.c \
        ch32h417_pwr.c  \
        ch32h417_qspi.c \
        ch32h417_rcc.c \
        ch32h417_rng.c \
        ch32h417_rtc.c \
        ch32h417_sai.c \
        ch32h417_sdio.c \
        ch32h417_sdmmc.c \
        ch32h417_spi.c \
        ch32h417_swpmi.c \
        ch32h417_tim.c \
        ch32h417_usart.c \
        ch32h417_wwdg.c

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC) $(CMSIS_CORE_SRC)

STARTUP_SRC     = startup/startup_ch32h417_v5f.S

VPATH           := $(VPATH):$(CMSIS_DIR)/Core:$(CMSIS_DIR)/Debug:$(STDPERIPH_DIR)/src:$(MIDDLEWARES_DIR)

# CMSIS RISC-V core support (cycle counter, interrupt helpers)
CMSIS_CORE_SRC  = core_riscv.c

VCP_SRC = \
           ch32h41x_hs/usb_ch32h41x_usbhs_reg.c \
           class/cdc/usbd_cdc_acm.c \
           class/msc/usbd_msc.c \
           core/usbd_core.c \
           board/cdc_vcp_ch32h41x.c

VCP_INCLUDES = \
        $(MIDDLEWARES_DIR)/ch32h41x_hs \
        $(MIDDLEWARES_DIR)/class/cdc \
        $(MIDDLEWARES_DIR)/common \
        $(MIDDLEWARES_DIR)/board \
        $(MIDDLEWARES_DIR)/core

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup \
                   $(SRC_DIR)/drivers \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/Core \
                   $(CMSIS_DIR)/Debug \
                   $(MIDDLEWARES_DIR)/class/msc \
                   $(VCP_INCLUDES)

LD_SCRIPT       = $(LINKER_DIR)/ch32h41x_v5f.ld

# Override LD_FLAGS for RISC-V (remove --no-wchar-size-warning which is ARM-specific)
LD_FLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -Wl,--print-memory-usage \
              -T$(LD_SCRIPT) \
               $(EXTRA_LD_FLAGS)

# RISC-V Architecture Flags
# rv32imafc  = 32-bit, Integer, Multiply, Atomic, Float, Compressed
# zba/zbb/zbc/zbs = Bit manipulation extensions
# xw = WCH custom extension
# ilp32f = soft-double, hard-float ABI
ARCH_FLAGS      = -march=rv32imafc_zba_zbb_zbc_zbs_xw -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fmax-errors=5 -fsigned-char -fsingle-precision-constant -Wunused -Wuninitialized -lprintfloat -g

DEVICE_FLAGS    += -DUSE_CHBSP_DRIVER -DCH32H417 -DCH32H41x -DHSE_VALUE=$(HSE_VALUE) -DCH32 -DUSE_OTG_HOST_MODE -DCH32H4

# Upstream FBUS master currently indexes two flag channels beyond its 16-channel buffer.
# Keep the CH32H4 port buildable until that upstream bounds issue is corrected.
CFLAGS          += -Wno-error=array-bounds

# Disable double-promotion warning (RISC-V single-precision float)
DOUBLE_PROMOTION        := no
# Disable LTO for now (can enable later once everything compiles)
LTO                     := no
OPTIMISATION_BASE := -ffast-math -fmerge-all-constants
# Override the ARM_SDK_PREFIX to use WCH RISC-V toolchain
ARM_SDK_PREFIX := riscv-wch-elf-

# MCU-specific common source files
# These will be created in Parts 2-5. For now, list only what exists.
MCU_COMMON_SRC = \
        startup/system_ch32h417.c \
        startup/ch32h417_it.c \
        drivers/accgyro/accgyro_mpu.c \
        drivers/adc.c \
        drivers/bus_spi_config.c \
        drivers/bus_i2c_timing.c \
        drivers/dshot_bitbang_decode.c \
        drivers/dshot_bitbang_stdperiph_ch32h41x.c \
        drivers/inverter.c \
        drivers/serial_escserial.c \
        drivers/serial_pinconfig.c \
        drivers/usb_io.c \
        drivers/usb_msc_common.c \
        drivers/adc_ch32h41x.c \
        drivers/bus_i2c_config.c \
        drivers/serial_uart.c \
        drivers/serial_uart_ch32bsp.c \
        drivers/serial_uart_pinconfig.c \
        drivers/dma_ch32h41x.c \
        drivers/dma_common.c \
        drivers/dshot_bitbang_ch32h41x.c \
        drivers/serial_uart_ch32h41x.c \
        drivers/pwm_output_dshot_shared.c \
        drivers/pwm_output_dshot.c \
        drivers/pwm_output.c \
        drivers/bus_i2c_ch32h41x.c \
        drivers/bus_i2c_busdev.c \
        drivers/bus_spi_ch32h41x.c \
        sensors/acceleration.c \
        sensors/acceleration_init.c \
        drivers/dshot_dpwm.c \
        drivers/motor.c \
        drivers/serial_usb_vcp_ch32h4.c \
        drivers/usb_msc_ch32h41x.c \
        drivers/persistent_ch32h41x.c \
        drivers/timer_ch32h41x.c \
        $(VCP_SRC) \
        drivers/system_ch32h41x.c \
        drivers/system.c \
        drivers/light_ws2811strip.c \
        drivers/light_ws2811strip_ch32h41x.c

MCU_COMMON_SRC += msc/usbd_storage.c

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
MSC_SRC += drivers/usb_msc_sdcard_spi_ch32h41x.c
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
MSC_SRC += msc/usbd_storage_emfat.c msc/emfat.c msc/emfat_file.c
endif

MCU_EXCLUDES += drivers/adc_stm32f4xx.c drivers/adc_stm32f7xx.c \
                drivers/adc_stm32g4xx.c drivers/adc_stm32h7xx.c \
                


# Speed-optimized source files
SPEED_OPTIMISED_SRC += \
            drivers/bus_spi.c

# Size-optimized source files
SIZE_OPTIMISED_SRC += \
            drivers/bus_i2c_timing.c \
            drivers/inverter.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c



# CH32H4.mk — WCH RISC-V toolchain is stricter about enum*/uint8_t* pointer
# mismatches than the pinned ARM GCC 9.3.1; several shared CMS menu files
# rely on this being non-fatal. Demote to a warning to match ARM behavior.
CFLAGS += -Wno-error=incompatible-pointer-types