
# src/main/common holds time.h, which shadows glibc's <time.h> when the
# directory is on the -I path (pthread.h then cannot see clockid_t).
# Search it for #include "..." only.
INCLUDE_DIRS    := $(filter-out $(SRC_DIR)/common,$(INCLUDE_DIRS)) \
                   $(ROOT)/lib/main/dyad

# build/atomic.c provides the emulated atomic_BASEPRI storage used by
# ATOMIC_BLOCK() under UNIT_TEST/SIMULATOR_BUILD; it's not part of the
# common source list since real hardware targets use inline BASEPRI asm
# instead and never reference the variable.
MCU_COMMON_SRC  := $(ROOT)/lib/main/dyad/dyad.c \
                   $(ROOT)/src/main/build/atomic.c

#Flags
# Same float-constant semantics as the STM32 targets: an unsuffixed constant
# is a float, so -Wdouble-promotion behaves as it does on a board build.
ARCH_FLAGS      = -fsingle-precision-constant
DEVICE_FLAGS    =
LD_SCRIPT       = src/main/target/SITL/pg.ld
STARTUP_SRC     =

TARGET_FLAGS    = -D$(TARGET) -iquote $(SRC_DIR)/common
MCU_FLASH_SIZE  := 2048

# On MinGW/MSYS (native Windows) toolchains, GCC defaults to MS-style bitfield
# layout, which breaks the packed wire-format structs (SBUS/FBUS/etc.) that
# assume GNU/System V bitfield packing, as used on the other (ARM/Linux)
# targets. Force GNU-style bitfield layout so the STATIC_ASSERTs on those
# struct sizes hold on Windows too. This flag is a no-op on Linux/macOS gcc.
ifeq ($(OSFAMILY),windows)
ARCH_FLAGS      := $(ARCH_FLAGS) -mno-ms-bitfields
endif

ARM_SDK_PREFIX  =

MCU_EXCLUDES = \
            drivers/adc.c \
            drivers/bus_i2c.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/dma.c \
            drivers/pwm_output.c \
            drivers/timer.c \
            drivers/system.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_uart.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/rx/rx_xn297.c \
            drivers/display_ug2864hsweg01.c \
            telemetry/crsf.c \
            telemetry/ghst.c \
            telemetry/srxl.c \
            io/displayport_oled.c \
            common/string_light.c

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# MinGW/MSYS (native Windows) toolchains link libc implicitly and have no
# separate librt (clock_gettime/nanosleep are provided via winpthreads /
# msvcrt instead), so -lc/-lrt must be omitted there; they're required on
# Linux/macOS.
ifeq ($(OSFAMILY),windows)
LD_FLAGS    := \
              -lm \
              -lpthread \
              -lws2_32 \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT)
else
LD_FLAGS    := \
              -lm \
              -lpthread \
              -lc \
              -lrt \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT)
endif

ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
LD_FLAGS     += \
              -static \
              -static-libgcc
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif
