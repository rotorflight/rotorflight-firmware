
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

# dyad is third-party host code working in doubles: it must not get the
# float constants below. dyad_getTime() is `tv_sec + tv_usec / 1e6`, which in
# float has a resolution of 128 s at today's Unix time (1024 s for the Windows
# FILETIME): dyad's clock jumped in 128 s steps, its 120 s stream timeout fired
# at random, and a client connection was closed seconds after it was opened.
%/dyad.o: CFLAGS += -fno-single-precision-constant -Wno-double-promotion

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

# SITL is a native build: no ARM cross toolchain. On Windows, use the MinGW-w64
# GCC from 'make mingw_sdk_install' (tools/mingw64) when present, else the gcc
# on PATH.
MINGW_SDK_DIR   ?= $(TOOLS_DIR)/mingw64
ifeq ($(OSFAMILY),windows)
  ifeq ($(shell [ -x "$(MINGW_SDK_DIR)/bin/gcc.exe" ] && echo "exists"), exists)
    ARM_SDK_PREFIX := $(MINGW_SDK_DIR)/bin/
  else
    ARM_SDK_PREFIX =
  endif
else
ARM_SDK_PREFIX  =
endif

MCU_EXCLUDES = \
            drivers/adc.c \
            pg/adc.c \
            drivers/freq.c \
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
            telemetry/ghst.c \
            telemetry/srxl.c \
            io/displayport_oled.c \
            common/string_light.c

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# MinGW/MSYS (native Windows) toolchains link libc implicitly and have no
# separate librt (clock_gettime/nanosleep are provided via winpthreads /
# msvcrt instead), so -lc/-lrt must be omitted there; they're required on
# Linux/macOS. -static on Windows, so the executable starts without the
# toolchain's DLLs (libwinpthread-1.dll, libgcc_s_seh-1.dll) on PATH.
ifeq ($(OSFAMILY),windows)
LD_FLAGS    := \
              -lm \
              -lpthread \
              -lws2_32 \
              -lwinmm \
              -static \
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
