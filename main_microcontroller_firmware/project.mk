# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

#Uncomment below if use RTT
TERMINAL_IO_VARIANT = TERMINAL_IO_USE_SEGGER_RTT

ifeq ($(TERMINAL_IO_VARIANT), TERMINAL_IO_USE_SEGGER_RTT)
IPATH += ../third_party/RTT
VPATH += ../third_party/RTT
endif

ifneq ($(TERMINAL_IO_VARIANT),)
PROJ_CFLAGS += -D$(TERMINAL_IO_VARIANT)
endif

# use the BSP defined in the root of the main firmware location
BSP_SEARCH_DIR := $(abspath ./)
BOARD = BSP

# Use custom linker script with separate SRAM regions for DMA buffers
# This extends the default MSDK linker script with additional SRAM sections
LINKERFILE = max32650_custom.ld
LINKERPATH = $(abspath ./)

LIB_CMSIS_DSP = 1

PROJ_CFLAGS += -Dtimegm=mktime # needed for the minmea lib

PROJ_LDFLAGS += -Wl,--print-memory-usage
PROJ_CFLAGS += -DNATIVE_SDHC=1

#FIRST_SET_RTC = 1
PROJ_CFLAGS += -DSDHC_CLK_FREQ=23750000

PROJ_CFLAGS+=-mno-unaligned-access

MXC_OPTIMIZE_CFLAGS = -O2

# do manual pin config, don't use the pin constants in msdk/Libraries/PeriphDrivers/Source/SYS/pins_me14.c
# this means we need to explicitly set up all the pins for the peripherals we use
PROJ_CFLAGS += -DMSDK_NO_GPIO_CLK_INIT

# Use the SDHC lib in ./MSDK_overrides/SDHC/ instead of the files supplied by the MSKD, this is because the MSKD
# version is hardcoded to 1-bit mode, and we want 4-bit mode
FATFS_VERSION = ff15
SDHC_DRIVER_DIR = ./MSDK_overrides/SDHC/
include ./MSDK_overrides/SDHC/sdhc.mk
include ./MSDK_overrides/SDHC/ff15/fat32.mk

IPATH += ./MSDK_overrides/SDHC/Include/
IPATH += ./MSDK_overrides/SDHC/ff15/source/
IPATH += ./MSDK_overrides/SDHC/ff15/source/conf/

IPATH += ./lib/audio/
VPATH += ./lib/audio/

IPATH += ./lib/sd_card/
VPATH += ./lib/sd_card/

IPATH += ./lib/sensors/
VPATH += ./lib/sensors/

IPATH += ./lib/timekeeping/
VPATH += ./lib/timekeeping/

IPATH += ./lib/ext_memory/
VPATH += ./lib/ext_memory/

IPATH += ./lib/utils/
VPATH += ./lib/utils/

IPATH += ./third_party/minmea/
VPATH += ./third_party/minmea/

IPATH += ./third_party/littlefs/
VPATH += ./third_party/littlefs/

IPATH += ./third_party/RTT/
VPATH += ./third_party/RTT/


VPATH += ./app/
