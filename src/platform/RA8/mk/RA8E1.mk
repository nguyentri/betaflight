#
# RA8E1 Make file include
#

# MCU Configuration
MCU_FAMILY := RA8
MCU_TYPE := RA8E1
TARGET_MCU := R7FA8E1AF
BOARD_NAME := fpb_ra8e1

# Toolchain Configuration
CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump
SIZE := $(CROSS_COMPILE)size

# Architecture and CPU Configuration
ARCH_FLAGS := -mcpu=cortex-m85 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# FSP Configuration
FSP_DIR := $(PLATFORM_DIR)/RA8/fsp/ra
BOARD_DIR := $(PLATFORM_DIR)/RA8/board/$(BOARD_NAME)
FSP_INC_DIR := $(FSP_DIR)/inc
FSP_SRC_DIR := $(FSP_DIR)/src

# FSP Include Directories
INCLUDE_DIRS += \
    $(BOARD_DIR)
    $(FSP_INC_DIR) \
    $(FSP_INC_DIR)/api \
    $(FSP_INC_DIR)/instances \
    $(PLATFORM_DIR)/RA8/include \
    $(PLATFORM_DIR)/RA8/target/$(TARGET) \

# FSP Source Files
FSP_SRC := \
    $(BOARD_DIR)/bsp_init.c \
    $(BOARD_DIR)/bsp_leds.c

FSP_SRC += $(FSP_SRC_DIR)/bsp/mcu/all/bsp_clocks.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_clocks.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_common.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_delay.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_group_irq.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_guard.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_io.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_irq.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_register_protection.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_rom_registers.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_sbrk.c \
    $(FSP_SRC_DIR)/bsp/mcu/all/bsp_security.c \
    $(FSP_SRC_DIR)/r_ioport/r_ioport.c \
    $(FSP_SRC_DIR)/r_gpt/r_gpt.c \
    $(FSP_SRC_DIR)/r_spi/r_spi.c \
    $(FSP_SRC_DIR)/r_sci_uart/r_sci_uart.c \
    $(FSP_SRC_DIR)/r_flash_lp/r_flash_lp.c \
    $(FSP_SRC_DIR)/r_adc/r_adc.c

# Platform-specific source files
PLATFORM_SRC := \
    $(PLATFORM_DIR)/RA8/system_ra8e1.c \
    $(PLATFORM_DIR)/RA8/startup_ra8e1.c \
    $(PLATFORM_DIR)/RA8/io_ra8.c \
    $(PLATFORM_DIR)/RA8/serial_uart_ra8.c \
    $(PLATFORM_DIR)/RA8/bus_spi_ra8.c \
    $(PLATFORM_DIR)/RA8/pwm_output_ra8.c \
    $(PLATFORM_DIR)/RA8/adc_ra8.c \
    $(PLATFORM_DIR)/RA8/flash_ra8.c \
    $(PLATFORM_DIR)/RA8/exti_ra8.c

# Compiler Flags
CFLAGS += $(ARCH_FLAGS)
CFLAGS += -DTARGET_MCU_$(TARGET_MCU)
CFLAGS += -DBSP_MCU_GROUP_RA8E1=1
CFLAGS += -DBSP_CFG_MCU_PART_SERIES=8
CFLAGS += -DBSP_CFG_CPU_CORE=0
CFLAGS += -D_RENESAS_RA_
CFLAGS += -DUSE_HAL_DRIVER

# Optimization flags
CFLAGS += -Os -g3
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra -Wundef -Wshadow
CFLAGS += -Wno-unused-parameter -Wno-unused-function

# Linker Flags
LDFLAGS += $(ARCH_FLAGS)
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage

# Linker Script
LINKER_SCRIPT := $(PLATFORM_DIR)/RA8/link/ra8e1.ld
LDFLAGS += -T$(LINKER_SCRIPT)

# Startup file
STARTUP_SRC := $(PLATFORM_DIR)/RA8/startup/startup_ra8e1.s

# Add FSP sources to build
SRC += $(addprefix $(FSP_SRC_DIR)/, $(FSP_SRC))
SRC += $(PLATFORM_SRC)
SRC += $(STARTUP_SRC)

# Flash configuration
FLASH_SIZE ?= 512
FLASH_PAGE_SIZE := 0x4000

# Memory configuration
RAM_SIZE := 128
CCRAM_SIZE := 0

# Board-specific defines
CFLAGS += -DFLASH_SIZE=$(FLASH_SIZE)
CFLAGS += -DFLASH_PAGE_SIZE=$(FLASH_PAGE_SIZE)

# Enable specific peripherals for RA8E1
CFLAGS += -DUSE_SPI
CFLAGS += -DUSE_UART
CFLAGS += -DUSE_PWM
CFLAGS += -DUSE_ADC
CFLAGS += -DUSE_FLASH
CFLAGS += -DUSE_EXTI

# Board-specific sensor support
CFLAGS += -DUSE_MPU9250
CFLAGS += -DUSE_BMP280

# Debug configuration
ifeq ($(DEBUG), GDB)
    CFLAGS += -Og -ggdb3
    CFLAGS += -DDEBUG
else ifeq ($(DEBUG), INFO)
    CFLAGS += -Os -g3
    CFLAGS += -DNDEBUG
else
    CFLAGS += -Os
    CFLAGS += -DNDEBUG
endif