#
# RA8E1 Make file include
#

# FSP Configuration - simplified paths
FSP_RA_FSP_DIR := $(TARGET_PLATFORM_DIR)/fsp/RA8E1
FSP_RA_CFG_DIR := $(FSP_RA_FSP_DIR)/ra_cfg/fsp_cfg
FSP_RA_GEN_DIR := $(FSP_RA_FSP_DIR)/ra_gen
FSP_RA_DRV_DIR := $(FSP_RA_FSP_DIR)/ra
FSP_RA_SAMPLE_DIR := $(FSP_RA_FSP_DIR)/src

# Platform includes
INCLUDE_DIRS += RA8

# FSP Include Directories
INCLUDE_DIRS += \
	$(FSP_RA_DRV_DIR)/arm/CMSIS_6/CMSIS/Core/Include \
	$(FSP_RA_DRV_DIR)/arm/CMSIS_6/CMSIS/Core/Include/m-profile \
    $(FSP_RA_DRV_DIR)/board/ra8e1_fpb \
    $(FSP_RA_DRV_DIR)/fsp/inc\
    $(FSP_RA_DRV_DIR)/fsp/inc/api \
    $(FSP_RA_DRV_DIR)/fsp/inc/instances \
	$(FSP_RA_DRV_DIR)/fsp/src/bsp/cmsis/Device/RENESAS/Include \
	$(FSP_RA_DRV_DIR)/fsp/src/bsp/mcu/all \
	$(FSP_RA_DRV_DIR)/fsp/src/bsp/mcu/ra8e1

# fsp configuration
INCLUDE_DIRS += $(FSP_RA_CFG_DIR) \
			$(FSP_RA_CFG_DIR)/bsp

# fsp generation source
INCLUDE_DIRS += $(FSP_RA_GEN_DIR)

# FSP Fixed Source Files
VARIANT_SRC += \
    $(FSP_RA_DRV_DIR)/board/ra8e1_fpb /bsp_init.c \
    $(FSP_RA_DRV_DIR)/board/ra8e1_fpb /bsp_leds.c

VARIANT_SRC += $(FSP_RA_DRV_DIR)/bsp/mcu/all/bsp_clocks.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_clocks.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_common.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_delay.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_group_irq.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_guard.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_io.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_irq.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_register_protection.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_rom_registers.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_sbrk.c \
    $(FSP_RA_DRV_DIR)/fsp/src/bsp/bsp/mcu/all/bsp_security.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_ioport/r_ioport.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_iic_master/r_iic_master.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_gpt/r_gpt.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_spi/r_spi.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_sci_uart/r_sci_uart.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_flash_lp/r_flash_lp.c \
    $(FSP_RA_DRV_DIR)/fsp/src/r_adc/r_adc.c

VARIANT_SRC += $(FSP_GEN_SRC)/common_data.c \
                $(FSP_GEN_SRC)/hal_data.c \
                $(FSP_GEN_SRC)/pin_data.c \
				$(FSP_GEN_SRC)/vector_data.c

# Basic FSP source files (only include what exists)
STDPERIPH_SRC := \
    startup.c \
    system.c

STARTUP_SRC = RA8/startup/startup_ra8e1.s

VPATH := $(VPATH):$(TARGET_PLATFORM_DIR)/startup

VCP_SRC = \
    RA8/usb_ra8.c

VCP_INCLUDES = \
    $(TARGET_PLATFORM_DIR)/include

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

INCLUDE_DIRS := $(INCLUDE_DIRS) \
    $(TARGET_PLATFORM_DIR) \
    $(TARGET_PLATFORM_DIR)/include \
    $(TARGET_PLATFORM_DIR)/startup \
    $(VCP_INCLUDES)

LD_SCRIPT = $(LINKER_DIR)/ra8e1.ld

# -mcmse                 # enable TrustZone if you need it
ARCH_FLAGS = \
    -std=c99 \
    -mthumb \
    -mcpu=cortex-m85 \
    -mfloat-abi=hard \
    -mfpu=fpv5-d16 \
    -Wno-old-style-definition \
    -Wno-error=old-style-definition

DEVICE_FLAGS += -DUSE_FSP_DRIVER -DRA8E1 -DR7FA8E1AF -DHSE_VALUE=$(HSE_VALUE) -DRA8 -DUSE_USB_CDC

MCU_COMMON_SRC = \
    RA8/system_ra8e1.c \
    RA8/adc_ra8.c \
    RA8/bus_spi_ra8.c \
    RA8/dma_ra8.c \
    RA8/flash_ra8.c \
    RA8/i2c_ra8.c \
    RA8/io_ra8.c \
    RA8/pwm_output_ra8.c \
    RA8/serial_uart_ra8.c \
    RA8/timer_ra8.c \
    RA8/usb_ra8.c \
    drivers/accgyro/accgyro_mpu.c \
    drivers/adc.c \
    drivers/bus_spi_config.c \
    drivers/serial_pinconfig.c \
    drivers/serial_uart_pinconfig.c

SPEED_OPTIMISED_SRC += \
    RA8/system_ra8e1.c

SIZE_OPTIMISED_SRC += \
    drivers/bus_spi_config.c \
    drivers/serial_pinconfig.c \
    drivers/serial_uart_pinconfig.c