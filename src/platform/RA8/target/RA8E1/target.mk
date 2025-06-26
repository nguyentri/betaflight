TARGET_MCU        := RA8E1
MCU_FLASH_SIZE    := 512
DEVICE_FLAGS       = -D$(TARGET_MCU) -DR7FA8E1AF -DRA8E1
TARGET_MCU_FAMILY := RA8E1

RA8E1_TARGETS += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu9250.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_ist8310.c \
            drivers/compass/compass_ist8308.c \
            drivers/compass/compass_mag3110.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c \
            drivers/rx/rx_cc2500.c \
            rx/cc2500_common.c \
            rx/cc2500_frsky_shared.c \
            rx/cc2500_frsky_d.c \
            rx/cc2500_frsky_x.c \
            rx/cc2500_sfhss.c \
            drivers/rx/rx_a7105.c \
            rx/flysky.c \
            rx/flysky_a7105.c \
            drivers/rx/rx_cyrf6936.c \
            rx/spektrum.c \
            rx/cyrf6936_spektrum.c \
            drivers/rx/rx_nrf24l01.c \
            rx/nrf24_cx10.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_inav.c \
            rx/nrf24_kn.c