/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "RA8E1"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight RA8E1"
#endif

// RA8E1 FPB Board Configuration
#define TARGET_BOARD_FPB_RA8E1

// MCU Configuration
#define MCU_TYPE_RA8E1
#define BSP_MCU_GROUP_RA8E1     1
#define BSP_CFG_MCU_PART_SERIES 8

// Clock Configuration
#define SYSTEM_CLOCK_FREQ       200000000  // 200MHz
#define PERIPHERAL_CLOCK_FREQ   100000000  // 100MHz

// Memory Configuration
#define FLASH_SIZE              512        // 512KB Flash
#define RAM_SIZE                128        // 128KB RAM
#define FLASH_PAGE_SIZE         ((uint32_t)0x4000) // 16K sectors

// GPIO Port Configuration
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

// SPI Configuration for GY-91 (MPU9255 + BMP280)
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI_FULL_RECONFIGURABILITY

// SPI0 Pin Configuration (Pmod 1)
#define SPI1_SCK_PIN            BSP_IO_PORT_06_PIN_11  // P611
#define SPI1_MISO_PIN           BSP_IO_PORT_06_PIN_10  // P610
#define SPI1_MOSI_PIN           BSP_IO_PORT_06_PIN_09  // P609

// Chip Select Pins for GY-91
#define MPU9255_CS_PIN          BSP_IO_PORT_06_PIN_12  // P612 (SS1)
#define BMP280_CS_PIN           BSP_IO_PORT_06_PIN_14  // P614 (SS2)

// IMU Configuration (MPU9255)
#define USE_GYRO
#define USE_GYRO_MPU9250
#define USE_ACC
#define USE_ACC_MPU9250
#define USE_MAG
#define USE_MAG_MPU9250

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           MPU9255_CS_PIN
#define GYRO_1_EXTI_PIN         BSP_IO_PORT_00_PIN_06  // P006 (IRQ11) - Optional

// Barometer Configuration (BMP280)
#define USE_BARO
#define USE_BARO_BMP280

#define BARO_SPI_INSTANCE       SPI1
#define BARO_CS_PIN             BMP280_CS_PIN

// UART Configuration for Receiver
#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4

// Primary receiver UART (J18-1 RXD3 or Pmod2 P802 RXD2)
#define SERIALRX_UART           SERIAL_PORT_USART3
#define SERIALRX_PROVIDER       SERIALRX_SBUS

// UART3 Pin Configuration (J18-1)
#define UART3_RX_PIN            BSP_IO_PORT_03_PIN_02  // P302 (RXD3)
#define UART3_TX_PIN            BSP_IO_PORT_03_PIN_01  // P301 (TXD3)

// Alternative UART2 Pin Configuration (Pmod2)
#define UART2_RX_PIN            BSP_IO_PORT_08_PIN_02  // P802 (RXD2)
#define UART2_TX_PIN            BSP_IO_PORT_08_PIN_01  // P801 (TXD2)

// PWM Configuration for ESC Outputs
#define USE_PWM_OUTPUT
#define USE_DSHOT
#define USE_ESC_SENSOR

// PWM Output Pins (Arduino-style pins)
#define PWM1_PIN                BSP_IO_PORT_04_PIN_15  // P415 (D3)
#define PWM2_PIN                BSP_IO_PORT_01_PIN_14  // P114 (D5)
#define PWM3_PIN                BSP_IO_PORT_01_PIN_13  // P113 (D6)
#define PWM4_PIN                BSP_IO_PORT_01_PIN_12  // P112 (D9)

// Timer Configuration for PWM
#define USE_TIMER
#define USE_TIMER_MGMT

// GPT Timer assignments for PWM
#define PWM1_TIMER              GPT0
#define PWM2_TIMER              GPT1
#define PWM3_TIMER              GPT2
#define PWM4_TIMER              GPT3

// I2C Configuration (if needed for future expansion)
#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define I2C_FULL_RECONFIGURABILITY

// ADC Configuration
#define USE_ADC
#define ADC_INSTANCE            ADC0
#define ADC_CHANNEL_COUNT       4

// Battery monitoring (if available)
#define VBAT_ADC_PIN            BSP_IO_PORT_00_PIN_00  // P000 (AN000)
#define CURRENT_METER_ADC_PIN   BSP_IO_PORT_00_PIN_01  // P001 (AN001)

// External Interrupt Configuration
#define USE_EXTI

// Beeper Configuration (if available)
#define USE_BEEPER
#define BEEPER_PIN              BSP_IO_PORT_01_PIN_06  // P106

// LED Configuration
#define USE_LED_STRIP
#define LED_STRIP_PIN           BSP_IO_PORT_01_PIN_05  // P105

// USB Configuration
#define USE_USB_DETECT
#define USB_DETECT_PIN          BSP_IO_PORT_01_PIN_08  // P108

// Flash Configuration for Settings Storage
#define USE_FLASH_CHIP
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            BSP_IO_PORT_01_PIN_03  // P103
#define FLASH_SPI_INSTANCE      SPI1

// On-chip Flash for Configuration Storage
#define USE_FLASH_W25N01G       // Use on-chip flash instead
#define CONFIG_IN_FLASH
#define EEPROM_SIZE             8192

// Board-specific features
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

// Disable features not available on this target
#undef USE_SDCARD
#undef USE_CAMERA_CONTROL

// Target-specific optimizations
#define FAST_CODE_PREF          // RA8E1 has sufficient memory for fast code

// Debug Configuration
#ifdef DEBUG
#define USE_DEBUG_PIN
#define DEBUG_PIN               BSP_IO_PORT_01_PIN_07  // P107
#endif

// Board initialization function
void targetBoardInit(void);
void targetBoardInitComplete(void);