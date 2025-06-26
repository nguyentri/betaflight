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

#define TARGET_BOARD_IDENTIFIER "RA8E"
#define USBD_PRODUCT_STRING     "Betaflight RA8E1"

/* RA8E1 MCU Configuration */
#define TARGET_MCU              R7FA8E1AF
#define TARGET_MCU_FAMILY       RA8E1

/* Memory Configuration */
#define FLASH_SIZE              512
#define RAM_SIZE                128
#define DATA_FLASH_SIZE         8

/* Clock Configuration */
#define HSE_VALUE               24000000  /* 24MHz external crystal */

/* LED Configuration */
#define LED0_PIN                BSP_IO_PORT_01_PIN_06  /* P106 */
#define LED1_PIN                BSP_IO_PORT_01_PIN_07  /* P107 */
#define LED2_PIN                BSP_IO_PORT_01_PIN_08  /* P108 */

#define LED0                    LED0_PIN
#define LED1                    LED1_PIN
#define LED2                    LED2_PIN

/* Button Configuration */
#define USE_BUTTONS
#define BUTTON_A_PIN            BSP_IO_PORT_01_PIN_00  /* P100 */
#define BUTTON_B_PIN            BSP_IO_PORT_01_PIN_01  /* P101 */
#define BUTTON_A_INVERTED
#define BUTTON_B_INVERTED

/* Beeper Configuration */
#define USE_BEEPER
#define BEEPER_PIN              BSP_IO_PORT_01_PIN_05  /* P105 */
#define BEEPER_INVERTED

/* SPI Configuration */
#define USE_SPI
#define USE_SPI_DEVICE_0

#define SPI0_SCK_PIN            BSP_IO_PORT_04_PIN_12  /* P412 */
#define SPI0_MISO_PIN           BSP_IO_PORT_04_PIN_11  /* P411 */
#define SPI0_MOSI_PIN           BSP_IO_PORT_04_PIN_10  /* P410 */

/* IMU Configuration (MPU9250 on SPI0) */
#define USE_GYRO
#define USE_GYRO_SPI_MPU9250
#define USE_ACC
#define USE_ACC_SPI_MPU9250

#define GYRO_1_CS_PIN           BSP_IO_PORT_06_PIN_12  /* P612 - IMU CS */
#define GYRO_1_SPI_INSTANCE     SPI0
#define GYRO_1_ALIGN            CW0_DEG

#define ACC_1_CS_PIN            BSP_IO_PORT_06_PIN_12  /* P612 - IMU CS */
#define ACC_1_SPI_INSTANCE      SPI0
#define ACC_1_ALIGN             CW0_DEG

/* Magnetometer Configuration (AK8963 on MPU9250) */
#define USE_MAG
#define USE_MAG_AK8963
#define MAG_1_ALIGN             CW0_DEG

/* Barometer Configuration (BMP280 on SPI0) */
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define BARO_CS_PIN             BSP_IO_PORT_06_PIN_14  /* P614 - Barometer CS */
#define BARO_SPI_INSTANCE       SPI0

/* UART Configuration */
#define USE_UART9
#define UART9_RX_PIN            BSP_IO_PORT_01_PIN_09  /* P109 */
#define UART9_TX_PIN            BSP_IO_PORT_01_PIN_10  /* P110 */

/* Serial RX Configuration */
#define USE_SERIALRX
#define SERIALRX_UART           SERIAL_PORT_USART9
#define SERIALRX_PROVIDER       SERIALRX_SBUS

/* PWM Output Configuration */
#define USE_PWM_OUTPUT
#define USE_DSHOT

#define MOTOR1_PIN              BSP_IO_PORT_05_PIN_04  /* P504 - GPT4A */
#define MOTOR2_PIN              BSP_IO_PORT_05_PIN_05  /* P505 - GPT4B */
#define MOTOR3_PIN              BSP_IO_PORT_05_PIN_06  /* P506 - GPT5A */
#define MOTOR4_PIN              BSP_IO_PORT_05_PIN_07  /* P507 - GPT5B */
#define MOTOR5_PIN              BSP_IO_PORT_05_PIN_08  /* P508 - GPT6A */
#define MOTOR6_PIN              BSP_IO_PORT_05_PIN_09  /* P509 - GPT6B */

/* ADC Configuration */
#define USE_ADC
#define ADC_INSTANCE            ADC0

#define VBAT_ADC_PIN            BSP_IO_PORT_00_PIN_00  /* P000/AN000 */
#define CURRENT_METER_ADC_PIN   BSP_IO_PORT_00_PIN_01  /* P001/AN001 */
#define RSSI_ADC_PIN            BSP_IO_PORT_00_PIN_02  /* P002/AN002 */
#define EXTERNAL_ADC_PIN        BSP_IO_PORT_00_PIN_03  /* P003/AN003 */

#define ADC_CHANNEL_1_PIN       VBAT_ADC_PIN
#define ADC_CHANNEL_2_PIN       CURRENT_METER_ADC_PIN
#define ADC_CHANNEL_3_PIN       RSSI_ADC_PIN
#define ADC_CHANNEL_4_PIN       EXTERNAL_ADC_PIN

#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL ADC_CHN_2
#define RSSI_ADC_CHANNEL        ADC_CHN_3

/* I2C Configuration */
#define USE_I2C
#define USE_I2C_DEVICE_0
#define I2C0_SCL_PIN            BSP_IO_PORT_04_PIN_00  /* P400 */
#define I2C0_SDA_PIN            BSP_IO_PORT_04_PIN_01  /* P401 */

/* USB Configuration */
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          BSP_IO_PORT_01_PIN_11  /* P111 - USB_DM */

/* Flash Configuration */
#define USE_FLASH
#define USE_FLASH_CHIP
#define FLASH_CS_PIN            BSP_IO_PORT_06_PIN_15  /* P615 - External flash CS if available */
#define FLASH_SPI_INSTANCE      SPI0

/* Default Configuration */
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)

/* Timer Configuration */
#define USE_TIMER
#define USE_TIMER_MGMT

/* DMA Configuration */
#define USE_DMA
#define USE_DMA_SPEC

/* Target IO Ports */
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0xffff

/* Used pins */
#define USED_TIMERS             (TIM_N(4) | TIM_N(5) | TIM_N(6))

/* Board specific features */
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT