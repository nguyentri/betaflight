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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/bus_spi.h"
#include "drivers/timer.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/adc.h"

#include "target.h"

// FSP includes
#include "hal_data.h"
#include "bsp_api.h"
#include "r_ioport.h"
#include "r_gpt.h"
#include "r_spi.h"
#include "r_sci_uart.h"
#include "r_adc.h"

// Timer definitions for PWM outputs
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // PWM outputs for ESCs
    DEF_TIM(GPT0, CH1, PWM1_PIN, TIM_USE_MOTOR,               0, 0), // D3 (P415)
    DEF_TIM(GPT1, CH1, PWM2_PIN, TIM_USE_MOTOR,               0, 0), // D5 (P114)
    DEF_TIM(GPT2, CH1, PWM3_PIN, TIM_USE_MOTOR,               0, 0), // D6 (P113)
    DEF_TIM(GPT3, CH1, PWM4_PIN, TIM_USE_MOTOR,               0, 0), // D9 (P112)
    
    // Additional timers for LED strip, beeper, etc.
    DEF_TIM(GPT4, CH1, LED_STRIP_PIN, TIM_USE_LED,            0, 0), // P105
    DEF_TIM(GPT5, CH1, BEEPER_PIN, TIM_USE_BEEPER,            0, 0), // P106
};

// SPI device definitions
const spiHardware_t spiHardware[SPIDEV_COUNT] = {
    {
        .device = SPIDEV_1,
        .reg = SPI0,
        .sckPin = IO_TAG(SPI1_SCK_PIN),
        .misoPin = IO_TAG(SPI1_MISO_PIN),
        .mosiPin = IO_TAG(SPI1_MOSI_PIN),
        .rcc = 0, // RA8 doesn't use RCC like STM32
        .af = 0,
        .dmaIrqHandler = NULL,
        .txDMAChannel = 0,
        .rxDMAChannel = 0,
        .txDMAPeripheralBaseAddr = 0,
        .rxDMAPeripheralBaseAddr = 0,
    }
};

// UART device definitions
const serialPortConfig_t serialPortConfig[SERIAL_PORT_COUNT] = {
    // VCP
    { SERIAL_PORT_USB_VCP, FUNCTION_MSP, MSP_BAUDRATE, MODE_RXTX, PORTSHARING_UNUSED, 0 },
    
    // UART3 for receiver (J18-1)
    { SERIAL_PORT_USART3, FUNCTION_RX_SERIAL, 100000, MODE_RX, PORTSHARING_UNUSED, 0 },
    
    // UART2 alternative for receiver (Pmod2)
    { SERIAL_PORT_USART2, FUNCTION_NONE, 115200, MODE_RXTX, PORTSHARING_UNUSED, 0 },
    
    // UART1 for telemetry/MSP
    { SERIAL_PORT_USART1, FUNCTION_MSP, 115200, MODE_RXTX, PORTSHARING_UNUSED, 0 },
};

// ADC channel definitions
const adcTagMap_t adcTagMap[ADC_CHANNEL_COUNT] = {
    { DEFIO_TAG_E(VBAT_ADC_PIN),        ADC_BATTERY     },
    { DEFIO_TAG_E(CURRENT_METER_ADC_PIN), ADC_CURRENT   },
    { DEFIO_TAG_E(RSSI_ADC_PIN),        ADC_RSSI        },
    { DEFIO_TAG_E(EXTERNAL1_ADC_PIN),   ADC_EXTERNAL1   },
};

// Board-specific initialization
void targetBoardInit(void)
{
    // Initialize FSP
    R_BSP_WarmStart(BSP_WARM_START_RESET);
    
    // Initialize I/O ports
    R_IOPORT_Open(&g_ioport_ctrl, &g_ioport_cfg);
    
    // Configure SPI pins for GY-91
    // SPI0 configuration is handled by FSP configuration
    
    // Configure chip select pins as GPIO outputs
    ioInit(IOGetByTag(IO_TAG(MPU9255_CS_PIN)), OWNER_SPI_CS, 0);
    ioInit(IOGetByTag(IO_TAG(BMP280_CS_PIN)), OWNER_SPI_CS, 0);
    
    // Set CS pins high (inactive)
    IOHi(IOGetByTag(IO_TAG(MPU9255_CS_PIN)));
    IOHi(IOGetByTag(IO_TAG(BMP280_CS_PIN)));
    
    // Configure PWM output pins
    ioInit(IOGetByTag(IO_TAG(PWM1_PIN)), OWNER_MOTOR, 1);
    ioInit(IOGetByTag(IO_TAG(PWM2_PIN)), OWNER_MOTOR, 2);
    ioInit(IOGetByTag(IO_TAG(PWM3_PIN)), OWNER_MOTOR, 3);
    ioInit(IOGetByTag(IO_TAG(PWM4_PIN)), OWNER_MOTOR, 4);
    
    // Configure UART pins
    ioInit(IOGetByTag(IO_TAG(UART3_RX_PIN)), OWNER_SERIAL_RX, 0);
    ioInit(IOGetByTag(IO_TAG(UART3_TX_PIN)), OWNER_SERIAL_TX, 0);
    
    // Configure LED and beeper pins
    ioInit(IOGetByTag(IO_TAG(LED_STRIP_PIN)), OWNER_LED_STRIP, 0);
    ioInit(IOGetByTag(IO_TAG(BEEPER_PIN)), OWNER_BEEPER, 0);
    
    // Configure IMU interrupt pin (optional)
    #ifdef GYRO_1_EXTI_PIN
    ioInit(IOGetByTag(IO_TAG(GYRO_1_EXTI_PIN)), OWNER_MPU_EXTI, 0);
    #endif
    
    // Configure USB detect pin
    #ifdef USB_DETECT_PIN
    ioInit(IOGetByTag(IO_TAG(USB_DETECT_PIN)), OWNER_USB_DETECT, 0);
    #endif
}

void targetBoardInitComplete(void)
{
    // Additional initialization after all systems are up
    
    // Enable SPI peripheral
    // This will be handled by the SPI driver initialization
    
    // Enable UART peripherals
    // This will be handled by the UART driver initialization
    
    // Enable PWM timers
    // This will be handled by the PWM driver initialization
    
    // Board-specific post-initialization
    // Add any board-specific setup that needs to happen after
    // all core systems are initialized
}

// Board-specific system reset function
void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

// Board-specific system reset to bootloader function
void systemResetToBootloader(void)
{
    // RA8E1 specific bootloader entry
    // This may need to be implemented based on RA8E1 bootloader requirements
    systemReset();
}

// Board-specific memory functions
void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // RA8E1 specific power optimizations
    // Disable unused peripherals to reduce power consumption
}

// Board-specific clock configuration
bool isMPUSoftReset(void)
{
    // Check if this is a soft reset from MPU
    // Implementation depends on RA8E1 reset status registers
    return false;
}