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

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/uart.h"
#include "drivers/usb.h"

// FSP includes
#include "hal_data.h"
#include "common_data.h"

void systemInit(void)
{
    // Initialize FSP
    fsp_err_t err = R_BSP_WarmStart(BSP_WARM_START_RESET);
    if (err != FSP_SUCCESS) {
        // Handle error
        while(1);
    }

    // Initialize common data (IO ports)
    err = g_ioport.p_api->open(g_ioport.p_ctrl, g_ioport.p_cfg);
    if (err != FSP_SUCCESS) {
        // Handle error
        while(1);
    }

    // Configure system clocks
    systemClockInit();

    // Initialize NVIC
    nvicInit();

    // Initialize IO system
    IOInitGlobal();

    // Initialize LEDs
    ledInit(false);

    // Initialize timers
    timerInit();

    // Initialize USB if enabled
#ifdef USE_VCP
    usbInit();
#endif
}

void systemClockInit(void)
{
    // Clock initialization is handled by FSP startup code
    // Additional clock configuration can be added here if needed
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    
    // Set magic value in RAM to indicate bootloader request
    *((uint32_t*)0x20000000) = 0xDEADBEEF;
    
    systemReset();
}

bool isMPUSoftReset(void)
{
    // Check if this is a soft reset
    return false; // Simplified implementation
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // RA8E1 specific power optimizations can be added here
}

bool isMotorProtocolDshot(void)
{
    // This will be implemented by motor driver
    return false;
}

void systemCheckResetReason(void)
{
    // Check reset reason and handle accordingly
    // This can be implemented using RA8E1 reset status registers
}

uint32_t systemBootloaderAddress(void)
{
    // Return bootloader address if available
    return 0x00000000; // Default to start of flash
}

void systemProcessResetReason(void)
{
    // Process reset reason
    systemCheckResetReason();
}