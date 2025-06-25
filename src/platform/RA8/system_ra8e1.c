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

#include "drivers/system.h"
#include "drivers/time.h"

// FSP includes
#include "hal_data.h"
#include "bsp_api.h"
#include "r_cgc.h"

// System clock frequency
uint32_t SystemCoreClock = SYSTEM_CLOCK_FREQ;

void systemClockInit(void)
{
    // Clock initialization is handled by FSP BSP
    // The FSP configuration tool generates the clock setup
    
    // Update SystemCoreClock variable
    SystemCoreClock = SYSTEM_CLOCK_FREQ;
    
    // Initialize system tick for timing
    SysTick_Config(SystemCoreClock / 1000); // 1ms tick
}

void systemInit(void)
{
    // Initialize clocks
    systemClockInit();
    
    // Initialize microsecond timer
    microsInit();
    
    // Board-specific initialization
    targetBoardInit();
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
    // RA8E1 specific bootloader entry
    systemReset();
}

uint32_t systemGetResetCause(void)
{
    // Read RA8E1 reset status registers
    // This needs to be implemented based on RA8E1 reset status registers
    return 0;
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // RA8E1 specific power optimizations
}

bool isMPUSoftReset(void)
{
    return false;
}