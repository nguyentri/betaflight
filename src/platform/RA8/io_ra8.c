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
#include "drivers/io_impl.h"

// FSP includes
#include "hal_data.h"
#include "common_data.h"

// IO port definitions for RA8E1
#define IO_PORT_COUNT 16  // Maximum ports (0-15)
#define IO_PIN_COUNT  16  // Maximum pins per port (0-15)

// IO port mapping
typedef struct {
    bsp_io_port_t port;
    bsp_io_port_pin_t pin;
} ioPortPin_t;

static ioPortPin_t ioPortPinMap[IO_PORT_COUNT][IO_PIN_COUNT];
static bool ioInitialized = false;

// Convert ioTag to port and pin
static bool ioTagToPortPin(ioTag_t tag, bsp_io_port_t *port, bsp_io_port_pin_t *pin)
{
    if (tag == IO_TAG_NONE) {
        return false;
    }
    
    // Extract port and pin from tag
    uint8_t portNum = DEFIO_TAG_GPIOID(tag);
    uint8_t pinNum = DEFIO_TAG_PIN(tag);
    
    if (portNum >= IO_PORT_COUNT || pinNum >= IO_PIN_COUNT) {
        return false;
    }
    
    // Map to BSP port/pin format
    *port = (bsp_io_port_t)(BSP_IO_PORT_00_PIN_00 + (portNum << 8));
    *pin = (bsp_io_port_pin_t)(1 << pinNum);
    
    return true;
}

void IOInitGlobal(void)
{
    if (ioInitialized) {
        return;
    }
    
    // Initialize IO port mapping
    for (int port = 0; port < IO_PORT_COUNT; port++) {
        for (int pin = 0; pin < IO_PIN_COUNT; pin++) {
            ioPortPinMap[port][pin].port = (bsp_io_port_t)(BSP_IO_PORT_00_PIN_00 + (port << 8));
            ioPortPinMap[port][pin].pin = (bsp_io_port_pin_t)(1 << pin);
        }
    }
    
    ioInitialized = true;
}

IO_t IOGetByTag(ioTag_t tag)
{
    bsp_io_port_t port;
    bsp_io_port_pin_t pin;
    
    if (!ioTagToPortPin(tag, &port, &pin)) {
        return NULL;
    }
    
    // Return a pointer to the port/pin structure
    uint8_t portNum = DEFIO_TAG_GPIOID(tag);
    uint8_t pinNum = DEFIO_TAG_PIN(tag);
    
    return (IO_t)&ioPortPinMap[portNum][pinNum];
}

void IOInit(IO_t io, ioConfig_t cfg, uint8_t speed)
{
    if (!io) {
        return;
    }
    
    ioPortPin_t *portPin = (ioPortPin_t *)io;
    
    // Configure pin based on configuration
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    bsp_io_direction_t direction = BSP_IO_DIRECTION_INPUT;
    
    switch (cfg) {
        case IO_CONFIG(GPIO_Mode_IN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL):
            direction = BSP_IO_DIRECTION_INPUT;
            break;
            
        case IO_CONFIG(GPIO_Mode_IN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_UP):
            direction = BSP_IO_DIRECTION_INPUT;
            // Pull-up configuration would be set in pin configuration
            break;
            
        case IO_CONFIG(GPIO_Mode_IN, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN):
            direction = BSP_IO_DIRECTION_INPUT;
            // Pull-down configuration would be set in pin configuration
            break;
            
        case IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL):
        case IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL):
        case IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL):
            direction = BSP_IO_DIRECTION_OUTPUT;
            break;
            
        default:
            direction = BSP_IO_DIRECTION_INPUT;
            break;
    }
    
    // Configure the pin
    R_IOPORT_PinCfg(&g_ioport_ctrl, portPin->port | portPin->pin, 
                    (uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH));
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOInit(io, cfg, GPIO_Speed_2MHz);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t config, uint8_t af)
{
    UNUSED(af); // Alternate function configuration would be handled differently in RA8
    IOInit(io, config, GPIO_Speed_2MHz);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    
    ioPortPin_t *portPin = (ioPortPin_t *)io;
    R_IOPORT_PinWrite(&g_ioport_ctrl, portPin->port | portPin->pin, BSP_IO_LEVEL_HIGH);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    
    ioPortPin_t *portPin = (ioPortPin_t *)io;
    R_IOPORT_PinWrite(&g_ioport_ctrl, portPin->port | portPin->pin, BSP_IO_LEVEL_LOW);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    
    ioPortPin_t *portPin = (ioPortPin_t *)io;
    
    // Read current state and toggle
    bsp_io_level_t currentLevel;
    R_IOPORT_PinRead(&g_ioport_ctrl, portPin->port | portPin->pin, &currentLevel);
    
    bsp_io_level_t newLevel = (currentLevel == BSP_IO_LEVEL_HIGH) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH;
    R_IOPORT_PinWrite(&g_ioport_ctrl, portPin->port | portPin->pin, newLevel);
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    
    ioPortPin_t *portPin = (ioPortPin_t *)io;
    bsp_io_level_t level;
    
    R_IOPORT_PinRead(&g_ioport_ctrl, portPin->port | portPin->pin, &level);
    
    return (level == BSP_IO_LEVEL_HIGH);
}

void IOWrite(IO_t io, bool value)
{
    if (!io) {
        return;
    }
    
    if (value) {
        IOHi(io);
    } else {
        IOLo(io);
    }
}

// IO utility functions
bool IOIsInputHigh(IO_t io)
{
    return IORead(io);
}

bool IOIsInputLow(IO_t io)
{
    return !IORead(io);
}

void IOSetHigh(IO_t io)
{
    IOHi(io);
}

void IOSetLow(IO_t io)
{
    IOLo(io);
}

// IO configuration helpers
IO_t IOGetByTag_Config(ioTag_t tag, ioConfig_t cfg)
{
    IO_t io = IOGetByTag(tag);
    if (io) {
        IOConfigGPIO(io, cfg);
    }
    return io;
}

void IOConfigGPIOSpeed(IO_t io, ioConfig_t config, GPIO_Speed_TypeDef speed)
{
    IOInit(io, config, speed);
}

// Port manipulation functions
void IOPortSet(GPIO_TypeDef *port, uint16_t pins)
{
    UNUSED(port);
    UNUSED(pins);
    // Port-level operations not directly supported in this implementation
}

void IOPortClear(GPIO_TypeDef *port, uint16_t pins)
{
    UNUSED(port);
    UNUSED(pins);
    // Port-level operations not directly supported in this implementation
}

uint16_t IOPortRead(GPIO_TypeDef *port)
{
    UNUSED(port);
    // Port-level read not directly supported in this implementation
    return 0;
}

void IOPortWrite(GPIO_TypeDef *port, uint16_t value)
{
    UNUSED(port);
    UNUSED(value);
    // Port-level write not directly supported in this implementation
}

// IO resource management
bool IOIsOwned(IO_t io)
{
    UNUSED(io);
    // Resource ownership not implemented
    return false;
}

resourceOwner_e IOGetOwner(IO_t io)
{
    UNUSED(io);
    // Resource ownership not implemented
    return OWNER_FREE;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    UNUSED(io);
    // Resource management not implemented
    return true;
}

void IORelease(IO_t io)
{
    UNUSED(io);
    // Resource release not implemented
}

IO_t IOAllocate(ioTag_t tag, resourceOwner_e owner, uint8_t resourceIndex)
{
    UNUSED(owner);
    UNUSED(resourceIndex);
    return IOGetByTag(tag);
}

// IO tag utilities
ioTag_t IOGetTag(IO_t io)
{
    if (!io) {
        return IO_TAG_NONE;
    }
    
    // Convert IO back to tag
    // This is a simplified implementation
    return IO_TAG_NONE; // Would need proper reverse mapping
}

bool IOIsValid(IO_t io)
{
    return (io != NULL);
}

// LED functions
void ledInit(bool polarity)
{
    UNUSED(polarity);
    
    // Initialize LED pins
    IO_t led0 = IOGetByTag(IO_TAG(LED0_PIN));
    IO_t led1 = IOGetByTag(IO_TAG(LED1_PIN));
    IO_t led2 = IOGetByTag(IO_TAG(LED2_PIN));
    
    if (led0) {
        IOConfigGPIO(led0, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL));
        IOLo(led0);
    }
    
    if (led1) {
        IOConfigGPIO(led1, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL));
        IOLo(led1);
    }
    
    if (led2) {
        IOConfigGPIO(led2, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL));
        IOLo(led2);
    }
}

void ledSet(int lednum, bool on)
{
    IO_t led = NULL;
    
    switch (lednum) {
        case 0:
            led = IOGetByTag(IO_TAG(LED0_PIN));
            break;
        case 1:
            led = IOGetByTag(IO_TAG(LED1_PIN));
            break;
        case 2:
            led = IOGetByTag(IO_TAG(LED2_PIN));
            break;
        default:
            return;
    }
    
    if (led) {
        IOWrite(led, on);
    }
}

void ledToggle(int lednum)
{
    IO_t led = NULL;
    
    switch (lednum) {
        case 0:
            led = IOGetByTag(IO_TAG(LED0_PIN));
            break;
        case 1:
            led = IOGetByTag(IO_TAG(LED1_PIN));
            break;
        case 2:
            led = IOGetByTag(IO_TAG(LED2_PIN));
            break;
        default:
            return;
    }
    
    if (led) {
        IOToggle(led);
    }
}