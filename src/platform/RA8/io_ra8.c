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
#include "r_ioport.h"

// IO port control
static ioport_ctrl_t g_ioport_ctrl;
static const ioport_cfg_t g_ioport_cfg = {
    .number_of_pins = 0,
    .p_pin_cfg_data = NULL,
};

// Convert Betaflight IO tag to RA8 port/pin
static bool ioTagToPortPin(ioTag_t tag, bsp_io_port_t *port, bsp_io_port_pin_t *pin)
{
    if (!tag) {
        return false;
    }

    // Extract port and pin from tag
    // RA8 uses different port/pin encoding than STM32
    uint16_t portPin = DEFIO_TAG_GPIOID(tag);
    
    *port = (bsp_io_port_t)((portPin >> 8) & 0xFF);
    *pin = (bsp_io_port_pin_t)(1 << (portPin & 0xFF));
    
    return true;
}

void IOInit(void)
{
    // Initialize IOPORT
    R_IOPORT_Open(&g_ioport_ctrl, &g_ioport_cfg);
}

void IOInitGlobal(void)
{
    IOInit();
}

IO_t IOGetByTag(ioTag_t tag)
{
    bsp_io_port_t port;
    bsp_io_port_pin_t pin;
    
    if (!ioTagToPortPin(tag, &port, &pin)) {
        return NULL;
    }
    
    // Return a handle that encodes port and pin
    return (IO_t)((port << 16) | pin);
}

void IOInit(IO_t io, ioOwner_e owner, uint8_t index)
{
    if (!io) {
        return;
    }
    
    // Extract port and pin from IO handle
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    // Configure pin as GPIO output by default
    ioport_pin_cfg_t pin_cfg = {
        .pin = (bsp_io_port_pin_t)((port << 8) | pin),
        .pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH,
    };
    
    R_IOPORT_PinCfg(&g_ioport_ctrl, pin_cfg.pin, pin_cfg.pin_cfg);
    
    UNUSED(owner);
    UNUSED(index);
}

void IORelease(IO_t io)
{
    UNUSED(io);
    // IO release implementation if needed
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }
    
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    uint32_t pin_cfg = 0;
    
    // Configure direction
    if (cfg & IO_CONFIG_OUT) {
        pin_cfg |= IOPORT_CFG_PORT_DIRECTION_OUTPUT;
        
        // Configure output type
        if (cfg & IO_CONFIG_OD) {
            pin_cfg |= IOPORT_CFG_PORT_OUTPUT_OPEN_DRAIN;
        }
        
        // Set initial output state
        if (cfg & IO_CONFIG_OUT_HIGH) {
            pin_cfg |= IOPORT_CFG_PORT_OUTPUT_HIGH;
        } else {
            pin_cfg |= IOPORT_CFG_PORT_OUTPUT_LOW;
        }
    } else {
        pin_cfg |= IOPORT_CFG_PORT_DIRECTION_INPUT;
        
        // Configure pull-up/pull-down
        if (cfg & IO_CONFIG_PULLUP) {
            pin_cfg |= IOPORT_CFG_PULLUP_ENABLE;
        } else if (cfg & IO_CONFIG_PULLDOWN) {
            pin_cfg |= IOPORT_CFG_PULLDOWN_ENABLE;
        }
    }
    
    bsp_io_port_pin_t full_pin = (bsp_io_port_pin_t)((port << 8) | pin);
    R_IOPORT_PinCfg(&g_ioport_ctrl, full_pin, pin_cfg);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t)((port << 8) | pin), BSP_IO_LEVEL_HIGH);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t)((port << 8) | pin), BSP_IO_LEVEL_LOW);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    bsp_io_level_t current_level;
    R_IOPORT_PinRead(&g_ioport_ctrl, (bsp_io_port_pin_t)((port << 8) | pin), &current_level);
    
    bsp_io_level_t new_level = (current_level == BSP_IO_LEVEL_HIGH) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH;
    R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t)((port << 8) | pin), new_level);
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    
    bsp_io_port_t port = (bsp_io_port_t)((uint32_t)io >> 16);
    bsp_io_port_pin_t pin = (bsp_io_port_pin_t)((uint32_t)io & 0xFFFF);
    
    bsp_io_level_t level;
    R_IOPORT_PinRead(&g_ioport_ctrl, (bsp_io_port_pin_t)((port << 8) | pin), &level);
    
    return (level == BSP_IO_LEVEL_HIGH);
}

void IOWrite(IO_t io, bool value)
{
    if (value) {
        IOHi(io);
    } else {
        IOLo(io);
    }
}

// GPIO port access functions
void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    // RA8 doesn't use the same GPIO structure as STM32
    // This is a compatibility function
    UNUSED(gpio);
    UNUSED(config);
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    // EXTI configuration for RA8
    UNUSED(portsrc);
    UNUSED(pinsrc);
}

// IO ownership tracking
static ioOwner_e ioOwners[IO_MAX_PINS];
static uint8_t ioIndices[IO_MAX_PINS];

ioOwner_e IOGetOwner(IO_t io)
{
    if (!io) {
        return OWNER_FREE;
    }
    
    // Simple implementation - would need proper pin indexing
    return OWNER_FREE;
}

bool IOIsFreeByOwner(IO_t io, ioOwner_e owner)
{
    return (IOGetOwner(io) == OWNER_FREE) || (IOGetOwner(io) == owner);
}