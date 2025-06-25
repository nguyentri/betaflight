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
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"

// FSP includes
#include "hal_data.h"
#include "r_spi.h"

// SPI instance control blocks
static spi_ctrl_t g_spi0_ctrl;
static const spi_cfg_t g_spi0_cfg = {
    .channel             = 0,
    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),
    .operating_mode      = SPI_MODE_MASTER,
    .clk_phase           = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity        = SPI_CLK_POLARITY_LOW,
    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
    .p_transfer_tx       = NULL,
    .p_transfer_rx       = NULL,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
};

static bool spiInitialized[SPIDEV_COUNT] = { false };

static void spiInitDevice(SPIDevice device)
{
    if (spiInitialized[device]) {
        return;
    }

    switch (device) {
    case SPIDEV_1:
        // Initialize SPI0 for GY-91 sensors
        R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg);
        break;
    default:
        return;
    }

    spiInitialized[device] = true;
}

void spiInitBusByDevice(const busDevice_t *dev)
{
    if (dev->busType != BUS_TYPE_SPI) {
        return;
    }

    SPIDevice device = dev->busdev_u.spi.device;
    spiInitDevice(device);
}

bool spiIsBusBusy(const busDevice_t *dev)
{
    if (dev->busType != BUS_TYPE_SPI) {
        return false;
    }

    SPIDevice device = dev->busdev_u.spi.device;
    
    switch (device) {
    case SPIDEV_1:
        // Check if SPI0 is busy
        return false; // Simplified for now
    default:
        return false;
    }
}

bool spiTransfer(const busDevice_t *dev, uint8_t *rxData, const uint8_t *txData, int length)
{
    if (dev->busType != BUS_TYPE_SPI) {
        return false;
    }

    SPIDevice device = dev->busdev_u.spi.device;
    
    if (!spiInitialized[device]) {
        spiInitDevice(device);
    }

    // Assert CS
    if (dev->busdev_u.spi.csnPin) {
        IOLo(IOGetByTag(dev->busdev_u.spi.csnPin));
    }

    bool result = false;
    
    switch (device) {
    case SPIDEV_1:
        {
            // Perform SPI transfer using FSP SPI driver
            fsp_err_t err;
            
            if (txData && rxData) {
                // Full duplex transfer
                err = R_SPI_WriteRead(&g_spi0_ctrl, txData, rxData, length, SPI_BIT_WIDTH_8_BITS);
            } else if (txData) {
                // Transmit only
                err = R_SPI_Write(&g_spi0_ctrl, txData, length, SPI_BIT_WIDTH_8_BITS);
            } else if (rxData) {
                // Receive only
                err = R_SPI_Read(&g_spi0_ctrl, rxData, length, SPI_BIT_WIDTH_8_BITS);
            } else {
                err = FSP_ERR_INVALID_ARGUMENT;
            }
            
            result = (err == FSP_SUCCESS);
        }
        break;
    default:
        break;
    }

    // Wait for transfer completion (simplified)
    // In a real implementation, this should use interrupts or polling
    delayMicroseconds(length * 10); // Rough estimate

    // Deassert CS
    if (dev->busdev_u.spi.csnPin) {
        IOHi(IOGetByTag(dev->busdev_u.spi.csnPin));
    }

    return result;
}

uint16_t spiGetErrorCounter(const busDevice_t *dev)
{
    UNUSED(dev);
    return 0; // Error counting not implemented yet
}

void spiResetErrorCounter(const busDevice_t *dev)
{
    UNUSED(dev);
    // Error counter reset not implemented yet
}

// SPI device configuration
bool spiSetBusInstance(busDevice_t *dev, uint32_t device)
{
    if (device >= SPIDEV_COUNT) {
        return false;
    }

    dev->busType = BUS_TYPE_SPI;
    dev->busdev_u.spi.device = device;
    
    return true;
}

// Initialize all SPI devices
void spiPreInit(void)
{
    // Pre-initialization if needed
}

void spiInit(SPIDevice device)
{
    spiInitDevice(device);
}

// SPI pin configuration
void spiPinConfigure(const spiPinConfig_t *pConfig)
{
    // Pin configuration is handled by FSP configuration
    UNUSED(pConfig);
}

#endif // USE_SPI