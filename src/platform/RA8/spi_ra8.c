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
#include "drivers/io.h"
#include "drivers/rcc.h"

// FSP includes
#include "hal_data.h"

// SPI device mapping
typedef struct {
    spi_instance_t *instance;
    ioTag_t sckPin;
    ioTag_t misoPin;
    ioTag_t mosiPin;
} spiHardware_t;

static const spiHardware_t spiHardware[] = {
    {
        .instance = &g_spi0,
        .sckPin = IO_TAG(SPI0_SCK_PIN),
        .misoPin = IO_TAG(SPI0_MISO_PIN),
        .mosiPin = IO_TAG(SPI0_MOSI_PIN),
    },
};

#define SPI_HARDWARE_COUNT (sizeof(spiHardware) / sizeof(spiHardware[0]))

static bool spiInitialized[SPI_HARDWARE_COUNT] = { false };

bool spiInit(SPIDevice device)
{
    if (device >= SPI_HARDWARE_COUNT) {
        return false;
    }

    if (spiInitialized[device]) {
        return true;
    }

    const spiHardware_t *hw = &spiHardware[device];
    
    // Initialize SPI instance
    fsp_err_t err = hw->instance->p_api->open(hw->instance->p_ctrl, hw->instance->p_cfg);
    if (err != FSP_SUCCESS) {
        return false;
    }

    spiInitialized[device] = true;
    return true;
}

uint32_t spiTimeoutUserCallback(void)
{
    return 0;
}

bool spiIsBusBusy(const busDevice_t *bus)
{
    if (!bus || bus->busType != BUS_TYPE_SPI) {
        return false;
    }

    SPIDevice device = bus->busdev_u.spi.device;
    if (device >= SPI_HARDWARE_COUNT) {
        return false;
    }

    // Check if SPI is busy
    // This would require checking FSP SPI status
    return false; // Simplified implementation
}

bool spiTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{
    if (!bus || bus->busType != BUS_TYPE_SPI) {
        return false;
    }

    SPIDevice device = bus->busdev_u.spi.device;
    if (device >= SPI_HARDWARE_COUNT || !spiInitialized[device]) {
        return false;
    }

    const spiHardware_t *hw = &spiHardware[device];
    
    // Perform SPI transfer
    fsp_err_t err;
    if (txData && rxData) {
        // Full duplex transfer
        err = hw->instance->p_api->writeRead(hw->instance->p_ctrl, txData, rxData, length, SPI_BIT_WIDTH_8_BITS);
    } else if (txData) {
        // Transmit only
        err = hw->instance->p_api->write(hw->instance->p_ctrl, txData, length, SPI_BIT_WIDTH_8_BITS);
    } else if (rxData) {
        // Receive only
        err = hw->instance->p_api->read(hw->instance->p_ctrl, rxData, length, SPI_BIT_WIDTH_8_BITS);
    } else {
        return false;
    }

    return (err == FSP_SUCCESS);
}

void spiSetDivisor(const busDevice_t *bus, uint16_t divisor)
{
    if (!bus || bus->busType != BUS_TYPE_SPI) {
        return;
    }

    SPIDevice device = bus->busdev_u.spi.device;
    if (device >= SPI_HARDWARE_COUNT || !spiInitialized[device]) {
        return;
    }

    // SPI divisor configuration would require reconfiguring the SPI instance
    // This is a simplified implementation
    UNUSED(divisor);
}

uint16_t spiGetErrorCounter(const busDevice_t *bus)
{
    UNUSED(bus);
    return 0; // Error counter not implemented
}

void spiResetErrorCounter(const busDevice_t *bus)
{
    UNUSED(bus);
    // Error counter reset not implemented
}

// Bus device initialization
bool spiBusInit(const busDevice_t *bus)
{
    if (!bus || bus->busType != BUS_TYPE_SPI) {
        return false;
    }

    return spiInit(bus->busdev_u.spi.device);
}

// CS pin control
void spiBusSetInstance(busDevice_t *bus, uint32_t device)
{
    if (!bus) {
        return;
    }

    bus->busType = BUS_TYPE_SPI;
    bus->busdev_u.spi.device = device;
}

// DMA support (simplified)
bool spiUseDMA(const busDevice_t *bus)
{
    UNUSED(bus);
    return false; // DMA not implemented yet
}

void spiDMAEnable(const busDevice_t *bus, bool enable)
{
    UNUSED(bus);
    UNUSED(enable);
    // DMA enable/disable not implemented
}

// SPI device configuration
void spiInitDevice(SPIDevice device)
{
    spiInit(device);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
    // Internal reset not needed for this implementation
}

void spiInternalResetDevice(busDevice_t *bus)
{
    UNUSED(bus);
    // Internal reset not needed for this implementation
}

void spiInternalStartDMA(const busDevice_t *bus)
{
    UNUSED(bus);
    // DMA start not implemented
}

void spiInternalStopDMA(const busDevice_t *bus)
{
    UNUSED(bus);
    // DMA stop not implemented
}

// Utility functions
uint8_t spiTransferByte(const busDevice_t *bus, uint8_t data)
{
    uint8_t rxData = 0;
    spiTransfer(bus, &data, &rxData, 1);
    return rxData;
}

bool spiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    uint8_t txData[2] = { reg, data };
    return spiTransfer(bus, txData, NULL, 2);
}

bool spiReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t txData = reg | 0x80; // Set read bit
    
    // First send register address
    if (!spiTransfer(bus, &txData, NULL, 1)) {
        return false;
    }
    
    // Then read data
    return spiTransfer(bus, NULL, data, length);
}

bool spiWriteRegisterBuffer(const busDevice_t *bus, uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t txBuffer[length + 1];
    txBuffer[0] = reg;
    memcpy(&txBuffer[1], data, length);
    
    return spiTransfer(bus, txBuffer, NULL, length + 1);
}

uint8_t spiReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data = 0;
    spiReadRegisterBuffer(bus, reg, &data, 1);
    return data;
}

// SPI device management
const busDevice_t *spiDeviceByInstance(uint32_t instance)
{
    static busDevice_t busDevice;
    
    if (instance >= SPI_HARDWARE_COUNT) {
        return NULL;
    }
    
    busDevice.busType = BUS_TYPE_SPI;
    busDevice.busdev_u.spi.device = instance;
    
    return &busDevice;
}

#endif // USE_SPI