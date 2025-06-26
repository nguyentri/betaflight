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

#ifdef USE_I2C

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

// FSP includes
#include "hal_data.h"

// I2C device mapping
typedef struct {
    i2c_master_instance_t *instance;
    ioTag_t sclPin;
    ioTag_t sdaPin;
    bool initialized;
    volatile bool transferComplete;
    volatile bool transferError;
} i2cHardware_t;

static i2cHardware_t i2cHardware[] = {
    {
        .instance = &g_i2c0_master,
        .sclPin = IO_TAG(I2C0_SCL_PIN),
        .sdaPin = IO_TAG(I2C0_SDA_PIN),
        .initialized = false,
        .transferComplete = false,
        .transferError = false,
    },
};

#define I2C_HARDWARE_COUNT (sizeof(i2cHardware) / sizeof(i2cHardware[0]))

// I2C callback
void i2c0_master_callback(i2c_master_callback_args_t *p_args)
{
    i2cHardware_t *i2c = &i2cHardware[0]; // I2C0 is index 0

    switch (p_args->event) {
        case I2C_MASTER_EVENT_TX_COMPLETE:
        case I2C_MASTER_EVENT_RX_COMPLETE:
            i2c->transferComplete = true;
            i2c->transferError = false;
            break;

        case I2C_MASTER_EVENT_ABORTED:
            i2c->transferComplete = true;
            i2c->transferError = true;
            break;

        default:
            break;
    }
}

bool i2cInit(I2CDevice device)
{
    if (device >= I2C_HARDWARE_COUNT) {
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];

    if (i2c->initialized) {
        return true;
    }

    // Initialize I2C master
    fsp_err_t err = i2c->instance->p_api->open(i2c->instance->p_ctrl, i2c->instance->p_cfg);
    if (err != FSP_SUCCESS) {
        return false;
    }

    i2c->initialized = true;
    return true;
}

static bool i2cWaitForTransfer(i2cHardware_t *i2c, uint32_t timeoutMs)
{
    uint32_t startTime = millis();

    while (!i2c->transferComplete) {
        if (millis() - startTime > timeoutMs) {
            return false; // Timeout
        }
        // Small delay to prevent busy waiting
        delayMicroseconds(10);
    }

    return !i2c->transferError;
}

bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data)
{
    if (device >= I2C_HARDWARE_COUNT) {
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];
    if (!i2c->initialized) {
        return false;
    }

    uint8_t txData[2] = { reg, data };

    // Reset transfer flags
    i2c->transferComplete = false;
    i2c->transferError = false;

    // Start I2C write
    fsp_err_t err = i2c->instance->p_api->write(i2c->instance->p_ctrl, txData, 2, false);
    if (err != FSP_SUCCESS) {
        return false;
    }

    // Wait for transfer completion
    return i2cWaitForTransfer(i2c, 100); // 100ms timeout
}

bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (device >= I2C_HARDWARE_COUNT) {
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];
    if (!i2c->initialized) {
        return false;
    }

    // First write register address
    i2c->transferComplete = false;
    i2c->transferError = false;

    fsp_err_t err = i2c->instance->p_api->write(i2c->instance->p_ctrl, &reg, 1, true); // Restart condition
    if (err != FSP_SUCCESS) {
        return false;
    }

    if (!i2cWaitForTransfer(i2c, 100)) {
        return false;
    }

    // Then read data
    i2c->transferComplete = false;
    i2c->transferError = false;

    err = i2c->instance->p_api->read(i2c->instance->p_ctrl, buf, len, false);
    if (err != FSP_SUCCESS) {
        return false;
    }

    return i2cWaitForTransfer(i2c, 100);
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, const uint8_t *data)
{
    if (device >= I2C_HARDWARE_COUNT) {
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];
    if (!i2c->initialized) {
        return false;
    }

    // Create buffer with register address + data
    uint8_t txBuffer[len + 1];
    txBuffer[0] = reg;
    memcpy(&txBuffer[1], data, len);

    // Reset transfer flags
    i2c->transferComplete = false;
    i2c->transferError = false;

    // Start I2C write
    fsp_err_t err = i2c->instance->p_api->write(i2c->instance->p_ctrl, txBuffer, len + 1, false);
    if (err != FSP_SUCCESS) {
        return false;
    }

    return i2cWaitForTransfer(i2c, 100);
}

bool i2cBusy(I2CDevice device, bool *error)
{
    if (device >= I2C_HARDWARE_COUNT) {
        if (error) *error = true;
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];
    if (!i2c->initialized) {
        if (error) *error = true;
        return false;
    }

    if (error) {
        *error = i2c->transferError;
    }

    return !i2c->transferComplete;
}

// Bus device functions
bool i2cBusInit(const busDevice_t *bus)
{
    if (!bus || bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    return i2cInit(bus->busdev_u.i2c.device);
}

bool i2cBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (!bus || bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    return i2cRead(bus->busdev_u.i2c.device, bus->busdev_u.i2c.address, reg, length, data);
}

bool i2cBusReadRegisterBufferStart(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    // Non-blocking read start - simplified implementation
    return i2cBusReadRegisterBuffer(bus, reg, data, length);
}

bool i2cBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    if (!bus || bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    return i2cWrite(bus->busdev_u.i2c.device, bus->busdev_u.i2c.address, reg, data);
}

bool i2cBusWriteRegisterBuffer(const busDevice_t *bus, uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (!bus || bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    return i2cWriteBuffer(bus->busdev_u.i2c.device, bus->busdev_u.i2c.address, reg, length, data);
}

uint8_t i2cBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data = 0;
    i2cBusReadRegisterBuffer(bus, reg, &data, 1);
    return data;
}

bool i2cBusIsBusy(const busDevice_t *bus, bool *error)
{
    if (!bus || bus->busType != BUS_TYPE_I2C) {
        if (error) *error = true;
        return false;
    }

    return i2cBusy(bus->busdev_u.i2c.device, error);
}

// Device detection
bool i2cDeviceDetect(I2CDevice device, uint8_t address)
{
    if (device >= I2C_HARDWARE_COUNT) {
        return false;
    }

    i2cHardware_t *i2c = &i2cHardware[device];
    if (!i2c->initialized) {
        return false;
    }

    // Try to read from device
    uint8_t dummy;
    i2c->transferComplete = false;
    i2c->transferError = false;

    fsp_err_t err = i2c->instance->p_api->read(i2c->instance->p_ctrl, &dummy, 1, false);
    if (err != FSP_SUCCESS) {
        return false;
    }

    // Wait for completion (shorter timeout for detection)
    return i2cWaitForTransfer(i2c, 10);
}

// I2C utility functions
void i2cSetOverclock(I2CDevice device, bool overclock)
{
    UNUSED(device);
    UNUSED(overclock);
    // Overclock not implemented
}

uint16_t i2cGetErrorCounter(void)
{
    // Error counter not implemented
    return 0;
}

void i2cResetErrorCounter(void)
{
    // Error counter reset not implemented
}

// Bus device creation
const busDevice_t *i2cDeviceByInstance(I2CDevice device)
{
    static busDevice_t busDevice;

    if (device >= I2C_HARDWARE_COUNT) {
        return NULL;
    }

    busDevice.busType = BUS_TYPE_I2C;
    busDevice.busdev_u.i2c.device = device;

    return &busDevice;
}

void i2cBusSetInstance(busDevice_t *bus, uint32_t device, uint8_t address)
{
    if (!bus) {
        return;
    }

    bus->busType = BUS_TYPE_I2C;
    bus->busdev_u.i2c.device = device;
    bus->busdev_u.i2c.address = address;
}

#endif // USE_I2C