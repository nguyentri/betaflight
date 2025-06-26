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

#ifdef USE_VCP

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/usb.h"

// FSP includes
#include "hal_data.h"

// USB CDC buffers
#define USB_CDC_BUFFER_SIZE 512
static volatile uint8_t usbRxBuffer[USB_CDC_BUFFER_SIZE];
static volatile uint8_t usbTxBuffer[USB_CDC_BUFFER_SIZE];

static volatile uint16_t usbRxBufferHead = 0;
static volatile uint16_t usbRxBufferTail = 0;
static volatile uint16_t usbTxBufferHead = 0;
static volatile uint16_t usbTxBufferTail = 0;

static bool usbInitialized = false;
static bool usbConnected = false;

// USB CDC callbacks
void usb_pcdc_callback(usb_event_info_t *p_event_info, usb_hdl_t handle, usb_onoff_t on_off)
{
    UNUSED(handle);

    switch (p_event_info->event) {
        case USB_STATUS_CONFIGURED:
            usbConnected = (on_off == USB_ON);
            break;

        case USB_STATUS_SUSPEND:
        case USB_STATUS_DETACH:
            usbConnected = false;
            break;

        default:
            break;
    }
}

void usb_pcdc_read_complete(usb_event_info_t *p_event_info, usb_hdl_t handle, usb_onoff_t on_off)
{
    UNUSED(handle);
    UNUSED(on_off);

    if (p_event_info->data_size > 0) {
        // Copy received data to buffer
        for (uint32_t i = 0; i < p_event_info->data_size; i++) {
            uint16_t nextHead = (usbRxBufferHead + 1) % USB_CDC_BUFFER_SIZE;
            if (nextHead != usbRxBufferTail) {
                usbRxBuffer[usbRxBufferHead] = ((uint8_t*)p_event_info->p_data)[i];
                usbRxBufferHead = nextHead;
            }
        }
    }
}

void usbInit(void)
{
    if (usbInitialized) {
        return;
    }

    // Initialize USB basic
    fsp_err_t err = g_basic0.p_api->open(g_basic0.p_ctrl, g_basic0.p_cfg);
    if (err != FSP_SUCCESS) {
        return;
    }

    // Initialize USB PCDC
    err = g_usb_pcdc0.p_api->open(g_usb_pcdc0.p_ctrl, g_usb_pcdc0.p_cfg);
    if (err != FSP_SUCCESS) {
        return;
    }

    usbInitialized = true;
}

bool usbIsConnected(void)
{
    return usbConnected;
}

bool usbIsConfigured(void)
{
    return usbConnected;
}

uint32_t usbTotalRxBytesWaiting(const serialPort_t *instance)
{
    UNUSED(instance);

    if (usbRxBufferHead >= usbRxBufferTail) {
        return usbRxBufferHead - usbRxBufferTail;
    } else {
        return USB_CDC_BUFFER_SIZE - usbRxBufferTail + usbRxBufferHead;
    }
}

uint32_t usbTotalTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);

    uint16_t used;
    if (usbTxBufferHead >= usbTxBufferTail) {
        used = usbTxBufferHead - usbTxBufferTail;
    } else {
        used = USB_CDC_BUFFER_SIZE - usbTxBufferTail + usbTxBufferHead;
    }

    return USB_CDC_BUFFER_SIZE - used - 1;
}

bool usbIsTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return (usbTxBufferHead == usbTxBufferTail);
}

uint8_t usbRead(serialPort_t *instance)
{
    UNUSED(instance);

    if (usbRxBufferHead == usbRxBufferTail) {
        return 0;
    }

    uint8_t data = usbRxBuffer[usbRxBufferTail];
    usbRxBufferTail = (usbRxBufferTail + 1) % USB_CDC_BUFFER_SIZE;

    return data;
}

void usbWrite(serialPort_t *instance, uint8_t ch)
{
    UNUSED(instance);

    if (!usbConnected) {
        return;
    }

    // Add character to transmit buffer
    uint16_t nextHead = (usbTxBufferHead + 1) % USB_CDC_BUFFER_SIZE;
    if (nextHead != usbTxBufferTail) {
        usbTxBuffer[usbTxBufferHead] = ch;
        usbTxBufferHead = nextHead;
    }

    // Try to send data if buffer has data
    if (usbTxBufferHead != usbTxBufferTail) {
        // Calculate available data
        uint16_t dataSize;
        if (usbTxBufferHead > usbTxBufferTail) {
            dataSize = usbTxBufferHead - usbTxBufferTail;
        } else {
            dataSize = USB_CDC_BUFFER_SIZE - usbTxBufferTail;
        }

        // Send data
        fsp_err_t err = g_usb_pcdc0.p_api->write(g_usb_pcdc0.p_ctrl,
                                                (uint8_t*)&usbTxBuffer[usbTxBufferTail],
                                                dataSize, USB_CLASS_PCDC_DATA_INTERFACE);
        if (err == FSP_SUCCESS) {
            usbTxBufferTail = (usbTxBufferTail + dataSize) % USB_CDC_BUFFER_SIZE;
        }
    }
}

void usbSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
    // Baud rate not applicable for USB CDC
}

void usbSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
    // Mode setting not applicable for USB CDC
}

void usbSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
    // Control line state callback not implemented
}

void usbSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
    // Baud rate callback not applicable for USB CDC
}

bool usbIsIdle(serialPort_t *instance)
{
    return usbIsTransmitBufferEmpty(instance);
}

// VCP serial port interface
const struct serialPortVTable usbVTable = {
    .serialWrite = usbWrite,
    .serialTotalRxWaiting = usbTotalRxBytesWaiting,
    .serialTotalTxFree = usbTotalTxBytesFree,
    .serialRead = usbRead,
    .serialSetBaudRate = usbSetBaudRate,
    .isSerialTransmitBufferEmpty = usbIsTransmitBufferEmpty,
    .setMode = usbSetMode,
    .setCtrlLineStateCb = usbSetCtrlLineStateCb,
    .setBaudRateCb = usbSetBaudRateCb,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
    .isIdle = usbIsIdle,
};

serialPort_t *usbVcpOpen(void)
{
    if (!usbInitialized) {
        usbInit();
    }

    static vcpPort_t vcpPort;
    vcpPort.port.vTable = &usbVTable;

    return (serialPort_t *)&vcpPort;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    return 115200; // Virtual baud rate
}

// USB utility functions
void usbGenerateDisconnectPulse(void)
{
    // Generate disconnect pulse by disabling/enabling USB
    if (usbInitialized) {
        g_basic0.p_api->close(g_basic0.p_ctrl);
        // Small delay
        for (volatile int i = 0; i < 100000; i++);
        g_basic0.p_api->open(g_basic0.p_ctrl, g_basic0.p_cfg);
    }
}

void usbCableDetectInit(void)
{
    // USB cable detection initialization
    // This would configure the USB detect pin if available
}

bool usbCableIsInserted(void)
{
    // Check if USB cable is inserted
    // This would read the USB detect pin if available
    return usbConnected;
}

void usbReset(void)
{
    if (usbInitialized) {
        g_usb_pcdc0.p_api->close(g_usb_pcdc0.p_ctrl);
        g_basic0.p_api->close(g_basic0.p_ctrl);

        // Reset buffers
        usbRxBufferHead = 0;
        usbRxBufferTail = 0;
        usbTxBufferHead = 0;
        usbTxBufferTail = 0;

        usbConnected = false;
        usbInitialized = false;
    }
}

// USB device state
bool usbDeviceIsConnected(void)
{
    return usbConnected;
}

bool usbDeviceIsSuspended(void)
{
    // Suspend state not tracked
    return false;
}

void usbDeviceHidSendReport(uint8_t *report, uint8_t len)
{
    UNUSED(report);
    UNUSED(len);
    // HID not implemented
}

#endif // USE_VCP