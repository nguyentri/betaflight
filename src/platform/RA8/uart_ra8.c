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

#ifdef USE_UART

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

// FSP includes
#include "hal_data.h"

// UART hardware mapping
typedef struct {
    uart_instance_t *instance;
    ioTag_t rxPin;
    ioTag_t txPin;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint16_t rxBufferSize;
    uint16_t txBufferSize;
    uint16_t rxBufferHead;
    uint16_t rxBufferTail;
    uint16_t txBufferHead;
    uint16_t txBufferTail;
    bool initialized;
} uartHardware_t;

// UART buffers
#define UART_BUFFER_SIZE 256
static volatile uint8_t uart9RxBuffer[UART_BUFFER_SIZE];
static volatile uint8_t uart9TxBuffer[UART_BUFFER_SIZE];

static uartHardware_t uartHardware[] = {
    {
        .instance = &g_uart9,
        .rxPin = IO_TAG(UART9_RX_PIN),
        .txPin = IO_TAG(UART9_TX_PIN),
        .rxBuffer = uart9RxBuffer,
        .txBuffer = uart9TxBuffer,
        .rxBufferSize = UART_BUFFER_SIZE,
        .txBufferSize = UART_BUFFER_SIZE,
        .rxBufferHead = 0,
        .rxBufferTail = 0,
        .txBufferHead = 0,
        .txBufferTail = 0,
        .initialized = false,
    },
};

#define UART_HARDWARE_COUNT (sizeof(uartHardware) / sizeof(uartHardware[0]))

// UART callbacks
void uart9_callback(uart_callback_args_t *p_args)
{
    uartHardware_t *uart = &uartHardware[0]; // UART9 is index 0
    
    switch (p_args->event) {
        case UART_EVENT_RX_CHAR:
            // Store received character in buffer
            uart->rxBuffer[uart->rxBufferHead] = (uint8_t)p_args->data;
            uart->rxBufferHead = (uart->rxBufferHead + 1) % uart->rxBufferSize;
            break;
            
        case UART_EVENT_TX_COMPLETE:
            // Transmission complete
            break;
            
        default:
            break;
    }
}

static uartHardware_t *uartGetHardware(UARTDevice_e device)
{
    switch (device) {
        case UARTDEV_9:
            return &uartHardware[0];
        default:
            return NULL;
    }
}

void uartInit(void)
{
    // Initialize all UART devices
    for (int i = 0; i < UART_HARDWARE_COUNT; i++) {
        uartHardware_t *uart = &uartHardware[i];
        
        if (!uart->initialized) {
            fsp_err_t err = uart->instance->p_api->open(uart->instance->p_ctrl, uart->instance->p_cfg);
            if (err == FSP_SUCCESS) {
                uart->initialized = true;
            }
        }
    }
}

void uartStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
    // DMA transmission not implemented yet
}

uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart) {
        return 0;
    }
    
    if (uart->rxBufferHead >= uart->rxBufferTail) {
        return uart->rxBufferHead - uart->rxBufferTail;
    } else {
        return uart->rxBufferSize - uart->rxBufferTail + uart->rxBufferHead;
    }
}

uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart) {
        return 0;
    }
    
    uint16_t used;
    if (uart->txBufferHead >= uart->txBufferTail) {
        used = uart->txBufferHead - uart->txBufferTail;
    } else {
        used = uart->txBufferSize - uart->txBufferTail + uart->txBufferHead;
    }
    
    return uart->txBufferSize - used - 1; // -1 to prevent buffer full condition
}

bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart) {
        return true;
    }
    
    return (uart->txBufferHead == uart->txBufferTail);
}

uint8_t uartRead(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart || uart->rxBufferHead == uart->rxBufferTail) {
        return 0;
    }
    
    uint8_t data = uart->rxBuffer[uart->rxBufferTail];
    uart->rxBufferTail = (uart->rxBufferTail + 1) % uart->rxBufferSize;
    
    return data;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart) {
        return;
    }
    
    // Add character to transmit buffer
    uint16_t nextHead = (uart->txBufferHead + 1) % uart->txBufferSize;
    if (nextHead != uart->txBufferTail) {
        uart->txBuffer[uart->txBufferHead] = ch;
        uart->txBufferHead = nextHead;
        
        // Start transmission if not already transmitting
        if (uart->txBufferHead != uart->txBufferTail) {
            uint8_t data = uart->txBuffer[uart->txBufferTail];
            uart->txBufferTail = (uart->txBufferTail + 1) % uart->txBufferSize;
            uart->instance->p_api->write(uart->instance->p_ctrl, &data, 1);
        }
    }
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *s = (uartPort_t *)instance;
    uartHardware_t *uart = uartGetHardware(s->USARTx);
    
    if (!uart) {
        return;
    }
    
    // Reconfigure UART with new baud rate
    uart_cfg_t cfg = *(uart->instance->p_cfg);
    cfg.baud_rate = baudRate;
    
    uart->instance->p_api->close(uart->instance->p_ctrl);
    uart->instance->p_api->open(uart->instance->p_ctrl, &cfg);
}

void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
    // Mode setting not implemented
}

void uartSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
    // Control line state callback not implemented
}

void uartSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
    // Baud rate callback not implemented
}

bool isUartIdle(serialPort_t *instance)
{
    return isUartTransmitBufferEmpty(instance);
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = uartSetCtrlLineStateCb,
        .setBaudRateCb = uartSetBaudRateCb,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
        .isIdle = isUartIdle,
    }
};

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    UNUSED(rxCallback);
    UNUSED(rxCallbackData);
    UNUSED(mode);
    UNUSED(options);
    
    uartHardware_t *uart = uartGetHardware(device);
    if (!uart || !uart->initialized) {
        return NULL;
    }
    
    static uartPort_t uartPort;
    uartPort.port.vTable = uartVTable;
    uartPort.USARTx = device;
    
    // Set baud rate
    uartSetBaudRate((serialPort_t *)&uartPort, baudRate);
    
    return (serialPort_t *)&uartPort;
}

void uartGetPortPins(UARTDevice_e device, ioTag_t *rxPin, ioTag_t *txPin)
{
    uartHardware_t *uart = uartGetHardware(device);
    if (!uart) {
        *rxPin = IO_TAG_NONE;
        *txPin = IO_TAG_NONE;
        return;
    }
    
    *rxPin = uart->rxPin;
    *txPin = uart->txPin;
}

bool isUartAvailable(UARTDevice_e device)
{
    uartHardware_t *uart = uartGetHardware(device);
    return (uart != NULL);
}

void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
    // DMA transmission not implemented
}

uint32_t uartGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    return 115200; // Default baud rate
}

#endif // USE_UART