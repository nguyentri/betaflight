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

#ifdef USE_UART

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

// FSP includes
#include "hal_data.h"
#include "r_sci_uart.h"

// UART instance control blocks
static sci_uart_ctrl_t g_uart1_ctrl;
static sci_uart_ctrl_t g_uart2_ctrl;
static sci_uart_ctrl_t g_uart3_ctrl;

// UART configurations
static const sci_uart_cfg_t g_uart1_cfg = {
    .channel             = 1,
    .data_bits           = UART_DATA_BITS_8,
    .parity              = UART_PARITY_OFF,
    .stop_bits           = UART_STOP_BITS_1,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
    .p_transfer_tx       = NULL,
    .p_transfer_rx       = NULL,
    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),
};

static const sci_uart_cfg_t g_uart2_cfg = {
    .channel             = 2,
    .data_bits           = UART_DATA_BITS_8,
    .parity              = UART_PARITY_OFF,
    .stop_bits           = UART_STOP_BITS_1,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
    .p_transfer_tx       = NULL,
    .p_transfer_rx       = NULL,
    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),
};

static const sci_uart_cfg_t g_uart3_cfg = {
    .channel             = 3,
    .data_bits           = UART_DATA_BITS_8,
    .parity              = UART_PARITY_OFF,
    .stop_bits           = UART_STOP_BITS_1,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
    .p_transfer_tx       = NULL,
    .p_transfer_rx       = NULL,
    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),
};

// UART port structures
static uartPort_t uartPort1;
static uartPort_t uartPort2;
static uartPort_t uartPort3;

// UART hardware definitions
const uartHardware_t uartHardware[UARTDEV_COUNT] = {
    {
        .device = UARTDEV_1,
        .reg = (void*)&g_uart1_ctrl,
        .rxPins = { { DEFIO_TAG_E(UART1_RX_PIN), 0 } },
        .txPins = { { DEFIO_TAG_E(UART1_TX_PIN), 0 } },
        .rcc = 0,
        .rxIrq = 0,
        .txIrq = 0,
        .rxPriority = NVIC_PRIO_SERIALUART1_RXDMA,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .txBuffer = NULL,
        .rxBuffer = NULL,
        .txBufferSize = 0,
        .rxBufferSize = 0,
    },
    {
        .device = UARTDEV_2,
        .reg = (void*)&g_uart2_ctrl,
        .rxPins = { { DEFIO_TAG_E(UART2_RX_PIN), 0 } },
        .txPins = { { DEFIO_TAG_E(UART2_TX_PIN), 0 } },
        .rcc = 0,
        .rxIrq = 0,
        .txIrq = 0,
        .rxPriority = NVIC_PRIO_SERIALUART2_RXDMA,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .txBuffer = NULL,
        .rxBuffer = NULL,
        .txBufferSize = 0,
        .rxBufferSize = 0,
    },
    {
        .device = UARTDEV_3,
        .reg = (void*)&g_uart3_ctrl,
        .rxPins = { { DEFIO_TAG_E(UART3_RX_PIN), 0 } },
        .txPins = { { DEFIO_TAG_E(UART3_TX_PIN), 0 } },
        .rcc = 0,
        .rxIrq = 0,
        .txIrq = 0,
        .rxPriority = NVIC_PRIO_SERIALUART3_RXDMA,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .txBuffer = NULL,
        .rxBuffer = NULL,
        .txBufferSize = 0,
        .rxBufferSize = 0,
    },
};

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    // Pin configuration is handled by FSP configuration
    UNUSED(pSerialPinConfig);
}

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *s = NULL;
    
    switch (device) {
    case UARTDEV_1:
        s = &uartPort1;
        s->USARTx = (void*)&g_uart1_ctrl;
        R_SCI_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);
        break;
    case UARTDEV_2:
        s = &uartPort2;
        s->USARTx = (void*)&g_uart2_ctrl;
        R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
        break;
    case UARTDEV_3:
        s = &uartPort3;
        s->USARTx = (void*)&g_uart3_ctrl;
        R_SCI_UART_Open(&g_uart3_ctrl, &g_uart3_cfg);
        break;
    default:
        return NULL;
    }

    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.options = options;

    // Configure baud rate
    uart_baud_setting_t baud_setting;
    R_SCI_UART_BaudCalculate(baudRate, true, 5000, &baud_setting);
    R_SCI_UART_BaudSet(s->USARTx, &baud_setting);

    return (serialPort_t *)s;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    sci_uart_ctrl_t *uart = (sci_uart_ctrl_t *)s->USARTx;
    
    R_SCI_UART_Write(uart, &ch, 1);
}

uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    // Simplified implementation
    UNUSED(instance);
    return 0;
}

uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    // Simplified implementation
    UNUSED(instance);
    return 256; // Assume buffer space available
}

bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    // Simplified implementation
    UNUSED(instance);
    return true;
}

uint8_t uartRead(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    sci_uart_ctrl_t *uart = (sci_uart_ctrl_t *)s->USARTx;
    
    uint8_t data = 0;
    size_t bytes_read = 0;
    
    R_SCI_UART_Read(uart, &data, 1);
    
    return data;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *s = (uartPort_t *)instance;
    sci_uart_ctrl_t *uart = (sci_uart_ctrl_t *)s->USARTx;
    
    uart_baud_setting_t baud_setting;
    R_SCI_UART_BaudCalculate(baudRate, true, 5000, &baud_setting);
    R_SCI_UART_BaudSet(uart, &baud_setting);
    
    s->port.baudRate = baudRate;
}

void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.mode = mode;
}

void uartStartTxDMA(uartPort_t *s)
{
    // DMA implementation would go here
    UNUSED(s);
}

uint32_t uartGetBaudRate(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    return s->port.baudRate;
}

#endif // USE_UART