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

#ifdef USE_DMA

#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/nvic.h"

// FSP includes
#include "hal_data.h"
#include "r_dmac.h"

// DMA channel control blocks
static dmac_ctrl_t g_dmac_ctrl[DMA_MAX_CHANNELS];
static bool dmaChannelUsed[DMA_MAX_CHANNELS] = {false};
static dmaCallbackHandlerFuncPtr dmaCallbacks[DMA_MAX_CHANNELS] = {NULL};

// DMA configuration template
static const dmac_cfg_t g_dmac_cfg_template = {
    .channel             = 0,
    .p_info              = NULL,
    .p_callback          = NULL,
    .p_context           = NULL,
    .activation_source   = ELC_EVENT_NONE,
    .ack_mode            = DMAC_ACK_MODE_LEVEL_MODE,
    .detection_mode      = DMAC_DETECTION_MODE_FALLING_EDGE,
    .activation_request_source_select = DMAC_REQUEST_DIRECTION_SOURCE_MODULE,
};

// DMA extended configuration template
static dmac_extended_cfg_t g_dmac_extend_template = {
    .offset              = 1,
    .src_buffer_size     = 1,
    .channel_scheduling  = DMAC_CHANNEL_SCHEDULING_FIXED,
    .p_descriptor        = NULL,
    .p_callback          = NULL,
    .p_context           = NULL,
    .activation_source   = ELC_EVENT_NONE,
};

void dmaInit(void)
{
    // Initialize all DMA channels as unused
    for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
        dmaChannelUsed[i] = false;
        dmaCallbacks[i] = NULL;
    }
}

dmaChannelDescriptor_t *dmaGetChannelByTag(dmaTag_t tag)
{
    // Convert tag to channel index
    uint8_t channel = DMA_TAG_GET_CHANNEL(tag);

    if (channel >= DMA_MAX_CHANNELS) {
        return NULL;
    }

    // Return a descriptor for the channel
    static dmaChannelDescriptor_t descriptors[DMA_MAX_CHANNELS];
    descriptors[channel].tag = tag;
    descriptors[channel].channel = channel;

    return &descriptors[channel];
}

bool dmaAllocate(dmaChannelDescriptor_t *descriptor, dmaResource_t *resource, dmaCallbackHandlerFuncPtr callback)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return false;
    }

    uint8_t channel = descriptor->channel;

    if (dmaChannelUsed[channel]) {
        return false; // Channel already in use
    }

    dmaChannelUsed[channel] = true;
    dmaCallbacks[channel] = callback;

    if (resource) {
        resource->channel = channel;
        resource->descriptor = descriptor;
    }

    return true;
}

void dmaSetHandler(dmaChannelDescriptor_t *descriptor, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    dmaCallbacks[channel] = callback;

    // Set interrupt priority
    NVIC_SetPriority(DMAC0_INT_IRQn + channel, priority);
}

void dmaEnable(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    R_DMAC_Enable(&g_dmac_ctrl[channel]);
}

void dmaDisable(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    R_DMAC_Disable(&g_dmac_ctrl[channel]);
}

bool dmaConfigureTransfer(dmaChannelDescriptor_t *descriptor, const void *src, void *dst, uint16_t len, dmaTransferDirection_t direction)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return false;
    }

    uint8_t channel = descriptor->channel;

    // Configure DMA transfer info
    dmac_info_t transfer_info = {0};

    switch (direction) {
    case DMA_PERIPH_TO_MEMORY:
        transfer_info.transfer_settings_word_b.src_addr_mode = DMAC_ADDRESS_MODE_FIXED;
        transfer_info.transfer_settings_word_b.dest_addr_mode = DMAC_ADDRESS_MODE_INCREMENTED;
        break;
    case DMA_MEMORY_TO_PERIPH:
        transfer_info.transfer_settings_word_b.src_addr_mode = DMAC_ADDRESS_MODE_INCREMENTED;
        transfer_info.transfer_settings_word_b.dest_addr_mode = DMAC_ADDRESS_MODE_FIXED;
        break;
    case DMA_MEMORY_TO_MEMORY:
        transfer_info.transfer_settings_word_b.src_addr_mode = DMAC_ADDRESS_MODE_INCREMENTED;
        transfer_info.transfer_settings_word_b.dest_addr_mode = DMAC_ADDRESS_MODE_INCREMENTED;
        break;
    default:
        return false;
    }

    transfer_info.transfer_settings_word_b.src_size = DMAC_TRANSFER_SIZE_1_BYTE;
    transfer_info.transfer_settings_word_b.dest_size = DMAC_TRANSFER_SIZE_1_BYTE;
    transfer_info.transfer_settings_word_b.mode = DMAC_MODE_NORMAL;

    transfer_info.p_src = (void*)src;
    transfer_info.p_dest = dst;
    transfer_info.length = len;

    // Configure DMA channel
    dmac_cfg_t cfg = g_dmac_cfg_template;
    cfg.channel = channel;
    cfg.p_info = &transfer_info;
    cfg.p_callback = (dmaCallbacks[channel] != NULL) ? dmac_callback : NULL;
    cfg.p_context = (void*)(uintptr_t)channel;

    fsp_err_t err = R_DMAC_Open(&g_dmac_ctrl[channel], &cfg);
    return (err == FSP_SUCCESS);
}

void dmaStartTransfer(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    R_DMAC_SoftwareStart(&g_dmac_ctrl[channel], DMAC_START_TRIGGER_SOFTWARE);
}

void dmaStopTransfer(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    R_DMAC_SoftwareStop(&g_dmac_ctrl[channel]);
}

bool dmaIsTransferComplete(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return true;
    }

    uint8_t channel = descriptor->channel;
    dmac_stat_t status;
    R_DMAC_StatusGet(&g_dmac_ctrl[channel], &status);

    return (status.transfer_end_flag == true);
}

uint16_t dmaGetRemainingCount(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return 0;
    }

    uint8_t channel = descriptor->channel;
    dmac_stat_t status;
    R_DMAC_StatusGet(&g_dmac_ctrl[channel], &status);

    return (uint16_t)status.remaining_length;
}

void dmaFree(dmaChannelDescriptor_t *descriptor)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;

    if (dmaChannelUsed[channel]) {
        R_DMAC_Close(&g_dmac_ctrl[channel]);
        dmaChannelUsed[channel] = false;
        dmaCallbacks[channel] = NULL;
    }
}

// DMA callback function
void dmac_callback(dmac_callback_args_t *p_args)
{
    if (!p_args) {
        return;
    }

    uint8_t channel = (uint8_t)(uintptr_t)p_args->p_context;

    if (channel < DMA_MAX_CHANNELS && dmaCallbacks[channel]) {
        dmaCallbacks[channel](channel, p_args->event == DMAC_EVENT_TRANSFER_END);
    }
}

// DMA interrupt handlers
void DMAC0_INT_IRQHandler(void)
{
    R_DMAC_InterruptHandler(&g_dmac_ctrl[0]);
}

void DMAC1_INT_IRQHandler(void)
{
    R_DMAC_InterruptHandler(&g_dmac_ctrl[1]);
}

void DMAC2_INT_IRQHandler(void)
{
    R_DMAC_InterruptHandler(&g_dmac_ctrl[2]);
}

void DMAC3_INT_IRQHandler(void)
{
    R_DMAC_InterruptHandler(&g_dmac_ctrl[3]);
}

// Utility functions
bool dmaIsChannelAvailable(uint8_t channel)
{
    return (channel < DMA_MAX_CHANNELS && !dmaChannelUsed[channel]);
}

void dmaSetPriority(dmaChannelDescriptor_t *descriptor, uint8_t priority)
{
    if (!descriptor || descriptor->channel >= DMA_MAX_CHANNELS) {
        return;
    }

    uint8_t channel = descriptor->channel;
    NVIC_SetPriority(DMAC0_INT_IRQn + channel, priority);
}

#endif // USE_DMA