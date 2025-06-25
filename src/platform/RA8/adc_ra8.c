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

#ifdef USE_ADC

#include "drivers/adc.h"
#include "drivers/adc_impl.h"
#include "drivers/io.h"

// FSP includes
#include "hal_data.h"
#include "r_adc.h"

// ADC instance control block
static adc_ctrl_t g_adc0_ctrl;
static const adc_cfg_t g_adc0_cfg = {
    .unit                = 0,
    .mode                = ADC_MODE_SINGLE_SCAN,
    .resolution          = ADC_RESOLUTION_12_BIT,
    .alignment           = ADC_ALIGNMENT_RIGHT,
    .trigger             = ADC_TRIGGER_SOFTWARE,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
    .scan_end_ipl        = (12),
    .scan_end_irq        = FSP_INVALID_VECTOR,
    .scan_end_b_ipl      = (BSP_IRQ_DISABLED),
    .scan_end_b_irq      = FSP_INVALID_VECTOR,
};

// ADC channel configuration
static adc_channel_cfg_t g_adc_channel_cfg = {
    .scan_mask           = 0,
    .scan_mask_group_b   = 0,
    .priority_group_a    = ADC_GROUP_A_PRIORITY_OFF,
    .add_mask            = 0,
    .sample_hold_mask    = 0,
    .sample_hold_states  = 24,
};

static bool adcInitialized = false;
static uint16_t adcValues[ADC_CHANNEL_COUNT];

void adcInit(const adcConfig_t *config)
{
    UNUSED(config);
    
    if (adcInitialized) {
        return;
    }

    // Initialize ADC0
    R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    
    // Configure channels based on adcTagMap
    uint32_t scan_mask = 0;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adcTagMap[i].tag) {
            // Convert IO tag to ADC channel
            // This is a simplified mapping - real implementation would need proper channel mapping
            uint8_t channel = i; // Simplified channel assignment
            scan_mask |= (1 << channel);
        }
    }
    
    g_adc_channel_cfg.scan_mask = scan_mask;
    R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc_channel_cfg);
    
    adcInitialized = true;
}

uint16_t adcGetChannel(uint8_t channel)
{
    if (!adcInitialized || channel >= ADC_CHANNEL_COUNT) {
        return 0;
    }
    
    return adcValues[channel];
}

void adcStartConversion(void)
{
    if (!adcInitialized) {
        return;
    }
    
    // Start ADC scan
    R_ADC_ScanStart(&g_adc0_ctrl);
}

bool adcConversionComplete(void)
{
    if (!adcInitialized) {
        return true;
    }
    
    // Check if scan is complete
    adc_status_t status;
    R_ADC_StatusGet(&g_adc0_ctrl, &status);
    
    return (status.state == ADC_STATE_SCAN_IN_PROGRESS) ? false : true;
}

void adcReadChannels(void)
{
    if (!adcInitialized) {
        return;
    }
    
    // Read all configured channels
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adcTagMap[i].tag) {
            uint16_t data;
            R_ADC_Read(&g_adc0_ctrl, i, &data);
            adcValues[i] = data;
        }
    }
}

// Battery voltage monitoring
uint16_t getBatteryVoltage(void)
{
    return adcGetChannel(ADC_BATTERY);
}

// Current monitoring
uint16_t getCurrentMeterValue(void)
{
    return adcGetChannel(ADC_CURRENT);
}

// RSSI monitoring
uint16_t getRSSIValue(void)
{
    return adcGetChannel(ADC_RSSI);
}

// External ADC input
uint16_t getExternalValue(void)
{
    return adcGetChannel(ADC_EXTERNAL1);
}

// ADC interrupt handler (if using interrupts)
void ADC0_SCAN_END_IRQHandler(void)
{
    // Read all channels when scan completes
    adcReadChannels();
}

#endif // USE_ADC