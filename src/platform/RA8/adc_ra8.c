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

#ifdef USE_ADC

#include "drivers/adc.h"
#include "drivers/adc_impl.h"
#include "drivers/io.h"

// FSP includes
#include "hal_data.h"

// ADC channel mapping
typedef struct {
    adc_instance_t *instance;
    adc_channel_t channel;
    ioTag_t pin;
} adcHardware_t;

static const adcHardware_t adcHardware[] = {
    { .instance = &g_adc0, .channel = ADC_CHANNEL_0, .pin = IO_TAG(VBAT_ADC_PIN) },           // Battery voltage
    { .instance = &g_adc0, .channel = ADC_CHANNEL_1, .pin = IO_TAG(CURRENT_METER_ADC_PIN) }, // Current sensor
    { .instance = &g_adc0, .channel = ADC_CHANNEL_2, .pin = IO_TAG(RSSI_ADC_PIN) },          // RSSI
    { .instance = &g_adc0, .channel = ADC_CHANNEL_3, .pin = IO_TAG(EXTERNAL_ADC_PIN) },      // External ADC
};

#define ADC_HARDWARE_COUNT (sizeof(adcHardware) / sizeof(adcHardware[0]))

static bool adcInitialized = false;
static uint16_t adcValues[ADC_HARDWARE_COUNT];

// ADC callback
void adc0_callback(adc_callback_args_t *p_args)
{
    if (p_args->event == ADC_EVENT_SCAN_COMPLETE) {
        // Scan complete - values are available
        for (int i = 0; i < ADC_HARDWARE_COUNT; i++) {
            // Read converted values
            // This would need to be implemented based on FSP ADC API
        }
    }
}

void adcInit(const adcConfig_t *config)
{
    UNUSED(config);
    
    if (adcInitialized) {
        return;
    }
    
    // Initialize ADC0
    fsp_err_t err = g_adc0.p_api->open(g_adc0.p_ctrl, g_adc0.p_cfg);
    if (err != FSP_SUCCESS) {
        return;
    }
    
    // Configure scan group
    err = g_adc0.p_api->scanCfg(g_adc0.p_ctrl, g_adc0.p_channel_cfg);
    if (err != FSP_SUCCESS) {
        return;
    }
    
    // Start calibration
    err = g_adc0.p_api->calibrate(g_adc0.p_ctrl, NULL);
    if (err != FSP_SUCCESS) {
        return;
    }
    
    // Start continuous scan
    err = g_adc0.p_api->scanStart(g_adc0.p_ctrl);
    if (err != FSP_SUCCESS) {
        return;
    }
    
    adcInitialized = true;
}

uint16_t adcGetChannel(uint8_t channel)
{
    if (!adcInitialized || channel >= ADC_HARDWARE_COUNT) {
        return 0;
    }
    
    // Read current value from ADC
    uint16_t value = 0;
    fsp_err_t err = g_adc0.p_api->read(g_adc0.p_ctrl, adcHardware[channel].channel, &value);
    if (err == FSP_SUCCESS) {
        adcValues[channel] = value;
    }
    
    return adcValues[channel];
}

// ADC channel functions
uint16_t adcGetVBat(void)
{
    return adcGetChannel(0); // VBAT is channel 0
}

uint16_t adcGetCurrent(void)
{
    return adcGetChannel(1); // Current is channel 1
}

uint16_t adcGetRSSI(void)
{
    return adcGetChannel(2); // RSSI is channel 2
}

uint16_t adcGetExternal(void)
{
    return adcGetChannel(3); // External is channel 3
}

// ADC configuration functions
void adcConfigChannel(ADCDevice device, ioTag_t ioTag, uint8_t adcChannel, uint8_t sampleTime)
{
    UNUSED(device);
    UNUSED(ioTag);
    UNUSED(adcChannel);
    UNUSED(sampleTime);
    // Channel configuration is handled by FSP configuration
}

void adcEnable(ADCDevice device)
{
    UNUSED(device);
    // ADC is enabled during initialization
}

void adcStartConversion(void)
{
    if (!adcInitialized) {
        return;
    }
    
    // Start single conversion
    g_adc0.p_api->scanStart(g_adc0.p_ctrl);
}

bool adcConversionComplete(void)
{
    if (!adcInitialized) {
        return false;
    }
    
    // Check if conversion is complete
    // This would need to be implemented based on FSP ADC status
    return true; // Simplified implementation
}

uint16_t adcGetValue(uint8_t channel)
{
    return adcGetChannel(channel);
}

// ADC utility functions
float adcToVoltage(uint16_t adcValue, float vref, uint16_t resolution)
{
    return ((float)adcValue * vref) / (float)((1 << resolution) - 1);
}

uint16_t voltageToADC(float voltage, float vref, uint16_t resolution)
{
    return (uint16_t)((voltage * (float)((1 << resolution) - 1)) / vref);
}

// Battery monitoring functions
float adcVBatToVoltage(uint16_t adcValue)
{
    // Convert ADC value to actual battery voltage
    // Assuming 3.3V reference and voltage divider
    const float vref = 3.3f;
    const float voltageDivider = 11.0f; // 10:1 voltage divider
    const uint16_t resolution = 12; // 12-bit ADC
    
    float voltage = adcToVoltage(adcValue, vref, resolution);
    return voltage * voltageDivider;
}

float adcCurrentToAmps(uint16_t adcValue)
{
    // Convert ADC value to current in amps
    // This depends on the current sensor used
    const float vref = 3.3f;
    const uint16_t resolution = 12;
    const float sensitivity = 0.066f; // 66mV/A for ACS712-30A
    const float offset = 2.5f; // 2.5V offset
    
    float voltage = adcToVoltage(adcValue, vref, resolution);
    return (voltage - offset) / sensitivity;
}

// ADC device management
bool adcIsEnabled(ADCDevice device)
{
    UNUSED(device);
    return adcInitialized;
}

void adcSetSampleTime(ADCDevice device, uint8_t sampleTime)
{
    UNUSED(device);
    UNUSED(sampleTime);
    // Sample time is configured in FSP configuration
}

uint8_t adcGetSampleTime(ADCDevice device)
{
    UNUSED(device);
    return 0; // Sample time not tracked
}

// ADC DMA functions (not implemented)
void adcStartDMA(void)
{
    // DMA not implemented
}

void adcStopDMA(void)
{
    // DMA not implemented
}

bool adcIsDMAEnabled(void)
{
    return false; // DMA not implemented
}

// ADC interrupt functions
void adcEnableInterrupt(ADCDevice device)
{
    UNUSED(device);
    // Interrupts are configured in FSP configuration
}

void adcDisableInterrupt(ADCDevice device)
{
    UNUSED(device);
    // Interrupts are configured in FSP configuration
}

// ADC temperature sensor (if available)
uint16_t adcGetTemperature(void)
{
    // Temperature sensor not implemented
    return 0;
}

float adcTemperatureToDegreesC(uint16_t adcValue)
{
    UNUSED(adcValue);
    // Temperature conversion not implemented
    return 25.0f; // Return room temperature
}

#endif // USE_ADC