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

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/io.h"

// FSP includes
#include "hal_data.h"
#include "r_gpt.h"

// Timer instances
static timer_ctrl_t g_timer_ctrl[USABLE_TIMER_CHANNEL_COUNT];
static bool timerInitialized[USABLE_TIMER_CHANNEL_COUNT] = {false};

// Timer configuration template
static const timer_cfg_t g_timer_cfg_template = {
    .mode                = TIMER_MODE_PERIODIC,
    .period_counts       = 0xFFFFFFFF,
    .source_div          = TIMER_SOURCE_DIV_1,
    .channel             = 0,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = NULL,
    .cycle_end_ipl       = (12),
    .cycle_end_irq       = FSP_INVALID_VECTOR,
};

// Timer resource mapping
typedef struct {
    uint8_t timerId;
    uint8_t channel;
    ioTag_t ioTag;
} timerChannelMap_t;

// Timer channel mapping for RA8E1
static const timerChannelMap_t timerChannelMap[] = {
    // GPT0 channels
    { TIM_GPT0, 0, IO_TAG(GPIOA, 0) },  // GPT0 CH0 - PA0
    { TIM_GPT0, 1, IO_TAG(GPIOA, 1) },  // GPT0 CH1 - PA1
    
    // GPT1 channels  
    { TIM_GPT1, 0, IO_TAG(GPIOA, 2) },  // GPT1 CH0 - PA2
    { TIM_GPT1, 1, IO_TAG(GPIOA, 3) },  // GPT1 CH1 - PA3
    
    // GPT2 channels
    { TIM_GPT2, 0, IO_TAG(GPIOA, 4) },  // GPT2 CH0 - PA4
    { TIM_GPT2, 1, IO_TAG(GPIOA, 5) },  // GPT2 CH1 - PA5
    
    // GPT3 channels
    { TIM_GPT3, 0, IO_TAG(GPIOA, 6) },  // GPT3 CH0 - PA6
    { TIM_GPT3, 1, IO_TAG(GPIOA, 7) },  // GPT3 CH1 - PA7
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM_GPT0, CH_1, PA0,  TIM_USE_PWM | TIM_USE_PPM,   0, 0),
    DEF_TIM(TIM_GPT0, CH_2, PA1,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT1, CH_1, PA2,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT1, CH_2, PA3,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT2, CH_1, PA4,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT2, CH_2, PA5,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT3, CH_1, PA6,  TIM_USE_PWM,                0, 0),
    DEF_TIM(TIM_GPT3, CH_2, PA7,  TIM_USE_PWM,                0, 0),
};

void timerInit(void)
{
    // Initialize all timer channels
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        timerInitialized[i] = false;
    }
}

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].tag == ioTag) {
            return &timerHardware[i];
        }
    }
    return NULL;
}

timerOvrHandlerRec_t *timerCaptureInit(const timerHardware_t *timHw, uint16_t period, timerCaptureCallbackPtr callback, void *callbackParam)
{
    UNUSED(timHw);
    UNUSED(period);
    UNUSED(callback);
    UNUSED(callbackParam);
    
    // Simplified implementation - would need proper capture setup
    return NULL;
}

void timerCaptureCompareInit(const timerHardware_t *timHw, uint16_t period, uint32_t hz, timerCaptureCallbackPtr callback, void *callbackParam)
{
    UNUSED(timHw);
    UNUSED(period);
    UNUSED(hz);
    UNUSED(callback);
    UNUSED(callbackParam);
    
    // Simplified implementation
}

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    UNUSED(irqPriority);
    UNUSED(irq);
    
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT) {
        return;
    }
    
    if (timerInitialized[timerIndex]) {
        return;
    }
    
    // Configure timer based on type
    timer_cfg_t cfg = g_timer_cfg_template;
    cfg.channel = timHw->channel;
    
    switch (type) {
    case TYPE_PWM:
        cfg.mode = TIMER_MODE_PWM;
        break;
    case TYPE_PPM:
        cfg.mode = TIMER_MODE_ONE_SHOT;
        break;
    default:
        cfg.mode = TIMER_MODE_PERIODIC;
        break;
    }
    
    // Open timer
    fsp_err_t err = R_GPT_Open(&g_timer_ctrl[timerIndex], &cfg);
    if (err == FSP_SUCCESS) {
        timerInitialized[timerIndex] = true;
    }
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
}

void timerConfigBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return;
    }
    
    // Calculate period counts based on frequency
    uint32_t period_counts = (SystemCoreClock / hz) - 1;
    
    // Set period
    R_GPT_PeriodSet(&g_timer_ctrl[timerIndex], period_counts);
}

void timerStart(const timerHardware_t *timHw)
{
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return;
    }
    
    R_GPT_Start(&g_timer_ctrl[timerIndex]);
}

void timerStop(const timerHardware_t *timHw)
{
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return;
    }
    
    R_GPT_Stop(&g_timer_ctrl[timerIndex]);
}

uint16_t timerGetPeriod(const timerHardware_t *timHw)
{
    if (!timHw) {
        return 0;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return 0;
    }
    
    timer_info_t info;
    R_GPT_InfoGet(&g_timer_ctrl[timerIndex], &info);
    return (uint16_t)info.period_counts;
}

void timerPWMConfigChannel(const timerHardware_t *timHw, uint8_t reference, uint16_t value)
{
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return;
    }
    
    // Set duty cycle
    R_GPT_DutyCycleSet(&g_timer_ctrl[timerIndex], value, GPT_IO_PIN_GTIOCA);
}

uint16_t timerGetValue(const timerHardware_t *timHw)
{
    if (!timHw) {
        return 0;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return 0;
    }
    
    timer_status_t status;
    R_GPT_StatusGet(&g_timer_ctrl[timerIndex], &status);
    return (uint16_t)status.counter;
}

void timerSetValue(const timerHardware_t *timHw, uint16_t value)
{
    if (!timHw) {
        return;
    }
    
    uint8_t timerIndex = timHw - timerHardware;
    if (timerIndex >= USABLE_TIMER_CHANNEL_COUNT || !timerInitialized[timerIndex]) {
        return;
    }
    
    // Reset counter to specific value
    R_GPT_Reset(&g_timer_ctrl[timerIndex]);
    // Note: FSP GPT doesn't directly support setting counter value
    // This would need additional implementation
}

// System tick timer (usually SysTick)
void systemInit(void)
{
    // System initialization is handled in system_ra8e1.c
}

uint32_t micros(void)
{
    // Use system tick counter for microsecond timing
    // This is a simplified implementation
    static uint32_t lastTick = 0;
    static uint32_t microsOffset = 0;
    
    uint32_t currentTick = HAL_GetTick();
    if (currentTick != lastTick) {
        microsOffset += (currentTick - lastTick) * 1000;
        lastTick = currentTick;
    }
    
    return microsOffset;
}

uint32_t millis(void)
{
    return HAL_GetTick();
}

void delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void delayMicroseconds(uint32_t us)
{
    // Simple delay loop - not very accurate
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 4;
    while (count--) {
        __NOP();
    }
}