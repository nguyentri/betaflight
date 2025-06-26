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

#ifdef USE_TIMER

#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

// FSP includes
#include "hal_data.h"

// Timer definitions for RA8E1
typedef struct {
    gpt_instance_t *instance;
    uint8_t channel;
    ioTag_t ioTag;
} timerHardware_t;

// Timer hardware mapping
static const timerHardware_t timerHardware[] = {
    // GPT4 - Motors 1 & 2
    { .instance = &g_timer4, .channel = 0, .ioTag = IO_TAG(MOTOR1_PIN) },  // GPT4A
    { .instance = &g_timer4, .channel = 1, .ioTag = IO_TAG(MOTOR2_PIN) },  // GPT4B
    
    // GPT5 - Motors 3 & 4
    { .instance = &g_timer5, .channel = 0, .ioTag = IO_TAG(MOTOR3_PIN) },  // GPT5A
    { .instance = &g_timer5, .channel = 1, .ioTag = IO_TAG(MOTOR4_PIN) },  // GPT5B
    
    // GPT6 - Motors 5 & 6
    { .instance = &g_timer6, .channel = 0, .ioTag = IO_TAG(MOTOR5_PIN) },  // GPT6A
    { .instance = &g_timer6, .channel = 1, .ioTag = IO_TAG(MOTOR6_PIN) },  // GPT6B
};

#define TIMER_HARDWARE_COUNT (sizeof(timerHardware) / sizeof(timerHardware[0]))

static bool timersInitialized = false;

void timerInit(void)
{
    if (timersInitialized) {
        return;
    }

    // Initialize all GPT timers
    fsp_err_t err;
    
    // Initialize GPT4
    err = g_timer4.p_api->open(g_timer4.p_ctrl, g_timer4.p_cfg);
    if (err != FSP_SUCCESS) {
        // Handle error
        return;
    }
    
    // Initialize GPT5
    err = g_timer5.p_api->open(g_timer5.p_ctrl, g_timer5.p_cfg);
    if (err != FSP_SUCCESS) {
        // Handle error
        return;
    }
    
    // Initialize GPT6
    err = g_timer6.p_api->open(g_timer6.p_ctrl, g_timer6.p_cfg);
    if (err != FSP_SUCCESS) {
        // Handle error
        return;
    }

    timersInitialized = true;
}

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    for (unsigned i = 0; i < TIMER_HARDWARE_COUNT; i++) {
        if (timerHardware[i].ioTag == ioTag) {
            return &timerHardware[i];
        }
    }
    return NULL;
}

void timerStart(const timerHardware_t *timer)
{
    if (!timer || !timer->instance) {
        return;
    }
    
    timer->instance->p_api->start(timer->instance->p_ctrl);
}

void timerStop(const timerHardware_t *timer)
{
    if (!timer || !timer->instance) {
        return;
    }
    
    timer->instance->p_api->stop(timer->instance->p_ctrl);
}

void timerConfigBase(const timerHardware_t *timer, uint16_t period, uint32_t hz)
{
    if (!timer || !timer->instance) {
        return;
    }
    
    // Configure timer period and frequency
    timer_cfg_t cfg = *(timer->instance->p_cfg);
    cfg.period_counts = period;
    
    // Reconfigure timer
    timer->instance->p_api->close(timer->instance->p_ctrl);
    timer->instance->p_api->open(timer->instance->p_ctrl, &cfg);
}

void timerPWMConfigChannel(const timerHardware_t *timer, uint8_t channel, bool inverted, uint16_t value)
{
    if (!timer || !timer->instance) {
        return;
    }
    
    // Set PWM duty cycle
    timer->instance->p_api->dutyCycleSet(timer->instance->p_ctrl, value, 
                                        (channel == 0) ? GPT_IO_PIN_GTIOCA : GPT_IO_PIN_GTIOCB);
}

void timerPWMStart(const timerHardware_t *timer)
{
    timerStart(timer);
}

void timerPWMStop(const timerHardware_t *timer)
{
    timerStop(timer);
}

uint16_t timerGetPeriod(const timerHardware_t *timer)
{
    if (!timer || !timer->instance) {
        return 0;
    }
    
    timer_info_t info;
    timer->instance->p_api->infoGet(timer->instance->p_ctrl, &info);
    return (uint16_t)info.period_counts;
}

void timerSetPeriod(const timerHardware_t *timer, uint16_t period)
{
    if (!timer || !timer->instance) {
        return;
    }
    
    timer->instance->p_api->periodSet(timer->instance->p_ctrl, period);
}

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    // Return system clock frequency
    return SystemCoreClock;
}

// DShot support functions
void timerDMASafetySwitch(bool enable)
{
    UNUSED(enable);
    // DMA safety implementation for DShot
}

bool timerPWMConfigChannelDMA(const timerHardware_t *timer, uint8_t channel)
{
    UNUSED(timer);
    UNUSED(channel);
    // DMA configuration for DShot
    return false; // Not implemented yet
}

void timerDMACallback(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
    // DMA callback for DShot
}

// Timer utility functions
uint32_t timerGetBaseClock(const timerHardware_t *timer)
{
    UNUSED(timer);
    return SystemCoreClock;
}

void timerReconfigureTimeBase(const timerHardware_t *timer, uint16_t period, uint32_t hz)
{
    timerConfigBase(timer, period, hz);
}

// Motor timer functions
const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag)
{
    return timerGetByTag(ioTag);
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    UNUSED(owner);
    UNUSED(resourceIndex);
    return timerGetByTag(ioTag);
}

void timerReleaseAll(void)
{
    // Release all timers
    for (unsigned i = 0; i < TIMER_HARDWARE_COUNT; i++) {
        timerStop(&timerHardware[i]);
    }
}

#endif // USE_TIMER