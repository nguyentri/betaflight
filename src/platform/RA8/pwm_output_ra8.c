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

#ifdef USE_PWM_OUTPUT

#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/io.h"

// FSP includes
#include "hal_data.h"
#include "r_gpt.h"

// GPT timer control blocks
static gpt_ctrl_t g_timer0_ctrl;
static gpt_ctrl_t g_timer1_ctrl;
static gpt_ctrl_t g_timer2_ctrl;
static gpt_ctrl_t g_timer3_ctrl;

// GPT timer configurations for PWM
static const gpt_cfg_t g_timer0_cfg = {
    .channel                = 0,
    .mode                   = GPT_MODE_PWM,
    .period_counts          = 20000,  // 20ms period for 50Hz PWM
    .duty_cycle_counts      = 1500,   // 1.5ms pulse width (neutral)
    .source_div             = GPT_SOURCE_DIV_1,
    .p_callback             = NULL,
    .p_context              = NULL,
    .p_extend               = NULL,
    .cycle_end_ipl          = (BSP_IRQ_DISABLED),
    .cycle_end_irq          = FSP_INVALID_VECTOR,
};

static const gpt_cfg_t g_timer1_cfg = {
    .channel                = 1,
    .mode                   = GPT_MODE_PWM,
    .period_counts          = 20000,
    .duty_cycle_counts      = 1500,
    .source_div             = GPT_SOURCE_DIV_1,
    .p_callback             = NULL,
    .p_context              = NULL,
    .p_extend               = NULL,
    .cycle_end_ipl          = (BSP_IRQ_DISABLED),
    .cycle_end_irq          = FSP_INVALID_VECTOR,
};

static const gpt_cfg_t g_timer2_cfg = {
    .channel                = 2,
    .mode                   = GPT_MODE_PWM,
    .period_counts          = 20000,
    .duty_cycle_counts      = 1500,
    .source_div             = GPT_SOURCE_DIV_1,
    .p_callback             = NULL,
    .p_context              = NULL,
    .p_extend               = NULL,
    .cycle_end_ipl          = (BSP_IRQ_DISABLED),
    .cycle_end_irq          = FSP_INVALID_VECTOR,
};

static const gpt_cfg_t g_timer3_cfg = {
    .channel                = 3,
    .mode                   = GPT_MODE_PWM,
    .period_counts          = 20000,
    .duty_cycle_counts      = 1500,
    .source_div             = GPT_SOURCE_DIV_1,
    .p_callback             = NULL,
    .p_context              = NULL,
    .p_extend               = NULL,
    .cycle_end_ipl          = (BSP_IRQ_DISABLED),
    .cycle_end_irq          = FSP_INVALID_VECTOR,
};

// PWM output structures
typedef struct {
    gpt_ctrl_t *timer;
    const gpt_cfg_t *config;
    bool initialized;
} pwmOutputPort_t;

static pwmOutputPort_t pwmOutputPorts[MAX_SUPPORTED_MOTORS] = {
    { &g_timer0_ctrl, &g_timer0_cfg, false },
    { &g_timer1_ctrl, &g_timer1_cfg, false },
    { &g_timer2_ctrl, &g_timer2_cfg, false },
    { &g_timer3_ctrl, &g_timer3_cfg, false },
};

void pwmOutputInit(void)
{
    // Initialize all PWM output timers
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (!pwmOutputPorts[i].initialized) {
            R_GPT_Open(pwmOutputPorts[i].timer, pwmOutputPorts[i].config);
            R_GPT_Start(pwmOutputPorts[i].timer);
            pwmOutputPorts[i].initialized = true;
        }
    }
}

void pwmWriteMotor(uint8_t index, float value)
{
    if (index >= MAX_SUPPORTED_MOTORS) {
        return;
    }

    if (!pwmOutputPorts[index].initialized) {
        return;
    }

    // Convert value (0.0 to 1.0) to PWM duty cycle
    // Standard ESC range: 1000-2000 microseconds
    // For 20ms period (20000 counts), 1000us = 1000 counts, 2000us = 2000 counts
    uint32_t pulseWidth = 1000 + (uint32_t)(value * 1000); // 1000-2000 microseconds
    
    // Clamp to safe range
    if (pulseWidth < 1000) pulseWidth = 1000;
    if (pulseWidth > 2000) pulseWidth = 2000;

    R_GPT_DutyCycleSet(pwmOutputPorts[index].timer, pulseWidth, GPT_IO_PIN_GTIOCA);
}

void pwmWriteServo(uint8_t index, float value)
{
    // Servo control uses the same PWM mechanism as motors
    // but with different pulse width range (typically 500-2500us)
    if (index >= MAX_SUPPORTED_SERVOS) {
        return;
    }

    if (!pwmOutputPorts[index].initialized) {
        return;
    }

    // Convert value (-1.0 to 1.0) to servo PWM duty cycle
    uint32_t pulseWidth = 1500 + (int32_t)(value * 500); // 1000-2000 microseconds
    
    // Clamp to servo range
    if (pulseWidth < 500) pulseWidth = 500;
    if (pulseWidth > 2500) pulseWidth = 2500;

    R_GPT_DutyCycleSet(pwmOutputPorts[index].timer, pulseWidth, GPT_IO_PIN_GTIOCA);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        if (pwmOutputPorts[i].initialized) {
            // Set to minimum pulse width (motor off)
            R_GPT_DutyCycleSet(pwmOutputPorts[i].timer, 1000, GPT_IO_PIN_GTIOCA);
        }
    }
}

void pwmDisableMotors(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (pwmOutputPorts[i].initialized) {
            R_GPT_Stop(pwmOutputPorts[i].timer);
        }
    }
}

void pwmEnableMotors(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (pwmOutputPorts[i].initialized) {
            R_GPT_Start(pwmOutputPorts[i].timer);
        }
    }
}

bool pwmAreMotorsEnabled(void)
{
    // Check if any motor timer is running
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (pwmOutputPorts[i].initialized) {
            gpt_info_t info;
            R_GPT_InfoGet(pwmOutputPorts[i].timer, &info);
            if (info.status == GPT_STATUS_COUNTING) {
                return true;
            }
        }
    }
    return false;
}

// DShot support (simplified implementation)
#ifdef USE_DSHOT
void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command, bool blocking)
{
    // DShot implementation would require bit-banging or specialized timer configuration
    // This is a placeholder for future DShot implementation
    UNUSED(index);
    UNUSED(motorCount);
    UNUSED(command);
    UNUSED(blocking);
}

void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    // DShot implementation placeholder
    UNUSED(index);
    UNUSED(value);
}

void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    // DShot hardware configuration placeholder
    UNUSED(timerHardware);
    UNUSED(motorIndex);
    UNUSED(reorderedMotorIndex);
    UNUSED(pwmProtocolType);
    UNUSED(output);
}

#endif // USE_DSHOT

// Timer hardware configuration
void timerInit(void)
{
    // Timer initialization is handled by pwmOutputInit()
}

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    // Find timer hardware by IO tag
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].tag == ioTag) {
            return &timerHardware[i];
        }
    }
    return NULL;
}

#endif // USE_PWM_OUTPUT