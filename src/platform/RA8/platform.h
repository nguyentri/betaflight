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

#pragma once

// Platform identification
#define PLATFORM_RA8

// FSP and HAL includes
#include "hal_data.h"
#include "bsp_api.h"
#include "r_ioport.h"

// Core ARM includes
#include "core_cm85.h"

// Platform-specific definitions
#define FLASH_SIZE              512  // KB
#define FLASH_PAGE_SIZE         256  // bytes
#define FLASH_SECTOR_SIZE       16384 // bytes (16KB)

// Memory layout
#define FLASH_BASE              0x00000000
#define SRAM_BASE               0x20000000
#define DATA_FLASH_BASE         0x40100000

// System clock
#define SYSTEM_CORE_CLOCK       200000000  // 200MHz

// Timer definitions
#define TIM_GPT0                0
#define TIM_GPT1                1
#define TIM_GPT2                2
#define TIM_GPT3                3
#define TIM_GPT4                4
#define TIM_GPT5                5
#define TIM_GPT6                6
#define TIM_GPT7                7

#define USABLE_TIMER_CHANNEL_COUNT  8

// DMA definitions
#define DMA_MAX_CHANNELS        8
#define DMA_TAG_GET_CHANNEL(tag) ((tag) & 0xFF)

// ADC definitions
#define ADC_CHANNEL_COUNT       8
#define ADC_BATTERY             0
#define ADC_CURRENT             1
#define ADC_RSSI                2
#define ADC_EXTERNAL1           3

// SPI definitions
#define SPI_DEVICE_COUNT        3
#define SPI_DEVICE_1            0
#define SPI_DEVICE_2            1
#define SPI_DEVICE_3            2

// UART definitions
#define UART_COUNT              10
#define UART_DEVICE_1           0
#define UART_DEVICE_2           1
#define UART_DEVICE_3           2
#define UART_DEVICE_4           3
#define UART_DEVICE_5           4
#define UART_DEVICE_6           5
#define UART_DEVICE_7           6
#define UART_DEVICE_8           7
#define UART_DEVICE_9           8
#define UART_DEVICE_10          9

// I2C definitions
#define I2C_DEVICE_COUNT        2
#define I2C_DEVICE_1            0
#define I2C_DEVICE_2            1

// PWM definitions
#define PWM_OUTPUT_COUNT        8

// GPIO definitions
#define GPIOA                   0
#define GPIOB                   1
#define GPIOC                   2
#define GPIOD                   3
#define GPIOE                   4

// IO tag definitions
#define IO_TAG(port, pin)       ((((port) & 0x0F) << 4) | ((pin) & 0x0F))
#define IO_TAG_GET_PORT(tag)    (((tag) >> 4) & 0x0F)
#define IO_TAG_GET_PIN(tag)     ((tag) & 0x0F)

// Channel definitions for timer hardware
#define CH_1                    0
#define CH_2                    1
#define CH_3                    2
#define CH_4                    3

// Timer usage flags
#define TIM_USE_NONE            0x00
#define TIM_USE_PWM             0x01
#define TIM_USE_PPM             0x02
#define TIM_USE_MOTOR           0x04
#define TIM_USE_SERVO           0x08
#define TIM_USE_LED             0x10
#define TIM_USE_TRANSPONDER     0x20
#define TIM_USE_BEEPER          0x40

// Timer hardware definition macro
#define DEF_TIM(tim, ch, pin, usage, dmavar, dmaopt) \
    { \
        .tim = tim, \
        .channel = ch, \
        .tag = IO_TAG(pin), \
        .usageFlags = usage, \
        .dmaTag = dmavar, \
        .dmaOpt = dmaopt \
    }

// Platform-specific type definitions
typedef uint32_t ioTag_t;
typedef uint32_t dmaTag_t;

// Timer hardware structure
typedef struct {
    uint8_t tim;
    uint8_t channel;
    ioTag_t tag;
    uint8_t usageFlags;
    dmaTag_t dmaTag;
    uint8_t dmaOpt;
} timerHardware_t;

// DMA channel descriptor
typedef struct {
    dmaTag_t tag;
    uint8_t channel;
} dmaChannelDescriptor_t;

// DMA resource structure
typedef struct {
    uint8_t channel;
    dmaChannelDescriptor_t *descriptor;
} dmaResource_t;

// DMA transfer direction
typedef enum {
    DMA_PERIPH_TO_MEMORY,
    DMA_MEMORY_TO_PERIPH,
    DMA_MEMORY_TO_MEMORY
} dmaTransferDirection_t;

// Channel types
typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPM,
    TYPE_PWM,
    TYPE_SOFTSERIAL,
    TYPE_SERIAL,
    TYPE_TIMER
} channelType_t;

// Function pointer types
typedef void (*dmaCallbackHandlerFuncPtr)(uint8_t channel, bool complete);
typedef void (*timerCaptureCallbackPtr)(uint16_t capture, uint16_t period);
typedef void (*timerCCHandlerCallback)(uint16_t capture);
typedef void (*timerOvrHandlerCallback)(uint16_t capture);

// Timer callback handler structures
typedef struct {
    timerCCHandlerCallback *fn;
} timerCCHandlerRec_t;

typedef struct {
    timerOvrHandlerCallback *fn;
} timerOvrHandlerRec_t;

// Platform-specific macros
#define FAST_CODE               __attribute__((section(".fast_code")))
#define FAST_CODE_PREF          FAST_CODE
#define FAST_DATA               __attribute__((section(".fast_data")))

// Interrupt priorities
#define NVIC_PRIORITY_GROUPING  NVIC_PRIORITYGROUP_4
#define NVIC_PRIORITY_BASE(p)   (p)
#define NVIC_PRIORITY_SUB(p)    0

// System functions
void systemInit(void);
void systemReset(void);
void systemResetToBootloader(void);

// Time functions
uint32_t micros(void);
uint32_t millis(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// Memory functions
void *memset(void *s, int c, size_t n);
void *memcpy(void *dest, const void *src, size_t n);
int memcmp(const void *s1, const void *s2, size_t n);

// Platform initialization
void platformInit(void);

// External declarations
extern const timerHardware_t timerHardware[];

// HAL compatibility layer
#define HAL_GetTick()           millis()
#define HAL_Delay(ms)           delay(ms)

// Utility macros
#define UNUSED(x)               ((void)(x))
#define ARRAYLEN(x)             (sizeof(x) / sizeof((x)[0]))

// Bit manipulation macros
#define BIT(n)                  (1U << (n))
#define BITSET(var, bit)        ((var) |= BIT(bit))
#define BITCLEAR(var, bit)      ((var) &= ~BIT(bit))
#define BITTOGGLE(var, bit)     ((var) ^= BIT(bit))
#define BITCHECK(var, bit)      (((var) >> (bit)) & 1)

// Min/Max macros
#ifndef MIN
#define MIN(a, b)               ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)               ((a) > (b) ? (a) : (b))
#endif

// Constraint macro
#define CONSTRAIN(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))