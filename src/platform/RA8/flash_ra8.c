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

#ifdef USE_FLASH_CHIP

#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/time.h"

// FSP includes
#include "hal_data.h"
#include "r_flash_lp.h"

// Flash instance control block
static flash_lp_ctrl_t g_flash_ctrl;
static const flash_lp_cfg_t g_flash_cfg = {
    .data_flash_bgo      = false,
    .p_callback          = NULL,
    .p_context           = NULL,
    .ipl                 = (BSP_IRQ_DISABLED),
    .irq                 = FSP_INVALID_VECTOR,
};

static bool flashInitialized = false;

// Flash geometry
#define FLASH_SECTOR_SIZE       0x4000  // 16KB sectors
#define FLASH_PAGE_SIZE         0x100   // 256 bytes per page
#define DATA_FLASH_START        0x40100000
#define DATA_FLASH_SIZE         0x2000  // 8KB data flash

bool flashInit(void)
{
    if (flashInitialized) {
        return true;
    }

    fsp_err_t err = R_FLASH_LP_Open(&g_flash_ctrl, &g_flash_cfg);
    if (err != FSP_SUCCESS) {
        return false;
    }

    flashInitialized = true;
    return true;
}

void flashPartitionSet(uint8_t type, uint32_t size)
{
    // RA8E1 uses fixed partitioning
    UNUSED(type);
    UNUSED(size);
}

uint32_t flashPartitionSize(uint8_t type)
{
    switch (type) {
    case FLASH_PARTITION_TYPE_CONFIG:
        return DATA_FLASH_SIZE;
    case FLASH_PARTITION_TYPE_FIRMWARE:
        return FLASH_SIZE * 1024 - DATA_FLASH_SIZE;
    default:
        return 0;
    }
}

bool flashIsReady(void)
{
    return flashInitialized;
}

bool flashWaitForReady(uint32_t timeoutMillis)
{
    uint32_t timeout = millis() + timeoutMillis;
    
    while (!flashIsReady()) {
        if (millis() > timeout) {
            return false;
        }
        delay(1);
    }
    
    return true;
}

bool flashEraseSector(uint32_t address)
{
    if (!flashInitialized) {
        return false;
    }

    // For data flash (configuration storage)
    if (address >= DATA_FLASH_START && address < (DATA_FLASH_START + DATA_FLASH_SIZE)) {
        fsp_err_t err = R_FLASH_LP_Erase(&g_flash_ctrl, address, 1);
        return (err == FSP_SUCCESS);
    }
    
    // For code flash, use different erase function
    // This would require different FSP flash driver for code flash
    return false;
}

bool flashPageProgram(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (!flashInitialized) {
        return false;
    }

    // Ensure length is aligned to flash write size
    if (length % 4 != 0) {
        return false;
    }

    fsp_err_t err = R_FLASH_LP_Write(&g_flash_ctrl, (uint32_t)data, address, length);
    return (err == FSP_SUCCESS);
}

void flashPageProgramBegin(uint32_t address, void (*callback)(uint32_t length))
{
    // Simplified implementation - callback not used
    UNUSED(address);
    UNUSED(callback);
}

uint32_t flashPageProgramContinue(const uint8_t **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    // Simplified implementation
    UNUSED(buffers);
    UNUSED(bufferSizes);
    UNUSED(bufferCount);
    return 0;
}

void flashPageProgramFinish(void)
{
    // Simplified implementation
}

void flashReadBytes(uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!flashInitialized) {
        return;
    }

    // Direct memory read for flash
    memcpy(buffer, (void*)address, length);
}

const flashGeometry_t *flashGetGeometry(void)
{
    static flashGeometry_t geometry = {
        .sectors = FLASH_SIZE * 1024 / FLASH_SECTOR_SIZE,
        .pagesPerSector = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE,
        .sectorSize = FLASH_SECTOR_SIZE,
        .totalSize = FLASH_SIZE * 1024,
        .pageSize = FLASH_PAGE_SIZE,
    };
    
    return &geometry;
}

// Configuration storage functions
bool flashReadConfig(uint8_t *buffer, uint32_t size)
{
    if (size > DATA_FLASH_SIZE) {
        return false;
    }
    
    flashReadBytes(DATA_FLASH_START, buffer, size);
    return true;
}

bool flashWriteConfig(const uint8_t *buffer, uint32_t size)
{
    if (size > DATA_FLASH_SIZE) {
        return false;
    }
    
    // Erase data flash first
    if (!flashEraseSector(DATA_FLASH_START)) {
        return false;
    }
    
    // Write configuration data
    return flashPageProgram(DATA_FLASH_START, buffer, size);
}

bool flashEraseConfig(void)
{
    return flashEraseSector(DATA_FLASH_START);
}

// Flash status functions
uint32_t flashGetSize(void)
{
    return FLASH_SIZE * 1024;
}

bool flashInConfigPartition(uint32_t address)
{
    return (address >= DATA_FLASH_START && address < (DATA_FLASH_START + DATA_FLASH_SIZE));
}

#endif // USE_FLASH_CHIP