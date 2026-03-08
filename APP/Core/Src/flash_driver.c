/**
 * @file    flash_driver.c
 * @brief   GD32F425RET6 Flash 擦写驱动实现
 *
 * 使用GD32官方FMC驱动库实现Flash擦除、编程与校验接口
 */

#include "flash_driver.h"

#include <string.h>

/* ---------- 内部常量 ---------- */

#define FLASH_MAIN_START_ADDRESS       (0x08000000U)

static const flash_sector_descriptor_t s_flash_sectors[] = {
    {0x08000000U, 0x00004000U, CTL_SECTOR_NUMBER_0}, /* 扇区0，16KB */
    {0x08004000U, 0x00004000U, CTL_SECTOR_NUMBER_1}, /* 扇区1，16KB */
    {0x08008000U, 0x00004000U, CTL_SECTOR_NUMBER_2}, /* 扇区2，16KB */
    {0x0800C000U, 0x00004000U, CTL_SECTOR_NUMBER_3}, /* 扇区3，16KB */
    {0x08010000U, 0x00010000U, CTL_SECTOR_NUMBER_4}, /* 扇区4，64KB */
    {0x08020000U, 0x00020000U, CTL_SECTOR_NUMBER_5}, /* 扇区5，128KB */
    {0x08040000U, 0x00020000U, CTL_SECTOR_NUMBER_6}, /* 扇区6，128KB */
    {0x08060000U, 0x00020000U, CTL_SECTOR_NUMBER_7}, /* 扇区7，128KB */
};

/* ---------- 内部工具函数 ---------- */

static flash_status_t flash_status_from_fmc(fmc_state_enum state)
{
    switch (state)
    {
        case FMC_READY:
            return FLASH_OP_SUCCESS;
        case FMC_BUSY:
            return FLASH_OP_BUSY;
        case FMC_WPERR:
            return FLASH_OP_PROTECTED;
        case FMC_PGMERR:
        case FMC_PGSERR:
        case FMC_RDDERR:
        case FMC_OPERR:
        case FMC_TOERR:
        default:
            return FLASH_OP_ERROR;
    }
}

static flash_status_t flash_wait_ready(void)
{
    fmc_state_enum state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    return flash_status_from_fmc(state);
}

static inline uint8_t flash_address_in_range(uint32_t address)
{
    return (address >= FLASH_MAIN_START_ADDRESS) &&
           (address < (FLASH_MAIN_START_ADDRESS + FLASH_TOTAL_SIZE_BYTES));
}

static flash_status_t flash_begin(void)
{
    fmc_unlock();
    flash_status_t status = flash_wait_ready();
    if (status != FLASH_OP_SUCCESS)
    {
        return status;
    }

    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR | FMC_FLAG_RDDERR);
    return FLASH_OP_SUCCESS;
}

static void flash_end(void)
{
    (void)flash_wait_ready();
    fmc_lock();
}

/* ---------- 对外实现 ---------- */

const flash_sector_descriptor_t *flash_sector_by_index(uint32_t index)
{
    if (index >= (sizeof(s_flash_sectors) / sizeof(s_flash_sectors[0])))
    {
        return NULL;
    }
    return &s_flash_sectors[index];
}

const flash_sector_descriptor_t *flash_sector_from_address(uint32_t address)
{
    if (!flash_address_in_range(address))
    {
        return NULL;
    }

    for (size_t i = 0U; i < (sizeof(s_flash_sectors) / sizeof(s_flash_sectors[0])); ++i)
    {
        const flash_sector_descriptor_t *sector = &s_flash_sectors[i];
        uint32_t end_address = sector->start_address + sector->size_bytes;
        if ((address >= sector->start_address) && (address < end_address))
        {
            return sector;
        }
    }

    return NULL;
}

flash_status_t flash_page_erase(uint32_t page_address)
{
    if ((page_address % FLASH_PAGE_SIZE_BYTES) != 0U)
    {
        return FLASH_OP_INVALID_PARAM;
    }
    if (!flash_address_in_range(page_address))
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    if (page_address < FLASH_APP_START)
    {
        return FLASH_OP_PROTECTED;
    }

    flash_status_t status = flash_begin();
    if (status != FLASH_OP_SUCCESS)
    {
        flash_end();
        return status;
    }

    /* 使用GD32官方页擦除函数 */
#if defined (GD32F425) || defined (GD32F427) || defined (GD32F470)
    fmc_state_enum state = fmc_page_erase(page_address);
    status = flash_status_from_fmc(state);
#else
    (void)page_address;
    status = FLASH_OP_ERROR;
#endif

    flash_end();
    return status;
}

flash_status_t flash_sector_erase(uint32_t sector_start_address)
{
    const flash_sector_descriptor_t *sector = flash_sector_from_address(sector_start_address);
    if (sector == NULL)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    if (sector->start_address != sector_start_address)
    {
        return FLASH_OP_INVALID_PARAM;
    }
    if (sector_start_address < FLASH_APP_START)
    {
        return FLASH_OP_PROTECTED;
    }

    /* 按照官方例程：解锁 */
    fmc_unlock();

    /* 按照官方例程：清除标志位 */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR | FMC_FLAG_RDDERR);

    /* 按照官方例程：擦除扇区（内部会等待完成） */
    fmc_state_enum state = fmc_sector_erase(sector->sector_id);

    /* 按照官方例程：检查擦除结果 */
    flash_status_t status;
    if (FMC_READY != state)
    {
        status = flash_status_from_fmc(state);
    }
    else
    {
        status = FLASH_OP_SUCCESS;
    }

    /* 按照官方例程：锁定 */
    fmc_lock();

    return status;
}

flash_status_t flash_app_erase(uint32_t app_image_size)
{
    if (app_image_size == 0U)
    {
        return FLASH_OP_SUCCESS;
    }
    if (app_image_size > FLASH_APP_MAX_SIZE_BYTES)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }

    uint32_t erase_start = FLASH_APP_START;
    uint32_t erase_end = FLASH_APP_START + app_image_size;
    if (erase_end < erase_start)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }

    if (!flash_address_in_range(erase_start) || !flash_address_in_range(erase_end - 1U))
    {
        return FLASH_OP_OUT_OF_RANGE;
    }

    /* 按照官方例程方式：每个扇区单独解锁-擦除-锁定 */
    while (erase_start < erase_end)
    {
        const flash_sector_descriptor_t *sector = flash_sector_from_address(erase_start);
        if (sector == NULL)
        {
            return FLASH_OP_OUT_OF_RANGE;
        }

        /* 按照官方例程：解锁 */
        fmc_unlock();

        /* 按照官方例程：清除标志位 */
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR | FMC_FLAG_RDDERR);

        /* 按照官方例程：擦除扇区（内部会等待完成） */
        fmc_state_enum state = fmc_sector_erase(sector->sector_id);

        /* 按照官方例程：检查擦除结果 */
        if (FMC_READY != state)
        {
            fmc_lock();
            return flash_status_from_fmc(state);
        }

        /* 按照官方例程：锁定 */
        fmc_lock();

        erase_start = sector->start_address + sector->size_bytes;
    }

    return FLASH_OP_SUCCESS;
}

flash_status_t flash_program_words(uint32_t address, const uint8_t *data, size_t length_bytes)
{
    if ((data == NULL) || (length_bytes == 0U))
    {
        return FLASH_OP_INVALID_PARAM;
    }
    if ((address % 4U) != 0U || (length_bytes % 4U) != 0U)
    {
        return FLASH_OP_INVALID_PARAM;
    }
    if (length_bytes > FLASH_TOTAL_SIZE_BYTES)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    uint32_t end_address = address + (uint32_t)length_bytes - 1U;
    if (end_address < address)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    if (!flash_address_in_range(address) || !flash_address_in_range(end_address))
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    if (address < FLASH_APP_START)
    {
        return FLASH_OP_PROTECTED;
    }

    flash_status_t status = flash_begin();
    if (status != FLASH_OP_SUCCESS)
    {
        flash_end();
        return status;
    }

    uint32_t current_address = address;

    for (size_t offset = 0U; offset < length_bytes; offset += 4U)
    {
        uint32_t word;
        memcpy(&word, &data[offset], sizeof(word));

        /* 使用GD32官方字编程函数 */
        fmc_state_enum state = fmc_word_program(current_address, word);
        status = flash_status_from_fmc(state);
        if (status != FLASH_OP_SUCCESS)
        {
            break;
        }

        current_address += 4U;
    }

    flash_end();
    return status;
}

flash_status_t flash_verify_blank(uint32_t start_address, uint32_t length_bytes)
{
    if ((length_bytes == 0U) || !flash_address_in_range(start_address))
    {
        return FLASH_OP_INVALID_PARAM;
    }
    if (length_bytes > FLASH_TOTAL_SIZE_BYTES)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    uint32_t end_address = start_address + (uint32_t)length_bytes - 1U;
    if (end_address < start_address)
    {
        return FLASH_OP_OUT_OF_RANGE;
    }
    if (!flash_address_in_range(end_address))
    {
        return FLASH_OP_OUT_OF_RANGE;
    }

    const uint32_t *ptr = (const uint32_t *)start_address;
    size_t words = length_bytes / sizeof(uint32_t);
    size_t remainder = length_bytes % sizeof(uint32_t);

    for (size_t i = 0U; i < words; ++i)
    {
        if (ptr[i] != 0xFFFFFFFFU)
        {
            return FLASH_OP_VERIFY_FAILED;
        }
    }

    if (remainder != 0U)
    {
        const uint8_t *tail = (const uint8_t *)&ptr[words];
        for (size_t j = 0U; j < remainder; ++j)
        {
            if (tail[j] != 0xFFU)
            {
                return FLASH_OP_VERIFY_FAILED;
            }
        }
    }

    return FLASH_OP_SUCCESS;
}
