/**
 * @file    flash_driver.h
 * @brief   GD32F425RET6 Flash 擦写驱动抽象
 *
 * 基于《GD32F4xx 用户手册》3.2 章 FMC 描述和官方 FMC 例程，
 * 提供 BootLoader/APP 共用的 Flash 擦除、编程与校验接口。
 */

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#include "gd32f4xx.h"
#include "gd32f4xx_fmc.h"
#include "iap_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Flash 页面大小（GD32F425xx 支持 4KB 页擦除） */
#define FLASH_PAGE_SIZE_BYTES          (0x1000U)

/** GD32F425RET6 主 Flash 总容量（512KB） */
#define FLASH_TOTAL_SIZE_BYTES         (0x00080000U)

/** BootLoader 起始地址与大小（默认占用扇区0~3，64KB） */
#define FLASH_BOOT_START               (BOOTLOADER_START_ADDRESS)
#define FLASH_BOOT_SIZE_BYTES          (APPLICATION_START_ADDRESS - BOOTLOADER_START_ADDRESS)

/** APP 起始地址（默认从扇区4开始，64KB 对齐） */
#define FLASH_APP_START                (APPLICATION_START_ADDRESS)
/** APP 最大可用容量 */
#define FLASH_APP_MAX_SIZE_BYTES       (FLASH_TOTAL_SIZE_BYTES - FLASH_BOOT_SIZE_BYTES)

/** Flash 操作状态 */
typedef enum
{
    FLASH_OP_SUCCESS = 0,
    FLASH_OP_INVALID_PARAM,
    FLASH_OP_OUT_OF_RANGE,
    FLASH_OP_BUSY,
    FLASH_OP_PROTECTED,
    FLASH_OP_VERIFY_FAILED,
    FLASH_OP_ERROR
} flash_status_t;

/** Flash 扇区描述 */
typedef struct
{
    uint32_t start_address;
    uint32_t size_bytes;
    uint32_t sector_id; /**< 对应 gd32f4xx_fmc.h 中的 CTL_SECTOR_NUMBER_x 宏 */
} flash_sector_descriptor_t;

/* ---------- 查询接口 ---------- */

/**
 * @brief  根据扇区索引获取扇区信息。
 * @param  index  扇区索引（0~7）。
 * @return 指向描述符的指针；若索引非法则返回 NULL。
 */
const flash_sector_descriptor_t *flash_sector_by_index(uint32_t index);

/**
 * @brief  根据地址查找所在扇区。
 * @param  address Flash 绝对地址。
 * @return 指向描述符的指针；若地址不在主 Flash 中则返回 NULL。
 */
const flash_sector_descriptor_t *flash_sector_from_address(uint32_t address);

/* ---------- 擦除接口 ---------- */

/**
 * @brief  擦除 4KB 页（需要 4KB 对齐，GD32F425 专有）。
 */
flash_status_t flash_page_erase(uint32_t page_address);

/**
 * @brief  擦除单个扇区（地址需与扇区起始对齐）。
 */
flash_status_t flash_sector_erase(uint32_t sector_start_address);

/**
 * @brief  擦除 APP 区域（扇区粒度，长度向上对齐）。
 * @param  app_image_size APP 镜像大小（字节），用于计算擦除范围。
 */
flash_status_t flash_app_erase(uint32_t app_image_size);

/* ---------- 编程与校验 ---------- */

/**
 * @brief  按字（4 字节）编程 Flash。
 *         地址和长度需 4 字节对齐。
 */
flash_status_t flash_program_words(uint32_t address, const uint8_t *data, size_t length_bytes);

/**
 * @brief  校验指定范围是否为擦除态（0xFFFFFFFF）。
 */
flash_status_t flash_verify_blank(uint32_t start_address, uint32_t length_bytes);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_DRIVER_H */
