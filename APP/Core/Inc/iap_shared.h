/**
 * @file    iap_shared.h
 * @brief   IAP共享定义（BootLoader和APP共用）
 *
 * 包含地址定义、标志位操作等共享内容
 */

#ifndef IAP_SHARED_H
#define IAP_SHARED_H

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_pmu.h"
#include <stdint.h>

/* IAP 标志和地址定义 */
#define IAP_BOOT_FLAG_VALUE        ((uint16_t)0x5AA5U)
#define IAP_BOOT_RTC_INDEX         0U
#define IAP_BOOT_FLAG_ADDRESS      (RTC_BASE + 0x50U + (IAP_BOOT_RTC_INDEX * 4U))

#define BOOTLOADER_START_ADDRESS   ((uint32_t)0x08000000U)
#define APPLICATION_START_ADDRESS  ((uint32_t)0x08010000U)

/* IAP 数据存储区定义 */
#define IAP_DATA_FLASH_START       ((uint32_t)0x08008000U)  /* Flash中间区域用于IAP数据 */
#define IAP_DATA_FLASH_END         ((uint32_t)0x0800FFFFU)  /* IAP数据区结束 */
#define IAP_DATA_MAX_SIZE          (IAP_DATA_FLASH_END - IAP_DATA_FLASH_START + 1)

/* IAP 状态枚举 */
typedef enum
{
    IAP_RESULT_OK = 0,
    IAP_RESULT_ERROR,
    IAP_RESULT_CRC_ERROR,
    IAP_RESULT_FLASH_ERROR,
    IAP_RESULT_SIZE_ERROR,
    IAP_RESULT_TIMEOUT,
    IAP_RESULT_NOT_READY,
    IAP_RESULT_SD_ERROR,
    IAP_RESULT_FILE_ERROR
} iap_status_t;

/* IAP 升级信息结构 */
typedef struct
{
    uint32_t firmware_size;         /* 固件总大小 */
    uint32_t received_size;         /* 已接收大小 */
    uint16_t firmware_crc16;        /* 固件CRC16校验值 */
    int16_t calculated_crc16;       /* 计算得到的CRC16 */
    uint16_t current_packet;        /* 当前数据包号 */
    uint8_t  upgrade_status;        /* 升级状态 */
    uint8_t  error_code;            /* 错误代码 */
} iap_upgrade_info_t;

/* IAP 升级状态 */
typedef enum
{
    IAP_STATUS_IDLE = 0,
    IAP_STATUS_READY,
    IAP_STATUS_RECEIVING,
    IAP_STATUS_VERIFYING,
    IAP_STATUS_SUCCESS,
    IAP_STATUS_ERROR,
    IAP_STATUS_FLASH_ERROR,
    IAP_STATUS_CRC_ERROR,
    IAP_STATUS_SD_ERROR
} iap_status_enum;

static inline void iap_backup_write_enable(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPSRAM);
    pmu_backup_write_enable();
}

static inline void iap_backup_write_disable(void)
{
    pmu_backup_write_disable();
}

static inline void iap_flag_set(void)
{
    iap_backup_write_enable();
    REG32(IAP_BOOT_FLAG_ADDRESS) = IAP_BOOT_FLAG_VALUE;
    
    /* 等待写入完成：读回验证 */
    volatile uint32_t verify_count = 0;
    while (REG32(IAP_BOOT_FLAG_ADDRESS) != IAP_BOOT_FLAG_VALUE)
    {
        if (++verify_count > 1000U)  /* 防止死循环 */
        {
            break;
        }
    }
    
    iap_backup_write_disable();
}

static inline void iap_flag_clear(void)
{
    iap_backup_write_enable();
    REG32(IAP_BOOT_FLAG_ADDRESS) = 0U;
    iap_backup_write_disable();
}

static inline uint8_t iap_flag_is_set(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPSRAM);
    return (REG32(IAP_BOOT_FLAG_ADDRESS) == IAP_BOOT_FLAG_VALUE) ? 1U : 0U;
}

#endif /* IAP_SHARED_H */
