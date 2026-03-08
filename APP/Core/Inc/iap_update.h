/**
 * @file    iap_update.h
 * @brief   IAP固件升级模块 - SD卡读取与FLASH写入
 *
 * 功能：
 * 1. 从SD卡读取固件文件
 * 2. 擦除APP区域FLASH
 * 3. 将固件写入FLASH
 * 4. 校验固件完整性
 * 5. 设置升级标志并重启
 */

#ifndef IAP_UPDATE_H
#define IAP_UPDATE_H

#include <stdint.h>
#include <stdbool.h>
#include "iap_shared.h"
#include "flash_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 固件文件路径 */
#define IAP_FIRMWARE_PATH       "0:firmware.bin"

/* IAP升级状态 */
typedef enum {
    IAP_UPDATE_IDLE = 0,
    IAP_UPDATE_SD_INIT,
    IAP_UPDATE_FILE_OPEN,
    IAP_UPDATE_FLASH_ERASE,
    IAP_UPDATE_WRITING,
    IAP_UPDATE_VERIFYING,
    IAP_UPDATE_SUCCESS,
    IAP_UPDATE_ERROR
} iap_update_state_t;

/* IAP升级上下文 */
typedef struct {
    iap_update_state_t state;       /* 当前状态 */
    uint32_t firmware_size;         /* 固件总大小 */
    uint32_t written_size;          /* 已写入大小 */
    uint32_t flash_address;         /* 当前FLASH写入地址 */
    uint16_t error_code;            /* 错误代码 */
    uint8_t  progress;              /* 进度百分比 (0-100) */
} iap_update_context_t;

/* 错误代码定义 */
#define IAP_ERROR_NONE              0x00
#define IAP_ERROR_SD_INIT           0x01    /* SD卡初始化失败 */
#define IAP_ERROR_SD_NO_CARD        0x02    /* 无SD卡 */
#define IAP_ERROR_FILE_OPEN         0x03    /* 文件打开失败 */
#define IAP_ERROR_FILE_READ         0x04    /* 文件读取失败 */
#define IAP_ERROR_FILE_SIZE         0x05    /* 文件大小无效 */
#define IAP_ERROR_FLASH_ERASE       0x06    /* FLASH擦除失败 */
#define IAP_ERROR_FLASH_WRITE       0x07    /* FLASH写入失败 */
#define IAP_ERROR_FLASH_VERIFY      0x08    /* FLASH校验失败 */
#define IAP_ERROR_FIRMWARE_TOO_LARGE 0x09   /* 固件过大 */

/*===========================================================================*/
/* 初始化函数                                                                 */
/*===========================================================================*/

/**
 * @brief 初始化IAP升级模块
 * @return true-成功, false-失败
 */
bool iap_update_init(void);

/**
 * @brief 获取IAP升级上下文
 * @return 上下文指针
 */
const iap_update_context_t* iap_update_get_context(void);

/*===========================================================================*/
/* 升级操作函数                                                               */
/*===========================================================================*/

/**
 * @brief 检查SD卡是否存在固件文件
 * @param path 固件文件路径
 * @return true-存在, false-不存在
 */
bool iap_check_firmware_file(const char* path);

/**
 * @brief 获取固件文件大小
 * @param path 固件文件路径
 * @return 文件大小(字节)，0表示失败
 */
uint32_t iap_get_firmware_size(const char* path);

/**
 * @brief 执行IAP升级（阻塞式）
 * @param path 固件文件路径，NULL使用默认路径
 * @return 升级结果
 */
iap_status_t iap_perform_upgrade(const char* path);

/**
 * @brief 执行IAP升级（非阻塞式，需轮询）
 * @param path 固件文件路径，NULL使用默认路径
 * @return true-开始成功, false-开始失败
 */
bool iap_start_upgrade_async(const char* path);

/**
 * @brief IAP升级轮询处理（非阻塞模式）
 * @return true-升级完成, false-进行中
 */
bool iap_update_poll(void);

/**
 * @brief 取消升级
 */
void iap_cancel_upgrade(void);

/*===========================================================================*/
/* 工具函数                                                                   */
/*===========================================================================*/

/**
 * @brief 设置IAP标志并重启进入BootLoader
 */
void iap_trigger_bootloader(void);

/**
 * @brief 计算CRC16校验值
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16值
 */
uint16_t iap_crc16_calc(const uint8_t* data, uint32_t len);

/**
 * @brief 校验FLASH中的固件
 * @param start_addr 起始地址
 * @param size 大小
 * @param expected_crc 期望的CRC值
 * @return true-校验通过, false-校验失败
 */
bool iap_verify_firmware(uint32_t start_addr, uint32_t size, uint16_t expected_crc);

#ifdef __cplusplus
}
#endif

#endif /* IAP_UPDATE_H */
