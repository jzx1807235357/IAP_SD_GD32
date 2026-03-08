/**
 * @file    sd_iap.h
 * @brief   SD卡IAP升级模块头文件
 * 
 * 基于SD卡和Ymodem协议的IAP在线升级功能
 */

#ifndef SD_IAP_H
#define SD_IAP_H

#include <stdint.h>
#include <stdbool.h>
#include "ymodem.h"

/* IAP状态枚举 */
typedef enum {
    SD_IAP_IDLE = 0,
    SD_IAP_NO_CARD,         /* 无SD卡 */
    SD_IAP_MOUNT_FAILED,    /* 挂载失败 */
    SD_IAP_FILE_NOT_FOUND,  /* 固件文件未找到 */
    SD_IAP_RECEIVING,       /* 正在接收 */
    SD_IAP_FLASH_ERROR,     /* Flash错误 */
    SD_IAP_CRC_ERROR,       /* CRC校验错误 */
    SD_IAP_SUCCESS,         /* 升级成功 */
    SD_IAP_READY_JUMP       /* 准备跳转 */
} sd_iap_status_t;

/* IAP配置结构 */
typedef struct {
    const char *firmware_path;      /* 固件文件路径 */
    uint32_t expected_size;         /* 期望的固件大小 (0表示不检查) */
    uint16_t expected_crc;          /* 期望的CRC16 (0表示不检查) */
} sd_iap_config_t;

/* SD-IAP句柄 */
typedef struct {
    sd_iap_status_t status;         /* 当前状态 */
    ymodem_handle_t ymodem;         /* Ymodem句柄 */
    uint32_t firmware_size;         /* 固件大小 */
    uint32_t bytes_written;         /* 已写入字节数 */
    uint8_t  progress;              /* 进度百分比 */
    uint8_t  error_code;            /* 错误码 */
    char     file_name[64];         /* 固件文件名 */
} sd_iap_handle_t;

/**
 * @brief 初始化SD-IAP模块
 * @return true成功，false失败
 */
bool sd_iap_init(void);

/**
 * @brief 检查SD卡是否存在
 * @return true存在，false不存在
 */
bool sd_iap_card_detected(void);

/**
 * @brief 挂载SD卡文件系统
 * @return 0成功，其他失败
 */
int sd_iap_mount(void);

/**
 * @brief 检查固件文件是否存在
 * @param path 文件路径
 * @return true存在，false不存在
 */
bool sd_iap_file_exists(const char *path);

/**
 * @brief 执行IAP升级
 * @param handle IAP句柄
 * @param config IAP配置
 * @return 0成功，其他失败
 */
int sd_iap_upgrade(sd_iap_handle_t *handle, const sd_iap_config_t *config);

/**
 * @brief 验证固件完整性
 * @param handle IAP句柄
 * @return true验证通过，false失败
 */
bool sd_iap_verify(sd_iap_handle_t *handle);

/**
 * @brief 获取升级进度
 * @param handle IAP句柄
 * @return 进度百分比(0-100)
 */
uint8_t sd_iap_get_progress(sd_iap_handle_t *handle);

/**
 * @brief 获取IAP状态字符串
 * @param status 状态码
 * @return 状态字符串
 */
const char* sd_iap_get_status_string(sd_iap_status_t status);

#endif /* SD_IAP_H */
