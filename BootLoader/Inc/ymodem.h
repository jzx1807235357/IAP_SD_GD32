/**
 * @file    ymodem.h
 * @brief   Ymodem协议接收模块头文件（BootLoader版本）
 * 
 * 支持从SD卡文件读取固件，使用Ymodem格式解析
 */

#ifndef YMODEM_H
#define YMODEM_H

#include <stdint.h>
#include <stdbool.h>

/* Ymodem协议常量 */
#define YMODEM_SOH          0x01    /* 128字节数据包 */
#define YMODEM_STX          0x02    /* 1024字节数据包 */
#define YMODEM_EOT          0x04    /* 传输结束 */
#define YMODEM_ACK          0x06    /* 确认 */
#define YMODEM_NAK          0x15    /* 否认 */
#define YMODEM_CAN          0x18    /* 取消 */
#define YMODEM_C            0x43    /* 'C' - CRC模式请求 */

/* Ymodem数据包最大大小 */
#define YMODEM_PACKET_SIZE      1024
#define YMODEM_PACKET_128_SIZE  128

/* Ymodem接收状态 */
typedef enum {
    YMODEM_STATE_IDLE = 0,
    YMODEM_STATE_WAIT_HEADER,
    YMODEM_STATE_RECEIVING,
    YMODEM_STATE_WAIT_EOT,
    YMODEM_STATE_DONE,
    YMODEM_STATE_ERROR,
    YMODEM_STATE_ABORT
} ymodem_state_t;

/* Ymodem错误码 */
typedef enum {
    YMODEM_OK = 0,
    YMODEM_ERROR_CRC,
    YMODEM_ERROR_SEQUENCE,
    YMODEM_ERROR_TIMEOUT,
    YMODEM_ERROR_ABORT,
    YMODEM_ERROR_FILE,
    YMODEM_ERROR_SIZE,
    YMODEM_ERROR_FLASH
} ymodem_error_t;

/* Ymodem接收句柄 */
typedef struct {
    ymodem_state_t state;               /* 当前状态 */
    ymodem_error_t error;               /* 错误码 */
    uint8_t  packet_buf[1024 + 6];      /* 数据包缓冲区 (STX + seq + ~seq + data + crc) */
    uint16_t packet_len;                /* 数据包长度 */
    uint8_t  expected_seq;              /* 期望的包序号 */
    uint32_t total_size;                /* 固件总大小 */
    uint32_t received_size;             /* 已接收大小 */
    uint32_t flash_addr;                /* Flash写入地址 */
    uint16_t file_crc;                  /* 文件CRC16 */
    char     file_name[64];             /* 文件名 */
} ymodem_handle_t;

/* Ymodem解析结果 */
typedef enum {
    YMODEM_PARSE_OK = 0,
    YMODEM_PARSE_DONE,
    YMODEM_PARSE_EOT,
    YMODEM_PARSE_ERROR,
    YMODEM_PARSE_SKIP
} ymodem_parse_result_t;

/**
 * @brief 初始化Ymodem句柄
 */
void ymodem_init(ymodem_handle_t *handle);

/**
 * @brief 解析Ymodem数据包
 * @param handle Ymodem句柄
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 解析结果
 */
ymodem_parse_result_t ymodem_parse_packet(ymodem_handle_t *handle, const uint8_t *data, uint16_t len);

/**
 * @brief 从文件读取并解析Ymodem格式固件
 * @param handle Ymodem句柄
 * @param file_path 固件文件路径
 * @return 0成功，其他失败
 */
int ymodem_receive_from_file(ymodem_handle_t *handle, const char *file_path);

/**
 * @brief 获取接收进度百分比
 */
uint8_t ymodem_get_progress(ymodem_handle_t *handle);

#endif /* YMODEM_H */
