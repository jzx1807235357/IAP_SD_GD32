#ifndef MODBUS_IAP_H
#define MODBUS_IAP_H

#include <stdint.h>
#include "modbus_rtu_iap.h"

/* IAP寄存器地址定义 */
#define MODBUS_IAP_REG_CTRL           20  /* IAP控制寄存器 */
#define MODBUS_IAP_REG_STATUS          21  /* IAP状态寄存器 */
#define MODBUS_IAP_REG_FIRMWARE_SIZE_L 22  /* 固件大小低16位 */
#define MODBUS_IAP_REG_FIRMWARE_SIZE_H 23  /* 固件大小高16位 */
#define MODBUS_IAP_REG_RECEIVED_SIZE_L 24  /* 已接收大小低16位 */
#define MODBUS_IAP_REG_RECEIVED_SIZE_H 25  /* 已接收大小高16位 */
#define MODBUS_IAP_REG_FIRMWARE_CRC    26  /* 固件CRC16 */
#define MODBUS_IAP_REG_PACKET_NUM      27  /* 数据包序号 */
#define MODBUS_IAP_REG_ERROR_CODE      28  /* 错误代码 */
#define MODBUS_IAP_REG_DATA_START      30  /* 数据缓冲区起始地址 */
#define MODBUS_IAP_REG_DATA_COUNT      64  /* 数据缓冲区寄存器数量（128字节） */

/* IAP控制命令 */
#define IAP_CMD_IDLE        0x0000  /* 空闲状态 */
#define IAP_CMD_START        0x0001  /* 开始IAP升级 */
#define IAP_CMD_DATA         0x0002  /* 数据传输模式 */
#define IAP_CMD_VERIFY       0x0003  /* 完成传输，开始校验 */
#define IAP_CMD_JUMP         0x0004  /* 跳转到应用程序 */
#define IAP_CMD_CANCEL       0x00FF  /* 取消升级 */

/* IAP状态 */
#define IAP_STATUS_READY       0x0000  /* 就绪状态 */
#define IAP_STATUS_RECEIVING   0x0001  /* 正在接收数据 */
#define IAP_STATUS_VERIFYING   0x0002  /* 数据校验中 */
#define IAP_STATUS_VERIFY_OK   0x0003  /* 校验成功，等待跳转命令 */
#define IAP_STATUS_READY_JUMP   0x0004  /* 准备跳转 */

/* IAP错误状态 */
#define IAP_STATUS_RX_ERROR    0x00E1  /* 数据接收错误 */
#define IAP_STATUS_CRC_ERROR    0x00E2  /* 校验失败 */
#define IAP_STATUS_FLASH_ERROR  0x00E3  /* Flash操作错误 */

/* IAP句柄 */
typedef struct
{
    uint16_t status;              /* 当前状态 */
    uint32_t firmware_size;       /* 固件总大小 */
    uint32_t received_size;       /* 已接收大小 */
    uint16_t expected_crc;        /* 期望的CRC16 */
    uint16_t calculated_crc;      /* 计算得到的CRC16 */
    uint16_t packet_num;          /* 当前包序号（来自寄存器） */
    uint16_t expected_packet;     /* 期望接收的下一个包号 */
    uint32_t flash_address;       /* Flash写入地址 */
    uint8_t data_buffer[128];     /* 数据缓冲区（128字节） */
    uint16_t error_code;          /* 错误代码 */
    uint32_t crc_pos;             /* CRC计算当前位置 */
    uint16_t crc_result;          /* CRC计算中间结果 */
    uint32_t verify_start_tick;   /* 开始VERIFY时的时间戳 */
    uint32_t verify_max_ticks;    /* 允许的最大tick差（超时保护） */
} modbus_iap_handle_t;

/* 函数声明 */
void modbus_iap_init(modbus_iap_handle_t *handle);
void modbus_iap_process(modbus_rtu_handle_t *rtu_handle, modbus_iap_handle_t *iap_handle);
uint16_t modbus_iap_get_register(modbus_iap_handle_t *handle, uint16_t reg_addr);
modbus_rtu_status_t modbus_iap_set_register(modbus_iap_handle_t *handle, uint16_t reg_addr, uint16_t value);
modbus_rtu_status_t modbus_iap_write_data_registers(modbus_iap_handle_t *handle, uint16_t start_reg, uint16_t *values, uint16_t count);
void modbus_iap_verify_step(modbus_iap_handle_t *handle);

#endif /* MODBUS_IAP_H */

