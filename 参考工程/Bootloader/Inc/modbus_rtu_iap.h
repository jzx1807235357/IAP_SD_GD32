#ifndef MODBUS_RTU_IAP_H
#define MODBUS_RTU_IAP_H

#include <stdint.h>
#include "rs485_host.h"

/* Modbus RTU 配置 */
#define MODBUS_RTU_SLAVE_ADDR_DEFAULT    0x01    /* 默认从机地址 */
#define MODBUS_RTU_RX_BUFFER_SIZE        256     /* 接收缓冲区大小 */
#define MODBUS_RTU_TX_BUFFER_SIZE        256     /* 发送缓冲区大小 */

/* 保持寄存器数量 */
#define HOLDING_REGISTERS                100     /* 保持寄存器总数 */

/* Modbus 功能码 */
#define READ_HOLDING_REGISTERS           0x03    /* 读保持寄存器 */
#define WRITE_HOLDING_REGISTER           0x06    /* 写单个寄存器 */
#define WRITE_MULTIPLE_REGISTERS         0x10    /* 写多个寄存器 */

/* 兼容旧代码的功能码宏定义 */
#define MODBUS_FUNC_READ_HOLDING_REGS     READ_HOLDING_REGISTERS
#define MODBUS_FUNC_WRITE_SINGLE_REG      WRITE_HOLDING_REGISTER
#define MODBUS_FUNC_WRITE_MULTIPLE_REGS   WRITE_MULTIPLE_REGISTERS

/* Modbus 异常码 */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE 0x04

/* 系统寄存器分配 */
#define REG_SLAVE_ADDR                   0       /* 从机地址 */
#define REG_BAUD_RATE                    1       /* 波特率 (值*100) */
#define REG_STOP_BITS                    2       /* 停止位 */
#define REG_PARITY                       3       /* 校验位 */
#define REG_VERSION                      4       /* 固件版本 (只读) */

/* 保留寄存器 5-19 用于未来扩展 */

/* IAP 寄存器分配 (20-93) */
#define IAP_REG_CONTROL                  20      /* IAP 控制命令 */
#define IAP_REG_STATUS                   21      /* IAP 状态反馈 */
#define IAP_REG_FW_SIZE_LOW              22      /* 固件总大小低位 */
#define IAP_REG_FW_SIZE_HIGH             23      /* 固件总大小高位 */
#define IAP_REG_RX_SIZE_LOW              24      /* 当前已接收大小低位 */
#define IAP_REG_RX_SIZE_HIGH             25      /* 当前已接收大小高位 */
#define IAP_REG_FW_CRC16                 26      /* 固件 CRC16 校验值 */
#define IAP_REG_PACKET_NUM               27      /* 数据包序号 */
#define IAP_REG_ERROR_CODE               28      /* 错误代码 */
#define IAP_REG_READ_ADDR_LOW            29      /* 回读地址低位(相对于APP起始地址的偏移) */
#define IAP_REG_READ_ADDR_HIGH           29      /* 回读地址高位(暂未使用,16位偏移足够) */
#define IAP_REG_DATA_START               30      /* 数据传输起始寄存器(30-93) */
#define IAP_REG_DATA_END                 93      /* 数据传输结束寄存器 */
#define IAP_DATA_REG_COUNT               64      /* 数据寄存器数量 (128字节) */

/* IAP 控制命令定义 */
#define IAP_CMD_IDLE                     0x0000  /* 空闲状态 */
#define IAP_CMD_START                    0x0001  /* 开始 IAP 升级 */
#define IAP_CMD_DATA                     0x0002  /* 数据传输模式 */
#define IAP_CMD_VERIFY                   0x0003  /* 完成传输，开始校验 */
#define IAP_CMD_JUMP                     0x0004  /* 跳转到 Bootloader */
#define IAP_CMD_READ_FLASH               0x0005  /* 回读Flash数据 */
#define IAP_CMD_CANCEL                   0x00FF  /* 取消升级 */

/* IAP 状态反馈定义 */
#define IAP_STATUS_READY                 0x0000  /* 就绪状态 */
#define IAP_STATUS_RECEIVING             0x0001  /* 正在接收数据 */
#define IAP_STATUS_VERIFYING             0x0002  /* 数据校验中 */
#define IAP_STATUS_VERIFY_OK             0x0003  /* 校验成功 */
#define IAP_STATUS_READY_JUMP            0x0004  /* 准备跳转 */

/* IAP 错误状态 */
#define IAP_STATUS_RX_ERROR              0x00E1  /* 数据接收错误 */
#define IAP_STATUS_CRC_ERROR             0x00E2  /* 校验失败 */
#define IAP_STATUS_FLASH_ERROR           0x00E3  /* Flash 操作错误 */

/* IAP 数据缓存大小 */
#define IAP_CACHE_SIZE                   256     /* 256字节内部缓存 */

/* Modbus RTU 状态 */
typedef enum
{
    MODBUS_RTU_OK = 0,
    MODBUS_RTU_ERROR,
    MODBUS_RTU_TIMEOUT,
    MODBUS_RTU_CRC_ERROR,
    MODBUS_RTU_ADDR_ERROR
} modbus_rtu_status_t;

/* 兼容旧代码的状态类型和宏定义 */
typedef modbus_rtu_status_t modbus_status_t;
#define MODBUS_OK           MODBUS_RTU_OK
#define MODBUS_ERROR        MODBUS_RTU_ERROR
#define MODBUS_TIMEOUT      MODBUS_RTU_TIMEOUT
#define MODBUS_CRC_ERROR    MODBUS_RTU_CRC_ERROR
#define MODBUS_ADDR_ERROR   MODBUS_RTU_ADDR_ERROR

/* Modbus RTU 句柄 */
typedef struct
{
    uint8_t slave_addr;                          /* 从机地址 */
    uint8_t rx_buffer[MODBUS_RTU_RX_BUFFER_SIZE];  /* 接收缓冲区 */
    uint16_t rx_length;                          /* 接收数据长度 */
    uint8_t tx_buffer[MODBUS_RTU_TX_BUFFER_SIZE];  /* 发送缓冲区 */
    uint16_t tx_length;                          /* 发送数据长度 */
} modbus_rtu_handle_t;

/* 全局保持寄存器数组 */
extern uint16_t modbus_holding_registers[HOLDING_REGISTERS];

/* 基础函数 */
void modbus_rtu_init(modbus_rtu_handle_t *handle, uint8_t slave_addr);
void modbus_rtu_set_register(uint16_t reg_addr, uint16_t value);
uint16_t modbus_rtu_get_register(uint16_t reg_addr);
uint16_t *modbus_rtu_get_register_ptr(void);

/* 通信函数 */
modbus_rtu_status_t modbus_rtu_receive_frame(modbus_rtu_handle_t *handle, uint32_t timeout_ms);
modbus_rtu_status_t modbus_rtu_send_response(modbus_rtu_handle_t *handle);
modbus_rtu_status_t modbus_rtu_send_exception(modbus_rtu_handle_t *handle, uint8_t func_code, uint8_t exception_code);

/* 帧处理函数 */
void modbus_rtu_parse_frame(modbus_rtu_handle_t *handle, uint8_t *data, uint16_t len);
void modbus_rtu_start_receive(modbus_rtu_handle_t *handle);
void modbus_rtu_send_frame(modbus_rtu_handle_t *handle, uint16_t len);
void modbus_rtu_process_frame(modbus_rtu_handle_t *handle);

/* 具体功能码处理函数 */
void modbus_rtu_read_holding_registers(modbus_rtu_handle_t *handle, uint16_t start_addr, uint16_t count);
void modbus_rtu_write_single_register(modbus_rtu_handle_t *handle, uint16_t reg_addr, uint16_t value);
void modbus_rtu_write_multiple_registers(modbus_rtu_handle_t *handle, uint16_t start_addr, uint16_t count, const uint8_t *data);

/* IAP 相关处理函数 */
void modbus_rtu_handle_iap_registers(uint16_t reg_addr);

/* 工具函数 */
void modbus_rtu_convert_to_uint16(const uint8_t *source, uint16_t *dest, uint16_t byte_count);
void modbus_rtu_convert_to_uint8(const uint16_t *source, uint8_t *dest, uint16_t reg_count);

/* CRC16 计算函数 */
uint16_t modbus_rtu_crc16(const uint8_t *data, uint16_t length);
uint16_t modbus_rtu_crc16_accumulate(uint16_t crc_init, const uint8_t *data, uint16_t length);

#endif /* MODBUS_RTU_IAP_H */

