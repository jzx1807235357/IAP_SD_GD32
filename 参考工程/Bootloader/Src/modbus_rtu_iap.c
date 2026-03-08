#include "modbus_rtu_iap.h"
#include "rs485_host.h"
#include "rs485_host_cfg.h"
#include "iap_shared.h"
#include "flash_driver.h"
#include "delay.h"
#include <string.h>

/* 全局保持寄存器数组 */
uint16_t modbus_holding_registers[HOLDING_REGISTERS] = {0};

/* 全局句柄指针，用于中断回调 */
static modbus_rtu_handle_t *g_modbus_handle = NULL;

/* CRC16 查表法 - 来自 L431Solar 标准表 */
const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40};

const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5,
    0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B,
    0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
    0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
    0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8,
    0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
    0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A,
    0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
    0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51,
    0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D,
    0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40};

/* 静态函数声明 */
static void modbus_rtu_handle_iap_control(uint16_t cmd);
static void modbus_rtu_special_convert_to_uint8(const uint16_t *source, uint8_t *dest, uint16_t length);

/**
 * @brief 初始化Modbus RTU句柄
 */
void modbus_rtu_init(modbus_rtu_handle_t *handle, uint8_t slave_addr)
{
    if (handle == NULL)
    {
        return;
    }
    
    /* 清空保持寄存器 */
    memset(modbus_holding_registers, 0, sizeof(modbus_holding_registers));
    
    /* 初始化句柄 */
    handle->slave_addr = slave_addr;
    handle->rx_length = 0;
    handle->tx_length = 0;
    memset(handle->rx_buffer, 0, sizeof(handle->rx_buffer));
    memset(handle->tx_buffer, 0, sizeof(handle->tx_buffer));
    
    /* 设置全局句柄指针 */
    g_modbus_handle = handle;
    
    /* 设置默认值 */
    modbus_holding_registers[REG_SLAVE_ADDR] = slave_addr;
    modbus_holding_registers[REG_BAUD_RATE] = 1152;  /* 115200 / 100 */
    modbus_holding_registers[REG_STOP_BITS] = 1;
    modbus_holding_registers[REG_PARITY] = 0;        /* 无校验 */
    modbus_holding_registers[REG_VERSION] = 0x0100;  /* 版本 1.00 */
    
    /* 初始化IAP状态 */
    modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_READY;
    modbus_holding_registers[IAP_REG_CONTROL] = IAP_CMD_IDLE;
}

/**
 * @brief 设置寄存器值
 */
void modbus_rtu_set_register(uint16_t reg_addr, uint16_t value)
{
    if (reg_addr < HOLDING_REGISTERS)
    {
        modbus_holding_registers[reg_addr] = value;
    }
}

/**
 * @brief 获取寄存器值
 */
uint16_t modbus_rtu_get_register(uint16_t reg_addr)
{
    if (reg_addr < HOLDING_REGISTERS)
    {
        return modbus_holding_registers[reg_addr];
    }
    return 0;
}

/**
 * @brief 获取寄存器数组指针
 */
uint16_t *modbus_rtu_get_register_ptr(void)
{
    return modbus_holding_registers;
}

/**
 * @brief 计算Modbus CRC16校验
 */
uint16_t modbus_rtu_crc16(const uint8_t *data, uint16_t length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i;        /* will index into CRC lookup */

    /* pass through message buffer */
    while (length--) {
        i = crc_lo ^ *data++; /* calculate the CRC  */
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

/**
 * @brief 累加计算CRC16（用于分段数据）
 */
uint16_t modbus_rtu_crc16_accumulate(uint16_t crc_init, const uint8_t *data, uint16_t length)
{
    uint8_t crc_hi = (uint8_t)(crc_init >> 8);
    uint8_t crc_lo = (uint8_t)(crc_init & 0xFF);
    uint16_t i;
    
    for (uint16_t idx = 0; idx < length; idx++)
    {
        i = crc_lo ^ data[idx];
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }
    
    return (uint16_t)((crc_hi << 8) | crc_lo);
}

/**
 * @brief 接收Modbus RTU帧
 */
modbus_rtu_status_t modbus_rtu_receive_frame(modbus_rtu_handle_t *handle, uint32_t timeout_ms)
{
    if (handle == NULL)
    {
        return MODBUS_RTU_ERROR;
    }
    
    /* 确保RS485处于接收模式 */
    rs485_host_set_direction(RS485_DIRECTION_RECEIVE);
    
    uint32_t timeout_us = timeout_ms * 1000U;
    uint8_t byte;
    uint16_t index = 0;
    const uint32_t frame_gap_ms = 5U; /* 帧间隔5ms */
    uint32_t no_data_count = 0;
    
    handle->rx_length = 0;
    
    /* 等待第一个字节 */
    if (rs485_host_receive(&byte, 1U, timeout_us) != 1)
    {
        return MODBUS_RTU_TIMEOUT;
    }
    
    handle->rx_buffer[index++] = byte;
    no_data_count = 0;
    
    /* 接收后续字节，直到帧间隔超时 */
    while (index < MODBUS_RTU_RX_BUFFER_SIZE)
    {
        if (rs485_host_receive(&byte, 1U, 1000U) == 1) /* 1ms超时 */
        {
            handle->rx_buffer[index++] = byte;
            no_data_count = 0;
        }
        else
        {
            no_data_count++;
            if (no_data_count >= frame_gap_ms)
            {
                break; /* 帧接收完成 */
            }
            delay_ms(1U);
        }
    }
    
    handle->rx_length = index;
    
    /* 检查最小帧长度（地址+功能码+CRC = 4字节） */
    if (handle->rx_length < 4)
    {
        return MODBUS_RTU_ERROR;
    }
    
    /* 验证CRC */
    uint16_t crc_received = (uint16_t)(handle->rx_buffer[handle->rx_length - 2]) |
                           ((uint16_t)(handle->rx_buffer[handle->rx_length - 1]) << 8);
    uint16_t crc_calculated = modbus_rtu_crc16(handle->rx_buffer, handle->rx_length - 2);
    
    if (crc_received != crc_calculated)
    {
        return MODBUS_RTU_CRC_ERROR;
    }
    
    /* 检查从机地址 */
    if (handle->rx_buffer[0] != handle->slave_addr && handle->rx_buffer[0] != 0)
    {
        return MODBUS_RTU_ADDR_ERROR;
    }
    
    return MODBUS_RTU_OK;
}

/**
 * @brief 发送Modbus RTU响应帧
 */
modbus_rtu_status_t modbus_rtu_send_response(modbus_rtu_handle_t *handle)
{
    if (handle == NULL || handle->tx_length == 0)
    {
        return MODBUS_RTU_ERROR;
    }
    
    /* 计算并添加CRC */
    uint16_t crc = modbus_rtu_crc16(handle->tx_buffer, handle->tx_length);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(crc & 0xFF);        /* 低字节在前 */
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(crc >> 8);          /* 高字节在后 */
    
    /* 发送数据 */
    rs485_host_send(handle->tx_buffer, handle->tx_length);
    
    return MODBUS_RTU_OK;
}

/**
 * @brief 发送Modbus异常响应
 */
modbus_rtu_status_t modbus_rtu_send_exception(modbus_rtu_handle_t *handle, uint8_t func_code, uint8_t exception_code)
{
    if (handle == NULL)
    {
        return MODBUS_RTU_ERROR;
    }
    
    handle->tx_length = 0;
    handle->tx_buffer[handle->tx_length++] = handle->slave_addr;
    handle->tx_buffer[handle->tx_length++] = func_code | 0x80; /* 异常响应标志 */
    handle->tx_buffer[handle->tx_length++] = exception_code;
    
    return modbus_rtu_send_response(handle);
}

/**
 * @brief 启动DMA接收（兼容接口）
 */
void modbus_rtu_start_receive(modbus_rtu_handle_t *handle)
{
    /* 这个函数保留为兼容性，实际接收在 modbus_rtu_receive_frame 中处理 */
    (void)handle;
}

/**
 * @brief 发送响应帧（兼容接口）
 */
void modbus_rtu_send_frame(modbus_rtu_handle_t *handle, uint16_t len)
{
    if (handle != NULL)
    {
        handle->tx_length = len;
        modbus_rtu_send_response(handle);
    }
}

/**
 * @brief 处理当前接收缓冲区中的Modbus RTU帧（兼容旧接口）
 */
void modbus_rtu_process_frame(modbus_rtu_handle_t *handle)
{
    if (handle == NULL || handle->rx_length == 0)
    {
        return;
    }

    modbus_rtu_parse_frame(handle, handle->rx_buffer, handle->rx_length);
}

/**
 * @brief 解析Modbus RTU帧，校验地址和CRC，处理功能码
 */
void modbus_rtu_parse_frame(modbus_rtu_handle_t *handle, uint8_t *data, uint16_t len)
{
    if (len < 4) return; /* 最小帧长度 */
    if (data[0] != handle->slave_addr && data[0] != 0x00) return; /* 只处理本机地址 */

    uint16_t crc_calc = modbus_rtu_crc16(data, len - 2);
    uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
    if (crc_calc != crc_recv) return; /* CRC校验失败 */

    uint8_t func = data[1];

    switch(func)
    {
        case READ_HOLDING_REGISTERS:
        {
            if (len >= 8)
            {
                uint16_t start_addr = (data[2] << 8) | data[3];
                uint16_t count = (data[4] << 8) | data[5];
                modbus_rtu_read_holding_registers(handle, start_addr, count);
            }
            break;
        }
        
        case WRITE_HOLDING_REGISTER:
        {
            if (len >= 8)
            {
                uint16_t reg_addr = (data[2] << 8) | data[3];
                uint16_t value = (data[4] << 8) | data[5];
                modbus_rtu_write_single_register(handle, reg_addr, value);
            }
            break;
        }

        case WRITE_MULTIPLE_REGISTERS:
        {
            if (len >= 9)
            {
                uint16_t start_addr = (data[2] << 8) | data[3];
                uint16_t count = (data[4] << 8) | data[5];
                uint8_t byte_count = data[6];
                
                if (len >= 9 + byte_count && byte_count == count * 2)
                {
                    modbus_rtu_write_multiple_registers(handle, start_addr, count, &data[7]);
                }
                else
                {
                    modbus_rtu_send_exception(handle, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                }
            }
            break;
        }
        
        default:
            modbus_rtu_send_exception(handle, func, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }
}

/**
 * @brief 读保持寄存器
 */
void modbus_rtu_read_holding_registers(modbus_rtu_handle_t *handle, uint16_t start_addr, uint16_t count)
{
    /* 检查地址范围 */
    if (start_addr + count > HOLDING_REGISTERS)
    {
        modbus_rtu_send_exception(handle, READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 构建响应 */
    handle->tx_length = 0;
    handle->tx_buffer[handle->tx_length++] = handle->slave_addr;
    handle->tx_buffer[handle->tx_length++] = READ_HOLDING_REGISTERS;
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(count * 2); /* 字节数 */
    
    /* 添加寄存器数据 */
    modbus_rtu_convert_to_uint8(&modbus_holding_registers[start_addr], &handle->tx_buffer[3], count);
    handle->tx_length += count * 2;
    
    modbus_rtu_send_response(handle);
}

/**
 * @brief 写单个寄存器
 */
void modbus_rtu_write_single_register(modbus_rtu_handle_t *handle, uint16_t reg_addr, uint16_t value)
{
    /* 检查地址范围 */
    if (reg_addr >= HOLDING_REGISTERS)
    {
        modbus_rtu_send_exception(handle, WRITE_HOLDING_REGISTER, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 检查写保护寄存器 */
    if (reg_addr == REG_VERSION)
    {
        modbus_rtu_send_exception(handle, WRITE_HOLDING_REGISTER, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 写入寄存器 */
    modbus_holding_registers[reg_addr] = value;
    
    /* IAP控制寄存器特殊处理 */
    if (reg_addr == IAP_REG_CONTROL)
    {
        modbus_rtu_handle_iap_registers(reg_addr);
    }
    
    /* 构建响应 (回传原始请求) */
    handle->tx_length = 0;
    handle->tx_buffer[handle->tx_length++] = handle->slave_addr;
    handle->tx_buffer[handle->tx_length++] = WRITE_HOLDING_REGISTER;
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(reg_addr >> 8);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(reg_addr & 0xFF);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(value >> 8);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(value & 0xFF);
    
    modbus_rtu_send_response(handle);
}

/**
 * @brief 写多个寄存器
 */
void modbus_rtu_write_multiple_registers(modbus_rtu_handle_t *handle, uint16_t start_addr, uint16_t count, const uint8_t *data)
{
    /* 检查地址范围 */
    if (start_addr + count > HOLDING_REGISTERS)
    {
        modbus_rtu_send_exception(handle, WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 写入寄存器数据 */
    modbus_rtu_convert_to_uint16(data, &modbus_holding_registers[start_addr], count * 2);
    
    /* 构建响应 */
    handle->tx_length = 0;
    handle->tx_buffer[handle->tx_length++] = handle->slave_addr;
    handle->tx_buffer[handle->tx_length++] = WRITE_MULTIPLE_REGISTERS;
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(start_addr >> 8);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(start_addr & 0xFF);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(count >> 8);
    handle->tx_buffer[handle->tx_length++] = (uint8_t)(count & 0xFF);
    
    modbus_rtu_send_response(handle);
}

/**
 * @brief 处理 IAP 寄存器操作
 */
void modbus_rtu_handle_iap_registers(uint16_t reg_addr)
{
    if (reg_addr == IAP_REG_CONTROL)
    {
        uint16_t cmd = modbus_holding_registers[IAP_REG_CONTROL];
        modbus_rtu_handle_iap_control(cmd);
    }
}

/**
 * @brief 处理IAP控制命令
 */
static void modbus_rtu_handle_iap_control(uint16_t cmd)
{
    switch (cmd)
    {
        case IAP_CMD_START:
        {
            /* 开始 IAP 升级 */
            
            /* 设置状态为接收中 */
            modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_RECEIVING;
            modbus_holding_registers[IAP_REG_RX_SIZE_LOW] = 0;
            modbus_holding_registers[IAP_REG_RX_SIZE_HIGH] = 0;
            
            break;
        }
        
        case IAP_CMD_DATA:
        {
            /* 数据传输模式 */
            if (modbus_holding_registers[IAP_REG_STATUS] == IAP_STATUS_READY ||
                modbus_holding_registers[IAP_REG_STATUS] == IAP_STATUS_RECEIVING)
            {
                /* 处理数据包 - 复制数据到缓存 */
                uint8_t data_bytes[IAP_DATA_REG_COUNT * 2];
                modbus_rtu_special_convert_to_uint8(&modbus_holding_registers[IAP_REG_DATA_START], data_bytes, IAP_DATA_REG_COUNT);
                
                /* 写入Flash (仅在 BootLoader 中实现) */
                #ifdef BOOTLOADER_MODE
                extern uint32_t g_flash_write_address;
                extern uint32_t g_received_size;
                
                /* 计算实际需要写入的字节数: 取剩余长度和128中的较小值 */
                uint32_t firmware_size = ((uint32_t)modbus_holding_registers[IAP_REG_FW_SIZE_HIGH] << 16) |
                                        modbus_holding_registers[IAP_REG_FW_SIZE_LOW];
                uint32_t remaining = (firmware_size > g_received_size) ? (firmware_size - g_received_size) : 0U;
                uint32_t write_len = (remaining < (IAP_DATA_REG_COUNT * 2)) ? remaining : (IAP_DATA_REG_COUNT * 2);
                
                if (write_len > 0U && flash_program_data(g_flash_write_address, data_bytes, write_len) == FLASH_OP_SUCCESS)
                {
                    g_flash_write_address += write_len;
                    g_received_size += write_len;
                    
                    /* 更新接收计数 */
                    modbus_holding_registers[IAP_REG_RX_SIZE_LOW] = (uint16_t)(g_received_size & 0xFFFF);
                    modbus_holding_registers[IAP_REG_RX_SIZE_HIGH] = (uint16_t)(g_received_size >> 16);
                    
                    modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_RECEIVING;
                }
                else
                {
                    modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_FLASH_ERROR;
                    modbus_holding_registers[IAP_REG_ERROR_CODE] = 0x02; /* Flash写入错误 */
                }
                #else
                /* APP 模式下仅更新计数器，实际写入由 BootLoader 完成 */
                uint32_t current_size = ((uint32_t)modbus_holding_registers[IAP_REG_RX_SIZE_HIGH] << 16) |
                                       modbus_holding_registers[IAP_REG_RX_SIZE_LOW];
                current_size += IAP_DATA_REG_COUNT * 2;
                modbus_holding_registers[IAP_REG_RX_SIZE_LOW] = (uint16_t)(current_size & 0xFFFF);
                modbus_holding_registers[IAP_REG_RX_SIZE_HIGH] = (uint16_t)(current_size >> 16);
                
                modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_RECEIVING;
                #endif
            }
            break;
        }
        
        case IAP_CMD_VERIFY:
        {
            /* 校验固件 */
            modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_VERIFYING;
            
            #ifdef BOOTLOADER_MODE
            uint16_t expected_crc = modbus_holding_registers[IAP_REG_FW_CRC16];
            uint32_t firmware_size = ((uint32_t)modbus_holding_registers[IAP_REG_FW_SIZE_HIGH] << 16) |
                                     (uint32_t)modbus_holding_registers[IAP_REG_FW_SIZE_LOW];

            /* 合法性检查：固件大小不能为0，且不超过APP最大容量 */
            if ((firmware_size == 0U) || (firmware_size > FLASH_APP_MAX_SIZE_BYTES))
            {
                modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_CRC_ERROR;
                modbus_holding_registers[IAP_REG_ERROR_CODE] = 0x01; /* 固件大小非法 */
                break;
            }

            /* 分块从Flash中读取并计算CRC16
             * 警告: 此校验过程会阻塞Modbus通信!
             * 对于7KB固件约需100ms,期间无法响应其他Modbus请求
             */
            uint16_t actual_crc = 0xFFFFU;
            uint32_t address = APPLICATION_START_ADDRESS;
            uint32_t remaining = firmware_size;
            uint32_t processed = 0;

            #ifdef BOOTLOADER_MODE
            extern void led_toggle(void);  /* LED闪烁指示校验进行中 */
            #endif

            while (remaining > 0U)
            {
                /* 每次处理256字节 */
                uint32_t chunk = (remaining > 256U) ? 256U : remaining;
                actual_crc = modbus_rtu_crc16_accumulate(actual_crc, (uint8_t *)address, (uint16_t)chunk);
                address   += chunk;
                remaining -= chunk;
                processed += chunk;

                /* 每处理1KB闪烁一次LED,提供及时反馈 */
                #ifdef BOOTLOADER_MODE
                if ((processed % 1024U) == 0U)
                {
                    led_toggle();
                    /* 处理Modbus请求,避免完全阻塞通信 */
                }
                #endif
            }

            /* 校验完成,恢复LED状态 */
            #ifdef BOOTLOADER_MODE
            extern void led_on(void);
            led_on();  /* LED常亮表示校验完成 */
            #endif

            if (actual_crc == expected_crc)
            {
                modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_READY_JUMP;
                modbus_holding_registers[IAP_REG_ERROR_CODE] = 0x00;
            }
            else
            {
                modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_CRC_ERROR;
                modbus_holding_registers[IAP_REG_ERROR_CODE] = 0x03; /* CRC校验不一致 */
            }
            #else
            /* APP 模式下假设校验成功 */
            modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_READY_JUMP;
            #endif
            
            break;
        }
        
        case IAP_CMD_JUMP:
        {
            /* 跳转到应用程序：由 BootLoader 主循环统一处理跳转逻辑 */
            #ifdef BOOTLOADER_MODE
            /* 仅当状态已经是 READY_JUMP 或 VERIFY_OK 时，确认跳转状态 */
            if (modbus_holding_registers[IAP_REG_STATUS] == IAP_STATUS_READY_JUMP ||
                modbus_holding_registers[IAP_REG_STATUS] == IAP_STATUS_VERIFY_OK)
            {
                /* 确保状态为READY_JUMP,主循环会检测到并执行跳转 */
                modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_READY_JUMP;
            }
            #endif
            break;
        }
        
        case IAP_CMD_READ_FLASH:
        {
            /* 回读Flash中的固件数据 */
            #ifdef BOOTLOADER_MODE
            /* 从寄存器29获取读取偏移地址(相对于APP起始地址) */
            uint32_t read_offset = modbus_holding_registers[IAP_REG_READ_ADDR_LOW];
            uint32_t firmware_size = ((uint32_t)modbus_holding_registers[IAP_REG_FW_SIZE_HIGH] << 16) |
                                     modbus_holding_registers[IAP_REG_FW_SIZE_LOW];
            
            /* 检查地址是否有效 */
            if (read_offset < firmware_size)
            {
                /* 计算实际读取长度: 剩余长度和128字节中的较小值 */
                uint32_t remaining = firmware_size - read_offset;
                uint32_t read_len = (remaining < (IAP_DATA_REG_COUNT * 2)) ? remaining : (IAP_DATA_REG_COUNT * 2);
                
                /* 从Flash读取数据到数据寄存器 */
                uint8_t *flash_addr = (uint8_t *)(APPLICATION_START_ADDRESS + read_offset);
                uint8_t read_buffer[IAP_DATA_REG_COUNT * 2];
                
                /* 读取Flash数据 */
                for (uint32_t i = 0; i < read_len; i++)
                {
                    read_buffer[i] = flash_addr[i];
                }
                
                /* 剩余部分填充0xFF */
                for (uint32_t i = read_len; i < (IAP_DATA_REG_COUNT * 2); i++)
                {
                    read_buffer[i] = 0xFF;
                }
                
                /* 转换为寄存器格式(小端序) */
                for (uint16_t i = 0; i < IAP_DATA_REG_COUNT; i++)
                {
                    modbus_holding_registers[IAP_REG_DATA_START + i] = 
                        (uint16_t)(read_buffer[i * 2] | (read_buffer[i * 2 + 1] << 8));
                }
                
                /* 更新读取进度 */
                modbus_holding_registers[IAP_REG_RX_SIZE_LOW] = (uint16_t)(read_offset & 0xFFFF);
                modbus_holding_registers[IAP_REG_RX_SIZE_HIGH] = (uint16_t)(read_offset >> 16);
            }
            else
            {
                /* 地址超出范围,数据区填充0xFF */
                for (uint16_t i = 0; i < IAP_DATA_REG_COUNT; i++)
                {
                    modbus_holding_registers[IAP_REG_DATA_START + i] = 0xFFFF;
                }
            }
            #endif
            break;
        }
        
        case IAP_CMD_CANCEL:
        {
            /* 取消升级 */
            modbus_holding_registers[IAP_REG_STATUS] = IAP_STATUS_READY;
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief 将uint8数组转换为uint16数组
 */
void modbus_rtu_convert_to_uint16(const uint8_t *source, uint16_t *dest, uint16_t byte_count)
{
    uint16_t reg_count = (byte_count + 1) / 2;
    
    for (uint16_t i = 0; i < reg_count; i++)
    {
        if (i * 2 + 1 < byte_count)
        {
            /* 高字节在前，低字节在后 (Big-Endian) */
            dest[i] = (source[i * 2] << 8) | source[i * 2 + 1];
        }
        else
        {
            /* 奇数字节数，最后一个字节 */
            dest[i] = source[i * 2] << 8;
        }
    }
}

/**
 * @brief 将uint16数组转换为uint8数组
 */
void modbus_rtu_convert_to_uint8(const uint16_t *source, uint8_t *dest, uint16_t reg_count)
{
    for (uint16_t i = 0; i < reg_count; i++)
    {
        /* 高字节在前，低字节在后 (Big-Endian) */
        dest[i * 2] = (uint8_t)(source[i] >> 8);
        dest[i * 2 + 1] = (uint8_t)(source[i] & 0xFF);
    }
}

/**
 * @brief IAP专用的uint16到uint8转换 (Little-Endian)
 */
static void modbus_rtu_special_convert_to_uint8(const uint16_t *source, uint8_t *dest, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        /* 低字节在前，高字节在后 (Little-Endian) */
        dest[i * 2] = (uint8_t)(source[i] & 0xFF);
        dest[i * 2 + 1] = (uint8_t)(source[i] >> 8);
    }
}

/* UART 空闲中断+DMA接收回调（兼容接口） */
void HAL_UARTEx_RxEventCallback(void *huart, uint16_t Size)
{
    if (!g_modbus_handle) return;

    g_modbus_handle->rx_length = Size;
    modbus_rtu_parse_frame(g_modbus_handle, g_modbus_handle->rx_buffer, Size);

    /* 重新启动DMA接收，持续监听 */
    modbus_rtu_start_receive(g_modbus_handle);
}

/* UART 错误回调（兼容接口） */
void HAL_UART_ErrorCallback(void *huart)
{
    if (g_modbus_handle) 
    {
        modbus_rtu_start_receive(g_modbus_handle);
    }
}
