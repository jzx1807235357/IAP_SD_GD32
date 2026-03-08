#include "modbus_iap.h"
#include "flash_driver.h"
#include "iap_shared.h"
#include "modbus_rtu_iap.h"
#include "systick.h"
#include <string.h>

/**
 * @brief 初始化IAP句柄
 */
void modbus_iap_init(modbus_iap_handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }
    
    handle->status = IAP_STATUS_READY;
    handle->firmware_size = 0;
    handle->received_size = 0;
    handle->expected_crc = 0;
    handle->calculated_crc = 0;
    handle->packet_num = 0;
    handle->expected_packet = 1;  /* 期望第一个包从 1 开始 */
    handle->flash_address = APPLICATION_START_ADDRESS;
    handle->error_code = 0;
    handle->crc_pos = 0;
    handle->crc_result = 0xFFFF;
    handle->verify_start_tick = 0;
    handle->verify_max_ticks = 0;
    memset(handle->data_buffer, 0, sizeof(handle->data_buffer));
}

/**
 * @brief 分段执行CRC16校验(在主循环中调用,不阻塞Modbus)
 * @note 每次调用只计算128字节,保持Modbus响应能力
 */
void modbus_iap_verify_step(modbus_iap_handle_t *handle)
{
    if (handle == NULL || handle->status != IAP_STATUS_VERIFYING)
    {
        return;
    }
    
    /* 超时保护：防止CRC校验卡死 */
    if (handle->verify_max_ticks != 0)
    {
        uint32_t now = systick_get_tick();
        if ((now - handle->verify_start_tick) > handle->verify_max_ticks)
        {
            /* 校验超时 */
            handle->status = IAP_STATUS_CRC_ERROR;
            handle->error_code = 0x0004; /* 错误码：校验超时 */
            return;
        }
    }
    
    /* 每次尽量多计算一些字节,加快整体校验速度 */
    if (handle->crc_pos < handle->received_size)
    {
        uint8_t read_buffer[128];
        uint32_t total_remaining = handle->received_size - handle->crc_pos;
        /* 限制单次调用最多处理 4096 字节,避免阻塞时间过长 */
        uint32_t bytes_to_process = (total_remaining > 4096U) ? 4096U : total_remaining;

        while (bytes_to_process > 0U)
        {
            uint32_t chunk_size = (bytes_to_process > 128U) ? 128U : bytes_to_process;

            /* 从Flash读取数据 */
            for (uint32_t i = 0; i < chunk_size; i++)
            {
                read_buffer[i] = *(volatile uint8_t *)(APPLICATION_START_ADDRESS + handle->crc_pos + i);
            }

            /* 累加计算CRC */
            handle->crc_result = modbus_rtu_crc16_accumulate(handle->crc_result, read_buffer, (uint16_t)chunk_size);
            handle->crc_pos += chunk_size;
            bytes_to_process -= chunk_size;
        }
    }
    else
    {
        /* CRC计算完成,比较结果 */
        handle->calculated_crc = handle->crc_result;
        
        if (handle->calculated_crc == handle->expected_crc || 
            handle->expected_crc == 0) /* 允许CRC为0跳过校验 */
        {
            /* 校验成功 */
            handle->status = IAP_STATUS_VERIFY_OK;
        }
        else
        {
            /* 校验失败 */
            handle->status = IAP_STATUS_CRC_ERROR;
            handle->error_code = 0x0003;
        }
    }
}

/**
 * @brief 获取IAP寄存器值
 */
uint16_t modbus_iap_get_register(modbus_iap_handle_t *handle, uint16_t reg_addr)
{
    if (handle == NULL)
    {
        return 0;
    }
    
    switch (reg_addr)
    {
        case MODBUS_IAP_REG_CTRL:
            return 0; /* 控制寄存器只写 */
            
        case MODBUS_IAP_REG_STATUS:
            return handle->status;
            
        case MODBUS_IAP_REG_FIRMWARE_SIZE_L:
            return (uint16_t)(handle->firmware_size & 0xFFFF);
            
        case MODBUS_IAP_REG_FIRMWARE_SIZE_H:
            return (uint16_t)(handle->firmware_size >> 16);
            
        case MODBUS_IAP_REG_RECEIVED_SIZE_L:
            return (uint16_t)(handle->received_size & 0xFFFF);
            
        case MODBUS_IAP_REG_RECEIVED_SIZE_H:
            return (uint16_t)(handle->received_size >> 16);
            
        case MODBUS_IAP_REG_FIRMWARE_CRC:
            return handle->expected_crc;
            
        case MODBUS_IAP_REG_PACKET_NUM:
            return handle->packet_num;
            
        case MODBUS_IAP_REG_ERROR_CODE:
            return handle->error_code;
            
        default:
            if (reg_addr >= MODBUS_IAP_REG_DATA_START && 
                reg_addr < MODBUS_IAP_REG_DATA_START + MODBUS_IAP_REG_DATA_COUNT)
            {
                uint16_t offset = (reg_addr - MODBUS_IAP_REG_DATA_START) * 2;
                if (offset + 1 < sizeof(handle->data_buffer))
                {
                    return (uint16_t)(handle->data_buffer[offset] | 
                                     ((uint16_t)handle->data_buffer[offset + 1] << 8));
                }
            }
            return 0;
    }
}

/**
 * @brief 设置IAP寄存器值
 */
modbus_rtu_status_t modbus_iap_set_register(modbus_iap_handle_t *handle, uint16_t reg_addr, uint16_t value)
{
    if (handle == NULL)
    {
        return MODBUS_RTU_ERROR;
    }
    
    switch (reg_addr)
    {
        case MODBUS_IAP_REG_CTRL:
        {
            switch (value)
            {
                case IAP_CMD_START:
                    /* 开始升级命令：只重置状态和计数器，不擦除Flash
                     * 说明：bootloader_main.c 启动时已完成Flash擦除，此处只需重置IAP状态机
                     * 这样上位机可以"先发START，再写FIRMWARE_SIZE"，避免顺序冲突
                     */
                    handle->status = IAP_STATUS_RECEIVING;
                    handle->received_size = 0;
                    handle->flash_address = APPLICATION_START_ADDRESS;
                    handle->packet_num = 0;
                    handle->expected_packet = 1;  /* 从第 1 包开始 */
                    handle->error_code = 0;
                    break;
                    
                case IAP_CMD_DATA:
                    /* 处理数据：使用 REG_PACKET_NUM + data_buffer 全部内容写入Flash */
                    if (handle->status == IAP_STATUS_RECEIVING)
                    {
                        /* 0. 检查 firmware_size 是否已设置 */
                        if (handle->firmware_size == 0U)
                        {
                            handle->status = IAP_STATUS_RX_ERROR;
                            handle->error_code = 0x0007; /* 固件大小未设置 */
                            break;
                        }

                        /* 1. 检查包序号是否符合预期（packet_num 由 REG_PACKET_NUM 写入） */
                        if (handle->packet_num != handle->expected_packet)
                        {
                            handle->status = IAP_STATUS_RX_ERROR;
                            handle->error_code = 0x0005; /* 包序号错误 */
                            break;
                        }

                        /* 2. 计算本次有效数据长度（data_buffer 全部128字节都是固件数据） */
                        uint32_t remaining = handle->firmware_size - handle->received_size;
                        uint32_t max_payload = 128U;  /* 完整128字节 */
                        uint32_t data_length = (remaining < max_payload) ? remaining : max_payload;

                        if (data_length > 0U)
                        {
                            /* 3. 4字节对齐 */
                            uint32_t aligned_length = (data_length + 3U) & ~0x03U;

                            /* 4. 不足部分填充0xFF */
                            if (aligned_length > data_length)
                            {
                                for (uint32_t i = data_length; i < aligned_length; i++)
                                {
                                    handle->data_buffer[i] = 0xFF;
                                }
                            }

                            /* 5. 写入Flash：data_buffer[0..aligned_length-1] 全是固件数据 */
                            if (flash_program_words(handle->flash_address,
                                                    handle->data_buffer,
                                                    aligned_length) == FLASH_OP_SUCCESS)
                            {
                                handle->received_size += data_length;      /* 用真实数据长度 */
                                handle->flash_address += aligned_length;   /* Flash地址按对齐增加 */

                                /* 下一包序号 */
                                handle->expected_packet++;
                            }
                            else
                            {
                                handle->status = IAP_STATUS_FLASH_ERROR;
                                handle->error_code = 0x0002; /* Flash写入失败 */
                            }
                        }
                        /* data_length == 0：所有数据已接收完，忽略 */
                    }
                    break;
                    
                case IAP_CMD_VERIFY:
                    /* 校验固件 - 启动分段CRC计算,不阻塞 */
                    if (handle->status == IAP_STATUS_RECEIVING)
                    {
                        /* 检查接收长度是否与声明的一致 */
                        if (handle->received_size != handle->firmware_size || handle->firmware_size == 0U)
                        {
                            handle->status = IAP_STATUS_RX_ERROR;
                            handle->error_code = 0x0006; /* 接收长度与声明不符 */
                            break;
                        }

                        /* 初始化CRC计算状态 */
                        handle->status = IAP_STATUS_VERIFYING;
                        handle->crc_pos = 0;
                        handle->crc_result = 0xFFFF; /* Modbus CRC16初始值 */
                        
                        /* 设置超时保护：根据固件大小计算预期时间 */
                        uint32_t estimated_ms = (handle->received_size / 1024U) * 30U + 10000U;
                        handle->verify_start_tick = systick_get_tick();
                        handle->verify_max_ticks = estimated_ms;
                    }
                    break;
                    
                case IAP_CMD_JUMP:
                    /* 跳转到应用程序 */
                    if (handle->status == IAP_STATUS_VERIFY_OK)
                    {
                        /* 校验成功后才允许跳转 */
                        handle->status = IAP_STATUS_READY_JUMP;
                    }
                    break;
                    
                case IAP_CMD_CANCEL:
                    /* 取消升级 - 完全复位IAP状态 */
                    handle->status = IAP_STATUS_READY;
                    handle->firmware_size = 0;
                    handle->received_size = 0;
                    handle->expected_crc = 0;
                    handle->calculated_crc = 0;
                    handle->packet_num = 0;
                    handle->expected_packet = 1;
                    handle->flash_address = APPLICATION_START_ADDRESS;
                    handle->error_code = 0;
                    handle->crc_pos = 0;
                    handle->crc_result = 0xFFFF;
                    handle->verify_start_tick = 0;
                    handle->verify_max_ticks = 0;
                    /* 可以由上位机在取消后决定是否跳转APP或重新升级 */
                    break;
                    
                default:
                    break;
            }
            break;
        }
        
        case MODBUS_IAP_REG_FIRMWARE_SIZE_L:
            handle->firmware_size = (handle->firmware_size & 0xFFFF0000) | value;
            /* 写入低16位后检查范围 */
            if (handle->firmware_size > FLASH_APP_MAX_SIZE_BYTES)
            {
                handle->firmware_size = 0;  /* 非法值,清零 */
                handle->status = IAP_STATUS_RX_ERROR;
                handle->error_code = 0x0008;  /* 固件大小超出范围 */
            }
            break;
            
        case MODBUS_IAP_REG_FIRMWARE_SIZE_H:
            handle->firmware_size = (handle->firmware_size & 0x0000FFFF) | ((uint32_t)value << 16);
            /* 写入高16位后检查范围 */
            if (handle->firmware_size > FLASH_APP_MAX_SIZE_BYTES)
            {
                handle->firmware_size = 0;  /* 非法值,清零 */
                handle->status = IAP_STATUS_RX_ERROR;
                handle->error_code = 0x0008;  /* 固件大小超出范围 */
            }
            break;
            
        case MODBUS_IAP_REG_FIRMWARE_CRC:
            handle->expected_crc = value;
            break;
            
        case MODBUS_IAP_REG_PACKET_NUM:
            handle->packet_num = value;
            break;
            
        default:
            return MODBUS_RTU_ERROR;
    }
    
    return MODBUS_RTU_OK;
}

/**
 * @brief 写入数据寄存器（功能码0x10）
 */
modbus_rtu_status_t modbus_iap_write_data_registers(modbus_iap_handle_t *handle, 
                                                     uint16_t start_reg, 
                                                     uint16_t *values, 
                                                     uint16_t count)
{
    if (handle == NULL || values == NULL)
    {
        return MODBUS_RTU_ERROR;
    }
    
    if (start_reg >= MODBUS_IAP_REG_DATA_START && 
        start_reg < MODBUS_IAP_REG_DATA_START + MODBUS_IAP_REG_DATA_COUNT)
    {
        uint16_t offset = (start_reg - MODBUS_IAP_REG_DATA_START) * 2;
        
        for (uint16_t i = 0; i < count && (offset + i * 2 + 1) < sizeof(handle->data_buffer); i++)
        {
            handle->data_buffer[offset + i * 2] = (uint8_t)(values[i] & 0xFF);
            handle->data_buffer[offset + i * 2 + 1] = (uint8_t)(values[i] >> 8);
        }
        
        return MODBUS_RTU_OK;
    }
    
    return MODBUS_RTU_ERROR;
}

/**
 * @brief 处理Modbus IAP请求
 */
void modbus_iap_process(modbus_rtu_handle_t *rtu_handle, modbus_iap_handle_t *iap_handle)
{
    if (rtu_handle == NULL || iap_handle == NULL || rtu_handle->rx_length < 4)
    {
        return;
    }
    
    uint8_t func_code = rtu_handle->rx_buffer[1];
    rtu_handle->tx_length = 0;
    
    switch (func_code)
    {
        case MODBUS_FUNC_READ_HOLDING_REGS:
        {
            /* 读保持寄存器 0x03 */
            uint16_t start_addr = ((uint16_t)rtu_handle->rx_buffer[2] << 8) | rtu_handle->rx_buffer[3];
            uint16_t reg_count = ((uint16_t)rtu_handle->rx_buffer[4] << 8) | rtu_handle->rx_buffer[5];
            
            if (reg_count > 0 && reg_count <= 125) /* Modbus限制最多125个寄存器 */
            {
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = rtu_handle->slave_addr;
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = func_code;
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_count * 2);
                
                for (uint16_t i = 0; i < reg_count; i++)
                {
                    uint16_t reg_value = modbus_iap_get_register(iap_handle, start_addr + i);
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_value >> 8);
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_value & 0xFF);
                }
                
                modbus_rtu_send_response(rtu_handle);
            }
            else
            {
                modbus_rtu_send_exception(rtu_handle, func_code, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            }
            break;
        }
        
        case MODBUS_FUNC_WRITE_SINGLE_REG:
        {
            /* 写单个寄存器 0x06 */
            uint16_t reg_addr = ((uint16_t)rtu_handle->rx_buffer[2] << 8) | rtu_handle->rx_buffer[3];
            uint16_t reg_value = ((uint16_t)rtu_handle->rx_buffer[4] << 8) | rtu_handle->rx_buffer[5];
            
            if (modbus_iap_set_register(iap_handle, reg_addr, reg_value) == MODBUS_RTU_OK)
            {
                /* 回显写入的值 */
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = rtu_handle->slave_addr;
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = func_code;
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_addr >> 8);
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_addr & 0xFF);
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_value >> 8);
                rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_value & 0xFF);
                
                modbus_rtu_send_response(rtu_handle);
            }
            else
            {
                modbus_rtu_send_exception(rtu_handle, func_code, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            }
            break;
        }
        
        case MODBUS_FUNC_WRITE_MULTIPLE_REGS:
        {
            /* 写多个寄存器 0x10 */
            uint16_t start_addr = ((uint16_t)rtu_handle->rx_buffer[2] << 8) | rtu_handle->rx_buffer[3];
            uint16_t reg_count = ((uint16_t)rtu_handle->rx_buffer[4] << 8) | rtu_handle->rx_buffer[5];
            uint8_t byte_count = rtu_handle->rx_buffer[6];
            
            if (reg_count > 0 && reg_count <= 123 && 
                byte_count == reg_count * 2 &&
                rtu_handle->rx_length >= (7 + byte_count + 2)) /* 地址+功能码+起始地址+数量+字节数+数据+CRC */
            {
                uint16_t values[123];
                
                /* 解析寄存器值 */
                for (uint16_t i = 0; i < reg_count; i++)
                {
                    uint8_t idx = 7 + i * 2;
                    values[i] = ((uint16_t)rtu_handle->rx_buffer[idx] << 8) | 
                               rtu_handle->rx_buffer[idx + 1];
                }
                
                if (modbus_iap_write_data_registers(iap_handle, start_addr, values, reg_count) == MODBUS_RTU_OK)
                {
                    /* 回显写入的地址和数量 */
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = rtu_handle->slave_addr;
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = func_code;
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(start_addr >> 8);
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(start_addr & 0xFF);
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_count >> 8);
                    rtu_handle->tx_buffer[rtu_handle->tx_length++] = (uint8_t)(reg_count & 0xFF);
                    
                    modbus_rtu_send_response(rtu_handle);
                }
                else
                {
                    modbus_rtu_send_exception(rtu_handle, func_code, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                }
            }
            else
            {
                modbus_rtu_send_exception(rtu_handle, func_code, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            }
            break;
        }
        
        default:
            modbus_rtu_send_exception(rtu_handle, func_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }
}

