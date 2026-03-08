/**
 * @file    ymodem.c
 * @brief   Ymodem协议接收模块实现（BootLoader版本）
 * 
 * 从SD卡文件读取固件，解析Ymodem格式并写入Flash
 */

#include "ymodem.h"
#include "flash_driver.h"
#include "ff.h"
#include <string.h>

/* 计算CRC16 (CCITT标准，运行时计算) */
static uint16_t crc16_calc(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    uint16_t i;
    while (len--) {
        crc ^= ((uint16_t)*data++) << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* 初始化Ymodem句柄 */
void ymodem_init(ymodem_handle_t *handle)
{
    memset(handle, 0, sizeof(ymodem_handle_t));
    handle->state = YMODEM_STATE_IDLE;
    handle->expected_seq = 0;
    handle->flash_addr = APPLICATION_START_ADDRESS;
}

/* 解析单个数据包 */
ymodem_parse_result_t ymodem_parse_packet(ymodem_handle_t *handle, const uint8_t *data, uint16_t len)
{
    uint8_t  packet_type;
    uint8_t  seq_num;
    uint8_t  seq_complement;
    uint16_t packet_crc;
    uint16_t calc_crc;
    uint16_t data_len;
    const uint8_t *packet_data;
    
    if (len < 6) {
        return YMODEM_PARSE_ERROR;
    }
    
    packet_type = data[0];
    
    /* 检查包类型 */
    if (packet_type == YMODEM_EOT) {
        return YMODEM_PARSE_EOT;
    }
    
    if (packet_type == YMODEM_CAN) {
        handle->error = YMODEM_ERROR_ABORT;
        return YMODEM_PARSE_ERROR;
    }
    
    if (packet_type != YMODEM_SOH && packet_type != YMODEM_STX) {
        return YMODEM_PARSE_ERROR;
    }
    
    /* 解析包序号 */
    seq_num = data[1];
    seq_complement = data[2];
    
    if ((seq_num + seq_complement) != 0xFF) {
        handle->error = YMODEM_ERROR_SEQUENCE;
        return YMODEM_PARSE_ERROR;
    }
    
    /* 确定数据长度 */
    if (packet_type == YMODEM_SOH) {
        data_len = 128;
    } else {
        data_len = 1024;
    }
    
    /* 检查总长度 */
    if (len < (data_len + 5)) {
        return YMODEM_PARSE_ERROR;
    }
    
    packet_data = &data[3];
    packet_crc = (data[data_len + 3] << 8) | data[data_len + 4];
    
    /* 验证CRC */
    calc_crc = crc16_calc(packet_data, data_len);
    if (calc_crc != packet_crc) {
        handle->error = YMODEM_ERROR_CRC;
        return YMODEM_PARSE_ERROR;
    }
    
    /* 第一个包（序号0）包含文件信息 */
    if (seq_num == 0) {
        /* 解析文件名和大小 */
        if (packet_data[0] != 0) {
            /* 提取文件名 */
            uint8_t i = 0;
            while (packet_data[i] != 0 && i < sizeof(handle->file_name) - 1) {
                handle->file_name[i] = packet_data[i];
                i++;
            }
            handle->file_name[i] = '\0';
            
            /* 提取文件大小 */
            i++;
            handle->total_size = 0;
            while (packet_data[i] >= '0' && packet_data[i] <= '9') {
                handle->total_size = handle->total_size * 10 + (packet_data[i] - '0');
                i++;
            }
        }
        
        handle->expected_seq = 1;
        handle->received_size = 0;
        handle->flash_addr = APPLICATION_START_ADDRESS;
        
        return YMODEM_PARSE_OK;
    }
    
    /* 检查包序号 */
    if (seq_num != handle->expected_seq) {
        if (seq_num == (handle->expected_seq - 1)) {
            /* 重复包，跳过 */
            return YMODEM_PARSE_SKIP;
        }
        handle->error = YMODEM_ERROR_SEQUENCE;
        return YMODEM_PARSE_ERROR;
    }
    
    /* 写入Flash */
    uint32_t write_size = data_len;
    if (handle->received_size + write_size > handle->total_size) {
        write_size = handle->total_size - handle->received_size;
    }
    
    /* 对齐到4字节 */
    write_size = (write_size + 3) & ~3;
    
    if (write_size > 0) {
        flash_status_t flash_status = flash_program_words(handle->flash_addr, packet_data, write_size);
        if (flash_status != FLASH_OP_SUCCESS) {
            handle->error = YMODEM_ERROR_FLASH;
            return YMODEM_PARSE_ERROR;
        }
        
        handle->flash_addr += write_size;
        handle->received_size += write_size;
    }
    
    handle->expected_seq++;
    
    /* 检查是否完成 */
    if (handle->received_size >= handle->total_size) {
        return YMODEM_PARSE_DONE;
    }
    
    return YMODEM_PARSE_OK;
}

/* 从文件读取并解析Ymodem格式固件 */
int ymodem_receive_from_file(ymodem_handle_t *handle, const char *file_path)
{
    FIL file;
    FRESULT fres;
    UINT br;
    uint8_t read_buf[1030];  /* 最大包大小 + 冗余 */
    ymodem_parse_result_t result;
    uint32_t bytes_read = 0;
    bool eot_received = false;
    
    /* 初始化 */
    ymodem_init(handle);
    
    /* 打开文件 */
    fres = f_open(&file, file_path, FA_READ);
    if (fres != FR_OK) {
        handle->error = YMODEM_ERROR_FILE;
        return -1;
    }
    
    /* 读取并解析文件 */
    while (!eot_received) {
        /* 读取包头 */
        fres = f_read(&file, read_buf, 1, &br);
        if (fres != FR_OK || br == 0) {
            break;
        }
        
        uint8_t packet_type = read_buf[0];
        
        /* 检查EOT */
        if (packet_type == YMODEM_EOT) {
            eot_received = true;
            result = YMODEM_PARSE_EOT;
            break;
        }
        
        /* 检查CAN */
        if (packet_type == YMODEM_CAN) {
            handle->error = YMODEM_ERROR_ABORT;
            f_close(&file);
            return -1;
        }
        
        /* 检查包类型 */
        if (packet_type != YMODEM_SOH && packet_type != YMODEM_STX) {
            continue;  /* 跳过无效数据 */
        }
        
        /* 确定包大小 */
        uint16_t data_len = (packet_type == YMODEM_SOH) ? 128 : 1024;
        uint16_t packet_size = data_len + 4;  /* seq + ~seq + data + crc(2) */
        
        /* 读取剩余包数据 */
        fres = f_read(&file, &read_buf[1], packet_size, &br);
        if (fres != FR_OK || br != packet_size) {
            handle->error = YMODEM_ERROR_TIMEOUT;
            f_close(&file);
            return -1;
        }
        
        /* 解析数据包 */
        result = ymodem_parse_packet(handle, read_buf, packet_size + 1);
        
        if (result == YMODEM_PARSE_ERROR) {
            f_close(&file);
            return -1;
        }
        
        if (result == YMODEM_PARSE_DONE) {
            break;
        }
    }
    
    f_close(&file);
    
    if (result == YMODEM_PARSE_DONE || result == YMODEM_PARSE_EOT) {
        handle->state = YMODEM_STATE_DONE;
        return 0;
    }
    
    return -1;
}

/* 获取接收进度百分比 */
uint8_t ymodem_get_progress(ymodem_handle_t *handle)
{
    if (handle->total_size == 0) {
        return 0;
    }
    uint32_t progress = (handle->received_size * 100) / handle->total_size;
    if (progress > 100) {
        progress = 100;
    }
    return (uint8_t)progress;
}
