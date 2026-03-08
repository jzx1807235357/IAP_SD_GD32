/**
 * @file    sd_iap.c
 * @brief   SD卡IAP升级模块实现
 * 
 * 直接从SD卡读取原始binary固件并写入Flash
 */

#include "sd_iap.h"
#include "sdcard.h"
#include "flash_driver.h"
#include "ff.h"
#include <string.h>

/* 默认固件文件路径 */
#define DEFAULT_FIRMWARE_PATH   "0:firmware.bin"

/* 读取缓冲区大小 */
#define READ_BUFFER_SIZE        4096

/* FatFs工作区 */
static FATFS fatfs;

/* CRC16-CCITT 查找表 (多项式 0x1021) */
static const uint16_t s_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/* 计算CRC16-CCITT */
static uint16_t crc16_calc(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0x0000;  /* 初始值 */
    
    while (len--) {
        crc = (crc << 8) ^ s_crc16_table[((crc >> 8) ^ *data++) & 0xFF];
    }
    
    return crc;
}

/* 初始化SD-IAP模块 */
bool sd_iap_init(void)
{
    sd_card_info_struct card_info;
    
    /* 检查SD卡是否存在 */
    if (!sd_card_detect_pin_read()) {
        return false;
    }
    
    /* 初始化SD卡 */
    sd_error_enum status = sd_init();
    if (status != SD_OK) {
        return false;
    }
    
    /* 获取SD卡信息 */
    status = sd_card_information_get(&card_info);
    if (status != SD_OK) {
        return false;
    }
    
    /* 选择SD卡（CMD7）- 必需步骤 */
    status = sd_card_select_deselect(card_info.card_rca);
    if (status != SD_OK) {
        return false;
    }
    
    return true;
}

/* 检查SD卡是否存在 */
bool sd_iap_card_detected(void)
{
    return sd_card_detect_pin_read();
}

/* 挂载SD卡文件系统 */
int sd_iap_mount(void)
{
    FRESULT fres;
    
    /* 检查SD卡 */
    if (!sd_card_detect_pin_read()) {
        return -1;
    }
    
    /* 挂载文件系统 */
    fres = f_mount(&fatfs, "0:", 1);
    if (fres != FR_OK) {
        return -2;
    }
    
    return 0;
}

/* 检查固件文件是否存在 */
bool sd_iap_file_exists(const char *path)
{
    FIL file;
    FRESULT fres;
    
    if (path == NULL) {
        path = DEFAULT_FIRMWARE_PATH;
    }
    
    /* 用f_open检查文件是否存在 */
    fres = f_open(&file, path, FA_READ);
    if (fres == FR_OK) {
        f_close(&file);
        return true;
    }
    return false;
}

/* 执行IAP升级 - 原始binary格式 */
int sd_iap_upgrade(sd_iap_handle_t *handle, const sd_iap_config_t *config)
{
    const char *firmware_path;
    FIL file;
    FRESULT fres;
    UINT br;
    static uint8_t read_buf[READ_BUFFER_SIZE];  /* 读取缓冲区 */
    uint32_t flash_addr;
    uint32_t total_read = 0;
    uint16_t calc_crc = 0;
    flash_status_t flash_status;
    
    if (handle == NULL) {
        return -1;
    }
    
    /* 初始化句柄 */
    memset(handle, 0, sizeof(sd_iap_handle_t));
    handle->status = SD_IAP_IDLE;
    
    /* 检查SD卡 */
    if (!sd_card_detect_pin_read()) {
        handle->status = SD_IAP_NO_CARD;
        return -2;
    }
    
    /* 挂载文件系统 */
    if (sd_iap_mount() != 0) {
        handle->status = SD_IAP_MOUNT_FAILED;
        return -3;
    }
    
    /* 确定固件路径 */
    firmware_path = (config && config->firmware_path) ? config->firmware_path : DEFAULT_FIRMWARE_PATH;
    
    /* 打开固件文件 */
    fres = f_open(&file, firmware_path, FA_READ);
    if (fres != FR_OK) {
        handle->status = SD_IAP_FILE_NOT_FOUND;
        return -4;
    }
    
    /* 获取文件大小 */
    handle->firmware_size = f_size(&file);
    if (handle->firmware_size == 0 || handle->firmware_size > APPLICATION_SIZE) {
        f_close(&file);
        handle->status = SD_IAP_FILE_NOT_FOUND;
        return -5;
    }
    
    /* 检查是否有期望大小限制 */
    if (config && config->expected_size > 0 && handle->firmware_size != config->expected_size) {
        f_close(&file);
        handle->status = SD_IAP_FILE_NOT_FOUND;
        return -6;
    }
    
    /* 擦除APP区域Flash */
    flash_status = flash_app_erase(APPLICATION_SIZE);
    if (flash_status != FLASH_OP_SUCCESS) {
        f_close(&file);
        handle->status = SD_IAP_FLASH_ERROR;
        handle->error_code = flash_status;
        return -7;
    }
    
    /* 更新状态 */
    handle->status = SD_IAP_RECEIVING;
    flash_addr = APPLICATION_START_ADDRESS;
    
    /* 读取文件并写入Flash */
    while (total_read < handle->firmware_size) {
        /* 计算本次读取大小 */
        uint32_t read_size = READ_BUFFER_SIZE;
        if (total_read + read_size > handle->firmware_size) {
            read_size = handle->firmware_size - total_read;
        }
        
        /* 读取数据 */
        fres = f_read(&file, read_buf, read_size, &br);
        if (fres != FR_OK || br != read_size) {
            f_close(&file);
            handle->status = SD_IAP_FLASH_ERROR;
            handle->error_code = fres;
            return -8;
        }
        
        /* 计算CRC */
        calc_crc = crc16_calc(read_buf, read_size);
        
        /* 对齐到4字节边界 */
        uint32_t write_size = (read_size + 3) & ~3;
        
        /* 如果不是4字节对齐，填充0xFF */
        if (write_size > read_size) {
            for (uint32_t i = read_size; i < write_size; i++) {
                read_buf[i] = 0xFF;
            }
        }
        
        /* 写入Flash */
        flash_status = flash_program_words(flash_addr, read_buf, write_size);
        if (flash_status != FLASH_OP_SUCCESS) {
            f_close(&file);
            handle->status = SD_IAP_FLASH_ERROR;
            handle->error_code = flash_status;
            return -9;
        }
        
        flash_addr += write_size;
        total_read += read_size;
        handle->bytes_written = total_read;
        
        /* 更新进度 */
        handle->progress = (uint8_t)((total_read * 100) / handle->firmware_size);
    }
    
    /* 关闭文件 */
    f_close(&file);
    
    /* 验证固件 */
    if (!sd_iap_verify(handle)) {
        handle->status = SD_IAP_CRC_ERROR;
        return -10;
    }
    
    /* 检查CRC（如果配置了期望CRC） */
    if (config && config->expected_crc > 0 && calc_crc != config->expected_crc) {
        handle->status = SD_IAP_CRC_ERROR;
        return -11;
    }
    
    handle->status = SD_IAP_SUCCESS;
    return 0;
}

/* 验证固件完整性 */
bool sd_iap_verify(sd_iap_handle_t *handle)
{
    uint32_t addr;
    uint32_t size;
    
    if (handle == NULL || handle->firmware_size == 0) {
        return false;
    }
    
    /* 检查APP起始地址是否有效 */
    addr = APPLICATION_START_ADDRESS;
    
    /* 检查第一个字是否为有效栈指针 (应该在RAM范围内) */
    uint32_t stack_ptr = *((volatile uint32_t*)addr);
    if ((stack_ptr < 0x20000000) || (stack_ptr > 0x20030000)) {
        return false;
    }
    
    /* 检查第二个字是否为有效复位向量 (应该在Flash范围内) */
    uint32_t reset_handler = *((volatile uint32_t*)(addr + 4));
    if ((reset_handler < APPLICATION_START_ADDRESS) || 
        (reset_handler > (APPLICATION_START_ADDRESS + APPLICATION_SIZE))) {
        return false;
    }
    
    /* 空白检查 - 确保数据已正确写入 */
    size = handle->firmware_size;
    if (flash_verify_blank(APPLICATION_START_ADDRESS, size) != FLASH_OP_SUCCESS) {
        /* 非空白表示有数据写入 */
        return true;
    }
    
    return false;
}

/* 获取升级进度 */
uint8_t sd_iap_get_progress(sd_iap_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    return handle->progress;
}

/* 获取IAP状态字符串 */
const char* sd_iap_get_status_string(sd_iap_status_t status)
{
    switch (status) {
        case SD_IAP_IDLE:           return "Idle";
        case SD_IAP_NO_CARD:        return "No SD Card";
        case SD_IAP_MOUNT_FAILED:   return "Mount Failed";
        case SD_IAP_FILE_NOT_FOUND: return "File Not Found";
        case SD_IAP_RECEIVING:      return "Receiving";
        case SD_IAP_FLASH_ERROR:    return "Flash Error";
        case SD_IAP_CRC_ERROR:      return "CRC Error";
        case SD_IAP_SUCCESS:        return "Success";
        case SD_IAP_READY_JUMP:     return "Ready to Jump";
        default:                    return "Unknown";
    }
}
