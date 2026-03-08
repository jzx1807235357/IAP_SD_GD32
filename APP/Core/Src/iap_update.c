/**
 * @file    iap_update.c
 * @brief   IAP固件升级模块实现 - SD卡读取与FLASH写入
 */

#include "iap_update.h"
#include "ff.h"
#include "sdcard.h"
#include <string.h>

/* 缓冲区大小（按字对齐，用于FLASH编程） */
#define IAP_BUFFER_SIZE     4096U

/* CRC16查找表（MODBUS CRC） */
static const uint16_t s_crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0140,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x01E0,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5140,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* IAP上下文 */
static iap_update_context_t s_iap_ctx = {0};

/* FatFs对象 */
static FATFS s_fs;
static bool s_fs_mounted = false;

/* 数据缓冲区（4字节对齐） */
static __attribute__((aligned(4))) uint8_t s_buffer[IAP_BUFFER_SIZE];

/*===========================================================================*/
/* 内部函数                                                                   */
/*===========================================================================*/

/**
 * @brief 初始化SD卡并挂载文件系统
 */
static bool iap_sd_init(void)
{
    FRESULT res;
    sd_error_enum sd_status;
    sd_card_info_struct cardinfo;
    
    /* 检查卡是否存在 */
    if (!sd_card_detect_pin_read()) {
        s_iap_ctx.error_code = IAP_ERROR_SD_NO_CARD;
        return false;
    }
    
    /* 初始化SD卡 */
    sd_status = sd_init();
    if (sd_status != SD_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    /* 获取卡信息 */
    sd_status = sd_card_information_get(&cardinfo);
    if (sd_status != SD_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    /* 选中卡片 */
    sd_status = sd_card_select_deselect(cardinfo.card_rca);
    if (sd_status != SD_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    /* 配置4位总线 */
    sd_status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
    if (sd_status != SD_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    /* 配置轮询模式（简单可靠） */
    sd_status = sd_transfer_mode_config(SD_POLLING_MODE);
    if (sd_status != SD_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    /* 挂载文件系统 */
    res = f_mount(&s_fs, "0:", 1);
    if (res != FR_OK) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        return false;
    }
    
    s_fs_mounted = true;
    return true;
}

/**
 * @brief 卸载文件系统
 */
static void iap_sd_deinit(void)
{
    if (s_fs_mounted) {
        f_mount(NULL, "0:", 0);
        s_fs_mounted = false;
    }
}

/*===========================================================================*/
/* 公共函数实现                                                               */
/*===========================================================================*/

bool iap_update_init(void)
{
    memset(&s_iap_ctx, 0, sizeof(s_iap_ctx));
    s_iap_ctx.state = IAP_UPDATE_IDLE;
    return true;
}

const iap_update_context_t* iap_update_get_context(void)
{
    return &s_iap_ctx;
}

bool iap_check_firmware_file(const char* path)
{
    FILINFO fno;
    FRESULT res;
    
    if (path == NULL) {
        path = IAP_FIRMWARE_PATH;
    }
    
    /* 检查文件是否存在 */
    res = f_stat(path, &fno);
    if (res != FR_OK) {
        return false;
    }
    
    /* 检查是否为文件（非目录） */
    if (fno.fattrib & AM_DIR) {
        return false;
    }
    
    return true;
}

uint32_t iap_get_firmware_size(const char* path)
{
    FILINFO fno;
    FRESULT res;
    
    if (path == NULL) {
        path = IAP_FIRMWARE_PATH;
    }
    
    res = f_stat(path, &fno);
    if (res != FR_OK) {
        return 0;
    }
    
    return (uint32_t)fno.fsize;
}

iap_status_t iap_perform_upgrade(const char* path)
{
    FIL file;
    FRESULT res;
    UINT bytes_read;
    uint32_t total_read = 0;
    flash_status_t flash_status;
    
    if (path == NULL) {
        path = IAP_FIRMWARE_PATH;
    }
    
    /* 初始化 */
    memset(&s_iap_ctx, 0, sizeof(s_iap_ctx));
    
    /* Step 1: 初始化SD卡 */
    s_iap_ctx.state = IAP_UPDATE_SD_INIT;
    if (!iap_sd_init()) {
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        return IAP_RESULT_SD_ERROR;
    }
    
    /* Step 2: 打开固件文件 */
    s_iap_ctx.state = IAP_UPDATE_FILE_OPEN;
    res = f_open(&file, path, FA_READ);
    if (res != FR_OK) {
        s_iap_ctx.error_code = IAP_ERROR_FILE_OPEN;
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        iap_sd_deinit();
        return IAP_RESULT_FILE_ERROR;
    }
    
    /* 获取文件大小 */
    s_iap_ctx.firmware_size = (uint32_t)f_size(&file);
    
    /* 检查文件大小 */
    if (s_iap_ctx.firmware_size == 0 || s_iap_ctx.firmware_size > FLASH_APP_MAX_SIZE_BYTES) {
        s_iap_ctx.error_code = (s_iap_ctx.firmware_size == 0) ? IAP_ERROR_FILE_SIZE : IAP_ERROR_FIRMWARE_TOO_LARGE;
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        f_close(&file);
        iap_sd_deinit();
        return IAP_RESULT_SIZE_ERROR;
    }
    
    /* Step 3: 擦除FLASH */
    s_iap_ctx.state = IAP_UPDATE_FLASH_ERASE;
    flash_status = flash_app_erase(s_iap_ctx.firmware_size);
    if (flash_status != FLASH_OP_SUCCESS) {
        s_iap_ctx.error_code = IAP_ERROR_FLASH_ERASE;
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        f_close(&file);
        iap_sd_deinit();
        return IAP_RESULT_FLASH_ERROR;
    }
    
    /* Step 4: 写入FLASH */
    s_iap_ctx.state = IAP_UPDATE_WRITING;
    s_iap_ctx.flash_address = FLASH_APP_START;
    
    while (total_read < s_iap_ctx.firmware_size) {
        /* 读取数据 */
        res = f_read(&file, s_buffer, IAP_BUFFER_SIZE, &bytes_read);
        if (res != FR_OK || bytes_read == 0) {
            s_iap_ctx.error_code = IAP_ERROR_FILE_READ;
            s_iap_ctx.state = IAP_UPDATE_ERROR;
            f_close(&file);
            iap_sd_deinit();
            return IAP_RESULT_FILE_ERROR;
        }
        
        /* 对齐到4字节（FLASH编程要求） */
        uint32_t aligned_len = (bytes_read + 3) & ~3;
        if (aligned_len > bytes_read) {
            /* 填充0xFF */
            memset(&s_buffer[bytes_read], 0xFF, aligned_len - bytes_read);
        }
        
        /* 写入FLASH */
        flash_status = flash_program_words(s_iap_ctx.flash_address, s_buffer, aligned_len);
        if (flash_status != FLASH_OP_SUCCESS) {
            s_iap_ctx.error_code = IAP_ERROR_FLASH_WRITE;
            s_iap_ctx.state = IAP_UPDATE_ERROR;
            f_close(&file);
            iap_sd_deinit();
            return IAP_RESULT_FLASH_ERROR;
        }
        
        /* 更新进度 */
        total_read += bytes_read;
        s_iap_ctx.written_size = total_read;
        s_iap_ctx.flash_address += aligned_len;
        s_iap_ctx.progress = (uint8_t)((total_read * 100) / s_iap_ctx.firmware_size);
    }
    
    /* 关闭文件 */
    f_close(&file);
    iap_sd_deinit();
    
    /* Step 5: 校验 */
    s_iap_ctx.state = IAP_UPDATE_VERIFYING;
    
    /* 校验FLASH内容（简单比较） */
    const uint8_t *flash_ptr = (const uint8_t *)FLASH_APP_START;
    res = f_open(&file, path, FA_READ);
    if (res == FR_OK) {
        iap_sd_init();
        uint32_t verify_offset = 0;
        
        while (verify_offset < s_iap_ctx.firmware_size) {
            UINT chunk = (s_iap_ctx.firmware_size - verify_offset) > IAP_BUFFER_SIZE ? 
                         IAP_BUFFER_SIZE : (s_iap_ctx.firmware_size - verify_offset);
            
            res = f_read(&file, s_buffer, chunk, &bytes_read);
            if (res != FR_OK || bytes_read != chunk) {
                break;
            }
            
            /* 比较 */
            if (memcmp(&flash_ptr[verify_offset], s_buffer, chunk) != 0) {
                s_iap_ctx.error_code = IAP_ERROR_FLASH_VERIFY;
                s_iap_ctx.state = IAP_UPDATE_ERROR;
                f_close(&file);
                iap_sd_deinit();
                return IAP_RESULT_FLASH_ERROR;
            }
            
            verify_offset += chunk;
        }
        
        f_close(&file);
        iap_sd_deinit();
    }
    
    /* 成功 */
    s_iap_ctx.state = IAP_UPDATE_SUCCESS;
    s_iap_ctx.progress = 100;
    
    return IAP_RESULT_OK;
}

void iap_trigger_bootloader(void)
{
    /* 设置IAP标志 */
    iap_flag_set();
    
    /* 等待写入完成 */
    for (volatile uint32_t i = 0; i < 100000; i++);
    
    /* 触发软件复位 */
    NVIC_SystemReset();
}

uint16_t iap_crc16_calc(const uint8_t* data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (uint32_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ s_crc16_table[(crc ^ data[i]) & 0xFF];
    }
    
    return crc;
}

bool iap_verify_firmware(uint32_t start_addr, uint32_t size, uint16_t expected_crc)
{
    const uint8_t *ptr = (const uint8_t *)start_addr;
    uint16_t calc_crc = iap_crc16_calc(ptr, size);
    
    return (calc_crc == expected_crc);
}
