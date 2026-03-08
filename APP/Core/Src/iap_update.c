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

/* CRC16查找表（Ymodem/Xmodem CRC-16-CCITT，多项式 0x1021） */
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

/* IAP上下文 */
static iap_update_context_t s_iap_ctx = {0};

/* FatFs对象 */
static FATFS s_fs;
static bool s_fs_mounted = false;

/* 数据缓冲区（4字节对齐） */
static __attribute__((aligned(4))) uint8_t s_buffer[IAP_BUFFER_SIZE];

/*===========================================================================*/
/* 内部函数声明 */
static bool iap_sd_init(void);

/**
 * @brief 初始化SD卡并挂载文件系统（供main.c调用）
 */
bool iap_sd_init_and_mount(void)
{
    return iap_sd_init();
}

/**
 * @brief SD卡自检：创建测试文件、写入、读回比对
 */
bool iap_sd_selftest(void)
{
    FIL file;
    FRESULT res;
    UINT bw, br;
    const char* test_path = "0:sd_test.txt";
    const char* write_data = "SD Card Self-Test OK\r\n";
    char read_buffer[64];
    bool result = false;

    /* 创建测试文件 */
    res = f_open(&file, test_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return false;
    }

    /* 写入测试数据 */
    res = f_write(&file, write_data, strlen(write_data), &bw);
    if (res != FR_OK || bw != strlen(write_data)) {
        f_close(&file);
        f_unlink(test_path);
        return false;
    }

    f_close(&file);

    /* 重新打开文件读取 */
    res = f_open(&file, test_path, FA_READ);
    if (res != FR_OK) {
        f_unlink(test_path);
        return false;
    }

    /* 读取数据 */
    res = f_read(&file, read_buffer, sizeof(read_buffer) - 1, &br);
    f_close(&file);

    if (res == FR_OK && br == strlen(write_data)) {
        read_buffer[br] = '\0';
        if (strcmp(read_buffer, write_data) == 0) {
            result = true;
        }
    }

    /* 删除测试文件 */
    f_unlink(test_path);

    return result;
}

/**
 * @brief 初始化SD卡并挂载文件系统（内部实现）
 */
static bool iap_sd_init(void)
{
    FRESULT res;
    sd_error_enum sd_status;
    sd_card_info_struct cardinfo;

    if (s_fs_mounted) {
        return true;
    }
    
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
void iap_sd_deinit(void)
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
    
    /* Step 5: 校验 - 先初始化SD卡，再打开文件 */
    s_iap_ctx.state = IAP_UPDATE_VERIFYING;

    /* 重新初始化SD卡（之前已卸载） */
    if (!iap_sd_init()) {
        s_iap_ctx.error_code = IAP_ERROR_SD_INIT;
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        return IAP_RESULT_SD_ERROR;
    }

    /* 打开固件文件进行校验 */
    res = f_open(&file, path, FA_READ);
    if (res != FR_OK) {
        s_iap_ctx.error_code = IAP_ERROR_FILE_OPEN;
        s_iap_ctx.state = IAP_UPDATE_ERROR;
        iap_sd_deinit();
        return IAP_RESULT_FILE_ERROR;
    }

    /* 校验FLASH内容 */
    const uint8_t *flash_ptr = (const uint8_t *)FLASH_APP_START;
    uint32_t verify_offset = 0;

    while (verify_offset < s_iap_ctx.firmware_size) {
        UINT chunk = (s_iap_ctx.firmware_size - verify_offset) > IAP_BUFFER_SIZE ?
                     IAP_BUFFER_SIZE : (s_iap_ctx.firmware_size - verify_offset);

        res = f_read(&file, s_buffer, chunk, &bytes_read);
        if (res != FR_OK || bytes_read != chunk) {
            s_iap_ctx.error_code = IAP_ERROR_FILE_READ;
            s_iap_ctx.state = IAP_UPDATE_ERROR;
            f_close(&file);
            iap_sd_deinit();
            return IAP_RESULT_FILE_ERROR;
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
    uint16_t crc = 0x0000;  /* Ymodem/Xmodem CRC-16-CCITT 初始值 */
    
    for (uint32_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ s_crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    
    return crc;
}

bool iap_verify_firmware(uint32_t start_addr, uint32_t size, uint16_t expected_crc)
{
    const uint8_t *ptr = (const uint8_t *)start_addr;
    uint16_t calc_crc = iap_crc16_calc(ptr, size);
    
    return (calc_crc == expected_crc);
}
