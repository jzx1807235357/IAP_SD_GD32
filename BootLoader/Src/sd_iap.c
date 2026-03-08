/**
 * @file    sd_iap.c
 * @brief   SD卡IAP升级模块实现
 * 
 * 基于SD卡和Ymodem协议的IAP在线升级功能
 */

#include "sd_iap.h"
#include "sdcard.h"
#include "flash_driver.h"
#include "ff.h"
#include <string.h>

/* 默认固件文件路径 */
#define DEFAULT_FIRMWARE_PATH   "0:firmware.bin"

/* FatFs工作区 */
static FATFS fatfs;

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

/* 执行IAP升级 */
int sd_iap_upgrade(sd_iap_handle_t *handle, const sd_iap_config_t *config)
{
    const char *firmware_path;
    int result;
    
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
    
    /* 检查文件是否存在 */
    if (!sd_iap_file_exists(firmware_path)) {
        handle->status = SD_IAP_FILE_NOT_FOUND;
        return -4;
    }
    
    /* 擦除APP区域Flash */
    flash_status_t flash_status = flash_app_erase(APPLICATION_SIZE);
    if (flash_status != FLASH_OP_SUCCESS) {
        handle->status = SD_IAP_FLASH_ERROR;
        handle->error_code = flash_status;
        return -5;
    }
    
    /* 更新状态 */
    handle->status = SD_IAP_RECEIVING;
    
    /* 执行Ymodem接收 */
    result = ymodem_receive_from_file(&handle->ymodem, firmware_path);
    if (result != 0) {
        switch (handle->ymodem.error) {
            case YMODEM_ERROR_CRC:
                handle->status = SD_IAP_CRC_ERROR;
                break;
            case YMODEM_ERROR_FLASH:
                handle->status = SD_IAP_FLASH_ERROR;
                break;
            default:
                handle->status = SD_IAP_FLASH_ERROR;
                break;
        }
        handle->error_code = handle->ymodem.error;
        return -6;
    }
    
    /* 更新信息 */
    handle->firmware_size = handle->ymodem.total_size;
    handle->bytes_written = handle->ymodem.received_size;
    strncpy(handle->file_name, handle->ymodem.file_name, sizeof(handle->file_name) - 1);
    
    /* 验证固件 */
    if (!sd_iap_verify(handle)) {
        handle->status = SD_IAP_CRC_ERROR;
        return -7;
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
    return ymodem_get_progress(&handle->ymodem);
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
