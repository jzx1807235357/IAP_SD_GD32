/*!
    \file    sd_logger.c
    \brief   SD卡日志记录模块实现（大容量数据存储）
    
    功能：日志记录、历史数据、校准记录
    特点：大容量、可扩展、易导出分析
    
    \version 2025-01-26, V1.0.0, SD Logger Implementation
*/

#include "sd_logger.h"
#include "ff.h"
#include "app_rtc.h"
#include "sun_position.h"  /* 引入统一时间接口 */
#include "sdcard.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"  /* 用于 pvPortMalloc/vPortFree */
#include "host_comm.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* 文件路径定义 */
#define SD_DIR_ROOT            "/solar_tracking"
#define SD_DIR_TRACKING_STATE  "/solar_tracking/tracking_state"
#define SD_FILE_CALIB_CSV      "/solar_tracking/calib_history.csv"
#define SD_FILE_LOG            "/solar_tracking/system.log"
#define SD_FILE_TRACKING_STATE "/solar_tracking/tracking_state.csv"  /* 保留以兼容 */

/* FatFs对象 */
static FATFS g_fs;
static FIL g_file;
static SemaphoreHandle_t g_sd_mutex = NULL;

/* 读取tracking日志文件的状态 */
static FIL s_tracking_read_file;
static bool s_tracking_file_opened = false;

/* 状态标志 */
static volatile bool g_sd_mounted = false;
static volatile bool g_sd_initialized = false;

/* 热插拔事件标志 */
static volatile bool g_sd_card_insert_event = false;
static volatile bool g_sd_card_remove_event = false;
static volatile bool g_sd_rebuild_request = false;
static volatile bool g_sd_rebuild_in_progress = false;

/* 函数声明 */
static bool sd_create_directory(void);
static void sd_lock(void);
static void sd_unlock(void);
static bool sd_init_internal(void);  /* 内部初始化函数 */
static bool sd_is_valid_date_csv(const char* fname, uint16_t* year, uint8_t* month, uint8_t* day);
static bool sd_process_month_dir(FIL* index_file, const char* year_path, const char* month_name, int* p_file_count);

/* FatFs错误处理辅助函数 */
static bool sd_is_fatfs_severe(FRESULT res);
static void sd_log_and_handle_fatfs_error(const char* tag, const char* path, FRESULT res);

/*===========================================================================*/
/* 初始化和挂载                                                               */
/*===========================================================================*/

/*!
    \brief      内部SD卡初始化实现（可重复调用）
    \param[in]  none
    \param[out] none
    \retval     true - 成功, false - 失败
*/
static bool sd_init_internal(void)
{
    FRESULT res;
    sd_error_enum sd_status;
    sd_card_info_struct cardinfo;
    
    /* 先检查物理卡是否存在 */
    if (!sd_card_detect_pin_read()) {
        return false;  /* 无卡 */
    }
    
    /* !!! 重要：先初始化SD卡 !!! */
    sd_status = sd_init();
    if (sd_status != SD_OK) {
        return false;
    }
    
    /* 获取卡信息 */
    sd_status = sd_card_information_get(&cardinfo);
    if (sd_status != SD_OK) {
        return false;
    }
    
    /* 选中卡片 */
    sd_status = sd_card_select_deselect(cardinfo.card_rca);
    if (sd_status != SD_OK) {
        return false;
    }
    
    /* 配置4位总线 */
    sd_status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
    if (sd_status != SD_OK) {
        return false;
    }
    
    /* 配置DMA模式 - 减少CPU占用 */
    sd_status = sd_transfer_mode_config(SD_DMA_MODE);
    if (sd_status != SD_OK) {
        return false;
    }
    
    /* 启用SDIO中断 - DMA模式必需 */
    nvic_irq_enable(SDIO_IRQn, 2, 0);
    
    /* 挂载文件系统 */
    res = f_mount(&g_fs, "0:", 1);
    
    if (res != FR_OK) {
        /* 挂载失败，检查错误类型 */
        if (res == FR_NO_FILESYSTEM) {
            /* 没有文件系统，需要格式化 */
            BYTE work[FF_MAX_SS];
            res = f_mkfs("0:", FM_FAT32, 0, work, sizeof(work));
            
            if (res != FR_OK) {
                return false;
            }
            
            /* 重新挂载 */
            res = f_mount(&g_fs, "0:", 1);
            
            if (res != FR_OK) {
                return false;
            }
        } else {
            /* 其他错误（如FR_NOT_READY），不进行格式化 */
            return false;
        }
    }
    
    g_sd_mounted = true;
    
    /* 创建目录结构 */
    if (!sd_create_directory()) {
        return false;
    }
    
    /* 注意：索引重建不再自动执行，由上位机命令触发
       调用 sd_logger_rebuild_index() 接口来重建 */
    
    return true;
}/*!
    \brief      首次初始化SD卡日志系统
    \param[in]  none
    \param[out] none
    \retval     true - 成功, false - 失败
*/
bool sd_logger_init(void)
{
    if (g_sd_initialized) {
        return g_sd_mounted;
    }
    
    /* 创建互斥锁 */
    if (g_sd_mutex == NULL) {
        g_sd_mutex = xSemaphoreCreateMutex();
        if (g_sd_mutex == NULL) {
            return false;
        }
    }
    
    /* 配置卡检测引脚的外部中断 */
    sd_card_detect_exti_init();
    
    /* 尝试初始化SD卡（如果当前有卡） */
    if (sd_init_internal()) {
        g_sd_initialized = true;
        return true;
    }
    
    /* 即使当前无卡，也标记为已初始化（等待热插拔） */
    g_sd_initialized = true;
    return false;  /* 返回false表示当前无卡 */
}

/*!
    \brief      重新初始化SD卡（热插拔后调用）
    \param[in]  none
    \param[out] none
    \retval     true - 成功, false - 失败
*/
bool sd_logger_reinit(void)
{
    sd_lock();
    
    /* 先卸载旧的 */
    if (g_sd_mounted) {
        f_mount(NULL, "0:", 0);
        g_sd_mounted = false;
    }
    
    /* 重新初始化 */
    bool result = sd_init_internal();
    
    sd_unlock();
    
    return result;
}

void sd_logger_deinit(void)
{
    if (g_sd_mounted) {
        f_mount(NULL, "0:", 0);
        g_sd_mounted = false;
    }
    g_sd_initialized = false;
}

bool sd_logger_is_mounted(void)
{
    return g_sd_mounted;
}

/*===========================================================================*/
/* 热插拔回调函数                                                             */
/*===========================================================================*/

/*!
    \brief      SD卡插入中断回调（在EXTI中断中调用）
    \param[in]  none
    \param[out] none
    \retval     none
*/
void sd_logger_card_inserted_callback(void)
{
    /* 在中断中只设置标志位，实际初始化在任务中完成 */
    g_sd_card_insert_event = true;
}

/*!
    \brief      SD卡拔出中断回调（在EXTI中断中调用）
    \param[in]  none
    \param[out] none
    \retval     none
*/
void sd_logger_card_removed_callback(void)
{
    /* 在中断中只设置标志位，实际卸载在任务中完成 */
    g_sd_card_remove_event = true;
}

/*!
    \brief      处理热插拔事件（在任务中周期调用）
    \param[in]  none
    \param[out] none
    \retval     none
    \note       建议在低优先级任务中每100-500ms调用一次
*/
void sd_logger_process_hotplug_events(void)
{
    bool card_present = sd_card_detect_pin_read();

    /* 有中断事件就清一次标志，后续完全根据物理电平判断 */
    if (g_sd_card_insert_event || g_sd_card_remove_event) {
        g_sd_card_insert_event = false;
        g_sd_card_remove_event = false;
    }

    /* 物理上有卡，但逻辑上未挂载 -> 重新初始化并挂载 */
    if (card_present && !g_sd_mounted) {
        if (sd_logger_reinit()) {
            sd_logger_printf("=== SD Card Inserted & Mounted ===");
        }
    }
    /* 物理上无卡，但逻辑上仍认为已挂载 -> 卸载 */
    else if (!card_present && g_sd_mounted) {
        sd_logger_printf("=== SD Card Removed ===");
        sd_logger_deinit();
    }
}

/*===========================================================================*/
/* 日志记录                                                                   */
/*===========================================================================*/
void sd_logger_printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    UINT bw;
    rtc_datetime_t local_time;
    
    if (!g_sd_mounted || !format) {
        return;
    }
    
    sd_lock();
    
    /* 打开日志文件（追加模式） */
    if (f_open(&g_file, SD_FILE_LOG, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        /* 写入时间戳（本地时间） */
        if (time_get_local_datetime(&local_time)) {
            snprintf(buffer, sizeof(buffer), "[%04d-%02d-%02d %02d:%02d:%02d] ",
                     local_time.year, local_time.month, local_time.day,
                     local_time.hour, local_time.minute, local_time.second);
            f_write(&g_file, buffer, strlen(buffer), &bw);
        }
        
        /* 写入日志内容 */
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        f_write(&g_file, buffer, strlen(buffer), &bw);
        f_write(&g_file, "\r\n", 2, &bw);
        
        f_close(&g_file);
    }
    
    sd_unlock();
}

void sd_logger_error(uint16_t error_code, const char* message)
{
    sd_logger_printf("[ERROR:0x%04X] %s", error_code, message ? message : "");
}

void sd_logger_log_calibration(const calibration_history_t* data)
{
    char buffer[512];
    UINT bw;
    FRESULT res;
    FILINFO fno;
    bool need_header = false;
    
    if (!g_sd_mounted || !data) {
        return;
    }
    
    sd_lock();
    
    /* 检查CSV文件是否存在 */
    if (f_stat(SD_FILE_CALIB_CSV, &fno) != FR_OK) {
        need_header = true;
    }
    
    /* 打开CSV文件 */
    res = f_open(&g_file, SD_FILE_CALIB_CSV, FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK) {
        /* 写入表头（首次） */
        if (need_header) {
            snprintf(buffer, sizeof(buffer), 
                     "DateTime,AzOffset,ElOffset,SunAz,SunEl,Lat,Lon,TotalLight,Ex,Ey,Count\r\n");
            f_write(&g_file, buffer, strlen(buffer), &bw);
        }
        
        /* 写入数据 */
        snprintf(buffer, sizeof(buffer),
                 "%04d-%02d-%02d %02d:%02d:%02d,%.3f,%.3f,%.2f,%.2f,%.6f,%.6f,%.3f,%.4f,%.4f,%lu\r\n",
                 data->year, data->month, data->day,
                 data->hour, data->minute, data->second,
                 data->azimuth_offset, data->elevation_offset,
                 data->sun_azimuth, data->sun_elevation,
                 data->latitude, data->longitude,
                 data->total_light, data->ex, data->ey,
                 data->calib_count);
        
        f_write(&g_file, buffer, strlen(buffer), &bw);
        f_close(&g_file);
    }
    
    sd_unlock();
}

void sd_logger_log_tracking(float ex, float ey, float az, float el)
{
    sd_logger_printf("[TRACKING] ex=%.4f, ey=%.4f, az=%.2f, el=%.2f", ex, ey, az, el);
}

/*===========================================================================*/
/* 工具函数                                                                   */
/*===========================================================================*/
void sd_logger_flush(void)
{
    /* FatFs自动同步，无需手动flush */
}

/*===========================================================================*/
/* 内部函数                                                                   */
/*===========================================================================*/

/*!
    \brief      判断是否为需要重新挂载的严重FatFs错误
    \param[in]  res - FatFs错误码
    \param[out] none
    \retval     true - 严重错误, false - 非严重错误
*/
static bool sd_is_fatfs_severe(FRESULT res)
{
    return (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR);
}

/*!
    \brief      在system.log中记录FatFs错误并在严重错误时自动deinit
    \param[in]  tag - 操作标签（用于日志）
    \param[in]  path - 文件路径（可为NULL）
    \param[in]  res - FatFs错误码
    \param[out] none
    \retval     none
    \note       调用本函数前必须已经 sd_unlock()，以避免与sd_logger_printf的互斥锁冲突
*/
static void sd_log_and_handle_fatfs_error(const char* tag, const char* path, FRESULT res)
{
    const char* p = (path != NULL) ? path : "-";

    /* 尝试写入系统日志（如果此时文件系统仍可用） */
    sd_logger_printf("[SDLOG] %s failed, res=%d, path=%s", tag, (int)res, p);

    if (sd_is_fatfs_severe(res)) {
        sd_logger_printf("[SDLOG] severe error, deinit SD");
        sd_logger_deinit();
    }
}

static bool sd_create_directory(void)
{
    FRESULT res;
    
    /* 创建根目录 */
    res = f_mkdir(SD_DIR_ROOT);
    if (res != FR_OK && res != FR_EXIST) {
        return false;
    }
    
    /* 创建tracking_state目录 */
    res = f_mkdir(SD_DIR_TRACKING_STATE);
    if (res != FR_OK && res != FR_EXIST) {
        return false;
    }
    
    return true;
}

static void sd_lock(void)
{
    if (g_sd_mutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }
}

static void sd_unlock(void)
{
    if (g_sd_mutex != NULL) {
        xSemaphoreGive(g_sd_mutex);
    }
}

/*===========================================================================*/
/* 诊断日志记录                                                               */
/*===========================================================================*/
void sd_logger_log_tracking_state(const tracking_state_log_t* st)
{
    char buffer[512];
    char path_dir_year[64];
    char path_dir_month[80];
    char path_file[96];
    UINT bw;
    FRESULT res;
    bool need_header = false;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    
    /* 重建索引期间跳过日志写入，避免阻塞 HostComm_Task 导致 Modbus 通信超时 */
    if (!g_sd_mounted || !st || g_sd_rebuild_in_progress) {
        return;
    }
    
    sd_lock();
    
    /* 从时间戳中提取年月日 */
    year  = st->time_local.year;
    month = st->time_local.month;
    day   = st->time_local.day;
    
    /* 构造年目录路径 */
    snprintf(path_dir_year, sizeof(path_dir_year), "%s/%04u", SD_DIR_TRACKING_STATE, year);
    
    /* 构造月目录路径 */
    snprintf(path_dir_month, sizeof(path_dir_month), "%s/%02u", path_dir_year, month);
    
    /* 构造文件路径 */
    snprintf(path_file, sizeof(path_file), "%s/%04u%02u%02u.csv", path_dir_month, year, month, day);
    
    /* 确保年目录存在 */
    res = f_mkdir(path_dir_year);
    if (res != FR_OK && res != FR_EXIST) {
        sd_unlock();
        sd_log_and_handle_fatfs_error("mkdir tracking year dir", path_dir_year, res);
        return;
    }
    
    /* 确保月目录存在 */
    res = f_mkdir(path_dir_month);
    if (res != FR_OK && res != FR_EXIST) {
        sd_unlock();
        sd_log_and_handle_fatfs_error("mkdir tracking month dir", path_dir_month, res);
        return;
    }
    
    /* 打开文件，如不存在则创建 */
    res = f_open(&g_file, path_file, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        sd_unlock();
        sd_log_and_handle_fatfs_error("open tracking csv", path_file, res);
        return;
    }
    
    /* 检查文件大小，决定是否需要写表头，并移动到末尾追加 */
    if (f_size(&g_file) == 0) {
        need_header = true;
    } else {
        /* 文件已有内容，移动到末尾追加 */
        f_lseek(&g_file, f_size(&g_file));
        need_header = false;
    }
    
    /* 写入表头（如果是新文件） */
    if (need_header) {
        const char* header = "DateTime,RunMode,OptMode,GpsFix,"
                           "SunAz_Astro,SunEl_Astro,"
                           "GimbalAz,GimbalEl,"
                           "UpV,LeftV,RightV,DownV,"
                           "Ex,Ey\r\n";
        FRESULT wres = f_write(&g_file, header, strlen(header), &bw);
        if (wres != FR_OK) {
            FRESULT cres = f_close(&g_file);
            sd_unlock();

            sd_log_and_handle_fatfs_error("write tracking header", path_file, wres);
            if (cres != FR_OK) {
                sd_log_and_handle_fatfs_error("close tracking csv after header error", path_file, cres);
            }
            return;
        }

        /* 写入索引文件 - 使用FA_OPEN_ALWAYS确保文件存在 */
        FIL index_file;
        FRESULT index_res;
        UINT bw_index;
        char index_line[32];
        int n;
        
        /* FA_OPEN_ALWAYS: 如果文件不存在则创建，存在则打开 */
        index_res = f_open(&index_file, "/solar_tracking/tracking_state/index.csv", FA_OPEN_ALWAYS | FA_WRITE);
        if (index_res == FR_OK) {
            /* 移动到文件末尾进行追加写入 */
            FRESULT lseek_res = f_lseek(&index_file, f_size(&index_file));
            
            n = snprintf(index_line, sizeof(index_line), "%04u-%02u-%02u\r\n", year, month, day);
            if (n > 0 && lseek_res == FR_OK) {
                FRESULT idx_wres = f_write(&index_file, index_line, (UINT)n, &bw_index);
                FRESULT idx_cres = f_close(&index_file);
                
                /* 索引文件错误也记录，但不中断主流程（tracking CSV已成功写入表头）*/
                if (idx_wres != FR_OK) {
                    /* 此时仍在sd_lock()中，需先解锁 */
                    f_close(&g_file);
                    sd_unlock();
                    sd_log_and_handle_fatfs_error("write index csv for new date", "/solar_tracking/tracking_state/index.csv", idx_wres);
                    return;  /* 索引失败，不继续写数据行 */
                } else if (idx_cres != FR_OK) {
                    f_close(&g_file);
                    sd_unlock();
                    sd_log_and_handle_fatfs_error("close index csv", "/solar_tracking/tracking_state/index.csv", idx_cres);
                    return;
                }
            } else {
                f_close(&index_file);
                if (lseek_res != FR_OK) {
                    f_close(&g_file);
                    sd_unlock();
                    sd_log_and_handle_fatfs_error("lseek index csv", "/solar_tracking/tracking_state/index.csv", lseek_res);
                    return;
                }
            }
        } else {
            /* 打开索引文件失败，记录错误但继续写tracking数据（tracking文件已打开）*/
            /* 注意：此时g_file仍打开，在后续代码会写入数据行并关闭 */
            /* 这里只是记录一条警告，不中断主流程 */
        }
    }
    
    /* 写入数据行 */
    snprintf(buffer, sizeof(buffer),
             "%04d-%02d-%02d %02d:%02d:%02d,%u,%u,%u,"
             "%.2f,%.2f,"
             "%.2f,%.2f,"
             "%.3f,%.3f,%.3f,%.3f,"
             "%.4f,%.4f\r\n",
             st->time_local.year, st->time_local.month, st->time_local.day,
             st->time_local.hour, st->time_local.minute, st->time_local.second,
             st->run_mode, st->opt_mode, st->gps_fix ? 1 : 0,
             st->sun_az, st->sun_el,
             st->gimbal_az, st->gimbal_el,
             st->up_v, st->left_v, st->right_v, st->down_v,
             st->ex, st->ey);
    
    FRESULT wres = f_write(&g_file, buffer, strlen(buffer), &bw);
    FRESULT cres = f_close(&g_file);
    
    sd_unlock();

    /* 在解锁之后记录错误 */
    if (wres != FR_OK) {
        sd_log_and_handle_fatfs_error("write tracking line", path_file, wres);
    } else if (cres != FR_OK) {
        sd_log_and_handle_fatfs_error("close tracking csv", path_file, cres);
    }
}

/*===========================================================================*/
/* 按日期读取tracking日志接口                                                */
/*===========================================================================*/

/*
 * 打开指定日期的tracking日志文件
 * 参数：year - 年份，month - 月份，day - 日期
 * 返回：true=成功，false=失败（文件不存在或SD卡错误）
 */
bool sd_logger_open_tracking_file_by_date(uint16_t year, uint8_t month, uint8_t day)
{
    char path_dir_year[64];
    char path_dir_month[80];
    char path_file[96];
    FRESULT res;
    
    if (!g_sd_mounted) {
        return false;
    }
    
    sd_lock();
    
    /* 如果之前有打开的文件，先关闭 */
    if (s_tracking_file_opened) {
        f_close(&s_tracking_read_file);
        s_tracking_file_opened = false;
    }
    
    /* 构造文件路径：/solar_tracking/tracking_state/YYYY/MM/YYYYMMDD.csv */
    snprintf(path_dir_year, sizeof(path_dir_year), "%s/%04u", SD_DIR_TRACKING_STATE, year);
    snprintf(path_dir_month, sizeof(path_dir_month), "%s/%02u", path_dir_year, month);
    snprintf(path_file, sizeof(path_file), "%s/%04u%02u%02u.csv", path_dir_month, year, month, day);
    
    /* 尝试打开文件（只读） */
    res = f_open(&s_tracking_read_file, path_file, FA_READ);
    
    if (res == FR_OK) {
        s_tracking_file_opened = true;
        sd_unlock();
        return true;
    }

    sd_unlock();

    /* 记录错误并在严重错误时自动deinit */
    sd_log_and_handle_fatfs_error("open tracking file by date", path_file, res);
    return false;
}

/*
 * 读取tracking日志文件的一块数据
 * 参数：buf - 缓冲区，max_len - 最大读取字节数
 * 返回：实际读取的字节数（0表示EOF或未打开文件）
 */
int sd_logger_read_tracking_chunk(uint8_t *buf, uint16_t max_len)
{
    UINT br = 0;
    
    if (!g_sd_mounted || !s_tracking_file_opened || !buf) {
        return 0;
    }
    
    sd_lock();
    
    /* 直接调用f_read，不检查返回值（简化代码，减少警告）*/
    f_read(&s_tracking_read_file, buf, max_len, &br);
    
    sd_unlock();
    
    /* 返回实际读取字节数，读到EOF时br为0 */
    return (int)br;
}

/*
 * 关闭当前打开的tracking日志文件
 */
void sd_logger_close_tracking_file(void)
{
    sd_lock();
    
    if (s_tracking_file_opened) {
        f_close(&s_tracking_read_file);
        s_tracking_file_opened = false;
    }
    
    sd_unlock();
}

bool sd_logger_open_tracking_index(void)
{
    FRESULT res;
    
    if (!g_sd_mounted) {
        return false;
    }
    
    sd_lock();
    
    if (s_tracking_file_opened) {
        f_close(&s_tracking_read_file);
        s_tracking_file_opened = false;
    }
    
    res = f_open(&s_tracking_read_file, "/solar_tracking/tracking_state/index.csv", FA_READ);
    
    if (res == FR_OK) {
        s_tracking_file_opened = true;
        sd_unlock();
        return true;
    } else {
        sd_unlock();
        return false;
    }
}

void sd_logger_request_rebuild_index_async(void)
{
    if (!g_sd_mounted) {
        host_comm_set_register(REG_SDLOG_STATUS, SDLOG_STATUS_ERROR);
        return;
    }

    g_sd_rebuild_request = true;
    host_comm_set_register(REG_SDLOG_STATUS, SDLOG_STATUS_BUSY);
}

void sd_logger_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        if (g_sd_rebuild_request && !g_sd_rebuild_in_progress && g_sd_mounted) {
            g_sd_rebuild_in_progress = true;

            if (sd_logger_rebuild_index()) {
                host_comm_set_register(REG_SDLOG_STATUS, SDLOG_STATUS_IDLE);
            } else {
                host_comm_set_register(REG_SDLOG_STATUS, SDLOG_STATUS_ERROR);
            }

            g_sd_rebuild_request = false;
            g_sd_rebuild_in_progress = false;
        }
        else if (g_sd_rebuild_request && !g_sd_mounted) {
            /* 收到重建请求后卡又不挂载了（例如拔出、接触不良、或被 deinit）
             * 直接报错并清理状态，避免上位机一直看到 BUSY 超时 */
            host_comm_set_register(REG_SDLOG_STATUS, SDLOG_STATUS_ERROR);
            g_sd_rebuild_request = false;
            g_sd_rebuild_in_progress = false;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*===========================================================================*/
/* Index 重建功能                                                             */
/*===========================================================================*/

/*!
    \brief      检查文件名是否为有效的日期CSV格式 (YYYYMMDD.csv)
    \param[in]  fname - 文件名
    \param[out] year - 年份
    \param[out] month - 月份
    \param[out] day - 日期
    \retval     true - 有效, false - 无效
*/
static bool sd_is_valid_date_csv(const char* fname, uint16_t* year, uint8_t* month, uint8_t* day)
{
    int i;
    unsigned int y, m, d;
    
    if (!fname || strlen(fname) != 12) {
        return false;  /* 必须是 12 字符: YYYYMMDD.csv */
    }
    
    /* 检查后缀（支持大小写 .csv/.CSV） */
    if (fname[8] != '.') {
        return false;
    }
    char c1 = fname[9];
    char c2 = fname[10];
    char c3 = fname[11];
    /* 转为小写后比较 */
    if ((c1 >= 'A' && c1 <= 'Z')) c1 = (char)(c1 - 'A' + 'a');
    if ((c2 >= 'A' && c2 <= 'Z')) c2 = (char)(c2 - 'A' + 'a');
    if ((c3 >= 'A' && c3 <= 'Z')) c3 = (char)(c3 - 'A' + 'a');
    if (c1 != 'c' || c2 != 's' || c3 != 'v') {
        return false;
    }
    
    /* 检查前8位是否都是数字 */
    for (i = 0; i < 8; i++) {
        if (fname[i] < '0' || fname[i] > '9') {
            return false;
        }
    }
    
    /* 解析日期 */
    if (sscanf(fname, "%4u%2u%2u", &y, &m, &d) != 3) {
        return false;
    }
    
    /* 简单的日期合法性检查 */
    if (y < 2020 || y > 2100 || m < 1 || m > 12 || d < 1 || d > 31) {
        return false;
    }
    
    if (year) *year = (uint16_t)y;
    if (month) *month = (uint8_t)m;
    if (day) *day = (uint8_t)d;
    
    return true;
}

/*!
    \brief      从现有日志文件重建 index.csv（公开接口）
    \param[in]  none
    \param[out] none
    \retval     true - 成功, false - 失败
    \note       由上位机命令触发，使用动态分配减少栈使用
*/
bool sd_logger_rebuild_index(void)
{
    FILINFO* fno = NULL;
    DIR* dir_year = NULL;
    FIL* index_file = NULL;
    FRESULT res;
    char path_year[64];
    bool success = false;
    bool need_reinit = false;  /* 检测到严重 FatFs 错误时，标记需要重新初始化 */
    int file_count = 0;
    
    if (!g_sd_mounted) {
        return false;  /* SD卡未挂载 */
    }
    
    sd_lock();

    /* 动态分配结构体（从堆分配，不占用栈） */
    fno = (FILINFO*)pvPortMalloc(sizeof(FILINFO));
    dir_year = (DIR*)pvPortMalloc(sizeof(DIR));
    index_file = (FIL*)pvPortMalloc(sizeof(FIL));
    
    if (!fno || !dir_year || !index_file) {
        goto cleanup;
    }
    
    /* 创建新的 index.csv（强制覆盖） */
    res = f_open(index_file, "/solar_tracking/tracking_state/index.csv", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        /* 检测严重的 FatFs 错误，标记需要重新初始化 SD 卡 */
        if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
            need_reinit = true;
        }
        goto cleanup;
    }
    
    /* 遍历 tracking_state 目录下的所有年份目录 */
    res = f_opendir(dir_year, SD_DIR_TRACKING_STATE);
    if (res != FR_OK) {
        f_close(index_file);
        /* 检测严重的 FatFs 错误，标记需要重新初始化 SD 卡 */
        if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
            need_reinit = true;
        }
        goto cleanup;
    }
    
    /* 遍历 tracking_state 下的所有项目：
       - 目录：按 年/月份 结构递归扫描
       - 普通文件：兼容旧版平铺布局 /solar_tracking/tracking_state/YYYYMMDD.csv */
    for (;;) {
        res = f_readdir(dir_year, fno);
        if (res != FR_OK) {
            /* 检测严重的 FatFs 错误 */
            if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
                need_reinit = true;
            }
            break;
        }
        if (fno->fname[0] == 0) {
            break;
        }

        /* 统一取得文件名（优先用长文件名，没有则用短文件名） */
        const char* name = (fno->fname[0] != 0) ? fno->fname : fno->altname;

        /* 跳过隐藏/系统目录项 */
        if (name[0] == '.') {
            continue;
        }

        if (fno->fattrib & AM_DIR) {
            /* 作为年份目录处理：/solar_tracking/tracking_state/YYYY */
            snprintf(path_year, sizeof(path_year), "%s/%s", SD_DIR_TRACKING_STATE, name);

            /* 分层处理每个年份下的月份目录 */
            DIR* dir_month = (DIR*)pvPortMalloc(sizeof(DIR));
            if (!dir_month) {
                continue;
            }

            res = f_opendir(dir_month, path_year);
            if (res == FR_OK) {
                FILINFO* fno_month = (FILINFO*)pvPortMalloc(sizeof(FILINFO));
                if (fno_month) {
                    /* 遍历月份目录 */
                    for (;;) {
                        res = f_readdir(dir_month, fno_month);
                        if (res != FR_OK || fno_month->fname[0] == 0) {
                            break;
                        }

                        /* 统一取得文件名（优先用长文件名，没有则用短文件名） */
                        const char* month_name_str = (fno_month->fname[0] != 0) ? fno_month->fname : fno_month->altname;

                        if ((fno_month->fattrib & AM_DIR) && month_name_str[0] != '.') {
                            /* 分层处理月份目录，检测是否遇到严重错误 */
                            if (sd_process_month_dir(index_file, path_year, month_name_str, &file_count)) {
                                need_reinit = true;
                            }
                        }
                    }
                    vPortFree(fno_month);
                }
                f_closedir(dir_month);
            }
            vPortFree(dir_month);
        }
    }
    
    f_closedir(dir_year);
    f_close(index_file);
    
    /* 如果一个有效的日期文件都没找到，则认为重建失败 */
    success = (file_count > 0);
    
cleanup:
    if (fno) vPortFree(fno);
    if (dir_year) vPortFree(dir_year);
    if (index_file) vPortFree(index_file);
    
    sd_unlock();
    
    /* 如果检测到严重的 FatFs 错误，记录日志并自动卸载文件系统
     * 这样可以让后续的热插拔处理或手动操作重新挂载 SD 卡
     * 不再需要物理拔插卡就能恢复 */
    if (need_reinit) {
        sd_logger_printf("[SDLOG] rebuild index encountered severe FatFs error, deinit SD");
        sd_logger_deinit();
    }
    
    return success;
}

/*!
    \brief      处理单个月份目录，扫描其中的日志文件
    \param[in]  index_file - index.csv 文件句柄
    \param[in]  year_path - 年份目录路径
    \param[in]  month_name - 月份目录名
    \param[in]  p_file_count - 输出：累计找到的有效日期文件数量指针，可为NULL
    \retval     true - 遇到严重 FatFs 错误需要重新初始化, false - 正常
*/
static bool sd_process_month_dir(FIL* index_file, const char* year_path, const char* month_name, int* p_file_count)
{
    DIR* dir_day = NULL;
    FILINFO* fno_day = NULL;
    FRESULT res;
    char path_month[80];
    char line[32];
    uint16_t year;
    uint8_t month, day;
    UINT bw;
    bool need_reinit = false;  /* 检测严重错误 */
    
    dir_day = (DIR*)pvPortMalloc(sizeof(DIR));
    fno_day = (FILINFO*)pvPortMalloc(sizeof(FILINFO));
    
    if (!dir_day || !fno_day) {
        goto cleanup_month;
    }
    
    /* 构造月份目录完整路径 */
    snprintf(path_month, sizeof(path_month), "%s/%s", year_path, month_name);
    
    /* 打开月份目录 */
    res = f_opendir(dir_day, path_month);
    if (res != FR_OK) {
        /* 检测严重错误 */
        if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
            need_reinit = true;
        }
        goto cleanup_month;
    }
    
    /* 遍历日期文件 */
    for (;;) {
        res = f_readdir(dir_day, fno_day);
        if (res != FR_OK) {
            /* 检测严重错误 */
            if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
                need_reinit = true;
            }
            break;
        }
        if (fno_day->fname[0] == 0) {
            break;
        }
        
        /* 统一取得文件名（优先用长文件名，没有则用短文件名） */
        const char* day_name = (fno_day->fname[0] != 0) ? fno_day->fname : fno_day->altname;
        
        /* 跳过目录 */
        if (fno_day->fattrib & AM_DIR) {
            continue;
        }
        
        /* 检查是否为有效的日期CSV文件 */
        if (sd_is_valid_date_csv(day_name, &year, &month, &day)) {
            char full_path[96];
            FIL  test_file;

            /* 先构造完整路径：/solar_tracking/tracking_state/YYYY/MM/YYYYMMDD.csv */
            snprintf(full_path, sizeof(full_path), "%s/%s", path_month, day_name);

            /* 尝试打开文件，确保当前能读 */
            res = f_open(&test_file, full_path, FA_READ);
            if (res == FR_OK) {
                f_close(&test_file);

                /* 只有能成功f_open的文件才写入index.csv */
                snprintf(line, sizeof(line), "%04u-%02u-%02u\r\n", year, month, day);
                f_write(index_file, line, strlen(line), &bw);

                if (p_file_count != NULL) {
                    (*p_file_count)++;
                }
            } else {
                /* 严重错误：让上层触发重新挂载 */
                if (res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_INT_ERR) {
                    need_reinit = true;
                    break;  /* 跳出循环，交给上层处理 */
                }
                /* 非严重错误（例如文件刚好被删除）：跳过该日期，不写入index */
            }
        }
    }
    
    f_closedir(dir_day);
    
cleanup_month:
    if (dir_day) vPortFree(dir_day);
    if (fno_day) vPortFree(fno_day);
    
    return need_reinit;  /* 返回是否需要重新初始化 */
}
