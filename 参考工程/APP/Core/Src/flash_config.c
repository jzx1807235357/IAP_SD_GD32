/*
 * flash_config.c - Flash核心配置存储（快速读取的关键参数）
 * 
 * 存储内容：系统配置、校准数据、设备参数
 * 存储位置：片内Flash最后2个扇区
 * 特点：掉电保持、快速读取、有限擦写次数
 */

#include "flash_config.h"
#include "optical_tracking.h"

#include "gd25q16_flash.h"

#include "gd32f4xx_rcu.h"
#include "gd32f4xx_pmu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

#define CONFIG_MAGIC_VALUE          0x534F4C41UL /* 'ALOS' */
#define CONFIG_VERSION              0x000BU      /* Version 11: only current-version records are loaded from Flash */

#define CONFIG_LOCK_TIMEOUT_MS      1000U

/* 多扇区配置区域定义 - 使用整片GD25Q16（512扇区，2MB）循环存储 */
#define CONFIG_REGION_SECTOR_COUNT  FLASH_SECTOR_COUNT  /* 512扇区，全片Flash参与wear-leveling */
#define CONFIG_REGION_START_ADDR    0U                  /* 从地址0开始 */
#define CONFIG_SECTOR_ADDRESS       (FLASH_SIZE - FLASH_SECTOR_SIZE)  /* 保持兼容性 */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint32_t sequence;         /* 序列号，用于识别最新配置 */
    int16_t  timezone_offset_hours;
    float    optical_enter_threshold;  /* 光控进入阈值 (V) */
    float    optical_exit_threshold;   /* 光控退出阈值 (V) */
    
    /* 光学追踪校准数据 */
    float    azimuth_offset;           /* 方位角校准偏差（度） */
    float    elevation_offset;         /* 仰角校准偏差（度） */
    uint8_t  calibration_valid;        /* 校准数据有效标志 (0xFF=有效) */
    
    /* 夜间归位位置 */
    float    night_azimuth;            /* 夜间归位方位角（度） */
    float    night_elevation;          /* 夜间归位仰角（度） */
    
    /* GPS校时间隔 */
    uint16_t gps_calib_interval_hours; /* GPS校时间隔（小时） */
    
    /* 经纬度坐标 */
    float    latitude;                 /* 纬度（度，-90~90） */
    float    longitude;                /* 经度（度，-180~180） */
    
    /* 云台增益系数 */
    float    azimuth_gain;             /* 方位角增益 */
    float    elevation_gain;           /* 仰角增益 */
    
    /* 光学追踪死区阈值 */
    float    optical_error_enter;      /* 进入死区阈值 */
    float    optical_error_exit;       /* 退出死区阈值 */

    /* 太阳高度角工作范围 */
    float    sun_elevation_work_min;   /* 太阳高度角工作范围下限（度） */
    float    sun_elevation_work_max;   /* 太阳高度角工作范围上限（度） */

    /* TODO: [TEST] 云台仰角工作范围 - 测试功能，测试后需移除此字段 */
    float    gimbal_elevation_min;     /* 云台仰角工作范围下限（度，默认5） */
    float    gimbal_elevation_max;     /* 云台仰角工作范围上限（度，默认90） */

    uint8_t  timezone_lock;

    uint32_t crc32;
} flash_config_block_t;
#pragma pack(pop)

static flash_config_block_t s_flash_block;
static config_runtime_t     s_runtime;
static bool                 s_loaded = false;
static uint32_t             s_current_sector_index = 0;  /* 当前使用的扇区索引 (0 ~ CONFIG_REGION_SECTOR_COUNT-1) */

static SemaphoreHandle_t    s_config_mutex = NULL;

static uint32_t config_calc_crc(const flash_config_block_t* block);
static void     config_runtime_from_flash(const flash_config_block_t* block);
static void     config_flash_from_runtime(flash_config_block_t* block);
static bool     config_write_flash(void);
static bool     config_version_supported(uint16_t version);

static bool     config_scheduler_running(void);
static bool     config_lock(uint32_t timeout_ms);
static void     config_unlock(void);

static void     config_backup_write_enable(void);
static void     config_backup_write_disable(void);
static uint32_t config_rtc_read_u32(uint32_t index);
static void     config_rtc_write_u32(uint32_t index, uint32_t value);

static bool     config_seq_is_newer(uint32_t a, uint32_t b);
static bool     config_read_and_validate_sector(uint32_t sector_index, flash_config_block_t* out_block);
static bool     config_try_load_from_rtc_hint(uint32_t* out_sector_index, flash_config_block_t* out_block);

bool flash_config_init(void)
{
    flash_status_t status;

    (void)config_lock(0U);

    if (!gd25q16_is_initialized()) {
        status = gd25q16_init();
        if (status != FLASH_OK) {
            /* Flash初始化失败，使用默认配置 */
            memset(&s_runtime, 0, sizeof(s_runtime));
            s_runtime.timezone_offset_hours = 0;
            s_runtime.optical_enter_threshold = 0.50f;
            s_runtime.optical_exit_threshold = 0.40f;
            s_runtime.night_azimuth = 90.0f;
            s_runtime.night_elevation = 90.0f;
            s_runtime.gps_calib_interval_hours = 24;
            s_runtime.azimuth_gain = 1.022f;
            s_runtime.elevation_gain = 1.0f;
            s_runtime.optical_error_enter = 0.02f;
            s_runtime.optical_error_exit = 0.025f;
            s_runtime.sun_elevation_work_min = -5.0f;
            s_runtime.sun_elevation_work_max = 90.0f;
            /* TODO: [TEST] 测试功能默认值 - 测试后需移除以下两行 */
            s_runtime.gimbal_elevation_min = 5.0f;
            s_runtime.gimbal_elevation_max = 90.0f;
            s_runtime.timezone_lock = 0;
            s_loaded = true;
            config_unlock();
            return false;
        }
    }

    /* 先尝试使用RTC备份寄存器缓存的扇区索引加速定位 */
    {
        flash_config_block_t hinted_block;
        uint32_t hinted_sector = 0;
        if (config_try_load_from_rtc_hint(&hinted_sector, &hinted_block)) {
            memcpy(&s_flash_block, &hinted_block, sizeof(s_flash_block));
            s_current_sector_index = hinted_sector;
            config_runtime_from_flash(&s_flash_block);
            s_loaded = true;

            config_rtc_write_u32(1U, s_current_sector_index);
            config_unlock();
            return true;
        }
    }

    /* 扫描所有配置扇区，找到序列号最大的有效配置 */
    flash_config_block_t temp_block;
    bool found_valid = false;
    uint32_t max_sequence = 0;
    uint32_t valid_sector_index = 0;

    for (uint32_t i = 0; i < CONFIG_REGION_SECTOR_COUNT; i++) {
        uint32_t sector_addr = CONFIG_REGION_START_ADDR + (i * FLASH_SECTOR_SIZE);
        
        status = gd25q16_read(sector_addr, (uint8_t*)&temp_block, sizeof(temp_block));
        if (status != FLASH_OK) {
            continue;  /* 读取失败，跳过该扇区 */
        }

        /* 验证魔数 */
        if (temp_block.magic != CONFIG_MAGIC_VALUE) {
            continue;  /* 无效配置，跳过 */
        }

        /* 验证版本：只接受当前版本配置块 */
        if (!config_version_supported(temp_block.version)) {
            continue;
        }

        /* 验证CRC */
        uint32_t crc = config_calc_crc(&temp_block);
        if (crc != temp_block.crc32) {
            continue;  /* CRC校验失败，跳过 */
        }

        /* 找到有效配置，比较序列号（考虑回绕） */
        if (!found_valid || config_seq_is_newer(temp_block.sequence, max_sequence)) {
            max_sequence = temp_block.sequence;
            valid_sector_index = i;
            memcpy(&s_flash_block, &temp_block, sizeof(s_flash_block));
            found_valid = true;
        }
    }

    if (found_valid) {
        /* 找到有效配置，加载到运行时 */
        config_runtime_from_flash(&s_flash_block);
        s_current_sector_index = valid_sector_index;
        s_loaded = true;

        config_rtc_write_u32(1U, s_current_sector_index);
        config_unlock();
        
        return true;
    }

    /* 未找到有效配置，使用默认值并初始化第一个扇区 */
    memset(&s_runtime, 0, sizeof(s_runtime));
    s_runtime.timezone_offset_hours = 0;
    s_runtime.optical_enter_threshold = 0.50f;
    s_runtime.optical_exit_threshold = 0.40f;
    s_runtime.night_azimuth = 90.0f;
    s_runtime.night_elevation = 90.0f;
    s_runtime.gps_calib_interval_hours = 24;
    s_runtime.latitude = 0.0f;
    s_runtime.longitude = 0.0f;
    s_runtime.azimuth_gain = 1.022f;
    s_runtime.elevation_gain = 1.0f;
    s_runtime.optical_error_enter = 0.02f;
    s_runtime.optical_error_exit = 0.025f;
    s_runtime.sun_elevation_work_min = -5.0f;
    s_runtime.sun_elevation_work_max = 90.0f;
    /* TODO: [TEST] 测试功能默认值 - 测试后需移除以下两行 */
    s_runtime.gimbal_elevation_min = 5.0f;
    s_runtime.gimbal_elevation_max = 90.0f;
    s_runtime.timezone_lock = 0;
    s_current_sector_index = 0;
    s_loaded = true;

    /* 写入默认配置到Flash（确保系统有有效配置可用）*/
    config_write_flash();

    config_unlock();

    return false;
}

static bool config_scheduler_running(void)
{
    return (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
}

static bool config_lock(uint32_t timeout_ms)
{
    if (!config_scheduler_running()) {
        return true;
    }

    if (s_config_mutex == NULL) {
        s_config_mutex = xSemaphoreCreateRecursiveMutex();
        if (s_config_mutex == NULL) {
            return false;
        }
    }

    TickType_t wait_ticks = (timeout_ms == 0U) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(s_config_mutex, wait_ticks) == pdTRUE);
}

static void config_unlock(void)
{
    if (config_scheduler_running() && (s_config_mutex != NULL)) {
        xSemaphoreGiveRecursive(s_config_mutex);
    }
}

static void config_backup_write_enable(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPSRAM);
    pmu_backup_write_enable();
}

static void config_backup_write_disable(void)
{
    pmu_backup_write_disable();
}

static uint32_t config_rtc_read_u32(uint32_t index)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPSRAM);
    return REG32(RTC_BASE + 0x50U + (index * 4U));
}

static void config_rtc_write_u32(uint32_t index, uint32_t value)
{
    config_backup_write_enable();
    REG32(RTC_BASE + 0x50U + (index * 4U)) = value;
    config_backup_write_disable();
}

static bool config_seq_is_newer(uint32_t a, uint32_t b)
{
    return ((int32_t)(a - b) > 0);
}

static bool config_version_supported(uint16_t version)
{
    return (version == CONFIG_VERSION);
}

static bool config_read_and_validate_sector(uint32_t sector_index, flash_config_block_t* out_block)
{
    if (out_block == NULL) {
        return false;
    }

    uint32_t sector_addr = CONFIG_REGION_START_ADDR + (sector_index * FLASH_SECTOR_SIZE);
    flash_status_t status = gd25q16_read(sector_addr, (uint8_t*)out_block, sizeof(*out_block));
    if (status != FLASH_OK) {
        return false;
    }

    if (out_block->magic != CONFIG_MAGIC_VALUE) {
        return false;
    }

    if (!config_version_supported(out_block->version)) {
        return false;
    }

    uint32_t crc = config_calc_crc(out_block);
    return (crc == out_block->crc32);
}

static bool config_try_load_from_rtc_hint(uint32_t* out_sector_index, flash_config_block_t* out_block)
{
    if (out_sector_index == NULL || out_block == NULL) {
        return false;
    }

    uint32_t hint = config_rtc_read_u32(1U);
    if (hint >= CONFIG_REGION_SECTOR_COUNT) {
        return false;
    }

    const uint32_t WINDOW = 16U;
    bool found = false;
    uint32_t best_seq = 0;
    uint32_t best_idx = 0;
    flash_config_block_t tmp;

    for (uint32_t d = 0; d <= WINDOW; d++) {
        uint32_t idx_a = (hint + d) % CONFIG_REGION_SECTOR_COUNT;
        if (config_read_and_validate_sector(idx_a, &tmp)) {
            if (!found || config_seq_is_newer(tmp.sequence, best_seq)) {
                best_seq = tmp.sequence;
                best_idx = idx_a;
                memcpy(out_block, &tmp, sizeof(*out_block));
                found = true;
            }
        }

        if (d == 0U) {
            continue;
        }

        uint32_t idx_b = (hint + CONFIG_REGION_SECTOR_COUNT - d) % CONFIG_REGION_SECTOR_COUNT;
        if (config_read_and_validate_sector(idx_b, &tmp)) {
            if (!found || config_seq_is_newer(tmp.sequence, best_seq)) {
                best_seq = tmp.sequence;
                best_idx = idx_b;
                memcpy(out_block, &tmp, sizeof(*out_block));
                found = true;
            }
        }
    }

    if (!found) {
        return false;
    }

    *out_sector_index = best_idx;
    return true;
}

const config_runtime_t* flash_config_get_runtime(void)
{
    if (!s_loaded) {
        flash_config_init();
    }
    return &s_runtime;
}

bool flash_config_set_timezone(int16_t offset_hours)
{
    return flash_config_set_timezone_and_lock(offset_hours, s_runtime.timezone_lock);
}

bool flash_config_set_timezone_and_lock(int16_t offset_hours, uint8_t lock)
{
    if (!s_loaded) {
        flash_config_init();
    }

    uint8_t new_lock = (lock == 1U) ? 1U : 0U;
    if (s_runtime.timezone_offset_hours == offset_hours && s_runtime.timezone_lock == new_lock) {
        return true;
    }

    int16_t old_tz = s_runtime.timezone_offset_hours;
    uint8_t old_lock = s_runtime.timezone_lock;

    s_runtime.timezone_offset_hours = offset_hours;
    s_runtime.timezone_lock = new_lock;
    if (!config_write_flash()) {
        s_runtime.timezone_offset_hours = old_tz;
        s_runtime.timezone_lock = old_lock;
        return false;
    }
    return true;
}

bool flash_config_set_optical_thresholds(float enter_threshold, float exit_threshold)
{
    /* 参数验证 */
    if (enter_threshold <= 0.0f || exit_threshold <= 0.0f ||
        enter_threshold > 4.0f || exit_threshold > 4.0f ||
        exit_threshold >= enter_threshold) {
        return false;  /* 退出阈值必须小于进入阈值 */
    }
    
    if (fabsf(s_runtime.optical_enter_threshold - enter_threshold) < 0.001f &&
        fabsf(s_runtime.optical_exit_threshold - exit_threshold) < 0.001f) {
        return true;  /* 没有变化 */
    }

    float old_enter = s_runtime.optical_enter_threshold;
    float old_exit = s_runtime.optical_exit_threshold;

    s_runtime.optical_enter_threshold = enter_threshold;
    s_runtime.optical_exit_threshold = exit_threshold;
    
    if (!config_write_flash()) {
        s_runtime.optical_enter_threshold = old_enter;
        s_runtime.optical_exit_threshold = old_exit;
        return false;
    }
    return true;
}

static uint32_t config_calc_crc(const flash_config_block_t* block)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* bytes = (const uint8_t*)block;
    size_t length = offsetof(flash_config_block_t, crc32);

    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void config_runtime_from_flash(const flash_config_block_t* block)
{
    memset(&s_runtime, 0, sizeof(s_runtime));

    if (block == NULL || !config_version_supported(block->version)) {
        s_runtime.timezone_offset_hours = 0;
        s_runtime.optical_enter_threshold = 0.50f;
        s_runtime.optical_exit_threshold = 0.40f;
        s_runtime.night_azimuth = 90.0f;
        s_runtime.night_elevation = 90.0f;
        s_runtime.gps_calib_interval_hours = 24;
        s_runtime.latitude = 0.0f;
        s_runtime.longitude = 0.0f;
        s_runtime.azimuth_gain = 1.022f;
        s_runtime.elevation_gain = 1.0f;
        s_runtime.optical_error_enter = 0.02f;
        s_runtime.optical_error_exit = 0.025f;
        s_runtime.timezone_lock = 0;
        return;
    }

    s_runtime.timezone_offset_hours = block->timezone_offset_hours;
    s_runtime.optical_enter_threshold = block->optical_enter_threshold;
    s_runtime.optical_exit_threshold = block->optical_exit_threshold;
    s_runtime.azimuth_offset = block->azimuth_offset;
    s_runtime.elevation_offset = block->elevation_offset;
    s_runtime.calibration_valid = block->calibration_valid;
    s_runtime.night_azimuth = block->night_azimuth;
    s_runtime.night_elevation = block->night_elevation;
    s_runtime.gps_calib_interval_hours = block->gps_calib_interval_hours;
    s_runtime.latitude = block->latitude;
    s_runtime.longitude = block->longitude;
    s_runtime.azimuth_gain = block->azimuth_gain;
    s_runtime.elevation_gain = block->elevation_gain;
    
    /* 光学追踪死区阈值（直接读取，不检查版本） */
    s_runtime.optical_error_enter = block->optical_error_enter;
    s_runtime.optical_error_exit = block->optical_error_exit;

    /* 太阳高度角工作范围（直接读取，不检查版本） */
    s_runtime.sun_elevation_work_min = block->sun_elevation_work_min;
    s_runtime.sun_elevation_work_max = block->sun_elevation_work_max;

    /* 云台仰角工作范围（直接读取，不检查版本） */
    s_runtime.gimbal_elevation_min = block->gimbal_elevation_min;
    s_runtime.gimbal_elevation_max = block->gimbal_elevation_max;

    s_runtime.timezone_lock = (block->timezone_lock == 1U) ? 1U : 0U;

    /* 验证阈值合法性（添加NaN检测），否则使用默认值 */
    if (isnan(s_runtime.optical_enter_threshold) || isnan(s_runtime.optical_exit_threshold) ||
        s_runtime.optical_enter_threshold <= 0.0f || 
        s_runtime.optical_enter_threshold > 4.0f ||
        s_runtime.optical_exit_threshold <= 0.0f ||
        s_runtime.optical_exit_threshold > 4.0f ||
        s_runtime.optical_exit_threshold >= s_runtime.optical_enter_threshold) {
        s_runtime.optical_enter_threshold = 0.50f;
        s_runtime.optical_exit_threshold = 0.40f;
    }
    
    /* 验证夜间归位位置合法性（添加NaN检测） */
    if (isnan(s_runtime.night_azimuth) || s_runtime.night_azimuth < 0.0f || s_runtime.night_azimuth >= 360.0f) {
        s_runtime.night_azimuth = 90.0f;
    }
    if (isnan(s_runtime.night_elevation) || s_runtime.night_elevation < 0.0f || s_runtime.night_elevation > 90.0f) {
        s_runtime.night_elevation = 90.0f;
    }
    
    /* 验证GPS校时间隔合法性 */
    if (s_runtime.gps_calib_interval_hours < 1 || s_runtime.gps_calib_interval_hours > 168) {
        s_runtime.gps_calib_interval_hours = 24;
    }
    
    /* 验证经纬度合法性（添加NaN检测） */
    if (isnan(s_runtime.latitude) || s_runtime.latitude < -90.0f || s_runtime.latitude > 90.0f) {
        s_runtime.latitude = 0.0f;
    }
    if (isnan(s_runtime.longitude) || s_runtime.longitude < -180.0f || s_runtime.longitude > 180.0f) {
        s_runtime.longitude = 0.0f;
    }
    
    /* 验证增益合法性（添加NaN检测）*/
    if (isnan(s_runtime.azimuth_gain) || s_runtime.azimuth_gain <= 0.0f || 
        s_runtime.azimuth_gain < 0.5f || s_runtime.azimuth_gain > 2.0f) {
        s_runtime.azimuth_gain = 1.022f;
    }
    if (isnan(s_runtime.elevation_gain) || s_runtime.elevation_gain <= 0.0f || 
        s_runtime.elevation_gain < 0.5f || s_runtime.elevation_gain > 2.0f) {
        s_runtime.elevation_gain = 1.0f;
    }
    
    /* 验证光学追踪死区阈值合法性（添加NaN检测）*/
    if (isnan(s_runtime.optical_error_enter) || isnan(s_runtime.optical_error_exit) ||
        s_runtime.optical_error_enter <= 0.0f || s_runtime.optical_error_enter > 1.0f ||
        s_runtime.optical_error_exit <= 0.0f || s_runtime.optical_error_exit > 1.0f ||
        s_runtime.optical_error_enter >= s_runtime.optical_error_exit) {
        s_runtime.optical_error_enter = 0.02f;
        s_runtime.optical_error_exit = 0.025f;
    }

    /* 验证太阳高度角工作范围合法性（添加NaN检测）*/
    if (isnan(s_runtime.sun_elevation_work_min) || isnan(s_runtime.sun_elevation_work_max) ||
        s_runtime.sun_elevation_work_min < -90.0f || s_runtime.sun_elevation_work_min > 90.0f ||
        s_runtime.sun_elevation_work_max < -90.0f || s_runtime.sun_elevation_work_max > 90.0f ||
        s_runtime.sun_elevation_work_min >= s_runtime.sun_elevation_work_max) {
        s_runtime.sun_elevation_work_min = -5.0f;
        s_runtime.sun_elevation_work_max = 90.0f;
    }

    /* TODO: [TEST] 验证云台仰角工作范围合法性 - 测试后需移除此验证块 */
    if (isnan(s_runtime.gimbal_elevation_min) || isnan(s_runtime.gimbal_elevation_max) ||
        s_runtime.gimbal_elevation_min < 0.0f || s_runtime.gimbal_elevation_min > 90.0f ||
        s_runtime.gimbal_elevation_max < 0.0f || s_runtime.gimbal_elevation_max > 90.0f ||
        s_runtime.gimbal_elevation_min >= s_runtime.gimbal_elevation_max) {
        s_runtime.gimbal_elevation_min = 5.0f;
        s_runtime.gimbal_elevation_max = 90.0f;
    }
}

static void config_flash_from_runtime(flash_config_block_t* block)
{
    /* 先保存当前序列号，因为block可能就是s_flash_block */
    uint32_t next_sequence = s_flash_block.sequence + 1;
    
    memset(block, 0xFF, sizeof(*block));
    block->magic = CONFIG_MAGIC_VALUE;
    block->version = CONFIG_VERSION;
    block->sequence = next_sequence;  /* 使用预先保存的序列号 */
    block->timezone_offset_hours = s_runtime.timezone_offset_hours;
    block->optical_enter_threshold = s_runtime.optical_enter_threshold;
    block->optical_exit_threshold = s_runtime.optical_exit_threshold;
    block->azimuth_offset = s_runtime.azimuth_offset;
    block->elevation_offset = s_runtime.elevation_offset;
    block->calibration_valid = s_runtime.calibration_valid;
    block->night_azimuth = s_runtime.night_azimuth;
    block->night_elevation = s_runtime.night_elevation;
    block->gps_calib_interval_hours = s_runtime.gps_calib_interval_hours;
    block->latitude = s_runtime.latitude;
    block->longitude = s_runtime.longitude;
    block->azimuth_gain = s_runtime.azimuth_gain;
    block->elevation_gain = s_runtime.elevation_gain;
    block->optical_error_enter = s_runtime.optical_error_enter;
    block->optical_error_exit = s_runtime.optical_error_exit;
    block->sun_elevation_work_min = s_runtime.sun_elevation_work_min;
    block->sun_elevation_work_max = s_runtime.sun_elevation_work_max;
    block->gimbal_elevation_min = s_runtime.gimbal_elevation_min;
    block->gimbal_elevation_max = s_runtime.gimbal_elevation_max;
    block->timezone_lock = (s_runtime.timezone_lock == 1U) ? 1U : 0U;
    block->crc32 = config_calc_crc(block);
}

static bool config_write_flash(void)
{
    flash_status_t status;

    if (!s_loaded) {
        flash_config_init();
    }

    if (!config_lock(CONFIG_LOCK_TIMEOUT_MS)) {
        return false;
    }

    /* 准备新配置块（序列号自动递增） */
    flash_config_block_t new_block;
    config_flash_from_runtime(&new_block);

    /* 计算下一个要写入的扇区索引（循环） */
    uint32_t next_sector_index = (s_current_sector_index + 1) % CONFIG_REGION_SECTOR_COUNT;
    uint32_t sector_addr = CONFIG_REGION_START_ADDR + (next_sector_index * FLASH_SECTOR_SIZE);

    /* 擦除下一个扇区 */
    status = gd25q16_erase_sector(sector_addr);
    if (status != FLASH_OK) {
        config_unlock();
        return false;
    }

    /* 写入新配置 */
    status = gd25q16_write(sector_addr,
                           (const uint8_t*)&new_block,
                           sizeof(new_block));
    if (status != FLASH_OK) {
        config_unlock();
        return false;
    }

    /* 更新当前扇区索引 */
    s_current_sector_index = next_sector_index;

    memcpy(&s_flash_block, &new_block, sizeof(s_flash_block));
    config_rtc_write_u32(1U, s_current_sector_index);

    config_unlock();

    return true;
}

/*===========================================================================*/
/* 校准数据快速访问接口                                                       */
/*===========================================================================*/
bool flash_config_save_calibration(float az_offset, float el_offset)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 检查是否需要更新 */
    if (s_runtime.calibration_valid == 0xFF &&
        fabsf(s_runtime.azimuth_offset - az_offset) < 0.001f &&
        fabsf(s_runtime.elevation_offset - el_offset) < 0.001f) {
        return true;  /* 没有变化 */
    }
    
    /* 更新校准数据 */
    s_runtime.azimuth_offset = az_offset;
    s_runtime.elevation_offset = el_offset;
    s_runtime.calibration_valid = 0xFF;  /* 标记为有效 */
    
    return config_write_flash();
}

bool flash_config_get_calibration(float* az_offset, float* el_offset)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    if (az_offset == NULL || el_offset == NULL) {
        return false;
    }
    
    if (s_runtime.calibration_valid != 0xFF) {
        return false;  /* 校准数据无效 */
    }
    
    *az_offset = s_runtime.azimuth_offset;
    *el_offset = s_runtime.elevation_offset;
    return true;
}

bool flash_config_has_valid_calibration(void)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    return (s_runtime.calibration_valid == 0xFF);
}

/*===========================================================================*/
/* 夜间归位位置设置接口                                                       */
/*===========================================================================*/
bool flash_config_set_night_position(float azimuth, float elevation)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 参数验证 */
    if (azimuth < 0.0f || azimuth >= 360.0f ||
        elevation < 0.0f || elevation > 90.0f) {
        return false;
    }
    
    /* 检查是否需要更新 */
    if (fabsf(s_runtime.night_azimuth - azimuth) < 0.01f &&
        fabsf(s_runtime.night_elevation - elevation) < 0.01f) {
        return true;  /* 没有变化 */
    }
    
    /* 更新夜间归位位置 */
    s_runtime.night_azimuth = azimuth;
    s_runtime.night_elevation = elevation;
    
    return config_write_flash();
}

/*===========================================================================*/
/* GPS校时间隔设置接口                                                        */
/*===========================================================================*/
bool flash_config_set_gps_interval(uint16_t hours)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 参数验证 */
    if (hours < 1 || hours > 168) {
        return false;
    }
    
    /* 检查是否需要更新 */
    if (s_runtime.gps_calib_interval_hours == hours) {
        return true;  /* 没有变化 */
    }
    
    /* 更新GPS校时间隔 */
    s_runtime.gps_calib_interval_hours = hours;
    
    return config_write_flash();
}

/*===========================================================================*/
/* 经纬度设置接口                                                             */
/*===========================================================================*/
bool flash_config_set_location(float latitude, float longitude)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 参数验证 */
    if (latitude < -90.0f || latitude > 90.0f ||
        longitude < -180.0f || longitude > 180.0f) {
        return false;
    }
    
    /* 检查是否需要更新 */
    if (fabsf(s_runtime.latitude - latitude) < 1e-4f &&
        fabsf(s_runtime.longitude - longitude) < 1e-4f) {
        return true;  /* 没有变化 */
    }
    
    /* 更新经纬度 */
    s_runtime.latitude = latitude;
    s_runtime.longitude = longitude;
    
    return config_write_flash();
}

/*===========================================================================*/
/* 云台增益设置接口                                                           */
/*===========================================================================*/
bool flash_config_set_gimbal_gain(float az_gain, float el_gain)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 参数验证 */
    if (az_gain <= 0.0f || az_gain < 0.5f || az_gain > 2.0f ||
        el_gain <= 0.0f || el_gain < 0.5f || el_gain > 2.0f) {
        return false;
    }
    
    /* 检查是否需要更新 */
    if (fabsf(s_runtime.azimuth_gain - az_gain) < 1e-4f &&
        fabsf(s_runtime.elevation_gain - el_gain) < 1e-4f) {
        return true;  /* 没有变化 */
    }
    
    /* 更新增益 */
    s_runtime.azimuth_gain = az_gain;
    s_runtime.elevation_gain = el_gain;
    
    return config_write_flash();
}

/*===========================================================================*/
/* 光学追踪死区阈值设置接口                                                   */
/*===========================================================================*/
bool flash_config_set_optical_error_deadband(float enter_threshold, float exit_threshold)
{
    if (!s_loaded) {
        flash_config_init();
    }
    
    /* 参数验证：enter < exit，且都在 (0, 1] 范围 */
    if (enter_threshold <= 0.0f || enter_threshold > 1.0f ||
        exit_threshold <= 0.0f || exit_threshold > 1.0f ||
        enter_threshold >= exit_threshold) {
        return false;
    }
    
    /* 检查是否需要更新 */
    if (fabsf(s_runtime.optical_error_enter - enter_threshold) < 1e-4f &&
        fabsf(s_runtime.optical_error_exit - exit_threshold) < 1e-4f) {
        return true;  /* 没有变化 */
    }
    
    /* 更新死区阈值 */
    s_runtime.optical_error_enter = enter_threshold;
    s_runtime.optical_error_exit = exit_threshold;

    return config_write_flash();
}

/*===========================================================================*/
/* 太阳高度角工作范围设置接口                                                 */
/*===========================================================================*/
bool flash_config_set_sun_elevation_work_range(float min_elevation, float max_elevation)
{
    if (!s_loaded) {
        flash_config_init();
    }

    /* 参数验证：min < max，且都在 [-90, 90] 范围 */
    if (min_elevation < -90.0f || min_elevation > 90.0f ||
        max_elevation < -90.0f || max_elevation > 90.0f ||
        min_elevation >= max_elevation) {
        return false;
    }

    /* 检查是否需要更新 */
    if (fabsf(s_runtime.sun_elevation_work_min - min_elevation) < 0.01f &&
        fabsf(s_runtime.sun_elevation_work_max - max_elevation) < 0.01f) {
        return true;  /* 没有变化 */
    }

    /* 更新工作范围 */
    s_runtime.sun_elevation_work_min = min_elevation;
    s_runtime.sun_elevation_work_max = max_elevation;

    return config_write_flash();
}

/*!
    \brief    设置云台仰角工作范围
    \param[in]  min_elevation: 最小仰角 (0-90度)
    \param[in]  max_elevation: 最大仰角 (0-90度)
    \retval     true: 成功，false: 失败

    TODO: [TEST] 测试功能 - 测试后需移除此函数
*/
bool flash_config_set_gimbal_elevation_range(float min_elevation, float max_elevation)
{
    if (!config_lock(CONFIG_LOCK_TIMEOUT_MS)) {
        return false;
    }

    /* 参数验证：min < max，且都在 [0, 90] 范围 */
    if (min_elevation < 0.0f || min_elevation > 90.0f ||
        max_elevation < 0.0f || max_elevation > 90.0f ||
        min_elevation >= max_elevation) {
        config_unlock();
        return false;
    }

    /* 检查是否需要更新 */
    if (fabsf(s_runtime.gimbal_elevation_min - min_elevation) < 0.01f &&
        fabsf(s_runtime.gimbal_elevation_max - max_elevation) < 0.01f) {
        config_unlock();
        return true;  /* 没有变化 */
    }

    /* 更新工作范围 */
    s_runtime.gimbal_elevation_min = min_elevation;
    s_runtime.gimbal_elevation_max = max_elevation;

    bool result = config_write_flash();
    config_unlock();
    return result;
}

