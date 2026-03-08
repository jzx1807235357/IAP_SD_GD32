/*!
    \file    sun_position_port.c
    \brief   太阳位置模块 - 平台适配层实现 (GD32F4xx)

    \version 2025-02-01, V1.0
    \note    实现平台适配层接口，连接太阳位置算法与硬件
*/

#include "sun_position_port.h"
#include "gps_nmea.h"
#include "flash_config.h"
#include "app_rtc.h"
#include "systick.h"
#include <string.h>

/* ==================== GPS相关接口实现 ==================== */

/*!
    \brief    获取GPS位置
*/
static bool port_get_gps_position(sun_port_position_t* position)
{
    if (!position) {
        return false;
    }

    float altitude;
    return gps_get_position(&position->latitude, &position->longitude, &altitude);
}

/*!
    \brief    获取GPS时间 (UTC)
*/
static bool port_get_gps_time(sun_port_datetime_t* utc_time)
{
    if (!utc_time) {
        return false;
    }

    return gps_get_time(&utc_time->year, &utc_time->month, &utc_time->day,
                       &utc_time->hour, &utc_time->minute, &utc_time->second);
}

/* ==================== RTC相关接口实现 ==================== */

/*!
    \brief    获取UTC时间
*/
static bool port_get_utc_time(sun_port_datetime_t* utc_time)
{
    if (!utc_time) {
        return false;
    }

    return app_rtc_get_datetime((rtc_datetime_t*)utc_time);
}

/*!
    \brief    设置RTC时间
*/
static bool port_set_rtc_time(const sun_port_datetime_t* utc_time)
{
    if (!utc_time) {
        return false;
    }

    return app_rtc_set_datetime((const rtc_datetime_t*)utc_time);
}

/*!
    \brief    检查RTC时间是否有效
*/
static bool port_rtc_time_valid(void)
{
    return app_rtc_has_valid_time();
}

/* ==================== Flash配置接口实现 ==================== */

/*!
    \brief    获取Flash中存储的位置
*/
static bool port_get_flash_position(sun_port_position_t* position)
{
    if (!position) {
        return false;
    }

    const config_runtime_t* config = flash_config_get_runtime();
    if (!config) {
        return false;
    }

    position->latitude = config->latitude;
    position->longitude = config->longitude;

    /* 检查位置是否有效 */
    if (config->latitude == 0.0f && config->longitude == 0.0f) {
        return false;
    }

    return true;
}

/*!
    \brief    获取Flash中存储的配置
*/
static bool port_get_flash_config(sun_port_config_t* config)
{
    if (!config) {
        return false;
    }

    const config_runtime_t* runtime = flash_config_get_runtime();
    if (!runtime) {
        return false;
    }

    config->timezone_offset_hours = (int8_t)runtime->timezone_offset_hours;
    config->gps_calib_interval_hours = (uint8_t)runtime->gps_calib_interval_hours;

    return true;
}

/* ==================== 系统接口实现 ==================== */

/*!
    \brief    获取系统时钟 (毫秒)
*/
static uint32_t port_get_tick(void)
{
    return systick_get_tick();
}

/* ==================== 平台适配层接口表 ==================== */

static sun_position_port_t g_port_interface = {
    .get_gps_position    = port_get_gps_position,
    .get_gps_time        = port_get_gps_time,
    .get_utc_time        = port_get_utc_time,
    .set_rtc_time        = port_set_rtc_time,
    .rtc_time_valid      = port_rtc_time_valid,
    .get_flash_position  = port_get_flash_position,
    .get_flash_config    = port_get_flash_config,
    .get_tick            = port_get_tick,
};

static bool g_port_initialized = false;

/* ==================== 公开API实现 ==================== */

const sun_position_port_t* sun_position_port_get_interface(void)
{
    if (g_port_initialized) {
        return &g_port_interface;
    }
    return NULL;
}

bool sun_position_port_is_initialized(void)
{
    return g_port_initialized;
}

bool sun_position_port_init(void)
{
    /* 初始化各硬件模块 */
    /* 注意: GPS、RTC、Flash等模块的初始化应在main.c中完成 */
    /* 这里只是标记port layer已就绪 */

    g_port_initialized = true;
    return true;
}
