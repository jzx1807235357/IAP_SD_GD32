/*!
    \file    sun_position.c
    \brief   太阳位置计算模块 - 平台相关层

    \version 2025-02-01, V3.4, 使用PSA MCU优化算法
    \note    本文件提供高级太阳位置服务
             使用PSA算法（平台无关）和
             平台抽象层（硬件相关）

             架构:
             ┌─────────────────────────────────────┐
             │   This file (sun_position.c)       │  ← 高级API
             ├─────────────────────────────────────┤
             │   PSA Algorithm (MCU优化版)         │  ← 纯数学计算
             ├─────────────────────────────────────┤
             │   Port Layer (sun_position_port)    │  ← 硬件接口
             └─────────────────────────────────────┘

             PSA算法特点:
             - 精度: 约±0.01°
             - 直接输出北=0°坐标系 (云台坐标系)
             - 使用float强制FPU运算
             - 适合有FPU的MCU (如GD32F425)
             - 来源: Plataforma Solar de Almería研究所

             公开API:
             - sun_position_calculate()
             - sun_position_from_gps()
             - sun_position_gimbal_from_gps()
             - sun_position_is_visible()
             - time_get_local_datetime()
*/

#include "sun_position.h"
#include "sun_position_psa.h"
#include "sun_position_port.h"
#include "flash_config.h"
#include <math.h>
#include <stdlib.h>  /* For NULL */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 本地常量定义 */
#define SUN_MIN_VISIBLE_ELEVATION     (-5.0f)   /* 太阳可见阈值（度） */

/* GPS Fix状态标志（用于检测Fix恢复） */
static bool s_last_gps_fix_valid = false;
static bool s_port_initialized = false;

/* 本地辅助函数 */
static int is_leap_year(int year);
static void utc_to_local(uint16_t yu, uint8_t mou, uint8_t du,
                         uint8_t hu, uint8_t miu,
                         int tz_min,
                         uint16_t* yl, uint8_t* mol, uint8_t* dl,
                         uint8_t* hl, uint8_t* mil);

/* ==================== 平台适配层管理 ==================== */

/*!
    \brief    初始化平台适配层（自动调用）
*/
static void ensure_port_initialized(void)
{
    if (!s_port_initialized) {
        /* 自动初始化为GD32平台实现 */
        if (sun_position_port_get_interface() != NULL) {
            s_port_initialized = sun_position_port_is_initialized();
        }
    }
}

/* ==================== 公开API实现（兼容原有接口） ==================== */

/*!
    \brief    计算太阳位置（云台坐标系：北=0°，顺时针为正）
    \note     此函数调用PSA MCU优化算法
              PSA算法直接输出北=0°坐标系，无需额外转换
              参数year/month/day/hour/minute应使用本地时间
*/
bool sun_position_calculate(double latitude, double longitude,
                            uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute,
                            float* azimuth, float* elevation)
{
    /* 使用PSA MCU优化算法，输出云台坐标系（北=0°） */
    const config_runtime_t* config = flash_config_get_runtime();
    int8_t timezone = 8;  /* 默认东8区 */
    if (config != NULL) {
        int16_t tz = config->timezone_offset_hours;
        /* 时区范围验证（-12到+14） */
        if (tz >= -12 && tz <= 14) {
            timezone = (int8_t)tz;
        }
    }
    return sun_position_psa_calculate(latitude, longitude,
                                       year, month, day,
                                       hour, minute, 0,
                                       timezone,
                                       azimuth, elevation);
}

/*!
    \brief    检查太阳是否可见
*/
bool sun_position_is_visible(float elevation)
{
    return sun_position_psa_is_visible(elevation, SUN_MIN_VISIBLE_ELEVATION);
}

/*!
    \brief    坐标系转换：天文坐标系（南=0°）→ 云台坐标系（北=0°）
*/
bool sun_position_convert_to_gimbal(float azimuth,
                                    float elevation,
                                    float* gimbal_azimuth,
                                    float* gimbal_elevation)
{
    return sun_position_psa_convert_to_gimbal(azimuth, elevation,
                                              gimbal_azimuth, gimbal_elevation);
}

/*!
    \brief    从GPS获取太阳位置（云台坐标系：北=0°，顺时针为正）
    \note     此函数使用平台适配层访问GPS和RTC
              PSA算法直接输出北=0°坐标系，无需额外转换
*/
bool sun_position_from_gps(float* azimuth, float* elevation)
{
    if (!azimuth || !elevation) {
        return false;
    }

    /* 确保平台适配层已初始化 */
    ensure_port_initialized();

    if (!sun_position_port_is_initialized()) {
        return false;
    }

    const sun_position_port_t* port = sun_position_port_get_interface();
    if (!port) {
        return false;
    }

    /* 获取位置和UTC时间 */
    sun_port_position_t position;
    sun_port_datetime_t utc_time;
    bool gps_position_valid = false;

    /* 尝试获取GPS位置，如果失败则使用Flash中存储的经纬度 */
    if (port->get_gps_position && port->get_gps_position(&position)) {
        gps_position_valid = true;
    } else {
        /* GPS无信号，使用Flash配置的经纬度 */
        if (port->get_flash_position && port->get_flash_position(&position)) {
            gps_position_valid = false;  /* 标记为非GPS源 */
        } else {
            /* Flash中也没有有效经纬度，无法计算 */
            s_last_gps_fix_valid = false;
            return false;
        }
    }

    /* 获取UTC时间：优先使用GPS时间，否则使用RTC时间 */
    if (gps_position_valid && port->get_gps_time && port->get_gps_time(&utc_time))
    {
        /* GPS时间有效，需要定期同步到RTC */
        static uint32_t last_rtc_sync_time = 0;
        static bool first_sync_done = false;
        uint32_t current_time = port->get_tick ? port->get_tick() : 0;
        bool should_sync = false;
        bool fix_recovered = false;

        /* 检测GPS Fix恢复 */
        if (!s_last_gps_fix_valid) {
            fix_recovered = true;
        }
        s_last_gps_fix_valid = true;

        /* 首次GPS有效时立即同步（确保日期真实有效，不是默认值） */
        if (!first_sync_done && utc_time.year > 2020) {
            should_sync = true;
            first_sync_done = true;
        }
        /* RTC无效时立即同步 */
        else if (port->rtc_time_valid && !port->rtc_time_valid() && utc_time.year > 2020) {
            should_sync = true;
        }
        /* GPS信号刚刚恢复时，立即同步一次 */
        else if (fix_recovered && utc_time.year > 2020) {
            should_sync = true;
        }
        /* 之后根据配置的间隔同步 */
        else {
            sun_port_config_t config;
            if (port->get_flash_config && port->get_flash_config(&config)) {
                uint32_t sync_interval_ms = 3600000;  /* 默认1小时 */

                if (config.gps_calib_interval_hours >= 1 &&
                    config.gps_calib_interval_hours <= 168) {
                    sync_interval_ms = (uint32_t)config.gps_calib_interval_hours * 3600000;
                }

                if ((current_time - last_rtc_sync_time) > sync_interval_ms) {
                    should_sync = true;
                }
            }
        }

        if (should_sync && port->set_rtc_time) {
            /* GPS提供UTC时间，直接存储到RTC */
            port->set_rtc_time(&utc_time);
            last_rtc_sync_time = current_time;
        }
    }
    else
    {
        /* GPS时间无效或GPS无信号，使用RTC时间 */
        s_last_gps_fix_valid = false;

        if (!port->get_utc_time || !port->get_utc_time(&utc_time)) {
            return false;
        }

        if (port->rtc_time_valid && !port->rtc_time_valid()) {
            return false;
        }
    }

    /* 使用PSA更新算法计算太阳位置，精度±0.005° (30角秒) */
    int8_t timezone = 0;  /* utc_time已经是UTC时间，无需转换 */
    return sun_position_psa_calculate(position.latitude, position.longitude,
                                       utc_time.year, utc_time.month, utc_time.day,
                                       utc_time.hour, utc_time.minute, utc_time.second,
                                       timezone,
                                       azimuth, elevation);
}

/*!
    \brief    从GPS获取云台位置（北=0°坐标系）
    \note     PSA算法直接输出北=0°坐标系，此函数与sun_position_from_gps()相同
              保留此函数是为了保持接口兼容性
*/
bool sun_position_gimbal_from_gps(float* gimbal_azimuth, float* gimbal_elevation)
{
    if (!gimbal_azimuth || !gimbal_elevation) {
        return false;
    }

    /* PSA算法已直接输出北=0°坐标系，无需转换 */
    float elevation = 0.0f;

    if (!sun_position_from_gps(gimbal_azimuth, &elevation)) {
        return false;
    }

    if (!sun_position_is_visible(elevation)) {
        return false;
    }

    *gimbal_elevation = elevation;
    return true;
}

/* ==================== 本地时间辅助函数 ==================== */

/*!
    \brief    判断是否为闰年
*/
static int is_leap_year(int year)
{
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        return 1;
    }
    return 0;
}

/*!
    \brief    UTC时间转换为本地时间
*/
static void utc_to_local(uint16_t yu, uint8_t mou, uint8_t du,
                         uint8_t hu, uint8_t miu,
                         int tz_min,
                         uint16_t* yl, uint8_t* mol, uint8_t* dl,
                         uint8_t* hl, uint8_t* mil)
{
    int total_minutes = (hu * 60 + miu) + tz_min;
    int carry_day = 0;

    while (total_minutes >= 1440) {
        total_minutes -= 1440;
        carry_day++;
    }
    while (total_minutes < 0) {
        total_minutes += 1440;
        carry_day--;
    }

    int hour = total_minutes / 60;
    int minute = total_minutes % 60;

    int year = yu;
    int month = mou;
    int day = du + carry_day;

    while (day > 31) {
        int days_in_month;
        switch (month) {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12:
            days_in_month = 31;
            break;
        case 4: case 6: case 9: case 11:
            days_in_month = 30;
            break;
        case 2:
            days_in_month = is_leap_year(year) ? 29 : 28;
            break;
        default:
            days_in_month = 30;
            break;
        }
        if (day <= days_in_month) {
            break;
        }
        day -= days_in_month;
        month++;
        if (month > 12) {
            month = 1;
            year++;
        }
    }

    while (day < 1) {
        month--;
        if (month < 1) {
            month = 12;
            year--;
        }
        int days_in_month;
        switch (month) {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12:
            days_in_month = 31;
            break;
        case 4: case 6: case 9: case 11:
            days_in_month = 30;
            break;
        case 2:
            days_in_month = is_leap_year(year) ? 29 : 28;
            break;
        default:
            days_in_month = 30;
            break;
        }
        day += days_in_month;
    }

    *yl  = (uint16_t)year;
    *mol = (uint8_t)month;
    *dl  = (uint8_t)day;
    *hl  = (uint8_t)hour;
    *mil = (uint8_t)minute;
}

/*!
    \brief    获取本地时间（外置RTC + 时区偏移）
*/
bool time_get_local_datetime(rtc_datetime_t* local_dt)
{
    if (!local_dt) {
        return false;
    }

    /* 确保平台适配层已初始化 */
    ensure_port_initialized();

    if (!sun_position_port_is_initialized()) {
        return false;
    }

    const sun_position_port_t* port = sun_position_port_get_interface();
    if (!port || !port->get_utc_time || !port->get_flash_config) {
        return false;
    }

    /* 从RTC读取UTC时间 */
    sun_port_datetime_t utc_time;
    if (!port->get_utc_time(&utc_time)) {
        return false;
    }

    /* 获取时区偏移 */
    sun_port_config_t config;
    if (!port->get_flash_config(&config)) {
        return false;
    }

    int tz_offset_minutes = config.timezone_offset_hours * 60;

    /* UTC转本地时间 */
    uint16_t local_year;
    uint8_t local_month, local_day, local_hour, local_minute;

    utc_to_local(utc_time.year, utc_time.month, utc_time.day,
                 utc_time.hour, utc_time.minute,
                 tz_offset_minutes,
                 &local_year, &local_month, &local_day,
                 &local_hour, &local_minute);

    /* 填充本地时间结构体 */
    local_dt->year   = local_year;
    local_dt->month  = local_month;
    local_dt->day    = local_day;
    local_dt->hour   = local_hour;
    local_dt->minute = local_minute;
    local_dt->second = utc_time.second;

    return true;
}
