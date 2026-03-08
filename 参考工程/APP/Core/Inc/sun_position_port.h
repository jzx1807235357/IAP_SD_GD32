/*!
    \file    sun_position_port.h
    \brief   太阳位置模块 - 平台适配层接口

    \version 2025-01-31, V1.0
    \note    定义平台适配层接口，用于抽象硬件相关操作
             支持GPS、RTC、Flash配置等硬件访问
*/

#ifndef SUN_POSITION_PORT_H
#define SUN_POSITION_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据类型定义 ==================== */

/*!
    \brief    GPS位置信息
*/
typedef struct {
    double latitude;    /* 纬度 (度，正值为北纬) */
    double longitude;   /* 经度 (度，正值为东经) */
} sun_port_position_t;

/*!
    \brief    日期时间信息
*/
typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} sun_port_datetime_t;

/*!
    \brief    Flash配置信息
*/
typedef struct {
    int8_t timezone_offset_hours;  /* 时区偏移 (小时，东8区为+8) */
    uint8_t gps_calib_interval_hours;  /* GPS校准间隔 (小时) */
} sun_port_config_t;

/* ==================== 平台适配层接口 ==================== */

/*!
    \brief    平台适配层函数表
*/
typedef struct {
    /* GPS相关接口 */
    bool (*get_gps_position)(sun_port_position_t* position);
    bool (*get_gps_time)(sun_port_datetime_t* utc_time);

    /* RTC相关接口 */
    bool (*get_utc_time)(sun_port_datetime_t* utc_time);
    bool (*set_rtc_time)(const sun_port_datetime_t* utc_time);
    bool (*rtc_time_valid)(void);

    /* Flash配置接口 */
    bool (*get_flash_position)(sun_port_position_t* position);
    bool (*get_flash_config)(sun_port_config_t* config);

    /* 系统接口 */
    uint32_t (*get_tick)(void);
} sun_position_port_t;

/* ==================== 平台适配层API ==================== */

/*!
    \brief    获取平台适配层接口
    \retval     平台适配层函数表指针，未初始化时返回NULL
*/
const sun_position_port_t* sun_position_port_get_interface(void);

/*!
    \brief    检查平台适配层是否已初始化
    \retval     true: 已初始化，false: 未初始化
*/
bool sun_position_port_is_initialized(void);

/*!
    \brief    初始化平台适配层
    \retval     true: 成功，false: 失败
*/
bool sun_position_port_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SUN_POSITION_PORT_H */
