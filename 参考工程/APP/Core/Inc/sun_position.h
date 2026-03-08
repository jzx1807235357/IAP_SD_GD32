/*!
    \file    sun_position.h
    \brief   Solar position calculation interfaces

    \version 2025-02-01, V3.4, PSA MCU优化算法，直接输出北=0°坐标系
    \note    Public API for the solar position module
             All functions are platform-independent through the port layer
             所有方位角输出均为云台坐标系（北=0°，顺时针为正）
*/

#ifndef SUN_POSITION_H
#define SUN_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 引用RTC时间类型（避免重复定义） */
#include "app_rtc.h"

typedef struct {
    double altitude;    /* Elevation angle in degrees */
    double azimuth;     /* Azimuth angle in degrees (north = 0°, clockwise positive) */
} sun_position_t;

/*!
    \brief    计算太阳位置（云台坐标系：北=0°，顺时针为正）
    \param[in]  latitude, longitude: GPS坐标
    \param[in]  year, month, day, hour, minute: 本地时间
    \param[out] azimuth: 方位角（北=0°，顺时针为正）
    \param[out] elevation: 仰角
    \retval     bool: 计算是否成功
    \note       PSA算法直接输出云台坐标系，无需额外转换
*/
bool sun_position_calculate(double latitude, double longitude,
                            uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute,
                            float* azimuth, float* elevation);

bool sun_position_is_visible(float elevation);

/*!
    \brief    坐标系转换函数（已过时，保留用于兼容性）
    \param[in]  azimuth: 方位角（南=0°，顺时针为正）
    \param[in]  elevation: 仰角
    \param[out] gimbal_azimuth: 方位角（北=0°，顺时针为正）
    \param[out] gimbal_elevation: 仰角（不变）
    \retval     bool: 转换是否成功
    \note       转换公式：云台方位角 = (天文方位角 + 180°) mod 360°
              PSA算法已直接输出北=0°坐标系，无需调用此函数
              保留此函数仅用于兼容旧代码
*/
bool sun_position_convert_to_gimbal(float azimuth,
                                    float elevation,
                                    float* gimbal_azimuth,
                                    float* gimbal_elevation);

bool sun_position_from_gps(float* azimuth, float* elevation);
bool sun_position_gimbal_from_gps(float* gimbal_azimuth, float* gimbal_elevation);

/*!
    \brief    获取本地时间（外置RTC + 时区偏移）
    \param[out] local_dt: 本地时间结构体指针
    \retval     bool: 是否成功获取本地时间
    \note       统一时间接口，所有需要记录本地时间的地方都应使用此接口
                自动从外置RTC读取UTC时间，并根据系统配置的时区转换为本地时间
*/
bool time_get_local_datetime(rtc_datetime_t* local_dt);

#ifdef __cplusplus
}
#endif

#endif /* SUN_POSITION_H */
