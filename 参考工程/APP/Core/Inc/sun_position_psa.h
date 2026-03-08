/*!
    \file    sun_position_psa.h
    \brief   PSA更新算法 - 太阳位置计算 (2020-2050)

    \version 2025-02-01, V1.0
    \note    PSA+算法 (2020年更新，有效期至2050年)
             精度: 约±0.005° (30角秒)
             适合有FPU的MCU (如GD32F425)

             来源: Plataforma Solar de Almería研究所
             基于Jean Meeus天文算法的高精度实现
*/

#ifndef SUN_POSITION_PSA_H
#define SUN_POSITION_PSA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \brief    PSA更新算法 - 计算太阳位置
    \param[in]  latitude: 纬度 (度，正值为北纬)
    \param[in]  longitude: 经度 (度，正值为东经)
    \param[in]  year: 年 (2020-2050有效)
    \param[in]  month: 月 (1-12)
    \param[in]  day: 日 (1-31)
    \param[in]  hour: 时 (0-23，本地时间)
    \param[in]  minute: 分 (0-59)
    \param[in]  second: 秒 (0-59)
    \param[in]  timezone: 时区偏移 (小时，东8区为+8)
    \param[out] azimuth: 方位角 (度，南=0°，顺时针为正，天文坐标系)
    \param[out] elevation: 高度角 (度)
    \retval     bool: 计算是否成功
    \note       算法特点：
                - MCU优化版，使用float强制FPU运算
                - 输出天文坐标系 (南=0°)，需转换为云台坐标系 (北=0°)
                - 转换公式：北=0°方位角 = (南=0°方位角 + 180°) mod 360°
                - 精度约±0.01°
    \warning    hour参数是本地时间，算法会自动转换为UTC：
                UTC小时 = hour - timezone
                如果传入的是UTC时间，请将timezone设为0
*/
bool sun_position_psa_calculate(double latitude, double longitude,
                                uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second,
                                int8_t timezone,
                                float* azimuth, float* elevation);

/*!
    \brief    检查太阳是否可见
    \param[in]  elevation: 高度角 (度)
    \param[in]  threshold: 可见阈值 (度，默认-5°)
    \retval     bool: 太阳是否可见
*/
bool sun_position_psa_is_visible(float elevation, float threshold);

/*!
    \brief    坐标系转换：天文坐标系（南=0°）→ 云台坐标系（北=0°）
    \param[in]  azimuth: 方位角 (南=0°，顺时针为正)
    \param[in]  elevation: 高度角 (度)
    \param[out] gimbal_azimuth: 方位角 (北=0°，顺时针为正)
    \param[out] gimbal_elevation: 高度角 (度，不变)
    \retval     bool: 转换是否成功
    \note       转换公式：云台方位角 = (天文方位角 + 180°) mod 360°
*/
bool sun_position_psa_convert_to_gimbal(float azimuth,
                                       float elevation,
                                       float* gimbal_azimuth,
                                       float* gimbal_elevation);

#ifdef __cplusplus
}
#endif

#endif /* SUN_POSITION_PSA_H */
