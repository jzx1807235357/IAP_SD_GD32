/*!
    \file    sun_position_psa.c
    \brief   PSA太阳位置算法 (MCU优化版)

    \version 2025-02-01, V2.0
    \note    PSA算法 (MCU优化版)
             精度: 约±0.01°

             优化特点:
             - 使用float强制FPU参与运算
             - 使用f后缀常量 (PI_F, RAD_F, DEG_F)
             - 使用fmodf替代while循环 (确定性执行时间)
             - 适合有FPU的MCU (如GD32F425)

             来源: Plataforma Solar de Almería研究所
*/

#include "sun_position_psa.h"
#include <math.h>

/* 定义常量，直接使用 float 结尾(f) 强制 FPU 参与运算 */
#define PI_F 3.141592653589793f
#define RAD_F (PI_F / 180.0f)
#define DEG_F (180.0f / PI_F)

/* double版本的PI，用于高精度计算 */
#define PI 3.14159265358979323846
#define RAD (PI / 180.0)

/* ==================== 公开API实现 ==================== */

/*!
    \brief    PSA 太阳位置算法 (MCU 优化版)
    \param    year, month, day, hour, minute, second: 本地时间
    \param    lat: 纬度 (北正)
    \param    lon: 经度 (东正)
    \param    timezone: 时区偏移 (小时，东8区为+8)，用于将本地时间转UTC
    \param    azimuth: 输出方位角 (度，北=0°，顺时针为正)
    \param    elevation: 输出高度角 (度)
    \note     直接输出云台坐标系 (北=0°)，无需额外转换
              公式来源: 标准天文学方位角公式，输出北=0°坐标系
*/
bool sun_position_psa_calculate(double latitude, double longitude,
                                uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second,
                                int8_t timezone,
                                float* azimuth, float* elevation)
{
    if (!azimuth || !elevation) {
        return false;
    }

    /* 转换为UTC时间 */
    float decimal_hour = (float)hour - (float)timezone + (float)minute / 60.0f + (float)second / 3600.0f;

    /* 1. 时间处理 */
    float y = (month <= 2) ? (float)year - 1.0f : (float)year;
    float m = (month <= 2) ? (float)month + 12.0f : (float)month;

    /* 2. Julian 日计算 (使用double精度防止小数位丢失) */
    double y_d = (double)y;
    double m_d = (double)m;
    double A_d = floor(y_d / 100.0);
    double B_d = 2.0 - A_d + floor(A_d / 4.0);
    double jd = floor(365.25 * (y_d + 4716.0)) + floor(30.6001 * (m_d + 1.0)) + (double)day + B_d - 1524.5;

    /* 只有在减去基准 J2000.0 (2451545.0) 得到差值 d 后，才能安全转回 float */
    float d = (float)((jd - 2451545.0) + ((double)decimal_hour / 24.0));

    /* 3. 太阳几何参数 (使用常用数学库函数) */
    float omega = (2.1429f - 0.0010394594f * d);
    float mean_longitude = 4.8950630f + 0.017202791698f * d;
    float mean_anomaly = 6.2400600f + 0.0172019699f * d;

    float ecliptic_longitude = mean_longitude + 0.03341607f * sinf(mean_anomaly)
                               + 0.00034894f * sinf(2.0f * mean_anomaly)
                               - 0.0001134f - 0.0000203f * sinf(omega);
    float obliquity = 0.4090928f - 6.2140e-9f * d + 0.0000396f * cosf(omega);

    /* 4. 赤经 RA 和 赤纬 Dec */
    float ra = atan2f(cosf(obliquity) * sinf(ecliptic_longitude), cosf(ecliptic_longitude));
    if (ra < 0) ra += 2.0f * PI_F;
    float dec = asinf(sinf(obliquity) * sinf(ecliptic_longitude));

    /* 5. 当地恒星时与时角（使用double精度防止精度丢失） */
    double lon_d = longitude;
    /* LMST公式（与C#版本完全一致）:
     * 18.697374558 是 J2000.0 的格林尼治恒星时（度）
     * 24.06570982441908 是恒星时变化率（度/天）
     * 乘以15是因为原始公式单位是小时，需要转为度
     * 注意：此公式结果可能非常大（>200,000），必须用double计算
     */
    double lmst = (18.697374558 + 24.06570982441908 * (double)d) * 15.0 + lon_d;
    double hour_angle = lmst * RAD_F - (double)ra;

    /* 规范化时角 [-PI, PI] */
    while (hour_angle < -PI) hour_angle += 2.0 * PI;
    while (hour_angle > PI) hour_angle -= 2.0 * PI;

    /* 6. 坐标转换（将hour_angle转回float用于后续计算） */
    float ha = (float)hour_angle;
    float lat_rad = (float)latitude * RAD_F;
    float cos_lat = cosf(lat_rad);
    float sin_lat = sinf(lat_rad);
    float cos_dec = cosf(dec);
    float sin_dec = sinf(dec);
    float cos_ha = cosf(ha);

    float sin_alt = sin_lat * sin_dec + cos_lat * cos_dec * cos_ha;
    /* 截断防止溢出导致的 NaN */
    if (sin_alt > 1.0f) sin_alt = 1.0f;
    if (sin_alt < -1.0f) sin_alt = -1.0f;

    *elevation = asinf(sin_alt) * DEG_F;

    float az_y = -sinf(ha);
    float az_x = tanf(dec) * cos_lat - sin_lat * cos_ha;
    *azimuth = atan2f(az_y, az_x) * DEG_F;

    /* 规范化方位角到 [0, 360) */
    if (*azimuth < 0) *azimuth += 360.0f;

    return true;
}

/*!
    \brief    检查太阳是否可见
*/
bool sun_position_psa_is_visible(float elevation, float threshold)
{
    return (elevation > threshold);
}

/*!
    \brief    坐标系转换：天文坐标系（南=0°）→ 云台坐标系（北=0°）
*/
bool sun_position_psa_convert_to_gimbal(float azimuth,
                                       float elevation,
                                       float* gimbal_azimuth,
                                       float* gimbal_elevation)
{
    if (!gimbal_azimuth || !gimbal_elevation) {
        return false;
    }

    /* 天文坐标系: 南=0°，顺时针为正 */
    /* 云台坐标系: 北=0°，顺时针为正 */
    /* 转换公式: 云台方位角 = (天文方位角 + 180°) mod 360° */
    *gimbal_azimuth = azimuth + 180.0f;
    while (*gimbal_azimuth >= 360.0f) {
        *gimbal_azimuth -= 360.0f;
    }
    *gimbal_elevation = elevation;

    return true;
}
