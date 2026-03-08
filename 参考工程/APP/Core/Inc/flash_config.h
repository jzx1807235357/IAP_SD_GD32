/*
 * flash_config.h - Flash核心配置存储（快速读取的关键参数）
 * 
 * 存储内容：系统配置、校准数据、设备参数
 * 存储位置：外部Flash (GD25Q16)
 * 特点：掉电保持、快速读取、有限擦写次数
 */
#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t timezone_offset_hours;
    float optical_enter_threshold;  /* 光控进入阈值（V，四路总和） */
    float optical_exit_threshold;   /* 光控退出阈值（V，四路总和） */
    
    /* 光学追踪校准数据 */
    float azimuth_offset;           /* 方位角校准偏差（度） */
    float elevation_offset;         /* 仰角校准偏差（度） */
    uint8_t calibration_valid;      /* 校准数据有效标志 (0xFF=有效) */
    
    /* 夜间归位位置 */
    float night_azimuth;            /* 夜间归位方位角（度） */
    float night_elevation;          /* 夜间归位仰角（度） */
    
    /* GPS校时间隔 */
    uint16_t gps_calib_interval_hours;  /* GPS校时间隔（小时，默认24） */
    
    /* 经纬度坐标 */
    float latitude;                 /* 纬度（度，-90~90） */
    float longitude;                /* 经度（度，-180~180） */
    
    /* 云台增益系数 */
    float azimuth_gain;             /* 方位角增益 */
    float elevation_gain;           /* 仰角增益 */
    
    /* 光学追踪死区阈值（归一化误差，范围0~1） */
    float optical_error_enter;      /* 进入死区阈值（默认0.02） */
    float optical_error_exit;       /* 退出死区阈值（默认0.025） */

    /* 太阳高度角工作范围 */
    float sun_elevation_work_min;   /* 太阳高度角工作范围下限（度，默认-5） */
    float sun_elevation_work_max;   /* 太阳高度角工作范围上限（度，默认90） */

    /* TODO: [TEST] 云台仰角工作范围 - 测试功能，测试后需移除或恢复固定值 */
    /* 测试期间允许上位机配置云台仰角工作范围，正常使用时建议移除或改为固定值 */
    float gimbal_elevation_min;     /* 云台仰角工作范围下限（度，默认5） */
    float gimbal_elevation_max;     /* 云台仰角工作范围上限（度，默认90） */

    uint8_t timezone_lock;          /* 1=已被上位机手动设置锁定，禁止GPS自动改时区 */
} config_runtime_t;

/* Flash配置初始化 */
bool flash_config_init(void);

/* 获取运行时配置 */
const config_runtime_t* flash_config_get_runtime(void);

/* 设置时区偏移 */
bool flash_config_set_timezone(int16_t offset_hours);

bool flash_config_set_timezone_and_lock(int16_t offset_hours, uint8_t lock);

/* 设置光控阈值 */
bool flash_config_set_optical_thresholds(float enter_threshold, float exit_threshold);

/* 校准数据快速访问接口 */
bool flash_config_save_calibration(float az_offset, float el_offset);
bool flash_config_get_calibration(float* az_offset, float* el_offset);
bool flash_config_has_valid_calibration(void);

/* 夜间归位位置设置 */
bool flash_config_set_night_position(float azimuth, float elevation);

/* GPS校时间隔设置 */
bool flash_config_set_gps_interval(uint16_t hours);

/* 经纬度设置 */
bool flash_config_set_location(float latitude, float longitude);

/* 云台增益设置 */
bool flash_config_set_gimbal_gain(float az_gain, float el_gain);

/* 光学追踪死区阈值设置 */
bool flash_config_set_optical_error_deadband(float enter_threshold, float exit_threshold);

/* 太阳高度角工作范围设置 */
bool flash_config_set_sun_elevation_work_range(float min_elevation, float max_elevation);

/* 云台仰角工作范围设置 */
bool flash_config_set_gimbal_elevation_range(float min_elevation, float max_elevation);

#endif /* FLASH_CONFIG_H */
