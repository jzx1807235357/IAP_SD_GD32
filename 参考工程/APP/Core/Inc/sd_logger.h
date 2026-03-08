/*!
    \file    sd_logger.h
    \brief   SD卡日志记录模块（大容量数据存储）
    
    功能：
    1. 光学追踪纠偏值存储（掉电不丢失）
    2. 系统运行日志记录（支持printf格式）
    3. 校准历史数据（CSV格式导出）
    
    文件结构：
    /solar_tracking/
        calibration.dat     - 当前纠偏值（二进制+CRC校验）
        calib_history.csv   - 校准历史（CSV格式，可Excel打开）
        system.log          - 系统日志（文本格式）
    
    特点：大容量、可扩展、易导出分析
    
    \version 2025-01-26, V1.0.0, SD Logger for Solar Tracking System
*/

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_rtc.h"  /* 引入rtc_datetime_t类型 */

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* 校准历史记录结构体（仅用于CSV导出）                                        */
/*===========================================================================*/
typedef struct {
    /* 校准结果 */
    float azimuth_offset;       /* 方位角偏差（度） */
    float elevation_offset;     /* 仰角偏差（度） */
    
    /* 校准环境 */
    float sun_azimuth;          /* 校准时太阳方位角 */
    float sun_elevation;        /* 校准时太阳仰角 */
    double latitude;            /* GPS纬度 */
    double longitude;           /* GPS经度 */
    
    /* 时间戳 */
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    
    /* 质量指标 */
    float total_light;          /* 总光强（V） */
    float ex;                   /* X轴误差 */
    float ey;                   /* Y轴误差 */
    uint32_t calib_count;       /* 累计校准次数 */
} calibration_history_t;

/*===========================================================================*/
/* 初始化和挂载                                                               */
/*===========================================================================*/
bool sd_logger_init(void);
void sd_logger_deinit(void);
bool sd_logger_is_mounted(void);

/* 热插拔支持 */
void sd_logger_card_inserted_callback(void);  /* 卡插入中断回调 */
void sd_logger_card_removed_callback(void);   /* 卡拔出中断回调 */
bool sd_logger_reinit(void);                  /* 重新初始化（支持热插拔） */
void sd_logger_process_hotplug_events(void);  /* 处理热插拔事件（任务中调用） */

/*===========================================================================*/
/* 校准历史记录（仅CSV日志）                                                  */
/*===========================================================================*/
void sd_logger_log_calibration(const calibration_history_t* data);

/*===========================================================================*/
/* 日志记录                                                                   */
/*===========================================================================*/
void sd_logger_printf(const char* format, ...);
void sd_logger_error(uint16_t error_code, const char* message);
void sd_logger_log_tracking(float ex, float ey, float az, float el);

/*===========================================================================*/
/* 工具函数                                                                   */
/*===========================================================================*/
void sd_logger_flush(void);

/*===========================================================================*/
/* 诊断日志结构体                                                             */
/*===========================================================================*/
typedef struct {
    /* 时间戳（本地时间） */
    rtc_datetime_t time_local;
    
    /* 系统模式 */
    uint16_t run_mode;          /* 主运行模式：0=Idle, 1=GPS, 2=Optical, 3=Manual */
    uint16_t opt_mode;          /* 光学追踪模式 */
    bool     gps_fix;           /* GPS定位状态 */
    
    /* 理论太阳位置（北=0°坐标系） */
    float    sun_az;            /* 太阳方位角（北=0°，顺时针为正） */
    float    sun_el;            /* 太阳仰角 */
    
    /* 云台实际位置（工程坐标系） */
    float    gimbal_az;         /* 云台方位角（北=0°） */
    float    gimbal_el;         /* 云台仰角 */
    
    /* 四路光敏电压 */
    float    up_v;
    float    left_v;
    float    right_v;
    float    down_v;
    
    /* 光学追踪误差 */
    float    ex;                /* X轴误差 */
    float    ey;                /* Y轴误差 */
} tracking_state_log_t;

/*===========================================================================*/
/* 诊断日志记录                                                               */
/*===========================================================================*/
void sd_logger_log_tracking_state(const tracking_state_log_t* st);

/*===========================================================================*/
/* 按日期读取tracking日志接口                                                */
/*===========================================================================*/
bool sd_logger_open_tracking_file_by_date(uint16_t year, uint8_t month, uint8_t day);
int  sd_logger_read_tracking_chunk(uint8_t *buf, uint16_t max_len);
void sd_logger_close_tracking_file(void);
bool sd_logger_open_tracking_index(void);

/*===========================================================================*/
/* 索引重建（上位机命令触发）                                              */
/*===========================================================================*/
bool sd_logger_rebuild_index(void);          /* 重建 index.csv，返回 true=成功（同步实现，内部使用） */
void sd_logger_request_rebuild_index_async(void);  /* 异步触发索引重建（快速返回，设置状态寄存器） */
void sd_logger_task(void *pvParameters);     /* SD日志后台任务入口，用于处理重建请求 */

#ifdef __cplusplus
}
#endif

#endif /* SD_LOGGER_H */
