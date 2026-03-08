/*
 * optical_tracking.h - 光学跟踪控制接口（FreeRTOS 版）
 *
 * 主要职责：
 * 1. 维护光学跟踪状态机（初始化、启动、停止、模式切换）
 * 2. 提供误差查询与基准位置信息接口
 * 3. 在光学模式下由调度任务周期性调用 optical_tracking_update()
 */

#ifndef OPTICAL_TRACKING_H
#define OPTICAL_TRACKING_H

#include <stdbool.h>

/* 光学追踪死区阈值默认值 */
#define OPTICAL_ERROR_ENTER_DEFAULT     0.02f    /* 进入死区阈值（默认0.02，范围：0.001~1.0） */
#define OPTICAL_ERROR_EXIT_DEFAULT      0.025f   /* 退出死区阈值（默认0.025，范围：0.001~1.0，必须 > 进入阈值） */

typedef enum {
    OPT_MODE_DISABLED = 0,        /* 禁用光学控制，保持静止或由其他模式接管 */
    OPT_MODE_GPS_TRACKING,        /* GPS/天文跟踪模式 */
    OPT_MODE_OPTICAL_TRACKING     /* 光学传感器闭环跟踪模式 */
} optical_control_mode_t;

bool optical_tracking_init(void);
void optical_tracking_deinit(void);

bool optical_tracking_set_mode(optical_control_mode_t mode);
optical_control_mode_t optical_tracking_get_mode(void);

bool optical_tracking_start(void);
void optical_tracking_stop(void);
void optical_tracking_update(void);

bool optical_tracking_get_errors(float* ex, float* ey);

void optical_tracking_set_base_position(float azimuth, float elevation);
bool optical_tracking_get_base_position(float* azimuth, float* elevation);

bool optical_tracking_get_deadband(float* enter_err, float* exit_err);
void optical_tracking_set_deadband(float enter_err, float exit_err);

#endif /* OPTICAL_TRACKING_H */

