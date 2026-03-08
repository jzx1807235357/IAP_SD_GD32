/*!
 * gimbal_control.h
 * Pelco-D gimbal control interface for the solar tracking project.
 *
 * The gimbal shares USART2/RS485 with the light sensor module.
 * This API exposes movement commands, absolute positioning, gain tuning,
 * and asynchronous angle polling support.
 *
 * 2025-01-09, V1.0.0
 */

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/* USART/RS485 configuration shared with the light sensor */
#define GIMBAL_USART                USART2
#define GIMBAL_USART_CLK            RCU_USART2
#define GIMBAL_BAUDRATE             9600U
#define GIMBAL_TX_PORT              GPIOB
#define GIMBAL_TX_PIN               GPIO_PIN_10
#define GIMBAL_RX_PORT              GPIOC
#define GIMBAL_RX_PIN               GPIO_PIN_5
#define GIMBAL_GPIO_AF              GPIO_AF_7

#define GIMBAL_RS485_DE_PORT        GPIOA
#define GIMBAL_RS485_DE_PIN         GPIO_PIN_7
#define GIMBAL_RS485_DE_CLK         RCU_GPIOA

/* Timings (milliseconds) */
#define GIMBAL_CMD_TIMEOUT_MS       3000U
#define GIMBAL_QUERY_TIMEOUT_MS     200U    /* 查询超时：缩短到200ms */
#define GIMBAL_QUERY_RETRY_COUNT    2U      /* 查询重试次数 */
#define GIMBAL_QUERY_INTERVAL_MS    50U     /* 查询间隔：缩短到50ms */
#define GIMBAL_CMD_MIN_INTERVAL_MS  500U    /* 控制命令最小间隔：防止频繁发送占用总线（适用于自动控制模式） */
#define GIMBAL_RS485_TX_DELAY_MS    5U
#define GIMBAL_RS485_RX_DELAY_MS    2U
#define GIMBAL_FRAME_TIMEOUT_MS     50U     /* 帧接收超时：50ms */

#define PELCO_D_DEVICE_ADDR         0x01U

typedef enum {
    GIMBAL_OK = 0,              /* 操作成功 */
    GIMBAL_INVALID_PARAM,       /* 参数无效 */
    GIMBAL_TIMEOUT,             /* 操作超时 */
    GIMBAL_NOT_INITIALIZED,     /* 模块未初始化 */
    GIMBAL_BUSY                 /* 设备忙（正在执行其他操作） */
} gimbal_result_t;

typedef enum {
    GIMBAL_STOP = 0,            /* 停止移动 */
    GIMBAL_UP,                  /* 向上移动 */
    GIMBAL_DOWN,                /* 向下移动 */
    GIMBAL_LEFT,                /* 向左移动 */
    GIMBAL_RIGHT                /* 向右移动 */
} gimbal_direction_t;

typedef struct {
    float azimuth;              /* 方位角（度）, 范围：0-359 */
    float elevation;            /* 仰角（度）, 范围：0-90 */
} gimbal_position_t;

typedef struct {
    float azimuth_gain;         /* 方位角增益系数（校正传动比） */
    float elevation_gain;       /* 仰角增益系数（校正传动比） */
} gimbal_gain_config_t;

extern gimbal_position_t g_gimbal_position;
extern gimbal_gain_config_t g_gimbal_gain;

gimbal_result_t gimbal_init(void);

gimbal_result_t gimbal_move(gimbal_direction_t direction);
gimbal_result_t gimbal_stop(void);

/* 非阻塞查询接口 - 发送和检查分离 */
gimbal_result_t gimbal_send_query_azimuth(void);
gimbal_result_t gimbal_send_query_elevation(void);
bool gimbal_is_azimuth_ready(float* azimuth_deg);
bool gimbal_is_elevation_ready(float* elevation_deg);
void gimbal_clear_query_state(void);  /* 清理查询状态（超时后调用） */

gimbal_result_t gimbal_set_azimuth(float azimuth);
gimbal_result_t gimbal_set_elevation(float elevation);
gimbal_result_t gimbal_set_position(float azimuth, float elevation);

gimbal_result_t gimbal_restore_default_selftest(void);

void gimbal_set_gain(float azimuth_gain, float elevation_gain);
void gimbal_get_gain(gimbal_gain_config_t* gain);
void gimbal_reset_gain(void);

bool gimbal_get_cached_position(gimbal_position_t* position);

void gimbal_register_task_handle(TaskHandle_t task_handle);
void gimbal_process(void);

void gimbal_on_byte(uint8_t byte, BaseType_t *pxHigherPriorityTaskWoken);

bool gimbal_queue_command(gimbal_direction_t direction);
bool gimbal_queue_set_position(float azimuth, float elevation);

static inline bool gimbal_is_position_valid(float azimuth, float elevation)
{
    return (azimuth >= 0.0f && azimuth < 360.0f &&
            elevation >= 0.0f && elevation <= 90.0f);
}

static inline const char* gimbal_get_status_string(gimbal_result_t status)
{
    switch (status) {
        case GIMBAL_OK: return "OK";
        case GIMBAL_INVALID_PARAM: return "INVALID_PARAM";
        case GIMBAL_TIMEOUT: return "TIMEOUT";
        case GIMBAL_NOT_INITIALIZED: return "NOT_INITIALIZED";
        case GIMBAL_BUSY: return "BUSY";
        default: return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CONTROL_H */
