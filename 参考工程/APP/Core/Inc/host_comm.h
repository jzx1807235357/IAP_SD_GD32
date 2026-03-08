/*!
    \file    host_comm.h
    \brief   上位机通信模块 - Modbus RTU从站

    \version 2024-12-21, V1.0.0, Host Communication for GD32F4xx
*/

#ifndef HOST_COMM_H
#define HOST_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f4xx.h"
#include "modbus_rtu.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/* 上位机通信配置（使用USART5 RS485总线） */
#define HOST_COMM_BAUDRATE            9600
#define HOST_COMM_USART               USART5
#define HOST_COMM_TX_PORT             GPIOC
#define HOST_COMM_TX_PIN              GPIO_PIN_6
#define HOST_COMM_RX_PORT             GPIOC
#define HOST_COMM_RX_PIN              GPIO_PIN_7
#define HOST_COMM_DE_RE_PORT          GPIOC
#define HOST_COMM_DE_RE_PIN           GPIO_PIN_9

/* 上位机数据结构 */
typedef struct {
    uint16_t device_status;        /* 设备状态 */
    uint16_t run_mode;             /* 运行模式 */
    uint16_t gps_status;           /* GPS状态 */
    uint16_t comm_status;          /* 通信状态 */
    uint16_t error_code;          /* 错误代码 */
    float sun_azimuth;             /* 太阳方位角 */
    float sun_elevation;           /* 太阳仰角 */
    float gimbal_azimuth;          /* 云台方位角 */
    float gimbal_elevation;        /* 云台仰角 */
    bool valid;                    /* 数据是否有效 */
} host_data_t;

/* 上位机通信状态 */
typedef enum {
    HOST_COMM_IDLE = 0,
    HOST_COMM_RECEIVING,
    HOST_COMM_TRANSMITTING,
    HOST_COMM_ERROR
} host_comm_state_t;

/* 上位机命令消息结构（用于FreeRTOS队列） */
typedef struct {
    uint16_t cmd_code;
    float param1;
    float param2;
} host_cmd_msg_t;

/* 全局变量声明 */
extern volatile host_comm_state_t g_host_comm_state;
extern volatile bool g_host_data_updated;
extern host_data_t g_host_data;

/* 初始化函数 */
bool host_comm_init(void);

/* 数据获取函数 */
bool host_comm_get_data(host_data_t* data);
bool host_comm_data_updated(void);
void host_comm_clear_update_flag(void);

/* 数据设置函数 */
void host_comm_set_light_data(float up, float left, float down, float right);
void host_comm_set_gimbal_data(float azimuth, float elevation);
void host_comm_set_device_status(uint16_t status);
void host_comm_set_error_code(uint16_t error_code);

/* 通信控制函数 */
void host_comm_process(void);
void host_comm_set_register(uint16_t addr, uint16_t value);
uint16_t host_comm_get_register(uint16_t addr);

/* 状态查询函数 */
host_comm_state_t host_comm_get_state(void);

/* 中断处理函数 */
/* host_comm_uart_irq_handler已废弃 - 只使用DMA+IDLE */
void host_comm_uart_idle_dma_rx_handler(uint16_t new_pos,
                                        BaseType_t* pxHigherPriorityTaskWoken);
void host_comm_register_task_handle(TaskHandle_t task_handle);

/* 命令执行函数（可从中断调用） */
void host_comm_execute_command(uint16_t cmd_code);

/* Flash参数管理函数 */
void host_comm_load_params_from_flash(void);

/* GPS经纬度自动更新函数 */
void host_comm_update_location_from_gps(float latitude, float longitude);

/* 二次校准offset自动更新函数 */
void host_comm_update_calibration_offset(float az_offset, float el_offset);
bool host_comm_request_calibration_save(float az_offset, float el_offset);
void host_comm_process_pending_calibration_save(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_COMM_H */
