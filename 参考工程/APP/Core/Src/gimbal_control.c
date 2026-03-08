/*!
    \file    gimbal_control.c
    \brief   云台控制模块 - Pelco-D协议（适配光追项目）
    
    本模块实现云台的Pelco-D协议通信，支持：
    - 方向控制（上下左右停止）
    - 绝对位置设置与查询
    - 增益系数应用（支持校准云台实际角度）
    - 与光敏传感器共用USART2 RS485总线（硬件由rs485_hal模块统一管理）
    
    \version 2025-01-09, V1.0.0, gimbal control for solar tracking project
*/

#include "gimbal_control.h"
#include "rs485_hal.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Pelco-D协议常数 */
#define GIMBAL_ELEVATION_SCALE  0.01f  /* 90.0 / 0x2328(9000)，预计算加速ISR */

/* Pelco-D协议命令表 */
/* 基础移动命令（速度：0x20） */
static const uint8_t CMD_UP[]    = {0xFF, 0x01, 0x00, 0x08, 0x00, 0x20, 0x29};
static const uint8_t CMD_DOWN[]  = {0xFF, 0x01, 0x00, 0x10, 0x00, 0x20, 0x31};
static const uint8_t CMD_LEFT[]  = {0xFF, 0x01, 0x00, 0x04, 0x20, 0x00, 0x25};
static const uint8_t CMD_RIGHT[] = {0xFF, 0x01, 0x00, 0x02, 0x20, 0x00, 0x23};
static const uint8_t CMD_STOP[]  = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

/* 恢复默认参数并启动云台自检（设备厂商自定义Pelco-D命令） */
static const uint8_t CMD_RESTORE_DEFAULT_SELFTEST[] = {0xFF, 0x01, 0x00, 0x07, 0x00, 0x7D, 0x85};

/* 垂直角度查询命令 */
static const uint8_t CMD_QUERY_ELEVATION[] = {0xFF, 0x01, 0x00, 0x53, 0x00, 0x00, 0x54};

/* 水平角度查询命令 */
static const uint8_t CMD_QUERY_AZIMUTH[] = {0xFF, 0x01, 0x00, 0x51, 0x00, 0x00, 0x52};

/* 全局变量 */
gimbal_position_t g_gimbal_position = {0};
gimbal_gain_config_t g_gimbal_gain = {1.022f, 1.0f};  /* 默认增益系数1.0（云台实际传动比校正） */

/* ==================== 私有变量 ==================== */

/* 初始化状态 */
static bool g_initialized = false;

/* 命令队列（异步执行：ISR/其他任务设置，gimbal_process()执行） */
static volatile bool g_has_pending_cmd = false;
static volatile gimbal_direction_t g_pending_direction = GIMBAL_STOP;
static volatile bool g_has_pending_position = false;
static volatile float g_pending_azimuth = 0.0f;
static volatile float g_pending_elevation = 0.0f;

/* 角度查询相关 */
static volatile bool g_angle_query_active = false;           /* 是否有查询正在进行 */

/* 接收缓冲区 */
static volatile uint8_t g_gimbal_rx_buf[16];                 /* Pelco-D响应缓冲区（7字节） */
static volatile uint8_t g_gimbal_rx_len = 0;                 /* 当前接收字节数 */
static volatile uint32_t g_gimbal_rx_start_tick = 0;         /* 接收帧开始时间戳（用于超时检测） */

/* 非阻塞查询状态（新增）*/
typedef enum {
    QUERY_TYPE_NONE = 0,
    QUERY_TYPE_AZIMUTH,
    QUERY_TYPE_ELEVATION
} query_type_t;

static volatile query_type_t g_current_query_type = QUERY_TYPE_NONE;  /* 当前查询类型 */
static volatile bool g_query_data_ready = false;                       /* 查询数据就绪标志 */
static volatile float g_query_result_azimuth = 0.0f;                   /* 方位角查询结果 */
static volatile float g_query_result_elevation = 0.0f;                 /* 仰角查询结果 */

/* FreeRTOS任务句柄 */
static TaskHandle_t s_gimbal_task_handle = NULL;             /* 用于任务通知唤醒 */


/* 私有函数声明 */
static gimbal_result_t gimbal_send_command(const uint8_t* cmd, uint8_t len);
static uint8_t gimbal_calc_checksum(const uint8_t* data);
static bool gimbal_verify_response(const uint8_t* buffer, uint8_t len);
static void gimbal_abort_query(void);


/*!
    \brief    初始化云台控制模块
    \detail   在使用云台前必须调用此函数进行初始化
              注意：
              1. USART2硬件由rs485_hal模块统一初始化，本函数不处理硬件
              2. 云台和光敏传感器共用USART2 RS485总线（RS485_BUS_SENSOR）
              3. 必须先调用rs485_init(RS485_BUS_SENSOR, ...)初始化总线
              4. 本函数只初始化软件逻辑状态（位置缓存、增益系数等）
    \retval   gimbal_result_t: 操作结果（GIMBAL_OK表示成功）
*/
gimbal_result_t gimbal_init(void)
{
    /* ==================== 第1步：验证RS485总线已初始化 ==================== */
    if (!rs485_is_initialized(RS485_BUS_SENSOR)) {
        return GIMBAL_NOT_INITIALIZED;  /* RS485总线未初始化 */
    }
    
    /* ==================== 第2步：初始化云台位置缓存 ==================== */
    /* 清零云台位置结构体（方位角、仰角初始化为0°）*/
    memset(&g_gimbal_position, 0, sizeof(gimbal_position_t));
    
    /* ==================== 第3步：初始化增益配置 ==================== */
    /* 增益系数用于在发送前补偿传动比，在读取后还原真实角度 */
    /* 默认：方位/仰角均采用 1.0 */
    g_gimbal_gain.azimuth_gain = 1.022f;
    g_gimbal_gain.elevation_gain = 1.0f;
    
    /* ==================== 第4步：标记初始化完成 ==================== */
    /* 命令队列、查询状态、接收缓冲区等变量已在定义时初始化为0/false */
    /* 无需重复赋值，直接标记模块初始化完成 */
    /* 此标志非常重要！所有其他云台函数都会检查它 */
    /* 如果未初始化就调用其他函数，会返回GIMBAL_NOT_INITIALIZED错误 */
    g_initialized = true;
    
    /* 初始化成功，返回OK */
    return GIMBAL_OK;
}

/*!
    \brief    云台方向移动控制
    \param[in]  direction: 移动方向
    \retval     gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_move(gimbal_direction_t direction)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;

    const uint8_t* cmd = NULL;

    switch (direction) {
        case GIMBAL_STOP:  cmd = CMD_STOP;  break;
        case GIMBAL_UP:    cmd = CMD_UP;    break;
        case GIMBAL_DOWN:  cmd = CMD_DOWN;  break;
        case GIMBAL_LEFT:  cmd = CMD_LEFT;  break;
        case GIMBAL_RIGHT: cmd = CMD_RIGHT; break;
        default: return GIMBAL_INVALID_PARAM;
    }

    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }

    gimbal_result_t result = gimbal_send_command(cmd, 7);
    
    /* 释放总线访问权 */
    rs485_unlock_bus(RS485_BUS_SENSOR);
    
    return result;
}

/*!
    \brief    停止云台移动
    \retval   gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_stop(void)
{
    return gimbal_move(GIMBAL_STOP);
}

gimbal_result_t gimbal_restore_default_selftest(void)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;

    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }

    gimbal_result_t result = gimbal_send_command(CMD_RESTORE_DEFAULT_SELFTEST, 7);

    /* 释放总线访问权 */
    rs485_unlock_bus(RS485_BUS_SENSOR);

    return result;
}

/*!
    \brief    设置云台水平角度（自动应用增益系数）
    \param[in]  azimuth: 水平角度 (0.0 - 359.0)
    \retval     gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_set_azimuth(float azimuth)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;
    
    /* 输入角度范围检查和规范化（0-359°） */
    while (azimuth < 0.0f) azimuth += 360.0f;
    while (azimuth >= 360.0f) azimuth -= 360.0f;

    /* 应用增益系数，将目标角度转换为云台实际需要的角度 */
    float adjusted_azimuth = azimuth * g_gimbal_gain.azimuth_gain;
    /* 注意：不再对adjusted_azimuth取模，允许超过360° */

    /* 构建Pelco-D命令（直接使用调整后的角度） */
    uint8_t cmd[7] = {0xFF, 0x01, 0x00, 0x4B, 0x00, 0x00, 0x00};
    /* Pelco-D方位角格式：值 = 角度 × 100 */
    uint16_t az_val = (uint16_t)(adjusted_azimuth * 100.0f);
    /* 确保不超过16位无符号整数最大值 */
    if (az_val > 0xFFFF) az_val = 0xFFFF;
    
    cmd[4] = (az_val >> 8) & 0xFF;
    cmd[5] = az_val & 0xFF;
    cmd[6] = gimbal_calc_checksum(cmd);

    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }

    gimbal_result_t result = gimbal_send_command(cmd, 7);
    
    /* 释放总线访问权 */
    rs485_unlock_bus(RS485_BUS_SENSOR);
    
    return result;
}

/*!
    \brief    设置云台垂直角度（自动应用增益系数）
    \param[in]  elevation: 垂直角度 (0.0 - 90.0)
    \retval     gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_set_elevation(float elevation)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;
    
    /* 输入角度范围检查（0-90°） */
    if (elevation < 0.0f) elevation = 0.0f;
    if (elevation > 90.0f) elevation = 90.0f;

    /* 应用增益系数，将目标角度转换为云台实际需要的角度 */
    float adjusted_elevation = elevation * g_gimbal_gain.elevation_gain;
    /* 注意：这里不再限制adjusted_elevation <= 90，允许超过90° */

    /* 构建Pelco-D命令（直接使用调整后的角度） */
    uint8_t cmd[7] = {0xFF, 0x01, 0x00, 0x4D, 0x00, 0x00, 0x00};
    /* Pelco-D仰角格式：0x2328对应90°，但应用增益后可能超过90° */
    uint16_t el_val = (uint16_t)((adjusted_elevation / 90.0f) * 0x2328);
    /* 确保不超过16位无符号整数最大值 */
    if (el_val > 0xFFFF) el_val = 0xFFFF;
    
    cmd[4] = (el_val >> 8) & 0xFF;
    cmd[5] = el_val & 0xFF;
    cmd[6] = gimbal_calc_checksum(cmd);

    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }

    gimbal_result_t result = gimbal_send_command(cmd, 7);
    
    /* 释放总线访问权 */
    rs485_unlock_bus(RS485_BUS_SENSOR);
    
    return result;
}

/*!
    \brief    设置云台绝对位置（自动应用增益系数）
    \param[in]  azimuth: 水平角度 (0.0 - 359.0)
    \param[in]  elevation: 垂直角度 (0.0 - 90.0)
    \retval     gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_set_position(float azimuth, float elevation)
{
    gimbal_result_t result;
    
    /* 设置水平角度 */
    result = gimbal_set_azimuth(azimuth);
    if (result != GIMBAL_OK) return result;
    
    /* 命令间隔 - RTOS兼容延时 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        delay_1ms(200);
    }
    
    /* 设置垂直角度 */
    result = gimbal_set_elevation(elevation);
    if (result != GIMBAL_OK) return result;
    
    return GIMBAL_OK;
}

/*!
    \brief    设置云台增益系数
    \param[in]  azimuth_gain: 水平角度增益系数
    \param[in]  elevation_gain: 垂直角度增益系数
*/
void gimbal_set_gain(float azimuth_gain, float elevation_gain)
{
    taskENTER_CRITICAL();
    if (azimuth_gain > 0.0f) {
        g_gimbal_gain.azimuth_gain = azimuth_gain;
    }
    if (elevation_gain > 0.0f) {
        g_gimbal_gain.elevation_gain = elevation_gain;
    }
    taskEXIT_CRITICAL();
}

/*!
    \brief    获取云台增益系数
    \param[out] gain: 增益配置结构
*/
void gimbal_get_gain(gimbal_gain_config_t* gain)
{
    if (gain) {
        taskENTER_CRITICAL();
        *gain = g_gimbal_gain;
        taskEXIT_CRITICAL();
    }
}

/*!
    \brief    重置云台增益系数为默认值（1.0）
*/
void gimbal_reset_gain(void)
{
    taskENTER_CRITICAL();
    g_gimbal_gain.azimuth_gain = 1.022f;
    g_gimbal_gain.elevation_gain = 1.0f;
    taskEXIT_CRITICAL();
}

/*!
    \brief    获取缓存的云台位置（非阻塞）
    \param[out] position: 云台位置结构
    \retval     bool: 数据是否有效
*/
bool gimbal_get_cached_position(gimbal_position_t* position)
{
    if (!position) return false;
    
    taskENTER_CRITICAL();
    *position = g_gimbal_position;
    taskEXIT_CRITICAL();
    /* Gain correction already applied in ISR when storing position */
    return true;
}

void gimbal_register_task_handle(TaskHandle_t task_handle)
{
    s_gimbal_task_handle = task_handle;
}

/*!
    \brief    云台处理函数（在主循环中定期调用）
    \note     非阻塞设计：位置命令分两步执行（先方位角、后仰角），避免200ms阻塞
*/
void gimbal_process(void)
{
    static uint8_t retry_count = 0;
    static uint8_t position_step = 0;      /* 0=待执行方位角, 1=待执行仰角 */
    const uint8_t MAX_RETRY = 10;
    
    /* 优先处理挂起的方向命令（提高响应性） */
    if (g_has_pending_cmd) {
        /* 先检查总线是否被占用（传感器正在查询） */
        if (rs485_is_bus_locked(RS485_BUS_SENSOR)) {
            /* 总线被占用，保留命令，等待下次循环 */
            return;
        }
        
        /* 总线空闲，执行命令 */
        gimbal_result_t result = gimbal_move(g_pending_direction);
        if (result == GIMBAL_OK) {
            g_has_pending_cmd = false;
            retry_count = 0;
        } else if (result == GIMBAL_TIMEOUT) {
            /* 超时也保留命令，下次重试（可能是总线竞争） */
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                /* 重试次数过多，强制清除命令 */
                g_has_pending_cmd = false;
                retry_count = 0;
            }
        } else {
            /* 其他错误清除标志，避免一直重试错误命令 */
            g_has_pending_cmd = false;
            retry_count = 0;
        }
        return; /* 本次处理完成，下次再处理位置命令 */
    }
    
    /* 处理挂起的位置命令（分两步非阻塞执行，无等待） */
    if (g_has_pending_position) {
        /* 先检查总线是否被占用 */
        if (rs485_is_bus_locked(RS485_BUS_SENSOR)) {
            return;
        }
        
        gimbal_result_t result;
        
        if (position_step == 0) {
            /* 步骤1：发送方位角命令 */
            result = gimbal_set_azimuth(g_pending_azimuth);
            if (result == GIMBAL_OK) {
                position_step = 1;       /* 下次循环发送仰角 */
                retry_count = 0;
            } else if (result == GIMBAL_TIMEOUT) {
                retry_count++;
                if (retry_count >= MAX_RETRY) {
                    g_has_pending_position = false;
                    position_step = 0;
                    retry_count = 0;
                }
            } else {
                g_has_pending_position = false;
                position_step = 0;
                retry_count = 0;
            }
        } else if (position_step == 1) {
            /* 步骤2：发送仰角命令（无等待，直接发送） */
            result = gimbal_set_elevation(g_pending_elevation);
            if (result == GIMBAL_OK) {
                g_has_pending_position = false;  /* 位置命令完成 */
                position_step = 0;
                retry_count = 0;
            } else if (result == GIMBAL_TIMEOUT) {
                retry_count++;
                if (retry_count >= MAX_RETRY) {
                    g_has_pending_position = false;
                    position_step = 0;
                    retry_count = 0;
                }
            } else {
                g_has_pending_position = false;
                position_step = 0;
                retry_count = 0;
            }
        }
        return;
    }
    
    /* 重置状态 */
    retry_count = 0;
    position_step = 0;
}


/*!
    \brief    USART2中断接收云台数据
*/
void gimbal_on_byte(uint8_t byte, BaseType_t *pxHigherPriorityTaskWoken)
{
    /* 只在查询激活时处理接收（云台移动时不响应查询，会超时）*/
    if (!g_angle_query_active) {
        return;
    }

    uint32_t now = systick_get_tick();
    
    /* 帧超时检测：如果接收了部分字节后超过50ms没有收到后续字节，重置接收状态 */
    if (g_gimbal_rx_len > 0 && (now - g_gimbal_rx_start_tick) > GIMBAL_FRAME_TIMEOUT_MS) {
        g_gimbal_rx_len = 0;  /* 超时，重置接收状态 */
    }

    /* 状态0: 等待同步字节 0xFF */
    if (g_gimbal_rx_len == 0U) {
        if (byte == 0xFF) {
            g_gimbal_rx_buf[0] = byte;
            g_gimbal_rx_len = 1;
            g_gimbal_rx_start_tick = now;  /* 记录接收开始时间 */
        }
        /* 非0xFF字节被忽略，继续等待 */
        return;
    }

    /* 状态1: 验证地址字节 0x01 */
    if (g_gimbal_rx_len == 1U) {
        if (byte == 0x01) {
            g_gimbal_rx_buf[1] = byte;
            g_gimbal_rx_len = 2;
        } else {
            /* 地址不匹配，重新开始寻找帧头 */
            g_gimbal_rx_len = 0;
            if (byte == 0xFF) {
                g_gimbal_rx_buf[0] = byte;
                g_gimbal_rx_len = 1;
            }
        }
        return;
    }

    /* 状态2-6: 接收剩余字节 (保留字节、功能码、数据1、数据2、校验和) */
    if (g_gimbal_rx_len < 7U && g_gimbal_rx_len < sizeof(g_gimbal_rx_buf)) {
        g_gimbal_rx_buf[g_gimbal_rx_len++] = byte;

        /* 接收完整帧（7字节）后立即解析（ISR优化） */
        if (g_gimbal_rx_len == 7U) {
            /* 验证Pelco-D帧完整性（校验和、格式） */
            if (gimbal_verify_response((const uint8_t*)g_gimbal_rx_buf, 7)) {
                /* 提取原始数据（整数运算，极快）*/
                uint8_t func_code = g_gimbal_rx_buf[3];
                uint16_t raw_value = ((uint16_t)g_gimbal_rx_buf[4] << 8) | g_gimbal_rx_buf[5];
                bool query_matched = false;  /* 标记是否匹配当前查询类型 */

                /* ISR中快速解析（GD32F4有硬件FPU，浮点运算约2-3周期）*/
                if (func_code == 0x51 || func_code == 0x59) {
                    /* 方位角响应 */
                    float azimuth_value = (float)raw_value / (100.0f * g_gimbal_gain.azimuth_gain);
                    g_gimbal_position.azimuth = azimuth_value;
                    
                    /* 只有当前查询类型是方位角时才标记完成 */
                    if (g_current_query_type == QUERY_TYPE_AZIMUTH) {
                        g_query_result_azimuth = azimuth_value;
                        g_query_data_ready = true;
                        query_matched = true;
                    }
                    /* 否则忽略这个响应（可能是残留数据） */
                } else if (func_code == 0x53 || func_code == 0x5B) {
                    /* 仰角响应 */
                    float elevation_value = ((float)raw_value * GIMBAL_ELEVATION_SCALE) / g_gimbal_gain.elevation_gain;
                    g_gimbal_position.elevation = elevation_value;
                    
                    /* 只有当前查询类型是仰角时才标记完成 */
                    if (g_current_query_type == QUERY_TYPE_ELEVATION) {
                        g_query_result_elevation = elevation_value;
                        g_query_data_ready = true;
                        query_matched = true;
                    }
                    /* 否则忽略这个响应（可能是残留数据） */
                }
                
                /* 只有功能码匹配当前查询类型时，才标记查询完成 */
                /* 避免残留的方位角响应干扰正在进行的仰角查询 */
                if (query_matched) {
                    g_angle_query_active = false;
                    
                    /* 通知等待任务（延迟处理）*/
                    if (s_gimbal_task_handle != NULL && pxHigherPriorityTaskWoken != NULL) {
                        vTaskNotifyGiveFromISR(s_gimbal_task_handle, pxHigherPriorityTaskWoken);
                    }
                }
            }
            
            /* 重置接收缓冲区（快速）*/
            g_gimbal_rx_len = 0;
        }
    }
}


/*!
    \brief    验证Pelco-D响应帧
    \param[in]  buffer: 响应缓冲区
    \param[in]  len: 缓冲区长度
    \retval     bool: 帧是否有效
*/
static bool gimbal_verify_response(const uint8_t* buffer, uint8_t len)
{
    if (!buffer || len < 7) {
        return false;
    }
    
    /* 检查帧头 */
    if (buffer[0] != 0xFF || buffer[1] != 0x01) {
        return false;
    }
    
    /* 检查功能码：支持查询命令回显(0x51/0x53)和查询响应(0x59/0x5B) */
    uint8_t func_code = buffer[3];
    if (func_code != 0x51 && func_code != 0x53 &&  /* 查询命令回显 */
        func_code != 0x59 && func_code != 0x5B) {  /* 查询响应 */
        return false;
    }
    
    /* 验证校验和 */
    uint8_t calc_checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) & 0xFF;
    if (calc_checksum != buffer[6]) {
        return false;
    }
    
    return true;
}

/* ============ 私有函数实现 ============ */

/*!
    \brief    发送Pelco-D命令到云台（使用统一RS485 HAL）
    \detail   通过统一的RS485 HAL层发送7字节Pelco-D命令
              自动处理：DE引脚控制、延时管理、超时保护、线程安全
    \param[in]  cmd: Pelco-D命令数组指针（7字节：帧头+地址+命令+数据+校验）
    \param[in]  len: 命令长度（通常为7）
    \retval     gimbal_result_t: 
                - GIMBAL_OK: 发送成功
                - GIMBAL_INVALID_PARAM: 参数无效
                - GIMBAL_TIMEOUT: 发送超时
                - GIMBAL_BUSY: 总线忙
*/
static gimbal_result_t gimbal_send_command(const uint8_t* cmd, uint8_t len)
{
    /* 参数验证 */
    if (!g_initialized || !cmd || len == 0) {
        return GIMBAL_INVALID_PARAM;
    }

    /* 使用统一的RS485 HAL发送数据（自动处理DE控制和延时，优先使用DMA）*/
    rs485_result_t result = rs485_send_dma_aware(RS485_BUS_SENSOR, cmd, len, 100);
    
    /* 转换返回值 */
    switch (result) {
        case RS485_OK:
            return GIMBAL_OK;
        case RS485_TIMEOUT:
            return GIMBAL_TIMEOUT;
        case RS485_BUSY:
            return GIMBAL_BUSY;
        case RS485_INVALID_PARAM:
        case RS485_NOT_INITIALIZED:
        default:
            return GIMBAL_INVALID_PARAM;
    }
}

/*!
    \brief    计算Pelco-D校验和 (字节1-5的模256和)
    \param[in]  data: 7字节Pelco-D命令数组
    \retval     uint8_t: 校验和
*/
static uint8_t gimbal_calc_checksum(const uint8_t* data)
{
    if (!data) return 0;
    /* 校验和 = (地址 + 命令1 + 命令2 + 数据1 + 数据2) mod 256 */
    return (data[1] + data[2] + data[3] + data[4] + data[5]) & 0xFF;
}

/*!
    \brief    中止当前正在进行的查询（需在临界区内调用）
*/
static void gimbal_abort_query(void)
{
    g_angle_query_active = false;
    g_gimbal_rx_len = 0;
    g_query_data_ready = false;
    g_current_query_type = QUERY_TYPE_NONE;
}

/*!
    \brief    清理查询状态（公开接口，查询超时后调用）
    \detail   在方位角或仰角查询超时后调用，清理上一轮未完成的查询状态
              确保下一次查询能正确启动
*/
void gimbal_clear_query_state(void)
{
    taskENTER_CRITICAL();
    gimbal_abort_query();
    taskEXIT_CRITICAL();
}

/*!
    \brief    将云台移动命令加入队列
    \param[in]  direction: 移动方向
    \retval     bool: 是否成功加入队列
*/
bool gimbal_queue_command(gimbal_direction_t direction)
{
    if (!g_initialized) {
        return false;
    }
    
    /* 将命令加入队列，由主循环处理 */
    g_pending_direction = direction;
    g_has_pending_cmd = true;
    
    /* 立即中断正在进行的角度查询，为控制命令让路 */
    taskENTER_CRITICAL();
    if (g_angle_query_active) {
        gimbal_abort_query();
    }
    taskEXIT_CRITICAL();
    
    return true;
}

/*!
    \brief    将云台位置设置命令加入队列
    \param[in]  azimuth: 水平角度
    \param[in]  elevation: 垂直角度
    \retval     bool: 是否成功加入队列
    \note     只加入队列，不打断正在进行的查询
*/
bool gimbal_queue_set_position(float azimuth, float elevation)
{
    if (!g_initialized) {
        return false;
    }
    
    /* 将位置命令加入队列，由 gimbal_process() 在安全时机执行 */
    /* 不中断正在进行的角度查询，让查询正常完成 */
    g_pending_azimuth = azimuth;
    g_pending_elevation = elevation;
    g_has_pending_position = true;
    
    return true;
}

/* ============ 旧的阻塞式辅助函数已删除 ============ */
/* 使用新的非阻塞接口：gimbal_send_query_xxx() + gimbal_is_xxx_ready() */

/* ============ 非阻塞查询接口实现（新增）============ */

/*!
    \brief    发送方位角查询请求（非阻塞，立即返回）
    \retval   gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_send_query_azimuth(void)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;
    
    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }
    
    /* 准备接收新数据 */
    taskENTER_CRITICAL();
    gimbal_abort_query();          /* 清理上一轮未完成的查询状态 */
    g_angle_query_active = true;
    g_current_query_type = QUERY_TYPE_AZIMUTH;
    taskEXIT_CRITICAL();
    
    /* 发送查询命令（非阻塞）*/
    gimbal_result_t result = gimbal_send_command(CMD_QUERY_AZIMUTH, 7);
    
    /* 释放总线（数据接收在ISR中进行）*/
    rs485_unlock_bus(RS485_BUS_SENSOR);
    
    if (result != GIMBAL_OK) {
        taskENTER_CRITICAL();
        gimbal_abort_query();
        taskEXIT_CRITICAL();
    }
    
    return result;
}

/*!
    \brief    发送仰角查询请求（非阻塞，立即返回）
    \retval   gimbal_result_t: 操作结果
*/
gimbal_result_t gimbal_send_query_elevation(void)
{
    if (!g_initialized) return GIMBAL_NOT_INITIALIZED;
    
    /* 获取总线访问权 */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return GIMBAL_TIMEOUT;
    }
    
    /* 准备接收新数据 */
    taskENTER_CRITICAL();
    gimbal_abort_query();          /* 清理上一轮未完成的查询状态 */
    g_angle_query_active = true;
    g_current_query_type = QUERY_TYPE_ELEVATION;
    taskEXIT_CRITICAL();
    
    /* 发送查询命令（非阻塞）*/
    gimbal_result_t result = gimbal_send_command(CMD_QUERY_ELEVATION, 7);
    
    /* 释放总线（数据接收在ISR中进行）*/
    rs485_unlock_bus(RS485_BUS_SENSOR);
    
    if (result != GIMBAL_OK) {
        taskENTER_CRITICAL();
        gimbal_abort_query();
        taskEXIT_CRITICAL();
    }
    
    return result;
}

/*!
    \brief    检查方位角查询结果是否就绪（非阻塞）
    \param[out] azimuth_deg: 方位角结果（仅在返回true时有效）
    \retval   bool: true=数据就绪，false=数据未就绪
*/
bool gimbal_is_azimuth_ready(float* azimuth_deg)
{
    if (!azimuth_deg) return false;
    
    bool ready = false;
    
    taskENTER_CRITICAL();
    if (g_query_data_ready && g_current_query_type == QUERY_TYPE_AZIMUTH) {
        *azimuth_deg = g_query_result_azimuth;
        ready = true;
        /* 读取后清除标志 */
        g_query_data_ready = false;
        g_current_query_type = QUERY_TYPE_NONE;
        g_angle_query_active = false;
    }
    taskEXIT_CRITICAL();
    
    return ready;
}

/*!
    \brief    检查仰角查询结果是否就绪（非阻塞）
    \param[out] elevation_deg: 仰角结果（仅在返回true时有效）
    \retval   bool: true=数据就绪，false=数据未就绪
*/
bool gimbal_is_elevation_ready(float* elevation_deg)
{
    if (!elevation_deg) return false;
    
    bool ready = false;
    
    taskENTER_CRITICAL();
    if (g_query_data_ready && g_current_query_type == QUERY_TYPE_ELEVATION) {
        *elevation_deg = g_query_result_elevation;
        ready = true;
        /* 读取后清除标志 */
        g_query_data_ready = false;
        g_current_query_type = QUERY_TYPE_NONE;
        g_angle_query_active = false;
    }
    taskEXIT_CRITICAL();
    
    return ready;
}
