/*
 * ============================================================================
 * light_sensor.c - 四象限光敏传感器驱动（RS485通信）
 * ============================================================================
 *
 * 【功能概述】
 * 本模块实现与四象限光敏传感器的RS485通信，用于太阳追踪系统
 * 通过自定义协议读取4个光电二极管的电压值，用于计算太阳偏移方向
 *
 * 【传感器布局】
 *        [0] 上方
 *         ↑
 *    [1]左 ● 右[2]
 *         ↓
 *        [3] 下方
 *
 * 【通信协议】
 * 请求帧格式（5字节）：
 *   [0xAA] [0x55] [0x01] [0x02] [校验和]
 *    帧头1   帧头2   地址    长度   XOR校验
 *
 * 响应帧格式（21字节）：
 *   [0xAA] [0x55] [0x01] [0x10] [数据16字节] [校验和]
 *    帧头1   帧头2   地址    长度    4个float   XOR校验
 *
 * 【数据格式】
 * 数据段包含4个float（IEEE 754，小端序），每个4字节
 * - float[0]: 上方传感器电压
 * - float[1]: 左侧传感器电压
 * - float[2]: 右侧传感器电压
 * - float[3]: 下方传感器电压
 *
 * 【通信流程】
 * 1. 发送请求 → 2. 等待响应 → 3. 接收数据 → 4. 校验并解析
 *
 * 【FreeRTOS适配】
 * - 使用任务通知实现异步等待
 * - 状态机处理接收逻辑
 * - 临界区保护共享数据
 * - 支持超时处理
 *
 * 【依赖模块】
 * - rs485_hal: 统一的RS485硬件抽象层
 * - systick: 时间戳和超时管理
 *
 * ============================================================================
 * \version 2024-12-21, V3.0.0, FreeRTOS优化版
 * ============================================================================
 */

#include "light_sensor.h"
#include "rs485_hal.h"
#include "gd32f4xx.h"
#include "systick.h"
#include "main.h"
#include "task.h"
#include <string.h>
#include <stdbool.h>

/* ========== 全局数据存储（对外可见）========== */
sensor_data_t g_sensor_data = {0};                               /* 传感器数据（唯一存储）*/

/* ========== 内部状态变量 ========== */
static volatile bool s_waiting_response = false;                 /* 等待响应标志 */
static TaskHandle_t s_waiting_task = NULL;                       /* 等待响应的任务句柄 */

/* ========== 接收状态机 ========== */
typedef enum {
    RX_WAIT_AA = 0,    /* 等待帧头1：0xAA */
    RX_WAIT_55,        /* 等待帧头2：0x55 */
    RX_WAIT_ADDR,      /* 等待地址：0x01 */
    RX_WAIT_LEN,       /* 等待长度字节 */
    RX_READ_DATA       /* 接收数据和校验和 */
} rx_state_t;

static volatile rx_state_t s_rx_state = RX_WAIT_AA;     /* 当前接收状态 */
static volatile uint8_t s_rx_buf[32];                   /* 接收缓冲区 */
static volatile uint16_t s_rx_index = 0;                /* 接收索引 */
static volatile uint8_t s_expected_len = 0;             /* 预期数据长度 */
static volatile uint16_t s_expected_total = 0;          /* 预期总长度（含帧头校验）*/

/* 高层通信状态（一次"请求+接收"的会话） */
typedef enum {
    SENSOR_COMM_STATE_IDLE = 0,
    SENSOR_COMM_STATE_WAITING
} sensor_comm_state_t;

static volatile sensor_comm_state_t s_comm_state = SENSOR_COMM_STATE_IDLE;
static volatile uint32_t s_comm_start_tick = 0;

static uint8_t calculate_checksum(const uint8_t* data, uint16_t len);
static inline void reset_rx_state(void);

/*!
    \brief    初始化光敏传感器模块
    \detail   硬件初始化由rs485_hal模块统一管理，本函数只初始化逻辑状态
              必须先调用rs485_init(RS485_BUS_SENSOR, ...)初始化RS485总线
*/
bool light_sensor_init(void)
{
    /* 验证RS485总线已初始化 */
    if (!rs485_is_initialized(RS485_BUS_SENSOR)) {
        return false;
    }
    
    /* 初始化逻辑状态 */
    s_waiting_response = false;
    reset_rx_state();

    return true;
}

/*!
    \brief    发送传感器数据请求
    \retval   bool: 是否成功发送
    \detail   工作流程：
              1. 检查是否已有请求在处理中
              2. 如果超时（>500ms），重置状态
              3. 构建请求帧（5字节）
              4. 通过RS485 HAL发送
              5. 设置等待响应标志
              
              请求帧格式：
              [0xAA][0x55][0x01][0x02][校验和]
    \note     本函数是非阻塞的，发送后需调用 light_sensor_wait_for_response() 等待
*/
bool light_sensor_send_request(void)
{
    static uint32_t last_send_time = 0;

    /* 防止重复请求：如果已有请求在处理，检查是否超时 */
    if (s_waiting_response) {
        uint32_t now = systick_get_tick();
        if ((now - last_send_time) > 500) {
            /* 超时重置：上次请求超过500ms未响应，强制重置 */
            s_waiting_response = false;
            reset_rx_state();
        } else {
            return false;  /* 正在等待响应，拒绝新请求 */
        }
    }

    /* 准备任务通知机制（用于异步等待响应） */
    TaskHandle_t current_task = NULL;
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        current_task = xTaskGetCurrentTaskHandle();  /* 获取当前任务句柄 */
        (void)ulTaskNotifyTake(pdTRUE, 0);           /* 清空之前的通知 */
    }

    s_waiting_task = current_task;
    last_send_time = systick_get_tick();

    /* 构建请求帧（5字节）*/
    uint8_t cmd[5];
    cmd[0] = 0xAA;  /* 帧头1：固定0xAA */
    cmd[1] = 0x55;  /* 帧头2：固定0x55 */
    cmd[2] = 0x01;  /* 设备地址：0x01 */
    cmd[3] = 0x02;  /* 数据长度：2字节（地址+长度） */
    cmd[4] = calculate_checksum(&cmd[2], 2);  /* XOR校验和 */

    /* 使用统一的RS485 HAL发送（自动处理DE控制和延时）*/
    s_waiting_response = true;
    
    rs485_result_t result = rs485_send_dma_aware(RS485_BUS_SENSOR, cmd, 5, 100);
    
    if (result != RS485_OK) {
        s_waiting_response = false;
        return false;
    }
    
    return true;
}


/*!
    \brief    等待传感器响应（阻塞式）
    \param[in]  timeout_ms: 超时时间（毫秒）
    \retval   bool: 是否成功接收到有效数据
    \detail   两种等待模式：
              模式1（FreeRTOS任务）：使用任务通知阻塞等待
              模式2（轮询模式）：循环检查标志位
              
              超时处理：
              - 超时后自动重置状态机
              - 返回false表示超时或失败
    \note     必须在 light_sensor_send_request() 之后调用
*/
bool light_sensor_wait_for_response(uint32_t timeout_ms)
{
    /* 模式1：FreeRTOS任务通知（推荐，高效） */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED && s_waiting_task == xTaskGetCurrentTaskHandle()) {
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        if (timeout_ms > 0U && ticks == 0U) {
            ticks = 1U;  /* 至少等待1个tick */
        }

        /* 阻塞等待任务通知（ISR会在接收完成时发送通知） */
        uint32_t notify = ulTaskNotifyTake(pdTRUE, ticks);
        s_waiting_task = NULL;
        if (notify == 0U) {
            /* 超时：未收到通知 */
            s_waiting_response = false;
            reset_rx_state();
            return false;
        }
        return g_sensor_data.valid;  /* 返回数据是否有效 */
    }

    /* 模式2：轮询等待（备用方案，适用于非任务上下文） */
    uint32_t start = systick_get_tick();
    while (s_waiting_response) {
        if ((systick_get_tick() - start) >= timeout_ms) {
            /* 超时：等待时间超过设定值 */
            s_waiting_response = false;
            reset_rx_state();
            return false;
        }
        taskYIELD();  /* 让出CPU给其他任务 */
    }
    return g_sensor_data.valid;  /* 返回数据是否有效 */
}

void light_sensor_request_abort(void)
{
    s_waiting_response = false;
    reset_rx_state();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED && s_waiting_task != NULL) {
        xTaskNotifyGive(s_waiting_task);
        s_waiting_task = NULL;
    }
}

/*!\n    \brief    计算XOR校验和
    \param[in]  data: 数据指针
    \param[in]  len: 数据长度
    \retval   uint8_t: XOR校验和
    \detail   算法：对所有字节进行异或运算
              校验和 = data[0] ^ data[1] ^ ... ^ data[len-1]
*/
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len)
{
    uint8_t checksum = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];  /* 逐字节异或 */
    }
    return checksum;
}

/*!\n    \brief    重置接收状态机到初始状态
    \detail   清空所有接收相关的状态变量，准备接收新帧
*/
static inline void reset_rx_state(void)
{
    s_rx_state = RX_WAIT_AA;     /* 回到等待帧头1状态 */
    s_rx_index = 0;              /* 清空接收索引 */
    s_expected_len = 0;          /* 清空预期长度 */
    s_expected_total = 0;        /* 清空预期总长度 */
}


/*!
    \brief    USART2接收中断处理（ISR中调用）
    \param[in]  byte: 接收到的字节
    \param[in/out]  pxHigherPriorityTaskWoken: FreeRTOS上下文切换标志
    \detail   状态机处理接收流程：
              
              状态转换流程：
              RX_WAIT_AA → RX_WAIT_55 → RX_WAIT_ADDR → RX_WAIT_LEN → RX_READ_DATA
                ↓           ↓            ↓              ↓              ↓
              等0xAA      等0x55       等0x01         读长度        读数据+校验
              
              完整帧结构（21字节）：
              [AA][55][01][10][float×4=16字节][校验]
    \note     本函数在ISR中执行，必须快速返回
*/
void light_sensor_on_byte(uint8_t byte, BaseType_t *pxHigherPriorityTaskWoken)
{
    /* 只在等待响应时处理接收数据 */
    if (!s_waiting_response) {
        return;
    }

    /* 状态机：逐字节解析协议帧 */
    switch (s_rx_state) {
        case RX_WAIT_AA:  /* 状态0：等待帧头1 */
            if (byte == 0xAA) {
                s_rx_buf[0] = byte;
                s_rx_index = 1;
                s_rx_state = RX_WAIT_55;  /* 转到下一状态 */
            }
            /* 非0xAA字节被忽略，继续等待 */
            break;

        case RX_WAIT_55:  /* 状态1：等待帧头2 */
            if (byte == 0x55) {
                s_rx_buf[s_rx_index++] = byte;
                s_rx_state = RX_WAIT_ADDR;  /* 转到下一状态 */
            } else {
                /* 不是0x55，重新开始 */
                s_rx_state = RX_WAIT_AA;
                s_rx_index = 0;
            }
            break;

        case RX_WAIT_ADDR:  /* 状态2：等待地址字节 */
            if (byte == 0x01) {
                s_rx_buf[s_rx_index++] = byte;
                s_rx_state = RX_WAIT_LEN;  /* 转到下一状态 */
            } else {
                /* 地址不匹配，重新开始 */
                s_rx_state = RX_WAIT_AA;
                s_rx_index = 0;
            }
            break;

        case RX_WAIT_LEN:  /* 状态3：读取数据长度 */
            s_rx_buf[s_rx_index++] = byte;
            s_expected_len = byte;  /* 保存数据段长度 */
            /* 计算总帧长：4(帧头+地址+长度) + 数据长度 + 1(校验) */
            s_expected_total = 4U + (uint16_t)s_expected_len + 1U;
            if (s_expected_total <= sizeof(s_rx_buf)) {
                s_rx_state = RX_READ_DATA;  /* 长度合法，继续接收数据 */
            } else {
                /* 长度超出缓冲区，丢弃此帧 */
                s_rx_state = RX_WAIT_AA;
                s_rx_index = 0;
                s_expected_len = 0;
                s_expected_total = 0;
            }
            break;

        case RX_READ_DATA:  /* 状态4：接收数据段和校验和 */
            s_rx_buf[s_rx_index++] = byte;
            if (s_rx_index >= s_expected_total) {
                /* 完整帧接收完毕，开始验证 */
                /* 验证帧格式和长度 */
                if (s_expected_len == 0x10 && s_expected_total == 21U && 
                    s_rx_buf[0] == 0xAA && s_rx_buf[1] == 0x55 && s_rx_buf[2] == 0x01) {
                    
                    /* 复制到局部缓冲区（避免volatile访问开销） */
                    uint8_t local_buf[21];
                    memcpy(local_buf, (const void *)s_rx_buf, 21U);
                    
                    /* 计算并验证校验和 */
                    uint8_t checksum = calculate_checksum(&local_buf[2], (uint16_t)(s_expected_len + 2U));
                    if (checksum == local_buf[20]) {
                        /* 校验成功：解析4个float并正确映射到结构体字段 */
                        /* 协议数据顺序: float[0]=上, float[1]=左, float[2]=下, float[3]=右 */
                        float* data = (float*)&local_buf[4];
                        g_sensor_data.up_voltage = data[0];     /* data[0]=上 */
                        g_sensor_data.left_voltage = data[1];   /* data[1]=左 */
                        g_sensor_data.down_voltage = data[2];   /* data[2]=下 */
                        g_sensor_data.right_voltage = data[3];  /* data[3]=右 */
                        g_sensor_data.valid = true;
                    } else {
                        /* 校验失败：标记数据无效 */
                        g_sensor_data.valid = false;
                    }
                } else {
                    /* 帧格式错误：标记数据无效 */
                    g_sensor_data.valid = false;
                }

                /* 重置状态 */
                s_waiting_response = false;
                s_rx_state = RX_WAIT_AA;  /* 重置状态机 */
                s_rx_index = 0;
                s_expected_len = 0;
                s_expected_total = 0;
                
                /* 通知等待的任务（FreeRTOS任务通知） */
                if (s_waiting_task != NULL && pxHigherPriorityTaskWoken != NULL) {
                    vTaskNotifyGiveFromISR(s_waiting_task, pxHigherPriorityTaskWoken);
                    s_waiting_task = NULL;
                }
            }
            break;
    }
}


/*!
    \brief    获取四路传感器的平均光强
    \retval   float: 平均光强（单位：V）
    \detail   计算公式：(上+左+下+右) / 4
              用于光控阈值判断
*/
float light_sensor_get_total_intensity(void)
{
    if (!g_sensor_data.valid) {
        return 0.0f;
    }
    
    float avg;
    taskENTER_CRITICAL();
    avg = (g_sensor_data.up_voltage + g_sensor_data.left_voltage + 
          g_sensor_data.down_voltage + g_sensor_data.right_voltage) / 4.0f;
    taskEXIT_CRITICAL();
    return avg;
}

/* ============================================================================
 * 高层传感器通信API (sensor_comm_*)
 * ============================================================================
 *
 * 【功能说明】
 * 提供简化的传感器通信接口，封装底层的发送、接收和超时处理逻辑
 * 适用于需要一次性完成"请求-等待-获取"流程的场景
 *
 * 【使用方法】
 * 1. 初始化：sensor_comm_init()
 * 2. 处理通信：sensor_comm_process(&data)  // 阻塞式，包含请求+等待+解析
 * 3. 获取数据：sensor_comm_get_data(&data)  // 非阻塞，读取缓存数据
 *
 * 【与底层API的关系】
 * 高层API内部调用底层API：
 * sensor_comm_process() = light_sensor_send_request() + 
 *                         light_sensor_wait_for_response()
 * 数据由ISR直接写入全局变量 g_sensor_data
 *
 * ============================================================================ */

/*!
 * \brief  Initialize sensor communication module
 */
bool sensor_comm_init(void)
{
    memset(&g_sensor_data, 0, sizeof(sensor_data_t));
    g_sensor_data.valid = false;

    s_comm_state = SENSOR_COMM_STATE_IDLE;
    s_comm_start_tick = 0;
    
    return true;
}

/*!
 * \brief  Get sensor data
 */
bool sensor_comm_get_data(sensor_data_t* data)
{
    if (data == NULL) {
        return false;
    }
    
    *data = g_sensor_data;
    return g_sensor_data.valid;
}

/*!
 * \brief  Start async sensor communication (non-blocking)
 * \return true if started successfully, false if failed or busy
 */
bool sensor_comm_start_async(void)
{
    /* 仅在空闲状态下允许启动新会话 */
    if (s_comm_state != SENSOR_COMM_STATE_IDLE) {
        return false;
    }

    /* 获取总线访问权（等待最多200ms） */
    if (rs485_lock_bus(RS485_BUS_SENSOR, 200) != RS485_OK) {
        return false;
    }

    /* 发送请求 */
    if (!light_sensor_send_request()) {
        rs485_unlock_bus(RS485_BUS_SENSOR);
        return false;
    }

    s_comm_state = SENSOR_COMM_STATE_WAITING;
    s_comm_start_tick = systick_get_tick();
    return true;
}

/*!
 * \brief  Poll async sensor communication
 * \param  now: Current tick count
 * \param  out_data: Output data structure (can be NULL)
 * \param  done: Pointer to completion flag (can be NULL)
 * \return true if data received successfully, false otherwise
 * \note   Call this periodically after sensor_comm_start_async()
 */
bool sensor_comm_poll_async(uint32_t now, sensor_data_t* out_data, bool* done)
{
    if (done != NULL) {
        *done = false;
    }

    /* 当前没有会话在进行 */
    if (s_comm_state != SENSOR_COMM_STATE_WAITING) {
        if (done != NULL) {
            *done = true;
        }
        return false;
    }

    /* ISR 已经结束接收（s_waiting_response 在 ISR 或 abort 中被清零） */
    if (!s_waiting_response) {
        bool ok = g_sensor_data.valid;

        if (ok && out_data != NULL) {
            taskENTER_CRITICAL();
            *out_data = g_sensor_data;
            taskEXIT_CRITICAL();
        }

        s_comm_state = SENSOR_COMM_STATE_IDLE;
        rs485_unlock_bus(RS485_BUS_SENSOR);

        if (done != NULL) {
            *done = true;
        }
        return ok;
    }

    /* 超时：等待时间超过 SENSOR_COMM_TIMEOUT_MS */
    if ((now - s_comm_start_tick) > SENSOR_COMM_TIMEOUT_MS) {
        light_sensor_request_abort();      /* 重置接收状态机 */
        s_comm_state = SENSOR_COMM_STATE_IDLE;
        rs485_unlock_bus(RS485_BUS_SENSOR);

        if (done != NULL) {
            *done = true;
        }
        return false;
    }

    /* 还在等待，不完成 */
    return false;
}

