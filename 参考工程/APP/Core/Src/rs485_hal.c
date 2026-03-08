/*!
    \file    rs485_hal.c
    \brief   RS485硬件抽象层实现
    
    \version 2025-01-21, V1.0.0
*/

#include "rs485_hal.h"
#include "rs485_config.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ==================== 私有类型定义 ==================== */

/*!
    \brief RS485总线控制块
*/
typedef struct {
    rs485_config_t config;          /*!< 总线配置 */
    SemaphoreHandle_t tx_mutex;     /*!< 发送互斥锁（保护单次发送操作）*/
    SemaphoreHandle_t bus_mutex;    /*!< 总线仲裁锁（保护完整的发送+接收周期）*/
    bool initialized;               /*!< 初始化标志 */
    bool tx_mode;                   /*!< 当前模式：true=发送, false=接收 */
} rs485_bus_t;

/* ==================== 私有变量 ==================== */

static rs485_bus_t s_buses[RS485_BUS_MAX] = {0};

/* USART5(上位机总线)专用DMA发送缓冲区 - 256字节足够容纳Modbus最大帧 */
static uint8_t s_host_dma_tx_buf[256] __attribute__((aligned(4)));
static volatile bool s_host_dma_tx_busy = false;

/* USART2(传感器总线)专用DMA发送缓冲区 */
static uint8_t s_sensor_dma_tx_buf[RS485_SENSOR_DMA_TX_BUF_SIZE] __attribute__((aligned(4)));
static volatile bool s_sensor_dma_tx_busy = false;

/* ==================== 私有函数声明 ==================== */

static rs485_result_t rs485_hw_init(rs485_bus_t* bus);
static void rs485_set_de_pin(rs485_bus_t* bus, bool tx_mode);
static void rs485_host_dma_init(const rs485_config_t* cfg);
static rs485_result_t rs485_send_dma_host(const uint8_t* data, uint16_t len, uint32_t timeout_ms);
static void rs485_sensor_dma_tx_init(const rs485_config_t* cfg);
static rs485_result_t rs485_send_dma_sensor(const uint8_t* data, uint16_t len, uint32_t timeout_ms);


/* ==================== 公共函数实现 ==================== */

/*!
    \brief    初始化RS485总线
*/
rs485_result_t rs485_init(rs485_bus_id_t bus_id, const rs485_config_t* config)
{
    /* 参数验证 */
    if (bus_id >= RS485_BUS_MAX || config == NULL) {
        return RS485_INVALID_PARAM;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    /* 检查是否已初始化 */
    if (bus->initialized) {
        return RS485_OK;  /* 已初始化，直接返回成功 */
    }
    
    /* 保存配置 */
    memcpy(&bus->config, config, sizeof(rs485_config_t));
    
    /* 创建发送互斥锁 */
    bus->tx_mutex = xSemaphoreCreateMutex();
    if (bus->tx_mutex == NULL) {
        return RS485_ERROR;
    }
    
    /* 创建总线仲裁锁 */
    bus->bus_mutex = xSemaphoreCreateMutex();
    if (bus->bus_mutex == NULL) {
        vSemaphoreDelete(bus->tx_mutex);
        bus->tx_mutex = NULL;
        return RS485_ERROR;
    }
    
    /* 初始化硬件 */
    rs485_result_t result = rs485_hw_init(bus);
    if (result != RS485_OK) {
        vSemaphoreDelete(bus->tx_mutex);
        vSemaphoreDelete(bus->bus_mutex);
        bus->tx_mutex = NULL;
        bus->bus_mutex = NULL;
        return result;
    }
    
    /* 默认设置为接收模式 */
    bus->tx_mode = false;
    rs485_set_de_pin(bus, false);
    
    /* 标记已初始化 */
    bus->initialized = true;
    
    return RS485_OK;
}

/*!
    \brief    设置RS485方向
*/
rs485_result_t rs485_set_direction(rs485_bus_id_t bus_id, bool tx_mode)
{
    if (bus_id >= RS485_BUS_MAX) {
        return RS485_INVALID_PARAM;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    if (!bus->initialized) {
        return RS485_NOT_INITIALIZED;
    }
    
    rs485_set_de_pin(bus, tx_mode);
    bus->tx_mode = tx_mode;
    
    return RS485_OK;
}

/*!
    \brief    获取总线是否已初始化
*/
bool rs485_is_initialized(rs485_bus_id_t bus_id)
{
    if (bus_id >= RS485_BUS_MAX) {
        return false;
    }
    return s_buses[bus_id].initialized;
}

/*!
    \brief    获取USART句柄
*/
uint32_t rs485_get_usart(rs485_bus_id_t bus_id)
{
    if (bus_id >= RS485_BUS_MAX || !s_buses[bus_id].initialized) {
        return 0;
    }
    return s_buses[bus_id].config.usart;
}

/* ==================== 私有函数实现 ==================== */

/*!
    \brief    硬件初始化
*/
static rs485_result_t rs485_hw_init(rs485_bus_t* bus)
{
    rs485_config_t* cfg = &bus->config;
    
    /* 使能时钟 */
    rcu_periph_clock_enable(cfg->tx_port_clk);
    rcu_periph_clock_enable(cfg->rx_port_clk);
    rcu_periph_clock_enable(cfg->de_port_clk);
    rcu_periph_clock_enable(cfg->usart_clk);
    
    /* 配置TX引脚 */
    gpio_mode_set(cfg->tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, cfg->tx_pin);
    gpio_output_options_set(cfg->tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, cfg->tx_pin);
    gpio_af_set(cfg->tx_port, cfg->tx_af, cfg->tx_pin);
    
    /* 配置RX引脚 */
    gpio_mode_set(cfg->rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, cfg->rx_pin);
    gpio_af_set(cfg->rx_port, cfg->rx_af, cfg->rx_pin);
    
    /* 配置DE/RE引脚 */
    gpio_mode_set(cfg->de_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, cfg->de_pin);
    gpio_output_options_set(cfg->de_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, cfg->de_pin);
    
    /* 配置USART */
    usart_deinit(cfg->usart);
    usart_baudrate_set(cfg->usart, cfg->baudrate);
    usart_word_length_set(cfg->usart, USART_WL_8BIT);
    usart_stop_bit_set(cfg->usart, USART_STB_1BIT);
    usart_parity_config(cfg->usart, USART_PM_NONE);
    usart_receive_config(cfg->usart, USART_RECEIVE_ENABLE);
    usart_transmit_config(cfg->usart, USART_TRANSMIT_ENABLE);
    
    /* 只对非DMA总线启用RBNE中断，HOST/SENSOR由各自模块的DMA init控制 */
    if (cfg->usart != RS485_HOST_USART && cfg->usart != RS485_SENSOR_USART) {
        usart_interrupt_enable(cfg->usart, USART_INT_RBNE);
    }
    
    /* 配置NVIC */
    nvic_irq_enable(cfg->irq_type, cfg->irq_priority, 0);
    
    /* 使能USART */
    usart_enable(cfg->usart);
    
    /* 如果是上位机总线(USART5)，初始化DMA发送 */
    if (cfg->usart == RS485_HOST_USART) {
        rs485_host_dma_init(cfg);
    }
    
    /* 如果是传感器总线(USART2)，初始化DMA发送 */
    if (cfg->usart == RS485_SENSOR_USART) {
        rs485_sensor_dma_tx_init(cfg);
    }
    
    return RS485_OK;
}

/*!
    \brief    设置DE引脚电平
*/
static void rs485_set_de_pin(rs485_bus_t* bus, bool tx_mode)
{
    if (tx_mode) {
        gpio_bit_set(bus->config.de_port, bus->config.de_pin);
    } else {
        gpio_bit_reset(bus->config.de_port, bus->config.de_pin);
    }
}

/* ==================== 总线仲裁函数实现 ==================== */

/*!
    \brief    初始化总线仲裁（已在rs485_init中自动完成，此函数保留用于兼容）
*/
rs485_result_t rs485_arbitration_init(void)
{
    /* 仲裁锁在rs485_init中已创建，无需额外操作 */
    return RS485_OK;
}

/*!
    \brief    获取总线访问权
*/
rs485_result_t rs485_lock_bus(rs485_bus_id_t bus_id, uint32_t timeout_ms)
{
    /* 参数检查 */
    if (bus_id >= RS485_BUS_MAX) {
        return RS485_INVALID_PARAM;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    /* 检查初始化 */
    if (!bus->initialized || bus->bus_mutex == NULL) {
        return RS485_NOT_INITIALIZED;
    }
    
    /* 尝试获取总线仲裁锁 */
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(bus->bus_mutex, ticks) != pdTRUE) {
        return RS485_TIMEOUT;  /* 超时或总线忙 */
    }
    
    return RS485_OK;
}

/*!
    \brief    释放总线访问权
*/
void rs485_unlock_bus(rs485_bus_id_t bus_id)
{
    /* 参数检查 */
    if (bus_id >= RS485_BUS_MAX) {
        return;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    /* 检查初始化 */
    if (!bus->initialized || bus->bus_mutex == NULL) {
        return;
    }
    
    /* 释放总线仲裁锁 */
    xSemaphoreGive(bus->bus_mutex);
    
    /* FreeRTOS会自动唤醒等待队列中优先级最高的任务 */
}

/*!
    \brief    检查总线是否被占用
*/
bool rs485_is_bus_locked(rs485_bus_id_t bus_id)
{
    /* 参数检查 */
    if (bus_id >= RS485_BUS_MAX) {
        return false;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    /* 检查初始化 */
    if (!bus->initialized || bus->bus_mutex == NULL) {
        return false;
    }
    
    /* 尝试立即获取锁（不阻塞） */
    if (xSemaphoreTake(bus->bus_mutex, 0) == pdTRUE) {
        /* 成功获取，说明总线空闲 */
        xSemaphoreGive(bus->bus_mutex);  /* 立即释放 */
        return false;
    } else {
        /* 获取失败，说明总线被占用 */
        return true;
    }
}

/* ==================== DMA发送实现(仅USART5上位机总线) ==================== */

/*!
    \brief    初始化USART5的DMA发送功能
    \note     根据GD32F425芯片手册,USART5 TX使用DMA1 Channel7, SUBPERI5
*/
static void rs485_host_dma_init(const rs485_config_t* cfg)
{
    dma_single_data_parameter_struct dma_init;

    /* 使能DMA1时钟 */
    rcu_periph_clock_enable(RCU_DMA1);

    /* 去初始化DMA1 Channel7 */
    dma_single_data_para_struct_init(&dma_init);
    dma_deinit(DMA1, DMA_CH7);

    /* 配置DMA参数 */
    dma_init.direction         = DMA_MEMORY_TO_PERIPH;
    dma_init.memory0_addr      = (uint32_t)s_host_dma_tx_buf;
    dma_init.memory_inc        = DMA_MEMORY_INCREASE_ENABLE;
    dma_init.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init.number            = 0;  /* 发送前再配置实际长度 */
    dma_init.periph_addr       = (uint32_t)&USART_DATA(cfg->usart);
    dma_init.periph_inc        = DMA_PERIPH_INCREASE_DISABLE;
    dma_init.priority          = DMA_PRIORITY_ULTRA_HIGH;

    dma_single_data_mode_init(DMA1, DMA_CH7, &dma_init);
    dma_circulation_disable(DMA1, DMA_CH7);
    
    /* USART5对应DMA_SUBPERI5 */
    dma_channel_subperipheral_select(DMA1, DMA_CH7, DMA_SUBPERI5);

    /* 注意：不在这里使能DMA，发送时再使能 */
}

/*!
    \brief    初始化USART2的DMA发送功能
    \note     根据GD32F425芯片手册,USART2 TX使用DMA0 Channel3, SUBPERI4
*/
static void rs485_sensor_dma_tx_init(const rs485_config_t* cfg)
{
    dma_single_data_parameter_struct dma_init;

    /* 使能DMA0时钟 */
    rcu_periph_clock_enable(RS485_SENSOR_DMA_RCU);

    /* 去初始化DMA0 Channel3 */
    dma_single_data_para_struct_init(&dma_init);
    dma_deinit(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_TX_CHANNEL);

    /* 配置DMA参数 */
    dma_init.direction         = DMA_MEMORY_TO_PERIPH;
    dma_init.memory0_addr      = (uint32_t)s_sensor_dma_tx_buf;
    dma_init.memory_inc        = DMA_MEMORY_INCREASE_ENABLE;
    dma_init.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init.number            = 0;  /* 发送前再配置实际长度 */
    dma_init.periph_addr       = (uint32_t)&USART_DATA(cfg->usart);
    dma_init.periph_inc        = DMA_PERIPH_INCREASE_DISABLE;
    dma_init.priority          = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(RS485_SENSOR_DMA_PERIPH,
                              RS485_SENSOR_DMA_TX_CHANNEL,
                              &dma_init);
    
    /* 不使用循环模式 */
    dma_circulation_disable(RS485_SENSOR_DMA_PERIPH,
                            RS485_SENSOR_DMA_TX_CHANNEL);
    
    /* USART2对应DMA_SUBPERI4 */
    dma_channel_subperipheral_select(RS485_SENSOR_DMA_PERIPH,
                                     RS485_SENSOR_DMA_TX_CHANNEL,
                                     RS485_SENSOR_DMA_TX_SUBPERI);

    /* 注意：不在这里使能DMA，发送时再使能 */
}

/*!
    \brief    上位机总线专用DMA发送
    \param[in]  data: 发送数据指针
    \param[in]  len: 发送长度(不超过256字节)
    \param[in]  timeout_ms: 超时时间(毫秒)
    \retval     rs485_result_t
*/
static rs485_result_t rs485_send_dma_host(const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0 || len > sizeof(s_host_dma_tx_buf)) {
        return RS485_INVALID_PARAM;
    }

    /* 检查DMA是否忙 */
    if (s_host_dma_tx_busy) {
        return RS485_BUSY;
    }

    /* 拷贝到DMA缓冲区 */
    memcpy((void*)s_host_dma_tx_buf, data, len);
    s_host_dma_tx_busy = true;

    /* 设置传输长度 */
    dma_transfer_number_config(DMA1, DMA_CH7, len);

    /* 方向引脚置为发送 */
    rs485_set_de_pin(&s_buses[RS485_BUS_HOST], true);
    delay_us(s_buses[RS485_BUS_HOST].config.tx_delay_us);

    /* 使能USART的DMA发送功能 */
    usart_dma_transmit_config(s_buses[RS485_BUS_HOST].config.usart, USART_TRANSMIT_DMA_ENABLE);

    /* 启动DMA */
    dma_channel_enable(DMA1, DMA_CH7);

    /* 等待DMA传输完成，期间可以让出CPU */
    TickType_t tick_start = xTaskGetTickCount();
    while (RESET == dma_flag_get(DMA1, DMA_CH7, DMA_FLAG_FTF)) {
        if ((xTaskGetTickCount() - tick_start) > pdMS_TO_TICKS(timeout_ms)) {
            /* 超时处理 */
            dma_channel_disable(DMA1, DMA_CH7);
            usart_dma_transmit_config(s_buses[RS485_BUS_HOST].config.usart, USART_TRANSMIT_DMA_DISABLE);
            rs485_set_de_pin(&s_buses[RS485_BUS_HOST], false);
            s_host_dma_tx_busy = false;
            return RS485_TIMEOUT;
        }
        /* 让出CPU给其它任务(云台/传感器) */
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            taskYIELD();
        }
    }

    /* 清除DMA标志 */
    dma_flag_clear(DMA1, DMA_CH7, DMA_FLAG_FTF);
    dma_channel_disable(DMA1, DMA_CH7);
    usart_dma_transmit_config(s_buses[RS485_BUS_HOST].config.usart, USART_TRANSMIT_DMA_DISABLE);

    /* 等待最后一个字节真正发完(TC置位) */
    TickType_t tc_start = xTaskGetTickCount();
    while (usart_flag_get(s_buses[RS485_BUS_HOST].config.usart, USART_FLAG_TC) == RESET) {
        if ((xTaskGetTickCount() - tc_start) > pdMS_TO_TICKS(20)) {
            break;  /* TC等待不超过20ms */
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            taskYIELD();
        }
    }

    /* 切换回接收模式 */
    rs485_set_de_pin(&s_buses[RS485_BUS_HOST], false);
    delay_us(s_buses[RS485_BUS_HOST].config.rx_delay_us);

    s_host_dma_tx_busy = false;
    return RS485_OK;
}

/*!
    \brief    传感器总线专用DMA发送
    \param[in]  data: 发送数据指针
    \param[in]  len: 发送长度(不超过RS485_SENSOR_DMA_TX_BUF_SIZE字节)
    \param[in]  timeout_ms: 超时时间(毫秒)
    \retval     rs485_result_t
*/
static rs485_result_t rs485_send_dma_sensor(const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0 || len > RS485_SENSOR_DMA_TX_BUF_SIZE) {
        return RS485_INVALID_PARAM;
    }

    /* 检查DMA是否忙 */
    if (s_sensor_dma_tx_busy) {
        return RS485_BUSY;
    }

    /* 拷贝到DMA缓冲区 */
    memcpy((void*)s_sensor_dma_tx_buf, data, len);
    s_sensor_dma_tx_busy = true;

    /* 设置传输长度 */
    dma_transfer_number_config(RS485_SENSOR_DMA_PERIPH, 
                               RS485_SENSOR_DMA_TX_CHANNEL, 
                               len);

    /* 方向引脚置为发送 */
    rs485_set_de_pin(&s_buses[RS485_BUS_SENSOR], true);
    delay_us(s_buses[RS485_BUS_SENSOR].config.tx_delay_us);

    /* 使能USART的DMA发送功能 */
    usart_dma_transmit_config(s_buses[RS485_BUS_SENSOR].config.usart, USART_TRANSMIT_DMA_ENABLE);

    /* 启动DMA */
    dma_channel_enable(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_TX_CHANNEL);

    /* 等待DMA传输完成，期间可以让出CPU */
    TickType_t tick_start = xTaskGetTickCount();
    while (RESET == dma_flag_get(RS485_SENSOR_DMA_PERIPH, 
                                 RS485_SENSOR_DMA_TX_CHANNEL, 
                                 DMA_FLAG_FTF)) {
        if ((xTaskGetTickCount() - tick_start) > pdMS_TO_TICKS(timeout_ms)) {
            /* 超时处理 */
            dma_channel_disable(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_TX_CHANNEL);
            usart_dma_transmit_config(s_buses[RS485_BUS_SENSOR].config.usart, USART_TRANSMIT_DMA_DISABLE);
            rs485_set_de_pin(&s_buses[RS485_BUS_SENSOR], false);
            s_sensor_dma_tx_busy = false;
            return RS485_TIMEOUT;
        }
        /* 让出CPU给其它任务 */
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            taskYIELD();
        }
    }

    /* 清除DMA标志 */
    dma_flag_clear(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_TX_CHANNEL, DMA_FLAG_FTF);
    dma_channel_disable(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_TX_CHANNEL);
    usart_dma_transmit_config(s_buses[RS485_BUS_SENSOR].config.usart, USART_TRANSMIT_DMA_DISABLE);

    /* 等待最后一个字节真正发完(TC置位) */
    TickType_t tc_start = xTaskGetTickCount();
    while (usart_flag_get(s_buses[RS485_BUS_SENSOR].config.usart, USART_FLAG_TC) == RESET) {
        if ((xTaskGetTickCount() - tc_start) > pdMS_TO_TICKS(20)) {
            break;  /* TC等待不超过20ms */
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            taskYIELD();
        }
    }

    /* 切换回接收模式 */
    rs485_set_de_pin(&s_buses[RS485_BUS_SENSOR], false);
    delay_us(s_buses[RS485_BUS_SENSOR].config.rx_delay_us);

    s_sensor_dma_tx_busy = false;
    return RS485_OK;
}

/*!
    \brief    DMA感知的RS485发送接口（纯DMA实现）
    \param[in]  bus_id: 总线ID
    \param[in]  data: 发送数据指针
    \param[in]  len: 数据长度
    \param[in]  timeout_ms: 超时时间（ms）
    \retval   rs485_result_t: 操作结果
    \note     HOST/SENSOR总线必须使用DMA，不再支持轮询发送
*/
rs485_result_t rs485_send_dma_aware(rs485_bus_id_t bus_id, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
    /* 参数验证 */
    if (bus_id >= RS485_BUS_MAX || data == NULL || len == 0) {
        return RS485_INVALID_PARAM;
    }
    
    rs485_bus_t* bus = &s_buses[bus_id];
    
    /* 检查初始化 */
    if (!bus->initialized || bus->tx_mutex == NULL) {
        return RS485_NOT_INITIALIZED;
    }
    
    /* 检查RTOS是否运行 - DMA发送依赖FreeRTOS信号量 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return RS485_NOT_INITIALIZED;  /* RTOS未启动，无法使用DMA */
    }
    
    /* 获取发送互斥锁 */
    if (xSemaphoreTake(bus->tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return RS485_BUSY;
    }
    
    rs485_result_t result;
    
    /* HOST总线：使用DMA1 CH7发送 */
    if (bus_id == RS485_BUS_HOST) {
        result = rs485_send_dma_host(data, len, timeout_ms);
        xSemaphoreGive(bus->tx_mutex);
        return result;
    }
    
    /* SENSOR总线：使用DMA0 CH3发送 */
    if (bus_id == RS485_BUS_SENSOR) {
        result = rs485_send_dma_sensor(data, len, timeout_ms);
        xSemaphoreGive(bus->tx_mutex);
        return result;
    }
    
    /* 其他总线ID无效或未实现 */
    xSemaphoreGive(bus->tx_mutex);
    return RS485_INVALID_PARAM;
}
