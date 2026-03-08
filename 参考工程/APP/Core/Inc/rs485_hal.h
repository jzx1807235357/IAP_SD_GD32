/*!
    \file    rs485_hal.h
    \brief   RS485硬件抽象层 - 统一管理多路RS485总线
    
    本模块提供统一的RS485通信接口，支持：
    - 多路RS485总线管理（USART0, USART5等）
    - 自动控制DE/RE方向切换
    - 线程安全的发送/接收操作（FreeRTOS互斥锁）
    - 超时保护机制
    
    使用示例：
        // 1. 初始化总线
        rs485_config_t config = {
            .usart = USART5,
            .baudrate = 9600,
            .de_port = GPIOB,
            .de_pin = GPIO_PIN_15,
            // ... 其他配置
        };
        rs485_init(RS485_BUS_SENSOR, &config);
        
        // 2. 发送数据 (DMA方式)
        uint8_t data[] = {0xFF, 0x01, 0x02};
        rs485_send_dma_aware(RS485_BUS_SENSOR, data, 3, 100);
        
        // 3. 接收数据
        uint8_t buf[10];
        rs485_recv(RS485_BUS_SENSOR, buf, 10, 500);
    
    \version 2025-01-21, V1.0.0, RS485 HAL for FreeRTOS
*/

#ifndef RS485_HAL_H
#define RS485_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>

/* ==================== 类型定义 ==================== */

/*!
    \brief RS485总线编号
    \note  根据实际硬件配置定义总线数量
*/
typedef enum {
    RS485_BUS_HOST = 0,      /*!< USART5 - 上位机通信总线 */
    RS485_BUS_SENSOR,        /*!< USART2 - 传感器总线（云台+光敏） */
    RS485_BUS_MAX            /*!< 总线数量 */
} rs485_bus_id_t;

/*!
    \brief RS485操作结果
*/
typedef enum {
    RS485_OK = 0,            /*!< 操作成功 */
    RS485_ERROR,             /*!< 一般错误 */
    RS485_TIMEOUT,           /*!< 超时 */
    RS485_BUSY,              /*!< 总线忙 */
    RS485_INVALID_PARAM,     /*!< 参数无效 */
    RS485_NOT_INITIALIZED    /*!< 未初始化 */
} rs485_result_t;

/*!
    \brief RS485总线配置结构体
*/
typedef struct {
    /* USART配置 */
    uint32_t usart;              /*!< USART外设编号 (USART0, USART5等) */
    rcu_periph_enum usart_clk;   /*!< USART时钟 (RCU_USART0, RCU_USART5等) */
    uint32_t baudrate;           /*!< 波特率 */
    
    /* GPIO配置 - TX引脚 */
    uint32_t tx_port;            /*!< TX引脚GPIO端口 (GPIOA, GPIOC等) */
    rcu_periph_enum tx_port_clk; /*!< TX端口时钟 (RCU_GPIOA, RCU_GPIOC等) */
    uint32_t tx_pin;             /*!< TX引脚编号 (GPIO_PIN_x) */
    uint32_t tx_af;              /*!< TX复用功能 (GPIO_AF_x) */
    
    /* GPIO配置 - RX引脚 */
    uint32_t rx_port;            /*!< RX引脚GPIO端口 */
    rcu_periph_enum rx_port_clk; /*!< RX端口时钟 */
    uint32_t rx_pin;             /*!< RX引脚编号 */
    uint32_t rx_af;              /*!< RX复用功能 */
    
    /* GPIO配置 - DE/RE引脚 */
    uint32_t de_port;            /*!< DE/RE引脚GPIO端口 */
    rcu_periph_enum de_port_clk; /*!< DE端口时钟 */
    uint32_t de_pin;             /*!< DE/RE引脚编号 */
    
    /* 中断配置 */
    IRQn_Type irq_type;          /*!< 中断类型 (USART0_IRQn等) */
    uint8_t irq_priority;        /*!< 中断优先级 (5-15, FreeRTOS要求>=5) */
    
    /* 时序配置 */
    uint16_t tx_delay_us;        /*!< 发送前延时（微秒）*/
    uint16_t rx_delay_us;        /*!< 接收前延时（微秒）*/
} rs485_config_t;

/* ==================== 公共函数接口 ==================== */

/*!
    \brief    初始化RS485总线
    \param[in]  bus_id: 总线编号
    \param[in]  config: 总线配置参数
    \retval     rs485_result_t: 初始化结果
*/
rs485_result_t rs485_init(rs485_bus_id_t bus_id, const rs485_config_t* config);

/*!
    \brief    发送数据（已废弃 - 请使用rs485_send_dma_aware）
    \note     此函数保留仅供内部使用，外部模块请使用rs485_send_dma_aware
*/
/* rs485_result_t rs485_send(...) - 已移至内部实现 */

/*!
    \brief    设置RS485方向（发送/接收）
    \param[in]  bus_id: 总线编号
    \param[in]  tx_mode: true=发送模式, false=接收模式
    \retval     rs485_result_t: 操作结果
*/
rs485_result_t rs485_set_direction(rs485_bus_id_t bus_id, bool tx_mode);

/*!
    \brief    获取总线是否已初始化
    \param[in]  bus_id: 总线编号
    \retval     bool: true=已初始化, false=未初始化
*/
bool rs485_is_initialized(rs485_bus_id_t bus_id);

/*!
    \brief    获取USART句柄（供中断使用）
    \param[in]  bus_id: 总线编号
    \retval     uint32_t: USART外设编号
*/
uint32_t rs485_get_usart(rs485_bus_id_t bus_id);

/*!
    \brief    DMA感知的发送函数(上位机总线自动使用DMA)
    \param[in]  bus_id: 总线编号
    \param[in]  data: 发送数据指针
    \param[in]  len: 发送长度
    \param[in]  timeout_ms: 超时时间(毫秒)
    \retval     rs485_result_t
    \note       RS485_BUS_HOST会自动使用DMA发送以减少CPU占用
*/
rs485_result_t rs485_send_dma_aware(rs485_bus_id_t bus_id, const uint8_t* data, uint16_t len, uint32_t timeout_ms);

/* ==================== 总线仲裁API ==================== */

/*!
    \brief    初始化总线仲裁（在rs485_init之后自动调用，无需手动调用）
    \retval     rs485_result_t: 初始化结果
*/
rs485_result_t rs485_arbitration_init(void);

/*!
    \brief    获取总线访问权（用于"发送+接收"完整周期的保护）
    \param[in]  bus_id: 总线编号
    \param[in]  timeout_ms: 等待超时（毫秒），0表示不等待
    \retval     rs485_result_t: RS485_OK=成功, RS485_TIMEOUT=超时, RS485_BUSY=总线忙
    \note       成功后必须调用 rs485_unlock_bus() 释放
    \example    
                // 传感器查询（占用总线直到接收完成）
                if (rs485_lock_bus(RS485_BUS_SENSOR, 200) == RS485_OK) {
                    light_sensor_send_request();
                    light_sensor_wait_for_response(100);
                    rs485_unlock_bus(RS485_BUS_SENSOR);
                }
*/
rs485_result_t rs485_lock_bus(rs485_bus_id_t bus_id, uint32_t timeout_ms);

/*!
    \brief    释放总线访问权
    \param[in]  bus_id: 总线编号
    \note       必须与 rs485_lock_bus() 成对使用
*/
void rs485_unlock_bus(rs485_bus_id_t bus_id);

/*!
    \brief    检查总线是否被占用
    \param[in]  bus_id: 总线编号
    \retval     bool: true=被占用, false=空闲
*/
bool rs485_is_bus_locked(rs485_bus_id_t bus_id);

#ifdef __cplusplus
}
#endif

#endif /* RS485_HAL_H */
