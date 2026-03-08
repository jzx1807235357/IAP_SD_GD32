/*!
    \file    rs485_config.h
    \brief   RS485总线配置 - 硬件相关配置集中管理
    
    本文件集中定义所有RS485总线的硬件配置，方便统一管理和修改
    
    \version 2025-01-21, V1.0.0
*/

#ifndef RS485_CONFIG_H
#define RS485_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs485_hal.h"

/* ==================== USART5总线配置（上位机通信）==================== */

#define RS485_HOST_USART           USART5
#define RS485_HOST_USART_CLK       RCU_USART5
#define RS485_HOST_BAUDRATE        115200U

#define RS485_HOST_TX_PORT         GPIOC
#define RS485_HOST_TX_PORT_CLK     RCU_GPIOC
#define RS485_HOST_TX_PIN          GPIO_PIN_6
#define RS485_HOST_TX_AF           GPIO_AF_8

#define RS485_HOST_RX_PORT         GPIOC
#define RS485_HOST_RX_PORT_CLK     RCU_GPIOC
#define RS485_HOST_RX_PIN          GPIO_PIN_7
#define RS485_HOST_RX_AF           GPIO_AF_8

#define RS485_HOST_DE_PORT         GPIOC
#define RS485_HOST_DE_PORT_CLK     RCU_GPIOC
#define RS485_HOST_DE_PIN          GPIO_PIN_9

#define RS485_HOST_IRQ             USART5_IRQn
#define RS485_HOST_IRQ_PRIO        5

#define RS485_HOST_TX_DELAY_US     100U
#define RS485_HOST_RX_DELAY_US     50U

/* ==================== USART2总线配置（传感器总线）==================== */

#define RS485_SENSOR_USART         USART2
#define RS485_SENSOR_USART_CLK     RCU_USART2
#define RS485_SENSOR_BAUDRATE      9600U

#define RS485_SENSOR_TX_PORT       GPIOB
#define RS485_SENSOR_TX_PORT_CLK   RCU_GPIOB
#define RS485_SENSOR_TX_PIN        GPIO_PIN_10
#define RS485_SENSOR_TX_AF         GPIO_AF_7

#define RS485_SENSOR_RX_PORT       GPIOC
#define RS485_SENSOR_RX_PORT_CLK   RCU_GPIOC
#define RS485_SENSOR_RX_PIN        GPIO_PIN_5
#define RS485_SENSOR_RX_AF         GPIO_AF_7

#define RS485_SENSOR_DE_PORT       GPIOA
#define RS485_SENSOR_DE_PORT_CLK   RCU_GPIOA
#define RS485_SENSOR_DE_PIN        GPIO_PIN_7

#define RS485_SENSOR_IRQ           USART2_IRQn
#define RS485_SENSOR_IRQ_PRIO      5

#define RS485_SENSOR_TX_DELAY_US   100U
#define RS485_SENSOR_RX_DELAY_US   50U

/* ==================== USART2传感器总线DMA配置 ==================== */

/* DMA外设和时钟：使用DMA0 */
#define RS485_SENSOR_DMA_RCU          RCU_DMA0
#define RS485_SENSOR_DMA_PERIPH       DMA0

/* 通道映射：USART2 TX -> DMA0 CH3, RX -> DMA0 CH1 */
#define RS485_SENSOR_DMA_TX_CHANNEL   DMA_CH3
#define RS485_SENSOR_DMA_RX_CHANNEL   DMA_CH1

/* Subperipheral选择：USART2对应SUBPERI4 */
#define RS485_SENSOR_DMA_TX_SUBPERI   DMA_SUBPERI4
#define RS485_SENSOR_DMA_RX_SUBPERI   DMA_SUBPERI4

/* 发送缓冲区大小：256字节足够容纳最大Modbus帧 */
#define RS485_SENSOR_DMA_TX_BUF_SIZE  256

/* ==================== 辅助宏定义 ==================== */

/*!
    \brief    快速创建USART5总线配置
*/
#define RS485_HOST_CONFIG_INIT() {                  \
    .usart = RS485_HOST_USART,                      \
    .usart_clk = RS485_HOST_USART_CLK,              \
    .baudrate = RS485_HOST_BAUDRATE,                \
    .tx_port = RS485_HOST_TX_PORT,                  \
    .tx_port_clk = RS485_HOST_TX_PORT_CLK,          \
    .tx_pin = RS485_HOST_TX_PIN,                    \
    .tx_af = RS485_HOST_TX_AF,                      \
    .rx_port = RS485_HOST_RX_PORT,                  \
    .rx_port_clk = RS485_HOST_RX_PORT_CLK,          \
    .rx_pin = RS485_HOST_RX_PIN,                    \
    .rx_af = RS485_HOST_RX_AF,                      \
    .de_port = RS485_HOST_DE_PORT,                  \
    .de_port_clk = RS485_HOST_DE_PORT_CLK,          \
    .de_pin = RS485_HOST_DE_PIN,                    \
    .irq_type = RS485_HOST_IRQ,                     \
    .irq_priority = RS485_HOST_IRQ_PRIO,            \
    .tx_delay_us = RS485_HOST_TX_DELAY_US,          \
    .rx_delay_us = RS485_HOST_RX_DELAY_US           \
}

/*!
    \brief    快速创建USART2总线配置
*/
#define RS485_SENSOR_CONFIG_INIT() {                \
    .usart = RS485_SENSOR_USART,                    \
    .usart_clk = RS485_SENSOR_USART_CLK,            \
    .baudrate = RS485_SENSOR_BAUDRATE,              \
    .tx_port = RS485_SENSOR_TX_PORT,                \
    .tx_port_clk = RS485_SENSOR_TX_PORT_CLK,        \
    .tx_pin = RS485_SENSOR_TX_PIN,                  \
    .tx_af = RS485_SENSOR_TX_AF,                    \
    .rx_port = RS485_SENSOR_RX_PORT,                \
    .rx_port_clk = RS485_SENSOR_RX_PORT_CLK,        \
    .rx_pin = RS485_SENSOR_RX_PIN,                  \
    .rx_af = RS485_SENSOR_RX_AF,                    \
    .de_port = RS485_SENSOR_DE_PORT,                \
    .de_port_clk = RS485_SENSOR_DE_PORT_CLK,        \
    .de_pin = RS485_SENSOR_DE_PIN,                  \
    .irq_type = RS485_SENSOR_IRQ,                   \
    .irq_priority = RS485_SENSOR_IRQ_PRIO,          \
    .tx_delay_us = RS485_SENSOR_TX_DELAY_US,        \
    .rx_delay_us = RS485_SENSOR_RX_DELAY_US         \
}

#ifdef __cplusplus
}
#endif

#endif /* RS485_CONFIG_H */
