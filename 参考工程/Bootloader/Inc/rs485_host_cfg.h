#ifndef RS485_HOST_CFG_H
#define RS485_HOST_CFG_H

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
#define RS485_HOST_RX_DELAY_US     100U  /* 增加到100us确保RS485芯片完全切换 */

/* Modbus configuration */
#define MODBUS_SLAVE_ADDRESS        0x01U  /* 可修改：Modbus从机地址，范围1-247 */

#endif

