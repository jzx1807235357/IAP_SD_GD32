/*!
    \file    debug_uart.h
    \brief   Debug UART for printf redirection (USART0)

    \version 2025-03-07, BootLoader version
*/

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

/* Debug UART configuration - USART0 */
#define DEBUG_UART              USART0
#define DEBUG_UART_CLK          RCU_USART0
#define DEBUG_UART_BAUDRATE     115200U

#define DEBUG_UART_TX_PORT      GPIOA
#define DEBUG_UART_TX_PORT_CLK  RCU_GPIOA
#define DEBUG_UART_TX_PIN       GPIO_PIN_9
#define DEBUG_UART_TX_AF        GPIO_AF_7

#define DEBUG_UART_RX_PORT      GPIOA
#define DEBUG_UART_RX_PORT_CLK  RCU_GPIOA
#define DEBUG_UART_RX_PIN       GPIO_PIN_10
#define DEBUG_UART_RX_AF        GPIO_AF_7

/* Function declarations */
void debug_uart_init(void);

#endif /* DEBUG_UART_H */
