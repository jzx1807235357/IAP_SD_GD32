/*!
    \file    debug_uart.c
    \brief   Debug UART for printf redirection (USART0)

    \version 2025-03-07, BootLoader version
*/

#include "debug_uart.h"
#include "gd32f4xx.h"
#include "gd32f4xx_usart.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_rcu.h"
#include <stdio.h>

/*!
    \brief      initialize debug UART (USART0)
*/
void debug_uart_init(void)
{
    /* Enable GPIO and USART clocks */
    rcu_periph_clock_enable(DEBUG_UART_TX_PORT_CLK);
    rcu_periph_clock_enable(DEBUG_UART_RX_PORT_CLK);
    rcu_periph_clock_enable(DEBUG_UART_CLK);

    /* Configure TX pin (PA9) as alternate function push-pull */
    gpio_mode_set(DEBUG_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, DEBUG_UART_TX_PIN);
    gpio_output_options_set(DEBUG_UART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, DEBUG_UART_TX_PIN);
    gpio_af_set(DEBUG_UART_TX_PORT, DEBUG_UART_TX_AF, DEBUG_UART_TX_PIN);

    /* Configure RX pin (PA10) as alternate function */
    gpio_mode_set(DEBUG_UART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, DEBUG_UART_RX_PIN);
    gpio_af_set(DEBUG_UART_RX_PORT, DEBUG_UART_RX_AF, DEBUG_UART_RX_PIN);

    /* Configure USART */
    usart_deinit(DEBUG_UART);
    usart_baudrate_set(DEBUG_UART, DEBUG_UART_BAUDRATE);
    usart_word_length_set(DEBUG_UART, USART_WL_8BIT);
    usart_stop_bit_set(DEBUG_UART, USART_STB_1BIT);
    usart_parity_config(DEBUG_UART, USART_PM_NONE);
    usart_receive_config(DEBUG_UART, USART_RECEIVE_ENABLE);
    usart_transmit_config(DEBUG_UART, USART_TRANSMIT_ENABLE);
    usart_enable(DEBUG_UART);
}

/*!
    \brief      redirect fputc to USART0 for printf
    \note       This function is called by printf in Keil MDK (requires MicroLIB)
*/
int fputc(int ch, FILE *f)
{
    (void)f;  /* Prevent unused parameter warning */
    
    /* 嵌入式终端换行符处理: '\n' -> "\r\n" */
    if (ch == '\n')
    {
        usart_data_transmit(DEBUG_UART, (uint8_t)'\r');
        while (RESET == usart_flag_get(DEBUG_UART, USART_FLAG_TBE))
        {
        }
    }
    
    usart_data_transmit(DEBUG_UART, (uint8_t)ch);
    while (RESET == usart_flag_get(DEBUG_UART, USART_FLAG_TBE))
    {
    }
    
    return ch;
}

/*!
    \brief      redirect fgetc from USART0 for scanf
    \note       This function is called by scanf in Keil MDK (requires MicroLIB)
*/
int fgetc(FILE *f)
{
    (void)f;  /* Prevent unused parameter warning */
    
    /* 等待接收数据 */
    while (RESET == usart_flag_get(DEBUG_UART, USART_FLAG_RBNE))
    {
    }
    
    /* 回显字符 (可选，终端交互友好) */
    uint8_t ch = (uint8_t)usart_data_receive(DEBUG_UART);
    
    /* 回显并处理换行 */
    if (ch == '\r')
    {
        ch = '\n';
    }
    
    return (int)ch;
}

/*!
    \brief      required by Keil MDK for proper stdio redirection
    \note       Called by library to check for errors
*/
void _ttywrch(int ch)
{
    fputc(ch, NULL);
}
