/*!
    \file    systick.h
    \brief   the header file of systick

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
*/

#ifndef SYS_TICK_H
#define SYS_TICK_H

#include <stdint.h>

/* configure systick */
void systick_config(void);
/* delay a time in milliseconds */
void delay_1ms(uint32_t count);
/* delay a time in microseconds */
void delay_1us(uint32_t count);
/* delay decrement (called by SysTick_Handler) */
void delay_decrement(void);
/* get millisecond timestamp */
uint32_t systick_get_tick(void);

/* 兼容原有接口 */
#define delay_init(freq) systick_config()
#define delay_ms(ms) delay_1ms(ms)
#define delay_us(us) delay_1us(us)

#endif /* SYS_TICK_H */

