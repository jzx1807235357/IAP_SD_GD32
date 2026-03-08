/*!
    \file    systick.h
    \brief   the header file of systick (DWT硬件延时版本)

    \detail  延时实现方式：
             - 毫秒延时: SysTick 中断驱动，允许 CPU 进入低功耗
             - 微秒延时: DWT 周期计数器，硬件级精度，无软件死循环

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
    \version 2024-12-29, V3.3.2, DWT硬件延时版本
*/

#ifndef SYS_TICK_H
#define SYS_TICK_H

#include <stdint.h>

/* configure systick and DWT */
void systick_config(void);
/* delay a time in milliseconds (SysTick中断驱动) */
void delay_1ms(uint32_t count);
/* delay a time in microseconds (DWT硬件计数器) */
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

