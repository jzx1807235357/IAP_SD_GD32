/*!
    \file    systick.c
    \brief   the systick configuration file (DWT硬件延时版本)

    \detail  使用 DWT (Data Watchpoint and Trace) 周期计数器实现精确延时
             - 微秒延时: DWT->CYCCNT 硬件计数器，无软件死循环
             - 毫秒延时: SysTick 中断驱动，允许低功耗
             
    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
    \version 2024-12-29, V3.3.2, DWT硬件延时版本
*/

#include "gd32f4xx.h"
#include "systick.h"

volatile static uint32_t delay;
volatile static uint32_t systick_ms = 0U; /* 毫秒时间戳计数器 */
static uint32_t dwt_cycles_per_us = 0U;   /* 每微秒CPU周期数 */

/*!
    \brief    初始化 DWT 周期计数器
    \param[in]  none
    \param[out] none
    \retval     none
    \note     DWT 是 ARM Cortex-M 内置的调试单元，包含 32位周期计数器
*/
static void dwt_init(void)
{
    /* 使能 DWT 单元 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    /* 复位周期计数器 */
    DWT->CYCCNT = 0U;
    
    /* 使能周期计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    /* 计算每微秒周期数 */
    dwt_cycles_per_us = SystemCoreClock / 1000000U;
}

/*!
    \brief    configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* 初始化 DWT 周期计数器 */
    dwt_init();
    
    /* setup systick timer for 1000Hz interrupts (用于毫秒延时和系统时基) */
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while(1) {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief    delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count)
{
    delay = count;

    while(0U != delay) {
    }
}

/*!
    \brief    delay a time in microseconds (DWT硬件计数器实现)
    \param[in]  count: count in microseconds
    \param[out] none
    \retval     none
    \note     使用 DWT->CYCCNT 硬件计数器，无软件死循环，精确且高效
*/
void delay_1us(uint32_t count)
{
    if (count == 0U || dwt_cycles_per_us == 0U) {
        return;
    }
    
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = count * dwt_cycles_per_us;
    
    /* 等待周期计数器达到目标值 */
    while ((DWT->CYCCNT - start) < cycles) {
        /* 硬件计数，CPU可进入低功耗 */
        __WFI();
    }
}

/*!
    \brief    delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
    if(0U != delay) {
        delay--;
    }
    systick_ms++; /* 每1ms递增 */
}

/*!
    \brief    get millisecond timestamp
    \param[in]  none
    \param[out] none
    \retval     milliseconds since systick_config()
*/
uint32_t systick_get_tick(void)
{
    return systick_ms;
}

