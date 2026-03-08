/*!
    \file    systick.c
    \brief   the systick configuration file (TIMER5 独立时基版本 + DWT硬件延时)
    
    \detail  使用 TIMER5（基本定时器）作为用户延时/计时的独立时基
             SysTick 完全交给 FreeRTOS 管理，避免相互干扰
             
             时基分离架构：
             - SysTick: FreeRTOS 专用（1ms tick，由 FreeRTOS 配置）
             - TIMER5:  用户毫秒延时/计时专用（1ms tick，独立配置）
             - DWT:     微秒延时专用（周期计数器，硬件级精度）

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
    \version 2024-12-29, V3.3.2, TIMER5 独立时基版本
    \version 2024-12-29, V3.3.3, DWT硬件延时版本
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"

static volatile uint32_t delay;
static volatile uint32_t g_systick_tick = 0;
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
    \brief    configure TIMER5 for user timing (独立于 FreeRTOS)
    \param[in]  none
    \param[out] none
    \retval     none
    \note     TIMER5 配置为 1ms 中断，作为用户延时/计时的独立时基
              SysTick 完全交给 FreeRTOS 管理，不再在钩子函数中处理
*/
void systick_config(void)
{
    timer_parameter_struct timer_initpara;
    
    /* 初始化变量 */
    delay = 0;
    g_systick_tick = 0;
    
    /* 初始化 DWT 周期计数器（用于微秒延时） */
    dwt_init();
    
    /* ==================== 配置 TIMER5 作为用户独立时基 ==================== */
    
    /* 1. 使能 TIMER5 时钟 (APB1 总线，最大 50MHz，但有 x2 倍频) */
    rcu_periph_clock_enable(RCU_TIMER5);
    
    /* 2. 复位 TIMER5 */
    timer_deinit(TIMER5);
    
    /* 3. 配置 TIMER5 参数 
     * APB1 时钟 = SystemCoreClock / 4 = 200MHz / 4 = 50MHz
     * APB1 定时器时钟 = APB1 x 2 = 100MHz (当 APB1 分频 > 1 时)
     * 
     * 目标：1ms 定时中断
     * prescaler = 10000 - 1 (100MHz / 10000 = 10kHz)
     * period = 10 - 1 (10kHz / 10 = 1kHz = 1ms)
     */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 10000U - 1U;   /* 100MHz / 10000 = 10kHz */
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 10U - 1U;      /* 10kHz / 10 = 1kHz (1ms) */
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER5, &timer_initpara);
    
    /* 4. 使能更新中断 */
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    
    /* 5. 配置 NVIC（优先级低于 FreeRTOS 管理的中断） */
    nvic_irq_enable(TIMER5_DAC_IRQn, 15, 0);  /* 最低优先级，不影响 FreeRTOS */
    
    /* 6. 使能 TIMER5 */
    timer_enable(TIMER5);
}

/*!
    \brief    delay a time in milliseconds (TIMER5中断驱动，允许低功耗)
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count)
{
    delay = count;
    while(0U != delay) {
        /* 等待 TIMER5 中断递减 delay 变量，CPU可进入低功耗 */
        __WFI();
    }
}

/*!
    \brief    delay a time in microseconds (DWT硬件计数器实现)
    \param[in]  count: count in microseconds
    \param[out] none
    \retval     none
    \note     使用 DWT->CYCCNT 硬件计数器，无软件死循环，精确且高效
              适用于短延时（如RS485切换、SPI时序等）
*/
void delay_us(uint32_t count)
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
    g_systick_tick++;
}

/*!
    \brief    获取系统时钟节拍
    \retval   uint32_t: 当前时钟节拍（毫秒）
*/
uint32_t systick_get_tick(void)
{
    return g_systick_tick;
}
