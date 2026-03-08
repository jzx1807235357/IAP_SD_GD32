/*!
    \file    systick.c
    \brief   the systick configuration file (TIMER5 独立时基版本)
    
    \detail  使用 TIMER5（基本定时器）作为用户延时/计时的独立时基
             SysTick 完全交给 FreeRTOS 管理，避免相互干扰
             
             时基分离架构：
             - SysTick: FreeRTOS 专用（1ms tick，由 FreeRTOS 配置）
             - TIMER5:  用户延时/计时专用（1ms tick，独立配置）

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
    \version 2024-12-29, V3.3.2, TIMER5 独立时基版本
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
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t delay;
static volatile uint32_t g_systick_tick = 0;

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
    \brief    delay a time in milliseconds (RTOS compatible)
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
    \note     在FreeRTOS环境中，如果调度器已启动则使用vTaskDelay，否则使用裸机延时
*/
void delay_1ms(uint32_t count)
{
    /* 检查FreeRTOS调度器是否已启动 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* 调度器已启动，使用FreeRTOS延时（允许任务切换） */
        vTaskDelay(pdMS_TO_TICKS(count));
    } else {
        /* 调度器未启动，使用忙等待延时（CPU循环） */
        /* 注意：在调度器启动前，SysTick中断不可用，只能用忙等待 */
        for (uint32_t i = 0; i < count; i++) {
            delay_us(1000);  /* 1ms = 1000us */
        }
    }
}

/*!
    \brief    delay a time in microseconds (busy-wait for short delays)
    \param[in]  count: count in microseconds
    \param[out] none
    \retval     none
    \note     使用CPU忙等待实现微秒延时，适用于短延时（如RS485切换）
*/
void delay_us(uint32_t count)
{
    /* 假设CPU运行在200MHz，每次循环约5ns，需要200次循环实现1us */
    /* 根据实际CPU频率调整此值 */
    uint32_t loops = count * (SystemCoreClock / 5000000U);
    for (volatile uint32_t i = 0; i < loops; i++) {
        /* 空循环，编译器优化保护 */
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
    \brief    获取系统时钟节拍（兼容FreeRTOS）
    \retval   uint32_t: 当前时钟节拍（毫秒）
*/
uint32_t systick_get_tick(void)
{
    /* 优先使用FreeRTOS的时钟节拍 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* 调度器已启动，使用FreeRTOS时钟 */
        return xTaskGetTickCount();
    } else {
        /* 调度器未启动，使用裸机计数器 */
        return g_systick_tick;
    }
}
