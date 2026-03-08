/*!
    \file    gd32f4xx_it.c
    \brief   interrupt service routines

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
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

#include "gd32f4xx_it.h"
#include "main.h"
#include "systick.h"
#include "sdcard.h"

/*
 * 说明：SysTick_Handler已由FreeRTOS接管
 * 如果需要在SysTick中断中执行用户代码，请使用FreeRTOS的vApplicationTickHook()钩子函数
 */

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/* 
 * 注意：以下三个中断处理函数由FreeRTOS接管，在port.c中定义
 * SVC_Handler - 用于FreeRTOS任务切换
 * PendSV_Handler - 用于FreeRTOS上下文切换
 * SysTick_Handler - 用于FreeRTOS时钟节拍
 */
 
#if 0  /* FreeRTOS接管这些中断，不使用裸机版本 */
/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}
#endif

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while(1) {
    }
}

#if 0  /* FreeRTOS接管这些中断，不使用裸机版本 */
/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief    this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    led_spark();
    delay_decrement();
}
#endif

/*
 * 说明：SysTick_Handler已由FreeRTOS接管
 * 如果需要在SysTick中断中执行用户代码，请使用FreeRTOS的vApplicationTickHook()钩子函数
 */

/*
 * USART1, USART2, USART5 中断处理已移除
 * EXTI SD卡检测中断处理已移除
 */

/*!
    \brief    this function handles SDIO interrupt (DMA mode)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SDIO_IRQHandler(void)
{
    /* 处理SDIO DMA传输中断 */
    sd_interrupts_process();
}

/*!
    \brief    TIMER5 中断处理函数（用户独立时基，1ms tick）
    \detail   TIMER5 作为用户延时/计时的独立时基，与 FreeRTOS 的 SysTick 完全分离
              避免 FreeRTOS 调度与用户计时之间的相互干扰
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER5_DAC_IRQHandler(void)
{
    /* 检查更新中断标志 */
    if (timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP) != RESET) {
        /* 清除中断标志 */
        timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
        
        /* 更新用户计时器（原 delay_decrement() 的功能）*/
        delay_decrement();
    }
}
