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
#include "host_comm.h"
#include "light_sensor.h"
#include "gps_nmea.h"
#include "modbus_rtu.h"
#include "gimbal_control.h"
#include "gd32f4xx_spi.h"
#include "sd_logger.h"
#include "sdcard.h"
#include "rs485_config.h"

/* FreeRTOS includes for ISR macros */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* USART5 RX DMA宏定义（与host_comm.c保持一致） */
#define HOST_COMM_DMA_PERIPH        DMA1
#define HOST_COMM_DMA_RX_CHANNEL    DMA_CH2
#define HOST_COMM_RX_DMA_BUF_SIZE   256

/* ==================== USART2 (传感器总线) RX DMA 配置 ==================== */

#define SENSOR_RX_DMA_BUF_SIZE   128  /* 根据期望的最大帧数选择 */

static uint8_t s_sensor_rx_dma_buf[SENSOR_RX_DMA_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_sensor_rx_dma_pos = 0;

/* RTOS版本：不再需要system_manager */

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
 * 该钩子函数已在main.c中实现
 */

/*!
    \brief    this function handles USART2 global interrupt (传感器总线)
    \note     USART2 使用 DMA0 CH1接收 + DMA0 CH3发送 + IDLE中断
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* IDLE中断：表示一段数据接收结束 / 线路空闲 */
    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE) != RESET) {
        /* 清除IDLE标志：必须依次读STAT和DATA */
        (void)usart_flag_get(USART2, USART_FLAG_IDLE);
        (void)usart_data_receive(USART2);

        uint16_t dma_pos = SENSOR_RX_DMA_BUF_SIZE -
                           dma_transfer_number_get(RS485_SENSOR_DMA_PERIPH,
                                                   RS485_SENSOR_DMA_RX_CHANNEL);
        uint16_t old_pos = s_sensor_rx_dma_pos;

        if (dma_pos != old_pos) {
            if (dma_pos > old_pos) {
                /* 未回绕 */
                for (uint16_t i = old_pos; i < dma_pos; i++) {
                    uint8_t byte = s_sensor_rx_dma_buf[i];
                    light_sensor_on_byte(byte, &xHigherPriorityTaskWoken);
                    gimbal_on_byte(byte, &xHigherPriorityTaskWoken);
                }
            } else {
                /* 回绕：先处理 [old_pos, BUF_END)，再处理 [0, dma_pos) */
                for (uint16_t i = old_pos; i < SENSOR_RX_DMA_BUF_SIZE; i++) {
                    uint8_t byte = s_sensor_rx_dma_buf[i];
                    light_sensor_on_byte(byte, &xHigherPriorityTaskWoken);
                    gimbal_on_byte(byte, &xHigherPriorityTaskWoken);
                }
                for (uint16_t i = 0; i < dma_pos; i++) {
                    uint8_t byte = s_sensor_rx_dma_buf[i];
                    light_sensor_on_byte(byte, &xHigherPriorityTaskWoken);
                    gimbal_on_byte(byte, &xHigherPriorityTaskWoken);
                }
            }

            s_sensor_rx_dma_pos = dma_pos;
        }
    }

    /* 保留原有的错误标志处理 */
    if (usart_flag_get(USART2, USART_FLAG_ORERR) != RESET) {
        usart_flag_clear(USART2, USART_FLAG_ORERR);
        (void)usart_data_receive(USART2);
    }

    if (usart_flag_get(USART2, USART_FLAG_FERR) != RESET) {
        usart_flag_clear(USART2, USART_FLAG_FERR);
        (void)usart_data_receive(USART2);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/*!
    \brief    this function handles USART1 global interrupt
*/
void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gps_uart_irq_handler(&xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/*!
    \brief    this function handles USART5 global interrupt (上位机通信)
    \note     USART5 使用 DMA1 CH2接收 + DMA1 CH7发送 + IDLE中断
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* IDLE中断：一帧Modbus帧结束或一段空闲 */
    if (usart_interrupt_flag_get(USART5, USART_INT_FLAG_IDLE) != RESET) {
        /* 必须读STAT和DATA清除IDLE标志 */
        (void)usart_flag_get(USART5, USART_FLAG_IDLE);
        (void)usart_data_receive(USART5);

        /* 计算DMA当前写指针位置 */
        uint16_t dma_pos = HOST_COMM_RX_DMA_BUF_SIZE -
                           dma_transfer_number_get(HOST_COMM_DMA_PERIPH,
                                                   HOST_COMM_DMA_RX_CHANNEL);

        host_comm_uart_idle_dma_rx_handler(dma_pos, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*!
    \brief    this function handles EXTI10_15 interrupt (SD card detect on PB14)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI10_15_IRQHandler(void)
{
    /* 检查是否是EXTI14（PB14）触发 */
    if (exti_interrupt_flag_get(EXTI_14) != RESET) {
        /* 清除中断标志 */
        exti_interrupt_flag_clear(EXTI_14);
        
        /* 读取当前引脚状态 */
        bool card_present = sd_card_detect_pin_read();
        
        if (card_present) {
            /* SD卡插入（低电平） */
            sd_logger_card_inserted_callback();
        } else {
            /* SD卡拔出（高电平） */
            sd_logger_card_removed_callback();
        }
    }
}

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
    \brief    初始化USART2 RX DMA功能
    \note     使用 DMA0 CH1 + IDLE中断
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usart2_sensor_dma_rx_init(void)
{
    dma_single_data_parameter_struct dma_init;

    /* 1. 使能DMA0时钟 */
    rcu_periph_clock_enable(RS485_SENSOR_DMA_RCU);

    /* 2. 配置DMA0 CH1为USART2 RX -> 内存，环形模式 */
    dma_single_data_para_struct_init(&dma_init);
    dma_deinit(RS485_SENSOR_DMA_PERIPH, RS485_SENSOR_DMA_RX_CHANNEL);

    dma_init.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init.memory0_addr        = (uint32_t)s_sensor_rx_dma_buf;
    dma_init.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init.number              = SENSOR_RX_DMA_BUF_SIZE;
    dma_init.periph_addr         = (uint32_t)&USART_DATA(USART2);
    dma_init.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init.priority            = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(RS485_SENSOR_DMA_PERIPH,
                              RS485_SENSOR_DMA_RX_CHANNEL,
                              &dma_init);

    dma_circulation_enable(RS485_SENSOR_DMA_PERIPH,
                           RS485_SENSOR_DMA_RX_CHANNEL);

    dma_channel_subperipheral_select(RS485_SENSOR_DMA_PERIPH,
                                     RS485_SENSOR_DMA_RX_CHANNEL,
                                     RS485_SENSOR_DMA_RX_SUBPERI);

    dma_flag_clear(RS485_SENSOR_DMA_PERIPH,
                   RS485_SENSOR_DMA_RX_CHANNEL,
                   DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_TAE | DMA_FLAG_FEE);

    dma_channel_enable(RS485_SENSOR_DMA_PERIPH,
                       RS485_SENSOR_DMA_RX_CHANNEL);

    /* 3. 开启USART2 DMA接收 */
    usart_dma_receive_config(USART2, USART_RECEIVE_DMA_ENABLE);

    /* 4. 打开IDLE中断，关闭RBNE中断 */
    usart_interrupt_disable(USART2, USART_INT_RBNE);
    usart_interrupt_enable(USART2, USART_INT_IDLE);
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
