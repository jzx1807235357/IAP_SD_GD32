/*!
    \file    systick.c
    \brief   the systick configuration file

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
*/

#include "gd32f4xx.h"
#include "systick.h"

volatile static uint32_t delay;
volatile static uint32_t systick_ms = 0U; /* 毫秒时间戳计数器 */

/*!
    \brief    configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
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
    \brief    delay a time in microseconds
    \param[in]  count: count in microseconds
    \param[out] none
    \retval     none
*/
void delay_1us(uint32_t count)
{
    uint32_t i;
    uint32_t loops = (SystemCoreClock / 1000000U) * count;
    
    for(i = 0; i < loops; i++) {
        __NOP();
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

