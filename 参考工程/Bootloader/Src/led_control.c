/**
 * @file    led_control.c
 * @brief   LED控制模块实现 - APP和BootLoader共用
 */

#include "led_control.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_gpio.h"
#include "delay.h"

/* LED引脚定义 */
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_11
#define LED_RCU         RCU_GPIOB

/**
 * @brief  初始化LED引脚（PB11）
 */
void led_init(void)
{
    /* 使能GPIOB时钟 */
    rcu_periph_clock_enable(LED_RCU);
    
    /* 配置PB11为推挽输出，50MHz */
    gpio_mode_set(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_output_options_set(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_PIN);
    
    /* 初始状态：LED关闭 */
    led_off();
}

/**
 * @brief  LED开启（PB11输出高电平）
 */
void led_on(void)
{
    gpio_bit_set(LED_PORT, LED_PIN);
}

/**
 * @brief  LED关闭（PB11输出低电平）
 */
void led_off(void)
{
    gpio_bit_reset(LED_PORT, LED_PIN);
}

/**
 * @brief  LED翻转状态
 */
void led_toggle(void)
{
    gpio_bit_toggle(LED_PORT, LED_PIN);
}

/**
 * @brief  LED闪烁指定时长
 * @param  duration_ms 闪烁总时长（毫秒）
 * @param  blink_period_ms 闪烁周期（毫秒），默认200ms
 * @note   此函数会阻塞指定的时长
 */
void led_blink(uint32_t duration_ms, uint32_t blink_period_ms)
{
    if (blink_period_ms == 0)
    {
        blink_period_ms = 200;  /* 默认200ms周期 */
    }
    
    uint32_t half_period = blink_period_ms / 2;
    uint32_t elapsed_time = 0;
    
    while (elapsed_time < duration_ms)
    {
        led_on();
        delay_ms(half_period);
        elapsed_time += half_period;
        
        if (elapsed_time >= duration_ms)
        {
            break;
        }
        
        led_off();
        delay_ms(half_period);
        elapsed_time += half_period;
    }
    
    /* 确保LED最终关闭 */
    led_off();
}

/**
 * @brief  LED闪烁指定次数
 * @param  count 闪烁次数
 * @param  on_time_ms LED点亮时间（毫秒）
 * @param  off_time_ms LED熄灭时间（毫秒）
 * @note   此函数会阻塞直到闪烁完成
 */
void led_blink_count(uint32_t count, uint32_t on_time_ms, uint32_t off_time_ms)
{
    for (uint32_t i = 0; i < count; i++)
    {
        led_on();
        delay_ms(on_time_ms);
        led_off();
        
        /* 最后一次闪烁后不需要延时 */
        if (i < count - 1)
        {
            delay_ms(off_time_ms);
        }
    }
}
