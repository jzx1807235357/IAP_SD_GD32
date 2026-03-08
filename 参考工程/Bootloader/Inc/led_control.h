/**
 * @file    led_control.h
 * @brief   LED控制模块 - APP和BootLoader共用
 * 
 * 功能：
 * 1. LED初始化（PB11）
 * 2. LED闪烁控制
 * 3. LED状态控制（亮/灭/翻转）
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "gd32f4xx.h"
#include <stdint.h>

/**
 * @brief  初始化LED引脚（PB11）
 */
void led_init(void);

/**
 * @brief  LED开启（PB11输出高电平）
 */
void led_on(void);

/**
 * @brief  LED关闭（PB11输出低电平）
 */
void led_off(void);

/**
 * @brief  LED翻转状态
 */
void led_toggle(void);

/**
 * @brief  LED闪烁指定时长
 * @param  duration_ms 闪烁总时长（毫秒）
 * @param  blink_period_ms 闪烁周期（毫秒），默认200ms
 * @note   此函数会阻塞指定的时长
 */
void led_blink(uint32_t duration_ms, uint32_t blink_period_ms);

/**
 * @brief  LED闪烁指定次数
 * @param  count 闪烁次数
 * @param  on_time_ms LED点亮时间（毫秒）
 * @param  off_time_ms LED熄灭时间（毫秒）
 * @note   此函数会阻塞直到闪烁完成
 */
void led_blink_count(uint32_t count, uint32_t on_time_ms, uint32_t off_time_ms);

#endif /* LED_CONTROL_H */
