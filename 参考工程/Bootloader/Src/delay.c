#include "delay.h"

static uint32_t delay_fac_us = 0U;

void delay_init(uint32_t sysclk_hz)
{
    delay_fac_us = sysclk_hz / 1000000U;
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

static void delay_ticks(uint32_t ticks)
{
    if (ticks == 0U || delay_fac_us == 0U)
    {
        return;
    }

    while (ticks != 0U)
    {
        uint32_t current = (ticks > 0xFFFFFFU) ? 0xFFFFFFU : ticks;
        SysTick->LOAD = current - 1U;
        SysTick->VAL = 0U;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
        {
        }
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        ticks -= current;
    }
}

void delay_us(uint32_t microseconds)
{
    uint64_t total_ticks = (uint64_t)microseconds * (uint64_t)delay_fac_us;
    while (total_ticks != 0U)
    {
        uint32_t current = (total_ticks > 0xFFFFFFU) ? 0xFFFFFFU : (uint32_t)total_ticks;
        delay_ticks(current);
        total_ticks -= (uint64_t)current;
    }
}

void delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U)
    {
        delay_us(1000U);
    }
}
