#ifndef DELAY_H
#define DELAY_H

#include "gd32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

void delay_init(uint32_t sysclk_hz);
void delay_us(uint32_t microseconds);
void delay_ms(uint32_t milliseconds);

#ifdef __cplusplus
}
#endif

#endif
