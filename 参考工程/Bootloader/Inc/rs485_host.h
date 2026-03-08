#ifndef RS485_HOST_H
#define RS485_HOST_H

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RS485_DIRECTION_RECEIVE = 0,
    RS485_DIRECTION_TRANSMIT = 1
} rs485_direction_t;

void rs485_host_init(void);
void rs485_host_set_direction(rs485_direction_t direction);
void rs485_host_send(const uint8_t *buffer, uint32_t length);
int32_t rs485_host_receive(uint8_t *buffer, uint32_t length, uint32_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif

