/*
 * pcf8563.h - Minimal driver interface for NXP PCF8563 RTC
 */
#ifndef PCF8563_H
#define PCF8563_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8563_I2C_ADDR_7BIT   (0x51u)

typedef struct
{
    uint16_t year;    /* 2000-2099 */
    uint8_t  month;   /* 1-12 */
    uint8_t  day;     /* 1-31 */
    uint8_t  weekday; /* 0-6 */
    uint8_t  hours;   /* 0-23 */
    uint8_t  minutes; /* 0-59 */
    uint8_t  seconds; /* 0-59 */
    bool     valid;
} pcf8563_datetime_t;

bool pcf8563_init(void);
bool pcf8563_get_datetime(pcf8563_datetime_t *dt);
bool pcf8563_set_datetime(const pcf8563_datetime_t *dt);

#ifdef __cplusplus
}
#endif

#endif /* PCF8563_H */
