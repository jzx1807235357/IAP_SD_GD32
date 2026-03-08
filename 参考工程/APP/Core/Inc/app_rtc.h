/*
 * app_rtc.h - Unified RTC interface used in FreeRTOS application
 */
#ifndef APP_RTC_H
#define APP_RTC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} rtc_datetime_t;

bool app_rtc_init(void);
bool app_rtc_has_valid_time(void);
bool app_rtc_get_datetime(rtc_datetime_t *dt);
bool app_rtc_set_datetime(const rtc_datetime_t *dt);

#endif /* APP_RTC_H */
