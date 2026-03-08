#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_WDG_TASK_SOLAR = 0,
    APP_WDG_TASK_HOST,
    APP_WDG_TASK_GPS,
    APP_WDG_TASK_LED,
    APP_WDG_TASK_COUNT
} app_wdg_task_id_t;

void app_watchdog_capture_reset_reason(void);
bool app_watchdog_was_reset_by_fwdgt(void);

void app_watchdog_init(void);
void app_watchdog_beat(app_wdg_task_id_t id);

void Watchdog_Task(void *pvParameters);

#endif /* APP_WATCHDOG_H */
