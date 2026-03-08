#include "app_watchdog.h"

#include "gd32f4xx_fwdgt.h"
#include "gd32f4xx_rcu.h"

#include "FreeRTOS.h"
#include "task.h"

#define APP_WDG_MONITOR_PERIOD_MS   500U
/* Host任务可能执行外部Flash擦写，超时阈值必须大于单次保存耗时 */
#define APP_WDG_TASK_STALE_MS       12000U

/* 典型配置下约16秒超时，覆盖一次外部Flash擦写 + 写入时长 */
#define APP_WDG_HW_RELOAD           2500U
#define APP_WDG_HW_PSC              FWDGT_PSC_DIV256

static volatile TickType_t s_last_beat[APP_WDG_TASK_COUNT];
static volatile bool s_reset_by_fwdgt = false;

void app_watchdog_capture_reset_reason(void)
{
    s_reset_by_fwdgt = (rcu_flag_get(RCU_FLAG_FWDGTRST) == SET);
    rcu_all_reset_flag_clear();
}

bool app_watchdog_was_reset_by_fwdgt(void)
{
    return s_reset_by_fwdgt;
}

void app_watchdog_init(void)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t i;

    taskENTER_CRITICAL();
    for (i = 0; i < APP_WDG_TASK_COUNT; i++) {
        s_last_beat[i] = now;
    }
    taskEXIT_CRITICAL();
}

void app_watchdog_beat(app_wdg_task_id_t id)
{
    if (id >= APP_WDG_TASK_COUNT) {
        return;
    }

    taskENTER_CRITICAL();
    s_last_beat[id] = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

static bool app_watchdog_all_tasks_alive(TickType_t now)
{
    uint32_t i;

    for (i = 0; i < APP_WDG_TASK_COUNT; i++) {
        if ((now - s_last_beat[i]) > pdMS_TO_TICKS(APP_WDG_TASK_STALE_MS)) {
            return false;
        }
    }

    return true;
}

static bool app_watchdog_hw_init(void)
{
    if (SUCCESS != fwdgt_config(APP_WDG_HW_RELOAD, APP_WDG_HW_PSC)) {
        return false;
    }

    fwdgt_counter_reload();
    fwdgt_enable();
    return true;
}

void Watchdog_Task(void *pvParameters)
{
    (void)pvParameters;

    /* 让系统先完成启动与初始数据同步 */
    vTaskDelay(pdMS_TO_TICKS(5000));

    app_watchdog_init();

    if (!app_watchdog_hw_init()) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (app_watchdog_all_tasks_alive(now)) {
            fwdgt_counter_reload();
        }

        vTaskDelay(pdMS_TO_TICKS(APP_WDG_MONITOR_PERIOD_MS));
    }
}
