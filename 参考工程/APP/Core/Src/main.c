/*!
    \file    main.c

    \version 2024-12-21, V3.0.0, firmware for GD32F4xx with FreeRTOS
*/

/* 固件版本号定义 */
#define FIRMWARE_VERSION_MAJOR      3
#define FIRMWARE_VERSION_MINOR      9
#define FIRMWARE_VERSION_PATCH      0
/* 版本号组合为 0xMMNN 格式 (Major.Minor) */
#define FIRMWARE_VERSION            ((FIRMWARE_VERSION_MAJOR << 8) | FIRMWARE_VERSION_MINOR)

#include "gd32f4xx.h"
#include "gd32f4xx_misc.h"
#include "systick.h"
#include "rs485_hal.h"
#include "rs485_config.h"
#include "light_sensor.h"
#include "host_comm.h"
#include "gimbal_control.h"
#include "optical_tracking.h"
#include "flash_config.h"
#include "gd25q16_flash.h"
#include "modbus_rtu.h"
#include "data_hub.h"
#include "event_groups.h"
#include "gps_nmea.h"
#include "sun_position.h"
#include "sun_position_port.h"
#include "app_rtc.h"
#include "app_watchdog.h"
#include "sd_logger.h"
#include "host_comm.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* LED GPIO configuration */
#define LED_GPIO_PORT    GPIOB
#define LED_GPIO_PIN     GPIO_PIN_11
#define LED_GPIO_CLK     RCU_GPIOB

/* FreeRTOS Task Handles */
static TaskHandle_t SolarTrackingTask_Handle = NULL;
static TaskHandle_t HostCommTask_Handle = NULL;
static TaskHandle_t GPSTask_Handle = NULL;
static TaskHandle_t LEDTask_Handle = NULL;
static TaskHandle_t SDLoggerTask_Handle = NULL;

/* FreeRTOS Queue Handles */
QueueHandle_t xSensorDataQueue = NULL;
QueueHandle_t xHostCmdQueue = NULL;

/* FreeRTOS Semaphore Handles */
SemaphoreHandle_t xUART0_TxSemaphore = NULL;
SemaphoreHandle_t xUART5_TxSemaphore = NULL;

/* Data synchronization event group */
static EventGroupHandle_t DataEventGroup = NULL;

/* Tracking state */
static volatile uint16_t g_runMode = 0;
static const config_runtime_t* g_config_runtime = NULL;

static int16_t calc_timezone_from_longitude(double longitude);

/* 光控阈值（可通过上位机动态配置） */
static float g_optical_enter_threshold = 0.50f;  /* 进入光学追踪的亮度阈值（V，默认0.50V） */
static float g_optical_exit_threshold = 0.40f;   /* 退出光学追踪的亮度阈值（V，默认0.40V，滞回） */

/* 光学追踪死区阈值（可通过上位机动态配置） */
static float g_optical_error_enter = OPTICAL_ERROR_ENTER_DEFAULT;
static float g_optical_error_exit = OPTICAL_ERROR_EXIT_DEFAULT;

/* 太阳位置缓存（北=0°坐标系，用于诊断日志和GPS追踪） */
typedef struct {
    float azimuth;
    float elevation;
    bool  valid;
} sun_cache_t;

static sun_cache_t g_sun_cache = {0.0f, 0.0f, false};

/* 夜间归位持续判断状态 */
#define NIGHT_POSITION_CONFIRM_TIME_MS  60000U  /* 持续1分钟才确认夜间归位 */
typedef enum {
    SUN_STATE_UNKNOWN = 0,      /* 未知状态 */
    SUN_STATE_BELOW_HORIZON,    /* 太阳在地平线以下(<-5°) */
    SUN_STATE_ABOVE_HORIZON     /* 太阳在地平线以上(>=-5°) */
} sun_position_state_t;

static sun_position_state_t g_sun_state = SUN_STATE_UNKNOWN;
static uint32_t g_sun_state_start_time = 0;     /* 当前状态开始时间 */
static bool g_night_position_confirmed = false;  /* 夜间位置已确认标志 */

#define OPTICAL_SENSOR_MAX_AGE_MS       2000U

/* 本轮查询得到的光感/云台数据是否是"最新且未被算法使用"的标志 */
static volatile bool g_light_data_fresh = false;   /* 光感数据新鲜标志 */
static volatile bool g_gimbal_data_fresh = false;  /* 云台数据新鲜标志 */

/* RS485总线严格顺序访问状态机 */
typedef enum {
    BUS_STATE_QUERY_SENSOR = 0,      //* 状态1: 查询传感器（禁用云台
    BUS_STATE_WAIT_SENSOR,            //* 状态2: 等待传感器数据（不可打断）
    BUS_STATE_QUERY_GIMBAL_AZ,        //* 状态3: 查询云台方位角（禁用传感器）
    BUS_STATE_WAIT_GIMBAL_AZ,         //* 状态4: 等待方位角数据（不可打断）
    BUS_STATE_QUERY_GIMBAL_EL,        //* 状态5: 查询云台仰角 */
    BUS_STATE_WAIT_GIMBAL_EL          //* 状态6: 等待仰角数据（不可打断）
} bus_access_state_t;

static volatile bus_access_state_t g_bus_state = BUS_STATE_QUERY_SENSOR;

/* 存储测试状态 */
static volatile bool g_storage_test_done = false;

static void tracking_apply_mode(uint16_t mode);
static bool tracking_is_brightness_sufficient(void);

/* Task function prototypes */
static void SolarTracking_Task(void *pvParameters);
static void HostComm_Task(void *pvParameters);
static void GPS_Task(void *pvParameters);
static void LED_Task(void *pvParameters);

/* Storage initialization function (called once at startup) */
static void storage_system_init(void);

void led_config(void)
{
    /* enable the LED GPIO clock */
    rcu_periph_clock_enable(LED_GPIO_CLK);

    /* configure LED GPIO pin */
    gpio_mode_set(LED_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GPIO_PIN);
    gpio_output_options_set(LED_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_GPIO_PIN);

    /* turn off LED */
    gpio_bit_reset(LED_GPIO_PORT, LED_GPIO_PIN);
}

static int16_t calc_timezone_from_longitude(double longitude)
{
    int tz;
    double x = longitude / 15.0;
    if (x >= 0) {
        tz = (int)(x + 0.5);
    } else {
        tz = (int)(x - 0.5);
    }

    if (tz < -12) {
        tz = -12;
    } else if (tz > 14) {
        tz = 14;
    }

    return (int16_t)tz;
}

void led_toggle(void)
{
    gpio_bit_toggle(LED_GPIO_PORT, LED_GPIO_PIN);
}

void led_set(bool on)
{
    if (on) {
        gpio_bit_set(LED_GPIO_PORT, LED_GPIO_PIN);
    } else {
        gpio_bit_reset(LED_GPIO_PORT, LED_GPIO_PIN);
    }
}

void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if (timingdelaylocal) {
        if (timingdelaylocal < 500U) {
            /* LED on */
            gpio_bit_reset(LED_GPIO_PORT, LED_GPIO_PIN);
        } else {
            /* LED off */
            gpio_bit_set(LED_GPIO_PORT, LED_GPIO_PIN);
        }
        timingdelaylocal--;
    } else {
        timingdelaylocal = 1000U;
    }
}

static bool tracking_is_brightness_sufficient(void)
{
    /* 检查数据有效性和亮度 */
    if (!g_sensor_data.valid) {
        return false;
    }
    return (light_sensor_get_total_intensity() >= g_optical_enter_threshold);
}

static void tracking_apply_mode(uint16_t mode)
{
    uint16_t desired_mode = mode;
    uint16_t error_code = 0U;

    if (desired_mode > 3U) {
        desired_mode = 0U;
    }

    if (desired_mode == 2U && !tracking_is_brightness_sufficient()) {
        desired_mode = 1U;
        if (mode == 2U) {
            error_code = 0x0202U; /* Insufficient light, auto fallback to GPS mode */
        }
    }

    switch (desired_mode) {
    case 0U: /* Idle */
        optical_tracking_set_mode(OPT_MODE_DISABLED);
        optical_tracking_stop();
        break;
    case 1U: /* GPS tracking */
        optical_tracking_set_mode(OPT_MODE_GPS_TRACKING);
        optical_tracking_stop();
        break;
    case 2U: /* Optical tracking */
        optical_tracking_set_mode(OPT_MODE_OPTICAL_TRACKING);
        (void)optical_tracking_start();
        break;
    case 3U: /* Manual override */
        optical_tracking_set_mode(OPT_MODE_DISABLED);
        optical_tracking_stop();
        break;
    default:
        break;
    }

    g_runMode = desired_mode;
    modbus_rtu_slave_set_register(REG_HOST_RUN_MODE, g_runMode);
    modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, error_code);
}

int main(void)
{
    /* Initialize LED and keep on during startup */
    led_config();
    led_set(true);

    /* Configure NVIC priority grouping so that FreeRTOS can manage priorities */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* Initialize systick (SysTick clock will be taken over by FreeRTOS) */
    systick_config();

    /* ========== 初始化RS485总线（必须在模块初始化之前）========== */
    
    /* 初始化传感器总线（USART2）- 云台 + 光敏传感器 */
    rs485_config_t sensor_bus_config = RS485_SENSOR_CONFIG_INIT();
    if (rs485_init(RS485_BUS_SENSOR, &sensor_bus_config) != RS485_OK) {
        led_set(true);
        while (1);  /* RS485传感器总线初始化失败 */
    }
    
    /* 启用USART2的DMA+IDLE接收 */
    extern void usart2_sensor_dma_rx_init(void);
    usart2_sensor_dma_rx_init();
    
    /* 初始化上位机总线（USART5）*/
    rs485_config_t host_bus_config = RS485_HOST_CONFIG_INIT();
    if (rs485_init(RS485_BUS_HOST, &host_bus_config) != RS485_OK) {
        led_set(true);
        while (1);  /* RS485上位机总线初始化失败 */
    }
    
    /* Initialize light sensor subsystem */
    if (!light_sensor_init()) {
        led_set(true);
        while (1);
    }

    /* Initialize sensor communication manager */
    if (!sensor_comm_init()) {
        led_set(true);
        while (1);
    } 	

    /* Initialize host communication */
    if (!host_comm_init()) {
        led_set(true);
        while (1);
    }

    /* Initialize gimbal control module */
    gimbal_init();

    /* Initialize optical tracking controller */
    if (!optical_tracking_init()) {
        led_set(true);
        while (1);
    }

    /* 注意：flash_config_init() 延迟到StorageTest_Task中执行，避免FreeRTOS未启动时调用API */
    g_config_runtime = NULL;  /* 临时设为NULL，等待Task中初始化 */

    /* Start in GPS tracking mode after power-on, allow manual mode switching from host */
    tracking_apply_mode(1U);

    /* Initialize GPS module */
    if (!gps_init()) {
        led_set(true);
        while (1);
    }

    /* Initialize external RTC (non-fatal if missing) */
    (void)app_rtc_init();
		
    /* 存储系统将在独立任务中初始化 */

    xSensorDataQueue = xQueueCreate(5, sizeof(sensor_data_t));
    xHostCmdQueue = xQueueCreate(20, sizeof(host_cmd_msg_t));  /* Increase queue depth to avoid gimbal control command loss */
    
    if (xSensorDataQueue == NULL || xHostCmdQueue == NULL) {
        led_set(true);
        while(1);
    }

    xUART0_TxSemaphore = xSemaphoreCreateBinary();
    xUART5_TxSemaphore = xSemaphoreCreateBinary();
    
    if (xUART0_TxSemaphore == NULL || xUART5_TxSemaphore == NULL) {
        led_set(true);
        while(1);
    }
    
    DataEventGroup = xEventGroupCreate();
    if (DataEventGroup == NULL) {
        led_set(true);
        while(1);
    }
    data_hub_init(DataEventGroup);

    /* 初始化太阳位置平台适配层 */
    sun_position_port_init();

    sensor_data_t initial_sensor = {0};
    initial_sensor.valid = false;
    data_hub_update_sensor(&initial_sensor);
    data_hub_update_sun(0.0f, 0.0f, false, false);
    data_hub_update_gimbal(0.0f, 0.0f, false);
    data_hub_update_gps_status(0);

    app_watchdog_capture_reset_reason();
    
    /* 初始化RTC时间寄存器为无效标记（等待GPS或上位机设置） */
    if (!app_rtc_has_valid_time()) {
        modbus_rtu_slave_set_register(REG_HOST_RTC_YEAR, 0xFFFF);
        modbus_rtu_slave_set_register(REG_HOST_RTC_MONTH, 0xFF);
        modbus_rtu_slave_set_register(REG_HOST_RTC_DAY, 0xFF);
        modbus_rtu_slave_set_register(REG_HOST_RTC_HOUR, 0xFF);
        modbus_rtu_slave_set_register(REG_HOST_RTC_MINUTE, 0xFF);
        modbus_rtu_slave_set_register(REG_HOST_RTC_SECOND, 0xFF);
    }

    xSemaphoreGive(xUART0_TxSemaphore);
    xSemaphoreGive(xUART5_TxSemaphore);

    /* Create FreeRTOS tasks */
    if (xTaskCreate(SolarTracking_Task, "SolarTrack", 768, NULL, 2, &SolarTrackingTask_Handle) != pdPASS) {
        led_set(true);
        while(1);
    }

    if (xTaskCreate(HostComm_Task, "HostComm", 768, NULL, 3, &HostCommTask_Handle) != pdPASS) {
        led_set(true);
        while(1);
    }

    /* Gimbal_Task已合并到SolarTracking_Task，不再单独创建 */

    if (xTaskCreate(GPS_Task, "GPS", 512, NULL, 1, &GPSTask_Handle) != pdPASS) {
        led_set(true);
        while(1);
    }

    /* Sun_Task已合并到SolarTracking_Task */

    if (xTaskCreate(LED_Task, "LED", 256, NULL, 1, &LEDTask_Handle) != pdPASS) {
        led_set(true);
        while(1);
    }

    /* SD日志后台任务：处理索引重建等耗时操作（低优先级，避免阻塞通信） */
		if (xTaskCreate(sd_logger_task, "SDLog", 768, NULL, 1, &SDLoggerTask_Handle) != pdPASS) {
				led_set(true);
				while(1);
		}

    if (xTaskCreate(Watchdog_Task, "Watchdog", 256, NULL, 1, NULL) != pdPASS) {
        led_set(true);
        while(1);
    }

    /* StorageTest_Task已优化：改为在任务启动后延迟初始化（节省1536字节栈空间） */

    led_set(false);

    vTaskStartScheduler();

    led_set(true);
    while(1) {
    }
}

/*!
    \brief    太阳追踪控制任务
    \detail   核心职责：
              1. 云台命令处理
              2. 传感器查询（四路光电压）
              3. GPS天文追踪（粗定位）
              4. 光学追踪（精确定位）
              5. 云台位置查询与更新
    \param[out] none
    \retval     none
*/

static void SolarTracking_Task(void *pvParameters)
{
    static uint32_t sensor_query_start = 0;
    static uint32_t gimbal_query_start = 0;
    static uint32_t last_sun_update = 0;  /* 太阳位置更新时间戳（移到函数开始处,确保作用域正确）*/
    static uint32_t last_gimbal_cmd_time = 0;  /* 上次发送云台控制命令的时间戳 */
    static uint32_t gimbal_queue_full_count = 0;  /* 云台命令队列满计数（诊断用） */
    static bool g_night_entry_selftest_done = false;  /* 进入夜间模式时的自检完成标志 */

    (void)pvParameters;

    /* 注册云台任务句柄（用于ISR通知） */
    gimbal_register_task_handle(xTaskGetCurrentTaskHandle());

    vTaskDelay(pdMS_TO_TICKS(500));  /* 等待系统初始化 */

    while (1) {
        /* ============ RS485总线严格顺序访问状态机 ============ */
        /* 保证传感器和云台按照固定顺序独占总线，接收数据时不可打断 */
        
        switch (g_bus_state) {
            case BUS_STATE_QUERY_SENSOR:
            {
                /* ========== 状态1: 查询传感器（此时禁止云台操作）========== */
                
                /* 注意：不在此处调用gimbal_process()，避免云台命令干扰后续云台查询 */
                /* 云台命令将在查询完成后统一处理 */
                
                /* 启动一次异步传感器查询 */
                if (sensor_comm_start_async()) {
                    sensor_query_start = systick_get_tick();
                    g_bus_state = BUS_STATE_WAIT_SENSOR;
                } else {
                    /* 启动失败（总线被占用/发送失败），直接进入云台查询 */
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_AZ;
                }
                break;
            }
            
            case BUS_STATE_WAIT_SENSOR:
            {
                /* ========== 状态2: 等待传感器数据接收（不可打断）========== */
                bool done = false;
                sensor_data_t sensor_data;
                uint32_t now = systick_get_tick();

                /* 通过高层异步接口检查是否完成 */
                bool ok = sensor_comm_poll_async(now, &sensor_data, &done);

                if (ok) {
                    /* 接收成功，更新上位机/数据中心 */
                    if (xSensorDataQueue != NULL) {
                        (void)xQueueSend(xSensorDataQueue, &sensor_data, 0);
                    }
                    data_hub_update_sensor(&sensor_data);

                    /* 本轮光感数据就绪且尚未被算法使用 */
                    g_light_data_fresh = true;

                    sensor_query_start = 0;
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_AZ;
                } else if (done) {
                    /* 会话结束但失败/超时，光感数据本轮视为无效 */
                    g_light_data_fresh = false;
                    sensor_query_start = 0;
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_AZ;
                } else if ((now - sensor_query_start) > 300U) {
                    /* 双保险：理论上不会到这里（因为 SENSOR_COMM_TIMEOUT_MS <= 300） */
                    g_light_data_fresh = false;
                    sensor_query_start = 0;
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_AZ;
                }
                /* 否则继续等待（不执行任何操作，等待下一次循环检查）*/
                break;
            }
            
            case BUS_STATE_QUERY_GIMBAL_AZ:
            {
                /* ========== 状态3: 查询云台方位角（此时禁止传感器查询）========== */
                
                /* 发送方位角查询请求（非阻塞，立即返回）*/
                if (gimbal_send_query_azimuth() == GIMBAL_OK) {
                    /* 发送成功，进入等待状态 */
                    gimbal_query_start = systick_get_tick();
                    g_bus_state = BUS_STATE_WAIT_GIMBAL_AZ;
                } else {
                    /* 发送失败，跳过云台查询，切回传感器查询 */
                    g_bus_state = BUS_STATE_QUERY_SENSOR;
                }
                break;
            }
            
            case BUS_STATE_WAIT_GIMBAL_AZ:
            {
                /* ========== 状态4: 等待方位角数据接收（不可打断）========== */
                /* ISR正在接收方位角数据，必须等待完成或超时 */
                
                static float temp_azimuth = 0.0f;
                
                /* 非阻塞检查数据是否就绪 */
                if (gimbal_is_azimuth_ready(&temp_azimuth)) {
                    /* 数据接收完成，保存结果 */
                    g_gimbal_position.azimuth = temp_azimuth;
                    gimbal_query_start = 0;
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_EL;
                } else if ((systick_get_tick() - gimbal_query_start) > 300) {
                    /* 超时（300ms），清理查询状态后进入查询仰角状态 */
                    gimbal_clear_query_state();  /* 关键：清理上一轮未完成的查询状态 */
                    gimbal_query_start = 0;
                    g_bus_state = BUS_STATE_QUERY_GIMBAL_EL;
                }
                /* 否则继续等待（不执行任何操作）*/
                break;
            }
            
            case BUS_STATE_QUERY_GIMBAL_EL:
            {
                /* ========== 状态5: 查询云台仰角（纯轮询，不干扰运动）========== */
                /* 云台移动时可能无法响应查询，失败就下一轮再试 */
                
                if (gimbal_send_query_elevation() == GIMBAL_OK) {
                    /* 发送成功，进入等待状态 */
                    gimbal_query_start = systick_get_tick();
                    g_bus_state = BUS_STATE_WAIT_GIMBAL_EL;
                } else {
                    /* 发送失败，切回传感器查询，下一轮再试 */
                    g_bus_state = BUS_STATE_QUERY_SENSOR;
                }
                break;
            }
            
            case BUS_STATE_WAIT_GIMBAL_EL:
            {
                /* ========== 状态6: 等待仰角数据接收（不可打断）========== */
                /* ISR正在接收仰角数据，必须等待完成或超时 */
                
                static float temp_elevation = 0.0f;
                static bool query_completed = false;
                
                /* 非阻塞检查数据是否就绪 */
                if (gimbal_is_elevation_ready(&temp_elevation)) {
                    /* 数据接收完成，保存结果并更新上位机 */
                    g_gimbal_position.elevation = temp_elevation;
                    data_hub_update_gimbal(g_gimbal_position.azimuth, 
                                           g_gimbal_position.elevation, 
                                           true);

                    /* 本轮云台角度就绪且尚未被算法使用 */
                    g_gimbal_data_fresh = true;
                    
                    /* 标记查询完成 */
                    gimbal_query_start = 0;
                    query_completed = true;
                    g_bus_state = BUS_STATE_QUERY_SENSOR;
                } else if ((systick_get_tick() - gimbal_query_start) > 300) {
                    /* 超时（300ms），本轮云台数据视为无效，但仍更新上位机寄存器 */
                    /* 使用上次有效的位置值，确保上位机显示不会卡住 */
                    gimbal_clear_query_state();  /* 关键：清理未完成的查询状态 */
                    g_gimbal_data_fresh = false;
                    data_hub_update_gimbal(g_gimbal_position.azimuth, 
                                           g_gimbal_position.elevation, 
                                           true);  /* 仍标记为有效，因为位置值是上次成功获取的 */
                    gimbal_query_start = 0;
                    query_completed = true;
                    g_bus_state = BUS_STATE_QUERY_SENSOR;
                }
                
                /* 查询完成后，执行追踪控制逻辑（先查询再控制）*/
                if (query_completed) {
                    query_completed = false;
                    
                    /* ========== 整合：天文解算和GPS状态更新（原Sun_Task功能）========== */
                    /* 每轮循环都计算太阳位置并更新data_hub，供上位机显示 */
                    uint32_t current_tick = systick_get_tick();
                    
                    /* ========== 统一天文解算（每轮只算一次）========== */
                    /* 计算太阳位置（北=0°坐标系） */
                    /* 变量定义在外层作用域，供后续GPS追踪使用 */
                    bool sun_ok = false;
                    bool sun_visible = false;
                    float sun_azimuth = 0.0f;
                    float sun_elevation = 0.0f;

                    /* 每500ms更新一次太阳位置数据（与循环周期同步） */
                    if ((current_tick - last_sun_update) >= 500) {
                        last_sun_update = current_tick;

                        /* 更新GPS状态到data_hub */
                        uint16_t gps_status = 0;
                        if (gps_has_fix()) {
                            gps_status = 2U;  /* GPS定位成功 */
                        } else if (gps_is_communicating()) {
                            gps_status = 1U;  /* GPS通信中，但未定位 */
                        } else {
                            gps_status = 0U;  /* GPS无通信 */
                        }
                        data_hub_update_gps_status(gps_status);

                        /* 执行天文解算（PSA算法直接输出北=0°云台坐标系） */
                        if (sun_position_from_gps(&sun_azimuth, &sun_elevation)) {
                            sun_ok = true;
                            sun_visible = sun_position_is_visible(sun_elevation);

                            /* 缓存北=0°坐标系角度供GPS追踪使用 */
                            g_sun_cache.azimuth = sun_azimuth;
                            g_sun_cache.elevation = sun_elevation;
                            g_sun_cache.valid = true;

                            /* 更新data_hub */
                            data_hub_update_sun(sun_azimuth, sun_elevation, true, sun_visible);
                        } else {
                            g_sun_cache.valid = false;
                            data_hub_update_sun(0.0f, 0.0f, false, false);
                        }
                    }
                    /* ========== 天文解算完成（后续所有模块使用本次结果）========== */
                    
                    /* ========== 夜间归位检测（带持续判断） ========== */
                    /* 检测太阳仰角，需要持续1分钟 < -5° 或 >= -5° 才改变状态 */
                    data_hub_snapshot_t sun_snapshot;
                    data_hub_get_snapshot(&sun_snapshot);
                    
                    uint32_t current_time = systick_get_tick();
                    
                    if (sun_snapshot.sun_valid) {
                        sun_position_state_t new_state = (sun_snapshot.sun_elevation < -5.0f) 
                            ? SUN_STATE_BELOW_HORIZON : SUN_STATE_ABOVE_HORIZON;
                        
                        /* 检测状态是否变化 */
                        if (new_state != g_sun_state) {
                            /* 状态变化，重置计时器 */
                            g_sun_state = new_state;
                            g_sun_state_start_time = current_time;
                        } else {
                            /* 状态保持不变，检查是否持续足够时间 */
                            uint32_t elapsed = current_time - g_sun_state_start_time;
                            
                            if (elapsed >= NIGHT_POSITION_CONFIRM_TIME_MS) {
                                /* 状态持续1分钟，确认有效 */
                                if (g_sun_state == SUN_STATE_BELOW_HORIZON) {
                                    /* 确认太阳落下：在进入夜间模式前先触发一次云台自检 */
                                    if (!g_night_position_confirmed) {
                                        if (!g_night_entry_selftest_done) {
                                            uint32_t now = systick_get_tick();
                                            if ((now - last_gimbal_cmd_time) >= GIMBAL_CMD_MIN_INTERVAL_MS) {
                                                gimbal_result_t selftest_ret = gimbal_restore_default_selftest();
                                                if (selftest_ret == GIMBAL_OK) {
                                                    g_night_entry_selftest_done = true;
                                                    last_gimbal_cmd_time = now;
                                                }
                                            }
                                        }

                                        if (g_night_entry_selftest_done) {
                                            g_night_position_confirmed = true;
                                        }
                                    }
                                } else {
                                    /* 确认太阳升起：直接退出夜间模式 */
                                    if (g_night_position_confirmed) {
                                        g_night_position_confirmed = false;
                                        g_night_entry_selftest_done = false;
                                    }
                                }
                            }
                        }
                    }
                    
                    /* 如果已确认夜间位置，执行夜间归位逻辑 */
                    if (g_night_position_confirmed) {
                        /* 太阳已落山（仰角持续1分钟 < -5°），夜间模式 */
                        if (g_runMode != 0U && g_config_runtime != NULL) {
                            float night_az = g_config_runtime->night_azimuth;
                            float night_el = g_config_runtime->night_elevation;
                            
                            /* 计算当前云台位置与夜间位置的误差 */
                            float az_error = fabsf(g_gimbal_position.azimuth - night_az);
                            float el_error = fabsf(g_gimbal_position.elevation - night_el);
                            
                            /* 处理方位角 360° 环绕误差 */
                            if (az_error > 180.0f) {
                                az_error = 360.0f - az_error;
                            }
                            
                            const float night_threshold = 0.5f;  /* 夜间位置误差阈值 */
                            
                            if (az_error > night_threshold || el_error > night_threshold) {
                                /* 只要偏离夜间位置，就再发一次夜间归位（使用节流控制） */
                                uint32_t now = systick_get_tick();
                                if ((now - last_gimbal_cmd_time) >= GIMBAL_CMD_MIN_INTERVAL_MS) {
                                    if (gimbal_queue_set_position(night_az, night_el)) {
                                        last_gimbal_cmd_time = now;
                                    } else {
                                        gimbal_queue_full_count++;  /* 记录队列满事件 */
                                    }
                                }
                            }
                        }
                        /* 夜间模式，跳过后续追踪逻辑 */
                        break;  /* 跳过后续追踪逻辑 */
                    }
                    /* 太阳已升起（仰角持续1分钟 >= -5°）时，继续执行后续追踪逻辑 */
                    /* ========== 夜间归位检测完成 ========== */
                    
                    /* 判断光照条件 */
                    bool brightness_sufficient = tracking_is_brightness_sufficient();
                    
                    if (g_runMode == 1U) {
                        /* ========== 天文追踪模式 ========== */
                        
                        /* 使用统一天文解算的结果（避免重复计算）*/
                        if (sun_ok && sun_visible) {
                            /* 应用纠偏值（光学追踪校准后的偏差补偿）*/
                            /* 纠偏值 = 实际位置 - 理论位置，因此需要加到理论值上 */
                            uint16_t az_offset_h = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_H);
                            uint16_t az_offset_l = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_L);
                            uint16_t el_offset_h = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_H);
                            uint16_t el_offset_l = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_L);
                            
                            float az_correction = modbus_rtu_registers_to_float(az_offset_h, az_offset_l);
                            float el_correction = modbus_rtu_registers_to_float(el_offset_h, el_offset_l);
                            
                            /* 应用纠偏值：发送角度 = 理论角度 + 纠偏值 */
                            float target_az = sun_azimuth + az_correction;
                            float target_el = sun_elevation + el_correction;
                            
                            /* 角度范围处理 */
                            while (target_az < 0.0f) target_az += 360.0f;
                            while (target_az >= 360.0f) target_az -= 360.0f;
                            if (target_el < 0.0f) target_el = 0.0f;
                            if (target_el > 90.0f) target_el = 90.0f;
                            
                            /* 判断云台是否已到位（实际位置与目标位置的差值）*/
                            float az_error = fabsf(g_gimbal_position.azimuth - target_az);
                            float el_error = fabsf(g_gimbal_position.elevation - target_el);
                            
                            /* 方位角处理360度边界（例如：359度到1度的差值是2度，而不是358度）*/
                            if (az_error > 180.0f) {
                                az_error = 360.0f - az_error;
                            }
                            
                            /* 到位阈值：0.5度 */
                            const float position_threshold = 0.5f;
                            
                            if (az_error > position_threshold || el_error > position_threshold) {
                                /* 未到位，发送云台命令（使用节流控制防止频繁发送） */
                                uint32_t now = systick_get_tick();
                                if ((now - last_gimbal_cmd_time) >= GIMBAL_CMD_MIN_INTERVAL_MS) {
                                    if (gimbal_queue_set_position(target_az, target_el)) {
                                        last_gimbal_cmd_time = now;
                                        optical_tracking_set_base_position(target_az, target_el);
                                    } else {
                                        gimbal_queue_full_count++;  /* 记录队列满事件 */
                                    }
                                }
                            } else {
                                /* 已到位，不发送命令，标记粗定位完成 */
                            }
                        }
                        
                        /* 判断是否切换到光学追踪
                         * 条件：光照充足（平均光强 >= 进入阈值）
                         * 只要亮度达到进入阈值，立即切换到光学模式
                         */
                        if (brightness_sufficient) {
                            tracking_apply_mode(2U);
                        }
                        
                    } else if (g_runMode == 2U) {
                        /* ========== 光学追踪模式 ========== */
                        
                        /* 使用退出阈值判断（滞回控制，防止频繁切换） */
                        bool brightness_below_exit = (g_sensor_data.valid && 
                                                      light_sensor_get_total_intensity() < g_optical_exit_threshold);
                        
                        if (brightness_below_exit) {
                            /* 光照低于退出阈值，切换回GPS模式 */
                            tracking_apply_mode(1U);
                        } else {
                            /* 仅在光感 + 云台这两份数据"新鲜且未被使用"时，才执行光学算法 */
                            if (g_light_data_fresh && g_gimbal_data_fresh) {
                                optical_tracking_update();

                                /* 光学跟踪模式使用过这一组数据后，清零标志 */
                                g_light_data_fresh  = false;
                                g_gimbal_data_fresh = false;
                            } else {
                                /* 任一标志为 false：本轮跳过光学算法，等待下一轮新数据 */
                            }
                        }
                    } else {
                        /* 手动模式 */
                    }
                }
                
                /* 否则继续等待（不执行任何操作）*/
                break;
            }
        }
        
        /* ========== 云台命令处理（仅在安全状态执行）========== */
        /* 关键：只在传感器查询阶段处理云台命令 */
        /* 避免位置命令发送干扰正在进行的云台角度查询 */
        if (g_bus_state == BUS_STATE_QUERY_SENSOR || 
            g_bus_state == BUS_STATE_WAIT_SENSOR) {
            gimbal_process();
        }

        app_watchdog_beat(APP_WDG_TASK_SOLAR);
        
        /* 最小延时让出CPU，保证快速循环 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*!
    \param[out] none
    \retval     none
*/
/*!
    \brief    Host communication task
    \param[in]  pvParameters: Task parameters (unused)
    \retval     none
    \note       Responsibilities:
                1. Process host Modbus RTU requests
                2. Update all register data (sensor, GPS, sun position, gimbal position)
                3. Execute host control commands
                4. Maintain device status and communication heartbeat
*/
static void HostComm_Task(void *pvParameters)
{
    host_cmd_msg_t cmd_msg;
    data_hub_snapshot_t snapshot;
    const EventBits_t wait_bits = DATA_HUB_EVENT_SENSOR |
                                  DATA_HUB_EVENT_SUN |
                                  DATA_HUB_EVENT_GIMBAL |
                                  DATA_HUB_EVENT_GPS;
    const TickType_t xWait = pdMS_TO_TICKS(20);
    
    /* Device status bit definitions */
    enum {
        STATUS_BIT_POWER_ON     = 0x0001,  /* Device powered on and running */
        STATUS_BIT_GPS_LOCKED   = 0x0002,  /* GPS locked */
        STATUS_BIT_SENSOR_OK    = 0x0004,  /* Sensor normal */
        STATUS_BIT_GIMBAL_OK    = 0x0008,  /* Gimbal communication normal */
        STATUS_BIT_TRACKING     = 0x0010,  /* Tracking in progress */
        STATUS_BIT_SUN_VISIBLE  = 0x0020,  /* Sun is visible (elevation >= -5°) */
        STATUS_BIT_SD_MOUNTED   = 0x0040   /* SD card mounted and ready */
    };

    (void)pvParameters;

    host_comm_register_task_handle(xTaskGetCurrentTaskHandle());
    
    /* 等待1秒让FreeRTOS和其他任务稳定 */
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    /* ========== 存储系统初始化（一次性执行）========== */
    storage_system_init();
    /* ========== 初始化完成 ========== */
    
    /* Initialize device status */
    modbus_rtu_slave_set_register(REG_HOST_DEVICE_STATUS, STATUS_BIT_POWER_ON);
    modbus_rtu_slave_set_register(REG_HOST_RUN_MODE, g_runMode);
    modbus_rtu_slave_set_register(REG_HOST_COMM_STATUS, 1);
    modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, 0);
    
    /* 设置固件版本：0xMMNN => Major.Minor (使用宏定义) */
    modbus_rtu_slave_set_register(REG_HOST_FIRMWARE_VERSION, FIRMWARE_VERSION);

    static bool s_last_gps_fix_valid = false;
    static bool s_boot_timezone_auto_checked = false;
    
    /* 上位机写经纬度事件检测 */
    static uint32_t s_last_host_loc_seq = 0;
    static bool s_host_loc_pending = false;
    static TickType_t s_host_loc_tick = 0;
    
    /* GPS首次Fix时区自动设置 */
    static bool s_gps_tz_pending = false;
    static TickType_t s_gps_tz_tick = 0;

    while (1) {
        EventBits_t bits = 0;

        (void)ulTaskNotifyTake(pdTRUE, xWait);

        if (DataEventGroup != NULL) {
            bits = xEventGroupWaitBits(DataEventGroup,
                                       wait_bits,
                                       pdTRUE,
                                       pdFALSE,
                                       0);
        }

        host_comm_process();
        host_comm_process_pending_calibration_save();

        if (g_config_runtime != NULL) {
            if (!s_boot_timezone_auto_checked) {
                s_boot_timezone_auto_checked = true;
                if (g_config_runtime->timezone_lock != 1U) {
                    int16_t current_tz = (int16_t)modbus_rtu_slave_get_register(REG_HOST_TIMEZONE);
                    int16_t auto_tz = calc_timezone_from_longitude((double)g_config_runtime->longitude);
                    if (auto_tz != current_tz) {
                        if (flash_config_set_timezone_and_lock(auto_tz, 0)) {
                            g_config_runtime = flash_config_get_runtime();
                            modbus_rtu_slave_set_register(REG_HOST_TIMEZONE, (uint16_t)(int16_t)auto_tz);
                        }
                    }
                }
            }

            int16_t tz_reg = (int16_t)modbus_rtu_slave_get_register(REG_HOST_TIMEZONE);
            if (tz_reg != g_config_runtime->timezone_offset_hours) {
                flash_config_set_timezone_and_lock(tz_reg, 1);
                g_config_runtime = flash_config_get_runtime();
            }

            /* ---------- 1) 上位机手动写经纬度：每次都自动计算并应用时区 ---------- */
            uint32_t loc_seq = modbus_rtu_slave_get_location_write_seq();
            if (loc_seq != s_last_host_loc_seq) {
                s_last_host_loc_seq = loc_seq;
                s_host_loc_pending = true;
                s_host_loc_tick = xTaskGetTickCount();
            }
            /* 合并处理：等待一小段时间，避免只写了一半寄存器就读取 */
            if (s_host_loc_pending && (xTaskGetTickCount() - s_host_loc_tick) > pdMS_TO_TICKS(80)) {
                s_host_loc_pending = false;
                uint16_t lat_h = modbus_rtu_slave_get_register(REG_HOST_LATITUDE_H);
                uint16_t lat_l = modbus_rtu_slave_get_register(REG_HOST_LATITUDE_L);
                uint16_t lon_h = modbus_rtu_slave_get_register(REG_HOST_LONGITUDE_H);
                uint16_t lon_l = modbus_rtu_slave_get_register(REG_HOST_LONGITUDE_L);
                float new_lat = modbus_rtu_registers_to_float(lat_h, lat_l);
                float new_lon = modbus_rtu_registers_to_float(lon_h, lon_l);
                
                /* 经纬度合法性校验 */
                if (new_lat >= -90.0f && new_lat <= 90.0f && 
                    new_lon >= -180.0f && new_lon <= 180.0f) {
                    /* 保存位置到flash */
                    if (flash_config_set_location(new_lat, new_lon)) {
                        g_config_runtime = flash_config_get_runtime();
                    }
                    /* 未锁定时区则每次按新经度自动算时区并应用 */
                    if (g_config_runtime != NULL && g_config_runtime->timezone_lock != 1U) {
                        int16_t current_tz = (int16_t)modbus_rtu_slave_get_register(REG_HOST_TIMEZONE);
                        int16_t auto_tz = calc_timezone_from_longitude((double)new_lon);
                        if (auto_tz != current_tz) {
                            if (flash_config_set_timezone_and_lock(auto_tz, 0)) {
                                g_config_runtime = flash_config_get_runtime();
                                modbus_rtu_slave_set_register(REG_HOST_TIMEZONE, (uint16_t)(int16_t)auto_tz);
                            }
                        }
                    }
                }
                /* else: 经纬度非法，不写flash、不算时区 */
            }

            /* ---------- 2) GPS 仅在首次获得Fix时自动计算并设置一次时区 ---------- */
            bool gps_fix_now = gps_has_fix();
            if (!gps_fix_now) {
                s_last_gps_fix_valid = false;
                s_gps_tz_pending = false;
            } else {
                if (!s_last_gps_fix_valid) {
                    s_last_gps_fix_valid = true;
                    s_gps_tz_pending = true;
                    s_gps_tz_tick = xTaskGetTickCount();
                }
            }
            /* pending后尝试一次：直接从GPS模块获取经度（避免读到旧寄存器值） */
            if (s_gps_tz_pending && g_config_runtime != NULL && g_config_runtime->timezone_lock != 1U) {
                double gps_lat, gps_lon;
                float gps_alt;
                if (gps_get_position(&gps_lat, &gps_lon, &gps_alt)) {
                    /* GPS模块返回有效位置，使用此经度计算时区 */
                    if (gps_lon >= -180.0 && gps_lon <= 180.0 && fabs(gps_lon) > 0.0001) {
                        int16_t current_tz = (int16_t)modbus_rtu_slave_get_register(REG_HOST_TIMEZONE);
                        int16_t auto_tz = calc_timezone_from_longitude(gps_lon);
                        if (auto_tz != current_tz) {
                            if (flash_config_set_timezone_and_lock(auto_tz, 0)) {
                                g_config_runtime = flash_config_get_runtime();
                                modbus_rtu_slave_set_register(REG_HOST_TIMEZONE, (uint16_t)(int16_t)auto_tz);
                            }
                        }
                        /* 关键：只做一次 */
                        s_gps_tz_pending = false;
                    }
                } else {
                    /* GPS模块暂未返回有效位置，超时放弃（3秒） */
                    if ((xTaskGetTickCount() - s_gps_tz_tick) > pdMS_TO_TICKS(3000)) {
                        s_gps_tz_pending = false;
                    }
                }
            }
            
            /* 检查光控阈值寄存器是否被修改 */
            uint16_t enter_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ENTER_THRESHOLD);
            uint16_t exit_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_EXIT_THRESHOLD);
            float new_enter = (float)enter_reg / 100.0f;  /* 寄存器单位：0.01V */
            float new_exit = (float)exit_reg / 100.0f;
            
            /* 检查是否有变化（容差0.01V） */
            if (fabsf(new_enter - g_optical_enter_threshold) > 0.005f ||
                fabsf(new_exit - g_optical_exit_threshold) > 0.005f) {
                
                /* 参数合法性检查：退出阈值应小于进入阈值 */
                if (new_exit < new_enter && new_enter > 0.0f && new_exit > 0.0f && 
                    new_enter <= 4.0f && new_exit <= 4.0f) {
                    
                    g_optical_enter_threshold = new_enter;
                    g_optical_exit_threshold = new_exit;
                    
                    /* 保存到Flash */
                    flash_config_set_optical_thresholds(new_enter, new_exit);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器为当前有效值 */
                    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ENTER_THRESHOLD, 
                                                  (uint16_t)(g_optical_enter_threshold * 100.0f + 0.5f));
                    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_EXIT_THRESHOLD, 
                                                  (uint16_t)(g_optical_exit_threshold * 100.0f + 0.5f));
                }
            }
            
            /* 检查GPS校时间隔寄存器是否被修改 */
            uint16_t gps_interval_reg = modbus_rtu_slave_get_register(REG_HOST_GPS_CALIB_INTERVAL);
            if (gps_interval_reg != g_config_runtime->gps_calib_interval_hours) {
                if (gps_interval_reg >= 1 && gps_interval_reg <= 168) {
                    flash_config_set_gps_interval(gps_interval_reg);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器 */
                    modbus_rtu_slave_set_register(REG_HOST_GPS_CALIB_INTERVAL, 
                                                  g_config_runtime->gps_calib_interval_hours);
                }
            }
            
            /* 检查夜间归位位置寄存器是否被修改 */
            uint16_t night_az_h = modbus_rtu_slave_get_register(REG_HOST_NIGHT_AZIMUTH_H);
            uint16_t night_az_l = modbus_rtu_slave_get_register(REG_HOST_NIGHT_AZIMUTH_L);
            uint16_t night_el_h = modbus_rtu_slave_get_register(REG_HOST_NIGHT_ELEVATION_H);
            uint16_t night_el_l = modbus_rtu_slave_get_register(REG_HOST_NIGHT_ELEVATION_L);
            
            float new_night_az = modbus_rtu_registers_to_float(night_az_h, night_az_l);
            float new_night_el = modbus_rtu_registers_to_float(night_el_h, night_el_l);
            
            if (fabsf(new_night_az - g_config_runtime->night_azimuth) > 0.1f ||
                fabsf(new_night_el - g_config_runtime->night_elevation) > 0.1f) {
                
                /* 参数合法性检查 */
                if (new_night_az >= 0.0f && new_night_az < 360.0f &&
                    new_night_el >= 0.0f && new_night_el <= 90.0f) {
                    flash_config_set_night_position(new_night_az, new_night_el);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器 */
                    uint16_t reg_h, reg_l;
                    modbus_rtu_float_to_registers(g_config_runtime->night_azimuth, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_L, reg_l);
                    modbus_rtu_float_to_registers(g_config_runtime->night_elevation, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_L, reg_l);
                }
            }
            
            /* 检查光学追踪死区阈值寄存器是否被修改 */
            uint16_t err_enter_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ERR_ENTER);
            uint16_t err_exit_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ERR_EXIT);
            float new_err_enter = (float)err_enter_reg / 1000.0f;  /* 寄存器单位：0.001 */
            float new_err_exit = (float)err_exit_reg / 1000.0f;
            
            /* 检查是否有变化（容差0.001） */
            if (fabsf(new_err_enter - g_optical_error_enter) > 0.0005f ||
                fabsf(new_err_exit - g_optical_error_exit) > 0.0005f) {
                
                /* 参数合法性检查：enter < exit，且都在 (0, 1] 范围 */
                if (new_err_enter > 0.0f && new_err_enter <= 1.0f &&
                    new_err_exit > 0.0f && new_err_exit <= 1.0f &&
                    new_err_enter < new_err_exit) {
                    
                    g_optical_error_enter = new_err_enter;
                    g_optical_error_exit = new_err_exit;
                    
                    /* 立即应用到光学追踪模块 */
                    optical_tracking_set_deadband(new_err_enter, new_err_exit);
                    
                    /* 保存到Flash */
                    flash_config_set_optical_error_deadband(new_err_enter, new_err_exit);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器为当前有效值 */
                    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_ENTER, 
                                                  (uint16_t)(g_optical_error_enter * 1000.0f + 0.5f));
                    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_EXIT, 
                                                  (uint16_t)(g_optical_error_exit * 1000.0f + 0.5f));
                }
            }

            /* 检查太阳高度角工作范围寄存器是否被修改 */
            uint16_t sun_elev_min_h = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MIN_H);
            uint16_t sun_elev_min_l = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MIN_L);
            uint16_t sun_elev_max_h = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MAX_H);
            uint16_t sun_elev_max_l = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MAX_L);

            float new_sun_elev_min = modbus_rtu_registers_to_float(sun_elev_min_h, sun_elev_min_l);
            float new_sun_elev_max = modbus_rtu_registers_to_float(sun_elev_max_h, sun_elev_max_l);

            if (fabsf(new_sun_elev_min - g_config_runtime->sun_elevation_work_min) > 0.1f ||
                fabsf(new_sun_elev_max - g_config_runtime->sun_elevation_work_max) > 0.1f) {

                /* 参数合法性检查：min < max，且在合理范围内 */
                if (new_sun_elev_min >= -90.0f && new_sun_elev_min <= 90.0f &&
                    new_sun_elev_max >= -90.0f && new_sun_elev_max <= 90.0f &&
                    new_sun_elev_min < new_sun_elev_max) {

                    /* 保存到Flash */
                    flash_config_set_sun_elevation_work_range(new_sun_elev_min, new_sun_elev_max);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器 */
                    uint16_t reg_h, reg_l;
                    modbus_rtu_float_to_registers(g_config_runtime->sun_elevation_work_min, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_L, reg_l);
                    modbus_rtu_float_to_registers(g_config_runtime->sun_elevation_work_max, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_L, reg_l);
                }
            }

            /* TODO: [TEST] 检查云台仰角工作范围寄存器变化 - 测试后需移除此代码块 (约30行) */
            /* 检查云台仰角工作范围寄存器是否被修改 */
            uint16_t gimbal_elev_min_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MIN_H);
            uint16_t gimbal_elev_min_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MIN_L);
            uint16_t gimbal_elev_max_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MAX_H);
            uint16_t gimbal_elev_max_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MAX_L);

            float new_gimbal_elev_min = modbus_rtu_registers_to_float(gimbal_elev_min_h, gimbal_elev_min_l);
            float new_gimbal_elev_max = modbus_rtu_registers_to_float(gimbal_elev_max_h, gimbal_elev_max_l);

            if (fabsf(new_gimbal_elev_min - g_config_runtime->gimbal_elevation_min) > 0.1f ||
                fabsf(new_gimbal_elev_max - g_config_runtime->gimbal_elevation_max) > 0.1f) {

                /* 参数合法性检查：min < max，且在 [0, 90] 范围内 */
                if (new_gimbal_elev_min >= 0.0f && new_gimbal_elev_min <= 90.0f &&
                    new_gimbal_elev_max >= 0.0f && new_gimbal_elev_max <= 90.0f &&
                    new_gimbal_elev_min < new_gimbal_elev_max) {

                    /* 保存到Flash */
                    flash_config_set_gimbal_elevation_range(new_gimbal_elev_min, new_gimbal_elev_max);
                    g_config_runtime = flash_config_get_runtime();
                } else {
                    /* 参数非法，恢复寄存器 */
                    uint16_t reg_h, reg_l;
                    modbus_rtu_float_to_registers(g_config_runtime->gimbal_elevation_min, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_L, reg_l);
                    modbus_rtu_float_to_registers(g_config_runtime->gimbal_elevation_max, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_L, reg_l);
                }
            }
        }

        while (xQueueReceive(xHostCmdQueue, &cmd_msg, 0) == pdTRUE) {
            switch (cmd_msg.cmd_code) {
                case CMD_MOVE_UP:
                    gimbal_queue_command(GIMBAL_UP);
                    break;
                case CMD_MOVE_DOWN:
                    gimbal_queue_command(GIMBAL_DOWN);
                    break;
                case CMD_MOVE_LEFT:
                    gimbal_queue_command(GIMBAL_LEFT);
                    break;
                case CMD_MOVE_RIGHT:
                    gimbal_queue_command(GIMBAL_RIGHT);
                    break;
                case CMD_STOP_MOVE:
                    gimbal_queue_command(GIMBAL_STOP);
                    break;
                case CMD_SET_POSITION:
                    gimbal_queue_set_position(cmd_msg.param1, cmd_msg.param2);
                    break;
                case CMD_ENTER_GPS_MODE:
                    tracking_apply_mode(1U);
                    break;
                case CMD_ENTER_OPTICAL_MODE:
                    tracking_apply_mode(2U);
                    break;
                case CMD_ENTER_IDLE_MODE:
                    tracking_apply_mode(0U);
                    break;
                case CMD_SAVE_PARAMS:
                    /* Reserved for future use */
                    break;
                case CMD_GIMBAL_SELFTEST:
                    /* 云台恢复默认参数+自检 */
                    (void)gimbal_restore_default_selftest();
                    break;
                default:
                    break;
            }
        }

        if ((bits != 0U) && (DataEventGroup != NULL)) {
            data_hub_get_snapshot(&snapshot);

            if (bits & DATA_HUB_EVENT_SENSOR) {
                host_comm_set_light_data(snapshot.sensor.up_voltage,
                                         snapshot.sensor.left_voltage,
                                         snapshot.sensor.down_voltage,
                                         snapshot.sensor.right_voltage);
                if (snapshot.sensor.valid) {
                    modbus_rtu_slave_set_register(REG_HOST_SENSOR_V0, (uint16_t)(snapshot.sensor.up_voltage * 1000));
                    modbus_rtu_slave_set_register(REG_HOST_SENSOR_V1, (uint16_t)(snapshot.sensor.left_voltage * 1000));
                    modbus_rtu_slave_set_register(REG_HOST_SENSOR_V2, (uint16_t)(snapshot.sensor.down_voltage * 1000));
                    modbus_rtu_slave_set_register(REG_HOST_SENSOR_V3, (uint16_t)(snapshot.sensor.right_voltage * 1000));
                }
            }

            static bool s_hasSunSample = false;
            static bool s_gpsSunErrorRaised = false;

            if ((bits & DATA_HUB_EVENT_SUN) != 0U) {
                if (snapshot.sun_valid) {
                    uint16_t reg_h, reg_l;
                    modbus_rtu_float_to_registers(snapshot.sun_azimuth, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_AZIMUTH_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_AZIMUTH_L, reg_l);
                    modbus_rtu_float_to_registers(snapshot.sun_elevation, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEVATION_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEVATION_L, reg_l);

                    s_hasSunSample = true;
                } else if (!s_hasSunSample) {
                    modbus_rtu_slave_set_register(REG_HOST_SUN_AZIMUTH_H, 0);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_AZIMUTH_L, 0);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEVATION_H, 0);
                    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEVATION_L, 0);
                }

                if (!snapshot.sun_valid) {
                    host_comm_set_error_code(0x0101U); /* GPS not locked, sun position invalid */
                    s_gpsSunErrorRaised = true;
                } else if (s_gpsSunErrorRaised) {
                    host_comm_set_error_code(0);
                    s_gpsSunErrorRaised = false;
                }
            }

            if (bits & DATA_HUB_EVENT_GIMBAL) {
                if (snapshot.gimbal_valid) {
                    uint16_t reg_h, reg_l;
                    modbus_rtu_float_to_registers(snapshot.gimbal_azimuth, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_L, reg_l);
                    modbus_rtu_float_to_registers(snapshot.gimbal_elevation, &reg_h, &reg_l);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_H, reg_h);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_L, reg_l);
                } else {
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_H, 0);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_L, 0);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_H, 0);
                    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_L, 0);
                }
            }

            if (bits & DATA_HUB_EVENT_GPS) {
                /* GPS status register composition:
                 * Bit 0: GPS fix valid
                 * Bit 1-3: Fix quality (0=invalid, 1=GPS, 2=DGPS)
                 * Bit 4-11: Satellite count (0-255)
                 */
                uint16_t gps_status = 0;
                
                if (gps_has_fix()) {
                    gps_status |= 0x0001;  /* Fix valid */
                    
                    /* Update GPS latitude/longitude to registers */
                    double latitude, longitude;
                    float altitude;
                    if (gps_get_position(&latitude, &longitude, &altitude)) {
                        uint16_t reg_h, reg_l;
                        modbus_rtu_float_to_registers((float)latitude, &reg_h, &reg_l);
                        modbus_rtu_slave_set_register(REG_HOST_LATITUDE_H, reg_h);
                        modbus_rtu_slave_set_register(REG_HOST_LATITUDE_L, reg_l);
                        modbus_rtu_float_to_registers((float)longitude, &reg_h, &reg_l);
                        modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_H, reg_h);
                        modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_L, reg_l);
                        /* GPS时区自动计算已移至HostComm_Task中的s_gps_tz_pending机制处理 */
                    }
                }
                
                gps_data_t* gps_data = gps_get_data();
                if (gps_data != NULL) {
                    /* Fix quality (bits 1-2) */
                    uint16_t quality = (uint16_t)gps_data->fix_quality & 0x07;
                    gps_status |= (quality << 1);
                    
                    /* Satellite count (bits 4-11) */
                    uint16_t sats = (uint16_t)gps_data->satellites & 0xFF;
                    gps_status |= (sats << 4);
                }
                
                modbus_rtu_slave_set_register(REG_HOST_GPS_STATUS, gps_status);
            }
        }

        /* Update device status every 100ms */
        static uint32_t last_status_update = 0;
        uint32_t current_tick = xTaskGetTickCount();
        
        if ((current_tick - last_status_update) >= pdMS_TO_TICKS(100)) {
            uint16_t device_status = STATUS_BIT_POWER_ON;
            
            /* 处理SD卡热插拔事件 */
             sd_logger_process_hotplug_events();
            
            /* Check GPS status */
            if (gps_has_fix()) {
                device_status |= STATUS_BIT_GPS_LOCKED;
            }
            
            /* Check sensor status */
            if (g_sensor_data.valid) {
                device_status |= STATUS_BIT_SENSOR_OK;
            }
            
            /* Check gimbal communication status */
            gimbal_position_t gimbal_pos;
            if (gimbal_get_cached_position(&gimbal_pos)) {
                /* 云台数据可用，认为通信正常 */
                device_status |= STATUS_BIT_GIMBAL_OK;
            }
            
            /* Check tracking status */
            if (g_runMode != 0U) {
                device_status |= STATUS_BIT_TRACKING;
            }
            
            /* Check sun visibility (elevation >= 5°) */
            data_hub_snapshot_t sun_snapshot;
            data_hub_get_snapshot(&sun_snapshot);
            if (sun_snapshot.sun_valid) {
                if (sun_position_is_visible(sun_snapshot.sun_elevation)) {
                    device_status |= STATUS_BIT_SUN_VISIBLE;
                }
            }
            
            /* Check SD card mount status */
            if (sd_logger_is_mounted()) {
                device_status |= STATUS_BIT_SD_MOUNTED;
            }
            
            modbus_rtu_slave_set_register(REG_HOST_DEVICE_STATUS, device_status);
            modbus_rtu_slave_set_register(REG_HOST_COMM_STATUS, 1); /* Communication normal */
            last_status_update = current_tick;
        }
        
        /* Periodically update RTC time to Modbus registers (every 1 second for better responsiveness) */
        static TickType_t last_rtc_update = 0;
        if ((current_tick - last_rtc_update) >= pdMS_TO_TICKS(1000)) {
            rtc_datetime_t local_time;
            /* 使用统一时间接口获取本地时间 */
            if (time_get_local_datetime(&local_time)) {
                modbus_rtu_slave_set_register(REG_HOST_RTC_YEAR, local_time.year);
                modbus_rtu_slave_set_register(REG_HOST_RTC_MONTH, local_time.month);
                modbus_rtu_slave_set_register(REG_HOST_RTC_DAY, local_time.day);
                modbus_rtu_slave_set_register(REG_HOST_RTC_HOUR, local_time.hour);
                modbus_rtu_slave_set_register(REG_HOST_RTC_MINUTE, local_time.minute);
                modbus_rtu_slave_set_register(REG_HOST_RTC_SECOND, local_time.second);
            } else {
                /* RTC无效或未初始化：保持寄存器不变，等待GPS同步 */
                /* 不设置0xFFFF，避免覆盖GPS可能已经设置的有效时间 */
            }
            last_rtc_update = current_tick;
        }
        
        /* Periodically log tracking state for diagnostics (every 10 seconds) */
        static TickType_t last_tracking_log = 0;
        if ((current_tick - last_tracking_log) >= pdMS_TO_TICKS(10000)) {
            /* 只在非Idle模式时记录，或者全程记录（根据需求调整） */
            if (g_runMode != 0U) {
                tracking_state_log_t log_state;
                
                /* 获取本地时间 */
                if (time_get_local_datetime(&log_state.time_local)) {
                    /* 系统模式 */
                    log_state.run_mode = g_runMode;
                    log_state.opt_mode = optical_tracking_get_mode();
                    log_state.gps_fix = gps_has_fix();
                    
                    /* 理论太阳位置（北=0°坐标系，从缓存读取） */
                    if (g_sun_cache.valid) {
                        log_state.sun_az = g_sun_cache.azimuth;
                        log_state.sun_el = g_sun_cache.elevation;
                    } else {
                        log_state.sun_az = 0.0f;
                        log_state.sun_el = 0.0f;
                    }
                    
                    /* 云台实际位置（工程坐标系） */
                    gimbal_position_t gimbal_pos;
                    if (gimbal_get_cached_position(&gimbal_pos)) {
                        log_state.gimbal_az = gimbal_pos.azimuth;
                        log_state.gimbal_el = gimbal_pos.elevation;
                    } else {
                        log_state.gimbal_az = 0.0f;
                        log_state.gimbal_el = 0.0f;
                    }
                    
                    /* 四路光敏电压 */
                    if (g_sensor_data.valid) {
                        log_state.up_v = g_sensor_data.up_voltage;
                        log_state.left_v = g_sensor_data.left_voltage;
                        log_state.right_v = g_sensor_data.right_voltage;
                        log_state.down_v = g_sensor_data.down_voltage;
                    } else {
                        log_state.up_v = 0.0f;
                        log_state.left_v = 0.0f;
                        log_state.right_v = 0.0f;
                        log_state.down_v = 0.0f;
                    }
                    
                    /* 光学追踪误差：仅在光追模式实际开启且传感器数据有效时记录 */
                    if (optical_tracking_get_mode() == OPT_MODE_OPTICAL_TRACKING && g_sensor_data.valid) {
                        if (!optical_tracking_get_errors(&log_state.ex, &log_state.ey)) {
                            log_state.ex = 0.0f;
                            log_state.ey = 0.0f;
                        }
                    } else {
                        log_state.ex = 0.0f;
                        log_state.ey = 0.0f;
                    }
                    
                    /* 写入SD卡日志 */
                    sd_logger_log_tracking_state(&log_state);
                }
            }
            last_tracking_log = current_tick;
        }

        app_watchdog_beat(APP_WDG_TASK_HOST);
    }
}

/*!
    \brief    GPS数据处理任务
    \param[out] none
    \retval     none
*/
static void GPS_Task(void *pvParameters)
{
    QueueHandle_t rxQueue = gps_get_rx_queue();
    uint8_t byte = 0;

    (void)pvParameters;

    for(;;) {
        if (rxQueue != NULL && xQueueReceive(rxQueue, &byte, pdMS_TO_TICKS(100)) == pdPASS) {
            gps_process_byte(byte);
            while (rxQueue != NULL && xQueueReceive(rxQueue, &byte, 0) == pdPASS) {
                gps_process_byte(byte);
            }
        }

        app_watchdog_beat(APP_WDG_TASK_GPS);
    }
}

/*!
    \brief    存储系统初始化（一次性函数，由HostComm_Task启动时调用）
    \detail   初始化Flash配置存储和SD卡日志系统
              - Flash: 存储系统配置参数（时区、光控阈值等）
              - SD卡: 存储系统日志和校准历史数据
              优化：不再使用独立任务，节省1536字节栈空间
*/
static void storage_system_init(void)
{
    /* 初始化Flash配置存储 */
    if (!flash_config_init()) {
        /* Config storage failed, using defaults */
    }
    g_config_runtime = flash_config_get_runtime();

    if (g_config_runtime != NULL) {
        modbus_rtu_slave_set_register(REG_HOST_TIMEZONE,
                                      (uint16_t)(int16_t)g_config_runtime->timezone_offset_hours);
        
        /* 从配置加载光控阈值 */
        g_optical_enter_threshold = g_config_runtime->optical_enter_threshold;
        g_optical_exit_threshold = g_config_runtime->optical_exit_threshold;
        
        /* 同步到Modbus寄存器（单位：0.01V） */
        modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ENTER_THRESHOLD, 
                                      (uint16_t)(g_optical_enter_threshold * 100.0f + 0.5f));
        modbus_rtu_slave_set_register(REG_HOST_OPTICAL_EXIT_THRESHOLD, 
                                      (uint16_t)(g_optical_exit_threshold * 100.0f + 0.5f));
        
        /* 从配置加载光学追踪死区阈值 */
        g_optical_error_enter = g_config_runtime->optical_error_enter;
        g_optical_error_exit = g_config_runtime->optical_error_exit;
        
        /* 同步到Modbus寄存器（单位：0.001） */
        modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_ENTER, 
                                      (uint16_t)(g_optical_error_enter * 1000.0f + 0.5f));
        modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_EXIT, 
                                      (uint16_t)(g_optical_error_exit * 1000.0f + 0.5f));
        
        /* 应用死区阈值到光学追踪模块 */
        optical_tracking_set_deadband(g_optical_error_enter, g_optical_error_exit);
    }
    
    /* 初始化SD卡日志系统 */
    if (sd_logger_init()) {
        sd_logger_printf("=== System Started ===");
        sd_logger_printf("Firmware: V3.3");
        sd_logger_printf("Device: TwoAxis Solar Tracker");
        sd_logger_printf("Architecture: Optimized (4 tasks)");
    }
    
    /* 从Flash加载参数到Modbus寄存器（供上位机读取） */
    host_comm_load_params_from_flash();
    
    /* 标记存储系统初始化完成 */
    g_storage_test_done = true;
}


static void LED_Task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        /* Check sensor data validity */
        bool led_error = !g_sensor_data.valid;
        app_watchdog_beat(APP_WDG_TASK_LED);
        
        if (led_error) {
            /* Error: fast blinking */
            led_toggle();
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
        } else {
            /* Normal: LED off */
            led_set(false);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        }
    }
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

/*!
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook(void)
{
}
#endif

/*!
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if (configUSE_TICK_HOOK == 1)
void vApplicationTickHook(void)
{
    /* 用户计时已移至 TIMER5 独立处理，此处为空 */
    /* TIMER5 中断每 1ms 调用 delay_decrement() */
}
#endif

/*!
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    led_set(true);
    for(;;);
}
#endif

/*!
    \param[out] none
    \retval     none
*/
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    led_set(true);
    for(;;);
}
#endif






