/*
 * light_sensor.h - Four-channel light sensor interface (unified)
 *
 * 2024-12-21, V4.0.0, Unified light sensor module for GD32F4xx
 */
#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "gd32f4xx.h"
#include "systick.h"
#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware configuration */
#define LIGHT_SENSOR_RS485_DE_RE_PORT      GPIOA
#define LIGHT_SENSOR_RS485_DE_RE_PIN       GPIO_PIN_7
#define LIGHT_SENSOR_RS485_PRE_DELAY_MS    5U
#define LIGHT_SENSOR_RS485_POST_DELAY_MS   2U
#define LIGHT_SENSOR_RS485_RX_SETTLE_MS    2U

/* Communication timeout */
#define SENSOR_COMM_TIMEOUT_MS         200U
#define SENSOR_COMM_INTERVAL_MS        100U

/* Sensor data structure */
typedef struct {
    float    up_voltage;
    float    left_voltage;
    float    down_voltage;
    float    right_voltage;
    bool     valid;
} sensor_data_t;

/* Global data variable */
extern sensor_data_t g_sensor_data;

/* ==================== High-level API (sensor_comm_*) ==================== */
bool sensor_comm_init(void);
bool sensor_comm_get_data(sensor_data_t* data);
bool sensor_comm_start_async(void);
bool sensor_comm_poll_async(uint32_t now, sensor_data_t* out_data, bool* done);

/* ==================== Low-level API (light_sensor_*) ==================== */
bool light_sensor_init(void);
bool light_sensor_send_request(void);
bool light_sensor_wait_for_response(uint32_t timeout_ms);
void light_sensor_request_abort(void);

float light_sensor_get_total_intensity(void);

void light_sensor_on_byte(uint8_t byte, BaseType_t *pxHigherPriorityTaskWoken);

#ifdef __cplusplus
}
#endif

#endif /* LIGHT_SENSOR_H */
