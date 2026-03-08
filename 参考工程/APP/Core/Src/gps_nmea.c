/*!
    \file    gps_nmea.c
    \brief   GPS NMEA0183 data parser implementation (FreeRTOS adaptation)

    \version 2025-01-10, V2.0, GPS NMEA parser for GD32F4xx + FreeRTOS
*/

#include "gps_nmea.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef GPS_NMEA_DEBUG
#define GPS_NMEA_DEBUG 0
#endif

#if GPS_NMEA_DEBUG
#include <stdio.h>
#define GPS_DEBUG_PRINTF(...)   printf(__VA_ARGS__)
#else
#define GPS_DEBUG_PRINTF(...)
#endif

/* -------------------------------------------------------------------------- */
/* Static variables                                                           */
/* -------------------------------------------------------------------------- */

static char gps_buffer[GPS_BUFFER_SIZE];
static uint16_t gps_buffer_index = 0;
static gps_data_t gps_data;
static uint32_t last_data_update_time = 0;
static volatile uint32_t last_rx_tick = 0;

static QueueHandle_t gpsRxQueue = NULL;

/* -------------------------------------------------------------------------- */
/* Local helper declarations                                                  */
/* -------------------------------------------------------------------------- */

static void gps_data_init(void);
static uint8_t gps_parse_sentence(char* sentence);
static uint8_t gps_parse_gprmc(char* sentence);
static uint8_t gps_parse_gpgga(char* sentence);
static uint8_t gps_parse_gpgsa(char* sentence);
static uint8_t gps_parse_gpgsv(char* sentence);
static float gps_convert_coordinate(char* coord_str, char direction);
static void gps_convert_time(char* time_str, gps_time_t* time);
static void gps_convert_date(char* date_str, gps_date_t* date);
static uint8_t gps_verify_checksum(char* sentence);
static uint8_t gps_calculate_checksum(char* sentence);

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

bool gps_init(void)
{
    if (gpsRxQueue == NULL) {
        gpsRxQueue = xQueueCreate(GPS_RX_QUEUE_LENGTH, sizeof(uint8_t));
        if (gpsRxQueue == NULL) {
            return false;
        }
    }

    gps_data_init();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        delay_1ms(100);
    }

    gps_uart_init();

    return true;
}

bool gps_reinit(void)
{
    usart_disable(GPS_USART);
    usart_interrupt_disable(GPS_USART, USART_INT_RBNE);
    nvic_irq_disable(GPS_USART_IRQn);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        delay_1ms(200);
    }

    gps_data_init();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(300));
    } else {
        delay_1ms(300);
    }

    gps_uart_init();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        delay_1ms(1000);
    }

    return true;
}

QueueHandle_t gps_get_rx_queue(void)
{
    return gpsRxQueue;
}

gps_data_t* gps_get_data(void)
{
    return &gps_data;
}

uint32_t gps_get_data_age_ms(void)
{
    uint32_t current = systick_get_tick();
    if (last_data_update_time == 0) {
        return UINT32_MAX;
    }
    if (current >= last_data_update_time) {
        return current - last_data_update_time;
    }
    return (UINT32_MAX - last_data_update_time) + current + 1;
}

bool gps_has_fix(void)
{
    return (gps_data.data_valid && gps_data.fix_status == GPS_FIX_VALID);
}

bool gps_is_communicating(void)
{
    uint32_t current = systick_get_tick();
    return (current - last_rx_tick) < 5000U;
}

bool gps_communication_healthy(void)
{
    return gps_is_communicating() && (gps_has_fix() || gps_data.satellites > 0);
}

bool gps_get_position(double* latitude, double* longitude, float* altitude)
{
    if (!latitude || !longitude || !altitude) {
        return false;
    }

    taskENTER_CRITICAL();
    bool valid = gps_has_fix();
    if (valid) {
        *latitude = gps_data.coordinate.latitude;
        *longitude = gps_data.coordinate.longitude;
        *altitude = gps_data.altitude;
    }
    taskEXIT_CRITICAL();
    return valid;
}

bool gps_get_time(uint16_t* year, uint8_t* month, uint8_t* day,
                  uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    if (!year || !month || !day || !hour || !minute || !second) {
        return false;
    }

    taskENTER_CRITICAL();
    bool valid = gps_has_fix();
    if (valid) {
        /* 额外检查日期有效性：年份应该大于2020 */
        if (gps_data.date.year > 2020 && gps_data.date.month >= 1 && gps_data.date.month <= 12 &&
            gps_data.date.day >= 1 && gps_data.date.day <= 31) {
            *year   = gps_data.date.year;
            *month  = gps_data.date.month;
            *day    = gps_data.date.day;
            *hour   = gps_data.time.hour;
            *minute = gps_data.time.minute;
            *second = gps_data.time.second;
        } else {
            valid = false;  /* 日期无效，不返回时间 */
        }
    }
    taskEXIT_CRITICAL();
    return valid;
}

/* -------------------------------------------------------------------------- */
/* UART handling                                                              */
/* -------------------------------------------------------------------------- */

void gps_uart_init(void)
{
    rcu_periph_clock_enable(GPS_GPIO_CLK);
    rcu_periph_clock_enable(GPS_USART_CLK);

    gpio_af_set(GPS_GPIO_PORT, GPS_GPIO_AF, GPS_TX_PIN);
    gpio_af_set(GPS_GPIO_PORT, GPS_GPIO_AF, GPS_RX_PIN);

    gpio_mode_set(GPS_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPS_TX_PIN | GPS_RX_PIN);
    gpio_output_options_set(GPS_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPS_TX_PIN | GPS_RX_PIN);

    usart_deinit(GPS_USART);
    usart_baudrate_set(GPS_USART, 9600U);
    usart_receive_config(GPS_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(GPS_USART, USART_TRANSMIT_ENABLE);

    usart_interrupt_enable(GPS_USART, USART_INT_RBNE);
    nvic_irq_enable(GPS_USART_IRQn, 6, 0);

    usart_enable(GPS_USART);
}

void gps_uart_irq_handler(BaseType_t* pxHigherPriorityTaskWoken)
{
    uint8_t byte = 0;

    if (usart_interrupt_flag_get(GPS_USART, USART_INT_FLAG_RBNE) != RESET) {
        byte = (uint8_t)usart_data_receive(GPS_USART);
        /* 检查调度器是否已启动且队列已创建 */
        if (gpsRxQueue != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            xQueueSendFromISR(gpsRxQueue, &byte, pxHigherPriorityTaskWoken);
        }
        last_rx_tick = systick_get_tick();
    }

    if (usart_flag_get(GPS_USART, USART_FLAG_ORERR) != RESET) {
        usart_flag_clear(GPS_USART, USART_FLAG_ORERR);
        (void)usart_data_receive(GPS_USART);
    }

    if (usart_flag_get(GPS_USART, USART_FLAG_FERR) != RESET) {
        usart_flag_clear(GPS_USART, USART_FLAG_FERR);
        (void)usart_data_receive(GPS_USART);
    }
}

/* -------------------------------------------------------------------------- */
/* Processing                                                                 */
/* -------------------------------------------------------------------------- */

static void gps_data_init(void)
{
    memset(&gps_data, 0, sizeof(gps_data_t));
    gps_data.fix_status = GPS_FIX_INVALID;
    gps_data.fix_quality = GPS_QUALITY_INVALID;
    
    /* 不设置默认日期，保持0值，等待GPS提供真实日期 */
    /* 这样可以避免在GPS未定位时将错误的默认日期写入RTC */
    gps_data.date.year = 0;
    gps_data.date.month = 0;
    gps_data.date.day = 0;
    
    gps_buffer_index = 0;
    last_data_update_time = 0;
    last_rx_tick = systick_get_tick();
}

void gps_process_byte(uint8_t byte)
{
    if (byte == '$') {
        gps_buffer_index = 0;
    }

    if (gps_buffer_index < (GPS_BUFFER_SIZE - 1U)) {
        gps_buffer[gps_buffer_index++] = (char)byte;

        if (byte == '\n') {
            gps_buffer[gps_buffer_index] = '\0';

            if (gps_parse_sentence(gps_buffer)) {
                gps_data.data_valid = 1U;
                last_data_update_time = systick_get_tick();
            }
        }
    } else {
        gps_buffer_index = 0;
    }
}

static uint8_t gps_parse_sentence(char* sentence)
{
    if (!sentence || !gps_verify_checksum(sentence)) {
        return 0U;
    }

    if (strstr(sentence, "$GPRMC") == sentence || strstr(sentence, "$GNRMC") == sentence) {
        uint8_t result;
        taskENTER_CRITICAL();
        result = gps_parse_gprmc(sentence);
        taskEXIT_CRITICAL();
        return result;
    }
    if (strstr(sentence, "$GPGGA") == sentence || strstr(sentence, "$GNGGA") == sentence) {
        uint8_t result;
        taskENTER_CRITICAL();
        result = gps_parse_gpgga(sentence);
        taskEXIT_CRITICAL();
        return result;
    }
    if (strstr(sentence, "$GPGSA") == sentence || strstr(sentence, "$GNGSA") == sentence) {
        uint8_t result;
        taskENTER_CRITICAL();
        result = gps_parse_gpgsa(sentence);
        taskEXIT_CRITICAL();
        return result;
    }
    if (strstr(sentence, "$GPGSV") == sentence || strstr(sentence, "$GNGSV") == sentence) {
        uint8_t result;
        taskENTER_CRITICAL();
        result = gps_parse_gpgsv(sentence);
        taskEXIT_CRITICAL();
        return result;
    }
    return 0U;
}

static uint8_t gps_parse_gprmc(char* sentence)
{
    char* token;
    uint8_t index = 0;

    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (index) {
        case 1:
            gps_convert_time(token, &gps_data.time);
            break;
        case 2:
            gps_data.fix_status = (token[0] == 'A') ? GPS_FIX_VALID : GPS_FIX_INVALID;
            break;
        case 3:
            gps_data.coordinate.latitude = gps_convert_coordinate(token, 'N');
            break;
        case 4:
            gps_data.coordinate.lat_direction = token[0];
            if (gps_data.coordinate.lat_direction == 'S') {
                gps_data.coordinate.latitude = -fabsf(gps_data.coordinate.latitude);
            }
            break;
        case 5:
            gps_data.coordinate.longitude = gps_convert_coordinate(token, 'E');
            break;
        case 6:
            gps_data.coordinate.lon_direction = token[0];
            if (gps_data.coordinate.lon_direction == 'W') {
                gps_data.coordinate.longitude = -fabsf(gps_data.coordinate.longitude);
            }
            break;
        case 7:
            gps_data.speed = (float)atof(token);
            break;
        case 8:
            gps_data.course = (float)atof(token);
            break;
        case 9:
            gps_convert_date(token, &gps_data.date);
            break;
        default:
            break;
        }
        token = strtok(NULL, ",");
        index++;
    }

    return 1U;
}

static uint8_t gps_parse_gpgga(char* sentence)
{
    char* token;
    uint8_t index = 0;

    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (index) {
        case 1:
            gps_convert_time(token, &gps_data.time);
            break;
        case 2:
            gps_data.coordinate.latitude = gps_convert_coordinate(token, 'N');
            break;
        case 3:
            gps_data.coordinate.lat_direction = token[0];
            if (gps_data.coordinate.lat_direction == 'S') {
                gps_data.coordinate.latitude = -fabsf(gps_data.coordinate.latitude);
            }
            break;
        case 4:
            gps_data.coordinate.longitude = gps_convert_coordinate(token, 'E');
            break;
        case 5:
            gps_data.coordinate.lon_direction = token[0];
            if (gps_data.coordinate.lon_direction == 'W') {
                gps_data.coordinate.longitude = -fabsf(gps_data.coordinate.longitude);
            }
            break;
        case 6:
            gps_data.fix_quality = (gps_quality_t)atoi(token);
            break;
        case 7:
            gps_data.satellites = (uint8_t)atoi(token);
            break;
        case 8:
            gps_data.hdop = (float)atof(token);
            break;
        case 9:
            gps_data.altitude = (float)atof(token);
            break;
        default:
            break;
        }
        token = strtok(NULL, ",");
        index++;
    }

    return 1U;
}

static uint8_t gps_parse_gpgsa(char* sentence)
{
    char* token;
    uint8_t index = 0;

    token = strtok(sentence, ",");
    while (token != NULL) {
        if (index == 2) {
            gps_data.fix_status = (token[0] == '3') ? GPS_FIX_VALID : GPS_FIX_INVALID;
            break;
        }
        token = strtok(NULL, ",");
        index++;
    }

    return 1U;
}

static uint8_t gps_parse_gpgsv(char* sentence)
{
    (void)sentence;
    return 1U;
}

static float gps_convert_coordinate(char* coord_str, char direction)
{
    if (!coord_str || strlen(coord_str) < 3U) {
        return 0.0f;
    }

    double raw = atof(coord_str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return (float)decimal;
}

static void gps_convert_time(char* time_str, gps_time_t* time)
{
    if (!time_str || !time) {
        return;
    }

    size_t len = strlen(time_str);
    if (len < 6U) {
        return;
    }

    time->hour = (uint8_t)((time_str[0] - '0') * 10 + (time_str[1] - '0'));
    time->minute = (uint8_t)((time_str[2] - '0') * 10 + (time_str[3] - '0'));
    time->second = (uint8_t)((time_str[4] - '0') * 10 + (time_str[5] - '0'));
    time->millisecond = 0;
    if (len > 7U && time_str[6] == '.') {
        time->millisecond = (uint16_t)(100.0f * (time_str[7] - '0'));
    }
}

static void gps_convert_date(char* date_str, gps_date_t* date)
{
    if (!date_str || !date) {
        return;
    }
    
    /* 检查日期字符串是否有效（至少6个数字字符） */
    size_t len = strlen(date_str);
    if (len < 6U) {
        return;
    }
    
    /* 检查所有字符是否为数字 */
    for (size_t i = 0; i < 6U; i++) {
        if (date_str[i] < '0' || date_str[i] > '9') {
            return;  /* 包含非数字字符，忽略此日期 */
        }
    }

    date->day = (uint8_t)((date_str[0] - '0') * 10 + (date_str[1] - '0'));
    date->month = (uint8_t)((date_str[2] - '0') * 10 + (date_str[3] - '0'));
    date->year = (uint16_t)(2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0'));
}

static uint8_t gps_calculate_checksum(char* sentence)
{
    uint8_t checksum = 0U;
    if (!sentence) {
        return checksum;
    }

    for (size_t i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= (uint8_t)sentence[i];
    }
    return checksum;
}

static uint8_t gps_verify_checksum(char* sentence)
{
    if (!sentence) {
        return 0U;
    }

    char* asterisk = strchr(sentence, '*');
    if (!asterisk || strlen(asterisk) < 3U) {
        return 0U;
    }

    uint8_t received_checksum = (uint8_t)strtol(asterisk + 1, NULL, 16);
    uint8_t calculated_checksum = gps_calculate_checksum(sentence);

    return (received_checksum == calculated_checksum) ? 1U : 0U;
}

