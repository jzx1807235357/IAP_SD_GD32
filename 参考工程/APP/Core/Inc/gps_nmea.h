/*!
    \file    gps_nmea.h
    \brief   GPS NMEA0183 data parser interface (FreeRTOS adaptation)

    \version 2025-01-10, V2.0, FreeRTOS-ready GPS parser for GD32F4xx
*/

#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <stdint.h>

/* GPS UART configuration - PA2(TX), PA3(RX) -> USART1 */
#define GPS_USART                USART1
#define GPS_USART_CLK            RCU_USART1
#define GPS_USART_IRQn           USART1_IRQn

#define GPS_GPIO_PORT            GPIOA
#define GPS_GPIO_CLK             RCU_GPIOA
#define GPS_TX_PIN               GPIO_PIN_2
#define GPS_RX_PIN               GPIO_PIN_3
#define GPS_GPIO_AF              GPIO_AF_7

#define GPS_BUFFER_SIZE          256U
#define GPS_MAX_SENTENCE_LEN     82U
#define GPS_RX_QUEUE_LENGTH      256U

typedef enum {
    GPS_FIX_INVALID = 0,
    GPS_FIX_VALID   = 1
} gps_fix_status_t;

typedef enum {
    GPS_QUALITY_INVALID       = 0,
    GPS_QUALITY_GPS           = 1,
    GPS_QUALITY_DGPS          = 2,
    GPS_QUALITY_PPS           = 3,
    GPS_QUALITY_RTK           = 4,
    GPS_QUALITY_FLOAT_RTK     = 5,
    GPS_QUALITY_ESTIMATED     = 6,
    GPS_QUALITY_MANUAL        = 7,
    GPS_QUALITY_SIMULATION    = 8
} gps_quality_t;

typedef struct {
    float latitude;          /* Latitude in decimal degrees */
    float longitude;         /* Longitude in decimal degrees */
    char  lat_direction;     /* N or S */
    char  lon_direction;     /* E or W */
} gps_coordinate_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} gps_time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} gps_date_t;

typedef struct {
    gps_fix_status_t fix_status;
    gps_quality_t    fix_quality;
    gps_coordinate_t coordinate;
    gps_time_t       time;
    gps_date_t       date;
    float            altitude;
    float            speed;
    float            course;
    uint8_t          satellites;
    float            hdop;
    uint8_t          data_valid;
} gps_data_t;

bool gps_init(void);
bool gps_reinit(void);

/* Runtime helpers */
QueueHandle_t gps_get_rx_queue(void);
void gps_process_byte(uint8_t byte);
gps_data_t* gps_get_data(void);
uint32_t gps_get_data_age_ms(void);
bool gps_has_fix(void);
bool gps_is_communicating(void);
bool gps_communication_healthy(void);
bool gps_get_position(double* latitude, double* longitude, float* altitude);
bool gps_get_time(uint16_t* year, uint8_t* month, uint8_t* day,
                  uint8_t* hour, uint8_t* minute, uint8_t* second);

/* UART support */
void gps_uart_init(void);
void gps_uart_irq_handler(BaseType_t* pxHigherPriorityTaskWoken);

#endif /* GPS_NMEA_H */
