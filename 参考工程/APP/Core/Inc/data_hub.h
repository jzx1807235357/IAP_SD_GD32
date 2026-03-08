/*!
    \file    data_hub.h
    \brief   Central data aggregation for host communication
*/

#ifndef DATA_HUB_H
#define DATA_HUB_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "light_sensor.h"
#include <stdbool.h>
#include <stdint.h>

#define DATA_HUB_EVENT_SENSOR   (1U << 0)
#define DATA_HUB_EVENT_SUN      (1U << 1)
#define DATA_HUB_EVENT_GIMBAL   (1U << 2)
#define DATA_HUB_EVENT_GPS      (1U << 3)

typedef struct {
    sensor_data_t sensor;
    float sun_azimuth;           /* 太阳方位角（北=0°，顺时针为正） */
    float sun_elevation;         /* 太阳仰角 */
    bool sun_valid;
    bool sun_visible;            /* 太阳是否可见（仰角 >= 5°） */
    float gimbal_azimuth;
    float gimbal_elevation;
    bool gimbal_valid;
    uint16_t gps_status;
} data_hub_snapshot_t;

void data_hub_init(EventGroupHandle_t event_group);
void data_hub_update_sensor(const sensor_data_t* sensor);
void data_hub_update_sun(float azimuth, float elevation, bool valid, bool visible);
void data_hub_update_gimbal(float azimuth, float elevation, bool valid);
void data_hub_update_gps_status(uint16_t status);
void data_hub_get_snapshot(data_hub_snapshot_t* snapshot);

#endif /* DATA_HUB_H */
