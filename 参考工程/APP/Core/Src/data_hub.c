/*!
    \file    data_hub.c
    \brief   Central data aggregation for host communication
*/

#include "data_hub.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static data_hub_snapshot_t s_snapshot;
static SemaphoreHandle_t s_mutex = NULL;
static EventGroupHandle_t s_event_group = NULL;

void data_hub_init(EventGroupHandle_t event_group)
{
    s_event_group = event_group;

    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        configASSERT(s_mutex != NULL);
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

void data_hub_update_sensor(const sensor_data_t* sensor)
{
    if (!sensor) {
        return;
    }

    if (s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_snapshot.sensor = *sensor;
        xSemaphoreGive(s_mutex);
    }

    if (s_event_group != NULL) {
        xEventGroupSetBits(s_event_group, DATA_HUB_EVENT_SENSOR);
    }
}

void data_hub_update_sun(float azimuth, float elevation, bool valid, bool visible)
{
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_snapshot.sun_azimuth = azimuth;
        s_snapshot.sun_elevation = elevation;
        s_snapshot.sun_valid = valid;
        s_snapshot.sun_visible = visible;
        xSemaphoreGive(s_mutex);
    }

    if (s_event_group != NULL) {
        xEventGroupSetBits(s_event_group, DATA_HUB_EVENT_SUN);
    }
}

void data_hub_update_gimbal(float azimuth, float elevation, bool valid)
{
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_snapshot.gimbal_azimuth = azimuth;
        s_snapshot.gimbal_elevation = elevation;
        s_snapshot.gimbal_valid = valid;
        xSemaphoreGive(s_mutex);
    }

    if (s_event_group != NULL) {
        xEventGroupSetBits(s_event_group, DATA_HUB_EVENT_GIMBAL);
    }
}

void data_hub_update_gps_status(uint16_t status)
{
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_snapshot.gps_status = status;
        xSemaphoreGive(s_mutex);
    }

    if (s_event_group != NULL) {
        xEventGroupSetBits(s_event_group, DATA_HUB_EVENT_GPS);
    }
}

void data_hub_get_snapshot(data_hub_snapshot_t* snapshot)
{
    if (!snapshot) {
        return;
    }

    if (s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        *snapshot = s_snapshot;
        xSemaphoreGive(s_mutex);
    }
}
