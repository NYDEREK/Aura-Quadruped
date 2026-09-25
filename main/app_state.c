#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app_state.h"

static SemaphoreHandle_t mutex;
static app_state_snapshot_t state = {.imu_error = ESP_ERR_INVALID_STATE};

esp_err_t app_state_init(void)
{
    mutex = xSemaphoreCreateMutex();
    return mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

void app_state_set_imu_status(esp_err_t error, uint8_t identity)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    state.imu_error = error;
    state.imu_identity = identity;
    xSemaphoreGive(mutex);
}

void app_state_set_imu_sample(esp_err_t error, const imu_sample_t *sample)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (error == ESP_OK && sample) {
        state.imu = *sample;
        // A transient I2C arbitration timeout must not leave the desktop
        // permanently showing an unavailable MPU after the next good sample.
        state.imu_error = ESP_OK;
    }
    else if (error != ESP_ERR_NOT_FINISHED) state.imu_error = error;
    xSemaphoreGive(mutex);
}

void app_state_set_power(const power_sample_t *sample)
{
    if (!sample) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    state.power = *sample;
    xSemaphoreGive(mutex);
}

void app_state_get(app_state_snapshot_t *snapshot)
{
    if (!snapshot) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    *snapshot = state;
    xSemaphoreGive(mutex);
}
