#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "icm42688.h"
#include "power.h"

typedef struct {
    esp_err_t imu_error;
    uint8_t imu_identity;
    imu_sample_t imu;
    power_sample_t power;
} app_state_snapshot_t;

esp_err_t app_state_init(void);
void app_state_set_imu_status(esp_err_t error, uint8_t identity);
void app_state_set_imu_sample(esp_err_t error, const imu_sample_t *sample);
void app_state_set_power(const power_sample_t *sample);
void app_state_get(app_state_snapshot_t *snapshot);
