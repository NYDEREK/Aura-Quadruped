#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    esp_err_t ina_error;
    esp_err_t battery_error;
    uint16_t manufacturer_id, die_id;
    float bus_v, shunt_mv, servo_current_a, servo_input_power_w;
    float servo_consumed_mAh;
    float battery_v;
    int battery_adc_raw;
    bool battery_calibrated;
} power_sample_t;
void power_init(void);
void power_read(power_sample_t *sample);
esp_err_t power_read_vin(float *voltage_v, int *raw_average);
