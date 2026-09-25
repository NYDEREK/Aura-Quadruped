#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "icm42688.h"

typedef struct {
    bool valid;
    uint8_t address_7bit;
    uint32_t sample_count;
    int64_t sample_time_us;
    float roll_radians;
    float pitch_radians;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
} mpu6050_attitude_t;

// MPU6050 connected to S1: I2C GPIO25/26, DATA_RDY interrupt on GPIO14.
// It owns S1's former XSHUT wire as an input and never drives that pin.
esp_err_t mpu6050_init(uint8_t *who_am_i);
esp_err_t mpu6050_reinitialize(uint8_t *who_am_i);
esp_err_t mpu6050_read(imu_sample_t *sample);
void mpu6050_get_attitude(mpu6050_attitude_t *attitude);
