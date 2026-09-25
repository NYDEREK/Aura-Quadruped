#pragma once

#include <stdbool.h>
#include <stdint.h>

// Lightweight roll/pitch observer for a legged platform. Accelerometer data
// supplies the long-term gravity reference; gyro integration preserves fast
// motion. Inputs use Aura's body frame: X forward, Y up, Z left.
typedef struct {
    bool initialized;
    float roll_radians;
    float pitch_radians;
    int64_t last_sample_time_us;
} attitude_filter_t;

void attitude_filter_reset(attitude_filter_t *filter);
bool attitude_filter_update(attitude_filter_t *filter, const float accel_g[3],
                            const float gyro_dps[3], int64_t sample_time_us,
                            float *roll_radians, float *pitch_radians);
