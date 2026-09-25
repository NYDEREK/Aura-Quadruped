#pragma once

#include <stdbool.h>
#include <stdint.h>

// A conservative, proprioceptive foot-contact observer. It deliberately
// exposes confidence before it is allowed to steer a gait: ST3215 load is a
// motor-drive estimate, rather than a calibrated foot force sensor.
typedef struct {
    bool armed;
    bool expected_stance;
    bool touchdown_window;
    bool feedback_valid;
    int16_t hip_load_raw;
    int16_t knee_load_raw;
    int16_t hip_current_raw;
    int16_t knee_current_raw;
    float hip_tracking_error_radians;
    float knee_tracking_error_radians;
    // |norm(acceleration) - 1 g| from the MPU6050. A body-wide impact is
    // only supporting evidence; it can never identify a leg by itself.
    float imu_specific_force_error_g;
} robot_contact_input_t;

typedef struct {
    bool contact;
    bool feedback_valid;
    bool early_touchdown;
    uint8_t confidence_percent;
    uint8_t contact_samples;
    uint8_t release_samples;
} robot_contact_estimate_t;

void robot_contact_reset(robot_contact_estimate_t *estimate);
void robot_contact_update(robot_contact_estimate_t *estimate,
                          const robot_contact_input_t *input);
