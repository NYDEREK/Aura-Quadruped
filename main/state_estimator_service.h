#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "icm42688.h"
#include "robot_state_estimator.h"

// Runs robot_state_estimator at 50 Hz in its own low-priority task. The IMU
// owner task adds every 100 Hz sample (averaged per step, so gait vibration
// is not aliased), the gait task publishes leg kinematics once per tick.
// Nothing is stored: the estimate lives in RAM only.
typedef struct {
    bool armed;
    bool walking;
    bool commanded;
    robot_vec3_t feet_body_mm[ROBOT_LEG_COUNT];
    bool feet_valid[ROBOT_LEG_COUNT];
    float trust[ROBOT_LEG_COUNT];
} estimator_legs_t;

esp_err_t estimator_service_start(void);
void estimator_service_add_imu(const imu_sample_t *sample);
void estimator_service_set_legs(const estimator_legs_t *legs);
void estimator_service_get(robot_estimate_t *out);
// Position and heading back to zero; the learned gyro bias is kept.
void estimator_service_rezero(void);
