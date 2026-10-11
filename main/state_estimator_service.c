#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "state_estimator_service.h"
#include "aura_health.h"

#define ESTIMATOR_PERIOD_MS 20
#define DEG_TO_RAD 0.01745329252f
// IMU -> IK frame (X forward, Y up, Z left), established on Aura 2026-10-11:
// roll runs opposite and pitch the same way (the posture loop balanced
// pitch and pushed roll the wrong way), and with those only accelerometer x
// opposite, z unchanged keeps gravity vertical on tilted static samples.
#define IMU_ROLL_SIGN (-1.0f)
#define IMU_PITCH_SIGN (1.0f)
#define IMU_ACCEL_X_SIGN (-1.0f)
#define IMU_ACCEL_Z_SIGN (1.0f)

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static robot_estimator_t estimator;
static robot_estimate_t latest;
static estimator_legs_t legs;
static float accel_sum[3], gyro_sum[3];
static unsigned imu_count;
static bool rezero_requested;
// This board's accelerometer reads about 1.17 g at rest. Taken as is, the
// excess is a constant upward acceleration, and the still detector (1 g +/-
// 0.04) never fires, so the gyro bias is never learned. The magnitude of
// gravity is learned while Aura is quiet and divided out.
static float gravity_g;
#define GRAVITY_LEARN_RATE 0.02f      // per 20 ms step: ~1 s time constant
static TaskHandle_t task;

void estimator_service_add_imu(const imu_sample_t *sample)
{
    if (!sample) return;
    portENTER_CRITICAL(&lock);
    for (int i = 0; i < 3; ++i) {
        accel_sum[i] += sample->accel_g[i];
        gyro_sum[i] += sample->gyro_dps[i];
    }
    ++imu_count;
    portEXIT_CRITICAL(&lock);
}

void estimator_service_set_legs(const estimator_legs_t *next)
{
    if (!next) return;
    portENTER_CRITICAL(&lock);
    legs = *next;
    portEXIT_CRITICAL(&lock);
}

void estimator_service_get(robot_estimate_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&lock);
    *out = latest;
    portEXIT_CRITICAL(&lock);
}

void estimator_service_rezero(void)
{
    portENTER_CRITICAL(&lock);
    rezero_requested = true;
    portEXIT_CRITICAL(&lock);
}

static void estimator_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        aura_health_beat(AURA_TASK_ESTIMATOR);
        float accel[3] = {0}, gyro[3] = {0};
        unsigned count;
        estimator_legs_t snapshot;
        bool rezero;
        portENTER_CRITICAL(&lock);
        count = imu_count;
        for (int i = 0; i < 3; ++i) {
            accel[i] = count ? accel_sum[i] / (float)count : 0.0f;
            gyro[i] = count ? gyro_sum[i] / (float)count : 0.0f;
            accel_sum[i] = gyro_sum[i] = 0.0f;
        }
        imu_count = 0;
        snapshot = legs;
        rezero = rezero_requested;
        rezero_requested = false;
        portEXIT_CRITICAL(&lock);

        if (rezero) robot_estimator_rezero(&estimator);
        mpu6050_attitude_t attitude = {0};
        mpu6050_get_attitude(&attitude);
        if (count) {
            const float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
            const bool quiet = !snapshot.walking && !snapshot.commanded &&
                               fabsf(gyro[0]) < 3.0f && fabsf(gyro[1]) < 3.0f && fabsf(gyro[2]) < 3.0f;
            if (norm > 0.7f && norm < 1.4f) {
                if (gravity_g == 0) gravity_g = norm;
                else if (quiet) gravity_g += GRAVITY_LEARN_RATE * (norm - gravity_g);
            }
            if (gravity_g > 0) {
                for (int i = 0; i < 3; ++i) accel[i] /= gravity_g;
            }
        }
        if (count && attitude.valid) {
            robot_estimator_input_t input = {
                .dt = ESTIMATOR_PERIOD_MS / 1000.0f,
                .armed = snapshot.armed,
                .walking = snapshot.walking,
                .commanded = snapshot.commanded,
                .roll_rad = IMU_ROLL_SIGN * attitude.roll_radians,
                .pitch_rad = IMU_PITCH_SIGN * attitude.pitch_radians,
                .yaw_rate_rad_s = gyro[1] * DEG_TO_RAD,
                .accel_mm_s2 = {IMU_ACCEL_X_SIGN * accel[0] * 9810.0f, accel[1] * 9810.0f,
                                IMU_ACCEL_Z_SIGN * accel[2] * 9810.0f},
                .accel_norm_g = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]),
                .use_accel = true,
            };
            memcpy(input.feet_body_mm, snapshot.feet_body_mm, sizeof(input.feet_body_mm));
            memcpy(input.feet_valid, snapshot.feet_valid, sizeof(input.feet_valid));
            memcpy(input.trust, snapshot.trust, sizeof(input.trust));
            robot_estimator_step(&estimator, &input);
        }
        robot_estimate_t estimate;
        robot_estimator_get(&estimator, &estimate);
        portENTER_CRITICAL(&lock);
        latest = estimate;
        portEXIT_CRITICAL(&lock);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ESTIMATOR_PERIOD_MS));
    }
}

esp_err_t estimator_service_start(void)
{
    if (task) return ESP_OK;
    robot_estimator_reset(&estimator);
    return xTaskCreate(estimator_task, "state_estimator", 3072, NULL, 2, &task) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}
