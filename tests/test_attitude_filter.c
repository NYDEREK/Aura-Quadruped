#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "attitude_filter.h"

#define PI 3.14159265358979323846f

static int close_enough(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    attitude_filter_t filter;
    attitude_filter_reset(&filter);
    const float stationary_gyro[3] = {0.0f, 0.0f, 0.0f};
    float roll = 0.0f, pitch = 0.0f;

    // Aura body frame: X forward, Y up, Z left. Level gravity establishes a
    // zero roll/pitch reference without a gyro-only transient.
    const float level[3] = {0.0f, 1.0f, 0.0f};
    assert(attitude_filter_update(&filter, level, stationary_gyro, 1000000, &roll, &pitch));
    assert(close_enough(roll, 0.0f, 0.0001f));
    assert(close_enough(pitch, 0.0f, 0.0001f));

    // A static 15 degree roll must converge from the accelerometer gravity
    // vector. Repeated 100 Hz samples model the MPU6050 owner task.
    const float desired_roll = 15.0f * PI / 180.0f;
    const float rolled[3] = {0.0f, cosf(desired_roll), sinf(desired_roll)};
    for (int sample = 1; sample <= 300; ++sample)
        assert(attitude_filter_update(&filter, rolled, stationary_gyro,
                                      1000000 + sample * 10000, &roll, &pitch));
    assert(close_enough(roll, desired_roll, 0.01f));
    assert(close_enough(pitch, 0.0f, 0.01f));

    // An acceleration magnitude outside the trust window must not reseed or
    // abruptly drag the gravity observer to an invented attitude.
    const float linear_acceleration[3] = {3.0f, 0.0f, 0.0f};
    assert(attitude_filter_update(&filter, linear_acceleration, stationary_gyro,
                                  4050000, &roll, &pitch));
    assert(close_enough(roll, desired_roll, 0.01f));

    printf("attitude filter: passed\n");
    return 0;
}
