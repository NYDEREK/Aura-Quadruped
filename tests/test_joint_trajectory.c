#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "joint_trajectory.h"

int main(void)
{
    const joint_trajectory_limits_t limits = {
        .maximum_velocity = 2.0f,
        .maximum_acceleration = 4.0f,
    };
    const float dt = 0.02f;
    joint_trajectory_t trajectory = {0};
    joint_trajectory_reset(&trajectory, 0.0f);

    float previous_position = trajectory.position;
    float previous_velocity = trajectory.velocity;
    for (int index = 0; index < 200; ++index) {
        const float position = joint_trajectory_step(&trajectory, 1.0f, dt, limits);
        // No update may exceed either physical software bound.
        assert(fabsf(trajectory.velocity) <= limits.maximum_velocity + 1e-5f);
        assert(fabsf(trajectory.velocity - previous_velocity) <=
               limits.maximum_acceleration * dt + 1e-4f);
        // A fixed reference must progress monotonically and not oscillate.
        assert(position + 1e-5f >= previous_position);
        assert(position <= 1.0f + 1e-5f);
        previous_position = position;
        previous_velocity = trajectory.velocity;
    }
    assert(fabsf(trajectory.position - 1.0f) < 1e-4f);
    assert(fabsf(trajectory.velocity) < 1e-4f);

    // A new reference on the other side retains the acceleration bound.
    previous_velocity = trajectory.velocity;
    for (int index = 0; index < 200; ++index) {
        (void)joint_trajectory_step(&trajectory, -0.5f, dt, limits);
        assert(fabsf(trajectory.velocity - previous_velocity) <=
               limits.maximum_acceleration * dt + 1e-4f);
        previous_velocity = trajectory.velocity;
    }
    assert(fabsf(trajectory.position + 0.5f) < 1e-4f);
    puts("joint trajectory tests passed");
    return 0;
}
