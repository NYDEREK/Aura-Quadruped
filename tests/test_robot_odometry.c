#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_odometry.h"

static robot_vec3_t inverse_world(float x, float z, float yaw, robot_vec3_t world)
{
    const float dx = world.x - x, dz = world.z - z;
    const float c = cosf(yaw), s = sinf(yaw);
    return (robot_vec3_t){.x = c * dx + s * dz, .y = world.y, .z = -s * dx + c * dz};
}

int main(void)
{
    const robot_vec3_t anchors[ROBOT_LEG_COUNT] = {
        {80, 0, 90}, {80, 0, -90}, {-80, 0, 90}, {-80, 0, -90},
    };
    const bool all[ROBOT_LEG_COUNT] = {true, true, true, true};
    robot_odometry_t odometry = {0};
    robot_odometry_reset(&odometry);
    robot_odometry_update(&odometry, anchors, all, true);
    assert(odometry.valid);

    robot_vec3_t moved[ROBOT_LEG_COUNT];
    for (int i = 0; i < ROBOT_LEG_COUNT; ++i)
        moved[i] = inverse_world(123.0f, -47.0f, 0.37f, anchors[i]);
    robot_odometry_update(&odometry, moved, all, true);
    assert(fabsf(odometry.x_mm - 123.0f) < 0.01f);
    assert(fabsf(odometry.z_mm + 47.0f) < 0.01f);
    assert(fabsf(odometry.yaw_radians - 0.37f) < 0.0001f);

    const bool diagonal[ROBOT_LEG_COUNT] = {true, false, false, true};
    robot_odometry_update(&odometry, moved, diagonal, false);
    assert(fabsf(odometry.x_mm - 123.0f) < 0.01f);
    assert(fabsf(odometry.z_mm + 47.0f) < 0.01f);
    puts("robot odometry: passed");
    return 0;
}
