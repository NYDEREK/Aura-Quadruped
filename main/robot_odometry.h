#pragma once

#include <stdbool.h>

#include "robot_kinematics.h"

typedef struct {
    bool valid;
    float x_mm;
    float z_mm;
    float yaw_radians;
    robot_vec3_t planted_feet[ROBOT_LEG_COUNT];
    bool has_planted_foot[ROBOT_LEG_COUNT];
} robot_odometry_t;

// Estimate the body pose from foot positions in the body frame and planned
// contact states. A stance foot is bound to its first observed world point;
// the rigid transform that best preserves all bound stance feet is the pose.
void robot_odometry_reset(robot_odometry_t *odometry);
void robot_odometry_update(robot_odometry_t *odometry,
                           const robot_vec3_t feet_body_mm[ROBOT_LEG_COUNT],
                           const bool contact[ROBOT_LEG_COUNT],
                           bool solve_yaw);
