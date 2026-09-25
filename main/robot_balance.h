#pragma once

#include "robot_kinematics.h"

// Numerical scene frame X forward, Y up, Z left. These matrices use the
// renderer's rotation convention. MPU gravity angles have the opposite sign.
typedef struct {
    robot_vec3_t position;
    float roll;
    float pitch;
} robot_balance_pose_t;

robot_vec3_t robot_balance_to_world(robot_vec3_t body, robot_balance_pose_t pose);
robot_vec3_t robot_balance_to_body(robot_vec3_t world, robot_balance_pose_t pose);

// Least-squares translation with measured orientation, from scheduled,
// non-slipping support feet. No airborne foot may enter this estimate.
bool robot_balance_estimate_pose(const robot_vec3_t feet_body[4],
                                const robot_vec3_t feet_world[4],
                                const bool support[4], float roll, float pitch,
                                robot_balance_pose_t *pose);

// Swing is expressed in the measured body frame. The actuator layer owns
// acceleration limiting; a desired-pose transform cannot preserve clearance.
robot_vec3_t robot_balance_swing_target(robot_vec3_t world, robot_balance_pose_t measured);

// Position-servo adaptation of angular-error + angular-rate feedback.
// Returned angle uses the existing Aura gravity/servo convention.
float robot_balance_posture_target(float error, float angular_rate,
                                   float kp, float kd_seconds, float limit);
