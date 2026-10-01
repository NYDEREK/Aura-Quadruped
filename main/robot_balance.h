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

// PI posture loop. The IMU sits on the torso, so the measured tilt already
// contains the correction this loop commands (measured = floor + c). A pure
// P law c = kp*e therefore settles at c = -kp/(1+kp) * floor: with kp = 1
// only HALF of a floor tilt was ever removed. The integral term drives the
// residual error to zero. `integral` is the controller state (radians).
float robot_balance_posture_pi(float *integral, float error, float rate,
                               float kp, float ki, float kd, float limit, float dt);
