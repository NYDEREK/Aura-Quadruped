#pragma once

#include <stdbool.h>

// A small, deterministic second-order setpoint generator.  It sits between
// inverse kinematics and the ST3215 bus: IK supplies a reference angle, while
// this block emits the next angle with bounded velocity and acceleration.
// Units are radians, radians/s and radians/s².
typedef struct {
    float position;
    float velocity;
    bool initialized;
    float previous_reference;
    bool tracking_reference;
} joint_trajectory_t;

typedef struct {
    float maximum_velocity;
    float maximum_acceleration;
} joint_trajectory_limits_t;

void joint_trajectory_reset(joint_trajectory_t *trajectory, float position);
float joint_trajectory_step(joint_trajectory_t *trajectory, float reference,
                            float dt, joint_trajectory_limits_t limits);

// Continuous timed-reference tracking: reference velocity feed-forward plus
// bounded position-error correction. Keeps velocity through successive gait
// points instead of braking to rest at every 20 ms sample.
float joint_trajectory_track(joint_trajectory_t *trajectory, float reference,
                             float dt, joint_trajectory_limits_t limits);
