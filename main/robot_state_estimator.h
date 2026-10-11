#pragma once

#include <stdbool.h>

#include "robot_kinematics.h"

// Body position and velocity estimator after MIT Cheetah Software's
// LinearKFPositionVelocityEstimator: a linear Kalman filter over
// [p_body, v_body, p_foot x4] in a world frame with Y up and X along the
// heading at reset. The IMU acceleration drives the prediction; every foot
// measured by leg kinematics (p_body - p_foot = -R * p_foot_body) corrects
// it, weighted by how much that foot is trusted to be planted. A planted
// foot barely moves in the model, a swinging foot is free. While the robot
// is provably still, a zero-velocity measurement is added. Heading comes
// from the gyro through a 2-state Kalman filter that learns the gyro bias
// whenever the robot is still. Units: mm, s, rad. Axes as Aura's IK frame:
// X forward, Y up, Z left; heading > 0 turns left.

#define ROBOT_ESTIMATOR_STATES 18
#define ROBOT_ESTIMATOR_MAX_ROWS 19   // 4 x (p - foot) + 4 foot heights + 3 still

typedef struct {
    float dt;
    bool armed;
    bool walking;                     // feet follow a gait schedule
    bool commanded;                   // operator asks for motion
    robot_vec3_t feet_body_mm[ROBOT_LEG_COUNT];   // FK of measured joints, body origin
    bool feet_valid[ROBOT_LEG_COUNT];
    float trust[ROBOT_LEG_COUNT];     // 0 = swinging, 1 = planted mid-stance
    float roll_rad, pitch_rad;        // torso tilt, IK convention
    float yaw_rate_rad_s;             // gyro about the torso's up axis
    robot_vec3_t accel_mm_s2;         // specific force in the body frame
    float accel_norm_g;               // |accelerometer| in g, for the still test
    bool use_accel;
} robot_estimator_input_t;

typedef struct {
    bool valid;
    bool still;
    robot_vec3_t position_mm;         // world, torso
    float ground_mm;                  // height of the ground under Aura: its lowest foot
    robot_vec3_t velocity_mm_s;       // world
    float heading_rad;
    float heading_sigma_rad;
    float gyro_bias_rad_s;
    float forward_mm_s, left_mm_s;    // velocity in the heading frame
} robot_estimate_t;

typedef struct {
    float x[ROBOT_ESTIMATOR_STATES];
    float P[ROBOT_ESTIMATOR_STATES * ROBOT_ESTIMATOR_STATES];
    bool feet_initialized;
    float heading, gyro_bias;
    float h00, h01, h11;              // heading filter covariance
    float still_time;
    bool still;
    bool valid;
} robot_estimator_t;

void robot_estimator_reset(robot_estimator_t *estimator);
// Zero position and heading, keep the learned gyro bias.
void robot_estimator_rezero(robot_estimator_t *estimator);
void robot_estimator_step(robot_estimator_t *estimator, const robot_estimator_input_t *input);
void robot_estimator_get(const robot_estimator_t *estimator, robot_estimate_t *out);
