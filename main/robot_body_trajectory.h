#pragma once

#include "robot_locomotion.h"
#include "robot_predictive_support.h"

#define ROBOT_BODY_TRAJECTORY_SAMPLES 64

// Constant planar body twist in the moving gait frame: X forward, Z left.
// A positive yaw uses J(x,z)=(-z,x); all feet share this angular velocity.
typedef struct { float x_mm_s, z_mm_s, yaw_rad_s; } robot_body_twist_t;

typedef struct {
    uint8_t gait;
    robot_locomotion_profile_t profile;
    robot_geometry_t geometry;
    float com_height_mm;
    float lateral_stance_mm;
    float forward;
    float lateral;
    float turn;
    float stride_mm;
    bool stepping_in_place;
    bool spin;
    // Optional path time-scaling. Zero limits leave the geometric model
    // unconstrained for analytic tests. No persisted profile is overwritten.
    float body_height_mm, com_offset_x_mm, com_offset_z_mm;
    float joint_velocity_limit, joint_acceleration_limit;
    robot_leg_t excluded_leg; // only gait 5; other profiles ignore it
} robot_body_trajectory_request_t;

typedef struct {
    robot_planar_point_t position_mm;
    robot_planar_point_t velocity_mm_s;
    robot_planar_point_t acceleration_mm_s2;
    robot_planar_point_t support_mm;
} robot_body_trajectory_sample_t;

typedef struct {
    bool valid;
    bool request_cached;
    robot_body_trajectory_request_t request;
    float omega;
    float dt;
    robot_body_twist_t twist;
    float effective_frequency_hz;
    bool joint_path_feasible;
    robot_planar_point_t support[ROBOT_BODY_TRAJECTORY_SAMPLES];
    robot_vec3_t feet[ROBOT_BODY_TRAJECTORY_SAMPLES][ROBOT_LEG_COUNT];
    robot_planar_point_t divergent[ROBOT_BODY_TRAJECTORY_SAMPLES];
    robot_planar_point_t convergent[ROBOT_BODY_TRAJECTORY_SAMPLES];
} robot_body_trajectory_t;

robot_body_twist_t robot_body_trajectory_twist(const robot_body_trajectory_request_t *request);
// exp(t * twist) acting on a point. Inverse transform is obtained with -t.
robot_vec3_t robot_body_twist_transform(robot_vec3_t point, robot_body_twist_t twist,
                                       float seconds);

// The same scheduled Cartesian foot path is used by the support prediction,
// firmware IK, and the desktop preview. Positions: X forward, Y up, Z left.
robot_vec3_t robot_body_trajectory_foot(const robot_body_trajectory_request_t *request,
                                       robot_leg_t leg, float global_phase);

// Project a reference into the convex hull of SCHEDULED contacts. Two feet
// define a segment, not an area; airborne feet cannot provide a reaction.
bool robot_body_support_reference(const robot_planar_point_t feet[4],
                                   const bool contact[4], robot_planar_point_t reference,
                                   robot_planar_point_t *out);

// Exact periodic solution of c'' = g/h * (c - p) for piecewise-linear p.
// Kajita et al. ICRA 2003, Eqs. (6)-(9). The periodic boundary condition is
// solved in both directions, so this is preview, not a lagging low-pass.
// This is a reduced constant-height reference model, NOT force MPC.
bool robot_body_trajectory_solve(robot_body_trajectory_t *plan,
                                 const robot_planar_point_t support[ROBOT_BODY_TRAJECTORY_SAMPLES],
                                 float com_height_mm, float cycle_seconds);
bool robot_body_trajectory_build(robot_body_trajectory_t *plan,
                                 const robot_body_trajectory_request_t *request);
// True when `plan` was built for exactly this request (no rebuild needed).
bool robot_body_trajectory_matches(const robot_body_trajectory_t *plan,
                                   const robot_body_trajectory_request_t *request);
robot_body_trajectory_sample_t robot_body_trajectory_sample(const robot_body_trajectory_t *plan,
                                                            float global_phase);
