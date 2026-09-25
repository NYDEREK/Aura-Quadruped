#pragma once

#include <stdbool.h>

#include "robot_model.h"

typedef struct { float x, y, z; } robot_vec3_t;

typedef struct {
    float frame_length_mm;
    float frame_width_mm;
    float corner_inset_mm;
    float abduction_link_mm;
    float upper_leg_mm;
    float lower_leg_mm;
} robot_geometry_t;

robot_geometry_t robot_kinematics_default_geometry(void);
robot_vec3_t robot_kinematics_leg_anchor(const robot_geometry_t *geometry, robot_leg_t leg,
                                          float body_height_mm);
robot_vec3_t robot_kinematics_neutral_foot(const robot_geometry_t *geometry, robot_leg_t leg);
bool robot_kinematics_inverse(const robot_geometry_t *geometry, robot_leg_t leg,
                              robot_vec3_t target_body_mm, float body_height_mm,
                              float out_radians[ROBOT_AXIS_COUNT]);
robot_vec3_t robot_kinematics_forward(const robot_geometry_t *geometry, robot_leg_t leg,
                                      const float radians[ROBOT_AXIS_COUNT], float body_height_mm);
