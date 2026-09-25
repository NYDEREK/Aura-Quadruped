#pragma once

#include <stdbool.h>

typedef struct {
    float stroke_mm;
    float lift_mm;
    bool stance;
} robot_foot_path_t;

// A closed, C² foot trajectory. `phase` is cyclic in [0, 1); during stance
// the foot sweeps backward under the body, then returns forward above ground.
// Swing is a single continuous minimum-jerk arch, without a flat top or
// vertical/horizontal corner in Cartesian space.
robot_foot_path_t robot_gait_foot_path(float phase, float duty_factor,
                                       float stride_mm, float step_height_mm,
                                       bool active);
