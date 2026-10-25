#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "robot_body_trajectory.h"
#include "robot_kinematics.h"

// Can the servos follow a gait profile? One cycle is simulated with the
// firmware's own foot path, body plan and inverse kinematics: forward at
// full stride, and forward while turning. Joint speed grows with the step
// frequency and acceleration with its square, so `rate` is the frequency
// scale (0..1] that brings both peaks within the given limits.
// Pure C: shared by the firmware (mode 4 tempo) and the desktop app.

typedef struct {
    bool valid;
    float peak_speed_rad_s;          // at the profile's own frequency
    float peak_acceleration_rad_s2;
    float two_foot_s;                // longest time on two feet, at that frequency
    float rate;                      // frequency scale within the limits, <= 1
} robot_gait_feasibility_t;

// `scratch` holds the body plan during the check (its cache is reset).
bool robot_gait_feasibility(uint8_t gait, const robot_locomotion_profile_t *profile,
                            const robot_geometry_t *geometry, float body_height_mm,
                            float lateral_stance_mm, float max_speed_rad_s,
                            float max_acceleration_rad_s2, robot_body_trajectory_t *scratch,
                            robot_gait_feasibility_t *out);
