#pragma once

#include <stdbool.h>

// A planar point in Aura's horizontal body frame: +x is forward, +z is left.
typedef struct {
    float x;
    float z;
} robot_planar_point_t;

// A quasistatic support plan for exactly three planted feet. `com_target` is
// the desired physical translation of the torso/CoM in the horizontal plane.
// It is deliberately separate from a leg target: to move the torso by this
// vector while a foot is planted, the leg IK target must be shifted by its
// negative.
typedef struct {
    bool valid;
    robot_planar_point_t com_target;
    // Signed minimum distance from the target to a support-triangle edge.
    // Positive means inside; this value is in millimetres.
    float margin_mm;
    // Inradius of the support triangle, the largest possible equal edge
    // margin. It lets the caller reject an impossible requested margin.
    float maximum_margin_mm;
} robot_static_support_plan_t;

// Signed support margin for a three-foot polygon. It is positive inside,
// zero on an edge, negative outside, and independent of winding direction.
float robot_static_support_margin(const robot_planar_point_t support[3],
                                  robot_planar_point_t point);

// Finds the nearest feasible CoM target to `current_com` inside the triangle
// reduced by `minimum_margin_mm` on every edge. The result preserves the
// smallest necessary body shift instead of jumping to a triangle centroid.
// It returns false for degenerate triangles or an impossible margin.
bool robot_static_support_plan(const robot_planar_point_t support[3],
                               robot_planar_point_t current_com,
                               float minimum_margin_mm,
                               robot_static_support_plan_t *out);

// Projects the CoM onto the infinite line passing through the two planted
// feet.  Unlike the three-foot routine this is not a static-stability plan:
// a line has no area and therefore cannot resist a disturbance without an
// IMU-controlled dynamic gait.  It is used only for the bounded feed-forward
// correction in trot/run/climb, and preserves the component along the line so
// it cannot cancel the intended forward gait stroke.
bool robot_support_line_projection(const robot_planar_point_t support[2],
                                   robot_planar_point_t current_com,
                                   robot_planar_point_t *out_projection,
                                   float *out_lateral_error_mm);
