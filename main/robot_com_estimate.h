#pragma once

#include <stdbool.h>

#include "robot_kinematics.h"
#include "robot_static_balance.h"

// Centre of mass from the static load distribution of four planted feet.
// In a still stance the vertical foot forces satisfy sum(F_i * p_i) =
// m*g * CoM (moment balance about the ground). Each ST3215 reports its motor
// load; with a symmetric stance every knee has the same lever arm to its
// foot, so |knee load_i| is proportional to F_i up to one common factor,
// which cancels in the weighted centroid. Feet are the forward kinematics
// of the MEASURED joints, in the torso frame (X forward, Z left), so the
// result is directly the torso-frame CoM offset Aura stores (com_x/com_z).
bool robot_com_estimate(const robot_vec3_t feet_body[4], const float weight[4],
                        robot_planar_point_t *out_com);
