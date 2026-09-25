#pragma once

#include <stdbool.h>

#include "robot_kinematics.h"
#include "robot_static_balance.h"

// Swing-leg foot placement feedback, MIT Cheetah 3 (Bledt et al., IROS 2018),
// section II-E, equation (6):
//
//   p_step,i = p_h,i + (T_stance/2) * v_des + sqrt(z0/g) * (v - v_des)
//
// The first two terms (hip location and the Raibert heuristic) are already
// the feed-forward touchdown of robot_body_trajectory_foot(). This module adds
// the third, capture-point term: the measured CoM velocity error moves every
// swinging foot's touchdown so the robot steps under a fall instead of
// completing the planned step while it tips over.
//
// On Aura the only fast velocity observation is the MPU6050 gyro. With all
// support feet planted and position-controlled legs, an unplanned rotation
// of the body is the robot tipping about its support line, so the CoM (at
// height z0 above the ground) moves horizontally with v = omega x r.
//
// In (6) p_h,i is the hip in the world, i.e. it already contains any drift
// of the body from its plan. Aura's planned feet live in the planned gait
// frame, so that drift is added explicitly: the tipping displacement
// z0 * (unplanned tilt). Displacement + sqrt(z0/g) * velocity is exactly the
// capture point of Pratt et al., reference [19] of the Cheetah 3 paper.
//
// Coordinates: gait/world frame X forward, Z left (ground plane), mm.

typedef struct {
    // Offset latched at touchdown and held for the whole stance: a planted
    // foot never slides.  Swing blends from this value to the live offset.
    robot_planar_point_t landed[4];
    robot_planar_point_t liftoff[4];
    robot_planar_point_t current[4];
    bool swinging[4];
    robot_planar_point_t filtered_velocity_error;
} robot_foot_placement_t;

typedef struct {
    float gain;               // 1.0 = equation (6) exactly
    float limit_mm;           // per-axis bound of the capture offset
    float deadband_mm_s;      // ignores gyro noise while standing still
    float filter_time_s;      // first-order low-pass of the velocity error
} robot_foot_placement_config_t;

robot_foot_placement_config_t robot_foot_placement_default_config(void);
void robot_foot_placement_reset(robot_foot_placement_t *placement);

// Horizontal CoM velocity of a rigid body tipping about ground contacts.
// Rates use the MPU6050 attitude convention of this firmware:
// roll > 0 = left side up (about X), pitch > 0 = nose down (about Z).
// `commanded_*_rate` is the body rotation requested by the attitude loop;
// only the remainder is unplanned tipping.
robot_planar_point_t robot_foot_placement_tipping_velocity(float roll_rate_rad_s,
                                                            float pitch_rate_rad_s,
                                                            float commanded_roll_rate_rad_s,
                                                            float commanded_pitch_rate_rad_s,
                                                            float com_height_mm);

// Same geometry for an angle instead of a rate: horizontal CoM displacement
// caused by an unplanned tilt (measured - reference - commanded correction).
robot_planar_point_t robot_foot_placement_tipping_displacement(float roll_error_rad,
                                                                float pitch_error_rad,
                                                                float com_height_mm);

// Filters the velocity error and returns the capture-point offset
// gain * (displacement + sqrt(z0/g) * (v - v_des)), bounded per axis.
robot_planar_point_t robot_foot_placement_update(robot_foot_placement_t *placement,
                                                 const robot_foot_placement_config_t *config,
                                                 robot_planar_point_t displacement_mm,
                                                 robot_planar_point_t velocity_error_mm_s,
                                                 float com_height_mm, float dt);

// Applies the offset to one planned foot.  `swing_phase` in [0,1] is the
// same progress used by the Bézier swing, so the correction follows the
// foot and reaches the live value exactly at touchdown.
robot_vec3_t robot_foot_placement_apply(robot_foot_placement_t *placement, robot_leg_t leg,
                                        robot_vec3_t planned, bool swing, float swing_phase,
                                        robot_planar_point_t live_offset);
