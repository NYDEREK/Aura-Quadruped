#pragma once

#include <stdbool.h>

#include "robot_kinematics.h"
#include "robot_predictive_support.h"

// Balance while walking (pad mode 3). See docs/balans-chodu.md.
//
// On two diagonal feet the support is a line. A position-controlled leg
// cannot push about that line, so the torso pose cannot keep the centre of
// mass (CoM) from falling sideways: c'' = g/h * c (LIPM, Kajita 2003). What
// can is where the next pair lands. MIT Cheetah 3 (Bledt et al. 2018, eq. 6)
// adds sqrt(h/g) * (v - v_des) to the touchdown point; with the drift of
// the hip this is the capture point of Pratt et al. (2006). Here the drift
// and the velocity come from the tilt of the rigid robot about its feet.
//
// Tilt convention ("scene", the one the IK body pose uses): roll > 0 lowers
// the left side, pitch > 0 raises the nose. Aura's posture loop is stable
// only if the MPU6050 angles have this sign, so the measured angles minus
// the captured reference are used as they are.
// Ground frame: X forward, Z left, millimetres.

typedef struct {
    float roll;   // rad
    float pitch;  // rad
} robot_tilt_t;

// Horizontal CoM displacement (mm) of a robot rotated by `tilt` about its
// ground contacts, and the CoM velocity (mm/s) for a tilt rate (rad/s).
robot_planar_point_t robot_walk_tilt_displacement(robot_tilt_t tilt, float com_height_mm);
robot_planar_point_t robot_walk_tilt_velocity(robot_tilt_t rate, float com_height_mm);

typedef struct {
    float gain;           // 1 = capture point
    float limit_mm;       // bound of the offset per axis
    float deadband_mm;    // ignores sensor noise around zero
    float filter_time_s;  // low-pass of the velocity estimate
    float freeze_swing;   // swing progress after which the target is held
} robot_walk_balance_config_t;

typedef struct {
    robot_planar_point_t landed[ROBOT_LEG_COUNT];   // held for the whole stance
    robot_planar_point_t liftoff[ROBOT_LEG_COUNT];
    robot_planar_point_t current[ROBOT_LEG_COUNT];
    bool swinging[ROBOT_LEG_COUNT];
    robot_planar_point_t filtered_velocity;
    robot_planar_point_t capture;                   // last capture offset
} robot_walk_balance_t;

robot_walk_balance_config_t robot_walk_balance_default_config(void);
void robot_walk_balance_reset(robot_walk_balance_t *balance);

// Capture offset relative to the planted support: the divergent component
// xi = displacement + sqrt(h/g) * filtered velocity (Cheetah 3 eq. 6 with the
// hip drift), bounded per axis. The caller lands the next foot at the mean
// offset of the planted feet plus this value. No extrapolation: predicting
// the growth on the support line amplified the unknown CoM offset in the
// host simulation (tests/test_robot_walk_balance.c).
robot_planar_point_t robot_walk_balance_capture(robot_walk_balance_t *balance,
                                                const robot_walk_balance_config_t *config,
                                                robot_planar_point_t displacement_mm,
                                                robot_planar_point_t velocity_mm_s,
                                                float com_height_mm, float dt);

// Moves one planned foot (ground frame). A planted foot keeps the offset it
// landed with, so it never slides. A swinging foot blends from its liftoff
// offset to the live capture offset and holds it after `freeze_swing`, so
// the last part of the swing is not chased by sensor noise.
robot_vec3_t robot_walk_balance_apply(robot_walk_balance_t *balance,
                                      const robot_walk_balance_config_t *config,
                                      robot_leg_t leg, robot_vec3_t planned,
                                      bool swing, float swing_phase,
                                      robot_planar_point_t capture);
