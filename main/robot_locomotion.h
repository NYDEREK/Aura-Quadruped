#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "robot_kinematics.h"

// Parameters are stored on Aura, in millimetres and centihertz. They never
// encode a servo command; the gait loop reads them before it builds a world
// foot plan and a single rigid torso pose.
typedef struct {
    uint16_t stride_mm;
    uint16_t step_height_mm;
    uint16_t frequency_centi_hz;
    // Fraction of the gait cycle that each scheduled leg remains in contact.
    // 100% is reserved for standing; walking profiles are constrained below.
    uint8_t duty_percent;
} robot_locomotion_profile_t;

typedef struct {
    bool scheduled_swing;
    bool support_contact;
    float local_phase;
    float swing_phase;
} robot_locomotion_leg_phase_t;

// The four moving profiles use the robot_gait_mode_t numeric values:
// Trot=1, one-foot crawl=2, Run=3, Climb=4. Keeping this interface numeric
// avoids a dependency cycle with robot_gait.h.
bool robot_locomotion_profile_valid(uint8_t gait, const robot_locomotion_profile_t *profile);
robot_locomotion_profile_t robot_locomotion_default_profile(uint8_t gait);
float robot_locomotion_frequency_hz(const robot_locomotion_profile_t *profile);
float robot_locomotion_duty_factor(const robot_locomotion_profile_t *profile);
bool robot_locomotion_is_single_support_gait(uint8_t gait);
float robot_locomotion_leg_phase_offset(uint8_t gait, robot_leg_t leg);
robot_locomotion_leg_phase_t robot_locomotion_leg_phase(uint8_t gait, robot_leg_t leg,
                                                          float global_phase,
                                                          const robot_locomotion_profile_t *profile);

// MIT Mini Cheetah's public FootSwingTrajectory uses cubic Bézier progress
// in X/Z and two equal vertical Bézier sections in Y, meeting at mid-swing.
robot_vec3_t robot_locomotion_swing_bezier(robot_vec3_t liftoff,
                                            robot_vec3_t touchdown,
                                            float swing_phase,
                                            float step_height_mm);

// One leg is always airborne; the other three step in order. At least two
// scheduled contacts remain. This is dynamic three-leg walking, not crawl.
robot_locomotion_leg_phase_t robot_locomotion_tripod_phase(robot_leg_t leg,
    robot_leg_t excluded, float global_phase, const robot_locomotion_profile_t *profile);

// One mode numbering for R1, telemetry, desktop and DualSense player LEDs.
uint8_t robot_locomotion_next_mode(uint8_t mode);
uint8_t robot_locomotion_player_leds(uint8_t mode);

// MIT ConvexMPCLocomotion command low-pass, expressed with a time constant
// instead of a sample-rate-dependent coefficient. Buttons are not filtered.
float robot_locomotion_filter_command(float previous, float command, float dt);
