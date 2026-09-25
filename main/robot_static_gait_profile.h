#pragma once

#include <stdbool.h>

// A quasistatic step first transfers the CoM, then follows a single smooth
// swing arch. The contact phase remains level on the ground; no separate
// vertical and horizontal segments create corners in the Cartesian foot path.
typedef struct {
    // 0…1: body transfer from the previous support triangle to the support
    // triangle that will remain after the current leg lifts.
    float body_shift_fraction;
    // -0.5…+0.5: fore/aft foot position within a stride.
    float stroke_fraction;
    // 0…1: height above the ground, scaled by the configured clearance.
    float lift_fraction;
    bool airborne;
} robot_static_step_profile_t;

// `slot_phase` spans one leg's slot, in [0, 1). `preload_fraction` is the
// beginning part of that slot reserved for CoM transfer. It is derived from
// the one-foot gait contact percentage, so the UI's Kontakt parameter changes
// real timing as well as the simulator.
robot_static_step_profile_t robot_static_step_profile_timed(float slot_phase,
                                                             float preload_fraction);
float robot_static_next_transition_timed(float slot_phase, float preload_fraction);
float robot_static_stance_stroke_fraction_timed(float leg_cycle_phase,
                                                 float preload_fraction);
bool robot_static_leg_airborne_timed(float leg_cycle_phase, float preload_fraction);

// Legacy 25% pre-load wrappers retained for focused unit tests and callers
// that deliberately need the original commissioning profile.
robot_static_step_profile_t robot_static_step_profile(float slot_phase);
float robot_static_next_transition(float slot_phase);
float robot_static_stance_stroke_fraction(float leg_cycle_phase);
bool robot_static_leg_airborne(float leg_cycle_phase);
