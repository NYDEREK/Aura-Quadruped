#pragma once

#include <stdbool.h>

#include "robot_static_balance.h"

// Continuous availability of a foot in a scheduled gait phase.  It follows
// the contact/swing weighting used by Cheetah 3's Virtual Predictive Support
// Polygon: a planted foot is most trusted at middle stance, while a swinging
// foot contributes least at middle swing and transitions smoothly at liftoff
// and touchdown.
float robot_predictive_support_availability(float local_phase, float duty_factor);

// Equations (13)-(15) from the Cheetah 3 VPSP construction. `feet` are the
// ground-plane locations of all four feet, including scheduled touchdown
// locations for legs in swing. `availability` is one phase weight per foot.
// The output is the continuously moving desired CoM ground projection.
bool robot_predictive_support_target(const robot_planar_point_t feet[4],
                                     const float availability[4],
                                     robot_planar_point_t *out_target);
