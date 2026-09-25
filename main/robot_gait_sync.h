#pragma once

#include <stdbool.h>

// Maps planner and measured-joint backlog to a normalized phase rate.  The
// phase may be held at zero, while actuator setpoints continue to be emitted.
float robot_gait_phase_rate(float reference_error_radians,
                            bool has_feedback, float feedback_error_radians);
