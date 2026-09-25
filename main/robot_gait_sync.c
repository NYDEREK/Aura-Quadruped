#include <math.h>

#include "robot_gait_sync.h"

#define ROBOT_PI 3.14159265358979323846f
#define REFERENCE_SOFT_ERROR_RAD (3.0f * ROBOT_PI / 180.0f)
#define REFERENCE_HOLD_ERROR_RAD (9.0f * ROBOT_PI / 180.0f)
#define FEEDBACK_SOFT_ERROR_RAD (5.0f * ROBOT_PI / 180.0f)
#define FEEDBACK_HOLD_ERROR_RAD (16.0f * ROBOT_PI / 180.0f)

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float smootherstep(float value)
{
    value = clampf(value, 0.0f, 1.0f);
    return value * value * value * (value * (value * 6.0f - 15.0f) + 10.0f);
}

static float rate_for_error(float error, float soft, float hold)
{
    if (!isfinite(error) || error <= soft) return 1.0f;
    if (error >= hold) return 0.0f;
    return 1.0f - smootherstep((error - soft) / (hold - soft));
}

float robot_gait_phase_rate(float reference_error_radians,
                            bool has_feedback, float feedback_error_radians)
{
    const float planner_rate = rate_for_error(fabsf(reference_error_radians),
                                              REFERENCE_SOFT_ERROR_RAD,
                                              REFERENCE_HOLD_ERROR_RAD);
    const float feedback_rate = has_feedback
        ? rate_for_error(fabsf(feedback_error_radians), FEEDBACK_SOFT_ERROR_RAD,
                         FEEDBACK_HOLD_ERROR_RAD)
        : 1.0f;
    return fminf(planner_rate, feedback_rate);
}
