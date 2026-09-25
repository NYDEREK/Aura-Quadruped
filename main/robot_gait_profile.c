#include <math.h>

#include "robot_gait_profile.h"
#include "robot_locomotion.h"

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

robot_foot_path_t robot_gait_foot_path(float phase, float duty_factor,
                                       float stride_mm, float step_height_mm,
                                       bool active)
{
    robot_foot_path_t result = {.stance = true};
    if (!active || !isfinite(phase) || !isfinite(duty_factor) ||
        !isfinite(stride_mm) || !isfinite(step_height_mm))
        return result;

    phase = fmodf(phase, 1.0f);
    if (phase < 0.0f) phase += 1.0f;
    const float duty = clampf(duty_factor, 0.05f, 0.95f);
    result.stance = phase < duty;

    if (result.stance) {
        const float stance_phase = phase / duty;
        // The ground stroke remains level. The body plan, rather than an
        // unrelated gravity modifier, moves the torso relative to it.
        // Stationary WORLD contact, viewed from the uniformly translating
        // gait frame. Easing a planted foot also eases the body velocity and
        // injects an unplanned acceleration into the balance calculation.
        result.stroke_mm = stride_mm * (0.5f - stance_phase);
        result.lift_mm = 0.0f;
        return result;
    }

    const float swing_phase = (phase - duty) / (1.0f - duty);
    // One authoritative FootSwingTrajectory implementation is shared by
    // dynamic gait, static crawl and the phase wrapper in this file.
    const robot_vec3_t swing = robot_locomotion_swing_bezier(
        (robot_vec3_t){.x = -0.5f * stride_mm},
        (robot_vec3_t){.x = -0.5f * stride_mm + stride_mm / duty},
        swing_phase, fmaxf(0.0f, step_height_mm));
    // MIT's Bézier is in the WORLD frame (zero endpoint velocity there).
    // Subtract the nominal body's travel during swing before local IK.
    // This gives the same -v endpoint velocity as stance, without changing
    // the requested contact-to-contact stride or lift.
    result.stroke_mm = swing.x - stride_mm * (1.0f - duty) / duty * swing_phase;
    result.lift_mm = swing.y;
    return result;
}
