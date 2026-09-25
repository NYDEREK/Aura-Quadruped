#include <math.h>

#include "robot_static_gait_profile.h"

#define STATIC_DEFAULT_PRELOAD_FRACTION 0.25f
#define STATIC_MIN_PRELOAD_FRACTION 0.02f
#define STATIC_MAX_PRELOAD_FRACTION 0.60f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float smootherstep(float value)
{
    value = clampf(value, 0.0f, 1.0f);
    return clampf(value * value * value * (value * (value * 6.0f - 15.0f) + 10.0f),
                  0.0f, 1.0f);
}

static float cyclic(float phase)
{
    phase = fmodf(phase, 1.0f);
    return phase < 0.0f ? phase + 1.0f : phase;
}

static float safe_preload(float preload_fraction)
{
    return clampf(preload_fraction, STATIC_MIN_PRELOAD_FRACTION,
                  STATIC_MAX_PRELOAD_FRACTION);
}

robot_static_step_profile_t robot_static_step_profile_timed(float slot_phase,
                                                             float preload_fraction)
{
    const float phase = cyclic(slot_phase);
    const float preload = safe_preload(preload_fraction);
    robot_static_step_profile_t result = {
        .body_shift_fraction = phase < preload ? smootherstep(phase / preload) : 1.0f,
        .stroke_fraction = -0.5f,
        .lift_fraction = 0.0f,
        .airborne = false,
    };
    if (phase < preload) return result;

    const float progress = smootherstep((phase - preload) / (1.0f - preload));
    result.stroke_fraction = -0.5f + progress;
    result.lift_fraction = 4.0f * progress * (1.0f - progress);
    result.airborne = phase < 0.9999f;
    return result;
}

float robot_static_next_transition_timed(float slot_phase, float preload_fraction)
{
    const float phase = cyclic(slot_phase);
    const float preload = safe_preload(preload_fraction);
    return phase < preload - 0.0001f ? preload : 1.0f;
}

float robot_static_stance_stroke_fraction_timed(float leg_cycle_phase,
                                                 float preload_fraction)
{
    const float cycle = cyclic(leg_cycle_phase);
    if (cycle < 0.25f)
        return robot_static_step_profile_timed(cycle * 4.0f, preload_fraction).stroke_fraction;

    // After touchdown a foot returns over ground while the other three legs
    // take their turns, reaching liftoff without a Cartesian discontinuity.
    const float progress = smootherstep((cycle - 0.25f) / 0.75f);
    return 0.5f - progress;
}

bool robot_static_leg_airborne_timed(float leg_cycle_phase, float preload_fraction)
{
    const float cycle = cyclic(leg_cycle_phase);
    return cycle < 0.25f &&
           robot_static_step_profile_timed(cycle * 4.0f, preload_fraction).airborne;
}

robot_static_step_profile_t robot_static_step_profile(float slot_phase)
{
    return robot_static_step_profile_timed(slot_phase, STATIC_DEFAULT_PRELOAD_FRACTION);
}

float robot_static_next_transition(float slot_phase)
{
    return robot_static_next_transition_timed(slot_phase, STATIC_DEFAULT_PRELOAD_FRACTION);
}

float robot_static_stance_stroke_fraction(float leg_cycle_phase)
{
    return robot_static_stance_stroke_fraction_timed(leg_cycle_phase,
                                                      STATIC_DEFAULT_PRELOAD_FRACTION);
}

bool robot_static_leg_airborne(float leg_cycle_phase)
{
    return robot_static_leg_airborne_timed(leg_cycle_phase,
                                           STATIC_DEFAULT_PRELOAD_FRACTION);
}
