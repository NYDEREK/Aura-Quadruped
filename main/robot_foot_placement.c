#include <math.h>

#include "robot_foot_placement.h"

#define GRAVITY_MM_S2 9810.0f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float deadband(float value, float band)
{
    if (value > band) return value - band;
    if (value < -band) return value + band;
    return 0.0f;
}

// Same progress as the MIT cubic Bézier used for the horizontal swing.
static float swing_progress(float phase)
{
    const float t = clampf(phase, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

robot_foot_placement_config_t robot_foot_placement_default_config(void)
{
    return (robot_foot_placement_config_t){
        .gain = 1.0f,
        .limit_mm = 45.0f,
        .deadband_mm_s = 5.0f,
        .filter_time_s = 0.04f,
    };
}

void robot_foot_placement_reset(robot_foot_placement_t *placement)
{
    if (placement) *placement = (robot_foot_placement_t){0};
}

robot_planar_point_t robot_foot_placement_tipping_velocity(float roll_rate, float pitch_rate,
                                                            float commanded_roll_rate,
                                                            float commanded_pitch_rate,
                                                            float height)
{
    if (!isfinite(roll_rate) || !isfinite(pitch_rate) || !isfinite(commanded_roll_rate) ||
        !isfinite(commanded_pitch_rate) || !isfinite(height) || height <= 0.0f)
        return (robot_planar_point_t){0};
    const float roll_tip = roll_rate - commanded_roll_rate;
    const float pitch_tip = pitch_rate - commanded_pitch_rate;
    // Left side up (roll > 0) swings the CoM to the right (-Z).
    // Nose down (pitch > 0) swings the CoM forward (+X).
    return (robot_planar_point_t){.x = height * pitch_tip, .z = -height * roll_tip};
}

robot_planar_point_t robot_foot_placement_tipping_displacement(float roll_error,
                                                                float pitch_error, float height)
{
    if (!isfinite(roll_error) || !isfinite(pitch_error) || !isfinite(height) || height <= 0.0f)
        return (robot_planar_point_t){0};
    return (robot_planar_point_t){.x = height * sinf(pitch_error),
                                  .z = -height * sinf(roll_error)};
}

robot_planar_point_t robot_foot_placement_update(robot_foot_placement_t *p,
                                                 const robot_foot_placement_config_t *config,
                                                 robot_planar_point_t displacement,
                                                 robot_planar_point_t velocity_error,
                                                 float height, float dt)
{
    if (!p || !config || !isfinite(velocity_error.x) || !isfinite(velocity_error.z) ||
        !isfinite(displacement.x) || !isfinite(displacement.z) ||
        !isfinite(height) || height <= 0.0f || !isfinite(dt) || dt <= 0.0f)
        return (robot_planar_point_t){0};
    const float alpha = config->filter_time_s > 0.0f
        ? -expm1f(-dt / config->filter_time_s) : 1.0f;
    p->filtered_velocity_error.x += alpha * (velocity_error.x - p->filtered_velocity_error.x);
    p->filtered_velocity_error.z += alpha * (velocity_error.z - p->filtered_velocity_error.z);
    // Equation (6), capture-point term sqrt(z0 / g) * (v - v_des), plus the
    // body drift that p_h,i contains in the paper (see header).
    const float time_constant = sqrtf(height / GRAVITY_MM_S2);
    const float band = config->deadband_mm_s * time_constant;
    const float x = displacement.x + time_constant * p->filtered_velocity_error.x;
    const float z = displacement.z + time_constant * p->filtered_velocity_error.z;
    return (robot_planar_point_t){
        .x = clampf(config->gain * deadband(x, band), -config->limit_mm, config->limit_mm),
        .z = clampf(config->gain * deadband(z, band), -config->limit_mm, config->limit_mm),
    };
}

robot_vec3_t robot_foot_placement_apply(robot_foot_placement_t *p, robot_leg_t leg,
                                        robot_vec3_t planned, bool swing, float swing_phase,
                                        robot_planar_point_t live)
{
    if (!p || leg >= ROBOT_LEG_COUNT || !isfinite(live.x) || !isfinite(live.z)) return planned;
    robot_planar_point_t offset;
    if (swing) {
        if (!p->swinging[leg]) {
            p->swinging[leg] = true;
            p->liftoff[leg] = p->landed[leg];
        }
        // Moving target: recomputed every tick like Cheetah's swing planner.
        // At liftoff the foot keeps the offset it stood on; at touchdown
        // it reaches the newest capture-point offset.
        const float b = swing_progress(swing_phase);
        offset = (robot_planar_point_t){
            .x = p->liftoff[leg].x + (live.x - p->liftoff[leg].x) * b,
            .z = p->liftoff[leg].z + (live.z - p->liftoff[leg].z) * b,
        };
        p->current[leg] = offset;
    } else {
        if (p->swinging[leg]) {
            p->swinging[leg] = false;
            p->landed[leg] = p->current[leg];
        }
        offset = p->landed[leg];
    }
    planned.x += offset.x;
    planned.z += offset.z;
    return planned;
}
