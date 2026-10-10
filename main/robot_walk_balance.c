#include <math.h>

#include "robot_walk_balance.h"

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

// Same progress as the MIT cubic Bézier of the horizontal swing.
static float swing_progress(float phase)
{
    const float t = clampf(phase, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

robot_planar_point_t robot_walk_tilt_displacement(robot_tilt_t tilt, float height)
{
    if (!isfinite(tilt.roll) || !isfinite(tilt.pitch) || !isfinite(height) || height <= 0.0f)
        return (robot_planar_point_t){0};
    // Rx(roll) carries a point at height h to +Z (left); Rz(pitch) to -X.
    return (robot_planar_point_t){.x = -height * sinf(tilt.pitch),
                                  .z = height * sinf(tilt.roll)};
}

robot_planar_point_t robot_walk_tilt_velocity(robot_tilt_t rate, float height)
{
    if (!isfinite(rate.roll) || !isfinite(rate.pitch) || !isfinite(height) || height <= 0.0f)
        return (robot_planar_point_t){0};
    return (robot_planar_point_t){.x = -height * rate.pitch, .z = height * rate.roll};
}

robot_walk_balance_config_t robot_walk_balance_default_config(void)
{
    return (robot_walk_balance_config_t){
        .gain = 1.0f,
        .limit_mm = 40.0f,
        .deadband_mm = 1.5f,
        .filter_time_s = 0.04f,
        .freeze_swing = 0.85f,
    };
}

void robot_walk_balance_reset(robot_walk_balance_t *balance)
{
    if (balance) *balance = (robot_walk_balance_t){0};
}

robot_planar_point_t robot_walk_balance_capture(robot_walk_balance_t *b,
                                                const robot_walk_balance_config_t *config,
                                                robot_planar_point_t displacement,
                                                robot_planar_point_t velocity,
                                                float height, float dt)
{
    if (!b || !config || !isfinite(displacement.x) || !isfinite(displacement.z) ||
        !isfinite(velocity.x) || !isfinite(velocity.z) || !isfinite(height) ||
        height <= 0.0f || !isfinite(dt) || dt <= 0.0f)
        return b ? b->capture : (robot_planar_point_t){0};
    const float alpha = config->filter_time_s > 0.0f ? -expm1f(-dt / config->filter_time_s) : 1.0f;
    b->filtered_velocity.x += alpha * (velocity.x - b->filtered_velocity.x);
    b->filtered_velocity.z += alpha * (velocity.z - b->filtered_velocity.z);
    const float time_constant = sqrtf(height / GRAVITY_MM_S2);
    const float x = displacement.x + time_constant * b->filtered_velocity.x;
    const float z = displacement.z + time_constant * b->filtered_velocity.z;
    b->capture = (robot_planar_point_t){
        .x = clampf(config->gain * deadband(x, config->deadband_mm), -config->limit_mm, config->limit_mm),
        .z = clampf(config->gain * deadband(z, config->deadband_mm), -config->limit_mm, config->limit_mm),
    };
    return b->capture;
}

robot_vec3_t robot_walk_balance_apply(robot_walk_balance_t *b,
                                      const robot_walk_balance_config_t *config,
                                      robot_leg_t leg, robot_vec3_t planned,
                                      bool swing, float swing_phase,
                                      robot_planar_point_t capture)
{
    if (!b || !config || leg >= ROBOT_LEG_COUNT || !isfinite(capture.x) || !isfinite(capture.z))
        return planned;
    robot_planar_point_t offset;
    if (swing) {
        if (!b->swinging[leg]) {
            b->swinging[leg] = true;
            b->liftoff[leg] = b->landed[leg];
        }
        if (swing_phase < config->freeze_swing) {
            const float blend = swing_progress(swing_phase / config->freeze_swing);
            b->current[leg] = (robot_planar_point_t){
                .x = b->liftoff[leg].x + (capture.x - b->liftoff[leg].x) * blend,
                .z = b->liftoff[leg].z + (capture.z - b->liftoff[leg].z) * blend,
            };
        }
        offset = b->current[leg];
    } else {
        if (b->swinging[leg]) {
            b->swinging[leg] = false;
            b->landed[leg] = b->current[leg];
        }
        offset = b->landed[leg];
    }
    planned.x += offset.x;
    planned.z += offset.z;
    return planned;
}
