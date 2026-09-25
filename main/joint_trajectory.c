#include <math.h>

#include "joint_trajectory.h"

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

void joint_trajectory_reset(joint_trajectory_t *trajectory, float position)
{
    if (!trajectory) return;
    trajectory->position = isfinite(position) ? position : 0.0f;
    trajectory->velocity = 0.0f;
    trajectory->initialized = true;
    trajectory->previous_reference = trajectory->position;
    trajectory->tracking_reference = false;
}

float joint_trajectory_step(joint_trajectory_t *trajectory, float reference,
                            float dt, joint_trajectory_limits_t limits)
{
    if (!trajectory || !isfinite(reference) || !isfinite(dt) || dt <= 0.0f ||
        !isfinite(limits.maximum_velocity) || !isfinite(limits.maximum_acceleration) ||
        limits.maximum_velocity <= 0.0f || limits.maximum_acceleration <= 0.0f)
        return trajectory ? trajectory->position : 0.0f;
    if (!trajectory->initialized) joint_trajectory_reset(trajectory, reference);

    trajectory->tracking_reference = false;
    trajectory->previous_reference = reference;
    const float error = reference - trajectory->position;
    if (fabsf(error) < 1e-5f && fabsf(trajectory->velocity) < 1e-4f) {
        trajectory->position = reference;
        trajectory->velocity = 0.0f;
        return trajectory->position;
    }

    // The braking bound makes the desired velocity converge to zero at the
    // reference instead of accelerating at full rate until it overshoots.
    // Account for the distance covered by the coming sample before braking.
    // This discrete form prevents a 20 ms controller from arriving one frame
    // beyond the reference and then having to reverse abruptly.
    const float acceleration_step = limits.maximum_acceleration * dt;
    const float braking_velocity = sqrtf(acceleration_step * acceleration_step +
                                         2.0f * limits.maximum_acceleration * fabsf(error)) -
                                    acceleration_step;
    const float desired_velocity = copysignf(fminf(limits.maximum_velocity, braking_velocity), error);
    const float maximum_delta_velocity = acceleration_step;
    float next_velocity = trajectory->velocity + clampf(desired_velocity - trajectory->velocity,
                                                         -maximum_delta_velocity,
                                                         maximum_delta_velocity);
    next_velocity = clampf(next_velocity, -limits.maximum_velocity, limits.maximum_velocity);
    float next_position = trajectory->position + next_velocity * dt;

    // The discrete braking bound above keeps this branch to numerical noise.
    // It is a final guard rather than a normal source of zeroing velocity.
    if ((error > 0.0f && next_position >= reference) ||
        (error < 0.0f && next_position <= reference)) {
        next_position = reference;
        next_velocity = 0.0f;
    }
    trajectory->position = next_position;
    trajectory->velocity = next_velocity;
    return trajectory->position;
}

float joint_trajectory_track(joint_trajectory_t *trajectory, float reference,
                             float dt, joint_trajectory_limits_t limits)
{
    if (!trajectory || !isfinite(reference) || !isfinite(dt) || dt<=0 ||
        !isfinite(limits.maximum_velocity) || !isfinite(limits.maximum_acceleration) ||
        limits.maximum_velocity<=0 || limits.maximum_acceleration<=0)
        return trajectory ? trajectory->position : 0;
    if (!trajectory->initialized) joint_trajectory_reset(trajectory,reference);
    // On entry there is no previous timed sample; acquire it without
    // interpreting an arbitrary pose difference as a reference velocity.
    if (!trajectory->tracking_reference) trajectory->previous_reference=reference;
    const float reference_velocity=(reference-trajectory->previous_reference)/dt;
    const float error=trajectory->previous_reference-trajectory->position;
    // Four 50 Hz samples is the acquisition time constant. In steady-state,
    // feasible references pass through exactly (zero causal position lag).
    const float desired=clampf(reference_velocity+error/0.08f,
        -limits.maximum_velocity,limits.maximum_velocity);
    trajectory->velocity += clampf(desired-trajectory->velocity,
        -limits.maximum_acceleration*dt,limits.maximum_acceleration*dt);
    trajectory->velocity=clampf(trajectory->velocity,-limits.maximum_velocity,limits.maximum_velocity);
    trajectory->position += trajectory->velocity*dt;
    trajectory->previous_reference=reference;
    trajectory->tracking_reference=true;
    return trajectory->position;
}
