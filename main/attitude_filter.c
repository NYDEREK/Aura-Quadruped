#include <math.h>

#include "attitude_filter.h"

#define ATTITUDE_PI 3.14159265358979323846f
#define ATTITUDE_MAX_DT_S 0.050f
#define ATTITUDE_ACCEL_MIN_G 0.78f
#define ATTITUDE_ACCEL_MAX_G 1.22f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float wrap_pi(float angle)
{
    while (angle > ATTITUDE_PI) angle -= 2.0f * ATTITUDE_PI;
    while (angle < -ATTITUDE_PI) angle += 2.0f * ATTITUDE_PI;
    return angle;
}

static float blend_angle(float current, float measurement, float weight)
{
    return wrap_pi(current + wrap_pi(measurement - current) * weight);
}

void attitude_filter_reset(attitude_filter_t *filter)
{
    if (!filter) return;
    *filter = (attitude_filter_t){0};
}

bool attitude_filter_update(attitude_filter_t *filter, const float accel_g[3],
                            const float gyro_dps[3], int64_t sample_time_us,
                            float *roll_radians, float *pitch_radians)
{
    if (!filter || !accel_g || !gyro_dps || sample_time_us <= 0 ||
        !isfinite(accel_g[0]) || !isfinite(accel_g[1]) || !isfinite(accel_g[2]) ||
        !isfinite(gyro_dps[0]) || !isfinite(gyro_dps[1]) || !isfinite(gyro_dps[2]))
        return false;

    const float magnitude = sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] +
                                  accel_g[2] * accel_g[2]);
    const bool gravity_trustworthy = magnitude >= ATTITUDE_ACCEL_MIN_G &&
                                     magnitude <= ATTITUDE_ACCEL_MAX_G;
    const float accel_roll = atan2f(accel_g[2], accel_g[1]);
    const float accel_pitch = atan2f(-accel_g[0],
                                     sqrtf(accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]));

    if (!filter->initialized) {
        if (!gravity_trustworthy) return false;
        filter->initialized = true;
        filter->roll_radians = accel_roll;
        filter->pitch_radians = accel_pitch;
        filter->last_sample_time_us = sample_time_us;
    } else {
        const float dt = clampf((float)(sample_time_us - filter->last_sample_time_us) / 1000000.0f,
                                0.0f, ATTITUDE_MAX_DT_S);
        filter->last_sample_time_us = sample_time_us;
        // X is the roll axis; Z is the pitch axis in Aura's X-forward,
        // Y-up, Z-left right-handed frame.
        filter->roll_radians = wrap_pi(filter->roll_radians +
                                        gyro_dps[0] * ATTITUDE_PI / 180.0f * dt);
        filter->pitch_radians = wrap_pi(filter->pitch_radians +
                                         gyro_dps[2] * ATTITUDE_PI / 180.0f * dt);
        if (gravity_trustworthy) {
            // At 100 Hz, this gives the gyro the fast path while gravity
            // removes drift over roughly half a second. The blend is reduced
            // as linear acceleration moves the measured norm away from 1 g.
            const float trust = 1.0f - fabsf(magnitude - 1.0f) / 0.22f;
            const float correction = clampf(0.025f * trust, 0.0f, 0.025f);
            filter->roll_radians = blend_angle(filter->roll_radians, accel_roll, correction);
            filter->pitch_radians = blend_angle(filter->pitch_radians, accel_pitch, correction);
        }
    }
    if (roll_radians) *roll_radians = filter->roll_radians;
    if (pitch_radians) *pitch_radians = filter->pitch_radians;
    return true;
}
