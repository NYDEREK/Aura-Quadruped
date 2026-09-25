#include <math.h>

#include "robot_locomotion.h"

#define GAIT_TROT 1u
#define GAIT_CRAWL 2u
#define GAIT_RUN 3u
#define GAIT_CLIMB 4u
#define GAIT_TRIPOD 5u

#define PROFILE_MIN_STRIDE_MM 15u
#define PROFILE_MAX_STRIDE_MM 300u
#define PROFILE_MIN_LIFT_MM 5u
#define PROFILE_MAX_LIFT_MM 100u
#define PROFILE_MIN_FREQUENCY_CHZ 15u
#define PROFILE_DYNAMIC_MAX_FREQUENCY_CHZ 300u
#define PROFILE_SINGLE_FOOT_MAX_FREQUENCY_CHZ 120u
#define PROFILE_MIN_DUTY_PERCENT 45u
#define PROFILE_MAX_DUTY_PERCENT 85u
#define PROFILE_SINGLE_FOOT_MIN_DUTY_PERCENT 76u
#define PROFILE_SINGLE_FOOT_MAX_DUTY_PERCENT 90u

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float wrap01(float phase)
{
    phase = fmodf(phase, 1.0f);
    return phase < 0.0f ? phase + 1.0f : phase;
}

// Cubic Bezier with coincident endpoint control points. It is precisely the
// interpolation used by MIT's FootSwingTrajectory for the horizontal swing.
static float cubic_bezier(float from, float to, float phase)
{
    const float t = clampf(phase, 0.0f, 1.0f);
    const float progress = t * t * (3.0f - 2.0f * t);
    return from + (to - from) * progress;
}

robot_locomotion_profile_t robot_locomotion_default_profile(uint8_t gait)
{
    switch (gait) {
    case GAIT_TRIPOD:
        return (robot_locomotion_profile_t){
            .stride_mm = 35, .step_height_mm = 40, .frequency_centi_hz = 65,
            .duty_percent = 78,
        };
    case GAIT_CRAWL:
        return (robot_locomotion_profile_t){
            .stride_mm = 50, .step_height_mm = 30, .frequency_centi_hz = 65,
            .duty_percent = 82,
        };
    case GAIT_TROT:
        // A trot stands on one diagonal line during swing; with the LIPM time
        // constant sqrt(h/g) ~ 0.15 s that phase must stay short. 2 Hz / 55 %
        // gives ~0.25 s of swing (ACC = 0), inside the range where the
        // capture-point foot placement recovers (tests/test_robot_foot_placement.c).
        return (robot_locomotion_profile_t){
            .stride_mm = 50, .step_height_mm = 35, .frequency_centi_hz = 200,
            .duty_percent = 55,
        };
    case GAIT_RUN:
        // "Run" here is a fast diagonal gait without a flight phase. A true
        // Mini Cheetah flight gait needs force/torque control unavailable on
        // this position-servo bus.
        return (robot_locomotion_profile_t){
            .stride_mm = 90, .step_height_mm = 72, .frequency_centi_hz = 190,
            .duty_percent = 52,
        };
    case GAIT_CLIMB:
        return (robot_locomotion_profile_t){
            .stride_mm = 50, .step_height_mm = 82, .frequency_centi_hz = 65,
            .duty_percent = 82,
        };
    default:
        return (robot_locomotion_profile_t){0};
    }
}

bool robot_locomotion_profile_valid(uint8_t gait, const robot_locomotion_profile_t *profile)
{
    if (!profile || gait < GAIT_TROT || gait > GAIT_TRIPOD) return false;
    const uint8_t minimum_duty = (robot_locomotion_is_single_support_gait(gait) || gait == GAIT_TRIPOD)
        ? PROFILE_SINGLE_FOOT_MIN_DUTY_PERCENT : PROFILE_MIN_DUTY_PERCENT;
    const uint8_t maximum_duty = (robot_locomotion_is_single_support_gait(gait) || gait == GAIT_TRIPOD)
        ? PROFILE_SINGLE_FOOT_MAX_DUTY_PERCENT : PROFILE_MAX_DUTY_PERCENT;
    const uint16_t maximum_frequency = (robot_locomotion_is_single_support_gait(gait) || gait == GAIT_TRIPOD)
        ? PROFILE_SINGLE_FOOT_MAX_FREQUENCY_CHZ : PROFILE_DYNAMIC_MAX_FREQUENCY_CHZ;
    return profile->stride_mm >= PROFILE_MIN_STRIDE_MM &&
           profile->stride_mm <= PROFILE_MAX_STRIDE_MM &&
           profile->step_height_mm >= PROFILE_MIN_LIFT_MM &&
           profile->step_height_mm <= PROFILE_MAX_LIFT_MM &&
           profile->frequency_centi_hz >= PROFILE_MIN_FREQUENCY_CHZ &&
           profile->frequency_centi_hz <= maximum_frequency &&
           profile->duty_percent >= minimum_duty &&
           profile->duty_percent <= maximum_duty;
}

float robot_locomotion_frequency_hz(const robot_locomotion_profile_t *profile)
{
    return profile ? (float)profile->frequency_centi_hz / 100.0f : 0.0f;
}

float robot_locomotion_duty_factor(const robot_locomotion_profile_t *profile)
{
    return profile ? (float)profile->duty_percent / 100.0f : 1.0f;
}

bool robot_locomotion_is_single_support_gait(uint8_t gait)
{
    return gait == GAIT_CRAWL || gait == GAIT_CLIMB;
}

float robot_locomotion_leg_phase_offset(uint8_t gait, robot_leg_t leg)
{
    // One-foot transfer: RR -> LF -> LR -> RF. Every other moving profile
    // uses diagonal pairs LF/RR and RF/LR.
    if (robot_locomotion_is_single_support_gait(gait)) {
        static const float offsets[ROBOT_LEG_COUNT] = {
            [ROBOT_LEG_LEFT_FRONT] = 0.50f,
            [ROBOT_LEG_RIGHT_FRONT] = 0.00f,
            [ROBOT_LEG_LEFT_REAR] = 0.25f,
            [ROBOT_LEG_RIGHT_REAR] = 0.75f,
        };
        return leg < ROBOT_LEG_COUNT ? offsets[leg] : 0.0f;
    }
    return (leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_RIGHT_REAR) ? 0.0f : 0.5f;
}

robot_locomotion_leg_phase_t robot_locomotion_leg_phase(uint8_t gait, robot_leg_t leg,
                                                          float global_phase,
                                                          const robot_locomotion_profile_t *profile)
{
    robot_locomotion_leg_phase_t result = {0};
    const float duty = clampf(robot_locomotion_duty_factor(profile), 0.01f, 0.99f);
    result.local_phase = wrap01(global_phase + robot_locomotion_leg_phase_offset(gait, leg));
    result.support_contact = result.local_phase < duty;
    result.scheduled_swing = !result.support_contact;
    result.swing_phase = result.scheduled_swing
        ? (result.local_phase - duty) / (1.0f - duty) : 0.0f;
    return result;
}

robot_vec3_t robot_locomotion_swing_bezier(robot_vec3_t liftoff,
                                            robot_vec3_t touchdown,
                                            float swing_phase,
                                            float step_height_mm)
{
    const float phase = clampf(swing_phase, 0.0f, 1.0f);
    robot_vec3_t result = {
        .x = cubic_bezier(liftoff.x, touchdown.x, phase),
        .y = 0.0f,
        .z = cubic_bezier(liftoff.z, touchdown.z, phase),
    };
    const float apex = fmaxf(liftoff.y, touchdown.y) + fmaxf(0.0f, step_height_mm);
    // The public MIT FootSwingTrajectory places the vertical apex at the
    // middle of swing, using two cubic Bézier sections.
    const float apex_phase = 0.5f;
    result.y = phase < apex_phase
        ? cubic_bezier(liftoff.y, apex, phase / apex_phase)
        : cubic_bezier(apex, touchdown.y, (phase - apex_phase) / (1.0f - apex_phase));
    return result;
}

robot_locomotion_leg_phase_t robot_locomotion_tripod_phase(robot_leg_t leg,
    robot_leg_t excluded, float phase, const robot_locomotion_profile_t *profile)
{
    if (leg == excluded) return (robot_locomotion_leg_phase_t){
        .scheduled_swing=true, .swing_phase=0.5f, .local_phase=0.9f};
    const robot_leg_t order[4] = {ROBOT_LEG_RIGHT_REAR, ROBOT_LEG_LEFT_FRONT,
                                 ROBOT_LEG_LEFT_REAR, ROBOT_LEG_RIGHT_FRONT};
    int slot = 0;
    for (int i=0; i<4; ++i) {
        if (order[i] == excluded) continue;
        if (order[i] == leg) break;
        ++slot;
    }
    const float duty = robot_locomotion_duty_factor(profile);
    // Phase zero starts in the all-three-contact overlap.
    const float local = wrap01(phase + (float)slot/3.0f);
    const bool swing = local >= duty;
    return (robot_locomotion_leg_phase_t){
        .scheduled_swing=swing, .support_contact=!swing, .local_phase=local,
        .swing_phase=swing ? (local-duty)/(1-duty) : 0};
}

uint8_t robot_locomotion_next_mode(uint8_t mode)
{ return mode < GAIT_TRIPOD ? mode+1 : 0; }
uint8_t robot_locomotion_player_leds(uint8_t mode)
{
    static const uint8_t masks[]={0,0x04,0x0a,0x15,0x1b,0x1f};
    return mode <= GAIT_TRIPOD ? masks[mode] : 0;
}

float robot_locomotion_filter_command(float previous, float command, float dt)
{
    if (!isfinite(previous) || !isfinite(command) || !isfinite(dt) || dt<0) return 0;
    // alpha = 0.1 at the ESP's 20 ms period (tau = -0.02/log(0.9)).
    return previous + (-expm1f(-dt/0.18982443f))*(command-previous);
}
