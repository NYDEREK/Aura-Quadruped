#include <math.h>
#include <string.h>

#include "robot_gait_feasibility.h"
#include "robot_locomotion.h"

#define SAMPLES_PER_CYCLE 64
// Matches ROBOT_BODY_COM_VERTICAL_OFFSET_MM in robot_gait.c.
#define COM_BELOW_TORSO_MM 25.0f

typedef struct {
    float speed, acceleration, two_foot;
    bool ok;
} cycle_peaks_t;

static cycle_peaks_t simulate(robot_body_trajectory_request_t *request, float body_height,
                              robot_body_trajectory_t *plan)
{
    cycle_peaks_t peaks = {.ok = true};
    plan->request_cached = false;
    const bool planned = robot_body_trajectory_build(plan, request);
    const float hz = robot_locomotion_frequency_hz(&request->profile);
    const float dt = 1.0f / (hz * SAMPLES_PER_CYCLE);
    float previous[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT] = {{0}};
    float previous_speed[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT] = {{0}};
    float two_foot = 0.0f;
    // Two cycles, so the wrap-around step is differenced as well.
    for (int k = 0; k <= 2 * SAMPLES_PER_CYCLE; ++k) {
        const float phase = (float)(k % SAMPLES_PER_CYCLE) / SAMPLES_PER_CYCLE;
        robot_planar_point_t body = {0};
        if (planned) body = robot_body_trajectory_sample(plan, phase).position_mm;
        float q[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
        int stance = 0;
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
            const robot_vec3_t foot = robot_body_trajectory_foot(request, (robot_leg_t)leg, phase);
            const robot_vec3_t target = {foot.x - body.x, foot.y, foot.z - body.z};
            if (!robot_kinematics_inverse(&request->geometry, (robot_leg_t)leg, target,
                                          body_height, q[leg]))
                peaks.ok = false;
            stance += robot_locomotion_leg_phase(request->gait, (robot_leg_t)leg, phase,
                                                 &request->profile).support_contact;
        }
        if (k >= SAMPLES_PER_CYCLE) {
            two_foot = stance == 2 ? two_foot + dt : 0.0f;
            if (two_foot > peaks.two_foot) peaks.two_foot = two_foot;
        }
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
            for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
                const float speed = (q[leg][axis] - previous[leg][axis]) / dt;
                if (k >= 1 && fabsf(speed) > peaks.speed) peaks.speed = fabsf(speed);
                if (k >= 2) {
                    const float acceleration = fabsf(speed - previous_speed[leg][axis]) / dt;
                    if (acceleration > peaks.acceleration) peaks.acceleration = acceleration;
                }
                previous_speed[leg][axis] = speed;
                previous[leg][axis] = q[leg][axis];
            }
    }
    return peaks;
}

bool robot_gait_feasibility(uint8_t gait, const robot_locomotion_profile_t *profile,
                            const robot_geometry_t *geometry, float body_height_mm,
                            float lateral_stance_mm, float max_speed_rad_s,
                            float max_acceleration_rad_s2, robot_body_trajectory_t *scratch,
                            robot_gait_feasibility_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!profile || !geometry || !scratch || !(body_height_mm > 0.0f) ||
        !(max_speed_rad_s > 0.0f) || !(max_acceleration_rad_s2 > 0.0f) ||
        !robot_locomotion_profile_valid(gait, profile))
        return false;
    robot_body_trajectory_request_t request = {
        .gait = gait, .profile = *profile, .geometry = *geometry,
        .com_height_mm = body_height_mm - COM_BELOW_TORSO_MM,
        .lateral_stance_mm = lateral_stance_mm, .forward = 1.0f,
        .stride_mm = (float)profile->stride_mm,
    };
    const cycle_peaks_t straight = simulate(&request, body_height_mm, scratch);
    request.forward = 0.7f;
    request.turn = 0.7f;
    const cycle_peaks_t turning = simulate(&request, body_height_mm, scratch);
    scratch->request_cached = false;
    scratch->valid = false;
    if (!straight.ok || !turning.ok) return false;
    out->peak_speed_rad_s = fmaxf(straight.speed, turning.speed);
    out->peak_acceleration_rad_s2 = fmaxf(straight.acceleration, turning.acceleration);
    out->two_foot_s = fmaxf(straight.two_foot, turning.two_foot);
    out->rate = fminf(1.0f, fminf(max_speed_rad_s / fmaxf(out->peak_speed_rad_s, 1e-3f),
                                  sqrtf(max_acceleration_rad_s2 /
                                        fmaxf(out->peak_acceleration_rad_s2, 1e-3f))));
    out->valid = true;
    return true;
}
