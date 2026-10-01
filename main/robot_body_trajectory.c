#include <math.h>
#include "robot_body_trajectory.h"
#include "robot_gait_profile.h"

#define N ROBOT_BODY_TRAJECTORY_SAMPLES
#define GRAVITY_MM_S2 9810.0f

static robot_planar_point_t add(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x + b.x, a.z + b.z}; }
static robot_planar_point_t sub(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x - b.x, a.z - b.z}; }
static robot_planar_point_t mul(robot_planar_point_t a, float s)
{ return (robot_planar_point_t){a.x * s, a.z * s}; }
static float dot(robot_planar_point_t a, robot_planar_point_t b)
{ return a.x * b.x + a.z * b.z; }
static float cross(robot_planar_point_t a, robot_planar_point_t b)
{ return a.x * b.z - a.z * b.x; }

robot_vec3_t robot_body_trajectory_foot(const robot_body_trajectory_request_t *r,
                                       robot_leg_t leg, float phase)
{
    robot_vec3_t base = robot_kinematics_neutral_foot(&r->geometry, leg);
    base.z += leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR
        ? r->lateral_stance_mm : -r->lateral_stance_mm;
    if (r->gait == 5 && leg == r->excluded_leg) {
        base.y = fmaxf(40, r->profile.step_height_mm);
        return base;
    }
    float local = r->gait == 5
        ? robot_locomotion_tripod_phase(leg, r->excluded_leg, phase, &r->profile).local_phase
        : phase + robot_locomotion_leg_phase_offset(r->gait, leg);
    local -= floorf(local);
    const robot_foot_path_t path = robot_gait_foot_path(local,
        robot_locomotion_duty_factor(&r->profile), r->stepping_in_place ? 0 : r->stride_mm,
        r->profile.step_height_mm, true);
    robot_vec3_t target = base;
    if (r->spin) {
        const float a = r->turn * path.stroke_mm / fmaxf(1, hypotf(base.x, base.z));
        target.x = cosf(a) * base.x + sinf(a) * base.z;
        target.z = -sinf(a) * base.x + cosf(a) * base.z;
    } else if (fabsf(r->turn) > 0.02f) {
        const float cx = -r->lateral * 220.0f / r->turn;
        const float cz = r->forward * 220.0f / r->turn;
        const float dx = base.x - cx, dz = base.z - cz;
        const float radius = fmaxf(1, hypotf(dx, dz));
        const float a = atan2f(dz, dx) + copysignf(1, r->turn) * path.stroke_mm / radius;
        target.x = cx + radius * cosf(a);
        target.z = cz + radius * sinf(a);
    } else {
        target.x += path.stroke_mm * r->forward;
        target.z += path.stroke_mm * r->lateral;
    }
    target.y = path.lift_mm;
    return target;
}

bool robot_body_support_reference(const robot_planar_point_t feet[4],
                                   const bool contact[4], robot_planar_point_t reference,
                                   robot_planar_point_t *out)
{
    if (!feet || !contact || !out || !isfinite(reference.x) || !isfinite(reference.z)) return false;
    const int order[4] = {0, 1, 3, 2};
    robot_planar_point_t hull[4];
    int count = 0;
    for (int j = 0; j < 4; ++j) if (contact[order[j]]) {
        if (!isfinite(feet[order[j]].x) || !isfinite(feet[order[j]].z)) return false;
        hull[count++] = feet[order[j]];
    }
    if (!count) return false; // no constant-height support model in flight
    if (count == 1) { *out = hull[0]; return true; }
    float best_distance = INFINITY;
    bool positive = false, negative = false;
    for (int j = 0; j < count; ++j) {
        const robot_planar_point_t a = hull[j], edge = sub(hull[(j + 1) % count], a);
        const robot_planar_point_t relative = sub(reference, a);
        const float side = cross(edge, relative);
        positive |= side > 0.0001f; negative |= side < -0.0001f;
        const float length2 = dot(edge, edge);
        const float t = length2 > 0.0001f ? fminf(1, fmaxf(0, dot(relative, edge) / length2)) : 0;
        const robot_planar_point_t candidate = add(a, mul(edge, t));
        const robot_planar_point_t error = sub(reference, candidate);
        const float distance = dot(error, error);
        if (distance < best_distance) { best_distance = distance; *out = candidate; }
    }
    if (count > 2 && !(positive && negative)) *out = reference;
    return true;
}

// A first-order stable exponential response to a linearly varying input.
// divergent component is integrated BACKWARD; convergent component FORWARD.
static robot_planar_point_t propagate(robot_planar_point_t state,
                                      robot_planar_point_t p0, robot_planar_point_t p1,
                                      float a, bool backward)
{
    const float one_minus_r = -expm1f(-a), r = 1.0f - one_minus_r;
    const float ramp = backward ? one_minus_r / a - r : 1.0f - one_minus_r / a;
    return add(mul(state, r), add(mul(p0, one_minus_r), mul(sub(p1, p0), ramp)));
}

bool robot_body_trajectory_solve(robot_body_trajectory_t *p,
                                 const robot_planar_point_t support[N],
                                 float height, float period)
{
    if (!p) return false;
    p->valid = false;
    if (!support || !isfinite(height) || height < 50 || height > 500 ||
        !isfinite(period) || period < 0.25f || period > 10) return false;
    for (int i = 0; i < N; ++i) {
        if (!isfinite(support[i].x) || !isfinite(support[i].z)) return false;
        p->support[i] = support[i];
    }
    p->omega = sqrtf(GRAVITY_MM_S2 / height);
    p->dt = period / N;
    const float a = p->omega * p->dt;
    const float periodic_gain = 1.0f / -expm1f(-p->omega * period);
    robot_planar_point_t xi = {0}, eta = {0};
    for (int i = N - 1; i >= 0; --i)
        xi = propagate(xi, p->support[i], p->support[(i + 1) % N], a, true);
    xi = mul(xi, periodic_gain);
    for (int i = N - 1; i >= 0; --i) {
        xi = propagate(xi, p->support[i], p->support[(i + 1) % N], a, true);
        p->divergent[i] = xi;
    }
    for (int i = 0; i < N; ++i)
        eta = propagate(eta, p->support[i], p->support[(i + 1) % N], a, false);
    eta = mul(eta, periodic_gain);
    for (int i = 0; i < N; ++i) {
        p->convergent[i] = eta;
        eta = propagate(eta, p->support[i], p->support[(i + 1) % N], a, false);
    }
    p->valid = true;
    return true;
}

static bool same_request(const robot_body_trajectory_request_t *a,
                         const robot_body_trajectory_request_t *b)
{
    return a->gait == b->gait && a->profile.stride_mm == b->profile.stride_mm &&
        a->profile.step_height_mm == b->profile.step_height_mm &&
        a->profile.frequency_centi_hz == b->profile.frequency_centi_hz &&
        a->profile.duty_percent == b->profile.duty_percent &&
        a->geometry.frame_length_mm == b->geometry.frame_length_mm &&
        a->geometry.frame_width_mm == b->geometry.frame_width_mm &&
        a->geometry.corner_inset_mm == b->geometry.corner_inset_mm &&
        a->geometry.abduction_link_mm == b->geometry.abduction_link_mm &&
        a->com_height_mm == b->com_height_mm && a->lateral_stance_mm == b->lateral_stance_mm &&
        a->forward == b->forward && a->lateral == b->lateral && a->turn == b->turn &&
        a->stride_mm == b->stride_mm && a->stepping_in_place == b->stepping_in_place &&
        a->spin == b->spin && a->excluded_leg == b->excluded_leg;
}

bool robot_body_trajectory_build(robot_body_trajectory_t *p,
                                 const robot_body_trajectory_request_t *r)
{
    if (!p || !r || (r->gait == 5 && r->excluded_leg >= ROBOT_LEG_COUNT)) return false;
    if (p->request_cached && same_request(&p->request, r)) return p->valid;
    p->request = *r; p->request_cached = true; p->valid = false;
    if (!robot_locomotion_profile_valid(r->gait, &r->profile) ||
        // Below 50% duty diagonal gaits have unsupported flight phases.
        (!robot_locomotion_is_single_support_gait(r->gait) && r->profile.duty_percent < 50))
        return false;
    for (int i = 0; i < N; ++i) {
        const float phase = (float)i / N;
        robot_planar_point_t feet[4]; float availability[4]; bool contact[4];
        for (int leg = 0; leg < 4; ++leg) {
            const robot_vec3_t foot = robot_body_trajectory_foot(r, (robot_leg_t)leg, phase);
            const robot_locomotion_leg_phase_t state = r->gait == 5
                ? robot_locomotion_tripod_phase((robot_leg_t)leg, r->excluded_leg, phase, &r->profile)
                : robot_locomotion_leg_phase(r->gait, (robot_leg_t)leg, phase, &r->profile);
            feet[leg] = (robot_planar_point_t){foot.x, foot.z};
            contact[leg] = state.support_contact;
            availability[leg] = robot_predictive_support_availability(state.local_phase,
                robot_locomotion_duty_factor(&r->profile));
        }
        robot_planar_point_t vpsp;
        if (r->gait == 5) {
            // The quadruped VPSP's neighbour-pair formula assumes four legs.
            // For three legs use the centroid of scheduled support as ZMP,
            // then the same periodic LIPM preview; never count the held leg.
            vpsp = (robot_planar_point_t){0};
            unsigned n = 0;
            for (int leg=0; leg<4; ++leg) if (contact[leg]) {
                vpsp = add(vpsp, feet[leg]); ++n;
            }
            if (n < 2) return false;
            vpsp = mul(vpsp, 1.0f/n);
        } else if (!robot_predictive_support_target(feet, availability, &vpsp)) return false;
        if (!robot_body_support_reference(feet, contact, vpsp, &p->support[i])) return false;
    }
    return robot_body_trajectory_solve(p, p->support, r->com_height_mm,
        1.0f / robot_locomotion_frequency_hz(&r->profile));
}

robot_body_trajectory_sample_t robot_body_trajectory_sample(const robot_body_trajectory_t *p,
                                                            float phase)
{
    if (!p || !p->valid || !isfinite(phase)) return (robot_body_trajectory_sample_t){0};
    phase -= floorf(phase);
    const float index = phase * N;
    const int i = (int)index, next = (i + 1) % N;
    const float u = index - i;
    const robot_planar_point_t support = add(p->support[i], mul(sub(p->support[next], p->support[i]), u));
    const robot_planar_point_t eta = u > 0.000001f
        ? propagate(p->convergent[i], p->support[i], support, p->omega * p->dt * u, false)
        : p->convergent[i];
    const robot_planar_point_t xi = u < 0.999999f
        ? propagate(p->divergent[next], support, p->support[next], p->omega * p->dt * (1 - u), true)
        : p->divergent[next];
    const robot_planar_point_t position = mul(add(xi, eta), 0.5f);
    return (robot_body_trajectory_sample_t){
        .position_mm = position,
        .velocity_mm_s = mul(sub(xi, eta), p->omega * 0.5f),
        .acceleration_mm_s2 = mul(sub(position, support), p->omega * p->omega),
        .support_mm = support,
    };
}
