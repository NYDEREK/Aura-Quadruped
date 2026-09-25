#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_foot_placement.h"

#define G 9810.0f

static int near(float a, float b, float tolerance) { return fabsf(a - b) <= tolerance; }

static void test_signs(void)
{
    // Left side up (MPU roll > 0) throws the CoM to the right (-Z).
    robot_planar_point_t v = robot_foot_placement_tipping_velocity(1.0f, 0.0f, 0.0f, 0.0f, 200.0f);
    assert(near(v.x, 0.0f, 1e-4f) && near(v.z, -200.0f, 1e-3f));
    // Nose down (pitch > 0) throws it forward (+X).
    v = robot_foot_placement_tipping_velocity(0.0f, 0.5f, 0.0f, 0.0f, 200.0f);
    assert(near(v.x, 100.0f, 1e-3f) && near(v.z, 0.0f, 1e-4f));
    // A rotation commanded by the attitude loop is not tipping.
    v = robot_foot_placement_tipping_velocity(0.3f, -0.2f, 0.3f, -0.2f, 200.0f);
    assert(near(v.x, 0.0f, 1e-4f) && near(v.z, 0.0f, 1e-4f));
    const robot_planar_point_t d = robot_foot_placement_tipping_displacement(0.1f, -0.1f, 200.0f);
    assert(d.z < 0.0f && d.x < 0.0f);
}

static void test_equation_six(void)
{
    robot_foot_placement_t p = {0};
    robot_foot_placement_config_t c = robot_foot_placement_default_config();
    c.deadband_mm_s = 0.0f;
    c.limit_mm = 1000.0f;
    robot_planar_point_t out = {0};
    for (int i = 0; i < 200; ++i)
        out = robot_foot_placement_update(&p, &c, (robot_planar_point_t){0},
                                          (robot_planar_point_t){100.0f, -50.0f}, 200.0f, 0.02f);
    const float k = sqrtf(200.0f / G);
    assert(near(out.x, 100.0f * k, 0.01f) && near(out.z, -50.0f * k, 0.01f));
    // The bound holds.
    c.limit_mm = 5.0f;
    out = robot_foot_placement_update(&p, &c, (robot_planar_point_t){80, 0},
                                      (robot_planar_point_t){100.0f, -50.0f}, 200.0f, 0.02f);
    assert(near(out.x, 5.0f, 1e-4f) && out.z >= -5.0f);
}

static void test_no_slip_and_continuity(void)
{
    robot_foot_placement_t p = {0};
    const robot_vec3_t planned = {100, 0, 50};
    const robot_planar_point_t live = {20, -10};
    // Stance before any swing: untouched.
    robot_vec3_t f = robot_foot_placement_apply(&p, ROBOT_LEG_LEFT_FRONT, planned, false, 0, live);
    assert(f.x == planned.x && f.z == planned.z);
    // Liftoff starts at the offset the foot stood on (zero) -> continuous.
    f = robot_foot_placement_apply(&p, ROBOT_LEG_LEFT_FRONT, planned, true, 0.0f, live);
    assert(near(f.x, 100, 1e-4f) && near(f.z, 50, 1e-4f));
    float previous = f.x;
    for (int i = 1; i <= 20; ++i) {
        f = robot_foot_placement_apply(&p, ROBOT_LEG_LEFT_FRONT, planned, true, i / 20.0f, live);
        assert(f.x >= previous - 1e-4f && f.x - previous < 3.1f);
        previous = f.x;
    }
    assert(near(f.x, 120, 1e-3f) && near(f.z, 40, 1e-3f));
    // Touchdown: the stance foot keeps the landed offset although the live
    // offset keeps changing -> a planted foot never slides.
    for (int i = 0; i < 10; ++i) {
        f = robot_foot_placement_apply(&p, ROBOT_LEG_LEFT_FRONT, planned, false, 0,
                                       (robot_planar_point_t){-30.0f * i, 15.0f * i});
        assert(near(f.x, 120, 1e-3f) && near(f.z, 40, 1e-3f));
    }
}

// Lateral linear inverted pendulum on alternating diagonal supports, the
// reduced model of a trot during its two-leg phase. The legs are attached to
// the hips, so a touchdown lands at (hip + offset): pure translation drift is
// carried by the leg itself; the tilt part is added in robot_gait.c.
// Returns the CoM speed after twelve steps (mm/s). Without feedback a small
// sideways push diverges; stepping on the capture point stops it.
static float simulate_trot_push(int with_feedback, float step_time, float push)
{
    const float h = 200.0f, omega = sqrtf(G / h), dt = 0.0005f;
    float c = 0.0f, v = push, support = 0.0f;
    robot_foot_placement_t p = {0};
    robot_foot_placement_config_t config = robot_foot_placement_default_config();
    config.filter_time_s = 0.0f;
    for (int step = 0; step < 12; ++step) {
        for (float t = 0; t < step_time; t += dt) {
            v += omega * omega * (c - support) * dt;
            c += v * dt;
            if (fabsf(c - support) > 300.0f) return 1e6f; // fell over
        }
        const robot_planar_point_t offset = robot_foot_placement_update(&p, &config,
            (robot_planar_point_t){0}, (robot_planar_point_t){0, v}, h, step_time);
        support = c + (with_feedback ? offset.z : 0.0f);
    }
    return fabsf(v);
}

int main(void)
{
    test_signs();
    test_equation_six();
    test_no_slip_and_continuity();
    const float open_loop = simulate_trot_push(0, 0.25f, 50.0f);
    const float closed_loop = simulate_trot_push(1, 0.25f, 50.0f);
    printf("sideways push 50 mm/s, two-leg phase 0.25 s: open loop %s, capture point |v| = %.1f mm/s\n",
           open_loop > 1e5f ? "falls" : "stands", closed_loop);
    const float phases[] = {0.15f, 0.25f, 0.35f, 0.60f};
    for (int i = 0; i < 4; ++i) {
        const float r = simulate_trot_push(1, phases[i], 120.0f);
        printf("  push 120 mm/s, two-leg phase %.2f s: %s\n", phases[i],
               r > 1e5f ? "falls (offset limit reached)" : "recovers");
    }
    fflush(stdout);
    assert(open_loop > 1e5f);
    assert(closed_loop < 25.0f); // bounded by the gyro deadband, no fall
    assert(simulate_trot_push(1, 0.60f, 120.0f) > 1e5f); // why the swing must be short
    puts("foot placement: passed");
    return 0;
}
