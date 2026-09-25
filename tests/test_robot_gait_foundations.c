#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "joint_trajectory.h"
#include "robot_gait_profile.h"
#include "robot_locomotion.h"
#include "robot_predictive_support.h"
#include "robot_gait_sync.h"
#include "robot_kinematics.h"
#include "robot_static_gait_profile.h"
#include "robot_static_balance.h"

#define PI_F 3.14159265358979323846f

static float distance(robot_vec3_t a, robot_vec3_t b)
{
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void test_inverse_forward_round_trip(void)
{
    const robot_geometry_t geometry = robot_kinematics_default_geometry();
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        const float knee_sign = (leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR) ? 1.0f : -1.0f;
        for (int i = 0; i < 7; ++i) {
            const float q[ROBOT_AXIS_COUNT] = {
                -0.42f + 0.14f * (float)i,
                -0.72f + 0.24f * (float)i,
                knee_sign * (0.42f + 0.13f * (float)i),
            };
            const robot_vec3_t target = robot_kinematics_forward(&geometry, (robot_leg_t)leg, q, 118.0f);
            float solved[ROBOT_AXIS_COUNT] = {0};
            assert(robot_kinematics_inverse(&geometry, (robot_leg_t)leg, target, 118.0f, solved));
            const robot_vec3_t reconstructed = robot_kinematics_forward(&geometry, (robot_leg_t)leg, solved, 118.0f);
            assert(distance(target, reconstructed) < 0.02f);
        }
        const robot_vec3_t neutral = robot_kinematics_neutral_foot(&geometry, (robot_leg_t)leg);
        float solved[ROBOT_AXIS_COUNT] = {0};
        assert(robot_kinematics_inverse(&geometry, (robot_leg_t)leg, neutral, 118.0f, solved));
        assert(distance(neutral, robot_kinematics_forward(&geometry, (robot_leg_t)leg, solved, 118.0f)) < 0.02f);
    }
}

static void test_foot_path_is_closed_and_lifted(void)
{
    const float duty = 0.5f, stride = 64.0f, height = 26.0f;
    const robot_foot_path_t start = robot_gait_foot_path(0.0f, duty, stride, height, true);
    const robot_foot_path_t liftoff = robot_gait_foot_path(duty, duty, stride, height, true);
    const robot_foot_path_t end = robot_gait_foot_path(0.999999f, duty, stride, height, true);
    const robot_foot_path_t mid_swing = robot_gait_foot_path(0.75f, duty, stride, height, true);
    assert(start.stance && fabsf(start.stroke_mm - stride * 0.5f) < 0.01f && start.lift_mm == 0.0f);
    assert(!liftoff.stance && fabsf(liftoff.stroke_mm + stride * 0.5f) < 0.01f && liftoff.lift_mm < 0.01f);
    assert(fabsf(end.stroke_mm - start.stroke_mm) < 0.01f && end.lift_mm < 0.01f);
    assert(!mid_swing.stance && mid_swing.lift_mm > height * 0.93f);
    const robot_foot_path_t early_swing = robot_gait_foot_path(0.60f, duty, stride, height, true);
    // The public MIT FootSwingTrajectory reaches its apex at the middle of
    // swing; before that it follows the first cubic Bézier half-arc.
    assert(!early_swing.stance && early_swing.lift_mm > height * 0.30f);
    const robot_foot_path_t idle = robot_gait_foot_path(0.25f, duty, stride, height, false);
    assert(idle.stance && idle.stroke_mm == 0.0f && idle.lift_mm == 0.0f);
}

static void test_mit_profile_scheduler_and_bezier_swing(void)
{
    for (uint8_t gait = 1; gait <= 4; ++gait) {
        const robot_locomotion_profile_t profile = robot_locomotion_default_profile(gait);
        assert(robot_locomotion_profile_valid(gait, &profile));
        assert(robot_locomotion_frequency_hz(&profile) > 0.0f);
        assert(robot_locomotion_duty_factor(&profile) > 0.0f);
    }
    robot_locomotion_profile_t invalid = robot_locomotion_default_profile(1);
    invalid.duty_percent = 20;
    assert(!robot_locomotion_profile_valid(1, &invalid));
    robot_locomotion_profile_t fastTrot = robot_locomotion_default_profile(1);
    fastTrot.stride_mm = 300;
    fastTrot.frequency_centi_hz = 300;
    fastTrot.step_height_mm = 100;
    assert(robot_locomotion_profile_valid(1, &fastTrot));
    fastTrot.stride_mm = 301;
    assert(!robot_locomotion_profile_valid(1, &fastTrot));
    fastTrot.stride_mm = 300;
    fastTrot.frequency_centi_hz = 301;
    assert(!robot_locomotion_profile_valid(1, &fastTrot));
    robot_locomotion_profile_t fastCrawl = robot_locomotion_default_profile(2);
    fastCrawl.frequency_centi_hz = 120;
    assert(robot_locomotion_profile_valid(2, &fastCrawl));
    fastCrawl.frequency_centi_hz = 121;
    assert(!robot_locomotion_profile_valid(2, &fastCrawl));

    // Crawl and Climb schedule at most one airborne foot. Trot and Run
    // schedule diagonal pairs and never leave all four without contact.
    for (int sample = 0; sample < 100; ++sample) {
        const float phase = (float)sample / 100.0f;
        for (uint8_t gait = 1; gait <= 4; ++gait) {
            const robot_locomotion_profile_t profile = robot_locomotion_default_profile(gait);
            int swings = 0;
            for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
                const robot_locomotion_leg_phase_t state =
                    robot_locomotion_leg_phase(gait, (robot_leg_t)leg, phase, &profile);
                assert(state.local_phase >= 0.0f && state.local_phase < 1.0f);
                assert(state.swing_phase >= 0.0f && state.swing_phase <= 1.0f);
                swings += state.scheduled_swing ? 1 : 0;
            }
            if (robot_locomotion_is_single_support_gait(gait)) assert(swings <= 1);
            else assert(swings <= 2);
        }
    }

    const robot_vec3_t p0 = {.x = -32, .y = 0, .z = 14};
    const robot_vec3_t pf = {.x = 32, .y = 0, .z = -14};
    const robot_vec3_t start = robot_locomotion_swing_bezier(p0, pf, 0.0f, 26.0f);
    const robot_vec3_t beforeApex = robot_locomotion_swing_bezier(p0, pf, 0.42f, 26.0f);
    const robot_vec3_t apex = robot_locomotion_swing_bezier(p0, pf, 0.50f, 26.0f);
    const robot_vec3_t finish = robot_locomotion_swing_bezier(p0, pf, 1.0f, 26.0f);
    assert(distance(start, p0) < 0.001f);
    assert(distance(finish, pf) < 0.001f);
    assert(beforeApex.y > 0.0f && beforeApex.y < 26.0f);
    assert(fabsf(apex.y - 26.0f) < 0.001f);
}

static void test_virtual_predictive_support_polygon(void)
{
    const robot_planar_point_t feet[ROBOT_LEG_COUNT] = {
        {.x = 180.0f, .z = 125.0f}, {.x = 180.0f, .z = -125.0f},
        {.x = -180.0f, .z = 125.0f}, {.x = -180.0f, .z = -125.0f},
    };
    const float full[ROBOT_LEG_COUNT] = {1, 1, 1, 1};
    robot_planar_point_t target = {0};
    assert(robot_predictive_support_target(feet, full, &target));
    assert(fabsf(target.x) < 0.001f && fabsf(target.z) < 0.001f);

    // The paper's phase weights are continuous: a planted foot is trusted at
    // middle stance, and a swing foot is least trusted at middle swing.
    const float duty = 0.58f;
    const float stance_edge = robot_predictive_support_availability(0.0f, duty);
    const float stance_middle = robot_predictive_support_availability(duty * 0.5f, duty);
    const float swing_middle = robot_predictive_support_availability(
        duty + (1.0f - duty) * 0.5f, duty);
    assert(stance_edge > 0.45f && stance_edge < 0.55f);
    assert(stance_middle > 0.9f);
    assert(swing_middle < 0.1f);

    // A known trot schedule must never create an undefined target at contact
    // transitions; the virtual polygon includes scheduled touchdown feet.
    const robot_locomotion_profile_t trot = robot_locomotion_default_profile(1);
    for (int sample = 0; sample < 100; ++sample) {
        float availability[ROBOT_LEG_COUNT] = {0};
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
            const robot_locomotion_leg_phase_t phase =
                robot_locomotion_leg_phase(1, (robot_leg_t)leg,
                                           (float)sample / 100.0f, &trot);
            availability[leg] = robot_predictive_support_availability(
                phase.local_phase, robot_locomotion_duty_factor(&trot));
        }
        assert(robot_predictive_support_target(feet, availability, &target));
        assert(isfinite(target.x) && isfinite(target.z));
    }

    // Aura's servo/leg order is LF, RF, LR, RR, while the physical perimeter
    // is LF, RF, RR, LR. VPSP's virtual neighbours must follow that perimeter
    // or a partial contact transition can point the torso toward a diagonal
    // swing pair. This asymmetric case locks the physical ordering down.
    const robot_planar_point_t staggered[ROBOT_LEG_COUNT] = {
        {.x = 220.0f, .z = 125.0f}, {.x = 140.0f, .z = -125.0f},
        {.x = -220.0f, .z = 125.0f}, {.x = -140.0f, .z = -125.0f},
    };
    const float transition_weight[ROBOT_LEG_COUNT] = {0.8f, 0.3f, 0.2f, 0.6f};
    assert(robot_predictive_support_target(staggered, transition_weight, &target));
    assert(fabsf(target.x - 41.16824f) < 0.01f);
    assert(fabsf(target.z - 3.49907f) < 0.01f);
}

static void test_phase_retiming(void)
{
    assert(fabsf(robot_gait_phase_rate(0.0f, false, 0.0f) - 1.0f) < 1e-6f);
    assert(robot_gait_phase_rate(10.0f * PI_F / 180.0f, false, 0.0f) == 0.0f);
    assert(robot_gait_phase_rate(0.0f, true, 20.0f * PI_F / 180.0f) == 0.0f);
    const float reduced = robot_gait_phase_rate(5.5f * PI_F / 180.0f, true, 8.0f * PI_F / 180.0f);
    assert(reduced > 0.0f && reduced < 1.0f);
}

static void test_single_foot_contact_configures_real_slot_timing(void)
{
    const robot_locomotion_profile_t crawl = robot_locomotion_default_profile(2);
    assert(crawl.duty_percent == 82);
    assert(robot_locomotion_profile_valid(2, &crawl));
    robot_locomotion_profile_t unsafe = crawl;
    unsafe.duty_percent = 75;
    assert(!robot_locomotion_profile_valid(2, &unsafe));

    // With 82% contact, one leg is airborne for 18% of the full cycle, or
    // 72% of its own slot. The preceding 28% is the CoM preload.
    const float preload = 1.0f - 4.0f * (1.0f - (float)crawl.duty_percent / 100.0f);
    assert(fabsf(preload - 0.28f) < 0.001f);
    const robot_static_step_profile_t before = robot_static_step_profile_timed(0.27f, preload);
    const robot_static_step_profile_t airborne = robot_static_step_profile_timed(0.29f, preload);
    assert(!before.airborne && before.body_shift_fraction > 0.99f);
    assert(airborne.airborne && airborne.lift_fraction > 0.0f);
    assert(fabsf(robot_static_next_transition_timed(0.0f, preload) - preload) < 0.001f);
}

static void test_static_step_is_a_single_smooth_arch(void)
{
    const robot_static_step_profile_t preload = robot_static_step_profile(0.12f);
    const robot_static_step_profile_t rise = robot_static_step_profile(0.36f);
    const robot_static_step_profile_t crest = robot_static_step_profile(0.625f);
    const robot_static_step_profile_t fall = robot_static_step_profile(0.90f);
    const robot_static_step_profile_t settle = robot_static_step_profile(0.99f);

    assert(preload.stroke_fraction == -0.5f && preload.lift_fraction == 0.0f && !preload.airborne);
    assert(rise.stroke_fraction > -0.5f && rise.stroke_fraction < 0.0f &&
           rise.lift_fraction > 0.0f && rise.airborne);
    assert(crest.stroke_fraction > -0.01f && crest.stroke_fraction < 0.01f &&
           crest.lift_fraction > 0.99f && crest.airborne);
    assert(fall.stroke_fraction > 0.0f && fall.stroke_fraction < 0.5f &&
           fall.lift_fraction > 0.0f && fall.airborne);
    assert(settle.stroke_fraction > 0.49f && settle.lift_fraction > 0.0f && settle.airborne);

    for (int index = 0; index <= 1000; ++index) {
        const float phase = (float)index / 1000.0f;
        const robot_static_step_profile_t sample = robot_static_step_profile(phase);
        assert(sample.lift_fraction >= 0.0f && sample.lift_fraction <= 1.0f);
        assert(sample.stroke_fraction >= -0.5f && sample.stroke_fraction <= 0.5f);
        if (sample.lift_fraction > 0.001f) assert(sample.airborne);
    }
}

static void test_static_stance_closes_at_liftoff_and_touchdown(void)
{
    const float justBefore = robot_static_stance_stroke_fraction(0.999999f);
    const float atLift = robot_static_stance_stroke_fraction(0.0f);
    const float touchdown = robot_static_stance_stroke_fraction(0.98f * 0.25f);
    assert(fabsf(justBefore + 0.5f) < 0.01f);
    assert(fabsf(atLift + 0.5f) < 0.01f);
    assert(fabsf(touchdown - 0.5f) < 0.01f);
}

static void test_static_step_transitions_are_ordered(void)
{
    assert(fabsf(robot_static_next_transition(0.00f) - 0.25f) < 1e-6f);
    assert(fabsf(robot_static_next_transition(0.25f) - 1.00f) < 1e-6f);
    assert(fabsf(robot_static_next_transition(0.80f) - 1.00f) < 1e-6f);
}

static void test_static_support_polygon_keeps_com_inside_three_feet(void)
{
    const robot_planar_point_t feet[ROBOT_LEG_COUNT] = {
        // LF, RF, LR, RR. Values match Aura's default 360 × 130 mm frame
        // with the configured 35 mm ab/ad link and 25 mm stance spread.
        {.x = 180.0f, .z = 125.0f}, {.x = 180.0f, .z = -125.0f},
        {.x = -180.0f, .z = 125.0f}, {.x = -180.0f, .z = -125.0f},
    };
    const robot_planar_point_t origin = {0};
    for (int excluded = 0; excluded < ROBOT_LEG_COUNT; ++excluded) {
        robot_planar_point_t support[3] = {0};
        int count = 0;
        for (int foot = 0; foot < ROBOT_LEG_COUNT; ++foot)
            if (foot != excluded) support[count++] = feet[foot];
        assert(count == 3);
        // The neutral CoM is on the diagonal of every three-foot triangle;
        // lifting a leg without a pre-load is therefore not statically safe.
        assert(fabsf(robot_static_support_margin(support, origin)) < 0.01f);

        robot_static_support_plan_t plan = {0};
        assert(robot_static_support_plan(support, origin, 8.0f, &plan));
        assert(plan.valid);
        assert(plan.margin_mm >= 7.98f);
        assert(plan.maximum_margin_mm >= plan.margin_mm);
        // The safe translation is away from the foot about to swing and into
        // the triangle of the three feet that remain on the floor.
        assert(plan.com_target.x * feet[excluded].x < 0.0f);
        assert(plan.com_target.z * feet[excluded].z < 0.0f);
    }
}

static void test_static_support_plan_accounts_for_actual_com_offset(void)
{
    const robot_planar_point_t support[3] = {
        {.x = 180.0f, .z = -125.0f}, {.x = -180.0f, .z = 125.0f},
        {.x = -180.0f, .z = -125.0f},
    };
    const robot_planar_point_t actual_com = {.x = 38.0f, .z = -19.0f};
    robot_static_support_plan_t plan = {0};
    assert(robot_static_support_plan(support, actual_com, 15.0f, &plan));
    // The physical torso translation is target CoM minus the fixed offset.
    const robot_planar_point_t torso_translation = {
        .x = plan.com_target.x - actual_com.x,
        .z = plan.com_target.z - actual_com.z,
    };
    const robot_planar_point_t shifted_com = {
        .x = torso_translation.x + actual_com.x,
        .z = torso_translation.z + actual_com.z,
    };
    assert(fabsf(shifted_com.x - plan.com_target.x) < 0.001f);
    assert(fabsf(shifted_com.z - plan.com_target.z) < 0.001f);
    assert(robot_static_support_margin(support, shifted_com) >= 14.98f);
}

static void test_static_support_rejects_invalid_or_impossible_triangle(void)
{
    const robot_planar_point_t flat[3] = {{0, 0}, {40, 0}, {80, 0}};
    robot_static_support_plan_t plan = {0};
    assert(!robot_static_support_plan(flat, (robot_planar_point_t){0}, 1.0f, &plan));

    const robot_planar_point_t small[3] = {{0, 0}, {20, 0}, {0, 20}};
    assert(!robot_static_support_plan(small, (robot_planar_point_t){0}, 20.0f, &plan));
}

static void test_virtual_servo_recovers_after_backlog(void)
{
    joint_trajectory_t axis = {0};
    joint_trajectory_reset(&axis, 0.0f);
    const joint_trajectory_limits_t limits = {.maximum_velocity = 2.0f, .maximum_acceleration = 12.0f};
    float phase = 0.0f;
    float last_phase = 0.0f;
    bool held = false;
    for (int tick = 0; tick < 1600; ++tick) {
        const robot_foot_path_t foot = robot_gait_foot_path(phase, 0.5f, 58.0f, 20.0f, true);
        const float reference = 0.8f * foot.stroke_mm / 58.0f;
        const float command = joint_trajectory_step(&axis, reference, 0.02f, limits);
        const float rate = robot_gait_phase_rate(fabsf(reference - command), true, 0.0f);
        if (rate < 0.99f) held = true;
        phase += 0.85f * rate * 0.02f;
        phase -= floorf(phase);
        assert(isfinite(phase) && isfinite(command));
        const float forward = fmodf(phase - last_phase + 1.0f, 1.0f);
        assert(forward < 0.05f);
        last_phase = phase;
    }
    assert(held);
    assert(fabsf(axis.position) < 0.85f);
}


int main(void)
{
    test_inverse_forward_round_trip();
    test_foot_path_is_closed_and_lifted();
    test_mit_profile_scheduler_and_bezier_swing();
    test_virtual_predictive_support_polygon();
    test_phase_retiming();
    test_single_foot_contact_configures_real_slot_timing();
    test_static_step_is_a_single_smooth_arch();
    test_static_stance_closes_at_liftoff_and_touchdown();
    test_static_step_transitions_are_ordered();
    test_static_support_polygon_keeps_com_inside_three_feet();
    test_static_support_plan_accounts_for_actual_com_offset();
    test_static_support_rejects_invalid_or_impossible_triangle();
    test_virtual_servo_recovers_after_backlog();
    puts("robot gait foundations: passed");
    return 0;
}
