#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "robot_body_trajectory.h"
#include "robot_gait_profile.h"

#define PI 3.14159265358979323846f
#define N ROBOT_BODY_TRAJECTORY_SAMPLES

static float distance(robot_planar_point_t a, robot_planar_point_t b)
{ return hypotf(a.x - b.x, a.z - b.z); }

static void test_periodic_dynamics_and_preview(void)
{
    robot_body_trajectory_t plan = {0};
    robot_planar_point_t support[N];
    for (int i = 0; i < N; ++i)
        support[i] = (robot_planar_point_t){40 * sinf(2 * PI * i / N), -23};
    assert(robot_body_trajectory_solve(&plan, support, 225, 1.0f));
    const float attenuation = 1 / (1 + 225.0f / 9810.0f * 4 * PI * PI);
    for (int i = 0; i < 1000; ++i) {
        const float t = (float)i / 1000;
        const robot_body_trajectory_sample_t s = robot_body_trajectory_sample(&plan, t);
        assert(fabsf(s.position_mm.x - attenuation * 40 * sinf(2 * PI * t)) < 0.05f);
        assert(fabsf(s.position_mm.z + 23) < 0.002f);
        // Independently differentiate positions; don't just check the
        // acceleration field against the same equation that produced it.
        const float dt = 0.002f;
        const robot_body_trajectory_sample_t before = robot_body_trajectory_sample(&plan, t - dt);
        const robot_body_trajectory_sample_t after = robot_body_trajectory_sample(&plan, t + dt);
        const float v = (after.position_mm.x - before.position_mm.x) / (2 * dt);
        const float acc = (after.velocity_mm_s.x - before.velocity_mm_s.x) / (2 * dt);
        assert(fabsf(v - s.velocity_mm_s.x) < 0.1f);
        assert(fabsf(s.position_mm.x - 225 / 9810.0f * acc - s.support_mm.x) < 0.08f);
    }
    // Anticipate an upcoming support transfer instead of following it late.
    for (int i = 0; i < N; ++i) support[i] = (robot_planar_point_t){i < N/2 ? -30 : 30, 0};
    assert(robot_body_trajectory_solve(&plan, support, 225, 1));
    assert(robot_body_trajectory_sample(&plan, 0.43f).position_mm.x > -20);
    const robot_body_trajectory_sample_t a = robot_body_trajectory_sample(&plan, 0);
    const robot_body_trajectory_sample_t b = robot_body_trajectory_sample(&plan, 1);
    assert(distance(a.position_mm, b.position_mm) < 0.001f);
    assert(distance(a.velocity_mm_s, b.velocity_mm_s) < 0.001f);
}

static void test_contact_geometry(void)
{
    const robot_planar_point_t feet[4] = {{180,125},{180,-125},{-180,125},{-180,-125}};
    const bool diagonal[4] = {true,false,false,true};
    robot_planar_point_t reference;
    assert(robot_body_support_reference(feet, diagonal, (robot_planar_point_t){20,60}, &reference));
    assert(fabsf(reference.z * 180 - reference.x * 125) < 0.01f);
    const bool all[4] = {true,true,true,true};
    assert(robot_body_support_reference(feet, all, (robot_planar_point_t){20,60}, &reference));
    assert(distance(reference, (robot_planar_point_t){20,60}) < 0.001f);
    const bool none[4] = {false};
    assert(!robot_body_support_reference(feet, none, (robot_planar_point_t){0}, &reference));
}

static void test_planted_foot_and_swing_velocity(void)
{
    const float stride = 120, duty = 0.58f, period = 1 / 1.4f;
    const float speed = stride / (duty * period), epsilon = 0.0001f;
    for (int i = 0; i < 50; ++i) {
        const float phase = duty * i / 50;
        const robot_foot_path_t a = robot_gait_foot_path(phase, duty, stride, 60, true);
        assert(fabsf(a.stroke_mm + speed * phase * period - stride/2) < 0.001f);
        assert(a.lift_mm == 0);
    }
    const float transitions[2] = {0, duty};
    for (int i = 0; i < 2; ++i) {
        const float phase = transitions[i];
        const robot_foot_path_t a = robot_gait_foot_path(phase-epsilon, duty, stride, 60, true);
        const robot_foot_path_t b = robot_gait_foot_path(phase, duty, stride, 60, true);
        const robot_foot_path_t c = robot_gait_foot_path(phase+epsilon, duty, stride, 60, true);
        assert(fabsf((b.stroke_mm-a.stroke_mm)/(epsilon*period) + speed) < 2);
        assert(fabsf((c.stroke_mm-b.stroke_mm)/(epsilon*period) + speed) < 2);
    }
}

static void test_trot_cycle(void)
{
    robot_body_trajectory_t plan = {0};
    robot_body_trajectory_request_t r = {
        .gait=1, .profile=robot_locomotion_default_profile(1),
        .geometry=robot_kinematics_default_geometry(), .com_height_mm=225,
        .lateral_stance_mm=25, .forward=1, .stride_mm=120,
    };
    r.profile.stride_mm=120;
    assert(robot_body_trajectory_build(&plan, &r));
    double old_squared_error=0, new_squared_error=0;
    for (int i = 0; i < N; ++i) {
        const float phase=(float)i/N;
        robot_planar_point_t feet[4]; bool contact[4]; float weights[4];
        for (int leg=0; leg<4; ++leg) {
            const robot_vec3_t f=robot_body_trajectory_foot(&r,(robot_leg_t)leg,phase);
            feet[leg]=(robot_planar_point_t){f.x,f.z};
            const robot_locomotion_leg_phase_t p=robot_locomotion_leg_phase(1,(robot_leg_t)leg,phase,&r.profile);
            contact[leg]=p.support_contact;
            weights[leg]=robot_predictive_support_availability(p.local_phase,0.58f);
        }
        const float dp=0.0005f, dt=dp/robot_locomotion_frequency_hz(&r.profile);
        const robot_body_trajectory_sample_t s=robot_body_trajectory_sample(&plan,phase);
        const robot_body_trajectory_sample_t a=robot_body_trajectory_sample(&plan,phase-dp);
        const robot_body_trajectory_sample_t b=robot_body_trajectory_sample(&plan,phase+dp);
        robot_planar_point_t inferred={
            s.position_mm.x - 225/9810.0f*(b.velocity_mm_s.x-a.velocity_mm_s.x)/(2*dt),
            s.position_mm.z - 225/9810.0f*(b.velocity_mm_s.z-a.velocity_mm_s.z)/(2*dt)};
        robot_planar_point_t projected;
        assert(robot_body_support_reference(feet,contact,inferred,&projected));
        const float error=distance(inferred,projected);
        assert(error < 0.6f); // mm, interpolation/differentiation near a switch
        new_squared_error+=error*error;
        // Baseline: assigning a varying support reference as CoM itself
        // ignores acceleration and produces a different required ZMP.
        const float sample_dt=plan.dt;
        const robot_planar_point_t current=plan.support[i];
        const robot_planar_point_t prev=plan.support[(i+N-1)%N], next=plan.support[(i+1)%N];
        const robot_planar_point_t old_zmp={
            current.x-225/9810.0f*(prev.x-2*current.x+next.x)/(sample_dt*sample_dt),
            current.z-225/9810.0f*(prev.z-2*current.z+next.z)/(sample_dt*sample_dt)};
        assert(robot_body_support_reference(feet,contact,old_zmp,&projected));
        old_squared_error+=pow(distance(old_zmp,projected),2);
        // Full body -> leg IK -> FK loop: preserve the foot lift rather
        // than clipping an unreachable foot silently in this nominal test.
        for (int leg=0; leg<4; ++leg) {
            robot_vec3_t target=robot_body_trajectory_foot(&r,(robot_leg_t)leg,phase);
            target.x-=s.position_mm.x; target.z-=s.position_mm.z;
            float q[3];
            assert(robot_kinematics_inverse(&r.geometry,(robot_leg_t)leg,target,250,q));
            const robot_vec3_t actual=robot_kinematics_forward(&r.geometry,(robot_leg_t)leg,q,250);
            assert(fabsf(actual.y-target.y)<0.03f);
        }
    }
    printf("Trot reduced-model ZMP error RMS: direct target %.2f mm, preview %.3f mm\n",
           sqrt(old_squared_error/N),sqrt(new_squared_error/N));
    r.profile.duty_percent=45;
    assert(!robot_body_trajectory_build(&plan,&r));
    assert(!plan.valid);
    r.profile.duty_percent=58; r.forward=-1;
    assert(robot_body_trajectory_build(&plan,&r));
    r.forward=0; r.lateral=1;
    assert(robot_body_trajectory_build(&plan,&r));
    r.forward=1; r.lateral=0; r.turn=0.5f;
    assert(robot_body_trajectory_build(&plan,&r));
}

int main(void)
{
    test_periodic_dynamics_and_preview();
    test_contact_geometry();
    test_planted_foot_and_swing_velocity();
    test_trot_cycle();
    puts("body trajectory: passed");
}
