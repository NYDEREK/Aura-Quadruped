#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_gait_feasibility.h"

// Mode 4 limits in robot_gait.c (ROBOT_TROT_FEASIBLE_*).
#define SPEED 3.4f
#define ACCELERATION 110.0f

static robot_gait_feasibility_t check(uint8_t gait, robot_locomotion_profile_t profile)
{
    static robot_body_trajectory_t scratch;
    const robot_geometry_t geometry = robot_kinematics_default_geometry();
    robot_gait_feasibility_t f;
    assert(robot_gait_feasibility(gait, &profile, &geometry, 233.0f, 25.0f, SPEED, ACCELERATION,
                                  &scratch, &f));
    assert(f.valid && f.rate > 0.0f && f.rate <= 1.0f);
    assert(!scratch.valid && !scratch.request_cached);   // the caller's plan rebuilds
    return f;
}

int main(void)
{
    // Mode 3's Run profile walks on Aura: well inside the limits.
    const robot_gait_feasibility_t run = check(3, (robot_locomotion_profile_t){44, 41, 100, 52});
    assert(run.rate == 1.0f && run.peak_speed_rad_s < 2.6f && run.two_foot_s > 0.4f);

    // The Trot profile saved on 11.10 needs ~255 deg/s: it runs slower.
    const robot_gait_feasibility_t old = check(1, (robot_locomotion_profile_t){60, 54, 125, 58});
    assert(old.rate < 0.85f && old.peak_speed_rad_s > SPEED);
    assert(fabsf(old.rate - SPEED / old.peak_speed_rad_s) < 1e-4f ||
           fabsf(old.rate - sqrtf(ACCELERATION / old.peak_acceleration_rad_s2)) < 1e-4f);

    // The short low step for a balanced trot: 0.2 s on two feet at full tempo.
    const robot_gait_feasibility_t trot = check(1, (robot_locomotion_profile_t){40, 20, 150, 70});
    assert(trot.rate == 1.0f && fabsf(trot.two_foot_s - 0.20f) < 0.02f);

    // Invalid input is refused, not guessed.
    static robot_body_trajectory_t scratch;
    const robot_geometry_t geometry = robot_kinematics_default_geometry();
    robot_gait_feasibility_t f;
    const robot_locomotion_profile_t bad = {40, 20, 150, 99};
    assert(!robot_gait_feasibility(1, &bad, &geometry, 233.0f, 25.0f, SPEED, ACCELERATION, &scratch, &f));
    assert(!f.valid);

    printf("run %.0f deg/s, old trot %.0f deg/s -> rate %.2f, new trot %.0f deg/s %.0f rad/s^2\n",
           run.peak_speed_rad_s * 57.3f, old.peak_speed_rad_s * 57.3f, old.rate,
           trot.peak_speed_rad_s * 57.3f, trot.peak_acceleration_rad_s2);
    puts("robot gait feasibility tests passed");
    return 0;
}
