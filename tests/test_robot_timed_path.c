#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "robot_body_trajectory.h"
#include "joint_trajectory.h"

int main(void)
{
    // The exact function called by firmware and the Swift R1 cycle. Holding
    // R1 is handled by their edge detectors; a new press must reach mode 5.
    uint8_t mode=0;
    for (unsigned press=1;press<=18;++press) {
        mode=robot_locomotion_next_mode(mode);
        assert(mode==press%6);
        assert(__builtin_popcount(robot_locomotion_player_leds(mode))==mode);
    }
    assert(robot_locomotion_filter_command(0,1,0.02f)>0.099f);
    assert(robot_locomotion_filter_command(0,1,0.02f)<0.101f);
    const float first=robot_locomotion_filter_command(0,1,0.01f);
    assert(fabsf(robot_locomotion_filter_command(first,1,0.01f)-0.1f)<1e-6f);

    const joint_trajectory_limits_t limits={5.2175f,38.96f};
    for (int gait=1;gait<=5;++gait) for (int direction=0;direction<3;++direction) {
        robot_body_trajectory_request_t r={
            .gait=gait,.profile=robot_locomotion_default_profile(gait),
            .geometry=robot_kinematics_default_geometry(),.com_height_mm=225,
            .body_height_mm=250,.lateral_stance_mm=25,.excluded_leg=ROBOT_LEG_RIGHT_REAR,
            .forward=direction<2 ? 1 : 0,.turn=direction>0 ? 0.7f : 0,
            .joint_velocity_limit=limits.maximum_velocity,
            .joint_acceleration_limit=limits.maximum_acceleration,
        };
        r.stride_mm=r.profile.stride_mm;
        const robot_locomotion_profile_t saved=r.profile;
        robot_body_trajectory_t plan={0};
        assert(robot_body_trajectory_build(&plan,&r));
        assert(plan.effective_frequency_hz<=robot_locomotion_frequency_hz(&r.profile));
        assert(r.profile.frequency_centi_hz==saved.frequency_centi_hz);
        assert(plan.request.profile.frequency_centi_hz==saved.frequency_centi_hz);
        // Repeated input reuses the same validated plan, not a new phase.
        assert(robot_body_trajectory_build(&plan,&r));
        joint_trajectory_t tracking[4][3]={0};
        float worst_error=0, worst_ground=0;
        for (int i=0;i<1500;++i) {
            const float phase=i*0.02f*plan.effective_frequency_hz;
            const robot_body_trajectory_sample_t body=robot_body_trajectory_sample(&plan,phase);
            for (int leg=0;leg<4;++leg) {
                robot_vec3_t goal=robot_body_trajectory_foot(&r,leg,phase);
                goal.x-=body.position_mm.x; goal.z-=body.position_mm.z;
                float q[3], commanded[3];
                assert(robot_kinematics_inverse(&r.geometry,leg,goal,250,q));
                for (int axis=0;axis<3;++axis) {
                    const float previous_velocity=tracking[leg][axis].velocity;
                    commanded[axis]=joint_trajectory_track(&tracking[leg][axis],q[axis],0.02f,limits);
                    assert(fabsf(tracking[leg][axis].velocity)<=limits.maximum_velocity+1e-5f);
                    assert(fabsf(tracking[leg][axis].velocity-previous_velocity)<=limits.maximum_acceleration*0.02f+1e-5f);
                    if (i>500) worst_error=fmaxf(worst_error,fabsf(commanded[axis]-q[axis]));
                }
                if (i>500) {
                    const robot_vec3_t actual=robot_kinematics_forward(&r.geometry,leg,commanded,250);
                    worst_ground=fminf(worst_ground,actual.y);
                }
            }
        }
        printf("gait=%d direction=%d requested=%.2f effective=%.2f Hz max reference lag=%.2f deg floor=%.2f mm\n",
            gait,direction,robot_locomotion_frequency_hz(&r.profile),plan.effective_frequency_hz,
            worst_error*180/3.14159265f,worst_ground);
        assert(worst_error<0.02f); // ~1 degree, after initial acquisition
        assert(worst_ground>-1.0f);
    }
    puts("mode 0..5, LED counts and timed full-body IK path: passed");
}
