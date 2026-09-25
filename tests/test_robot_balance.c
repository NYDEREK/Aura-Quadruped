#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "robot_balance.h"
#include "robot_body_trajectory.h"

#define PI 3.14159265358979323846f
static float error(robot_vec3_t a, robot_vec3_t b)
{ return sqrtf((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }

static void test_swing_clearance(void)
{
    const robot_geometry_t g=robot_kinematics_default_geometry();
    const robot_vec3_t world[4]={{180,0,135},{180,0,-135},{-180,0,135},{-180,0,-135}};
    const robot_balance_pose_t nominal={.position={0,250,0}};
    unsigned legacy_collisions=0;
    // Both diagonals, both signs, roll AND pitch. Build a physical rigid pose
    // and encoder positions, then reconstruct it from only support feet.
    for(int diagonal=0; diagonal<2; ++diagonal)
    for(int ir=-1;ir<=1;ir+=2) for(int ip=-1;ip<=1;ip+=2) {
        robot_balance_pose_t actual={.position={9,242,-6}, .roll=ir*8*PI/180, .pitch=ip*7*PI/180};
        robot_vec3_t measured[4]; bool contacts[4];
        for(int leg=0;leg<4;++leg) {
            contacts[leg]=((leg==0||leg==3)?0:1)==diagonal;
            robot_vec3_t p=robot_balance_to_body(world[leg],actual);
            float q[3]; assert(robot_kinematics_inverse(&g,(robot_leg_t)leg,p,0,q));
            measured[leg]=robot_kinematics_forward(&g,(robot_leg_t)leg,q,0);
        }
        robot_balance_pose_t estimated;
        assert(robot_balance_estimate_pose(measured,world,contacts,actual.roll,actual.pitch,&estimated));
        assert(error(actual.position,estimated.position)<.001f);
        for(int leg=0;leg<4;++leg) if(!contacts[leg]) {
            // Deliberately small clearance to expose the tilted-frame bug.
            robot_vec3_t foot=world[leg]; foot.y=12;
            const robot_vec3_t old=robot_balance_to_world(robot_balance_to_body(foot,nominal),actual);
            if(old.y<0) ++legacy_collisions;
            const robot_vec3_t corrected=robot_balance_swing_target(foot,estimated);
            float q[3]; assert(robot_kinematics_inverse(&g,(robot_leg_t)leg,corrected,0,q));
            const robot_vec3_t reached=robot_balance_to_world(robot_kinematics_forward(&g,(robot_leg_t)leg,q,0),actual);
            assert(error(reached,foot)<.01f); // includes horizontal drag/yaw source
            // Check the complete swing, including the low-clearance start
            // and end. Mixing desired and actual transforms would fail here.
            for(int sample=0;sample<=100;++sample) {
                float u=(float)sample/100;
                foot.y=40*sinf(PI*u)*sinf(PI*u);
                robot_vec3_t p=robot_balance_swing_target(foot,estimated);
                assert(error(robot_balance_to_world(p,actual),foot)<.001f);
            }
        }
        // Airborne positions must have no influence on the base estimate.
        for(int leg=0;leg<4;++leg) if(!contacts[leg]) measured[leg]=(robot_vec3_t){NAN,NAN,NAN};
        assert(robot_balance_estimate_pose(measured,world,contacts,actual.roll,actual.pitch,&estimated));
        assert(error(actual.position,estimated.position)<.001f);
    }
    assert(legacy_collisions>0);
    const bool absent[4]={0}; robot_balance_pose_t dummy;
    assert(!robot_balance_estimate_pose(world,world,absent,0,0,&dummy));
    printf("swing: recovered world path through FK/IK; old transform penetrated in %u cases\n",legacy_collisions);
}
static void test_tripod(void)
{
    for(int excluded=0;excluded<4;++excluded) {
        robot_body_trajectory_t plan={0};
        robot_body_trajectory_request_t r={.gait=5,.profile=robot_locomotion_default_profile(5),
            .geometry=robot_kinematics_default_geometry(),.excluded_leg=(robot_leg_t)excluded,
            .com_height_mm=225,.lateral_stance_mm=25,.forward=1,.stride_mm=35};
        assert(robot_body_trajectory_build(&plan,&r));
        for(int i=0;i<600;++i) {
            float phase=(float)i/600;
            const robot_body_trajectory_sample_t body=robot_body_trajectory_sample(&plan,phase);
            unsigned support=0;
            for(int leg=0;leg<4;++leg) {
                const robot_locomotion_leg_phase_t state=robot_locomotion_tripod_phase((robot_leg_t)leg,
                    (robot_leg_t)excluded,phase,&r.profile);
                support+=state.support_contact;
                robot_vec3_t foot=robot_body_trajectory_foot(&r,(robot_leg_t)leg,phase);
                if(leg==excluded) { assert(!state.support_contact); assert(foot.y>=40); }
                else if(state.support_contact) assert(fabsf(foot.y)<.001f);
                foot.x-=body.position_mm.x;foot.y-=250;foot.z-=body.position_mm.z;
                float q[3]; assert(robot_kinematics_inverse(&r.geometry,(robot_leg_t)leg,foot,0,q));
                assert(error(foot,robot_kinematics_forward(&r.geometry,(robot_leg_t)leg,q,0))<.01f);
            }
            assert(support>=2 && support<=3);
        }
        // Boundary continuity matters most when a supporting foot becomes
        // airborne. Inspect all three schedules, not just evenly spaced samples.
        for(int leg=0;leg<4;++leg) {
            const robot_vec3_t start=robot_body_trajectory_foot(&r,(robot_leg_t)leg,0);
            const robot_vec3_t end=robot_body_trajectory_foot(&r,(robot_leg_t)leg,1);
            assert(error(start,end)<.001f);
            for(int slot=0;slot<3;++slot) {
                const float boundaries[2]={1-(float)slot/3,
                    (float)r.profile.duty_percent/100-(float)slot/3};
                for(int b=0;b<2;++b) {
                    const robot_vec3_t before=robot_body_trajectory_foot(&r,(robot_leg_t)leg,boundaries[b]-1e-5f);
                    const robot_vec3_t after=robot_body_trajectory_foot(&r,(robot_leg_t)leg,boundaries[b]+1e-5f);
                    assert(error(before,after)<.02f);
                }
            }
        }
        // Invalid excluded index and too little stance overlap are rejected.
        r.excluded_leg=(robot_leg_t)4; assert(!robot_body_trajectory_build(&plan,&r));
    }
    robot_locomotion_profile_t p=robot_locomotion_default_profile(5);
    p.duty_percent=60; assert(!robot_locomotion_profile_valid(5,&p));
}
static void test_angular_feedback(void)
{
    float p=robot_balance_posture_target(-.1f,.5f,.45f,.08f,.35f);
    assert(p<-.045f); // already falling: damping must strengthen, not reverse correction
    assert(robot_balance_posture_target(-.1f,-.5f,.45f,.08f,.35f)>p);
    assert(robot_balance_posture_target(-1,2,.45f,.08f,.35f)==-.35f);
    assert(robot_balance_posture_target(.1f,2,1,0,.35f)==.1f); // standing unchanged
}
int main(void){test_swing_clearance();test_tripod();test_angular_feedback();puts("robot balance: passed");}
