#include <math.h>
#include "robot_balance.h"

static float clamp(float x, float lo, float hi)
{ return fminf(hi, fmaxf(lo, x)); }

robot_vec3_t robot_balance_to_world(robot_vec3_t p, robot_balance_pose_t pose)
{
    const float cr = cosf(pose.roll), sr = sinf(pose.roll);
    const float cp = cosf(pose.pitch), sp = sinf(pose.pitch);
    const float y = cr*p.y - sr*p.z;
    return (robot_vec3_t){cp*p.x - sp*y + pose.position.x,
                         sp*p.x + cp*y + pose.position.y,
                         sr*p.y + cr*p.z + pose.position.z};
}

robot_vec3_t robot_balance_to_body(robot_vec3_t p, robot_balance_pose_t pose)
{
    p.x -= pose.position.x; p.y -= pose.position.y; p.z -= pose.position.z;
    const float cr = cosf(pose.roll), sr = sinf(pose.roll);
    const float cp = cosf(pose.pitch), sp = sinf(pose.pitch);
    const float y = -sp*p.x + cp*p.y;
    return (robot_vec3_t){cp*p.x + sp*p.y, cr*y + sr*p.z, -sr*y + cr*p.z};
}

bool robot_balance_estimate_pose(const robot_vec3_t body[4],
                                const robot_vec3_t world[4], const bool support[4],
                                float roll, float pitch, robot_balance_pose_t *pose)
{
    if (!body || !world || !support || !pose || !isfinite(roll) || !isfinite(pitch)) return false;
    robot_balance_pose_t result = {.roll=roll, .pitch=pitch};
    robot_vec3_t sum = {0};
    unsigned count = 0;
    for (int leg = 0; leg < 4; ++leg) if (support[leg]) {
        robot_vec3_t p = robot_balance_to_world(body[leg], result);
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) ||
            !isfinite(world[leg].x) || !isfinite(world[leg].y) || !isfinite(world[leg].z)) return false;
        sum.x += world[leg].x - p.x; sum.y += world[leg].y - p.y; sum.z += world[leg].z - p.z;
        ++count;
    }
    if (count < 2) return false;
    result.position = (robot_vec3_t){sum.x/count, sum.y/count, sum.z/count};
    *pose = result;
    return true;
}

robot_vec3_t robot_balance_swing_target(robot_vec3_t world, robot_balance_pose_t measured)
{
    return robot_balance_to_body(world, measured);
}

float robot_balance_posture_target(float error, float rate, float kp, float kd, float limit)
{
    if (!isfinite(error) || !isfinite(rate)) return 0;
    return clamp(kp*error - kd*rate, -limit, limit);
}
