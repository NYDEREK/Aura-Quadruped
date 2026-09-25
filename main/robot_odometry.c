#include <math.h>
#include <string.h>

#include "robot_odometry.h"

static robot_vec3_t world_point(const robot_odometry_t *odometry, robot_vec3_t local)
{
    const float c = cosf(odometry->yaw_radians), s = sinf(odometry->yaw_radians);
    return (robot_vec3_t){
        .x = odometry->x_mm + c * local.x - s * local.z,
        .y = local.y,
        .z = odometry->z_mm + s * local.x + c * local.z,
    };
}

void robot_odometry_reset(robot_odometry_t *odometry)
{
    if (odometry) memset(odometry, 0, sizeof(*odometry));
}

void robot_odometry_update(robot_odometry_t *odometry,
                           const robot_vec3_t feet_body_mm[ROBOT_LEG_COUNT],
                           const bool contact[ROBOT_LEG_COUNT], bool solve_yaw)
{
    if (!odometry || !feet_body_mm || !contact) return;
    if (!odometry->valid) odometry->valid = true;

    unsigned count = 0;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        if (!contact[leg]) {
            odometry->has_planted_foot[leg] = false;
            continue;
        }
        if (!odometry->has_planted_foot[leg]) {
            odometry->planted_feet[leg] = world_point(odometry, feet_body_mm[leg]);
            odometry->has_planted_foot[leg] = true;
        }
        ++count;
    }
    if (!count) return;

    if (count == 1) {
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
            if (!contact[leg]) continue;
            const robot_vec3_t foot = feet_body_mm[leg], anchor = odometry->planted_feet[leg];
            const float c = cosf(odometry->yaw_radians), s = sinf(odometry->yaw_radians);
            odometry->x_mm = anchor.x - c * foot.x + s * foot.z;
            odometry->z_mm = anchor.z - s * foot.x - c * foot.z;
            return;
        }
    }

    float local_x = 0.0f, local_z = 0.0f, anchor_x = 0.0f, anchor_z = 0.0f;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        if (!contact[leg]) continue;
        local_x += feet_body_mm[leg].x;
        local_z += feet_body_mm[leg].z;
        anchor_x += odometry->planted_feet[leg].x;
        anchor_z += odometry->planted_feet[leg].z;
    }
    local_x /= (float)count; local_z /= (float)count;
    anchor_x /= (float)count; anchor_z /= (float)count;

    float dot = 0.0f, cross = 0.0f;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        if (!contact[leg]) continue;
        const float px = feet_body_mm[leg].x - local_x;
        const float pz = feet_body_mm[leg].z - local_z;
        const float ax = odometry->planted_feet[leg].x - anchor_x;
        const float az = odometry->planted_feet[leg].z - anchor_z;
        dot += px * ax + pz * az;
        cross += px * az - pz * ax;
    }
    // A straight or lateral gait has no commanded body yaw. Keeping the
    // previous orientation in those modes prevents unequal encoder latency
    // from becoming a fictitious turn. Yaw is solved only for an actual turn.
    if (solve_yaw && dot * dot + cross * cross > 1e-6f)
        odometry->yaw_radians = atan2f(cross, dot);
    const float c = cosf(odometry->yaw_radians), s = sinf(odometry->yaw_radians);
    odometry->x_mm = anchor_x - c * local_x + s * local_z;
    odometry->z_mm = anchor_z - s * local_x - c * local_z;
}
