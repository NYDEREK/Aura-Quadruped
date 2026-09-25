#include <math.h>

#include "robot_kinematics.h"

#define PI_F 3.14159265358979323846f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static bool left_leg(robot_leg_t leg)
{
    return leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR;
}

static bool front_leg(robot_leg_t leg)
{
    return leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_RIGHT_FRONT;
}

static float frame_angle(robot_leg_t leg)
{
    return left_leg(leg) ? PI_F / 2.0f : -PI_F / 2.0f;
}

robot_geometry_t robot_kinematics_default_geometry(void)
{
    return (robot_geometry_t){
        // Physical centre-to-centre dimensions of Aura's frame and leg.
        // Servo-case width and any construction clearance do not enter IK.
        .frame_length_mm = 360.0f, .frame_width_mm = 130.0f, .corner_inset_mm = 0.0f,
        .abduction_link_mm = 35.0f, .upper_leg_mm = 130.0f, .lower_leg_mm = 205.0f,
    };
}

robot_vec3_t robot_kinematics_leg_anchor(const robot_geometry_t *geometry, robot_leg_t leg,
                                          float body_height_mm)
{
    const float x = (front_leg(leg) ? 1.0f : -1.0f) *
                    (geometry->frame_length_mm / 2.0f - geometry->corner_inset_mm);
    const float z = (left_leg(leg) ? 1.0f : -1.0f) *
                    (geometry->frame_width_mm / 2.0f - geometry->corner_inset_mm);
    return (robot_vec3_t){.x = x, .y = body_height_mm, .z = z};
}

robot_vec3_t robot_kinematics_neutral_foot(const robot_geometry_t *geometry, robot_leg_t leg)
{
    robot_vec3_t anchor = robot_kinematics_leg_anchor(geometry, leg, 0.0f);
    anchor.z += left_leg(leg) ? geometry->abduction_link_mm : -geometry->abduction_link_mm;
    return anchor;
}

bool robot_kinematics_inverse(const robot_geometry_t *geometry, robot_leg_t leg,
                              robot_vec3_t target, float body_height_mm,
                              float out[ROBOT_AXIS_COUNT])
{
    if (!geometry || !out || !isfinite(target.x) || !isfinite(target.y) || !isfinite(target.z) ||
        geometry->abduction_link_mm <= 0 || geometry->upper_leg_mm <= 0 || geometry->lower_leg_mm <= 0)
        return false;
    const robot_vec3_t anchor = robot_kinematics_leg_anchor(geometry, leg, body_height_mm);
    const float a = frame_angle(leg), dx = target.x - anchor.x, dy = target.y - anchor.y,
                dz = target.z - anchor.z;
    const float local_x = cosf(a) * dx - sinf(a) * dz;
    const float local_y = dy;
    const float local_z = sinf(a) * dx + cosf(a) * dz;
    const float planar_sq = local_x * local_x + local_y * local_y -
                            geometry->abduction_link_mm * geometry->abduction_link_mm;
    if (planar_sq < -0.01f) return false;
    const float planar = sqrtf(fmaxf(0.0f, planar_sq));
    const float cosine_raw = (local_x * local_x + local_y * local_y + local_z * local_z -
                              geometry->abduction_link_mm * geometry->abduction_link_mm -
                              geometry->upper_leg_mm * geometry->upper_leg_mm -
                              geometry->lower_leg_mm * geometry->lower_leg_mm) /
                             (2.0f * geometry->upper_leg_mm * geometry->lower_leg_mm);
    if (cosine_raw < -1.0001f || cosine_raw > 1.0001f) return false;
    const float cosine = clampf(cosine_raw, -1.0f, 1.0f);
    const float knee = left_leg(leg) ? atan2f(sqrtf(fmaxf(0.0f, 1.0f - cosine * cosine)), cosine)
                                      : atan2f(-sqrtf(fmaxf(0.0f, 1.0f - cosine * cosine)), cosine);
    out[ROBOT_AXIS_ABDUCTION] = atan2f(local_y, local_x) + atan2f(planar, -geometry->abduction_link_mm);
    out[ROBOT_AXIS_HIP] = atan2f(local_z, planar) - atan2f(geometry->lower_leg_mm * sinf(knee),
                                                             geometry->upper_leg_mm + geometry->lower_leg_mm * cosf(knee));
    out[ROBOT_AXIS_KNEE] = knee;
    return true;
}

robot_vec3_t robot_kinematics_forward(const robot_geometry_t *geometry, robot_leg_t leg,
                                      const float q[ROBOT_AXIS_COUNT], float body_height_mm)
{
    if (!geometry || !q) return (robot_vec3_t){0};
    const float abduction = q[ROBOT_AXIS_ABDUCTION], hip = q[ROBOT_AXIS_HIP],
                knee = q[ROBOT_AXIS_KNEE], total = hip + knee;
    const robot_vec3_t anchor = robot_kinematics_leg_anchor(geometry, leg, body_height_mm);
    const robot_vec3_t p2 = {
        .x = -geometry->abduction_link_mm * cosf(abduction),
        .y = -geometry->abduction_link_mm * sinf(abduction), .z = 0,
    };
    const robot_vec3_t p3 = {
        .x = p2.x + geometry->upper_leg_mm * sinf(abduction) * cosf(hip),
        .y = p2.y - geometry->upper_leg_mm * cosf(abduction) * cosf(hip),
        .z = geometry->upper_leg_mm * sinf(hip),
    };
    const robot_vec3_t p4 = {
        .x = p3.x + geometry->lower_leg_mm * sinf(abduction) * cosf(total),
        .y = p3.y - geometry->lower_leg_mm * cosf(abduction) * cosf(total),
        .z = p3.z + geometry->lower_leg_mm * sinf(total),
    };
    const float a = frame_angle(leg);
    return (robot_vec3_t){
        .x = anchor.x + cosf(a) * p4.x + sinf(a) * p4.z,
        .y = anchor.y + p4.y,
        .z = anchor.z - sinf(a) * p4.x + cosf(a) * p4.z,
    };
}
