#include <math.h>

#include "robot_predictive_support.h"

#define VPSP_SQRT_TWO 1.4142135623730950488f
#define VPSP_SIGMA 4.0f
#define VPSP_EPSILON 0.0001f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float wrap01(float phase)
{
    phase = fmodf(phase, 1.0f);
    return phase < 0.0f ? phase + 1.0f : phase;
}

// Equation (10)'s smooth contact confidence: 0.5 at a switch and nearly 1
// at the middle of stance.  Its complement is the swing confidence used by
// equation (12); it falls to nearly zero at middle swing.
static float middle_phase_weight(float normalized_phase)
{
    const float phase = clampf(normalized_phase, 0.0f, 1.0f);
    const float left = erff(VPSP_SIGMA * phase / VPSP_SQRT_TWO);
    const float right = erff(VPSP_SIGMA * (1.0f - phase) / VPSP_SQRT_TWO);
    return clampf(0.5f * (left + right), 0.0f, 1.0f);
}

float robot_predictive_support_availability(float local_phase, float duty_factor)
{
    const float phase = wrap01(local_phase);
    const float duty = clampf(duty_factor, 0.01f, 0.99f);
    if (phase < duty) return middle_phase_weight(phase / duty);
    return 1.0f - middle_phase_weight((phase - duty) / (1.0f - duty));
}

static robot_planar_point_t interpolate(robot_planar_point_t from,
                                        robot_planar_point_t to, float to_weight)
{
    const float weight = clampf(to_weight, 0.0f, 1.0f);
    return (robot_planar_point_t){
        .x = from.x + (to.x - from.x) * weight,
        .z = from.z + (to.z - from.z) * weight,
    };
}

bool robot_predictive_support_target(const robot_planar_point_t feet[4],
                                     const float availability[4],
                                     robot_planar_point_t *out_target)
{
    if (!feet || !availability || !out_target) return false;
    // Aura stores legs as LF, RF, LR, RR.  That is useful for servo IDs but
    // it is *not* a perimeter order: LR and RR are reversed in the final two
    // array slots.  VPSP equations (13)–(15) require physical neighbours on
    // the support polygon.  Treating RF and LR as neighbours connects a
    // diagonal, which can pull the CoM toward the pair that is in swing.
    static const int perimeter_order[4] = {
        0, // LF
        1, // RF
        3, // RR
        2, // LR
    };
    robot_planar_point_t average = {0};
    for (int index = 0; index < 4; ++index) {
        const int leg = perimeter_order[index];
        if (!isfinite(feet[leg].x) || !isfinite(feet[leg].z) ||
            !isfinite(availability[leg]))
            return false;
        const int previous = perimeter_order[(index + 3) & 3];
        const int next = perimeter_order[(index + 1) & 3];
        const float phi = clampf(availability[leg], 0.0f, 1.0f);
        const float phi_previous = clampf(availability[previous], 0.0f, 1.0f);
        const float phi_next = clampf(availability[next], 0.0f, 1.0f);

        // Equation (13): virtual points slide along the lines from this foot
        // to its clockwise and counter-clockwise neighbours.
        const robot_planar_point_t virtual_previous = interpolate(feet[previous], feet[leg], phi);
        const robot_planar_point_t virtual_next = interpolate(feet[next], feet[leg], phi);
        const float total = phi + phi_previous + phi_next;
        if (total < VPSP_EPSILON) return false;

        // Equation (14): local predictive support vertex for this leg.
        const robot_planar_point_t vertex = {
            .x = (phi * feet[leg].x + phi_previous * virtual_previous.x +
                  phi_next * virtual_next.x) / total,
            .z = (phi * feet[leg].z + phi_previous * virtual_previous.z +
                  phi_next * virtual_next.z) / total,
        };
        average.x += vertex.x;
        average.z += vertex.z;
    }

    // Equation (15): desired CoM is the mean of the four virtual vertices.
    out_target->x = average.x * 0.25f;
    out_target->z = average.z * 0.25f;
    return isfinite(out_target->x) && isfinite(out_target->z);
}
