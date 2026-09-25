#include <math.h>
#include <string.h>

#include "robot_static_balance.h"

#define TRIANGLE_EPSILON_MM 0.001f
#define PLAN_EPSILON_MM 0.02f
#define PROJECTION_PASSES 12

typedef struct {
    robot_planar_point_t inward_normal;
    float length;
    robot_planar_point_t start;
} support_edge_t;

static float cross(robot_planar_point_t a, robot_planar_point_t b)
{
    return a.x * b.z - a.z * b.x;
}

static robot_planar_point_t subtract(robot_planar_point_t a, robot_planar_point_t b)
{
    return (robot_planar_point_t){.x = a.x - b.x, .z = a.z - b.z};
}

static float length(robot_planar_point_t vector)
{
    return hypotf(vector.x, vector.z);
}

static bool make_edges(const robot_planar_point_t support[3], support_edge_t edges[3],
                       float *signed_double_area)
{
    if (!support || !edges) return false;
    const float area2 = cross(subtract(support[1], support[0]),
                              subtract(support[2], support[0]));
    if (!isfinite(area2) || fabsf(area2) < TRIANGLE_EPSILON_MM) return false;
    const float winding = area2 > 0.0f ? 1.0f : -1.0f;
    for (int edge = 0; edge < 3; ++edge) {
        const robot_planar_point_t start = support[edge];
        const robot_planar_point_t end = support[(edge + 1) % 3];
        const robot_planar_point_t vector = subtract(end, start);
        const float edge_length = length(vector);
        if (!isfinite(edge_length) || edge_length < TRIANGLE_EPSILON_MM) return false;
        // For a counter-clockwise triangle, (-dz, dx) points inward. Reverse
        // it for clockwise support-point order so every edge uses one common
        // positive-inside convention.
        edges[edge] = (support_edge_t){
            .start = start,
            .length = edge_length,
            .inward_normal = {
                .x = winding * -vector.z / edge_length,
                .z = winding * vector.x / edge_length,
            },
        };
    }
    if (signed_double_area) *signed_double_area = area2;
    return true;
}

static float edge_distance(const support_edge_t *edge, robot_planar_point_t point)
{
    const robot_planar_point_t relative = subtract(point, edge->start);
    return relative.x * edge->inward_normal.x + relative.z * edge->inward_normal.z;
}

float robot_static_support_margin(const robot_planar_point_t support[3],
                                  robot_planar_point_t point)
{
    support_edge_t edges[3] = {0};
    if (!isfinite(point.x) || !isfinite(point.z) || !make_edges(support, edges, NULL))
        return -INFINITY;
    float margin = INFINITY;
    for (int edge = 0; edge < 3; ++edge)
        margin = fminf(margin, edge_distance(&edges[edge], point));
    return margin;
}

bool robot_static_support_plan(const robot_planar_point_t support[3],
                               robot_planar_point_t current_com,
                               float minimum_margin_mm,
                               robot_static_support_plan_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!isfinite(current_com.x) || !isfinite(current_com.z) ||
        !isfinite(minimum_margin_mm) || minimum_margin_mm < 0.0f)
        return false;

    support_edge_t edges[3] = {0};
    float area2 = 0.0f;
    if (!make_edges(support, edges, &area2)) return false;

    const float perimeter = edges[0].length + edges[1].length + edges[2].length;
    const float inradius = fabsf(area2) / perimeter;
    if (!isfinite(inradius) || minimum_margin_mm > inradius + PLAN_EPSILON_MM)
        return false;

    // Project the current CoM into the intersection of the three inward-shifted
    // edge half-planes. Sequential projections converge for this convex set;
    // choosing the nearest feasible point avoids an unnecessarily aggressive
    // body roll/translation before a single-foot transfer.
    robot_planar_point_t target = current_com;
    for (int pass = 0; pass < PROJECTION_PASSES; ++pass) {
        bool adjusted = false;
        for (int edge = 0; edge < 3; ++edge) {
            const float distance = edge_distance(&edges[edge], target);
            if (distance < minimum_margin_mm) {
                const float correction = minimum_margin_mm - distance;
                target.x += edges[edge].inward_normal.x * correction;
                target.z += edges[edge].inward_normal.z * correction;
                adjusted = true;
            }
        }
        if (!adjusted) break;
    }

    const float margin = robot_static_support_margin(support, target);
    if (!isfinite(margin) || margin + PLAN_EPSILON_MM < minimum_margin_mm)
        return false;

    out->valid = true;
    out->com_target = target;
    out->margin_mm = margin;
    out->maximum_margin_mm = inradius;
    return true;
}

bool robot_support_line_projection(const robot_planar_point_t support[2],
                                   robot_planar_point_t current_com,
                                   robot_planar_point_t *out_projection,
                                   float *out_lateral_error_mm)
{
    if (!support || !out_projection || !isfinite(current_com.x) ||
        !isfinite(current_com.z))
        return false;
    const robot_planar_point_t line = subtract(support[1], support[0]);
    const float length_squared = line.x * line.x + line.z * line.z;
    if (!isfinite(length_squared) || length_squared < TRIANGLE_EPSILON_MM)
        return false;
    // Do not clamp t to the segment: the dynamic planner must remove only
    // the perpendicular gravity lever arm.  Correcting along the line would
    // cancel the planned stance stroke and make the robot stop walking.
    const robot_planar_point_t relative = subtract(current_com, support[0]);
    const float t = (relative.x * line.x + relative.z * line.z) / length_squared;
    const robot_planar_point_t projection = {
        .x = support[0].x + line.x * t,
        .z = support[0].z + line.z * t,
    };
    if (!isfinite(projection.x) || !isfinite(projection.z)) return false;
    *out_projection = projection;
    if (out_lateral_error_mm)
        *out_lateral_error_mm = length(subtract(current_com, projection));
    return true;
}
