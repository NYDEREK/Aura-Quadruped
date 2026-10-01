#include <math.h>

#include "robot_com_estimate.h"

bool robot_com_estimate(const robot_vec3_t feet[4], const float weight[4],
                        robot_planar_point_t *out)
{
    if (!feet || !weight || !out) return false;
    float sum = 0, x = 0, z = 0, largest = 0;
    for (int leg = 0; leg < 4; ++leg) {
        const float w = fabsf(weight[leg]);
        if (!isfinite(w) || !isfinite(feet[leg].x) || !isfinite(feet[leg].z)) return false;
        sum += w; x += w * feet[leg].x; z += w * feet[leg].z;
        largest = fmaxf(largest, w);
    }
    // Every foot must carry something; a lifted or unread leg would bias the
    // centroid toward the others, so such a sample is rejected, not used.
    for (int leg = 0; leg < 4; ++leg)
        if (fabsf(weight[leg]) < 0.05f * largest) return false;
    if (sum <= 0) return false;
    *out = (robot_planar_point_t){.x = x / sum, .z = z / sum};
    return true;
}
