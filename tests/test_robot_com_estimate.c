#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_com_estimate.h"

int main(void)
{
    const robot_vec3_t feet[4] = {{180, 0, 100}, {180, 0, -100}, {-180, 0, 100}, {-180, 0, -100}};
    robot_planar_point_t c;
    // Equal loads: CoM at the centre of the stance.
    assert(robot_com_estimate(feet, (const float[4]){1, 1, 1, 1}, &c));
    assert(fabsf(c.x) < 1e-3f && fabsf(c.z) < 1e-3f);
    // Rear feet carry 60 %: moment balance puts the CoM 36 mm behind centre.
    assert(robot_com_estimate(feet, (const float[4]){0.4f, 0.4f, 0.6f, 0.6f}, &c));
    assert(fabsf(c.x + 36.0f) < 0.01f && fabsf(c.z) < 1e-3f);
    // Sign of the reported load does not matter (mirrored servos).
    assert(robot_com_estimate(feet, (const float[4]){-0.4f, 0.4f, -0.6f, 0.6f}, &c));
    assert(fabsf(c.x + 36.0f) < 0.01f);
    // A leg without load (lifted / no feedback) rejects the sample.
    assert(!robot_com_estimate(feet, (const float[4]){1, 1, 1, 0}, &c));
    puts("CoM estimate: passed");
    return 0;
}
