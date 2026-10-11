#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_state_estimator.h"

// Synthetic walk: diagonal pairs at 1 Hz / 52 % duty, planted feet fixed in
// the world, leg kinematics seen with 2 mm noise, a gyro with a 1 deg/s
// bias, and an accelerometer with gait-like noise.

#define PI 3.14159265f
#define DT 0.02f

static unsigned seed = 1;
static float accel_bias_y;   // mm/s^2 on the vertical accelerometer reading
static float noise(float sigma)
{
    seed = seed * 1103515245u + 12345u;
    return sigma * (((float)((seed >> 9) & 0xffff) / 65535.0f) * 2.0f - 1.0f);
}

typedef struct {
    float x, z, heading;          // true body pose
    robot_vec3_t foot[4];          // true world feet
    float phase;
} truth_t;

static const robot_vec3_t hip[4] = {{180, 0, 125}, {180, 0, -125}, {-180, 0, 125}, {-180, 0, -125}};

static robot_vec3_t to_body(const truth_t *t, robot_vec3_t w)
{
    const float dx = w.x - t->x, dz = w.z - t->z, c = cosf(t->heading), s = sinf(t->heading);
    return (robot_vec3_t){c * dx + s * dz, w.y - 233.0f, -s * dx + c * dz};
}

static robot_vec3_t nominal_world(const truth_t *t, int leg, float ahead)
{
    const float c = cosf(t->heading), s = sinf(t->heading);
    const float hx = hip[leg].x + ahead, hz = hip[leg].z;
    return (robot_vec3_t){t->x + c * hx - s * hz, 0, t->z + s * hx + c * hz};
}

// One tick: move the body, plant or swing feet, and build the sensor input.
static robot_estimator_input_t tick(truth_t *t, float speed, float turn_rate, bool armed, bool walking)
{
    t->heading += turn_rate * DT;
    t->x += cosf(t->heading) * speed * DT;
    t->z += sinf(t->heading) * speed * DT;
    if (walking) t->phase = fmodf(t->phase + DT, 1.0f);
    robot_estimator_input_t in = {.dt = DT, .armed = armed, .walking = walking, .commanded = walking,
                                  .use_accel = true, .accel_norm_g = 1.0f};
    for (int leg = 0; leg < 4; ++leg) {
        const float offset = (leg == 0 || leg == 3) ? 0.0f : 0.5f;
        const float local = fmodf(t->phase + offset, 1.0f);
        const bool stance = !walking || local < 0.52f;
        if (!stance) {
            // Swing: land half a stride ahead of the hip, 30 mm up mid-swing.
            const float u = (local - 0.52f) / 0.48f;
            const robot_vec3_t target = nominal_world(t, leg, speed * 0.52f / 2.0f);
            t->foot[leg].x += (target.x - t->foot[leg].x) * fminf(1.0f, DT / fmaxf(DT, (1 - u) * 0.48f));
            t->foot[leg].z += (target.z - t->foot[leg].z) * fminf(1.0f, DT / fmaxf(DT, (1 - u) * 0.48f));
            t->foot[leg].y = 30.0f * sinf(PI * u);
        } else {
            t->foot[leg].y = 0;
        }
        const float s = local / 0.52f;
        in.trust[leg] = stance ? (walking ? fminf(1.0f, fminf(s, 1 - s) / 0.15f) : 1.0f) : 0.0f;
        const robot_vec3_t body = to_body(t, t->foot[leg]);
        in.feet_body_mm[leg] = (robot_vec3_t){body.x + noise(2), body.y + noise(2), body.z + noise(2)};
        in.feet_valid[leg] = true;
    }
    in.yaw_rate_rad_s = turn_rate + 1.0f * PI / 180.0f + noise(0.3f * PI / 180.0f);
    in.accel_mm_s2 = (robot_vec3_t){noise(400), 9810.0f + accel_bias_y + noise(400), noise(400)};
    return in;
}

static void start(truth_t *t, robot_estimator_t *e)
{
    *t = (truth_t){0};
    for (int leg = 0; leg < 4; ++leg) t->foot[leg] = nominal_world(t, leg, 0);
    robot_estimator_reset(e);
}

int main(void)
{
    static robot_estimator_t e;
    robot_estimate_t out;
    truth_t t;

    // 1. Armed and still for 10 s: the gyro bias is learned, nothing drifts.
    start(&t, &e);
    for (int i = 0; i < 500; ++i) { robot_estimator_input_t in = tick(&t, 0, 0, true, false); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    printf("  still 10 s: bias %.2f deg/s, heading %.2f deg, drift %.1f mm\n",
           out.gyro_bias_rad_s * 180 / PI, out.heading_rad * 180 / PI, hypotf(out.position_mm.x, out.position_mm.z));
    assert(out.still && fabsf(out.gyro_bias_rad_s * 180 / PI - 1.0f) < 0.2f);
    assert(hypotf(out.position_mm.x, out.position_mm.z) < 5.0f);

    // 2. Straight walk, 150 mm/s for 20 s.
    for (int i = 0; i < 1000; ++i) { robot_estimator_input_t in = tick(&t, 150, 0, true, true); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    const float travelled = hypotf(t.x, t.z), error = hypotf(out.position_mm.x - t.x, out.position_mm.z - t.z);
    printf("  straight %.0f mm: error %.0f mm (%.1f %%), speed %.0f mm/s\n", travelled, error,
           100 * error / travelled, out.forward_mm_s);
    assert(error < 0.05f * travelled);
    assert(fabsf(out.forward_mm_s - 150) < 30);

    // 3. A 90 deg left arc in 10 s at 120 mm/s.
    for (int i = 0; i < 500; ++i) { robot_estimator_input_t in = tick(&t, 120, PI / 2 / 10, true, true); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    const float arc_error = hypotf(out.position_mm.x - t.x, out.position_mm.z - t.z);
    printf("  after arc: heading %.1f deg (true %.1f), position error %.0f mm\n",
           out.heading_rad * 180 / PI, t.heading * 180 / PI, arc_error);
    assert(fabsf(out.heading_rad - t.heading) < 3.0f * PI / 180);
    assert(arc_error < 0.06f * (travelled + 1200));
    assert(out.position_mm.z > 400);           // turned left = +Z

    // 4. Disarmed: the position is held while the legs are limp.
    const robot_vec3_t held = out.position_mm;
    for (int i = 0; i < 200; ++i) { robot_estimator_input_t in = tick(&t, 0, 0, false, false); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    assert(fabsf(out.position_mm.x - held.x) < 1e-3f && fabsf(out.position_mm.z - held.z) < 1e-3f);

    // 5. Re-zero keeps the bias.
    const float bias = out.gyro_bias_rad_s;
    robot_estimator_rezero(&e);
    robot_estimator_get(&e, &out);
    assert(out.position_mm.x == 0 && out.heading_rad == 0 && out.gyro_bias_rad_s == bias);
    // 6. A vertical accelerometer error (0.16 g, as Aura's IMU had before
    // its gravity was normalised) while walking: the planted feet hold the
    // ground at 0 and the torso at its height instead of drifting upward.
    start(&t, &e);
    accel_bias_y = 1600.0f;
    for (int i = 0; i < 100; ++i) { robot_estimator_input_t in = tick(&t, 0, 0, true, false); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    const float stand = out.position_mm.y;
    for (int i = 0; i < 1000; ++i) { robot_estimator_input_t in = tick(&t, 150, 0, true, true); robot_estimator_step(&e, &in); }
    robot_estimator_get(&e, &out);
    const float biased_error = hypotf(out.position_mm.x - t.x, out.position_mm.z - t.z);
    printf("  vertical accel error: ground %.1f mm, torso %+.1f mm from start, position error %.0f mm\n",
           out.ground_mm, out.position_mm.y - stand, biased_error);
    assert(fabsf(out.ground_mm) < 25.0f);
    assert(fabsf(out.position_mm.y - stand) < 30.0f);
    assert(biased_error < 0.05f * hypotf(t.x, t.z));
    accel_bias_y = 0;

    puts("state estimator tests passed");
    return 0;
}
