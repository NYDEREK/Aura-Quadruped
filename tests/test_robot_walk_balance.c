#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "robot_walk_balance.h"

// Unit checks of the balance module, then a reduced simulation of pad mode 3
// (diagonal pairs, 1.0 Hz, 52 % duty) on the linear inverted pendulum:
// c'' = w^2 (c - p), p = ZMP inside the scheduled support (Kajita 2003).
// On two feet p lies on their segment, so nothing the legs do can push the
// CoM back across that line; it is the model behind "falls onto a leg".

#define G 9810.0f
#define H 208.0f          // CoM height: torso 233 mm - 25 mm
#define DT 0.001f
#define CONTROL_DT 0.02f
#define FREQ 1.0f
#define DUTY 0.52f
#define LIFT 41.0f

static float frand(unsigned *seed)
{
    *seed = *seed * 1103515245u + 12345u;
    return ((float)((*seed >> 9) & 0xffff) / 65535.0f) * 2.0f - 1.0f;
}

static void test_signs_and_latching(void)
{
    // Scene convention: roll > 0 lowers the left side -> CoM moves left (+Z);
    // pitch > 0 raises the nose -> CoM moves back (-X).
    const robot_planar_point_t d = robot_walk_tilt_displacement((robot_tilt_t){0.1f, 0.1f}, H);
    assert(d.z > 20.0f && d.x < -20.0f);
    const robot_planar_point_t v = robot_walk_tilt_velocity((robot_tilt_t){0.5f, -0.5f}, H);
    assert(v.z > 100.0f && v.x > 100.0f);

    robot_walk_balance_t b; robot_walk_balance_reset(&b);
    const robot_walk_balance_config_t cfg = robot_walk_balance_default_config();
    const robot_planar_point_t capture = {12.0f, -7.0f};
    const robot_vec3_t planned = {100.0f, 0.0f, 50.0f};
    // Planted with zero offset; swing blends to the capture and holds it.
    robot_vec3_t p = robot_walk_balance_apply(&b, &cfg, ROBOT_LEG_LEFT_FRONT, planned, false, 0, capture);
    assert(p.x == 100.0f && p.z == 50.0f);
    p = robot_walk_balance_apply(&b, &cfg, ROBOT_LEG_LEFT_FRONT, planned, true, 0.0f, capture);
    assert(fabsf(p.x - 100.0f) < 1e-3f);
    p = robot_walk_balance_apply(&b, &cfg, ROBOT_LEG_LEFT_FRONT, planned, true, 0.84f, capture);
    assert(fabsf(p.x - 112.0f) < 0.2f && fabsf(p.z - 43.0f) < 0.2f);
    // After the freeze point a new capture no longer moves the foot.
    p = robot_walk_balance_apply(&b, &cfg, ROBOT_LEG_LEFT_FRONT, planned, true, 0.95f,
                                 (robot_planar_point_t){-30.0f, 30.0f});
    assert(fabsf(p.x - 112.0f) < 0.2f);
    // Touchdown latches; a planted foot never slides with the capture.
    p = robot_walk_balance_apply(&b, &cfg, ROBOT_LEG_LEFT_FRONT, planned, false, 0,
                                 (robot_planar_point_t){-30.0f, 30.0f});
    assert(fabsf(p.x - 112.0f) < 0.2f && fabsf(p.z - 43.0f) < 0.2f);

    // Deadband and bound.
    robot_walk_balance_reset(&b);
    robot_planar_point_t c = robot_walk_balance_capture(&b, &cfg, (robot_planar_point_t){1.0f, -1.0f},
                                                        (robot_planar_point_t){0}, H, CONTROL_DT);
    assert(c.x == 0.0f && c.z == 0.0f);
    c = robot_walk_balance_capture(&b, &cfg, (robot_planar_point_t){500.0f, -500.0f},
                                   (robot_planar_point_t){0}, H, CONTROL_DT);
    assert(c.x == cfg.limit_mm && c.z == -cfg.limit_mm);
}

// ---- Reduced walking simulation -------------------------------------------

typedef enum { CAPTURE_OFF, CAPTURE_NOW, CAPTURE_WRONG_SIGN } capture_mode_t;
typedef enum { SWING_LEVEL, SWING_WRONG_SIGN } swing_mode_t;

typedef struct {
    float freq, duty;
    capture_mode_t capture;
    swing_mode_t swing;
    bool adapt;                    // learn the CoM offset from the captures
    robot_planar_point_t com_bias; // true CoM relative to where the plan assumes it
    float kick_mm_s;               // random velocity kick at every touchdown
    unsigned seed;
} sim_case_t;

typedef struct {
    int steps, early_touchdowns;
    float worst_tilt_deg;
    bool fell;
    robot_planar_point_t learned;
} sim_result_t;

static robot_planar_point_t sub(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x - b.x, a.z - b.z}; }

static float signed_distance(robot_planar_point_t p, robot_planar_point_t a, robot_planar_point_t b)
{
    const robot_planar_point_t e = sub(b, a), r = sub(p, a);
    return (e.x * r.z - e.z * r.x) / hypotf(e.x, e.z);
}

static float lift_at(float s)
{
    return LIFT * (s < 0.5f ? 1 - powf(1 - 2 * s, 3) : 1 - powf(2 * s - 1, 3));
}

static sim_result_t simulate(sim_case_t k)
{
    const robot_geometry_t g = robot_kinematics_default_geometry();
    robot_planar_point_t nominal[4], foot[4];
    for (int leg = 0; leg < 4; ++leg) {
        const robot_vec3_t f = robot_kinematics_neutral_foot(&g, (robot_leg_t)leg);
        const bool left = leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR;
        foot[leg] = nominal[leg] = (robot_planar_point_t){f.x, f.z + (left ? 25.0f : -25.0f)};
    }
    const float w = sqrtf(G / H);
    const robot_walk_balance_config_t cfg = robot_walk_balance_default_config();
    robot_walk_balance_t bal; robot_walk_balance_reset(&bal);
    // support: mean offset of the planted feet. A new foot lands at
    // support + capture, so a robot that needs its feet shifted keeps them
    // there. learned: slow estimate of the CoM offset; the body moves by
    // -learned so that the needed foot offset returns to zero.
    robot_planar_point_t learned = {0}, support_offset = {0}, capture = {0};

    // kin: where the legs would carry the CoM (servo lag 50 ms behind the
    // plan). c: the true CoM. On a support line the legs cannot move c across
    // the line; c - kin is then the robot turning about its feet, which is
    // what the IMU sees: tilt = (c - kin)/h, tilt rate = (c' - kin')/h.
    robot_planar_point_t kin = k.com_bias, kin_v = {0}, c = k.com_bias, cv = {0};
    robot_planar_point_t delayed_rot = {0}, delayed_rot_v = {0};
    bool touched[4] = {0}, was_stance[4] = {1, 1, 1, 1};
    sim_result_t r = {0};
    unsigned seed = k.seed;
    float phase = 0.5f, control_clock = 0;

    for (int step = 0; step < (int)(14.0f / DT); ++step) {
        const float t = step * DT;
        phase += k.freq * DT; phase -= floorf(phase);
        // Legs hold the body at -learned; the CoM is com_bias further.
        const robot_planar_point_t target = {k.com_bias.x - learned.x, k.com_bias.z - learned.z};

        bool contact[4]; float swing_phase[4];
        for (int leg = 0; leg < 4; ++leg) {
            const float offset = (leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_RIGHT_REAR) ? 0 : 0.5f;
            float local = phase + offset; local -= floorf(local);
            const bool stance = local < k.duty;
            swing_phase[leg] = stance ? 0 : (local - k.duty) / (1 - k.duty);
            if (!stance && was_stance[leg] && (leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_RIGHT_FRONT)) {
                // Liftoff leaves two feet: push-off/backlash noise acts now.
                cv.x += k.kick_mm_s * frand(&seed); cv.z += k.kick_mm_s * frand(&seed);
            }
            if (stance && !was_stance[leg]) {
                // Touchdown: learn the CoM offset from the correction this
                // foot landed with.
                if (k.adapt) {
                    learned.x = fmaxf(-30, fminf(30, learned.x + 0.05f * bal.current[leg].x));
                    learned.z = fmaxf(-30, fminf(30, learned.z + 0.05f * bal.current[leg].z));
                }
                if (leg == ROBOT_LEG_LEFT_FRONT) ++r.steps;
            }
            if (stance) touched[leg] = false;
            was_stance[leg] = stance;
            contact[leg] = stance || touched[leg];
        }

        // Controller at 50 Hz with the IMU one sample (20 ms) late.
        control_clock += DT;
        if (control_clock >= CONTROL_DT - 1e-6f) {
            control_clock = 0;
            const float noise = 0.2f * 3.14159f / 180.0f, gyro_noise = 0.5f * 3.14159f / 180.0f;
            robot_tilt_t tilt = {asinf(fmaxf(-1, fminf(1, delayed_rot.z / H))) + noise * frand(&seed),
                                 -asinf(fmaxf(-1, fminf(1, delayed_rot.x / H))) + noise * frand(&seed)};
            robot_tilt_t rate = {delayed_rot_v.z / H + gyro_noise * frand(&seed),
                                 -delayed_rot_v.x / H + gyro_noise * frand(&seed)};
            delayed_rot = sub(c, kin); delayed_rot_v = sub(cv, kin_v);
            if (k.capture == CAPTURE_WRONG_SIGN) {
                tilt.roll = -tilt.roll; tilt.pitch = -tilt.pitch;
                rate.roll = -rate.roll; rate.pitch = -rate.pitch;
            }
            // The next foot lands at the planted support plus the capture.
            robot_planar_point_t mean = {0}; int n = 0;
            for (int leg = 0; leg < 4; ++leg) if (contact[leg] && !touched[leg]) {
                mean.x += bal.landed[leg].x; mean.z += bal.landed[leg].z; ++n;
            }
            if (n) support_offset = (robot_planar_point_t){mean.x / n, mean.z / n};
            const robot_planar_point_t xi = k.capture == CAPTURE_OFF ? (robot_planar_point_t){0}
                : robot_walk_balance_capture(&bal, &cfg, robot_walk_tilt_displacement(tilt, H),
                                             robot_walk_tilt_velocity(rate, H), H, CONTROL_DT);
            capture = k.capture == CAPTURE_OFF ? (robot_planar_point_t){0}
                : (robot_planar_point_t){support_offset.x + xi.x, support_offset.z + xi.z};
        }

        for (int leg = 0; leg < 4; ++leg) {
            const bool swing = !contact[leg];
            const robot_vec3_t p = robot_walk_balance_apply(&bal, &cfg, (robot_leg_t)leg,
                (robot_vec3_t){nominal[leg].x, 0, nominal[leg].z}, swing, swing_phase[leg], capture);
            if (swing) foot[leg] = (robot_planar_point_t){p.x, p.z};
        }

        int stance_legs[2], ns = 0, contacts = 0;
        for (int leg = 0; leg < 4; ++leg) if (contact[leg]) { ++contacts; if (ns < 2) stance_legs[ns++] = leg; }
        kin_v = (robot_planar_point_t){(target.x - kin.x) / 0.05f, (target.z - kin.z) / 0.05f};
        kin.x += kin_v.x * DT; kin.z += kin_v.z * DT;
        if (contacts == 2) {
            // Along the line the legs carry the CoM; across it c'' = w^2 d.
            const robot_planar_point_t a = foot[stance_legs[0]], b = foot[stance_legs[1]];
            const robot_planar_point_t e = sub(b, a);
            const float len = hypotf(e.x, e.z);
            const robot_planar_point_t u = {e.x / len, e.z / len}, n = {-u.z, u.x};
            const float d = signed_distance(c, a, b);
            const float along = kin_v.x * u.x + kin_v.z * u.z;
            const float across = (cv.x * n.x + cv.z * n.z) + w * w * d * DT;
            cv = (robot_planar_point_t){along * u.x + across * n.x, along * u.z + across * n.z};
            // A swing foot at distance r on the falling side drops by r*angle;
            // holding its world height leaves 30 % (servo lag), the inverted
            // sign of the old swing transform adds 70 %.
            const float angle = ((c.x - kin.x) * n.x + (c.z - kin.z) * n.z) / H;
            for (int leg = 0; leg < 4; ++leg) if (!contact[leg]) {
                const float r_mm = signed_distance(foot[leg], a, b);
                const float drop = r_mm * angle * (k.swing == SWING_LEVEL ? 0.3f : 1.7f);
                const float s = swing_phase[leg];
                if (s > 0.05f && s < 0.95f && lift_at(s) - drop <= 0.0f) {
                    touched[leg] = true; contact[leg] = true;
                    if (t > 3.0f) ++r.early_touchdowns;
                }
            }
        } else {
            // Three or four feet carry the robot; the legs level it again
            // within the servo time, absorbing the fall speed.
            cv = (robot_planar_point_t){kin_v.x - (c.x - kin.x) / 0.05f, kin_v.z - (c.z - kin.z) / 0.05f};
        }
        c.x += cv.x * DT; c.z += cv.z * DT;

        const float tilt_deg = hypotf(c.x - kin.x, c.z - kin.z) / H * 57.2958f;
        if (t > 3.0f && tilt_deg > r.worst_tilt_deg) r.worst_tilt_deg = tilt_deg;
        if (tilt_deg > 30.0f) { r.fell = true; break; }
    }
    r.learned = learned;
    return r;
}

static sim_result_t run_case(const char *name, sim_case_t k)
{
    const sim_result_t r = simulate(k);
    printf("  %-44s early=%3d/%-3d worst tilt=%5.1f deg  CoM learned=(%5.1f,%5.1f)%s\n", name,
           r.early_touchdowns, 2 * r.steps, r.worst_tilt_deg, r.learned.x, r.learned.z,
           r.fell ? "  FELL" : "");
    return r;
}

static void test_walking_simulation(void)
{
    const robot_planar_point_t bias = {-6.0f, 5.0f};   // CoM 6 mm back, 5 mm left
    const sim_case_t mode3 = {.freq = 1.0f, .duty = 0.52f, .com_bias = bias, .kick_mm_s = 10, .seed = 7};
    sim_case_t k;

    printf(" Mode 3 profile: 1.0 Hz, 52 %% -> %.2f s on two feet per step\n", (1 - 0.52f) / 1.0f);
    k = mode3; k.capture = CAPTURE_OFF; k.swing = SWING_WRONG_SIGN;
    const sim_result_t old = run_case("current firmware (inverted swing sign)", k);
    k = mode3; k.capture = CAPTURE_OFF; k.swing = SWING_LEVEL;
    const sim_result_t level = run_case("swing sign fixed only", k);
    k = mode3; k.capture = CAPTURE_NOW; k.adapt = true;
    const sim_result_t m3 = run_case("fixed + capture point + learned CoM", k);
    k.kick_mm_s = 25; k.seed = 11;
    const sim_result_t m3_noisy = run_case("  same, 25 mm/s push-off kicks", k);

    const sim_case_t fast = {.freq = 1.5f, .duty = 0.70f, .com_bias = bias, .kick_mm_s = 10, .seed = 7};
    printf(" Short swing: 1.5 Hz, 70 %% -> %.2f s on two feet per step\n", (1 - 0.70f) / 1.5f);
    k = fast; k.capture = CAPTURE_OFF;
    const sim_result_t fast_none = run_case("no capture", k);
    k = fast; k.capture = CAPTURE_NOW; k.adapt = true;
    const sim_result_t fast_cp = run_case("capture point + learned CoM", k);
    k.kick_mm_s = 25; k.seed = 11;
    const sim_result_t fast_noisy = run_case("  same, 25 mm/s push-off kicks", k);
    k = fast; k.capture = CAPTURE_WRONG_SIGN; k.adapt = false;
    const sim_result_t fast_wrong = run_case("capture with the reverted (inverted) sign", k);

    // Robust facts this model shows; the printed table is the evidence.
    assert(old.early_touchdowns > 0 && !old.fell);              // it "walks" by landing early
    assert(level.fell || level.worst_tilt_deg > old.worst_tilt_deg);
    assert(!fast_cp.fell && fast_cp.early_touchdowns == 0);     // two feet stay in the air
    assert(fast_cp.worst_tilt_deg < 5.0f);
    assert(fabsf(fast_cp.learned.x - bias.x) < 2.0f && fabsf(fast_cp.learned.z - bias.z) < 2.0f);
    assert(fast_wrong.fell || fast_wrong.worst_tilt_deg > 3 * fast_cp.worst_tilt_deg);
    // 0.48 s on two feet is 3.3 time constants: the fall grows x26 per step,
    // faster than foot placement can correct. It catches itself on a foot.
    assert(m3.fell || m3.early_touchdowns > 0);
    assert(!fast_noisy.fell && fast_noisy.early_touchdowns == 0);
    (void)m3_noisy; (void)fast_none;
}

int main(void)
{
    test_signs_and_latching();
    test_walking_simulation();
    puts("robot walk balance tests passed");
    return 0;
}
