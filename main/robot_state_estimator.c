#include <math.h>
#include <string.h>

#include "robot_balance.h"
#include "robot_state_estimator.h"

#define N ROBOT_ESTIMATOR_STATES
#define GRAVITY_MM_S2 9810.0f
#define PI_F 3.14159265f

// Noise, standard deviations. Process: the IMU acceleration is uncertain by
// the gait's impacts; without it the velocity is a random walk. A planted
// foot may creep by about a millimetre per tick. A swinging foot is free: its
// process noise grows with (1 - trust) so its state simply follows the
// kinematics and lands where the leg puts it, without pulling the body; its
// kinematic measurement is only mildly de-weighted (MIT's "suspect" factor).
#define SIGMA_POSITION_MM 0.5f
#define SIGMA_ACCEL_MM_S2 900.0f
#define SIGMA_NO_ACCEL_MM_S2 3500.0f
#define SIGMA_FOOT_MM 1.0f
#define SIGMA_FOOT_RELATIVE_MM 8.0f
#define SIGMA_STILL_MM_S 4.0f
#define SWING_FREEDOM 1.0e5f
#define SUSPECT 9.0f
// Heading filter: gyro white noise, bias random walk, zero-rate noise.
#define HEADING_GYRO_NOISE (0.6f * PI_F / 180.0f)
#define HEADING_BIAS_WALK (0.02f * PI_F / 180.0f)
#define HEADING_STILL_NOISE (0.25f * PI_F / 180.0f)
#define HEADING_BIAS_SIGMA0 (2.0f * PI_F / 180.0f)
#define STILL_SECONDS 0.5f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Body -> world: the IK tilt convention (balance transform) then heading.
static robot_vec3_t to_world(robot_vec3_t v, float roll, float pitch, float heading)
{
    const robot_vec3_t tilted = robot_balance_to_world(v, (robot_balance_pose_t){.roll = roll, .pitch = pitch});
    const float c = cosf(heading), s = sinf(heading);
    return (robot_vec3_t){c * tilted.x - s * tilted.z, tilted.y, s * tilted.x + c * tilted.z};
}

static void heading_reset(robot_estimator_t *e, bool keep_bias)
{
    e->heading = 0; e->h00 = 0; e->h01 = 0;
    if (!keep_bias) { e->gyro_bias = 0; e->h11 = HEADING_BIAS_SIGMA0 * HEADING_BIAS_SIGMA0; }
}

static void heading_predict(robot_estimator_t *e, float rate, float dt)
{
    e->heading += (rate - e->gyro_bias) * dt;
    if (e->heading > PI_F) e->heading -= 2 * PI_F;
    if (e->heading < -PI_F) e->heading += 2 * PI_F;
    const float q_heading = HEADING_GYRO_NOISE * dt * HEADING_GYRO_NOISE * dt;
    const float q_bias = HEADING_BIAS_WALK * HEADING_BIAS_WALK * dt;
    e->h00 = e->h00 - 2 * dt * e->h01 + dt * dt * e->h11 + q_heading;
    e->h01 = e->h01 - dt * e->h11;
    e->h11 = e->h11 + q_bias;
}

// Still: true yaw rate is zero = gyro - bias + noise, H = [0, -1].
static void heading_observe_still(robot_estimator_t *e, float rate)
{
    const float s = e->h11 + HEADING_STILL_NOISE * HEADING_STILL_NOISE;
    const float residual = -(rate - e->gyro_bias);
    e->heading += -e->h01 / s * residual;
    e->gyro_bias += -e->h11 / s * residual;
    const float h00 = e->h00 - e->h01 * e->h01 / s, h01 = e->h01 * (1 - e->h11 / s), h11 = e->h11 * (1 - e->h11 / s);
    e->h00 = h00; e->h01 = h01; e->h11 = h11;
}

static void set_diagonal(float *P, int first, int count, float variance)
{
    for (int i = first; i < first + count; ++i) P[i * N + i] = variance;
}

void robot_estimator_reset(robot_estimator_t *e)
{
    if (!e) return;
    memset(e, 0, sizeof(*e));
    heading_reset(e, false);
}

void robot_estimator_rezero(robot_estimator_t *e)
{
    if (!e) return;
    const float bias = e->gyro_bias, h11 = e->h11;
    memset(e->x, 0, sizeof(e->x));
    memset(e->P, 0, sizeof(e->P));
    e->feet_initialized = false;
    e->valid = false;
    heading_reset(e, true);
    e->gyro_bias = bias; e->h11 = h11;
}

// Cholesky solve of S X = B for symmetric positive definite S (m x m),
// B (m x n) row-major, in place. Returns false if S is not positive.
static bool cholesky_solve(float *S, int m, float *B, int n)
{
    for (int j = 0; j < m; ++j) {
        float d = S[j * m + j];
        for (int k = 0; k < j; ++k) d -= S[j * m + k] * S[j * m + k];
        if (!(d > 1e-9f)) return false;
        const float l = sqrtf(d);
        S[j * m + j] = l;
        for (int i = j + 1; i < m; ++i) {
            float v = S[i * m + j];
            for (int k = 0; k < j; ++k) v -= S[i * m + k] * S[j * m + k];
            S[i * m + j] = v / l;
        }
    }
    for (int c = 0; c < n; ++c) {
        for (int i = 0; i < m; ++i) {           // L y = b
            float v = B[i * n + c];
            for (int k = 0; k < i; ++k) v -= S[i * m + k] * B[k * n + c];
            B[i * n + c] = v / S[i * m + i];
        }
        for (int i = m - 1; i >= 0; --i) {      // L^T x = y
            float v = B[i * n + c];
            for (int k = i + 1; k < m; ++k) v -= S[k * m + i] * B[k * n + c];
            B[i * n + c] = v / S[i * m + i];
        }
    }
    return true;
}

typedef struct { int kind; int leg; int axis; } row_t;   // kind 0: p - foot, 1: velocity

// Entry of measurement row r for state index `state`.
static float h_entry(row_t r, int state)
{
    if (r.kind == 0) return (state == r.axis ? 1.0f : 0.0f) - (state == 6 + 3 * r.leg + r.axis ? 1.0f : 0.0f);
    return state == 3 + r.axis ? 1.0f : 0.0f;
}

void robot_estimator_step(robot_estimator_t *e, const robot_estimator_input_t *in)
{
    if (!e || !in || !isfinite(in->dt) || in->dt <= 0 || in->dt > 0.2f) return;
    const float dt = in->dt;

    // Heading runs whenever the IMU runs, armed or not (a carried robot turns).
    heading_predict(e, in->yaw_rate_rad_s, dt);
    const bool quiet = !in->commanded && !in->walking && fabsf(in->accel_norm_g - 1.0f) < 0.04f &&
                       fabsf(in->yaw_rate_rad_s) < 3.0f * PI_F / 180.0f;
    bool planted = true;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) planted &= in->feet_valid[leg] && in->trust[leg] > 0.9f;
    e->still_time = quiet ? e->still_time + dt : 0.0f;
    e->still = e->still_time >= STILL_SECONDS;
    if (e->still) heading_observe_still(e, in->yaw_rate_rad_s);

    // Position: only while the legs hold the body. A disarmed robot lies
    // limp; its position is held until the next arm.
    if (!in->armed) {
        e->feet_initialized = false;
        e->x[3] = e->x[4] = e->x[5] = 0;
        return;
    }

    robot_vec3_t relative[ROBOT_LEG_COUNT];
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        relative[leg] = to_world(in->feet_body_mm[leg], in->roll_rad, in->pitch_rad, e->heading);

    if (!e->feet_initialized) {
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
            e->x[6 + 3 * leg] = e->x[0] + relative[leg].x;
            e->x[7 + 3 * leg] = e->x[1] + relative[leg].y;
            e->x[8 + 3 * leg] = e->x[2] + relative[leg].z;
        }
        memset(e->P, 0, sizeof(e->P));
        set_diagonal(e->P, 0, 3, e->valid ? 25.0f : 1.0f);
        set_diagonal(e->P, 3, 3, 100.0f);
        set_diagonal(e->P, 6, 12, 400.0f);
        e->feet_initialized = true;
        e->valid = true;
    }

    // Predict: p += v dt + a dt^2/2, v += a dt; feet constant.
    robot_vec3_t a = {0};
    if (in->use_accel) {
        const robot_vec3_t world = to_world(in->accel_mm_s2, in->roll_rad, in->pitch_rad, e->heading);
        a = (robot_vec3_t){world.x, world.y - GRAVITY_MM_S2, world.z};
    }
    const float acc[3] = {a.x, a.y, a.z};
    for (int k = 0; k < 3; ++k) {
        e->x[k] += e->x[3 + k] * dt + 0.5f * acc[k] * dt * dt;
        e->x[3 + k] += acc[k] * dt;
    }
    // P = A P A^T with A = I + dt * (v -> p).
    for (int c = 0; c < N; ++c) for (int k = 0; k < 3; ++k) e->P[k * N + c] += dt * e->P[(3 + k) * N + c];
    for (int r = 0; r < N; ++r) for (int k = 0; k < 3; ++k) e->P[r * N + k] += dt * e->P[r * N + 3 + k];
    const float sigma_a = in->use_accel ? SIGMA_ACCEL_MM_S2 : SIGMA_NO_ACCEL_MM_S2;
    for (int k = 0; k < 3; ++k) {
        e->P[k * N + k] += SIGMA_POSITION_MM * SIGMA_POSITION_MM;
        e->P[(3 + k) * N + 3 + k] += sigma_a * dt * sigma_a * dt;
    }
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        const float freedom = 1.0f + SWING_FREEDOM * (1.0f - clampf(in->trust[leg], 0, 1));
        for (int k = 0; k < 3; ++k)
            e->P[(6 + 3 * leg + k) * N + 6 + 3 * leg + k] += SIGMA_FOOT_MM * SIGMA_FOOT_MM * freedom;
    }

    // Measurement rows.
    row_t rows[ROBOT_ESTIMATOR_MAX_ROWS];
    float y[ROBOT_ESTIMATOR_MAX_ROWS], noise[ROBOT_ESTIMATOR_MAX_ROWS];
    int m = 0;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        if (!in->feet_valid[leg]) continue;
        const float suspect = 1.0f + SUSPECT * (1.0f - clampf(in->trust[leg], 0, 1));
        const float rel[3] = {relative[leg].x, relative[leg].y, relative[leg].z};
        for (int k = 0; k < 3; ++k) {
            rows[m] = (row_t){0, leg, k};
            y[m] = -rel[k];
            noise[m] = SIGMA_FOOT_RELATIVE_MM * SIGMA_FOOT_RELATIVE_MM * suspect;
            ++m;
        }
    }
    if (e->still && planted) {
        for (int k = 0; k < 3; ++k) {
            rows[m] = (row_t){1, 0, k};
            y[m] = 0;
            noise[m] = SIGMA_STILL_MM_S * SIGMA_STILL_MM_S;
            ++m;
        }
    }
    if (!m) return;

    // PHt (N x m) and S = H P H^T + R (m x m), using H's 0/+1/-1 structure.
    static float PHt[N * ROBOT_ESTIMATOR_MAX_ROWS];
    static float S[ROBOT_ESTIMATOR_MAX_ROWS * ROBOT_ESTIMATOR_MAX_ROWS];
    static float KT[ROBOT_ESTIMATOR_MAX_ROWS * N];
    for (int r = 0; r < N; ++r) {
        for (int j = 0; j < m; ++j) {
            float v = 0;
            if (rows[j].kind == 0)
                v = e->P[r * N + rows[j].axis] - e->P[r * N + 6 + 3 * rows[j].leg + rows[j].axis];
            else
                v = e->P[r * N + 3 + rows[j].axis];
            PHt[r * m + j] = v;
        }
    }
    float innovation[ROBOT_ESTIMATOR_MAX_ROWS];
    for (int i = 0; i < m; ++i) {
        float hx = 0;
        for (int s = 0; s < N; ++s) {
            const float h = h_entry(rows[i], s);
            if (h != 0) hx += h * e->x[s];
        }
        innovation[i] = y[i] - hx;
        for (int j = 0; j < m; ++j) {
            float v = 0;
            if (rows[i].kind == 0)
                v = PHt[rows[i].axis * m + j] - PHt[(6 + 3 * rows[i].leg + rows[i].axis) * m + j];
            else
                v = PHt[(3 + rows[i].axis) * m + j];
            S[i * m + j] = v + (i == j ? noise[i] : 0);
        }
    }
    // K^T = S^-1 (PHt)^T
    for (int i = 0; i < m; ++i) for (int s = 0; s < N; ++s) KT[i * N + s] = PHt[s * m + i];
    if (!cholesky_solve(S, m, KT, N)) return;
    for (int s = 0; s < N; ++s) {
        float dx = 0;
        for (int i = 0; i < m; ++i) dx += KT[i * N + s] * innovation[i];
        e->x[s] += dx;
    }
    // P -= K (PHt)^T, then keep it symmetric.
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            float v = 0;
            for (int i = 0; i < m; ++i) v += KT[i * N + r] * PHt[c * m + i];
            e->P[r * N + c] -= v;
        }
    }
    for (int r = 0; r < N; ++r)
        for (int c = r + 1; c < N; ++c) {
            const float v = 0.5f * (e->P[r * N + c] + e->P[c * N + r]);
            e->P[r * N + c] = e->P[c * N + r] = v;
        }
}

void robot_estimator_get(const robot_estimator_t *e, robot_estimate_t *out)
{
    if (!e || !out) return;
    const float c = cosf(e->heading), s = sinf(e->heading);
    *out = (robot_estimate_t){
        .valid = e->valid,
        .still = e->still,
        .position_mm = {e->x[0], e->x[1], e->x[2]},
        .velocity_mm_s = {e->x[3], e->x[4], e->x[5]},
        .heading_rad = e->heading,
        .heading_sigma_rad = sqrtf(fmaxf(0.0f, e->h00)),
        .gyro_bias_rad_s = e->gyro_bias,
        .forward_mm_s = c * e->x[3] + s * e->x[5],
        .left_mm_s = -s * e->x[3] + c * e->x[5],
    };
}
