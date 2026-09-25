#include <math.h>
#include <string.h>

#include "robot_contact.h"

// ST3215 present-load is the signed 0.1%-duty report and present-current has
// 6.5 mA/raw-unit resolution. The deliberately low thresholds are not final
// safety limits; the observer also needs phase and tracking evidence and
// uses hysteresis. They will be exposed for per-robot commissioning once the
// live traces have been collected.
#define CONTACT_LOAD_NOMINAL_RAW 180
#define CONTACT_CURRENT_NOMINAL_RAW 60
#define CONTACT_TRACKING_NOMINAL_RAD (3.0f * 3.14159265358979323846f / 180.0f)
#define CONTACT_IMPACT_NOMINAL_G 0.18f
#define CONTACT_ENTER_SAMPLES 2
#define CONTACT_RELEASE_SAMPLES 3

static int absolute_i16(int16_t value)
{
    return value < 0 ? -(int)value : (int)value;
}

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float normalized(float value, float full_scale)
{
    if (!isfinite(value) || full_scale <= 0.0f) return 0.0f;
    return clampf(value / full_scale, 0.0f, 1.0f);
}

void robot_contact_reset(robot_contact_estimate_t *estimate)
{
    if (estimate) memset(estimate, 0, sizeof(*estimate));
}

void robot_contact_update(robot_contact_estimate_t *estimate,
                          const robot_contact_input_t *input)
{
    if (!estimate || !input) return;
    if (!input->armed || !input->feedback_valid) {
        robot_contact_reset(estimate);
        return;
    }

    const float knee_effort = fmaxf(
        normalized((float)absolute_i16(input->knee_load_raw), CONTACT_LOAD_NOMINAL_RAW),
        normalized((float)absolute_i16(input->knee_current_raw), CONTACT_CURRENT_NOMINAL_RAW));
    // The hip sees the same external foot force through another moment arm.
    // Weight it below the knee, which is the main vertical-support actuator.
    const float hip_effort = fmaxf(
        normalized((float)absolute_i16(input->hip_load_raw), CONTACT_LOAD_NOMINAL_RAW),
        normalized((float)absolute_i16(input->hip_current_raw), CONTACT_CURRENT_NOMINAL_RAW));
    const float effort = fmaxf(knee_effort, hip_effort * 0.65f);
    const float tracking = fmaxf(normalized(fabsf(input->hip_tracking_error_radians),
                                             CONTACT_TRACKING_NOMINAL_RAD),
                                 normalized(fabsf(input->knee_tracking_error_radians),
                                             CONTACT_TRACKING_NOMINAL_RAD));
    const float impact = normalized(input->imu_specific_force_error_g, CONTACT_IMPACT_NOMINAL_G);

    // Phase is a prior, never proof. A foot expected to support is allowed a
    // modest effort threshold; an early touchdown during swing needs strong
    // effort plus either an impact or a tracking residual.
    const bool expected_evidence = input->expected_stance &&
        (effort >= 0.30f || (effort >= 0.16f && tracking >= 0.55f));
    const bool early_evidence = !input->expected_stance && input->touchdown_window &&
        effort >= 0.72f && (tracking >= 0.35f || impact >= 0.55f);
    const bool contact_evidence = expected_evidence || early_evidence;

    const float phase_prior = input->expected_stance ? 25.0f : 0.0f;
    const float confidence = clampf(phase_prior + effort * 55.0f + tracking * 13.0f +
                                    (input->touchdown_window ? impact * 7.0f : 0.0f),
                                    0.0f, 100.0f);
    estimate->feedback_valid = true;
    estimate->confidence_percent = (uint8_t)lroundf(confidence);

    if (contact_evidence) {
        if (estimate->contact_samples < UINT8_MAX) ++estimate->contact_samples;
        estimate->release_samples = 0;
        if (estimate->contact_samples >= CONTACT_ENTER_SAMPLES) estimate->contact = true;
    } else {
        estimate->contact_samples = 0;
        if (estimate->release_samples < UINT8_MAX) ++estimate->release_samples;
        if (estimate->release_samples >= CONTACT_RELEASE_SAMPLES) estimate->contact = false;
    }
    estimate->early_touchdown = estimate->contact && early_evidence;
}
