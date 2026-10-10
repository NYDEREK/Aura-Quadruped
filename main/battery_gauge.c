#include "battery_gauge.h"

// Servo load sags the pack at gait rate; a 2 s low-pass shows the charge,
// not the last step.  Connecting a pack is a large rise and is taken at once
// so the bar does not crawl up from 0 V after a USB-only start.
#define BATTERY_FILTER_TIME_S 2.0f
#define BATTERY_SNAP_RISE_V 1.0f

// Step 0 is below bounds[0]. A step is entered once the voltage is
// BATTERY_HYSTERESIS_V above its lower bound and left as soon as the voltage
// is back at or below that bound, so a falling pack changes colour exactly
// on the documented threshold.
static unsigned step_with_hysteresis(float voltage_v, bool fresh, unsigned step,
                                     const float *bounds, unsigned count)
{
    const float rise = fresh ? 0.0f : BATTERY_HYSTERESIS_V;
    if (fresh) step = 0;
    while (step < count && voltage_v > bounds[step] + rise) ++step;
    while (step > 0 && voltage_v <= bounds[step - 1]) --step;
    return step;
}

void battery_gauge_update(battery_gauge_t *gauge, bool measured, float voltage_v, float dt_s)
{
    if (!gauge) return;
    if (!measured) {
        *gauge = (battery_gauge_t){0};
        return;
    }
    const bool fresh = !gauge->valid;
    if (fresh || voltage_v > gauge->voltage_v + BATTERY_SNAP_RISE_V || dt_s <= 0.0f) {
        gauge->voltage_v = voltage_v;
    } else {
        gauge->voltage_v += (voltage_v - gauge->voltage_v) * dt_s / (BATTERY_FILTER_TIME_S + dt_s);
    }
    const float v = gauge->voltage_v;

    static const float level_bounds[] = {BATTERY_EMPTY_V, BATTERY_LOW_V};
    gauge->level = (battery_level_t)step_with_hysteresis(
        v, fresh, (unsigned)gauge->level, level_bounds, 2);

    // Pixel n (1-based) lights once the charge enters its sixth of the
    // window; the first pixel always stays lit so an empty pack reads as a
    // short red bar rather than a dead strip.
    float bar_bounds[BATTERY_BAR_PIXELS - 1];
    for (unsigned i = 0; i < BATTERY_BAR_PIXELS - 1; ++i)
        bar_bounds[i] = BATTERY_EMPTY_V +
                        (BATTERY_FULL_V - BATTERY_EMPTY_V) * (float)(i + 1) / BATTERY_BAR_PIXELS;
    const unsigned previous_step = gauge->bar_pixels ? gauge->bar_pixels - 1U : 0U;
    gauge->bar_pixels = (uint8_t)(1U + step_with_hysteresis(v, fresh, previous_step, bar_bounds,
                                                            BATTERY_BAR_PIXELS - 1));
    gauge->valid = true;
}

bool battery_gauge_allows_arming(const battery_gauge_t *gauge)
{
    return gauge && gauge->valid && gauge->level != BATTERY_LEVEL_EMPTY;
}
