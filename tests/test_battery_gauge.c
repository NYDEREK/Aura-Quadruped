#include <assert.h>
#include <stdio.h>
#include "battery_gauge.h"

static battery_gauge_t settled(float voltage_v)
{
    battery_gauge_t gauge = {0};
    battery_gauge_update(&gauge, true, voltage_v, 0.05f);
    return gauge;
}

static void test_fresh_levels_and_bar(void)
{
    battery_gauge_t g = settled(12.6f);
    assert(g.valid && g.level == BATTERY_LEVEL_OK && g.bar_pixels == 6);
    g = settled(11.33f);
    assert(g.level == BATTERY_LEVEL_OK && g.bar_pixels == 4);
    g = settled(10.9f);
    assert(g.level == BATTERY_LEVEL_LOW && g.bar_pixels == 3);
    g = settled(10.2f);
    assert(g.level == BATTERY_LEVEL_LOW && g.bar_pixels == 1);
    assert(battery_gauge_allows_arming(&g));
    // 10.0 V is 0 %: red, one short pixel, arming refused.
    g = settled(10.0f);
    assert(g.level == BATTERY_LEVEL_EMPTY && g.bar_pixels == 1);
    assert(!battery_gauge_allows_arming(&g));
    // USB only: the INA226 sees no pack.
    g = settled(0.0f);
    assert(g.level == BATTERY_LEVEL_EMPTY && g.bar_pixels == 1);
    assert(!battery_gauge_allows_arming(&g));
}

static void test_unmeasured_refuses_arming(void)
{
    battery_gauge_t g = settled(12.0f);
    battery_gauge_update(&g, false, 0.0f, 0.05f);
    assert(!g.valid && g.bar_pixels == 0);
    assert(!battery_gauge_allows_arming(&g));
    assert(!battery_gauge_allows_arming(NULL));
}

static void hold(battery_gauge_t *g, float voltage_v, float seconds)
{
    for (float t = 0.0f; t < seconds; t += 0.05f) battery_gauge_update(g, true, voltage_v, 0.05f);
}

static void test_falling_pack_crosses_exact_thresholds(void)
{
    battery_gauge_t g = settled(11.2f);
    hold(&g, 11.02f, 30.0f);
    assert(g.level == BATTERY_LEVEL_OK);
    hold(&g, 10.98f, 30.0f);
    assert(g.level == BATTERY_LEVEL_LOW);
    hold(&g, 10.02f, 30.0f);
    assert(g.level == BATTERY_LEVEL_LOW && battery_gauge_allows_arming(&g));
    hold(&g, 9.98f, 30.0f);
    assert(g.level == BATTERY_LEVEL_EMPTY && !battery_gauge_allows_arming(&g));
    // Recovering a few millivolts does not flip back; 50 mV above does.
    hold(&g, 10.03f, 30.0f);
    assert(g.level == BATTERY_LEVEL_EMPTY);
    hold(&g, 10.1f, 30.0f);
    assert(g.level == BATTERY_LEVEL_LOW && battery_gauge_allows_arming(&g));
}

static void test_load_ripple_is_filtered(void)
{
    // 11.05 V rest with a 1 Hz, +-0.4 V gait sag: colour and bar hold still.
    battery_gauge_t g = settled(11.25f);
    const uint8_t pixels = g.bar_pixels;
    for (int i = 0; i < 400; ++i) {
        const float sag = (i / 10) % 2 ? -0.4f : 0.0f;
        battery_gauge_update(&g, true, 11.25f + sag, 0.05f);
        if (i > 200) assert(g.level == BATTERY_LEVEL_OK && g.bar_pixels == pixels);
    }
}

static void test_connecting_pack_snaps_up(void)
{
    battery_gauge_t g = settled(0.0f);
    battery_gauge_update(&g, true, 12.4f, 0.05f);
    assert(g.voltage_v == 12.4f && g.level == BATTERY_LEVEL_OK && g.bar_pixels == 6);
    assert(battery_gauge_allows_arming(&g));
}

int main(void)
{
    test_fresh_levels_and_bar();
    test_unmeasured_refuses_arming();
    test_falling_pack_crosses_exact_thresholds();
    test_load_ripple_is_filtered();
    test_connecting_pack_snaps_up();
    puts("battery gauge tests passed");
    return 0;
}
