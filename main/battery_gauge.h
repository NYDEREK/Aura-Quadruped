#pragma once
#include <stdbool.h>
#include <stdint.h>

// 3S pack window used by the strip and the arm gate: 10.0 V is 0 % (the
// robot refuses to arm), 12.6 V is 100 %.  Below 11.0 V the bar turns orange.
#define BATTERY_EMPTY_V 10.0f
#define BATTERY_LOW_V 11.0f
#define BATTERY_FULL_V 12.6f
// A level or bar step rises only this far past its boundary, so INA noise and
// gait-rate load ripple cannot make a pixel flicker on a threshold.
#define BATTERY_HYSTERESIS_V 0.05f
#define BATTERY_BAR_PIXELS 6

typedef enum {
    BATTERY_LEVEL_EMPTY = 0,  // <= 10.0 V: red, arming refused
    BATTERY_LEVEL_LOW = 1,    // < 11.0 V: orange
    BATTERY_LEVEL_OK = 2,     // green
} battery_level_t;

typedef struct {
    bool valid;
    float voltage_v;          // filtered pack voltage
    battery_level_t level;
    uint8_t bar_pixels;       // 1...BATTERY_BAR_PIXELS while valid
} battery_gauge_t;

// Feed one measurement every dt_s seconds. measured=false (no INA226 and no
// VIN ADC reading) invalidates the gauge, which also refuses arming.
void battery_gauge_update(battery_gauge_t *gauge, bool measured, float voltage_v, float dt_s);
bool battery_gauge_allows_arming(const battery_gauge_t *gauge);
