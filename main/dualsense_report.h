#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
// Payload excludes the report ID (ESP-IDF provides it separately).
typedef struct { uint8_t axes[4], triggers[2], buttons[3]; } dualsense_controls_t;
static inline bool dualsense_decode(uint8_t id, const uint8_t *data, size_t size,
                                    dualsense_controls_t *out)
{
    if (!data || !out) return false;
    size_t base, buttons, triggers;
    if (id == 0x01 && size == 9) {
        base = 0; buttons = 4; triggers = 7;
    } else if (id == 0x31 && size >= 77) {
        base = 1; buttons = 8; triggers = 5;
    } else return false;
    for (unsigned i = 0; i < 4; ++i) out->axes[i] = data[base + i];
    for (unsigned i = 0; i < 2; ++i) out->triggers[i] = data[triggers + i];
    for (unsigned i = 0; i < 3; ++i) out->buttons[i] = data[buttons + i];
    if (id == 0x01) out->buttons[2] &= 3; // Simple report has no mute button.
    return true;
}
