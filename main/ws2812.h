#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
// Optional future WS2812/GRB adapter. No calls are made by the diagnostic application.
// Confirm the actual strip model and pixel count before using it (not given in schematic).
esp_err_t ws2812_init(size_t pixel_count);
esp_err_t ws2812_write_rgb(const uint8_t *rgb, size_t pixel_count);
