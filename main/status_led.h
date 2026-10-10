#pragma once
#include <stdint.h>
#include "esp_err.h"

// Brightness of the WS2812 status strip, 0...100 %, persisted in its own NVS
// namespace. It scales the strip only; the DualSense lightbar is unchanged.
#define STATUS_LED_DEFAULT_BRIGHTNESS 20
void status_led_init(void);
uint8_t status_led_brightness(void);
esp_err_t status_led_set_brightness(uint8_t percent);
