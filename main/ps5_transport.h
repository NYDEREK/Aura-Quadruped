#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
typedef struct {
    void (*opened)(const uint8_t address[6]);
    void (*closed)(void);
    void (*input)(const uint8_t *data, size_t length);
} ps5_transport_callbacks_t;
esp_err_t ps5_transport_init(ps5_transport_callbacks_t callbacks);
esp_err_t ps5_transport_connect(const uint8_t address[6]);
esp_err_t ps5_transport_disconnect(void);
// Retains RGB and five player LEDs across reconnects. One combined report;
// no motor, trigger, audio or power flags are enabled.
void ps5_transport_set_indicators(uint8_t red, uint8_t green, uint8_t blue, uint8_t players);
void ps5_transport_poll(void);
void ps5_transport_deinit(void);
