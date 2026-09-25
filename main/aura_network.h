#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
// TCP framing: channel (0=command, 1=telemetry, 2=event), length, payload (1..20).
typedef void (*aura_network_command_cb)(const uint8_t *, uint16_t);
esp_err_t aura_network_start(aura_network_command_cb callback);
void aura_network_stop(void);
bool aura_network_connected(void);
void aura_network_send(uint8_t channel, const uint8_t *data, size_t size);

void aura_network_print_status(void);
