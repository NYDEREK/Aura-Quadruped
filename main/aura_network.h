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

// One telemetry cycle as one TCP write: frames are queued with batch_add and
// leave together on batch_send, in the same framing as aura_network_send.
#define AURA_NETWORK_BATCH_BYTES (22 * 32)
typedef struct {
    uint8_t data[AURA_NETWORK_BATCH_BYTES];
    size_t used;
} aura_network_batch_t;
void aura_network_batch_add(aura_network_batch_t *batch, uint8_t channel, const uint8_t *data, size_t size);
void aura_network_batch_send(aura_network_batch_t *batch);

void aura_network_print_status(void);
