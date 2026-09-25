#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SERVO_MAX_DATA 32
#define SERVO_MAX_SYNC_SERVOS 12
// A synchronous position update carries seven bytes for every servo, plus the
// protocol framing.  Keep the general packet buffer large enough for all 12.
#define SERVO_MAX_PACKET 128
typedef struct {
    uint8_t id, error, data[SERVO_MAX_DATA];
    size_t data_size;
} servo_status_t;

typedef struct {
    uint8_t id;
    const uint8_t *data;
} servo_sync_write_item_t;

size_t servo_make_ping(uint8_t id, uint8_t packet[6]);
size_t servo_make_read(uint8_t id, uint8_t reg, uint8_t count, uint8_t packet[8]);
size_t servo_make_write(uint8_t id, uint8_t reg, const uint8_t *data, size_t count,
                        uint8_t packet[SERVO_MAX_PACKET]);
size_t servo_make_sync_write(uint8_t reg, uint8_t data_size,
                             const servo_sync_write_item_t *items, size_t item_count,
                             uint8_t packet[SERVO_MAX_PACKET]);
bool servo_find_status(const uint8_t *stream, size_t size, uint8_t id,
                       const uint8_t *request, size_t request_size, servo_status_t *status);
int servo_signed_magnitude(uint16_t raw, unsigned sign_bit);
uint16_t servo_signed_magnitude_encode(int value, unsigned sign_bit);
