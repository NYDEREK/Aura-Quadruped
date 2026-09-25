#include <string.h>
#include "servo_protocol.h"

static uint8_t checksum(const uint8_t *data, size_t size)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) sum += data[i];
    return (uint8_t)~sum;
}

size_t servo_make_ping(uint8_t id, uint8_t packet[6])
{
    if (id > 253) return 0; // No broadcast requests.
    const uint8_t data[] = {0xff, 0xff, id, 2, 1, 0};
    memcpy(packet, data, sizeof(data));
    packet[5] = checksum(packet + 2, 3);
    return sizeof(data);
}

size_t servo_make_read(uint8_t id, uint8_t reg, uint8_t count, uint8_t packet[8])
{
    if (id > 253 || count == 0 || count > SERVO_MAX_DATA || (unsigned)reg + count > 256) return 0;
    const uint8_t data[] = {0xff, 0xff, id, 4, 2, reg, count, 0};
    memcpy(packet, data, sizeof(data));
    packet[7] = checksum(packet + 2, 5);
    return sizeof(data);
}

size_t servo_make_write(uint8_t id, uint8_t reg, const uint8_t *data, size_t count,
                        uint8_t packet[SERVO_MAX_PACKET])
{
    if (id > 253 || !data || count == 0 || count > SERVO_MAX_DATA ||
        (unsigned)reg + count > 256) return 0;
    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = id;
    packet[3] = (uint8_t)(count + 3);
    packet[4] = 3; // WRITE
    packet[5] = reg;
    memcpy(packet + 6, data, count);
    packet[count + 6] = checksum(packet + 2, count + 4);
    return count + 7;
}

size_t servo_make_sync_write(uint8_t reg, uint8_t data_size,
                             const servo_sync_write_item_t *items, size_t item_count,
                             uint8_t packet[SERVO_MAX_PACKET])
{
    // FF FF FE length 83 start-address data-length (id + data)*N checksum
    if (!items || item_count == 0 || item_count > SERVO_MAX_SYNC_SERVOS ||
        data_size == 0 || data_size > SERVO_MAX_DATA ||
        (unsigned)reg + data_size > 256) return 0;
    const size_t size = 8 + item_count * ((size_t)data_size + 1);
    if (size > SERVO_MAX_PACKET) return 0;
    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = 0xfe; // Broadcast ID. A sync write never receives a reply.
    packet[3] = (uint8_t)(4 + item_count * ((size_t)data_size + 1));
    packet[4] = 0x83; // SYNC_WRITE
    packet[5] = reg;
    packet[6] = data_size;
    size_t offset = 7;
    for (size_t i = 0; i < item_count; ++i) {
        if (items[i].id > 253 || !items[i].data) return 0;
        packet[offset++] = items[i].id;
        memcpy(packet + offset, items[i].data, data_size);
        offset += data_size;
    }
    packet[offset] = checksum(packet + 2, offset - 2);
    return offset + 1;
}

bool servo_find_status(const uint8_t *stream, size_t size, uint8_t id,
                       const uint8_t *request, size_t request_size, servo_status_t *status)
{
    for (size_t offset = 0; offset + 6 <= size; ++offset) {
        const uint8_t *frame = stream + offset;
        if (frame[0] != 0xff || frame[1] != 0xff || frame[2] != id) continue;
        const size_t length = frame[3];
        if (length < 2 || length > SERVO_MAX_DATA + 2) continue;
        const size_t total = length + 4;
        if (offset + total > size) continue;
        if (checksum(frame + 2, total - 3) != frame[total - 1]) continue;
        // RX is wired to TTL_DATA and receives our own transmission as an echo.
        if (total == request_size && memcmp(frame, request, total) == 0) {
            offset += total - 1;
            continue;
        }
        status->id = id;
        status->error = frame[4];
        status->data_size = length - 2;
        memcpy(status->data, frame + 5, status->data_size);
        return true;
    }
    return false;
}

int servo_signed_magnitude(uint16_t raw, unsigned sign_bit)
{
    const uint16_t mask = (uint16_t)(1U << sign_bit);
    const int magnitude = raw & (mask - 1);
    return (raw & mask) ? -magnitude : magnitude;
}

uint16_t servo_signed_magnitude_encode(int value, unsigned sign_bit)
{
    const uint16_t mask = (uint16_t)(1U << sign_bit);
    const unsigned magnitude = value < 0 ? (unsigned)(-value) : (unsigned)value;
    return (uint16_t)(magnitude | (value < 0 ? mask : 0));
}
