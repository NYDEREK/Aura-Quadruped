#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "servo_protocol.h"

static uint8_t packet_checksum(const uint8_t *data, size_t size)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) sum += data[i];
    return (uint8_t)~sum;
}

int main(void)
{
    uint8_t ping[6];
    assert(servo_make_ping(1, ping) == sizeof(ping));
    const uint8_t expected_ping[] = {0xff, 0xff, 0x01, 0x02, 0x01, 0xfb};
    assert(memcmp(ping, expected_ping, sizeof(ping)) == 0);
    assert(servo_make_ping(254, ping) == 0);

    uint8_t read[8];
    assert(servo_make_read(1, 56, 15, read) == sizeof(read));
    const uint8_t expected_read[] = {0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x0f, 0xb1};
    assert(memcmp(read, expected_read, sizeof(read)) == 0);
    assert(servo_make_read(1, 250, 15, read) == 0);

    // The one-wire receiver sees the transmitted request first, then the servo reply.
    const uint8_t reply[] = {0xff, 0xff, 0x01, 0x02, 0x00, 0xfc};
    uint8_t stream[1 + sizeof(ping) + sizeof(reply)];
    stream[0] = 0x55;
    memcpy(stream + 1, ping, sizeof(ping));
    memcpy(stream + 1 + sizeof(ping), reply, sizeof(reply));
    servo_status_t status = {0};
    assert(servo_find_status(stream, sizeof(stream), 1, ping, sizeof(ping), &status));
    assert(status.id == 1 && status.error == 0 && status.data_size == 0);

    uint8_t bad_reply[sizeof(reply)];
    memcpy(bad_reply, reply, sizeof(reply));
    bad_reply[sizeof(bad_reply) - 1] ^= 1;
    assert(!servo_find_status(bad_reply, sizeof(bad_reply), 1, ping, sizeof(ping), &status));
    assert(packet_checksum(reply + 2, 3) == reply[5]);

    assert(servo_signed_magnitude(123, 15) == 123);
    assert(servo_signed_magnitude(0x8005, 15) == -5);
    assert(servo_signed_magnitude(0x0407, 10) == -7);
    assert(servo_signed_magnitude_encode(123, 15) == 123);
    assert(servo_signed_magnitude_encode(-5, 15) == 0x8005);
    assert(servo_signed_magnitude_encode(-7, 10) == 0x0407);

    uint8_t write[SERVO_MAX_PACKET];
    const uint8_t position[] = {5, 0x34, 0x12};
    const size_t write_size = servo_make_write(7, 41, position, sizeof(position), write);
    const uint8_t expected_write[] = {0xff, 0xff, 7, 6, 3, 41, 5, 0x34, 0x12, 0x7b};
    assert(write_size == sizeof(expected_write));
    assert(memcmp(write, expected_write, sizeof(expected_write)) == 0);
    assert(servo_make_write(254, 40, position, 1, write) == 0);
    assert(servo_make_write(1, 255, position, 2, write) == 0);

    const uint8_t first[] = {5, 0x34, 0x12, 0, 0, 0x78, 0x56};
    const uint8_t second[] = {9, 0x78, 0x56, 0, 0, 0x34, 0x12};
    const servo_sync_write_item_t items[] = {{.id = 1, .data = first}, {.id = 7, .data = second}};
    uint8_t sync[SERVO_MAX_PACKET];
    const size_t sync_size = servo_make_sync_write(41, sizeof(first), items, 2, sync);
    assert(sync_size == 24);
    assert(sync[0] == 0xff && sync[1] == 0xff && sync[2] == 0xfe);
    assert(sync[3] == 20 && sync[4] == 0x83 && sync[5] == 41 && sync[6] == sizeof(first));
    assert(sync[7] == 1 && memcmp(sync + 8, first, sizeof(first)) == 0);
    assert(sync[15] == 7 && memcmp(sync + 16, second, sizeof(second)) == 0);
    assert(packet_checksum(sync + 2, sync_size - 3) == sync[sync_size - 1]);
    assert(servo_make_sync_write(41, sizeof(first), items, 0, sync) == 0);

    puts("servo protocol tests passed");
    return 0;
}
