#pragma once

#include <stdint.h>
#include <string.h>

// Bluetooth HID output 0x31, including the 0xa2 transaction byte. Offsets
// follow Linux hid-playstation.c's dualsense_output_report_common.
// Both RGB sides share one colour; player_leds contains five independent bits.
#define DUALSENSE_LED_PACKET_SIZE 79
static inline void dualsense_led_packet(uint8_t packet[DUALSENSE_LED_PACKET_SIZE],
    uint8_t sequence, uint8_t red, uint8_t green, uint8_t blue, uint8_t player_leds)
{
    memset(packet, 0, DUALSENSE_LED_PACKET_SIZE);
    packet[0] = 0xa2; packet[1] = 0x31;
    packet[2] = (sequence & 15) << 4; packet[3] = 0x10;
    packet[5] = 0x04 | 0x10; // RGB + player indicators only
    packet[42] = 0x02;       // permit lightbar setup
    packet[45] = 0x02;       // cancel the factory pairing-blue fade
    packet[47] = player_leds & 0x1f;
    packet[48] = red; packet[49] = green; packet[50] = blue;
    // No flags enable motors, audio, microphone, power or adaptive triggers.
    uint32_t crc = 0xffffffff;
    for (unsigned i = 0; i < 75; ++i) {
        crc ^= packet[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    crc = ~crc;
    for (unsigned i = 0; i < 4; ++i) packet[75+i] = crc >> (8*i);
}
