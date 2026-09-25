#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define DUALSENSE_MAX_SCAN_DEVICES 12
#define DUALSENSE_DEVICE_NAME_SIZE 32

typedef enum {
    DUALSENSE_STATE_OFF = 0,
    DUALSENSE_STATE_READY = 1,
    DUALSENSE_STATE_SCANNING = 2,
    DUALSENSE_STATE_CONNECTING = 3,
    DUALSENSE_STATE_CONNECTED = 4,
    DUALSENSE_STATE_ERROR = 5,
} dualsense_connection_state_t;

typedef struct {
    uint8_t address[6];
    int8_t rssi;
    char name[DUALSENSE_DEVICE_NAME_SIZE];
} dualsense_scan_device_t;

typedef struct {
    dualsense_connection_state_t state;
    bool initialized;
    bool has_saved_controller;
    bool has_input;
    bool has_battery;
    uint8_t address[6];
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t battery_percent;
    uint8_t report_id;
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t left_trigger;
    uint8_t right_trigger;
    uint8_t buttons[3];
    uint8_t report_length;
    uint32_t sample_count;
} dualsense_snapshot_t;

// The Bluetooth controller and Bluedroid must already be enabled with Classic Bluetooth support.
esp_err_t dualsense_init(void);
esp_err_t dualsense_deinit(void);
esp_err_t dualsense_start_pairing(void);
esp_err_t dualsense_connect_saved(void);
esp_err_t dualsense_connect_discovered(const uint8_t address[6]);
esp_err_t dualsense_disconnect(void);
esp_err_t dualsense_forget(void);
void dualsense_get_snapshot(dualsense_snapshot_t *snapshot);
size_t dualsense_get_scanned_devices(dualsense_scan_device_t *devices, size_t capacity);

void dualsense_poll(void);

// Mirrors Aura's state colour and selected gait's five-bit player-LED mask.
// Both indications are retained until the interrupt channel is available.
void dualsense_set_indicators(uint8_t red, uint8_t green, uint8_t blue, uint8_t players);

void dualsense_print_diagnostics(void);
