#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define TOF_SENSOR_COUNT 2

typedef struct {
    uint8_t address_7bit;
    bool enabled;
    uint16_t timing_budget_ms;
    uint16_t intermeasurement_ms;
    // Difference between the known reference distance and the reported range.
    // It is programmed into the VL53L4CD and persisted in NVS per port.
    int16_t offset_mm;
    // Optical and result-quality tuning, all stored per physical port.
    // Xtalk is for a cover window above the sensor; the remaining values
    // determine when a range result is considered trustworthy.
    uint16_t xtalk_kcps;
    uint16_t signal_threshold_kcps;
    uint16_t sigma_threshold_mm;
    bool detection_enabled;
    uint16_t detection_low_mm;
    uint16_t detection_high_mm;
    uint8_t detection_window;
} tof_config_t;

typedef struct {
    bool present;
    bool ranging;
    bool valid;
    uint8_t address_7bit;
    uint8_t range_status;
    uint16_t distance_mm;
    uint16_t signal_per_spad_kcps;
    uint16_t ambient_per_spad_kcps;
    uint16_t number_of_spad;
    uint16_t sigma_mm;
    uint32_t sample_count;
    esp_err_t error;
} tof_sensor_state_t;

typedef struct {
    tof_sensor_state_t sensor[TOF_SENSOR_COUNT];
} tof_snapshot_t;

// Reserve a connector before tof_prepare_pins(). A reserved XSHUT pin is put
// into high-impedance input mode and is never driven by the ToF service. Aura
// uses S1/GPIO14 this way when an MPU6050 DATA_RDY output is fitted there.
esp_err_t tof_set_port_reserved(uint8_t index, bool reserved);
// Drive each non-reserved active-low XSHUT line low as soon as the application
// starts. Call before initializing any peripheral that may take substantial time.
esp_err_t tof_prepare_pins(void);
// Runs before the ESP-IDF I2C driver takes ownership of GPIO25/GPIO26.
// It only probes S1 at the VL53L4CD factory address and leaves it reset.
esp_err_t tof_early_s1_wire_test(void);
esp_err_t tof_init(void);
esp_err_t tof_reinitialize(void);
esp_err_t tof_configure(uint8_t index, const tof_config_t *config);
esp_err_t tof_set_offset(uint8_t index, int16_t offset_mm);
esp_err_t tof_set_tuning(uint8_t index, const tof_config_t *config);
// Re-runs the VL53L4CD temperature compensation sequence while preserving the
// previous ranging state. The sensor should be updated after a >8 C change.
esp_err_t tof_temperature_update(uint8_t index);
uint8_t tof_present_count(void);
void tof_get_config(uint8_t index, tof_config_t *config);
void tof_get_snapshot(tof_snapshot_t *snapshot);
