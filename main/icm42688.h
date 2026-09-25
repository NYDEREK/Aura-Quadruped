#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
    uint32_t sample_count;
    int64_t sample_time_us;
} imu_sample_t;

typedef enum {
    ICM_PROFILE_AUTO,
    ICM_PROFILE_HXY,
} icm_profile_t;

// Drive CSB low before configuring the HXY SPI peripheral. CSB selects
// SPI dynamically during a transaction (high selects I²C).
void icm42688_prepare_spi_early(void);
esp_err_t icm42688_init(uint8_t *who_am_i);
esp_err_t icm42688_reinitialize(uint8_t *who_am_i);
// Diagnostic only: configure the documented HXY register map in SPI mode 3
// without validating WHO_AM_I, then return one raw sensor frame.
esp_err_t icm42688_force_hxy_diagnostic(imu_sample_t *sample, uint8_t *data_status);
// Diagnostic only: use the HXY-documented 3-wire SPI mode on SDX/pin 14.
// This bypasses the dedicated SDO/MISO line to isolate a pin-1 routing fault.
esp_err_t icm42688_force_hxy_3wire_diagnostic(imu_sample_t *sample, uint8_t *data_status);
// Read every register address in the current HXY bank using four SPI framing
// variants. This sends no register writes and never controls servos.
esp_err_t icm42688_hxy_read_only_scan(void);
// Sends only an I2C address phase on HXY pins 14/13 to check whether the
// device started in I2C mode. It does not write a register or control servos.
esp_err_t icm42688_hxy_i2c_address_probe(void);
esp_err_t icm42688_read(imu_sample_t *sample);
// Read-only low-speed test that counts MISO transitions during each documented ID frame.
esp_err_t icm42688_miso_activity_test(uint8_t *who_am_i);
const char *icm42688_profile_name(void);
