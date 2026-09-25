#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l4cd_uld/platform.h"

#define VL53L4CD_I2C_TIMEOUT_MS 100

static const char *TAG = "vl53l4cd_i2c";

static uint8_t platform_status(Dev_t dev, esp_err_t result)
{
    if (dev) dev->last_error = result;
    return result == ESP_OK ? 0 : 1;
}

esp_err_t vl53l4cd_platform_attach(Dev_t dev, i2c_master_bus_handle_t bus,
                                  uint8_t address_7bit)
{
    if (!dev || !bus || address_7bit > 0x7f) return ESP_ERR_INVALID_ARG;
    if (dev->device) vl53l4cd_platform_detach(dev);
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address_7bit,
        // Match the 50 kHz bit-banged diagnostic that already sees the sensor.
        // This makes the first optical calibration more tolerant of connector
        // and cable capacitance; normal ranging needs only modest bus bandwidth.
        .scl_speed_hz = 50000,
    };
    const esp_err_t result = i2c_master_bus_add_device(bus, &config, &dev->device);
    dev->bus = result == ESP_OK ? bus : NULL;
    dev->address_7bit = address_7bit;
    dev->last_error = result;
    return result;
}

void vl53l4cd_platform_detach(Dev_t dev)
{
    if (!dev) return;
    if (dev->device) i2c_master_bus_rm_device(dev->device);
    dev->device = NULL;
    dev->bus = NULL;
}

static uint8_t write_bytes(Dev_t dev, uint16_t address, const uint8_t *data, size_t size)
{
    if (!dev || !dev->device || !data || size > 4) return platform_status(dev, ESP_ERR_INVALID_ARG);
    uint8_t buffer[6] = {(uint8_t)(address >> 8), (uint8_t)address};
    memcpy(buffer + 2, data, size);
    esp_err_t result = i2c_master_transmit(dev->device, buffer, size + 2,
                                           VL53L4CD_I2C_TIMEOUT_MS);
    // The ESP32 I2C peripheral can report BUS_BUSY when an immediately
    // following command starts during its STOP-to-idle handover. The VL ULD
    // performs many back-to-back one-byte operations, so give that handover a
    // full scheduler tick. At the configured 1 kHz tick rate this is 1 ms.
    vTaskDelay(pdMS_TO_TICKS(1));
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "write reg 0x%04x (%u byte%s): %s", address, (unsigned)size,
                 size == 1 ? "" : "s", esp_err_to_name(result));
    }
    return platform_status(dev, result);
}

static uint8_t read_bytes(Dev_t dev, uint16_t address, uint8_t *data, size_t size)
{
    if (!dev || !dev->device || !data || !size) return platform_status(dev, ESP_ERR_INVALID_ARG);
    const uint8_t request[2] = {(uint8_t)(address >> 8), (uint8_t)address};
    esp_err_t result = i2c_master_transmit_receive(dev->device, request,
                                                    sizeof(request), data, size,
                                                    VL53L4CD_I2C_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "read reg 0x%04x (%u byte%s): %s", address, (unsigned)size,
                 size == 1 ? "" : "s", esp_err_to_name(result));
    }
    return platform_status(dev, result);
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t address, uint8_t value)
{
    return write_bytes(dev, address, &value, 1);
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t address, uint16_t value)
{
    const uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return write_bytes(dev, address, data, sizeof(data));
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t address, uint32_t value)
{
    const uint8_t data[4] = {
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8), (uint8_t)value,
    };
    return write_bytes(dev, address, data, sizeof(data));
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t address, uint8_t *value)
{
    return read_bytes(dev, address, value, 1);
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t address, uint16_t *value)
{
    uint8_t data[2];
    const uint8_t result = read_bytes(dev, address, data, sizeof(data));
    if (!result) *value = ((uint16_t)data[0] << 8) | data[1];
    return result;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t address, uint32_t *value)
{
    uint8_t data[4];
    const uint8_t result = read_bytes(dev, address, data, sizeof(data));
    if (!result) {
        *value = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                 ((uint32_t)data[2] << 8) | data[3];
    }
    return result;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t time_ms)
{
    (void)dev;
    vTaskDelay(pdMS_TO_TICKS(time_ms ? time_ms : 1));
    return 0;
}
