#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
    uint8_t address_7bit;
    esp_err_t last_error;
} vl53l4cd_platform_device_t;

typedef vl53l4cd_platform_device_t *Dev_t;

esp_err_t vl53l4cd_platform_attach(Dev_t dev, i2c_master_bus_handle_t bus,
                                  uint8_t address_7bit);
void vl53l4cd_platform_detach(Dev_t dev);

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t address, uint8_t *value);
uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t address, uint8_t value);
uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t address, uint16_t *value);
uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t address, uint16_t value);
uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t address, uint32_t *value);
uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t address, uint32_t value);
uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t time_ms);

