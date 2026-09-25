#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "attitude_filter.h"
#include "board.h"
#include "board_i2c.h"
#include "mpu6050.h"

enum {
    MPU6050_ADDRESS_LOW = 0x68,
    MPU6050_ADDRESS_HIGH = 0x69,
    MPU6050_REG_SMPLRT_DIV = 0x19,
    MPU6050_REG_CONFIG = 0x1a,
    MPU6050_REG_GYRO_CONFIG = 0x1b,
    MPU6050_REG_ACCEL_CONFIG = 0x1c,
    MPU6050_REG_INT_PIN_CFG = 0x37,
    MPU6050_REG_INT_ENABLE = 0x38,
    MPU6050_REG_INT_STATUS = 0x3a,
    MPU6050_REG_ACCEL_XOUT_H = 0x3b,
    MPU6050_REG_PWR_MGMT_1 = 0x6b,
    MPU6050_REG_WHO_AM_I = 0x75,
    MPU6050_DATA_READY = 0x01,
};

#define MPU6050_I2C_HZ 100000
#define MPU6050_TIMEOUT_MS 20
#define MPU6050_ACCEL_LSB_PER_G 2048.0f        // +/-16 g, GRUZIK3 config
#define MPU6050_GYRO_LSB_PER_DPS 16.4f          // +/-2000 dps, GRUZIK3 config

static SemaphoreHandle_t mutex;
static i2c_master_dev_handle_t device;
static uint8_t address;
static imu_sample_t latest_sample;
static mpu6050_attitude_t latest_attitude;
static attitude_filter_t filter;

static int16_t decode_be16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] << 8 | bytes[1]);
}

static esp_err_t read_register(uint8_t reg, uint8_t *data, size_t size)
{
    // Match the proven GRUZIK3 STM32 driver exactly: write the register
    // address, issue STOP, then start a separate receive transaction.
    esp_err_t result = i2c_master_transmit(device, &reg, 1, 10);
    if (result != ESP_OK) return result;
    return i2c_master_receive(device, data, size, 100);
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(device, bytes, sizeof(bytes), 100);
}

static void release_device(void)
{
    if (device) (void)i2c_master_bus_rm_device(device);
    device = NULL;
    address = 0;
}

static esp_err_t configure_interrupt_pin(void)
{
    // S1 XSHUT is physically reused as the MPU6050 DATA_RDY output. It must
    // be input-only: driving it would fight the MPU output stage.
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BOARD_XSHUT_1,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static esp_err_t configure_locked(uint8_t *who_am_i)
{
    if (who_am_i) *who_am_i = 0;
    release_device();
    attitude_filter_reset(&filter);
    latest_sample = (imu_sample_t){0};
    latest_attitude = (mpu6050_attitude_t){0};
    esp_err_t result = configure_interrupt_pin();
    if (result != ESP_OK) return result;
    result = board_i2c_init();
    if (result != ESP_OK) return result;

    // MPU-6050 register map: AD0 selects 0x68 or 0x69. WHO_AM_I is 0x68.
    uint8_t identity = 0;
    esp_err_t last_error = ESP_ERR_NOT_FOUND;
    for (uint8_t candidate = MPU6050_ADDRESS_LOW;
         candidate <= MPU6050_ADDRESS_HIGH; ++candidate) {
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = candidate,
            .scl_speed_hz = MPU6050_I2C_HZ,
        };
        result = i2c_master_bus_add_device(board_i2c_bus(), &config, &device);
        if (result != ESP_OK) {
            last_error = result;
            continue;
        }
        address = candidate;
        identity = 0;
        result = read_register(MPU6050_REG_WHO_AM_I, &identity, 1);
        printf("MPU6050 I2C 0x%02x: WHO_AM_I 0x%02x (%s)\n", candidate, identity,
               esp_err_to_name(result));
        if (who_am_i) *who_am_i = identity;
        if (result == ESP_OK && identity == MPU6050_ADDRESS_LOW) break;
        last_error = result == ESP_OK ? ESP_ERR_INVALID_RESPONSE : result;
        release_device();
    }
    if (!device) return last_error;
    result = write_register(MPU6050_REG_PWR_MGMT_1, 0x80); // reset
    if (result != ESP_OK) goto failed;
    vTaskDelay(pdMS_TO_TICKS(100));
    // Exact GRUZIK3 main.c configuration, not just the generic driver:
    // Internal 8 MHz clock, DLPF 184/188 Hz, gyro +/-2000 dps,
    // accel +/-16 g, and sample divider 0x04.
    result = write_register(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (result != ESP_OK) goto failed;
    vTaskDelay(pdMS_TO_TICKS(100));
    result = write_register(MPU6050_REG_CONFIG, 0x01);
    if (result != ESP_OK) goto failed;
    result = write_register(MPU6050_REG_GYRO_CONFIG, 0x18);
    if (result != ESP_OK) goto failed;
    result = write_register(MPU6050_REG_ACCEL_CONFIG, 0x18);
    if (result != ESP_OK) goto failed;
    result = write_register(MPU6050_REG_SMPLRT_DIV, 0x04);
    if (result != ESP_OK) goto failed;
    // DATA_RDY on S1/GPIO14 remains electrically passive. Polling the data
    // registers at 100 Hz is fully supported by the MPU6050; it also keeps
    // the basic bring-up independent from the optional interrupt wire.
    latest_attitude.address_7bit = address;
    return ESP_OK;

failed:
    release_device();
    return result;
}

esp_err_t mpu6050_init(uint8_t *who_am_i)
{
    if (!mutex) {
        mutex = xSemaphoreCreateMutex();
        if (!mutex) return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(250)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const esp_err_t result = configure_locked(who_am_i);
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t mpu6050_reinitialize(uint8_t *who_am_i)
{
    return mpu6050_init(who_am_i);
}

esp_err_t mpu6050_read(imu_sample_t *sample)
{
    if (!sample || !mutex || xSemaphoreTake(mutex, pdMS_TO_TICKS(12)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (!device) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t bytes[14] = {0};
    esp_err_t result = read_register(MPU6050_REG_ACCEL_XOUT_H, bytes, sizeof(bytes));
    if (result != ESP_OK) {
        xSemaphoreGive(mutex);
        return result;
    }
    // Mounting convention for S1: module X points forward, module Y left and
    // module Z up. Convert its flat-board axes to Aura X-forward/Y-up/Z-left.
    const float raw_accel[3] = {
        (float)decode_be16(bytes + 0) / MPU6050_ACCEL_LSB_PER_G,
        (float)decode_be16(bytes + 2) / MPU6050_ACCEL_LSB_PER_G,
        (float)decode_be16(bytes + 4) / MPU6050_ACCEL_LSB_PER_G,
    };
    const float raw_gyro[3] = {
        (float)decode_be16(bytes + 8) / MPU6050_GYRO_LSB_PER_DPS,
        (float)decode_be16(bytes + 10) / MPU6050_GYRO_LSB_PER_DPS,
        (float)decode_be16(bytes + 12) / MPU6050_GYRO_LSB_PER_DPS,
    };
    imu_sample_t next = {
        .accel_g = {raw_accel[0], raw_accel[2], raw_accel[1]},
        .gyro_dps = {raw_gyro[0], raw_gyro[2], raw_gyro[1]},
        .temperature_c = (float)decode_be16(bytes + 6) / 340.0f + 36.53f,
        .sample_count = latest_sample.sample_count + 1,
        .sample_time_us = esp_timer_get_time(),
    };
    float roll = 0.0f, pitch = 0.0f;
    const bool attitude_ok = attitude_filter_update(&filter, next.accel_g, next.gyro_dps,
                                                    next.sample_time_us, &roll, &pitch);
    latest_sample = next;
    latest_attitude = (mpu6050_attitude_t){
        .valid = attitude_ok,
        .address_7bit = address,
        .sample_count = next.sample_count,
        .sample_time_us = next.sample_time_us,
        .roll_radians = roll,
        .pitch_radians = pitch,
        .roll_rate_rad_s = next.gyro_dps[0] * 0.01745329252f,
        .pitch_rate_rad_s = next.gyro_dps[2] * 0.01745329252f,
    };
    *sample = next;
    xSemaphoreGive(mutex);
    return ESP_OK;
}

void mpu6050_get_attitude(mpu6050_attitude_t *attitude)
{
    if (!attitude) return;
    *attitude = (mpu6050_attitude_t){0};
    if (!mutex || xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
    *attitude = latest_attitude;
    xSemaphoreGive(mutex);
}
