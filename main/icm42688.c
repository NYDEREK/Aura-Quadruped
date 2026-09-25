#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "icm42688.h"

// This module talks only to the HXY ICM-42688P-compatible part.

enum {
    HXY_REG_WHO_AM_I = 0x01,
    HXY_REG_COM_CFG = 0x05,
    HXY_REG_HPF_LPF_CFG = 0x08,
    HXY_REG_DATA_STAT = 0x0B,
    HXY_REG_ACCEL_X_H = 0x0C,
    HXY_REG_TEMP_H = 0x22,
    HXY_REG_ACCEL_CONF = 0x40,
    HXY_REG_ACCEL_RANGE = 0x41,
    HXY_REG_GYRO_CONF = 0x42,
    HXY_REG_GYRO_RANGE = 0x43,
    HXY_REG_SOFT_RST = 0x4A,
    HXY_REG_I2C_UN = 0x6F,
    HXY_REG_PWR_CTRL = 0x7D,
    HXY_REG_SEG_SEL = 0x7F,
};

static const char *TAG = "icm42688";
static spi_device_handle_t imu_spi;
static icm_profile_t active_profile = ICM_PROFILE_AUTO;
static bool initialized;
static uint32_t sample_count;
static volatile uint32_t miso_edge_count;
static bool miso_isr_service_ready;

static int16_t decode_be16(const uint8_t *data);
static void imu_close_bus(void);

// The HXY reference initialization below selects +/-4 g and +/-2000 dps.
#define HXY_ACCEL_LSB_PER_G 8192.0f
#define HXY_GYRO_LSB_PER_DPS 16.4f

// HXY documents SPI with SPC idle high, SDI changing after the falling edge
// and sampled at the rising edge. This is SPI mode 3 at 500 kHz, well below
// the HXY 10 MHz limit.
static inline void hxy_bb_delay(void)
{
    esp_rom_delay_us(1);
}

static void hxy_bb_start(void)
{
    gpio_set_level(BOARD_IMU_CS, 1);
    gpio_set_level(BOARD_IMU_SCLK, 1);
    hxy_bb_delay();
    gpio_set_level(BOARD_IMU_CS, 0);
    hxy_bb_delay();
}

static void hxy_bb_finish(void)
{
    gpio_set_level(BOARD_IMU_SCLK, 1);
    hxy_bb_delay();
    gpio_set_level(BOARD_IMU_CS, 1);
    hxy_bb_delay();
}

static void hxy_bb_send_byte(uint8_t value)
{
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        gpio_set_level(BOARD_IMU_SCLK, 0);
        gpio_set_level(BOARD_IMU_MOSI, (value & mask) != 0);
        hxy_bb_delay();
        gpio_set_level(BOARD_IMU_SCLK, 1);
        hxy_bb_delay();
    }
}

static uint8_t hxy_bb_receive_byte(void)
{
    uint8_t value = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        gpio_set_level(BOARD_IMU_SCLK, 0);
        hxy_bb_delay();
        gpio_set_level(BOARD_IMU_SCLK, 1);
        hxy_bb_delay();
        value = (uint8_t)((value << 1) | gpio_get_level(BOARD_IMU_MOSI));
    }
    return value;
}

static void hxy_bb_data_output(void)
{
    gpio_set_direction(BOARD_IMU_MOSI, GPIO_MODE_OUTPUT);
}

static void hxy_bb_data_input(void)
{
    gpio_set_direction(BOARD_IMU_MOSI, GPIO_MODE_INPUT);
}

static void hxy_bb_write(uint8_t reg, uint8_t value)
{
    hxy_bb_data_output();
    hxy_bb_start();
    hxy_bb_send_byte(reg & 0x7F);
    hxy_bb_send_byte(value);
    hxy_bb_finish();
}

static void hxy_bb_read(uint8_t reg, uint8_t *data, size_t length)
{
    hxy_bb_data_output();
    hxy_bb_start();
    hxy_bb_send_byte(reg | 0x80);
    hxy_bb_data_input();
    for (size_t index = 0; index < length; ++index) data[index] = hxy_bb_receive_byte();
    hxy_bb_finish();
    hxy_bb_data_output();
}

static esp_err_t hxy_bb_init(void)
{
    imu_close_bus();
    const gpio_config_t output = {
        .pin_bit_mask = (1ULL << BOARD_IMU_CS) | (1ULL << BOARD_IMU_SCLK) |
                        (1ULL << BOARD_IMU_MOSI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output), TAG, "configure HXY 3-wire GPIO");
    gpio_set_level(BOARD_IMU_CS, 1);
    gpio_set_level(BOARD_IMU_SCLK, 1);
    gpio_set_level(BOARD_IMU_MOSI, 1);
    return ESP_OK;
}

// Probe the alternate HXY interface without writing an I2C register. Pin 14
// is SDA in I2C mode and pin 13 is SCL. The test emits only START, the slave
// address with the write bit, and STOP; it never sends a register or payload.
#define HXY_I2C_DELAY_US 12

static bool hxy_i2c_raise_scl(void)
{
    gpio_set_level(BOARD_IMU_SCLK, 1);
    for (unsigned attempt = 0; attempt < 20; ++attempt) {
        esp_rom_delay_us(HXY_I2C_DELAY_US);
        if (gpio_get_level(BOARD_IMU_SCLK)) return true;
    }
    return false;
}

static bool hxy_i2c_address_ack(uint8_t address)
{
    // Idle, then START.
    gpio_set_level(BOARD_IMU_MOSI, 1);
    if (!hxy_i2c_raise_scl()) return false;
    esp_rom_delay_us(HXY_I2C_DELAY_US);
    gpio_set_level(BOARD_IMU_MOSI, 0);
    esp_rom_delay_us(HXY_I2C_DELAY_US);
    gpio_set_level(BOARD_IMU_SCLK, 0);

    const uint8_t frame = (uint8_t)(address << 1); // write address only
    for (int bit = 7; bit >= 0; --bit) {
        gpio_set_level(BOARD_IMU_MOSI, (frame & (1U << bit)) != 0);
        esp_rom_delay_us(HXY_I2C_DELAY_US);
        if (!hxy_i2c_raise_scl()) goto stop;
        gpio_set_level(BOARD_IMU_SCLK, 0);
    }

    // Release SDA for the ACK clock. A selected I2C slave drives it low.
    gpio_set_level(BOARD_IMU_MOSI, 1);
    esp_rom_delay_us(HXY_I2C_DELAY_US);
    if (!hxy_i2c_raise_scl()) goto stop;
    const bool acknowledged = gpio_get_level(BOARD_IMU_MOSI) == 0;
    gpio_set_level(BOARD_IMU_SCLK, 0);

    // STOP.
    gpio_set_level(BOARD_IMU_MOSI, 0);
    (void)hxy_i2c_raise_scl();
    esp_rom_delay_us(HXY_I2C_DELAY_US);
    gpio_set_level(BOARD_IMU_MOSI, 1);
    return acknowledged;

stop:
    gpio_set_level(BOARD_IMU_SCLK, 0);
    gpio_set_level(BOARD_IMU_MOSI, 1);
    return false;
}

static const char *profile_name(icm_profile_t profile)
{
    switch (profile) {
        case ICM_PROFILE_HXY: return "HXY";
        default: return "none";
    }
}

const char *icm42688_profile_name(void)
{
    return profile_name(active_profile);
}

static esp_err_t imu_read_command(uint8_t command, uint8_t *data, size_t length)
{
    if (!imu_spi || !data || length == 0 || length > 32) return ESP_ERR_INVALID_ARG;

    // One transaction keeps CSB low for the command and every returned byte.
    uint8_t tx[33] = {0};
    uint8_t rx[33] = {0};
    tx[0] = command;
    spi_transaction_t transaction = {
        .length = (length + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    gpio_set_level(BOARD_IMU_CS, 0);
    const esp_err_t result = spi_device_polling_transmit(imu_spi, &transaction);
    gpio_set_level(BOARD_IMU_CS, 1);
    if (result == ESP_OK) memcpy(data, &rx[1], length);
    return result;
}

static esp_err_t imu_read(uint8_t reg, uint8_t *data, size_t length)
{
    // This is the convention used by HXY's supplied driver: bit 7 is R/W and
    // bits 6..0 form the register address.
    return imu_read_command(reg | 0x80, data, length);
}

static uint8_t reverse_bits(uint8_t value)
{
    value = (uint8_t)((value >> 4) | (value << 4));
    value = (uint8_t)(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
    return (uint8_t)(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
}

static esp_err_t imu_write(uint8_t reg, uint8_t value)
{
    spi_transaction_t transaction = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {reg & 0x7F, value},
    };
    gpio_set_level(BOARD_IMU_CS, 0);
    const esp_err_t result = spi_device_polling_transmit(imu_spi, &transaction);
    gpio_set_level(BOARD_IMU_CS, 1);
    return result;
}

void icm42688_prepare_spi_early(void)
{
    const gpio_config_t cs = {
        .pin_bit_mask = 1ULL << BOARD_IMU_CS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_set_level(BOARD_IMU_CS, 0);
    gpio_config(&cs);
    gpio_set_level(BOARD_IMU_CS, 0);
}

static void imu_close_bus(void)
{
    initialized = false;
    active_profile = ICM_PROFILE_AUTO;
    if (imu_spi) {
        spi_bus_remove_device(imu_spi);
        imu_spi = NULL;
    }
    spi_bus_free(SPI2_HOST);
}

static esp_err_t imu_open_bus(int clock_hz, int spi_mode)
{
    const spi_bus_config_t bus = {
        .mosi_io_num = BOARD_IMU_MOSI,
        .miso_io_num = BOARD_IMU_MISO,
        .sclk_io_num = BOARD_IMU_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 33,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG,
                        "initialize SPI2");

    const spi_device_interface_config_t device = {
        .mode = spi_mode,
        .clock_speed_hz = clock_hz,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    const esp_err_t result = spi_bus_add_device(SPI2_HOST, &device, &imu_spi);
    if (result != ESP_OK) spi_bus_free(SPI2_HOST);
    return result;
}

static esp_err_t open_profile(int clock_hz, int spi_mode)
{
    imu_close_bus();
    icm42688_prepare_spi_early();
    gpio_set_level(BOARD_IMU_CS, 1);
    return imu_open_bus(clock_hz, spi_mode);
}

static esp_err_t configure_hxy(void)
{
    // This is the complete SPI sequence supplied by HXY for this specific
    // part. It is intentionally not derived from the TDK ICM-42688-P map.
    // The BOOT/soft-reset phase reloads the chip's factory trim values.
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SEG_SEL, 0x00), TAG, "HXY general bank");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_PWR_CTRL, 0x00), TAG, "HXY power down");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_HPF_LPF_CFG, 0x87), TAG, "HXY reset flag");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_COM_CFG, 0x80), TAG, "HXY reload trim");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SOFT_RST, 0xA5), TAG, "HXY soft reset");
    vTaskDelay(pdMS_TO_TICKS(10));

    // HXY's SPI reference code explicitly disables I2C through special bank
    // 1 after the reset. This prevents the alternate interface from owning
    // the digital port when CSB is used for SPI.
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SEG_SEL, 0x00), TAG, "HXY return general bank");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_PWR_CTRL, 0x0E), TAG, "HXY PWR_CTRL");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SOFT_RST, 0x66), TAG, "HXY SPI mode prepare");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SEG_SEL, 0x83), TAG, "HXY select I2C control");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_I2C_UN, 0x04), TAG, "HXY disable I2C");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SEG_SEL, 0x00), TAG, "HXY leave I2C control");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_SOFT_RST, 0x00), TAG, "HXY finish SPI prepare");

    // HXY reference configuration: 100 Hz, +/-4 g, +/-2000 dps.
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_ACCEL_CONF, 0x88), TAG, "HXY ACC_CONF");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_ACCEL_RANGE, 0x01), TAG, "HXY ACC_RANGE");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_GYRO_CONF, 0xC8), TAG, "HXY GYR_CONF");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_GYRO_RANGE, 0x00), TAG, "HXY GYR_RANGE");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_COM_CFG, 0x50), TAG, "HXY COM_CFG");
    ESP_RETURN_ON_ERROR(imu_write(HXY_REG_HPF_LPF_CFG, 0x05), TAG, "HXY filter");
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

static void log_hxy_configuration_readback(void)
{
    uint8_t pwr = 0;
    uint8_t com = 0;
    uint8_t hpf = 0;
    uint8_t acc_conf = 0;
    uint8_t acc_range = 0;
    uint8_t gyr_conf = 0;
    uint8_t gyr_range = 0;
    uint8_t status = 0;

    if (imu_read(HXY_REG_PWR_CTRL, &pwr, 1) != ESP_OK ||
        imu_read(HXY_REG_COM_CFG, &com, 1) != ESP_OK ||
        imu_read(HXY_REG_HPF_LPF_CFG, &hpf, 1) != ESP_OK ||
        imu_read(HXY_REG_ACCEL_CONF, &acc_conf, 1) != ESP_OK ||
        imu_read(HXY_REG_ACCEL_RANGE, &acc_range, 1) != ESP_OK ||
        imu_read(HXY_REG_GYRO_CONF, &gyr_conf, 1) != ESP_OK ||
        imu_read(HXY_REG_GYRO_RANGE, &gyr_range, 1) != ESP_OK ||
        imu_read(HXY_REG_DATA_STAT, &status, 1) != ESP_OK) {
        ESP_LOGW(TAG, "HXY configuration readback failed at SPI driver level");
        return;
    }

    ESP_LOGW(TAG,
             "HXY readback: PWR=%02x COM=%02x HPF=%02x ACC=%02x/%02x GYR=%02x/%02x STAT=%02x",
             pwr, com, hpf, acc_conf, acc_range, gyr_conf, gyr_range, status);
}

static esp_err_t start_hxy_unchecked(uint8_t *identity)
{
    if (identity) *identity = 0;
    // HXY page 8 shows SPC idle high and sampling at the rising edge: SPI
    // mode 3. The HXY datasheet's documented start sequence is used directly;
    // no foreign register map or identity value participates in this path.
    ESP_RETURN_ON_ERROR(open_profile(1000000, 3), TAG, "open HXY SPI mode 3");
    ESP_RETURN_ON_ERROR(configure_hxy(), TAG, "configure HXY");
    vTaskDelay(pdMS_TO_TICKS(100));
    log_hxy_configuration_readback();
    active_profile = ICM_PROFILE_HXY;
    initialized = true;
    return ESP_OK;
}

esp_err_t icm42688_init(uint8_t *who_am_i)
{
    return start_hxy_unchecked(who_am_i);
}

esp_err_t icm42688_reinitialize(uint8_t *who_am_i)
{
    return start_hxy_unchecked(who_am_i);
}

esp_err_t icm42688_force_hxy_diagnostic(imu_sample_t *sample, uint8_t *data_status)
{
    if (!sample || !data_status) return ESP_ERR_INVALID_ARG;

    // This path intentionally does not read or compare WHO_AM_I. It is used
    // only to establish whether a device that follows the HXY register map
    // starts producing changing acceleration/gyro data after its documented
    // power and SPI configuration sequence.
    ESP_RETURN_ON_ERROR(start_hxy_unchecked(NULL), TAG, "configure unchecked HXY");

    uint8_t status = 0;
    uint8_t motion[12] = {0};
    uint8_t temperature[2] = {0};
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_DATA_STAT, &status, sizeof(status)), TAG,
                        "HXY diagnostic status");
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_ACCEL_X_H, motion, sizeof(motion)), TAG,
                        "HXY diagnostic motion");
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_TEMP_H, temperature, sizeof(temperature)), TAG,
                        "HXY diagnostic temperature");

    memset(sample, 0, sizeof(*sample));
    for (int axis = 0; axis < 3; ++axis) {
        sample->accel_g[axis] = decode_be16(&motion[axis * 2]) / HXY_ACCEL_LSB_PER_G;
        sample->gyro_dps[axis] = decode_be16(&motion[6 + axis * 2]) / HXY_GYRO_LSB_PER_DPS;
    }
    sample->temperature_c = decode_be16(temperature) / 512.0f + 23.0f;
    sample->sample_count = ++sample_count;
    sample->sample_time_us = esp_timer_get_time();
    *data_status = status;

    // Keep this profile active after the diagnostic. The normal periodic read
    // still requires DATA_STAT bits 0 and 1, so fabricated frames are not
    // published as normal IMU telemetry.
    active_profile = ICM_PROFILE_HXY;
    initialized = true;
    ESP_LOGW(TAG, "HXY blind diagnostic: DATA_STAT=0x%02x", status);
    return ESP_OK;
}

esp_err_t icm42688_force_hxy_3wire_diagnostic(imu_sample_t *sample, uint8_t *data_status)
{
    if (!sample || !data_status) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(hxy_bb_init(), TAG, "initialize HXY 3-wire diagnostic");

    // Start in default 4-wire mode, select HXY 3-wire mode (SIM=1), and then
    // configure and read through SDX. The final write restores 4-wire mode.
    hxy_bb_write(HXY_REG_PWR_CTRL, 0x0E);
    vTaskDelay(pdMS_TO_TICKS(10));
    hxy_bb_write(HXY_REG_COM_CFG, 0x51); // BDU + auto increment + 3-wire SPI
    hxy_bb_write(HXY_REG_ACCEL_CONF, 0xA8);
    hxy_bb_write(HXY_REG_ACCEL_RANGE, 0x00);
    hxy_bb_write(HXY_REG_GYRO_CONF, 0xA8);
    hxy_bb_write(HXY_REG_GYRO_RANGE, 0x03);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t status = 0;
    uint8_t motion[12] = {0};
    uint8_t temperature[2] = {0};
    hxy_bb_read(HXY_REG_DATA_STAT, &status, sizeof(status));
    hxy_bb_read(HXY_REG_ACCEL_X_H, motion, sizeof(motion));
    hxy_bb_read(HXY_REG_TEMP_H, temperature, sizeof(temperature));
    hxy_bb_write(HXY_REG_COM_CFG, 0x50); // Restore default 4-wire SPI.

    memset(sample, 0, sizeof(*sample));
    for (int axis = 0; axis < 3; ++axis) {
        sample->accel_g[axis] = decode_be16(&motion[axis * 2]) / HXY_ACCEL_LSB_PER_G;
        sample->gyro_dps[axis] = decode_be16(&motion[6 + axis * 2]) / HXY_GYRO_LSB_PER_DPS;
    }
    sample->temperature_c = decode_be16(temperature) / 512.0f + 23.0f;
    sample->sample_count = ++sample_count;
    sample->sample_time_us = esp_timer_get_time();
    *data_status = status;
    initialized = false;
    active_profile = ICM_PROFILE_AUTO;
    ESP_LOGW(TAG, "HXY 3-wire diagnostic: DATA_STAT=0x%02x", status);
    return ESP_OK;
}

static int16_t decode_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static esp_err_t read_hxy(imu_sample_t *sample)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_DATA_STAT, &status, 1), TAG, "HXY DATA_STAT");
    if ((status & 0x03) != 0x03) return ESP_ERR_NOT_FINISHED;

    uint8_t motion[12] = {0};
    uint8_t temperature[2] = {0};
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_ACCEL_X_H, motion, sizeof(motion)), TAG,
                        "HXY motion data");
    ESP_RETURN_ON_ERROR(imu_read(HXY_REG_TEMP_H, temperature, sizeof(temperature)), TAG,
                        "HXY temperature");
    for (int axis = 0; axis < 3; ++axis) {
        sample->accel_g[axis] = decode_be16(&motion[axis * 2]) / HXY_ACCEL_LSB_PER_G;
        sample->gyro_dps[axis] = decode_be16(&motion[6 + axis * 2]) / HXY_GYRO_LSB_PER_DPS;
    }
    sample->temperature_c = decode_be16(temperature) / 512.0f + 23.0f;
    return ESP_OK;
}

static void IRAM_ATTR miso_edge_isr(void *argument)
{
    (void)argument;
    ++miso_edge_count;
}

static esp_err_t count_miso_edges(uint8_t *data_status, uint32_t *edges)
{
    ESP_RETURN_ON_ERROR(open_profile(10000, 3), TAG, "open slow HXY SPI");

    if (!miso_isr_service_ready) {
        const esp_err_t install = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (install != ESP_OK && install != ESP_ERR_INVALID_STATE) {
            imu_close_bus();
            return install;
        }
        miso_isr_service_ready = true;
    }
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(BOARD_IMU_MISO, GPIO_INTR_ANYEDGE), TAG,
                        "enable MISO edge counter");
    const esp_err_t add_handler = gpio_isr_handler_add(BOARD_IMU_MISO, miso_edge_isr, NULL);
    if (add_handler != ESP_OK && add_handler != ESP_ERR_INVALID_STATE) {
        gpio_set_intr_type(BOARD_IMU_MISO, GPIO_INTR_DISABLE);
        imu_close_bus();
        return add_handler;
    }

    miso_edge_count = 0;
    const esp_err_t result = imu_read(HXY_REG_DATA_STAT, data_status, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    *edges = miso_edge_count;
    gpio_set_intr_type(BOARD_IMU_MISO, GPIO_INTR_DISABLE);
    gpio_isr_handler_remove(BOARD_IMU_MISO);
    imu_close_bus();
    return result;
}

esp_err_t icm42688_miso_activity_test(uint8_t *who_am_i)
{
    if (!who_am_i) return ESP_ERR_INVALID_ARG;
    uint8_t data_status = 0;
    uint32_t hxy_edges = 0;
    const esp_err_t hxy_result = count_miso_edges(&data_status, &hxy_edges);
    ESP_LOGW(TAG, "HXY MISO activity at 10 kHz: DATA_STAT=0x%02x edges=%lu (%s)",
             data_status, (unsigned long)hxy_edges, esp_err_to_name(hxy_result));
    const esp_err_t result = icm42688_reinitialize(who_am_i);
    return hxy_result != ESP_OK ? hxy_result : result;
}

esp_err_t icm42688_hxy_i2c_address_probe(void)
{
    imu_close_bus();
    gpio_set_level(BOARD_IMU_CS, 1); // CSB high selects the alternate interface.

    const gpio_config_t pins = {
        .pin_bit_mask = (1ULL << BOARD_IMU_MOSI) | (1ULL << BOARD_IMU_SCLK),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pins), TAG, "configure HXY I2C probe pins");
    const bool address_18 = hxy_i2c_address_ack(0x18);
    const bool address_19 = hxy_i2c_address_ack(0x19);
    ESP_LOGW(TAG, "HXY I2C address-only probe: 0x18=%s 0x19=%s",
             address_18 ? "ACK" : "NACK", address_19 ? "ACK" : "NACK");

    // The regular SPI path owns these pins again after the diagnostic.
    gpio_reset_pin(BOARD_IMU_MOSI);
    gpio_reset_pin(BOARD_IMU_SCLK);
    const esp_err_t restore = open_profile(1000000, 3);
    if (restore == ESP_OK) {
        active_profile = ICM_PROFILE_HXY;
        initialized = true;
    }
    return restore;
}

typedef struct {
    const char *name;
    int spi_mode;
    int clock_hz;
    bool lsb_first_command;
} hxy_scan_variant_t;

static uint8_t hxy_scan_command(uint8_t reg, bool lsb_first_command)
{
    if (!lsb_first_command) return reg | 0x80;

    // The prose in the HXY datasheet labels the first *wire* bit as bit 0
    // (R/W). ESP32 sends each byte MSB-first, hence the reversal below. This
    // is a diagnostic hypothesis only; HXY's own driver uses the normal form.
    return reverse_bits((uint8_t)((reg << 1) | 0x01));
}

static uint32_t hxy_scan_hash(const uint8_t *data, size_t length)
{
    uint32_t hash = 2166136261UL;
    for (size_t index = 0; index < length; ++index) {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    return hash;
}

static unsigned hxy_scan_unique_count(const uint8_t *data, size_t length)
{
    bool seen[256] = {0};
    unsigned count = 0;
    for (size_t index = 0; index < length; ++index) {
        if (!seen[data[index]]) {
            seen[data[index]] = true;
            ++count;
        }
    }
    return count;
}

esp_err_t icm42688_hxy_read_only_scan(void)
{
    // This test performs no IMU writes. It only sends read frames for each
    // possible 7-bit register number in the current HXY bank (0x00..0x7f).
    // Each map is acquired twice to distinguish a static MISO level from a
    // genuine, register-dependent response.
    static const hxy_scan_variant_t variants[] = {
        {"normal, mode 3, 1 MHz", 3, 1000000, false},
        {"normal, mode 0, 1 MHz", 0, 1000000, false},
        {"bit0-first, mode 3, 1 MHz", 3, 1000000, true},
        {"bit0-first, mode 0, 1 MHz", 0, 1000000, true},
        {"normal, mode 3, 100 kHz", 3, 100000, false},
        {"normal, mode 0, 100 kHz", 0, 100000, false},
        {"bit0-first, mode 3, 100 kHz", 3, 100000, true},
        {"bit0-first, mode 0, 100 kHz", 0, 100000, true},
    };
    uint8_t first[0x80] = {0};
    uint8_t second[0x80] = {0};
    esp_err_t result = ESP_OK;

    ESP_LOGW(TAG, "HXY read-only scan: 128 registers x two passes; no IMU writes");
    for (size_t variant = 0; variant < sizeof(variants) / sizeof(variants[0]); ++variant) {
        const hxy_scan_variant_t *current = &variants[variant];
        result = open_profile(current->clock_hz, current->spi_mode);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "HXY scan %s: cannot open SPI (%s)", current->name,
                     esp_err_to_name(result));
            continue;
        }

        for (uint16_t reg = 0; reg < 0x80; ++reg) {
            const esp_err_t read_result = imu_read_command(
                hxy_scan_command((uint8_t)reg, current->lsb_first_command), &first[reg], 1);
            if (read_result != ESP_OK && result == ESP_OK) result = read_result;
        }
        for (uint16_t reg = 0; reg < 0x80; ++reg) {
            const esp_err_t read_result = imu_read_command(
                hxy_scan_command((uint8_t)reg, current->lsb_first_command), &second[reg], 1);
            if (read_result != ESP_OK && result == ESP_OK) result = read_result;
        }

        unsigned stable = 0;
        for (size_t reg = 0; reg < sizeof(first); ++reg) stable += first[reg] == second[reg];
        ESP_LOGW(TAG,
                 "HXY scan %s: stable=%u/128 unique=%u hash=%08lx/%08lx",
                 current->name, stable, hxy_scan_unique_count(first, sizeof(first)),
                 (unsigned long)hxy_scan_hash(first, sizeof(first)),
                 (unsigned long)hxy_scan_hash(second, sizeof(second)));
        for (unsigned row = 0; row < 8; ++row) {
            char values[16 * 3 + 1] = {0};
            char *cursor = values;
            for (unsigned column = 0; column < 16; ++column) {
                const unsigned reg = row * 16 + column;
                cursor += snprintf(cursor, (size_t)(values + sizeof(values) - cursor),
                                   "%02x%s", first[reg], column == 15 ? "" : " ");
            }
            ESP_LOGW(TAG, "HXY scan %s %02x-%02x: %s", current->name, row * 16,
                     row * 16 + 15, values);
        }
    }

    // Return the bus to the normal read-only SPI framing. No configuration is
    // written here and robot/servo state is never touched by this diagnostic.
    const esp_err_t restore = open_profile(1000000, 3);
    if (restore == ESP_OK) {
        active_profile = ICM_PROFILE_HXY;
        initialized = true;
    } else if (result == ESP_OK) {
        result = restore;
    }
    return result;
}

esp_err_t icm42688_read(imu_sample_t *sample)
{
    if (!initialized || !sample) return ESP_ERR_INVALID_STATE;

    imu_sample_t next = {0};
    const esp_err_t result = read_hxy(&next);
    if (result != ESP_OK) return result;
    next.sample_count = ++sample_count;
    next.sample_time_us = esp_timer_get_time();
    *sample = next;
    return ESP_OK;
}
