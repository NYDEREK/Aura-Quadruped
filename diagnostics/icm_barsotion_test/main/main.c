#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ICM42688_Barsotion.h"

/* Aura Main Board: these are the only GPIOs used by this diagnostic image. */
#define LED_GPIO   GPIO_NUM_33
#define ICM_MISO   GPIO_NUM_19
#define ICM_MOSI   GPIO_NUM_23
#define ICM_SCLK   GPIO_NUM_22
#define ICM_CS     GPIO_NUM_21

static const char *TAG = "icm-test";
static ICM42688_t imu;
static bool imu_ready;
static spi_device_handle_t hxy_spi;
static spi_device_handle_t gruzik_spi;

/*
 * HXY is not a 4-wire drop-in replacement for the TDK part.  Its primary
 * interface uses pin 14 as a shared, bidirectional SPI data line.  This
 * probe deliberately uses only CS=21, SCLK=22 and the pin-14 trace=GPIO23.
 */
static void hxy_3wire_write_byte(uint8_t value, bool cpol, bool cpha)
{
    for (int bit = 7; bit >= 0; --bit) {
        if (cpha) {
            gpio_set_level(ICM_SCLK, !cpol);
            gpio_set_level(ICM_MOSI, (value >> bit) & 1u);
            esp_rom_delay_us(5);
            gpio_set_level(ICM_SCLK, cpol);
        } else {
            gpio_set_level(ICM_MOSI, (value >> bit) & 1u);
            esp_rom_delay_us(5);
            gpio_set_level(ICM_SCLK, !cpol);
            esp_rom_delay_us(5);
            gpio_set_level(ICM_SCLK, cpol);
        }
        esp_rom_delay_us(5);
    }
}

static uint8_t hxy_3wire_read_register(uint8_t reg, bool cpol, bool cpha)
{
    uint8_t value = 0;
    gpio_set_direction(ICM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_level(ICM_SCLK, cpol);
    gpio_set_level(ICM_CS, 0);
    hxy_3wire_write_byte(reg | 0x80u, cpol, cpha);

    gpio_set_direction(ICM_MOSI, GPIO_MODE_INPUT);
    for (int bit = 7; bit >= 0; --bit) {
        gpio_set_level(ICM_SCLK, !cpol);
        esp_rom_delay_us(5);
        if (!cpha) {
            value |= (uint8_t)(gpio_get_level(ICM_MOSI) << bit);
        }
        gpio_set_level(ICM_SCLK, cpol);
        esp_rom_delay_us(5);
        if (cpha) {
            value |= (uint8_t)(gpio_get_level(ICM_MOSI) << bit);
        }
    }
    gpio_set_level(ICM_CS, 1);
    gpio_set_level(ICM_SCLK, cpol);
    return value;
}

/* HXY powers up with COM_CFG.SIM = 0, i.e. conventional 4-wire SPI. */
static uint8_t hxy_4wire_read_register(uint8_t reg, bool cpol, bool cpha)
{
    uint8_t value = 0;
    gpio_set_direction(ICM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(ICM_MISO, GPIO_MODE_INPUT);
    gpio_set_level(ICM_SCLK, cpol);
    gpio_set_level(ICM_CS, 0);
    hxy_3wire_write_byte(reg | 0x80u, cpol, cpha);

    for (int bit = 7; bit >= 0; --bit) {
        gpio_set_level(ICM_SCLK, !cpol);
        esp_rom_delay_us(5);
        if (!cpha) {
            value |= (uint8_t)(gpio_get_level(ICM_MISO) << bit);
        }
        gpio_set_level(ICM_SCLK, cpol);
        esp_rom_delay_us(5);
        if (cpha) {
            value |= (uint8_t)(gpio_get_level(ICM_MISO) << bit);
        }
    }
    gpio_set_level(ICM_CS, 1);
    gpio_set_level(ICM_SCLK, cpol);
    return value;
}

static void hxy_4wire_probe_forever(void)
{
    const gpio_config_t output_pins = {
        .pin_bit_mask = (1ULL << ICM_CS) | (1ULL << ICM_SCLK) | (1ULL << ICM_MOSI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    const gpio_config_t input_pins = {
        .pin_bit_mask = 1ULL << ICM_MISO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_pins));
    ESP_ERROR_CHECK(gpio_config(&input_pins));
    gpio_set_level(ICM_CS, 1);
    gpio_set_level(ICM_SCLK, 1);
    gpio_set_level(ICM_MOSI, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    printf("=== HXY isolated 4-wire SPI test: CS=21 SCLK=22 SDI(pin14)=23 SDO(pin1)=19 ===\\n");
    while (true) {
        const uint8_t who_mode0 = hxy_4wire_read_register(0x01, false, false);
        const uint8_t who_mode1 = hxy_4wire_read_register(0x01, false, true);
        const uint8_t who_mode2 = hxy_4wire_read_register(0x01, true, false);
        const uint8_t who_mode3 = hxy_4wire_read_register(0x01, true, true);
        const uint8_t cfg_mode3 = hxy_4wire_read_register(0x05, true, true);
        const uint8_t reg75_mode0 = hxy_4wire_read_register(0x75, false, false);
        const uint8_t reg75_mode3 = hxy_4wire_read_register(0x75, true, true);
        printf("HXY 4-wire WHO_AM_I (expected 0x6A): m0=0x%02X m1=0x%02X m2=0x%02X m3=0x%02X; COM_CFG(m3)=0x%02X (default 0x50); reg0x75 m0=0x%02X m3=0x%02X\\n",
               who_mode0, who_mode1, who_mode2, who_mode3, cfg_mode3, reg75_mode0, reg75_mode3);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* HXY selects I2C while CSB is high.  Probe both SA0 address variants. */
static esp_err_t hxy_i2c_read_register(uint8_t address, uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    i2c_master_start(command);
    i2c_master_write_byte(command, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, reg, true);
    i2c_master_start(command);
    i2c_master_write_byte(command, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(command, value, I2C_MASTER_NACK);
    i2c_master_stop(command);
    const esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, command, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(command);
    return result;
}

static void hxy_i2c_probe_forever(void)
{
    gpio_set_level(ICM_CS, 1);
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ICM_MOSI,
        .scl_io_num = ICM_SCLK,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 10000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0));
    vTaskDelay(pdMS_TO_TICKS(20));

    printf("=== HXY I2C test: CSB=high SCL(pin13)=22 SDA(pin14)=23 ===\\n");
    while (true) {
        uint8_t who_18 = 0, cfg_18 = 0, who_19 = 0, cfg_19 = 0;
        gpio_set_pull_mode(ICM_MISO, GPIO_PULLDOWN_ONLY);
        const esp_err_t who_18_result = hxy_i2c_read_register(0x18, 0x01, &who_18);
        const esp_err_t cfg_18_result = hxy_i2c_read_register(0x18, 0x05, &cfg_18);
        gpio_set_pull_mode(ICM_MISO, GPIO_PULLUP_ONLY);
        const esp_err_t who_19_result = hxy_i2c_read_register(0x19, 0x01, &who_19);
        const esp_err_t cfg_19_result = hxy_i2c_read_register(0x19, 0x05, &cfg_19);
        printf("HXY I2C: 0x18 WHO=%s:0x%02X CFG=%s:0x%02X | 0x19 WHO=%s:0x%02X CFG=%s:0x%02X (expected WHO=0x6A CFG=0x50)\\n",
               esp_err_to_name(who_18_result), who_18,
               esp_err_to_name(cfg_18_result), cfg_18,
               esp_err_to_name(who_19_result), who_19,
               esp_err_to_name(cfg_19_result), cfg_19);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void hxy_3wire_probe_forever(void)
{
    const gpio_config_t spi_pins = {
        .pin_bit_mask = (1ULL << ICM_CS) | (1ULL << ICM_SCLK) | (1ULL << ICM_MOSI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&spi_pins));
    gpio_set_level(ICM_CS, 1);
    gpio_set_level(ICM_SCLK, 1);
    gpio_set_level(ICM_MOSI, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    printf("=== HXY isolated 3-wire SPI test: CS=21 SCLK=22 SDIO(pin14)=23 ===\\n");
    while (true) {
        const uint8_t who_mode0 = hxy_3wire_read_register(0x01, false, false);
        const uint8_t who_mode1 = hxy_3wire_read_register(0x01, false, true);
        const uint8_t who_mode2 = hxy_3wire_read_register(0x01, true, false);
        const uint8_t who_mode3 = hxy_3wire_read_register(0x01, true, true);
        printf("HXY 3-wire WHO_AM_I (expected 0x6A): mode0=0x%02X mode1=0x%02X mode2=0x%02X mode3=0x%02X\\n",
               who_mode0, who_mode1, who_mode2, who_mode3);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*
 * Literal transaction shape used by GRUZIK4.0:
 *   CS low, send register address, receive a byte, CS high.
 * The first transfer deliberately has no RX buffer and the second one
 * deliberately has no TX buffer, matching HAL_SPI_Transmit + HAL_SPI_Receive.
 */
static uint8_t gruzik_read_register(uint8_t reg)
{
    const uint8_t address = (uint8_t)(reg | 0x80u);
    uint8_t value = 0;
    spi_transaction_t send_address = {
        .length = 8,
        .tx_buffer = &address,
    };
    spi_transaction_t receive_value = {
        .length = 8,
        .rxlength = 8,
        .rx_buffer = &value,
    };

    gpio_set_level(ICM_CS, 0);
    const esp_err_t send_result = spi_device_polling_transmit(gruzik_spi, &send_address);
    const esp_err_t receive_result = (send_result == ESP_OK)
        ? spi_device_polling_transmit(gruzik_spi, &receive_value)
        : send_result;
    gpio_set_level(ICM_CS, 1);

    if (receive_result != ESP_OK) {
        printf("GRUZIK transfer error: %s\\n", esp_err_to_name(receive_result));
    }
    return value;
}

static void gruzik_select_bank_zero(void)
{
    const uint8_t command[2] = {0x76, 0x00};
    spi_transaction_t transaction = {
        .length = sizeof(command) * 8,
        .tx_buffer = command,
    };

    gpio_set_level(ICM_CS, 0);
    const esp_err_t result = spi_device_polling_transmit(gruzik_spi, &transaction);
    gpio_set_level(ICM_CS, 1);
    if (result != ESP_OK) {
        printf("GRUZIK bank-select error: %s\\n", esp_err_to_name(result));
    }
}

static uint8_t hxy_read_register(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t transaction = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    /* HXY specifies CS-low / idle-high timing, i.e. SPI mode 3. */
    gpio_set_level(ICM_CS, 0);
    const esp_err_t result = spi_device_polling_transmit(hxy_spi, &transaction);
    gpio_set_level(ICM_CS, 1);
    if (result != ESP_OK) {
        printf("HXY transfer error: %s\n", esp_err_to_name(result));
    }
    return rx[1];
}

static void heartbeat_task(void *arg)
{
    (void)arg;
    while (true) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

static void print_registers(void)
{
    uint8_t who_am_i = 0;
    uint8_t device_config = 0;
    uint8_t power = 0;
    uint8_t int_status = 0;

    /* These calls are the unmodified Barsotion library SPI read transaction. */
    ICM42688_readWhoAmI(&imu, &who_am_i);
    imu.readRegister(ICM_0_DEVICE_CONFIG, &device_config);
    imu.readRegister(ICM_0_PWR_MGMT0, &power);
    imu.readRegister(ICM_0_INT_STATUS, &int_status);

    const uint8_t hxy_who_am_i = hxy_read_register(0x01);
    const uint8_t hxy_com_cfg = hxy_read_register(0x05);
    gruzik_select_bank_zero();
    const uint8_t gruzik_who_am_i = gruzik_read_register(0x75);
    printf("TDK probe: WHO_AM_I=0x%02X DEVICE_CONFIG=0x%02X PWR_MGMT0=0x%02X INT_STATUS=0x%02X init=%s\n",
           who_am_i, device_config, power, int_status, imu_ready ? "OK" : "FAILED");
    printf("HXY probe: WHO_AM_I=0x%02X COM_CFG=0x%02X (expected WHO_AM_I=0x6A)\n",
           hxy_who_am_i, hxy_com_cfg);
    printf("GRUZIK4.0 literal probe: BANK_SEL=0 then WHO_AM_I=0x%02X (expected 0x47)\n",
           gruzik_who_am_i);
}

static void imu_task(void *arg)
{
    (void)arg;
    while (true) {
        if (!imu_ready) {
            print_registers();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int32_t raw[6] = {0};
        ICM42688_readRegAG(&imu, raw);
        ICM42688_calculateGyro(&imu, raw);
        ICM42688_calculateAccel(&imu, raw + 3);
        printf("ICM data: gyro_dps=[%.2f, %.2f, %.2f] accel_g=[%.3f, %.3f, %.3f] total_g=%.3f\n",
               imu.gyro.x, imu.gyro.y, imu.gyro.z,
               imu.accel.x, imu.accel.y, imu.accel.z, imu.accel_total);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));
    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 2, NULL);

    /* Do this before any ESP-IDF SPI peripheral takes ownership of GPIO23. */
    hxy_4wire_probe_forever();

    /* Keep chip select inactive until the SPI driver assumes ownership. */
    gpio_config_t cs_config = {
        .pin_bit_mask = 1ULL << ICM_CS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_config));
    ESP_ERROR_CHECK(gpio_set_level(ICM_CS, 1));
    vTaskDelay(pdMS_TO_TICKS(20));

    printf("\\n=== Aura ICM-42688 isolated UART test ===\n");
    printf("SPI mode 0, 5 MHz, hardware CS; MISO=GPIO19 MOSI=GPIO23 SCLK=GPIO22 CS=GPIO21\n");
    printf("Library: Barsy-Barsevich/ICM42688_Barsotion. Expected TDK WHO_AM_I=0x47.\n");

    ICM42688_Config_t config = {
        .protocol = Hardware_SPI,
        .spi = {
            .host = SPI3_HOST,
            .miso_pin = ICM_MISO,
            .mosi_pin = ICM_MOSI,
            .sck_pin = ICM_SCLK,
            .cs_pin = ICM_CS,
            .sck_freq = 5000000,
        },
        .accel = {
            .enable = ENABLE_XA | ENABLE_YA | ENABLE_ZA,
            .mode = ACCEL_LN_MODE,
            .scale = ACCEL_FS_SEL_2G,
            .odr = ACCEL_ODR_100HZ,
        },
        .gyro = {
            .enable = ENABLE_XG | ENABLE_YG | ENABLE_ZG,
            .mode = GYRO_LN_MODE,
            .scale = GYRO_FS_SEL_250DPS,
            .odr = GYRO_ODR_100HZ,
        },
        .fifo = {
            .mode = FIFO_BYPASS_MODE,
        },
    };

    imu_ready = ICM42688_Init(&imu, &config);
    const spi_device_interface_config_t hxy_config = {
        .clock_speed_hz = 100000,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    const spi_device_interface_config_t gruzik_config = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &hxy_config, &hxy_spi));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &gruzik_config, &gruzik_spi));
    ESP_LOGI(TAG, "Barsotion ICM42688_Init: %s", imu_ready ? "success" : "failed");
    print_registers();
    xTaskCreate(imu_task, "icm_uart", 4096, NULL, 3, NULL);
}
