#include <math.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board.h"
#include "board_i2c.h"
#include "power.h"

static i2c_master_dev_handle_t ina;
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t calibration;
static esp_err_t ina_init_error = ESP_ERR_INVALID_STATE;
static esp_err_t adc_init_error = ESP_ERR_INVALID_STATE;
static uint16_t manufacturer, die;
static bool efuse_calibration;
static float servo_consumed_mAh;
static int64_t last_charge_sample_us;

static esp_err_t ina_read16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    esp_err_t result = i2c_master_transmit_receive(ina, &reg, 1, data, 2, 30);
    if (result == ESP_OK) *value = ((uint16_t)data[0] << 8) | data[1];
    return result;
}

static esp_err_t init_ina(void)
{
    esp_err_t result = board_i2c_init();
    if (result != ESP_OK) return result;
    // INA226 A0 and A1 are grounded. VL53L4CD devices share this bus.
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x40,
        .scl_speed_hz = 100000,
    };
    result = i2c_master_bus_add_device(board_i2c_bus(), &config, &ina);
    if (result != ESP_OK) return result;
    result = ina_read16(0xfe, &manufacturer);
    if (result != ESP_OK) return result;
    result = ina_read16(0xff, &die);
    if (result != ESP_OK) return result;
    if (manufacturer != 0x5449 || die != 0x2260) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t init_adc(void)
{
    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    esp_err_t result = adc_oneshot_new_unit(&unit, &adc);
    if (result != ESP_OK) return result;
    // ESP32 ADC1 channel 6 is GPIO34, connected only to the VIN divider.
    const adc_oneshot_chan_cfg_t channel = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    result = adc_oneshot_config_channel(adc, ADC_CHANNEL_6, &channel);
    if (result != ESP_OK) return result;
    adc_cali_line_fitting_efuse_val_t efuse;
    if (adc_cali_scheme_line_fitting_check_efuse(&efuse) == ESP_OK) {
        efuse_calibration = efuse != ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    }
    const adc_cali_line_fitting_config_t config = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12, .default_vref = 1100,
    };
    return adc_cali_create_scheme_line_fitting(&config, &calibration);
}

void power_init(void)
{
    ina_init_error = init_ina();
    adc_init_error = init_adc();
}

esp_err_t power_read_vin(float *voltage_v, int *raw_average)
{
    if (!voltage_v) return ESP_ERR_INVALID_ARG;
    *voltage_v = 0.0f;
    if (raw_average) *raw_average = 0;
    if (adc_init_error != ESP_OK) return adc_init_error;

    int total = 0;
    for (int i = 0; i < 16; ++i) {
        int raw;
        esp_err_t result = adc_oneshot_read(adc, ADC_CHANNEL_6, &raw);
        if (result != ESP_OK) return result;
        total += raw;
    }

    const int raw = total / 16;
    if (raw_average) *raw_average = raw;
    // Line-fitting calibration has a non-zero intercept. Do not turn a
    // grounded ADC reading into a fictitious VIN voltage.
    if (raw <= 4) return ESP_OK;

    int millivolts;
    esp_err_t result = adc_cali_raw_to_voltage(calibration, raw, &millivolts);
    if (result == ESP_OK)
        *voltage_v = millivolts * 0.001f * BOARD_BATTERY_DIVIDER;
    return result;
}

void power_read(power_sample_t *sample)
{
    memset(sample, 0, sizeof(*sample));
    sample->manufacturer_id = manufacturer;
    sample->die_id = die;
    sample->ina_error = ina_init_error;
    sample->battery_error = adc_init_error;
    sample->battery_calibrated = efuse_calibration;
    if (ina_init_error == ESP_OK) {
        uint16_t config = 0, shunt = 0, voltage = 0;
        sample->ina_error = ina_read16(0x00, &config);
        // The power-on mode is continuous shunt + bus conversion (7).
        // Report a changed mode rather than silently presenting stale readings.
        if (sample->ina_error == ESP_OK && (config & 7) != 7) sample->ina_error = ESP_ERR_INVALID_STATE;
        if (sample->ina_error == ESP_OK) sample->ina_error = ina_read16(0x01, &shunt);
        if (sample->ina_error == ESP_OK) sample->ina_error = ina_read16(0x02, &voltage);
        if (sample->ina_error == ESP_OK) {
            sample->shunt_mv = (int16_t)shunt * 0.0025f;
            sample->bus_v = voltage * 0.00125f;
            // Use measured shunt voltage, not the uncalibrated INA CURRENT register.
            sample->servo_current_a = sample->shunt_mv * 0.001f / BOARD_SHUNT_OHMS;
            sample->servo_input_power_w = sample->bus_v * sample->servo_current_a;
            const int64_t now = esp_timer_get_time();
            if (last_charge_sample_us) {
                const float elapsed_hours = (float)(now - last_charge_sample_us) / 3600000000.0f;
                // INA226 measures the servo supply shunt.  Do not subtract
                // possible regenerative current from consumed capacity.
                servo_consumed_mAh += fmaxf(0.0f, sample->servo_current_a) * 1000.0f * elapsed_hours;
            }
            last_charge_sample_us = now;
        }
    }
    sample->servo_consumed_mAh = servo_consumed_mAh;
    sample->battery_error = power_read_vin(&sample->battery_v, &sample->battery_adc_raw);
}
