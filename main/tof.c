#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "board.h"
#include "board_i2c.h"
#include "tof.h"
#include "vl53l4cd_uld/vl53l4cd_api.h"

#define TOF_DEFAULT_ADDRESS 0x29
#define TOF_MODEL_ID 0xebaa
#define TOF_DEFAULT_BUDGET_MS 50
#define TOF_DEFAULT_XTALK_KCPS 0
#define TOF_DEFAULT_SIGNAL_THRESHOLD_KCPS 1024
// This matches the shipped ULD configuration (0x00a0 in 14.2 format).
#define TOF_DEFAULT_SIGMA_THRESHOLD_MM 40
#define TOF_DEFAULT_DETECTION_LOW_MM 100
#define TOF_DEFAULT_DETECTION_HIGH_MM 300
#define TOF_POLL_MS 10

typedef struct {
    vl53l4cd_platform_device_t device;
    tof_config_t config;
    tof_sensor_state_t state;
    int xshut_gpio;
} tof_sensor_t;

static const char *TAG = "tof";
static const char *const address_keys[TOF_SENSOR_COUNT] = {"a0", "a1"};
static const char *const enabled_keys[TOF_SENSOR_COUNT] = {"e0", "e1"};
static const char *const budget_keys[TOF_SENSOR_COUNT] = {"b0", "b1"};
static const char *const period_keys[TOF_SENSOR_COUNT] = {"p0", "p1"};
static const char *const offset_keys[TOF_SENSOR_COUNT] = {"o0", "o1"};
static const char *const xtalk_keys[TOF_SENSOR_COUNT] = {"x0", "x1"};
static const char *const signal_keys[TOF_SENSOR_COUNT] = {"s0", "s1"};
static const char *const sigma_keys[TOF_SENSOR_COUNT] = {"g0", "g1"};
static const char *const detect_enabled_keys[TOF_SENSOR_COUNT] = {"d0", "d1"};
static const char *const detect_low_keys[TOF_SENSOR_COUNT] = {"l0", "l1"};
static const char *const detect_high_keys[TOF_SENSOR_COUNT] = {"h0", "h1"};
static const char *const detect_window_keys[TOF_SENSOR_COUNT] = {"w0", "w1"};
static SemaphoreHandle_t mutex;
static TaskHandle_t task_handle;
static tof_sensor_t sensors[TOF_SENSOR_COUNT];
static bool port_reserved[TOF_SENSOR_COUNT];

static esp_err_t configure_xshut_pins(void)
{
    uint64_t output_mask = 0;
    uint64_t input_mask = 0;
    const int pins[TOF_SENSOR_COUNT] = {BOARD_XSHUT_1, BOARD_XSHUT_2};
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index) {
        if (port_reserved[index]) input_mask |= 1ULL << pins[index];
        else output_mask |= 1ULL << pins[index];
    }
    if (output_mask) {
        const gpio_config_t output = {
            .pin_bit_mask = output_mask,
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        const esp_err_t result = gpio_config(&output);
        if (result != ESP_OK) return result;
    }
    if (input_mask) {
        const gpio_config_t input = {
            .pin_bit_mask = input_mask,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        const esp_err_t result = gpio_config(&input);
        if (result != ESP_OK) return result;
    }
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
        if (!port_reserved[index]) gpio_set_level(pins[index], 0);
    return ESP_OK;
}

static void log_i2c_levels(const char *stage)
{
    // This is deliberately a warning: production logging is configured at
    // warning level, and these levels are the key distinction between an
    // absent sensor and a bus being electrically held low.
    ESP_LOGW(TAG, "%s: SDA(GPIO%d)=%d SCL(GPIO%d)=%d", stage,
             BOARD_I2C_SDA, gpio_get_level(BOARD_I2C_SDA),
             BOARD_I2C_SCL, gpio_get_level(BOARD_I2C_SCL));
}

// A deliberately small, open-drain I2C address probe used before the hardware
// I2C driver is initialized. It distinguishes an ACK from a controller-level
// timeout and also lets us test the unlikely physical SDA/SCL swap safely.
#define WIRE_TEST_DELAY_US 10

static bool wire_test_raise_scl(int scl)
{
    gpio_set_level(scl, 1);
    for (int i = 0; i < 20; ++i) {
        esp_rom_delay_us(WIRE_TEST_DELAY_US);
        if (gpio_get_level(scl)) return true;
    }
    return false;
}

static esp_err_t wire_test_probe(int sda, int scl, uint8_t address, int *timeout_phase,
                                 int *idle_sda, int *idle_scl)
{
    *timeout_phase = -3;  // Idle-line check.
    // GPIO routing can persist across an ESP reset. Detach any prior I2C
    // matrix route before using the pins as ordinary open-drain GPIO.
    ESP_RETURN_ON_ERROR(gpio_reset_pin(sda), TAG, "reset SDA GPIO");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(scl), TAG, "reset SCL GPIO");
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    const gpio_config_t pins = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&pins);
    if (result != ESP_OK) return result;

    // gpio_config may change the output latch; release the open-drain pins
    // after configuring them so the external 5.1 kOhm pull-ups can raise them.
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(WIRE_TEST_DELAY_US);
    *idle_sda = gpio_get_level(sda);
    *idle_scl = gpio_get_level(scl);
    if (!*idle_sda || !*idle_scl) return ESP_ERR_TIMEOUT;

    // START followed by the 7-bit address and write bit.
    gpio_set_level(sda, 0);
    esp_rom_delay_us(WIRE_TEST_DELAY_US);
    gpio_set_level(scl, 0);
    for (int bit = 7; bit >= 0; --bit) {
        gpio_set_level(sda, ((address << 1) & (1 << bit)) != 0);
        esp_rom_delay_us(WIRE_TEST_DELAY_US);
        if (!wire_test_raise_scl(scl)) {
            *timeout_phase = bit;
            result = ESP_ERR_TIMEOUT;
            goto stop;
        }
        gpio_set_level(scl, 0);
    }

    // Release SDA for the ninth, ACK bit. A slave acknowledges by pulling it low.
    gpio_set_level(sda, 1);
    esp_rom_delay_us(WIRE_TEST_DELAY_US);
    if (!wire_test_raise_scl(scl)) {
        *timeout_phase = -1;  // ACK clock.
        result = ESP_ERR_TIMEOUT;
        goto stop;
    }
    result = gpio_get_level(sda) ? ESP_ERR_NOT_FOUND : ESP_OK;
    gpio_set_level(scl, 0);

stop:
    // STOP, then release both wires before the ESP-IDF I2C driver owns them.
    gpio_set_level(sda, 0);
    esp_rom_delay_us(WIRE_TEST_DELAY_US);
    if (wire_test_raise_scl(scl)) {
        esp_rom_delay_us(WIRE_TEST_DELAY_US);
        gpio_set_level(sda, 1);
    }
    esp_rom_delay_us(WIRE_TEST_DELAY_US);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    return result;
}

esp_err_t tof_early_s1_wire_test(void)
{
    if (port_reserved[0]) return ESP_ERR_NOT_SUPPORTED;
    // S2 remains reset, so only an S1 module can ever ACK the shared factory
    // address. The test performs legal I2C start/address/stop transactions.
    gpio_set_level(BOARD_XSHUT_1, 0);
    gpio_set_level(BOARD_XSHUT_2, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(BOARD_XSHUT_1, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    int normal_phase;
    int swapped_phase;
    int normal_idle_sda;
    int normal_idle_scl;
    int swapped_idle_sda;
    int swapped_idle_scl;
    const esp_err_t normal = wire_test_probe(BOARD_I2C_SDA, BOARD_I2C_SCL,
                                             TOF_DEFAULT_ADDRESS, &normal_phase,
                                             &normal_idle_sda, &normal_idle_scl);
    const esp_err_t swapped = wire_test_probe(BOARD_I2C_SCL, BOARD_I2C_SDA,
                                              TOF_DEFAULT_ADDRESS, &swapped_phase,
                                              &swapped_idle_sda, &swapped_idle_scl);
    ESP_LOGW(TAG, "raw S1 probe @0x29: SDA=GPIO%d/SCL=GPIO%d -> %s "
                  "(phase %d, idle %d/%d); swapped -> %s "
                  "(phase %d, idle %d/%d)",
             BOARD_I2C_SDA, BOARD_I2C_SCL, esp_err_to_name(normal), normal_phase,
             normal_idle_sda, normal_idle_scl, esp_err_to_name(swapped), swapped_phase,
             swapped_idle_sda, swapped_idle_scl);

    gpio_set_level(BOARD_XSHUT_1, 0);
    gpio_set_level(BOARD_XSHUT_2, 0);
    return normal == ESP_OK || swapped == ESP_OK ? ESP_OK : normal;
}

static esp_err_t driver_error(tof_sensor_t *sensor, VL53L4CD_Error status)
{
    if (status == VL53L4CD_ERROR_NONE) return ESP_OK;
    if (sensor->device.last_error != ESP_OK) return sensor->device.last_error;
    if (status == VL53L4CD_ERROR_TIMEOUT) return ESP_ERR_TIMEOUT;
    if (status == VL53L4CD_ERROR_INVALID_ARGUMENT) return ESP_ERR_INVALID_ARG;
    return ESP_FAIL;
}

static bool valid_address(uint8_t address)
{
    return address >= 0x08 && address <= 0x77 &&
           address != TOF_DEFAULT_ADDRESS && address != 0x40;
}

static bool valid_config_pair(const tof_config_t config[TOF_SENSOR_COUNT])
{
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        if (!valid_address(config[i].address_7bit) ||
            config[i].timing_budget_ms < 10 || config[i].timing_budget_ms > 200 ||
            config[i].offset_mm < -1024 || config[i].offset_mm > 1023 ||
            config[i].xtalk_kcps > 128 ||
            config[i].signal_threshold_kcps > 16384 ||
            config[i].sigma_threshold_mm > 16383 ||
            config[i].detection_window > 3 ||
            (config[i].detection_enabled &&
             (config[i].intermeasurement_ms == 0 ||
              config[i].detection_low_mm > config[i].detection_high_mm)) ||
            (config[i].intermeasurement_ms != 0 &&
             (config[i].intermeasurement_ms <= config[i].timing_budget_ms ||
              config[i].intermeasurement_ms > 5000))) return false;
    }
    return config[0].address_7bit != config[1].address_7bit;
}

static void load_config(void)
{
    tof_config_t loaded[TOF_SENSOR_COUNT] = {
        {.address_7bit = 0x30, .enabled = true, .timing_budget_ms = TOF_DEFAULT_BUDGET_MS,
         .xtalk_kcps = TOF_DEFAULT_XTALK_KCPS,
         .signal_threshold_kcps = TOF_DEFAULT_SIGNAL_THRESHOLD_KCPS,
         .sigma_threshold_mm = TOF_DEFAULT_SIGMA_THRESHOLD_MM,
         .detection_low_mm = TOF_DEFAULT_DETECTION_LOW_MM,
         .detection_high_mm = TOF_DEFAULT_DETECTION_HIGH_MM},
        {.address_7bit = 0x31, .enabled = true, .timing_budget_ms = TOF_DEFAULT_BUDGET_MS,
         .xtalk_kcps = TOF_DEFAULT_XTALK_KCPS,
         .signal_threshold_kcps = TOF_DEFAULT_SIGNAL_THRESHOLD_KCPS,
         .sigma_threshold_mm = TOF_DEFAULT_SIGMA_THRESHOLD_MM,
         .detection_low_mm = TOF_DEFAULT_DETECTION_LOW_MM,
         .detection_high_mm = TOF_DEFAULT_DETECTION_HIGH_MM},
    };
    nvs_handle_t handle;
    if (nvs_open("tof_cfg", NVS_READONLY, &handle) == ESP_OK) {
        for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
            uint8_t value8;
            uint16_t value16;
            int16_t offset16;
            if (nvs_get_u8(handle, address_keys[i], &value8) == ESP_OK) loaded[i].address_7bit = value8;
            if (nvs_get_u8(handle, enabled_keys[i], &value8) == ESP_OK) loaded[i].enabled = value8 != 0;
            if (nvs_get_u16(handle, budget_keys[i], &value16) == ESP_OK) loaded[i].timing_budget_ms = value16;
            if (nvs_get_u16(handle, period_keys[i], &value16) == ESP_OK) loaded[i].intermeasurement_ms = value16;
            if (nvs_get_i16(handle, offset_keys[i], &offset16) == ESP_OK) loaded[i].offset_mm = offset16;
            if (nvs_get_u16(handle, xtalk_keys[i], &value16) == ESP_OK) loaded[i].xtalk_kcps = value16;
            if (nvs_get_u16(handle, signal_keys[i], &value16) == ESP_OK) loaded[i].signal_threshold_kcps = value16;
            if (nvs_get_u16(handle, sigma_keys[i], &value16) == ESP_OK) loaded[i].sigma_threshold_mm = value16;
            if (nvs_get_u8(handle, detect_enabled_keys[i], &value8) == ESP_OK) loaded[i].detection_enabled = value8 != 0;
            if (nvs_get_u16(handle, detect_low_keys[i], &value16) == ESP_OK) loaded[i].detection_low_mm = value16;
            if (nvs_get_u16(handle, detect_high_keys[i], &value16) == ESP_OK) loaded[i].detection_high_mm = value16;
            if (nvs_get_u8(handle, detect_window_keys[i], &value8) == ESP_OK) loaded[i].detection_window = value8;
        }
        nvs_close(handle);
    }
    if (!valid_config_pair(loaded)) {
        ESP_LOGW(TAG, "invalid saved configuration; using addresses 0x30 and 0x31");
        loaded[0] = (tof_config_t){.address_7bit = 0x30, .enabled = true,
                                  .timing_budget_ms = TOF_DEFAULT_BUDGET_MS,
                                  .signal_threshold_kcps = TOF_DEFAULT_SIGNAL_THRESHOLD_KCPS,
                                  .sigma_threshold_mm = TOF_DEFAULT_SIGMA_THRESHOLD_MM,
                                  .detection_low_mm = TOF_DEFAULT_DETECTION_LOW_MM,
                                  .detection_high_mm = TOF_DEFAULT_DETECTION_HIGH_MM};
        loaded[1] = (tof_config_t){.address_7bit = 0x31, .enabled = true,
                                  .timing_budget_ms = TOF_DEFAULT_BUDGET_MS,
                                  .signal_threshold_kcps = TOF_DEFAULT_SIGNAL_THRESHOLD_KCPS,
                                  .sigma_threshold_mm = TOF_DEFAULT_SIGMA_THRESHOLD_MM,
                                  .detection_low_mm = TOF_DEFAULT_DETECTION_LOW_MM,
                                  .detection_high_mm = TOF_DEFAULT_DETECTION_HIGH_MM};
    }
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) sensors[i].config = loaded[i];
}

static esp_err_t save_config(uint8_t index)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open("tof_cfg", NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    result = nvs_set_u8(handle, address_keys[index], sensors[index].config.address_7bit);
    if (result == ESP_OK) result = nvs_set_u8(handle, enabled_keys[index], sensors[index].config.enabled);
    if (result == ESP_OK) result = nvs_set_u16(handle, budget_keys[index], sensors[index].config.timing_budget_ms);
    if (result == ESP_OK) result = nvs_set_u16(handle, period_keys[index], sensors[index].config.intermeasurement_ms);
    if (result == ESP_OK) result = nvs_set_i16(handle, offset_keys[index], sensors[index].config.offset_mm);
    if (result == ESP_OK) result = nvs_set_u16(handle, xtalk_keys[index], sensors[index].config.xtalk_kcps);
    if (result == ESP_OK) result = nvs_set_u16(handle, signal_keys[index], sensors[index].config.signal_threshold_kcps);
    if (result == ESP_OK) result = nvs_set_u16(handle, sigma_keys[index], sensors[index].config.sigma_threshold_mm);
    if (result == ESP_OK) result = nvs_set_u8(handle, detect_enabled_keys[index], sensors[index].config.detection_enabled);
    if (result == ESP_OK) result = nvs_set_u16(handle, detect_low_keys[index], sensors[index].config.detection_low_mm);
    if (result == ESP_OK) result = nvs_set_u16(handle, detect_high_keys[index], sensors[index].config.detection_high_mm);
    if (result == ESP_OK) result = nvs_set_u8(handle, detect_window_keys[index], sensors[index].config.detection_window);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

static void reset_state(tof_sensor_t *sensor)
{
    memset(&sensor->state, 0, sizeof(sensor->state));
    sensor->state.address_7bit = sensor->config.address_7bit;
    sensor->state.range_status = 0xff;
    sensor->state.error = ESP_ERR_NOT_FOUND;
}

// ST's recommended diagnostic for the case where the model ID reads correctly
// but SensorInit cannot obtain a sample. Register 0x003a is overwritten by the
// official default configuration immediately afterwards, so this check has no
// lasting effect on the sensor setup.
static esp_err_t verify_vl53l4cd_writes(tof_sensor_t *sensor)
{
    enum { TEST_REGISTER = 0x003a };
    const uint32_t expected = 0xffeeddcc;
    uint8_t bytes[4] = {0};
    uint16_t words[2] = {0};
    uint32_t dword = 0;
    VL53L4CD_Error status = VL53L4CD_WrDWord(&sensor->device, TEST_REGISTER, expected);
    for (int i = 0; i < 4; ++i)
        status |= VL53L4CD_RdByte(&sensor->device, TEST_REGISTER + i, &bytes[i]);
    for (int i = 0; i < 2; ++i)
        status |= VL53L4CD_RdWord(&sensor->device, TEST_REGISTER + 2 * i, &words[i]);
    status |= VL53L4CD_RdDWord(&sensor->device, TEST_REGISTER, &dword);
    const esp_err_t result = driver_error(sensor, status);
    const uint32_t from_bytes = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
                                ((uint32_t)bytes[2] << 8) | bytes[3];
    const uint32_t from_words = ((uint32_t)words[0] << 16) | words[1];
    ESP_LOGW(TAG, "GPIO%d: write/read test 0x%08" PRIx32 "/0x%08" PRIx32
                  "/0x%08" PRIx32 " expected 0x%08" PRIx32 " (%s)",
             sensor->xshut_gpio, from_bytes, from_words, dword, expected,
             esp_err_to_name(result));
    if (result != ESP_OK) return result;
    return from_bytes == expected && from_words == expected && dword == expected
        ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t apply_optical_tuning(tof_sensor_t *sensor)
{
    const tof_config_t *config = &sensor->config;
    VL53L4CD_Error status = VL53L4CD_SetXtalk(&sensor->device, config->xtalk_kcps);
    status |= VL53L4CD_SetSignalThreshold(&sensor->device,
                                          config->signal_threshold_kcps);
    status |= VL53L4CD_SetSigmaThreshold(&sensor->device,
                                         config->sigma_threshold_mm);
    if (config->detection_enabled) {
        status |= VL53L4CD_SetDetectionThresholds(&sensor->device,
                                                   config->detection_low_mm,
                                                   config->detection_high_mm,
                                                   config->detection_window);
    } else {
        // The ULD's default 0x20 means "new sample ready". Restore it when
        // threshold detection is disabled so normal continuous ranging keeps
        // producing every sample.
        status |= VL53L4CD_WrByte(&sensor->device, VL53L4CD_SYSTEM__INTERRUPT,
                                  0x20);
    }
    return driver_error(sensor, status);
}

static esp_err_t start_sensor(tof_sensor_t *sensor)
{
    const int index = sensor == &sensors[0] ? 0 : 1;
    if (port_reserved[index]) return ESP_ERR_NOT_SUPPORTED;
    gpio_set_level(sensor->xshut_gpio, 1);
    // Datasheet tBOOT is at most 1.2 ms. The larger margin also tolerates
    // a cold peripheral supply and lets the I2C pull-ups settle.
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGW(TAG, "GPIO%d XSHUT after release=%d", sensor->xshut_gpio,
             gpio_get_level(sensor->xshut_gpio));
    log_i2c_levels(sensor->xshut_gpio == BOARD_XSHUT_1
                       ? "port 1 after XSHUT release" : "port 2 after XSHUT release");

    // i2c_master_probe() can leave this ESP-IDF bus in a busy state when a
    // VL53L4CD is released from XSHUT. Attach the 100 kHz device directly;
    // the model-ID transaction below already provides an authoritative probe.
    esp_err_t result = vl53l4cd_platform_attach(&sensor->device, board_i2c_bus(),
                                                 TOF_DEFAULT_ADDRESS);
    if (result != ESP_OK) goto failed;

    uint16_t model_id = 0;
    VL53L4CD_Error status = VL53L4CD_GetSensorId(&sensor->device, &model_id);
    result = driver_error(sensor, status);
    ESP_LOGW(TAG, "GPIO%d: model ID read 0x%04x (%s)", sensor->xshut_gpio,
             model_id, esp_err_to_name(result));
    if (result != ESP_OK || model_id != TOF_MODEL_ID) {
        if (result == ESP_OK) result = ESP_ERR_INVALID_RESPONSE;
        ESP_LOGW(TAG, "GPIO%d: model ID 0x%04x (%s)", sensor->xshut_gpio,
                 model_id, esp_err_to_name(result));
        goto failed;
    }

    uint8_t boot_status = 0;
    uint8_t gpio_mux = 0;
    uint8_t gpio_status = 0;
    const VL53L4CD_Error diagnostic_status =
        VL53L4CD_RdByte(&sensor->device, VL53L4CD_FIRMWARE__SYSTEM_STATUS, &boot_status) |
        VL53L4CD_RdByte(&sensor->device, VL53L4CD_GPIO_HV_MUX__CTRL, &gpio_mux) |
        VL53L4CD_RdByte(&sensor->device, VL53L4CD_GPIO__TIO_HV_STATUS, &gpio_status);
    const esp_err_t diagnostic_error = driver_error(sensor, diagnostic_status);
    ESP_LOGW(TAG, "GPIO%d: pre-init firmware=0x%02x GPIO mux/status=0x%02x/0x%02x (%s)",
             sensor->xshut_gpio, boot_status, gpio_mux, gpio_status,
             esp_err_to_name(diagnostic_error));

    result = verify_vl53l4cd_writes(sensor);
    if (result != ESP_OK) goto failed;

    status = VL53L4CD_SensorInit(&sensor->device);
    result = driver_error(sensor, status);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d: VL53L4CD SensorInit failed (%s)",
                 sensor->xshut_gpio, esp_err_to_name(result));
        goto failed;
    }

    status = VL53L4CD_SetI2CAddress(&sensor->device,
                                   (uint8_t)(sensor->config.address_7bit << 1));
    result = driver_error(sensor, status);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d: set I2C address failed (%s)",
                 sensor->xshut_gpio, esp_err_to_name(result));
        goto failed;
    }
    vl53l4cd_platform_detach(&sensor->device);
    result = vl53l4cd_platform_attach(&sensor->device, board_i2c_bus(),
                                      sensor->config.address_7bit);
    if (result != ESP_OK) goto failed;

    status = VL53L4CD_SetRangeTiming(&sensor->device,
                                    sensor->config.timing_budget_ms,
                                    sensor->config.intermeasurement_ms);
    result = driver_error(sensor, status);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d: set timing failed (%s)", sensor->xshut_gpio,
                 esp_err_to_name(result));
        goto failed;
    }

    status = VL53L4CD_SetOffset(&sensor->device, sensor->config.offset_mm);
    result = driver_error(sensor, status);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d: set offset %d mm failed (%s)", sensor->xshut_gpio,
                 sensor->config.offset_mm, esp_err_to_name(result));
        goto failed;
    }

    result = apply_optical_tuning(sensor);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d: apply optical tuning failed (%s)",
                 sensor->xshut_gpio, esp_err_to_name(result));
        goto failed;
    }

    sensor->state.present = true;
    sensor->state.error = ESP_OK;
    if (sensor->config.enabled) {
        status = VL53L4CD_StartRanging(&sensor->device);
        result = driver_error(sensor, status);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d: start ranging failed (%s)", sensor->xshut_gpio,
                     esp_err_to_name(result));
            goto failed;
        }
        sensor->state.ranging = true;
    }
    ESP_LOGI(TAG, "GPIO%d: VL53L4CD at 0x%02x, %u/%u ms, xtalk=%u, signal=%u, sigma=%u, %s",
             sensor->xshut_gpio, sensor->config.address_7bit,
             sensor->config.timing_budget_ms, sensor->config.intermeasurement_ms,
             sensor->config.xtalk_kcps, sensor->config.signal_threshold_kcps,
             sensor->config.sigma_threshold_mm,
             sensor->config.enabled ? "ranging" : "idle");
    return ESP_OK;

failed:
    ESP_LOGW(TAG,
             "GPIO%d before controlled shutdown: XSHUT=%d, SDA=%d, SCL=%d, error=%s",
             sensor->xshut_gpio, gpio_get_level(sensor->xshut_gpio),
             gpio_get_level(BOARD_I2C_SDA), gpio_get_level(BOARD_I2C_SCL),
             esp_err_to_name(result));
    sensor->state.error = result;
    sensor->state.present = false;
    sensor->state.ranging = false;
    vl53l4cd_platform_detach(&sensor->device);
    gpio_set_level(sensor->xshut_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (result == ESP_ERR_TIMEOUT) i2c_master_bus_reset(board_i2c_bus());
    return result;
}

static esp_err_t reinitialize_locked(void)
{
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        if (port_reserved[i]) {
            vl53l4cd_platform_detach(&sensors[i].device);
            reset_state(&sensors[i]);
            sensors[i].state.error = ESP_ERR_NOT_SUPPORTED;
            continue;
        }
        if (sensors[i].device.device && sensors[i].state.ranging)
            VL53L4CD_StopRanging(&sensors[i].device);
        vl53l4cd_platform_detach(&sensors[i].device);
        gpio_set_level(sensors[i].xshut_gpio, 0);
        reset_state(&sensors[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    log_i2c_levels("ToF ports in XSHUT reset");

    unsigned found = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        if (!port_reserved[i] && start_sensor(&sensors[i]) == ESP_OK) ++found;
    }
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t tof_reinitialize(void)
{
    if (!mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(mutex, portMAX_DELAY);
    const esp_err_t result = reinitialize_locked();
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t tof_configure(uint8_t index, const tof_config_t *config)
{
    if (!mutex || !config || index >= TOF_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;
    if (port_reserved[index]) return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(mutex, portMAX_DELAY);
    tof_config_t candidate[TOF_SENSOR_COUNT] = {sensors[0].config, sensors[1].config};
    candidate[index] = *config;
    if (!valid_config_pair(candidate)) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_ARG;
    }
    sensors[index].config = *config;
    esp_err_t result = save_config(index);
    if (result == ESP_OK) result = reinitialize_locked();
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t tof_set_offset(uint8_t index, int16_t offset_mm)
{
    if (!mutex || index >= TOF_SENSOR_COUNT || offset_mm < -1024 || offset_mm > 1023)
        return ESP_ERR_INVALID_ARG;
    if (port_reserved[index]) return ESP_ERR_NOT_SUPPORTED;

    xSemaphoreTake(mutex, portMAX_DELAY);
    tof_sensor_t *sensor = &sensors[index];
    if (!sensor->state.present || !sensor->device.device) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NOT_FOUND;
    }

    const bool resume_ranging = sensor->state.ranging;
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    if (resume_ranging) status |= VL53L4CD_StopRanging(&sensor->device);
    status |= VL53L4CD_SetOffset(&sensor->device, offset_mm);
    if (resume_ranging) status |= VL53L4CD_StartRanging(&sensor->device);
    esp_err_t result = driver_error(sensor, status);
    if (result == ESP_OK) {
        sensor->config.offset_mm = offset_mm;
        result = save_config(index);
    }
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t tof_set_tuning(uint8_t index, const tof_config_t *config)
{
    if (!mutex || !config || index >= TOF_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;
    if (port_reserved[index]) return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(mutex, portMAX_DELAY);
    tof_config_t candidate[TOF_SENSOR_COUNT] = {sensors[0].config, sensors[1].config};
    candidate[index] = *config;
    if (!valid_config_pair(candidate)) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_ARG;
    }
    sensors[index].config = *config;
    esp_err_t result = save_config(index);
    if (result == ESP_OK) result = reinitialize_locked();
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t tof_temperature_update(uint8_t index)
{
    if (!mutex || index >= TOF_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;
    if (port_reserved[index]) return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(mutex, portMAX_DELAY);
    tof_sensor_t *sensor = &sensors[index];
    if (!sensor->state.present || !sensor->device.device) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NOT_FOUND;
    }

    const bool resume_ranging = sensor->state.ranging;
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    if (resume_ranging) status |= VL53L4CD_StopRanging(&sensor->device);
    if (status == VL53L4CD_ERROR_NONE)
        status |= VL53L4CD_StartTemperatureUpdate(&sensor->device);
    // Keep the user's ranging setting after the ULD routine, including when
    // it already started and stopped its internal temperature sample.
    if (resume_ranging) status |= VL53L4CD_StartRanging(&sensor->device);
    const esp_err_t result = driver_error(sensor, status);
    sensor->state.ranging = result == ESP_OK && resume_ranging;
    if (result != ESP_OK) sensor->state.error = result;
    xSemaphoreGive(mutex);
    return result;
}

uint8_t tof_present_count(void)
{
    if (!mutex) return 0;
    uint8_t count = 0;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i)
        if (sensors[i].state.present) ++count;
    xSemaphoreGive(mutex);
    return count;
}

void tof_get_config(uint8_t index, tof_config_t *config)
{
    if (!mutex || !config || index >= TOF_SENSOR_COUNT) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    *config = sensors[index].config;
    xSemaphoreGive(mutex);
}

void tof_get_snapshot(tof_snapshot_t *snapshot)
{
    if (!mutex || !snapshot) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) snapshot->sensor[i] = sensors[i].state;
    xSemaphoreGive(mutex);
}

static void tof_task(void *argument)
{
    (void)argument;
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
            if (port_reserved[i]) continue;
            tof_sensor_t *sensor = &sensors[i];
            if (!sensor->state.ranging || !sensor->device.device) continue;
            uint8_t ready = 0;
            VL53L4CD_Error status = VL53L4CD_CheckForDataReady(&sensor->device, &ready);
            esp_err_t result = driver_error(sensor, status);
            if (result == ESP_OK && ready) {
                VL53L4CD_ResultsData_t reading;
                memset(&reading, 0, sizeof(reading));
                status = VL53L4CD_GetResult(&sensor->device, &reading);
                if (status == VL53L4CD_ERROR_NONE)
                    status = VL53L4CD_ClearInterrupt(&sensor->device);
                result = driver_error(sensor, status);
                if (result == ESP_OK) {
                    sensor->state.range_status = reading.range_status;
                    sensor->state.distance_mm = reading.distance_mm;
                    sensor->state.signal_per_spad_kcps = reading.signal_per_spad_kcps;
                    sensor->state.ambient_per_spad_kcps = reading.ambient_per_spad_kcps;
                    sensor->state.number_of_spad = reading.number_of_spad;
                    sensor->state.sigma_mm = reading.sigma_mm;
                    sensor->state.valid = reading.range_status == 0;
                    sensor->state.sample_count++;
                }
            }
            if (result != ESP_OK) {
                sensor->state.error = result;
                sensor->state.valid = false;
            }
        }
        xSemaphoreGive(mutex);
        vTaskDelay(pdMS_TO_TICKS(TOF_POLL_MS));
    }
}

esp_err_t tof_set_port_reserved(uint8_t index, bool reserved)
{
    // Pin ownership is a boot-time hardware decision. Refuse a live switch:
    // changing it after a ToF task was created could drive an MPU interrupt.
    if (index >= TOF_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;
    if (mutex || task_handle) return ESP_ERR_INVALID_STATE;
    port_reserved[index] = reserved;
    return ESP_OK;
}

esp_err_t tof_prepare_pins(void)
{
    return configure_xshut_pins();
}

esp_err_t tof_init(void)
{
    esp_err_t result = board_i2c_init();
    if (result != ESP_OK) return result;
    mutex = xSemaphoreCreateMutex();
    if (!mutex) return ESP_ERR_NO_MEM;
    sensors[0].xshut_gpio = BOARD_XSHUT_1;
    sensors[1].xshut_gpio = BOARD_XSHUT_2;
    load_config();

    result = configure_xshut_pins();
    if (result != ESP_OK) return result;

    xSemaphoreTake(mutex, portMAX_DELAY);
    const esp_err_t detect_result = reinitialize_locked();
    xSemaphoreGive(mutex);
    if (xTaskCreate(tof_task, "vl53l4cd", 4096, NULL, 4, &task_handle) != pdPASS)
        return ESP_ERR_NO_MEM;
    return detect_result;
}
