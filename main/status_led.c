#include "nvs.h"
#include "status_led.h"

#define STATUS_LED_NVS_NAMESPACE "aura_led"
#define STATUS_LED_NVS_KEY "bright"

static volatile uint8_t brightness = STATUS_LED_DEFAULT_BRIGHTNESS;

void status_led_init(void)
{
    nvs_handle_t nvs = 0;
    if (nvs_open(STATUS_LED_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    uint8_t saved = 0;
    if (nvs_get_u8(nvs, STATUS_LED_NVS_KEY, &saved) == ESP_OK && saved <= 100)
        brightness = saved;
    nvs_close(nvs);
}

uint8_t status_led_brightness(void)
{
    return brightness;
}

esp_err_t status_led_set_brightness(uint8_t percent)
{
    if (percent > 100) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(STATUS_LED_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_u8(nvs, STATUS_LED_NVS_KEY, percent);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    // The strip follows at once even if flash refused the write.
    brightness = percent;
    return result;
}
