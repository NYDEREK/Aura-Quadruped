#include <stdlib.h>
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "board.h"
#include "ws2812.h"

static rmt_channel_handle_t channel;
static rmt_encoder_handle_t encoder;
static rmt_symbol_word_t *symbols;
static size_t pixels;
// Both the state indicator and a manual app command may update the strip.
// They share the RMT symbol buffer, so serialize the whole encode/transmit
// operation; interleaving two frames produces visibly random colours.
static SemaphoreHandle_t write_lock;

esp_err_t ws2812_init(size_t pixel_count)
{
    if (channel) return ESP_ERR_INVALID_STATE;
    if (!pixel_count || pixel_count > 256) return ESP_ERR_INVALID_ARG;
    symbols = calloc(pixel_count * 24 + 1, sizeof(*symbols));
    if (!symbols) return ESP_ERR_NO_MEM;
    write_lock = xSemaphoreCreateMutex();
    if (!write_lock) { free(symbols); symbols = NULL; return ESP_ERR_NO_MEM; }
    const rmt_tx_channel_config_t config = {
        .gpio_num = BOARD_PIXEL_DATA, .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 20000000, .mem_block_symbols = 64, .trans_queue_depth = 1,
    };
    esp_err_t result = rmt_new_tx_channel(&config, &channel);
    if (result != ESP_OK) { vSemaphoreDelete(write_lock); write_lock = NULL; free(symbols); symbols = NULL; return result; }
    const rmt_copy_encoder_config_t copy = {};
    result = rmt_new_copy_encoder(&copy, &encoder);
    if (result == ESP_OK) result = rmt_enable(channel);
    if (result != ESP_OK) {
        if (encoder) rmt_del_encoder(encoder);
        rmt_del_channel(channel);
        vSemaphoreDelete(write_lock);
        free(symbols);
        channel = NULL; encoder = NULL; symbols = NULL; write_lock = NULL;
        return result;
    }
    pixels = pixel_count;
    return ESP_OK;
}

esp_err_t ws2812_write_rgb(const uint8_t *rgb, size_t pixel_count)
{
    if (!channel) return ESP_ERR_INVALID_STATE;
    if (!rgb || pixel_count != pixels) return ESP_ERR_INVALID_ARG;
    if (!write_lock || xSemaphoreTake(write_lock, pdMS_TO_TICKS(1100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    size_t index = 0;
    // 800 kHz, GRB order. Timing adapter only, not a claim about the attached strip type.
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        const uint8_t grb[] = {rgb[pixel * 3 + 1], rgb[pixel * 3], rgb[pixel * 3 + 2]};
        for (int component = 0; component < 3; ++component) {
            for (int bit = 7; bit >= 0; --bit) {
                bool one = (grb[component] >> bit) & 1;
                symbols[index++] = (rmt_symbol_word_t){
                    .level0 = 1, .duration0 = one ? 16 : 7,
                    .level1 = 0, .duration1 = one ? 9 : 18,
                };
            }
        }
    }
    // 300 us low covers the longer reset time of newer WS2812 variants.
    symbols[index++] = (rmt_symbol_word_t){.level0 = 0, .duration0 = 3000, .level1 = 0, .duration1 = 3000};
    const rmt_transmit_config_t transmit = {.loop_count = 0, .flags.eot_level = 0};
    esp_err_t result = rmt_transmit(channel, encoder, symbols, index * sizeof(*symbols), &transmit);
    if (result == ESP_OK) result = rmt_tx_wait_all_done(channel, 1000);
    xSemaphoreGive(write_lock);
    return result;
}
