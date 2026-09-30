#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t aura_radio_init(void);
esp_err_t aura_radio_deinit(void);
bool aura_radio_is_running(void);

// Number of automatic disarms caused by losing the DualSense since boot.
uint32_t aura_radio_pad_loss_disarms(void);
