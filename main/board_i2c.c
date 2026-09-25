#include "board.h"
#include "board_i2c.h"

static i2c_master_bus_handle_t bus;

esp_err_t board_i2c_init(void)
{
    if (bus) return ESP_OK;
    const i2c_master_bus_config_t config = {
        // Native board I2C controller shared by INA226, VL53 and MPU6050.
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // The board schematic specifies 5.1 kOhm external pull-ups. Keep a
        // weak internal pull-up active as a safe fallback while the distance
        // ports are being brought up; it changes no logic levels.
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &bus);
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return bus;
}
