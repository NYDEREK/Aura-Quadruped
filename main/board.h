#pragma once

// Verified against page 2 of SCH_Schematic1_2026-09-13.pdf.
enum {
    BOARD_LED = 33,             // LED_BLUE in schematic, green LED fitted; active HIGH.
    BOARD_IMU_MISO = 19,
    BOARD_IMU_MOSI = 23,
    BOARD_IMU_SCLK = 22,
    BOARD_IMU_CS = 21,
    BOARD_IMU_INT1 = 27,        // Poll the IMU: this pin is not configured.
    BOARD_I2C_SDA = 25,
    BOARD_I2C_SCL = 26,
    BOARD_INA_ALERT = 35,       // Not configured.
    BOARD_BATTERY_ADC = 34,
    BOARD_SERVO_RX = 16,
    BOARD_SERVO_TX = 17,
    BOARD_SERVO_DIR = 18,       // 74LVC1G126 OE: HIGH = transmit, LOW = high impedance.
    BOARD_PIXEL_DATA = 32,      // WS2812 output; initialized low until an API command is sent.
    BOARD_SERVO_PWM = 13,       // Not configured.
    BOARD_XSHUT_1 = 14,         // VL53L4CD port 1, active-low shutdown.
    BOARD_XSHUT_2 = 15,         // VL53L4CD port 2, active-low shutdown.
};
#define BOARD_SHUNT_OHMS 0.001f
#define BOARD_BATTERY_DIVIDER ((100.0f + 22.0f) / 22.0f)
