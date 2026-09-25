# ICM-42688 isolated UART test

This is a deliberately separate ESP-IDF firmware image for Aura Main Board.
It starts only UART logging, the IO33 heartbeat LED, and SPI for the IMU.
Wi-Fi, Bluetooth, the desktop protocol, servos, LEDs and all other board
drivers are absent so they cannot affect the result.

The test uses the `ICM42688_Barsotion` library unchanged for its SPI
transactions: SPI mode 0, 5 MHz, hardware-controlled CS and an expected
TDK ICM-42688 `WHO_AM_I` value of `0x47`.

| Signal | ESP32 GPIO |
| --- | --- |
| SDO / MISO | 19 |
| SDI / MOSI | 23 |
| AP_SCLK | 22 |
| AP_CS | 21 |
| Board heartbeat LED | 33 |

The serial output prints identity and status registers. When identity is
valid, it also prints gyro and accelerometer data at 10 Hz.

`main/barsotion` is a separate checkout of
<https://github.com/Barsy-Barsevich/ICM42688_Barsotion>, including its
GPL-3.0 license. It is intentionally kept outside the Aura production
firmware until the diagnostic result is known.
