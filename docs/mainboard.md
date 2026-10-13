# Aura MainBoard reference

Firmware for the ESP32-WROOM-32E-N8 board in `SCH_Schematic1_2026-09-13.pdf`: Wi-Fi telemetry for the Aura app, DualSense HID host and the robot's control loops.

## Interfaces

| Function | ESP32 pins | Behaviour |
|---|---|---|
| Fitted green LED | GPIO33 | Active-high heartbeat every 500 ms |
| MPU6050 I2C, S1 | SDA 25, SCL 26, DATA_RDY 14 | Address probe `0x68`/`0x69`, WHO_AM_I `0x68`, coherent raw samples at 100 Hz and filtered roll/pitch |
| INA226 I2C | SDA 25, SCL 26 | Voltage, shunt current and power at address `0x40` |
| VL53L4CD I2C, S2 | SDA 25, SCL 26; XSHUT 15 | Distance and signal telemetry; automatic address `0x31` |
| VIN ADC | GPIO34 / ADC1 channel 6 | Fitted 100k/22k divider |
| ST3215 TTL | RX 16, TX 17, direction 18 | 1 Mbit/s; starts in receive/high-impedance mode and sends nothing automatically |
| WS2812 data | GPIO32 | 8-pixel status strip: state, selected mode, 6-pixel battery bar (10.0–12.6 V); see `wifi-hid.md` |
| Wi-Fi | TCP `aura-main-board.local:4242` | Aura app telemetry and configuration while external VIN is present |
| Bluetooth Classic HID | DualSense | The ESP32 is HID host for the saved controller; it does not carry app telemetry |
| USB console | UART0 GPIO1/GPIO3 | 115200, 8N1 |

GPIO13, GPIO27 and GPIO35 are unused. GPIO14 is an input for MPU6050 `DATA_RDY`; the firmware never drives it. GPIO15 is the S2 VL53L4CD `XSHUT` output.

## Flash layout and power

| Partition | Offset | Size | Contents |
|---|---|---|---|
| `nvs` | 0x9000 | 24 KB | Servo axis map and calibration, gait profiles, level calibration, DualSense address, VL53L4CD settings, ESP-IDF RF calibration |
| `phy_init` | 0xF000 | 4 KB | RF data |
| `factory` | 0x10000 | 3 MB | The application. There is no OTA, so there is no boot-selection state. |
| `coredump` | 0x310000 | 128 KB | The last crash, reported to the app (frame 0x38) |

Wi-Fi and Bluetooth start only after Aura sees stable external VIN above 7 V, so USB alone can program the board without powering the radio or the servos. The Aura app finds the board with Bonjour and falls back to `aura-main-board.local:4242`.

The brownout threshold is the highest ESP32 setting, about 2.80 V. The task watchdog restarts the application if a core's idle task gets no time for 5 s. ROM download mode stays available through BOOT and RESET, whatever application image is flashed.

Before the radio starts, the firmware runs the GPIO33 heartbeat, the sensors and the UART console. Wi-Fi runs without modem sleep, so telemetry does not arrive in bursts while Bluetooth receives DualSense reports. The legacy-named `ble retry` UART command restarts the radio stack.

## Build and flash

Requires ESP-IDF 6.1 (`idf.sh` sources `~/.espressif/tools/activate_idf_v6.1.sh`).

```sh
tools/flash_firmware.sh
```

The script backs up NVS, builds, flashes the bootloader, partition table and application, and checks that NVS is unchanged.

> **Warning:** `idf.py erase-flash` deletes the servo calibration and every other setting. Use it only on a replacement ESP32 that has never been configured.

## Calibration

The app's **Calibration** page commissions one leg at a time, in this order: front right, front left, back left, back right. For each leg:

1. A 3D reference pose sets a software zero for ab/ad, hip and knee from live ST3215 feedback.
2. A limited ±5° move checks each joint's direction.
3. You position the joint by hand at its negative and positive limits, and the page records them.
4. An optional single-leg DualSense test runs in Aura's own 50 Hz loop. The app only starts and stops it.

Each capture is written to Aura's NVS axis map. A complete map of 12 distinct axes is required before Aura accepts an arm request.

## IMU posture loop

The MPU6050 task samples S1 at 100 Hz. It uses the accelerometer as the long-term gravity reference and integrates the gyro for fast roll/pitch updates. The gait task reads only this finished state; it never touches the I²C bus.

How the posture loop works:

- **Mode 1 only.** It keeps the body level. In the other modes the loop is off, and a correction left over from mode 1 glides back to zero.
- **Stored level reference.** Use **Calibrate level** on the IMU page once, armed in mode 1, standing still on flat ground. The reference is saved in NVS (`imu_level`) and used on every arm.
- **Bounded correction.** The correction is a slew-limited body rotation applied to the foot targets before inverse kinematics, so it stays within the calibrated joint limits.
- **Stale samples.** If the newest sample is older than 120 ms, the correction holds. After 250 ms without a fresh sample it is cleared.

The mounting map is **MPU X forward, Y left, Z up**. Check the roll and pitch signs on a disarmed robot, on the IMU page, before relying on the loop.

## UART commands

```text
help
status
servo ping <id>
servo read <id>
robot arm
robot disarm
gait virtual <forward> [lateral turn height]
gait virtual off
led <0-100>
tof retry
ble retry
```

The virtual gait input goes through the same 50 Hz planner, inverse kinematics, trajectory limiter and feedback retiming as DualSense input. Values are stick units from `-1000` to `1000`; it is meant for controlled bench tests.
