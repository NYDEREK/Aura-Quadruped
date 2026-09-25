# Aura Quadruped

Czworonożny robot na 12 serwach Waveshare **ST3215** (TTL, 1 Mbit/s), sterowany płytką **Aura MainBoard** z ESP32‑WROOM‑32E, padem DualSense i natywną aplikacją macOS **Aura**.

## Struktura repozytorium

| Ścieżka | Zawartość |
|---|---|
| `main/` | Firmware ESP‑IDF: sterowniki (serwa, IMU, INA226, VL53L4CD, WS2812), radio (Wi‑Fi + DualSense HID), kinematyka, generator chodu, balans |
| `desktop/MainBoardControl/` | Aplikacja macOS (SwiftUI): telemetria, serwa, kalibracja nóg, symulacja 3D, Test IK |
| `tests/` | Testy hosta (C/Swift) dla kinematyki, chodu, balansu i protokołów – `python3 tests/run_host_tests.py` |
| `gazebo/` | Model SDF i kontroler do symulacji w Gazebo |
| `docs/` | Protokół, model lokomocji, schemat płytki (tekst + PNG), notatki z uruchamiania |
| `diagnostics/` | Izolowane programy testowe (ICM‑42688, DualSense) |

Konfiguracja Wi‑Fi: skopiuj `main/wifi_credentials.example.h` do `main/wifi_credentials.h` (plik jest ignorowany przez git).

---

# Aura MainBoard — Wi-Fi telemetry + DualSense HID firmware

Firmware and a native macOS application for the ESP32-WROOM-32E-N8 board in `SCH_Schematic1_2026-09-13.pdf`.

## Active interfaces

| Function | ESP32 pins | Behaviour |
|---|---|---|
| Fitted green LED | GPIO33 | Active-high heartbeat every 500 ms |
| MPU6050 I2C, S1 | SDA 25, SCL 26, DATA_RDY 14 | Address probe `0x68`/`0x69`, WHO_AM_I `0x68`, coherent raw samples at 100 Hz and filtered roll/pitch |
| INA226 I2C | SDA 25, SCL 26 | Voltage, shunt current and power at address `0x40` |
| VL53L4CD I2C, S2 | SDA 25, SCL 26; XSHUT 15 | Distance and signal telemetry; automatic address `0x31` |
| VIN ADC | GPIO34 / ADC1 channel 6 | Fitted 100k/22k divider |
| ST3215 TTL | RX 16, TX 17, direction 18 | 1 Mbit/s; starts in receive/high-impedance mode and sends nothing automatically |
| WS2812 data | GPIO32 | Up to 64 GRB pixels, solid/rainbow/off test |
| Wi-Fi | TCP `aura-main-board.local:4242` | Aura desktop telemetry and configuration while external VIN is present |
| Bluetooth Classic HID | DualSense | ESP32 is HID host for the saved controller; it does not carry desktop telemetry |
| USB console | UART0 GPIO1/GPIO3 | 115200, 8N1 |

GPIO13, GPIO27 and GPIO35 remain untouched. GPIO14 is reserved as an input for MPU6050 `DATA_RDY`; firmware never drives it. GPIO15 remains the S2 VL53L4CD `XSHUT` output.

The flash contains one `factory` application partition, so there is no OTA boot selection state. Wi-Fi and Bluetooth start only after Aura sees stable external VIN above 7 V; USB is consequently suitable for programming without powering the radio or servos. The Aura app uses Bonjour and falls back to `aura-main-board.local:4242`. The DualSense address is stored separately in NVS. NVS also stores ESP-IDF RF calibration, robot-axis configuration and preferred VL53L4CD port settings.

The brownout threshold is configured at the highest ESP32 setting (approximately 2.80 V) and the task watchdog reboots the application if it stops making progress. ROM download mode remains available through BOOT and RESET independently of the application image.

Before enabling the radio, firmware starts the GPIO33 heartbeat, sensor telemetry and UART console. Wi-Fi runs without modem sleep so the TCP desktop telemetry does not arrive in visible bursts while Classic Bluetooth is receiving DualSense reports. The legacy-named `ble retry` UART command remains available to restart the radio stack.

## Build and flash

Requires ESP-IDF 6.1 (`idf.sh` sources `~/.espressif/tools/activate_idf_v6.1.sh`).

```sh
./idf.sh build
./idf.sh -p /dev/cu.usbserial-PORT erase-flash flash monitor
```

A full erase is used when fitting a replacement ESP32 so no former OTA metadata or network settings remain.

## macOS application

Build output: `desktop/MainBoardControl/dist/Aura.app`

```sh
desktop/MainBoardControl/build.sh
```

Aura communicates over Wi-Fi while DualSense uses a separate Classic HID connection on the same ESP32 controller. It displays power, MPU6050 and VL53L4CD distance readings, plots servo current, scans and controls ST3215 servos, assigns IDs one servo at a time, calibrates the midpoint and controls WS2812 pixels. S1 is reserved for the MPU6050. S2 may operate alone as a VL53L4CD port; an absent sensor never blocks the IMU or the rest of the board. Since a VL53L4CD always powers up at `0x29`, firmware reapplies its saved `0x31` assignment through GPIO15 XSHUT. ST3215 control includes a position-servo mode and a continuous-motor mode with signed speed, acceleration and a STOP action that disables torque first.

The **Calibration** tab commissions one physical leg at a time in the order Front Right, Front Left, Back Left, Back Right. Its 3D reference pose captures a software zero for Ab/ad, Hip and Knee directly from the live ST3215 feedback. It then applies a limited ±5° direction check, records the manually positioned negative and positive joint limits, and offers a single-leg DualSense test. Those parameters are written to Aura's NVS axis map after each capture. The one-leg test runs in Aura's own 50 Hz loop; the desktop only starts and stops it.

## MPU6050 attitude correction

The MPU6050 owner task samples S1 at 100 Hz. It uses the accelerometer as the long-term gravity reference and gyro integration for fast roll/pitch updates. The gait task reads only this finished state at 50 Hz; it never owns the I²C bus.

The correction is opt-in and may be changed only while disarmed. At the next arm request Aura captures the current roll/pitch while torque is still off. While samples remain fresh, it applies a bounded, slew-limited inverse body rotation to every foot target before inverse kinematics. This keeps the correction inside the existing per-axis calibrated limits. No MPU sample, a sample older than 120 ms, or a tilt error over 20° disables the correction and rejects a new arm request when correction is enabled. The code does not claim to be a full contact-force/MPC controller: the intended next step is fusing MPU data with encoder kinematics and verified foot contacts.

The default mounting map is **MPU X forward, Y left, Z up**. Verify roll and pitch sign on a disarmed robot in the IMU tab before enabling the correction.

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
tof retry
ble retry
```

The virtual gait input uses the same 50 Hz planner, inverse kinematics,
trajectory limiter and feedback retiming as a DualSense input. Values are
stick units from `-1000` to `1000`; it is intended for controlled bench tests.
