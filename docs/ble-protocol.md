# Aura Bluetooth Classic SPP protocol

The ESP32 is discoverable as `Aura Main Board` and exposes a Bluetooth Classic SPP/RFCOMM service. The same Classic Bluetooth controller also runs one DualSense HID host connection.

## Transport

Aura uses the standard Serial Port Profile (UUID `0x1101`) over Bluetooth Classic RFCOMM. The Aura app pairs Aura Main Board on its first connection. Every application payload is sent as a byte-stream frame: `A5 5A length payload checksum`, where `checksum` is the XOR of `length` and every payload byte.

All multibyte numbers are little-endian. Commands use the exact lengths below. Telemetry and event payloads are always 20 bytes.

## Commands

| Opcode | Bytes | Operation |
|---|---|---|
| `01` | `01` | Scan ST3215 IDs 0 through 253 |
| `02` | `02 id` | Read servo feedback |
| `03` | `03 id enabled` | Set torque; `enabled` is 0 or 1 |
| `04` | `04 id position:u16 speed:u16 acceleration:u8` | Move servo |
| `05` | `05 id` | Save the current servo position as midpoint |
| `06` | `06 current_id new_id` | Assign a new servo ID |
| `07` | `07 mode count red green blue` | WS2812 control; mode 0 off, 1 solid, 2 rainbow |
| `08` | `08` | Request an immediate power telemetry packet |
| `09` | `09 id mode` | Save ST3215 mode; 0 position servo, 1 continuous motor |
| `0A` | `0A id speed:i16 acceleration:u8` | Run motor at signed speed; zero stops and disables torque |
| `0B` | `0B port address enabled budget:u16 period:u16` | Save and apply one VL53L4CD port configuration |
| `0C` | `0C` | Request both VL53L4CD configuration events |
| `0D` | `0D` | Reset both XSHUT lines, detect sensors and reapply their addresses |
| `10` | `10 port xtalk:u16 signal:u16 sigma:u16 low:u16 high:u16 window enabled` | Save/apply VL53L4CD optical filters and distance-threshold detection |
| `11` | `11 port` | Run a VL53L4CD temperature update and restore its prior ranging state |
| `12` | `12` | Start a DualSense scan; put the pad into pairing mode with Create + PS, then select it in Aura |
| `13` | `13` | Disconnect the active DualSense without deleting its pairing |
| `14` | `14` | Connect the previously paired DualSense |
| `15` | `15` | Forget the stored DualSense address and Bluetooth bond |
| `16` | `16 address[6]` | Connect the selected address from the latest Classic-Bluetooth device list |

Servo position is 0 through 4095, position speed is 0 through 3400, signed motor speed is -3400 through 3400, acceleration is 0 through 254, and WS2812 count is 1 through 64. A negative motor speed reverses direction. Changing mode disables torque, unlocks EEPROM, writes and verifies register 33, locks EEPROM again, and leaves torque disabled. A nonzero motor-speed command writes speed before enabling torque. A zero motor-speed command disables torque first so STOP remains fail-safe.

VL53L4CD `port` is 0 or 1 and maps to XSHUT GPIO14 or GPIO15. `address` is a 7-bit address from `0x08` through `0x77`; `0x29` is reserved for sensor discovery, `0x40` belongs to the INA226, and both port addresses must differ. `budget` is 10 through 200 ms. `period` is 0 for continuous ranging or a value greater than `budget`, up to 5000 ms. The preferred settings are stored in ESP NVS. The sensor itself returns to `0x29` after power loss, so firmware always assigns the saved address during startup.

For command `10`, `xtalk` is 0 through 128 kcps, `signal` is 0 through 16384 kcps, and `sigma` is 0 through 16383 mm. `enabled` controls the sensor's GPIO1 threshold detector. When it is 1, `low` must not exceed `high`, `window` is 0 below / 1 above / 2 outside / 3 inside, and a nonzero period greater than the timing budget is required. The temperature command is intended after an ambient-temperature change of roughly 8 °C.

## Telemetry notifications

Power packet `0x10`:

| Offset | Type | Meaning |
|---|---|---|
| 0 | `u8` | `0x10` |
| 1 | `u8` | Protocol version, currently 1 |
| 2 | `u32` | Uptime in milliseconds |
| 6 | `u16` | Servo-bus voltage in millivolts |
| 8 | `i32` | Servo current in milliamps |
| 12 | `i32` | Servo power in milliwatts |
| 16 | `u16` | VIN in millivolts |
| 18 | `u8` | Flags: bit 0 INA226, bit 1 VIN ADC, bit 2 IMU available |
| 19 | `u8` | ICM `WHO_AM_I` value |

IMU packet `0x11`:

| Offset | Type | Meaning |
|---|---|---|
| 0 | `u8` | `0x11` |
| 1 | `u8` | Protocol version, currently 1 |
| 2 | `u32` | IMU sample counter |
| 6 | `i16[3]` | X/Y/Z acceleration in milli-g |
| 12 | `i16[3]` | X/Y/Z angular rate in 0.1 degree/s |
| 18 | `i16` | Temperature in 0.01 degrees Celsius |

VL53L4CD packet `0x12`:

| Offset | Type | Meaning |
|---|---|---|
| 0 | `u8` | `0x12` |
| 1 | `u8` | Protocol version, currently 1 |
| 2 | `u8` | Bits 0–1 present, 2–3 ranging, 4–5 valid for ports 0–1 |
| 4 / 12 | `u8` | Port 0 / port 1 I²C address |
| 5 / 13 | `u8` | Port 0 / port 1 ST range status; 0 means valid |
| 6 / 14 | `u16` | Distance in millimetres |
| 8 / 16 | `u16` | Signal per SPAD in kcps |
| 10 / 18 | `u16` | Sigma in millimetres |

DualSense input packet `0x14`:

| Offset | Type | Meaning |
|---|---|---|
| 0 | `u8` | `0x14` |
| 2 | `u8` | Flags: bit 0 connected, bit 1 pairing scan, bit 2 input received, bit 3 battery known |
| 3 | `u8` | HID report ID |
| 4…7 | `u8[4]` | Left X/Y and right X/Y sticks, 0…255 |
| 8…9 | `u8[2]` | L2 / R2 trigger values |
| 10…12 | `u8[3]` | Raw DualSense button bytes |
| 13 | `u8` | Battery percentage, if present |
| 14 | `u8` | Current HID report length |
| 16 | `u32` | Count of received input reports |

VL53L4CD diagnostics packet `0x13`:

| Offset | Type | Meaning |
|---|---|---|
| 0 | `u8` | `0x13` |
| 1 | `u8` | Protocol version, currently 1 |
| 2 | `u8` | Bits 0–1 present, bits 2–3 valid for ports 0–1 |
| 4 / 12 | `u16` | Port 0 / port 1 ambient rate per SPAD in kcps |
| 6 / 14 | `u16` | Port 0 / port 1 enabled SPAD count |
| 8 / 16 | `u32` | Port 0 / port 1 sample counter |

Power and VL53L4CD range packets are sent at 10 Hz. IMU is sent about 3.3 Hz and the optical diagnostics packet at 2 Hz.

## Event notifications

| Type | Layout | Meaning |
|---|---|---|
| `0x20` | `type opcode result:u32 detail ...` | Command completion; `result` is `esp_err_t` and zero means success |
| `0x21` | `type id ...` | Servo ID found during a scan |
| `0x22` | `type id status moving position:u16 speed:i16 load:i16 voltage:u8 temperature:u8 current:i16 mode:u8 ...` | ST3215 feedback; mode is 0 servo, 1 motor, or 255 if unavailable |
| `0x23` | `type port address enabled budget:u16 period:u16 present ranging error:u32 ...` | Saved VL53L4CD configuration and current port state |
| `0x24` | `type port xtalk:u16 signal:u16 sigma:u16 low:u16 high:u16 window enabled ...` | Saved VL53L4CD optical filters and threshold-detection configuration |
| `0x25` | `type state saved battery address[6] vid:u16 pid:u16 report_id has_input samples:u32` | DualSense host status; state 1 ready, 2 scanning, 3 connecting, 4 connected, 5 error |
| `0x26` | `type index rssi:i8 name_length address[6] name[10]` | A device found by the Classic-Bluetooth scan; name is truncated to 10 bytes for this fixed-size frame |

For a completed scan, byte 6 (`detail`) is the number of servos found. For successful ID assignment it is the new ID. For a mode change it is the saved mode. For motor control it reports whether torque is enabled.

## 0x31 — referencja korpusu (20 bajtów, 50 Hz)

Wyłącznie telemetria. Bajt 1: bit 0 = dostępna referencja, bit 1 = aktywny okresowy
planner LIPM. Offsety 2 i 4: przesunięcie X/Z, int16 LE, 0,1 mm. Offsety 6 i 8:
zadane korekcje roll/pitch, int16 LE, 0,01°. Offset 10: tick uint32 LE. Pozostałe
bajty zarezerwowane. Ramka przychodzi po statusie i odometrii, przed czterema
ramkami nóg. Starszy klient może ją pominąć. Nowy klient wykorzystuje pozę ESP
zamiast odtwarzać fazę trajektorii z częstotliwości odświeżania okna.
