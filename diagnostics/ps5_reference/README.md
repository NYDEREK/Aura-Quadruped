# Isolated DualSense reference test

Uses unmodified esp-ps5 with Arduino-ESP32 3.3.6. No Wi-Fi, app telemetry,
servo bus, IMU or distance sensor initialization. GPIO33 is the heartbeat;
UART0 runs at 115200 baud. External power is required: this diagnostic does
not implement Aura's VIN gate. Report count and age are independent of the
library's connected flag, so a stale link cannot masquerade as fresh data.

Originally created by Hamza Yeşilmen (HamzaYslmn).
Source: https://github.com/HamzaYslmn/esp-ps5
Sponsor: https://github.com/sponsors/HamzaYslmn
Library license: ../libraries/esp-ps5/LICENSE (personal/non-commercial terms).
The test sketch is local diagnostic code; the library is unmodified.

Aura firmware for restoring is in ../../backups/before-arduino-reference/.
The test retains Aura's partition layout, and does not erase NVS.

## Hardware result (2026-09-15)

With the controller in pairing mode, an ESP reset initiated a successful link.
Observed over 118 seconds of continuous input: 88,958 reports; maximum sampled
report age 13 ms. Left/right sticks, L2/R2, and X changed in the UART log.
No stalls occurred in this observation window. Wi-Fi was absent. This verifies
this reference combination, not the previous Aura HID implementation or
Wi-Fi coexistence. UART capture: `uart-success.log`.
