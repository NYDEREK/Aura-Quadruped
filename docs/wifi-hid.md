# Aura: Wi-Fi telemetry and Classic Bluetooth controller

`main/main.c` samples VIN on ADC1/GPIO34. Radios start only after stable VIN > 7.0 V. VIN <= 7.0 V or an ADC error shuts down both Wi-Fi and Bluetooth. GPIO33 heartbeat and wired diagnostics work independently of the radios. USB is used for programming; firmware cannot alter the PCB power paths.

`main/aura_radio.c` owns the Classic Bluetooth HID stack, command queue, and telemetry encoding. No BLE or SPP services are started. Controller pairing data stays in NVS across firmware updates. Press PS on a previously paired controller; new pairing is available from Aura's DualSense tab. This transport change does not assign controller buttons to robot motions.

`main/aura_network.c` joins the configured 2.4 GHz Wi-Fi in station mode. Credentials are in the ignored local `main/wifi_credentials.h` file (AURA_WIFI_SSID and AURA_WIFI_PASSWORD). Wi-Fi retries every 10 seconds without restarting Bluetooth. Espressif software coexistence is enabled.

The board advertises `_aura._tcp` with Bonjour/mDNS, hostname `aura-main-board.local`, TCP port 4242. The Mac must be on the same LAN with local-network access allowed; client isolation blocks discovery/communication. Router internet access is not needed. There is no OTA service.

TCP frames are `channel:u8, length:u8, payload:length`, with lengths 1..20. Channel 0 carries existing commands; channel 1 telemetry; channel 2 events. Receivers handle split and coalesced TCP reads. A stalled client is disconnected rather than building an unbounded telemetry backlog. One desktop client is served at a time. The protocol is intended for the trusted WPA2 LAN; it has no additional application authentication and must not be exposed through router port forwarding.

`desktop/MainBoardControl/AuraConnection.swift` discovers Bonjour services, receives framed TCP data and retries after 3 seconds. Commands and sensor parsers are shared with the previous UI behavior. Pairing a controller does not close or reconfigure the network connection.

## Validation

Build firmware with the ESP-IDF Python environment matching the configured build. Build the Mac app with `desktop/MainBoardControl/build.sh`. Verify flash hashes, then check UART for an assigned Wi-Fi address. Verify telemetry and read-only command replies through TCP. Hardware checks remaining until explicitly observed: pad + app concurrently, external-power removal/reapplication while USB stays connected, and cold startup with PS pressed on the saved controller.

Verified on 2026-09-15: firmware and macOS app builds pass; flash hash verified; board receives 192.168.0.33; split/coalesced read-only TCP commands pass; current telemetry runs around 10 Hz; a 20-second test received 695 frames while a controller connection attempt was in progress. App log confirms both TCP ready and receipt of telemetry. Bluetooth runs on core 0; Wi-Fi runs on core 1. Simultaneous successful HID input and Wi-Fi remains pending controller pairing verification.

DualSense parser regression test: `cc -I main tests/test_dualsense_report.c -o /tmp/test_dualsense_report && /tmp/test_dualsense_report`. It covers the 9-byte Bluetooth 0x01 payload, the extended 0x31 payload, button/trigger offsets, variable transport header, and truncated/unknown reports. Protocol references: https://github.com/nondebug/dualsense and https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32/parser/uni_hid_parser_ds5.c . The parser previously rejected the short Bluetooth reports and used the wrong layout; corrected.

## Independent diagnostic build switches

`main/aura_features.h` independently selects `AURA_ENABLE_WIFI` and
`AURA_ENABLE_DUALSENSE` (0 or 1; rebuild and flash after changes).
The current diagnostic build uses Wi-Fi=0, DualSense=1. It does not start
the Wi-Fi network service or the app telemetry task. The external VIN gate
still controls radio startup. Bluetooth modem sleep is disabled for the
ongoing diagnostic comparison.

UART commands: `pad connect` uses the saved controller address;
`pad disconnect`, `pad scan`, and `pad list` allow basic diagnosis without
Aura. Status output includes decoded controls, freshness, raw/decoded report
counts and free heap. This build is an isolation test, not a confirmed fix
for the stalled HID connection.
# Wskaźniki pada: stan i wybrany tryb

DualSense udostępnia jeden kolor RGB dla obu boków lightbara i pięć
niezależnych białych kontrolek pod touchpadem. Raport nie udostępnia
oddzielnych kolorów lewej i prawej strony. Układ raportu pochodzi z
[hid-playstation.c](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-playstation.c),
`dualsense_output_report_common` i flag LIGHTBAR/PLAYER_INDICATOR.

Bar i pasek robota nadal wskazują stan: pomarańczowy — rozbrojony,
zielony — uzbrojony, czerwony — test kalibracyjny, migający czerwony —
zatrzaśnięty błąd. Liczba białych kropek wskazuje **wybrany** profil:

| Kropki | Tryb |
|---:|---|
| 0 | Stanie |
| 1 | Trot |
| 2 | Krok 1× |
| 3 | Run |
| 4 | Climb |
| 5 | Chodzenie na 3 łapach |

Kontrolki używają wycentrowanych wzorów Sony: 0x00, 0x04, 0x0a, 0x15,
0x1b, 0x1f. Błąd i test kalibracji wygaszają kropki. W trybie Stanie
wychylenie drążka może uruchomić Trot, ale wskazanie nadal dotyczy
wybranego profilu, zgodnie z wyborem R1.

ESP odświeża wspólny raport koloru i kontrolek z dotychczasową częstotliwością
10 Hz; nie dodaje osobnego strumienia BT. Raport ma nadal 79 bajtów z nagłówkiem
HID, sekwencją i CRC32; pola i flagi silników, triggerów, audio i zasilania
pozostają nieaktywne. Obsługa działa bez aplikacji, stan jest przywracany po
ponownym połączeniu pada. Nie zmienia to uzbrajania ani żadnej kalibracji.
