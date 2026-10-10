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

Lightbar i dioda 1 paska wskazują stan: pomarańczowy — rozbrojony,
zielony — uzbrojony, czerwony — test kalibracyjny, migający czerwony —
zatrzaśnięty błąd. Liczba białych kropek pada i kolor diody 2 paska
wskazują **wybrany** tryb:

| Kropki | Dioda 2 | Tryb |
|---:|---|---|
| 1 | niebieska | Balans — stoi, IMU trzyma tułów w poziomie |
| 2 | turkusowa | Poza tułowia — lewy drążek przechyla tułów (±15° na boki i przód/tył) |
| 3 | fioletowa | Chód — zapisany profil Run (44 mm / 41 mm / 1,0 Hz / 52 %) |
| 4 | żółta | Stoi (zarezerwowany) |
| 5 | różowa | Stoi (zarezerwowany; chód na 3 łapach odłożony) |

## Sterowanie padem

| Przycisk | Działanie |
|---|---|
| Create | uzbrój / rozbrój |
| D-pad → / ← | następny / poprzedni tryb, bez zawijania; ignorowane, gdy w trybie 3 drążek jest wychylony |
| D-pad ↑ / ↓ | wysokość tułowia ±30 mm (płynnie, 60 mm/s), w zakresie 120–250 mm; po każdym uzbrojeniu wraca zapisana wysokość |
| Lewy drążek | tryb 2: przechył tułowia; tryb 3: ruch przód/bok |
| Prawy X | tryb 3: skręt |
| L1 | tryb 3: obrót w miejscu |
| × | skok |
| Options | test na 3 nogach (bez zmian) |

R1 i prawy drążek w pionie nie mają już przypisanej funkcji. Aura startuje w trybie 1.
W trybie 2 regulator IMU ma referencję przesuniętą o zadany przechył: kompensuje
pochylenie podłoża, ale nie kasuje przechyłu z drążka.

## Pasek LED (8 diod)

| Dioda | Znaczenie |
|---|---|
| 1 | Stan robota (jak lightbar) |
| 2 | Wybrany tryb (tabela wyżej); zgaszona przy błędzie i kalibracji |
| — | Jasność całego paska 0–100 % (domyślnie 20 %): aplikacja, zakładka LED, albo UART `led <0-100>`; zapis w NVS `aura_led/bright` |
| 3–8 | Bateria 3S: 10,0 V = 0 %, 12,6 V = 100 %, każda dioda to 1/6 zakresu |

Kolor paska baterii: zielony od 11,0 V, pomarańczowy poniżej 11,0 V,
czerwony przy ≤ 10,0 V. Przy ≤ 10,0 V świeci jedna czerwona dioda i Aura
odmawia uzbrojenia (`ESP_ERR_NOT_ALLOWED`, 0x10D) — z pada, aplikacji i
UART. Napięcie pochodzi z INA226 (VBUS = VIN; ADC dzielnika tylko awaryjnie),
jest filtrowane przez 2 s, a próg w górę ma histerezę 50 mV, więc pasek nie
miga pod obciążeniem chodu. Bez żadnego odczytu napięcia diody baterii są
zgaszone, a uzbrojenie jest zablokowane. Robot już uzbrojony nie jest
rozbrajany automatycznie przy spadku napięcia.

Kontrolki używają wycentrowanych wzorów Sony: 0x00, 0x04, 0x0a, 0x15,
0x1b, 0x1f (tryb 1 = jedna kropka … tryb 5 = pięć). Błąd i test kalibracji
wygaszają kropki.

ESP odświeża wspólny raport koloru i kontrolek z dotychczasową częstotliwością
10 Hz; nie dodaje osobnego strumienia BT. Raport ma nadal 79 bajtów z nagłówkiem
HID, sekwencją i CRC32; pola i flagi silników, triggerów, audio i zasilania
pozostają nieaktywne. Obsługa działa bez aplikacji, stan jest przywracany po
ponownym połączeniu pada. Nie zmienia to uzbrajania ani żadnej kalibracji.
