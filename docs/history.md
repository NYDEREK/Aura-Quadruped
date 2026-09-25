# Historia projektu — najważniejsze ustalenia (wrzesień 2026)

Streszczenie pracy z czatu „Aura Main Board Code” (13–25.09.2026). Hasła i adresy sieci celowo pominięte.

## Płytka Aura MainBoard (ESP32‑WROOM‑32E)

- LED statusu: GPIO33 (aktywny stanem wysokim). USB‑UART: CP2102, bezpiecznik 0,5 A na USB.
- Magistrala serw ST3215 (TTL półdupleks, 1 Mbit/s): RX GPIO16, TX GPIO17, kierunek GPIO18, bufor 74LVC1G125/126. ESP spaliło się, gdy serwa były podpięte przy zasilaniu tylko z USB — zalecane rezystory szeregowe 330 Ω zamiast 0 Ω na RX/TX (R10 było 100 Ω).
- I²C (SDA GPIO25, SCL GPIO26): INA226 (0x40) — pomiar prądu/mocy serw; MPU6050 na złączu S1 (INT na linii XSHUT1 = GPIO14); VL53L4CD na S2 (XSHUT GPIO15, adres przenoszony na 0x31).
- Pomiar VIN: GPIO34 (ADC1_CH6), dzielnik 100k/22k. Radio (Wi‑Fi + BT) startuje dopiero przy VIN > 7 V — na samym USB Bluetooth się nie podnosił.
- WS2812: GPIO32.
- ICM‑42688‑P (klon HXY) na SPI nigdy nie odpowiedział poprawnie mimo kilku wymian układu i przecięcia GND na pinach 9/11 — zastąpiony modułem MPU6050 na S1.
- Czujnik VL53L4CD wymagał dolutowania R4 i poprawienia zasilania C1/C2.

## Komunikacja

- Aplikacja macOS ↔ Aura: Wi‑Fi, TCP `aura-main-board.local:4242` (Bonjour). ~50 Hz telemetrii.
- DualSense: Bluetooth Classic HID bezpośrednio do ESP (BLE + Classic jednocześnie powodowało zrywanie). Odczyt pada ~60 Hz+.
- Pad: Create = uzbrój/rozbrój, R1 = cykl trybów 0–5 (diody gracza), L1 = obrót w miejscu, X = skok, Options = test na 3 nogach. Lightbar: czerwony = kalibracja, pomarańczowy = rozbrojony, zielony = uzbrojony.

## Robot

- Nogi: LF, RF, LR, RR. W każdej nodze: serwo 1 = ab/ad (u góry), 2 = biodro, 3 = kolano.
- Wymiary domyślne (kod): rama 360 × 130 mm (środki osi), ab/ad 35 mm, udo 130 mm, podudzie 205 mm. Wartości użytkownika są w NVS/aplikacji — nie zmieniać.
- Kalibracja w kolejności FR, FL, BL, BR: pozycja referencyjna 0/0/±90° (kolano +90° lewe, −90° prawe), test kierunku ±12° z powrotem do referencji, potem krańce. Dane kalibracji są w NVS i mają backup w aplikacji.
- Problem przejścia enkodera przez 0/4095 (pełny obrót nogi LR) — rozwiązany przez 15‑bitowy zapis ze znakiem i wirtualne zero.
- Zasilacz 17 A; serwa pracują z maks. prędkością 3400 kroków/s.

## Stan chodu na 25.09.2026

- Stanie z balansem IMU działa dobrze.
- Trot po ręcznym dostrojeniu długości/wysokości/częstotliwości kroku chodzi, ale przechyla się na jedną stronę (3 łapy na ziemi, szuranie → skręcanie).
- Tryb „3 łapy” (5) przechyla środek masy niewłaściwie; regres po aktualizacji 25.09.
- Analiza przyczyn: [kinematics-review.md](kinematics-review.md).
