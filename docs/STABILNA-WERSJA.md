# Punkty powrotu Aury

## Najnowszy — 10.10.2026: pasek LED i bateria

Kod ruchu bez zmian względem 1.10 (tryb 3 chodzi dobrze). Doszedł 8-diodowy pasek
(stan, tryb, bateria 10,0–12,6 V) i blokada uzbrojenia przy ≤ 10,0 V.

| Co | Gdzie |
|---|---|
| Firmware | tag `aura-led-bateria-2026-10-10` (gałąź `stable-2309`) |
| Ustawienia (NVS: kalibracja serw, profile chodu, pad) | `backups/nvs-led-bateria-2026-10-10.bin` (lokalnie, nie w gicie) — aktualna kalibracja, w tym nowa kalibracja serw 7/8/9 |

## 1.10.2026 — pierwsza potwierdzona wersja

| Co | Gdzie |
|---|---|
| Firmware | tag `aura-chodzi-2026-10-01` — kod ruchu wgrany 23.09.2026 19:26 + poprawka Wi‑Fi (bez skanowania przy uzbrojonym robocie) |
| Ustawienia | `backups/nvs-dobra-konfiguracja-2026-10-01.bin` |
| Profil trybu 3 | 44 mm / 41 mm / 1,0 Hz / 52 % |

**Uwaga:** ta kopia NVS ma starą kalibrację serw 7/8/9 (sprzed ponownej kalibracji tej
nogi). Do tego firmware z obecną kalibracją użyj kopii z 10.10.

## Powrót

Aura rozbrojona, podpięta USB-C:

```sh
cd ~/Documents/GitHub/Aura-Quadruped
tools/restore_stable.sh                       # najnowszy punkt powrotu
tools/restore_stable.sh aura-chodzi-2026-10-01 backups/nvs-led-bateria-2026-10-10.bin
```

Skrypt przełącza repo na tag, wgrywa firmware (z kopią ustawień, bez kasowania NVS) i zapisuje ustawienia z kopii.
