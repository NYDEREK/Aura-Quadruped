# Stabilna wersja Aury — 1.10.2026

Potwierdzone na robocie: tryb 3 chodzi dobrze.

| Co | Gdzie |
|---|---|
| Firmware | tag `aura-chodzi-2026-10-01` (gałąź `stable-2309`) — kod ruchu wgrany 23.09.2026 19:26 + poprawka Wi‑Fi (bez skanowania przy uzbrojonym robocie) |
| Ustawienia (NVS: kalibracja serw, profile chodu, pad) | `backups/nvs-dobra-konfiguracja-2026-10-01.bin` (lokalnie, nie w gicie) |
| Profil trybu 3 | 44 mm / 41 mm / 1,0 Hz / 52 % |

## Powrót do tej wersji

Aura rozbrojona, podpięta USB-C:

```sh
cd ~/Documents/GitHub/Aura-Quadruped
tools/restore_stable.sh
```

Skrypt przełącza repo na tag, wgrywa firmware (z kopią ustawień, bez kasowania NVS) i zapisuje ustawienia z kopii.
