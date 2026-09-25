# Przegląd kinematyki i chodu — 26.09.2026

Wyniki uzyskane testami hosta (`tests/`) i skryptami na tym samym kodzie C, który działa na ESP.

## Co jest poprawne

- **IK/FK nogi** (`robot_kinematics.c`): rozwiązanie zamknięte ab/ad + biodro + kolano jest spójne z FK (sprawdzone analitycznie i testem round‑trip). Układy lokalne lewej i prawej strony to właściwe obroty (det = +1), a lustrzane odbicie realizuje znak kolana (+ lewe, − prawe), zgodnie z kalibracją.
- **Trajektoria swing** = `FootSwingTrajectory` z Mini Cheetah (kubiczny Bézier, apeks w połowie).
- **Faza stóp i skręt** — wspólny twist SE(2), odległości podpartych stóp stałe.

## Znalezione problemy (przyczyny przewracania)

1. **Trot bez sprzężenia zwrotnego stawiania stóp.** VPSP z Cheetah 3 rzutowany na przekątną daje w trocie praktycznie środek ramy → korpus kołysze się tylko ±1,9 mm („sztywny jak kij”). Robot stoi wtedy na linii dwóch stóp w równowadze chwiejnej; stała czasowa przewracania 1/ω = √(h/g) ≈ 0,15 s. Cheetah 3 stabilizuje to **heurystyką położenia stopy** (Raibert: `p = p_hip + T_st/2·v + k(v − v_cmd)`) i sterowaniem siłowym. U nas stopa ląduje zawsze w miejscu z planu — nie ma korekcji z IMU, więc każdy przechył narasta aż łapa w powietrzu dotknie ziemi (3 łapy na ziemi, szuranie, skręcanie).
2. **Swing trwa za długo.** Limit przyspieszenia w plannerze (39 rad/s²) wyprowadzono z rejestru ACC = 254 serwa. Przez to zadane 1,4 Hz trotu jest zwalniane do **0,70–0,73 Hz**, a faza na dwóch nogach trwa ~0,6 s (4× dłużej niż 1/ω). Bez tego limitu ta sama ścieżka mieści się w 1,21 Hz. W bibliotece Feetech (`SMS_STS.h`) domyślne ACC = 0 oznacza brak rampy (maks. przyspieszenie) — do weryfikacji na serwie.
3. **Korekcja IMU w ruchu tylko poziomuje korpus.** Obrót korpusu (Kp 0,45, limit 35°/s) nie przesuwa środka masy nad linię podparcia i jest wolniejszy od przewracania.
4. **Plan przebudowywany co takt.** Klucz cache zawiera przefiltrowane gałki i offset CoM zależny od bieżącej korekcji IMU, więc przy chodzie z włączonym IMU cały plan (64 próbki × IK × do 6 iteracji skalowania czasu) liczy się od nowa co 20 ms, a efektywna częstotliwość „skacze”. Obciąża pętlę 50 Hz.
5. **Tryb „3 łapy” jest fizycznie niestabilny statycznie.** Przy jednej nodze stale w górze każdy krok zostawia tylko 2 podpory; plan wymaga przesunięcia korpusu nawet o **120–138 mm** (nad oś tylnych nóg), czyli CoM dokładnie na linii dwóch stóp. Kierunek przesunięcia w planie jest poprawny (od uniesionej nogi), ale najmniejszy błąd przewraca robota na stronę uniesionej nogi.

## Plan poprawek (pierwotny)

1. Heurystyka stawiania stóp Cheetah 3 / Raibert z prędkością i przechyłem z IMU (capture point), liczona na ESP w każdym takcie dla nóg w swingu.
2. Poprawny limit przyspieszenia (ACC = 0 lub zmierzony) → krótszy swing, trot 1,2–2 Hz.
3. Balans IMU w ruchu jako przesunięcie CoM (nie tylko obrót korpusu).
4. Cache planu niezależny od szumu wejść; plan przeliczany przy zmianie profilu, nie co takt.
5. Tryb „3 łapy”: statycznie stabilny wariant (ruch korpusu → krok → ruch korpusu, z marginesem) albo usunięcie trybu.

Bez zmian: wymiary robota w aplikacji, kalibracja, przypisanie serw.

## Wprowadzone poprawki — 26.09.2026

| # | Zmiana | Pliki |
|---|---|---|
| 1 | **Stawianie stóp wg Cheetah 3, równanie (6)**: do każdej stopy w swingu dodawany jest człon capture point `sqrt(z0/g)·(v − v_des)` + dryf korpusu `z0·(nieplanowany przechył)`. Prędkość przewracania liczona z żyroskopu MPU6050 (bez części zadanej przez regulator postawy). Stopa w podparciu trzyma offset z chwili postawienia (nie ślizga się), swing płynnie przechodzi do nowego offsetu. Limit ±45 mm. Działa tylko z włączonym balansem IMU, w trybach 1, 3, 5 i w dynamicznym chodzie 2/4. | `main/robot_foot_placement.{c,h}`, `main/robot_gait.c` |
| 2 | **ACC serwa = 0** w ramkach chodu (brak wewnętrznej rampy ST3215) i limit planera 120 rad/s². Kalibracja, uzbrajanie i stanie zostają na łagodnym limicie (ACC 254 / 39 rad/s²). | `main/robot_gait.c` |
| 3 | **Nowy domyślny trot**: 50 mm / 35 mm / 2,0 Hz / 55 % → faza na dwóch nogach ~0,25 s (było ~0,6 s). Zapisane w NVS profile użytkownika NIE są nadpisywane. | `main/robot_locomotion.c`, `AuraConnection.swift` |
| 4 | **Plan korpusu nie liczy się co takt**: wejście z pada kwantowane co 2 %, offset CoM co 1 mm, przebudowa najwyżej co 100 ms (w tym samym trybie). | `main/robot_gait.c`, `main/robot_body_trajectory.{c,h}` |
| 5 | Linia `Capture point: offset x=… z=… mm` w komendzie UART `status`. | `main/main.c` |

Weryfikacja: 11 zestawów testów hosta (nowy `test_robot_foot_placement.c`: znaki, równanie (6), brak poślizgu, symulacja LIPM z pchnięciem bocznym), firmware kompiluje się w ESP-IDF 6.1 bez ostrzeżeń w kodzie Aury.

Wynik symulacji (pchnięcie 120 mm/s w bok): faza dwóch nóg 0,15 s i 0,25 s → robot łapie równowagę; 0,35 s i 0,60 s → przewraca się mimo korekcji. Dlatego krótki swing jest warunkiem koniecznym.

### Tryb „3 łapy”

Bez zmian w planie: fizycznie przy jednej nodze stale w górze każdy krok zostawia 2 podpory, więc tryb nie może być statycznie stabilny. Dostał tylko korekcję stóp z pkt 1. Traktować jako eksperyment.

## Jak testować (robot uzbraja i testuje użytkownik)

1. Po wgraniu: `status` przez UART → `Planner: max=… us` powinno być wyraźnie < 20000 us, `overruns` nie powinno rosnąć.
2. **Test znaku (najważniejszy)**: robot uzbrojony, balans IMU włączony, trot w miejscu, trzymaj robota. Przechyl go lekko lewą stroną do góry → w `status` `z` ujemne (stopa idzie w prawo, pod spadający środek masy). Jeśli jest odwrotnie — natychmiast rozbroić i zgłosić (odwrócony znak = dodatnie sprzężenie).
3. Trot w miejscu bez trzymania, potem lekkie pchnięcie w bok — robot powinien zrobić krok w stronę pchnięcia.
4. Jeśli serwa po ACC = 0 drgają/stukają: zgłoś — wtedy dobierzemy ACC (np. 150) i limit planera.
5. Dla własnych profili trotu: celuj w ~1,8–2,2 Hz, duty 52–58 %, krok 40–60 mm, wysokość 30–40 mm.
