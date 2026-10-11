# Balans w chodzie — tryb 4 (11.10.2026)

## Problem

W trybie 3 Aura przed ruchem „waha się”, a w trakcie kroku opada na jedną z nóg
w powietrzu zamiast iść na dwóch. Przyczyny są dwie: fizyczna i w kodzie.

### Fizyka: linia dwóch stóp

Gdy po przekątnej stoją tylko dwie stopy, podparciem jest odcinek. Noga
sterowana pozycyjnie nie wytworzy momentu wokół tej linii, więc poza tułowia nie
zatrzyma spadania środka masy (CoM) w bok. To odwrócone wahadło liniowe (LIPM,
Kajita 2003): `c'' = (g/h)·d`, gdzie `d` to odległość CoM od linii, a stała czasowa
`√(h/g) ≈ 0,146 s` przy h ≈ 208 mm. Odchylenie rośnie jak `e^(t/0,146 s)`.

| Profil | Czas na dwóch stopach | Wzrost błędu w jednym kroku |
|---|---|---|
| Tryb 3: 1,0 Hz / 52 % | 0,48 s | ×26 |
| Tryb 4 do 11.10: 1,5 Hz / 70 % | 0,20 s | ×4 |

Jedyny skuteczny środek w fazie dwunożnej to **miejsce postawienia następnych
stóp** — MIT Cheetah 3 (Bledt i in. 2018, równanie 6) i capture point (Pratt i in.
2006). Przy ×26 nawet ono nie nadąża, dlatego tryb 4 ma krótką fazę dwunożną.

### Kod: dwa znaki tego samego IMU

- Regulator postawy (stanie) liczy korekcję jako `referencja − pomiar`. Jest stabilny
  tylko wtedy, gdy zmierzony kąt rośnie razem z zadanym przechyłem w konwencji IK
  (roll > 0 opuszcza lewy bok). Balans w staniu działa, więc tak jest.
- Prowadzenie nogi w powietrzu (`estimate_swing_body`) przyjmowało odwrotny znak.
  Gdy robot przechyla się na nogę w powietrzu, ta noga była dodatkowo opuszczana
  i uderzała w ziemię — dokładnie „opadanie na jedną nogę”.
- Wcześniejszy capture point (26.09, cofnięty 1.10) miał tę samą odwróconą
  konwencję, więc działał jak dodatnie sprzężenie.

W trybie 3 ten błąd paradoksalnie pomaga: przy 0,48 s robot i tak by upadł, a stopa
lądująca wcześniej go łapie. Dlatego **tryb 3 zostaje bez zmian**.

## Tryb 4 — co robi

| Element | Działanie |
|---|---|
| Profil | zapisany profil Trot z NVS, edytowalny w aplikacji po rozbrojeniu (11.10: 60 mm / 54 mm / 1,25 Hz / 58 %); przyspieszenia serw jak w trybie 3. Warunek: kontakt > 50 % i (kontakt − 50) × 1000 ≥ częstotliwość [cHz] × 20, inaczej tryb 4 nie umiałby się zatrzymać na czterech stopach — firmware odrzuca taki profil i nie startuje truchtu |
| Znak IMU | sprawdzony na robocie 11.10: roll z IMU ma znak odwrotny do kinematyki, pitch zgodny (`ROBOT_IMU_ROLL_SIGN`); ten sam w regulatorze postawy i w capture point trybu 4 |
| Postawa w ruchu | korekcja IMU zamrożona na czas truchtu (na linii dwóch stóp nie da się jej wykonać, a jej przechył to sygnał upadku) |
| Stawianie stóp | stopa ląduje w średnim przesunięciu postawionych stóp + `ξ = h·przechył + √(h/g)·h·prędkość przechyłu`, limit ±40 mm, cel zamrożony po 85 % przenoszenia |
| Środek masy | każde lądowanie uczy przesunięcie CoM (0,05 korekty, limit ±30 mm); tułów przesuwa się o −CoM, więc korekta stóp zanika |
| Start / stop | start z czterech stóp (faza 0,5), stop dopiero gdy wszystkie cztery stoją; drążek filtrowany (τ = 0,25 s) |
| Serwa | ACC 0 (fabryczny „bez rampy”) i limit planera 120 rad/s²; przy ACC 254 serwo ma 39 rad/s², a swing 0,2 s potrzebuje ~130 |

Tryby 1, 2 i 5: przesunięcia stóp po truchcie wygasają w ~1 s; tułów jest
przesuwany płynnie. Tryb 3 ma dokładnie dotychczasową ścieżkę.

## Symulacja (tests/test_robot_walk_balance.c)

Model: LIPM na linii dwóch stóp, nogi niosą tułów z opóźnieniem 50 ms, IMU
spóźnione o 20 ms z szumem 0,2° i 0,5°/s, CoM przesunięty o (−6, +5) mm względem
założenia, losowe pchnięcia przy oderwaniu stóp.

| Wariant | Uderzenia nogi przed czasem | Przechył | Wynik |
|---|---|---|---|
| Tryb 3, obecny firmware | 14 z 28 | 6,9° | idzie, łapiąc się nogą |
| Tryb 3, tylko poprawiony znak | — | — | upada |
| Tryb 3, nowy balans | — | — | upada |
| Tryb 4, bez capture point | 0 | 3,2° | idzie |
| Tryb 4, nowy balans | 0 | 1,7° | idzie, CoM nauczony (−6,3; 4,8) |
| Tryb 4, nowy balans, pchnięcia 25 mm/s | 0 | 2,1° | idzie |
| Tryb 4, capture point z odwróconym znakiem | — | — | upada |

Sprawdzenie osiągalności (IK + ogranicznik osi): swing 0,2 s przy 120 rad/s² ma
błąd śledzenia 2,5° i szczyt 26 mm; przy 39 rad/s² — 10,9° i 21 mm.

To model zredukowany: bez bezwładności nóg, poślizgu, luzów i realnej dynamiki
ST3215. Pokazuje mechanizm i znaki, nie zastępuje próby na robocie.

## Test na robocie (uzbraja i prowadzi użytkownik)

1. **Znak IMU, rozbrojony:** przechyl Aurę lewym bokiem w dół. W UART `status` linia
   `IMU ... roll/pitch` — roll powinien **rosnąć**. Nosem w górę — pitch **rośnie**.
   Jeśli jest odwrotnie, nie testuj trybu 4 i zgłoś.
2. Tryb 1, uzbrojony: stanie bez zmian.
3. Tryb 4 (4 kropki, żółta dioda 2), drążek lekko do przodu, robot asekurowany ręką.
   Oczekiwane: start bez kołysania, dwie nogi w powietrzu, krótkie szybkie kroki.
   W `status`: `Trot (mode 4): capture=…` — przy przechyle w lewo `capture` Z rośnie.
4. Jeśli serwa stukają lub drgają przy ACC 0 — rozbrój i zgłoś (dobierzemy limit).
5. Po kilkunastu krokach `learned CoM` powinien ustalić się na stałej wartości.

Powrót: `tools/restore_stable.sh` (tag `aura-led-bateria-2026-10-10`).
