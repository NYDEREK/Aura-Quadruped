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

## Tryb 4 — zrównoważony trucht w granicach serw (25.10.2026)

Symulacja z planerem i IK firmware (`robot_gait_feasibility.c`) pokazała dwa
powody, dla których tryb 4 nie chodził:

| Profil | Szczyt prędkości stawu | Szczyt przyspieszenia | Na dwóch stopach | Wzrost błędu |
|---|---|---|---|---|
| Tryb 3 (Run 44/41/1,0 Hz/52 %) | 137°/s | 38 rad/s² | 0,47 s | ×36 |
| Tryb 4 do 25.10 (Trot 60/54/1,25 Hz/58 %) | 255°/s | 102 rad/s² | 0,32 s | ×11 |
| Tryb 4 zalecany (Trot 40/20/1,50 Hz/70 %) | 178°/s | 104 rad/s² | 0,20 s | ×4 |

1. **Serwa nie nadążały.** ST3215 robi ~250°/s bez obciążenia przy 11–12 V, a rampa
   ACC 254 ogranicza przyspieszenie do 39 rad/s². Stary profil chciał więcej, więc
   stopa nie podnosiła się i nie lądowała tam, gdzie zakładał plan.
2. **Nic nie łapało upadku w bok.** Capture point był wyłączony. Model z
   `tests/test_robot_walk_balance.c` pokazuje, że stawianie stóp utrzymuje trucht
   bez podpierania się tylko przy fazie dwunożnej ≤ ~0,2 s (z nakładaniem się czterech
   stóp). Dłuższa faza: robot spada w bok i łapie się nogą, tilt 13–25°.

| Element | Działanie |
|---|---|
| Tempo | przy starcie i po każdej zmianie profilu Trot, wysokości lub rozstawu firmware symuluje cykl i zwalnia zegar chodu, aż staw ≤ 3,4 rad/s i ≤ 110 rad/s². Zapisany profil się nie zmienia; UART pisze `Trot profile ... -> runs at X Hz`, aplikacja pokazuje „Runs at” |
| Serwa w trybie 4 | ACC 0 (bez rampy), planer ≤ 3,5 rad/s i ≤ 120 rad/s² |
| Stawianie stóp | capture point z IMU włączony (ten sam znak co regulator postawy, sprawdzony 11.10); postawa (przechylanie tułowia) tylko w trybie 1 |
| Plan tułowia | okresowy LIPM jak dotąd, przeliczany najwyżej 10 razy na sekundę |
| Start / stop | bez zmian: start z czterech stóp, stop gdy wszystkie cztery stoją |

**Profil do ustawienia w aplikacji** (IK Test → tryb 4, robot rozbrojony):
krok 40 mm, wysokość kroku 20 mm, 1,50 Hz, kontakt 70 %. Panel „Servos” pod suwakami
pokazuje szczyty, czas na dwóch stopach i tempo dla każdej zmiany.

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
3. Tryb 4 (4 kropki, żółta dioda 2) z profilem 40/20/1,50/70, drążek lekko do przodu,
   robot asekurowany ręką. Oczekiwane: krótkie niskie kroki, dwie nogi w powietrzu,
   bez podpierania się. W `status`: `Trot (mode 4): capture=…` — przy przechyle
   w lewo `capture` Z rośnie.
4. Jeśli serwa stukają lub drgają przy ACC 0 — rozbrój i zgłoś (dobierzemy limit).
5. Po kilkunastu krokach `learned CoM` powinien ustalić się na stałej wartości.

Powrót: `tools/restore_stable.sh` (tag `aura-stabilna-2026-10-25`).
