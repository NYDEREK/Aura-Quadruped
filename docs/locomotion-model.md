# Trajektoria korpusu i stóp Aury

## Wykonanie

Planner pracuje w 50 Hz na ESP32 bez aplikacji. Ta sama faza steruje kontaktami,
stopami i korpusem. Aplikacja odbiera cele i pozycje serw oraz zadaną pozę korpusu
(ramka 0x31). Test IK kompiluje te same pliki C planera, zamiast przepisywać
równania do Swift. Zmiana nie modyfikuje rekordów NVS: kalibracji, geometrii,
wysokości, rozstawu, offsetu CoM ani zapisanych profili kroków.

## Stopy i układy odniesienia

Współrzędne: X przód, Y góra, Z lewo. Rama odniesienia chodu przemieszcza się
z nominalną prędkością, a trajektoria korpusu opisuje odchylenie od tej ramy.
Dla ruchu prostoliniowego v = długość_kroku / (duty * okres).

- Podczas podparcia stopa jest nieruchoma względem świata. W ramie chodu jej
  pozycja zmienia się liniowo. Ease-in/ease-out podpartej stopy wprowadzało
  dodatkowe, nieplanowane przyspieszenie korpusu.
- Przenoszenie jest kubicznym Bézierem MIT w świecie. Przed IK odejmujemy drogę
  ramy podczas swing. Prędkość względem ramy jest dzięki temu ciągła przy
  oderwaniu i postawieniu. Wysokość nadal używa dwóch połówek Béziera MIT.
- robot_body_trajectory_foot() jest wspólne dla przewidywania podparcia i celu
  rzeczywistej nogi i podglądu Swift. Skręt używa jednego wykładnika SE(2)
  dla wszystkich nóg, z jedną prędkością kątową. Nie dzielimy już długości
  kroku przez osobny promień każdej stopy. Przejście do zerowej prędkości
  kątowej jest ciągłe dzięki sinc/cosc, bez progu zmieniającego równanie.
  Spin i skręt przy ruchu mają tę samą konwencję znaku.
- Dla stałego twistu `(v, w)` podparcie jest `exp(-t*twist) p_anchor`.
  Końce swing wyznacza ten sam ruch ramy; kubiczny Bézier powstaje w świecie,
  a dopiero potem jest w całości transformowany do bieżącej ramy. Dzięki temu
  odległość dwóch podpartych stóp jest stała, również podczas skrętu.

## Korpus: cel podparcia a trajektoria CoM

Wcześniej zadawano bezpośrednio pozycję VPSP, obciętą do 24 mm w trocie. Taka
krzywa nie uwzględniała przyspieszenia korpusu. Geometryczna zgodność z podparciem
nie oznacza zgodności dynamicznej.

1. Równania (10)–(15) Cheetah 3 wyznaczają VPSP z fazy i położeń stóp.
2. Jako adaptacja do pozycyjnych serw Aury rzutujemy ten punkt na planowane
   podparcie. Dwie nogi wyznaczają odcinek; noga w powietrzu nie jest podporą.
   Wynik jest referencją ZMP, a nie bezpośrednio pozycją CoM. To jawna zmiana
   roli VPSP względem oryginalnego sterownika siłowego MIT.
3. Rozwiązujemy równanie Kajity (6)–(9) dla obu kierunków poziomych:

   c'' = (g/h) * (c - p), czyli p = c - (h/g) * c''.

   p oznacza ZMP, c pozycję CoM, h jego wysokość nad podłożem. Zachowano istniejące
   przybliżenie h = wysokość_ramy - 25 mm; nie jest to pomiar realnego CoM.
4. Pełny okres zawiera 64 próbki referencji podparcia. Dla liniowej interpolacji
   p rozwiązanie jest analityczne. Rozdzielenie ξ = c + c'/ω i η = c - c'/ω,
   gdzie ω = sqrt(g/h), daje ξ' = ω(ξ-p), η' = -ω(η-p). Pierwszą składową
   rozwiązujemy wstecz, drugą w przód z okresowym warunkiem brzegowym. Wynik
   uwzględnia przyszłe kontakty i ma ciągłą pozycję, prędkość i przyspieszenie
   dla ciągłego p. To wyprowadzenie z LIPM, nie implementacja LQR Kajity ani MPC.
5. Odczytujemy trajektorię korpusu dokładnie dla fazy stóp. Odejmujemy zapisany
   offset CoM obrócony zadaną orientacją korpusu. Nie obcinamy wyniku do 24 mm,
   bo obcięcie łamie równanie dynamiki. To offset w pozycji docelowej, nie
   estymacja rzeczywistego położenia CoM na podstawie IMU.
6. Translacja oraz istniejąca korekcja orientacji MPU przechodzą przez jeden
   transform sztywny przed IK. Regulator stania jest niezmieniony. Krańce
   serw i ograniczenia napędów nadal obowiązują.

Plan przebudowuje się tylko po zmianie danych wejściowych. Stały profil wymaga
w pętli jedynie interpolacji. Obliczenie nie alokuje pamięci i nie wykonuje I2C.
Nowa ramka korpusu ma 20 bajtów. Podgląd obraca korpus wokół jego środka, zgodnie
z IK, a nie wokół początku sceny na podłożu.

## Korekcja przenoszonej stopy — 23 września 2026

Wcześniej także stopa w powietrzu była przeliczana przez **zadaną** pozę korpusu.
Gdy realny korpus przechylał się szybciej niż reakcja serw, docelowe podniesienie
nie oznaczało dodatniej wysokości nad ziemią. Korekcja orientacji nóg podporowych
i geometryczne utrzymanie prześwitu nogi w powietrzu mają różne wejścia:

- Podparcie realizuje zaplanowaną pozę korpusu oraz istniejący regulator IMU.
- Przenoszenie używa transformacji z kodu MIT `ConvexMPCLocomotion`:
  `p_foot_body = R_measured^T * (p_foot_world - t_measured)`.
- Translację szacujemy kinematycznie: średnia
  `p_support_world - R_measured * FK(q_support)` po co najmniej dwóch aktualnych
  nogach podporowych. To rozwiązanie najmniejszych kwadratów przy znanej
  orientacji i założonym braku poślizgu. Nie jest to pomiar sił ani położenia CoM.
- Każdy odczyt TTL ma znacznik czasu. Przyjęty pomiar osi musi mieć mniej niż
  120 ms; do wyrównania czasów używana jest zmierzona prędkość enkodera, z
  ekstrapolacją ograniczoną do 60 ms. Błędne lub nieaktualne podparcie wyłącza
  estymację; wtedy pozostaje dotychczasowy transform zadanej pozy.
- Numeryczna macierz sceny to `Rz(pitch) * Rx(roll)`, X przód / Y góra / Z lewo.
  Kąty przyspieszeniomierza opisują wektor grawitacji w ramie, więc do tej
  macierzy używamy ich przeciwnych znaków względem uchwyconej referencji.
  Konwencja działającego regulatora stania i kalibracji serw jest zachowana.

Regulator podczas ruchu ma postać `clamp(Kp * e - Kd * omega, ±limit)`.
Kp pozostaje zapisanym ustawieniem użytkownika. Kd = 0,08 s jest początkowym
nastawem dla pozycyjnych ST3215, nie wartością z siłowego sterownika MIT.
Limit zmiany korekcji wynosi 35°/s i 220°/s². W staniu pozostało Kp = 1,
Kd = 0 i te same dotychczasowe limity. Przekroczenie nominalnych 25° nie kasuje
nagle zadanej korekcji do zera; ogranicza ją istniejący limit kąta. Nowa
estymacja stopy działa tylko wewnątrz 25° i przy aktywnym regulatorze IMU.

Wszystko wykonuje ESP w tej samej pętli 50 Hz. Nie dochodzą transakcje I2C,
odczyty TTL ani alokacje w pętli planera. Istniejące ograniczenia prędkości,
przyspieszeń i krańców nadal obowiązują. Maskę nóg, dla których IK lub zapisane
krańce ograniczają cel, wysyłamy w wolnych bajtach dotychczasowej ramki 0x31.

## Chodzenie na trzech łapach

Osobny, domyślnie wyłączony tryb 5 pozostawia wybraną nogę stale uniesioną,
a trzema pozostałymi wykonuje kroki. To inny tryb niż istniejący Krok 1×,
w którym wszystkie cztery nogi kolejno kroczą, pozostawiając trzy podpory.

Pozostałe nogi mają fazy przesunięte o 1/3 okresu, duty co najmniej 76%.
Zawsze planowane są dwie lub trzy podpory. Wybrana noga nie bierze udziału
w estymacji podparcia, VPSP ani wykrywaniu lądowania. Referencją ZMP jest
środek aktualnych podpór, a trajektoria korpusu korzysta z tego samego
okresowego rozwiązania LIPM. Nie stosujemy czterowierzchołkowego równania
VPSP do trzech nóg ani nie przedstawiamy tej adaptacji jako trybu MIT.

Po uzbrojeniu nadal obowiązuje dotychczasowe oczekiwanie na referencję IMU.
Potem w czasie 1,2 s najpierw przesuwany jest cel korpusu, następnie podnoszona
wybrana noga. Dopiero wtedy rusza faza kroków. Przycisk Create nadal uzbraja
i rozbraja. R1 przechodzi przez pełen cykl 0 → 1 → 2 → 3 → 4 → 5 → 0.
Numer 5 oznacza trzy łapy i pięć białych LED-ów. Skok i test Options nie
nakładają się na chód trójnożny. Wybór wyłączonej nogi jest dostępny po
rozbrojeniu w Quadruped. Przełączenia trybów nadal podlegają ograniczeniom
osi; nie są dowodem stabilnej fizycznej zmiany podparcia.

Domyślny profil: 35 mm / 40 mm / 0,65 Hz / 78%. Wysokość utrzymywanej nogi
wynosi co najmniej 40 mm. Nowe klucze NVS `triprof1` i `trimode1` nie zmieniają
rozmiaru ani zawartości wcześniejszego rekordu profili, kalibracji i geometrii.
Tryb jest eksperymentalny: podczas kroku pozostaje odcinek dwóch podpór,
więc nie zapewnia większej stabilności statycznej od chodu czterema nogami.

## Wspólne tempo i ciągłe śledzenie (2026-09-24)

W pełnym cyklu rozróżniamy geometrię ścieżki `q(s)` od tempa jej wykonania.
Dla stałej częstotliwości `f`, `q_dot=f*q_s`, `q_ddot=f²*q_ss`. Planner
sprawdza pochodne całej ścieżki IK (korpus i stopy razem), ogranicza wspólne
`f` przez limity prędkości/przyspieszenia osi i ponownie rozwiązuje LIPM dla
tego okresu. Nie opóźnia dwunastu serw dwunastoma osobnymi fazami. Iteracja
ma ograniczoną liczbę przebiegów; brak osiągalnego rozwiązania jest zgłaszany,
a nie maskowany. Próbkowanie 64 punktów i rezerwa 5% są przybliżeniem
numerycznym, nie dowodem ograniczeń między wszystkimi punktami.

Śledzenie ruchomego celu wykorzystuje prędkość referencji i korekcję błędu
pozycji (`v_ref + e/0.08 s`), z dotychczasowym ograniczeniem prędkości oraz
przyspieszenia. Stała 0.08 s jest parametrem integracji pozycyjnych serw Aury,
nie cytatem z MIT. Przy wykonalnej ścieżce blok przepuszcza kolejne punkty bez
sztucznego hamowania do zera co 20 ms. Arming/stanie/kalibracja zachowują
poprzedni generator ruchu do punktu. Krańce są nakładane przed śledzeniem i
ponownie przed wysłaniem. Nie usunięto żadnej bariery enkodera.

Zapisane ustawienie częstotliwości pozostaje żądaniem. Telemetria 0x31/v2
pokazuje wykonane tempo w bajcie 19 (0.02 Hz/LSB). Dotyczy to planera
okresowego: Trot, Run, trzy łapy oraz łuków Crawl/Climb. Prosty Crawl/Climb
nadal ma osobny quasistatyczny transfer podparcia; nie został tutaj zastąpiony.
Ograniczenie tempa nie jest pomiarem możliwości napędu pod obciążeniem.

Plan dla skrętu rozwiązuje LIPM w ramie obrotowej. Dla `J(x,z)=(-z,x)`:

```
c_world'' = R (c'' + 2 w J c' - w² c + w J v)
xi'  = (lambda I - w J) xi  - lambda p - v
eta' = (-lambda I - w J) eta + lambda p - v
c = (xi+eta)/2, lambda = sqrt(g/h)
```

`xi/eta` używają prędkości inercjalnej wyrażonej w ramie chodu. Rozwiązanie
wstecz/w przód jest analityczne dla odcinkami liniowego `p`; okresowy warunek
brzegowy dotyczy ramy chodu. Jest to wyprowadzenie z LIPM i zmiany układu
odniesienia, a nie pełny MPC MIT. Test różniczkuje wynik niezależnie i odtwarza
ZMP po dodaniu wszystkich składników obrotowej ramy.

## Przepływ i magistrala

Cała klatka 12 kątów jest teraz walidowana i publikowana pod jedną blokadą.
Zadanie TTL może zobaczyć poprzednią albo nową klatkę, nigdy mieszankę.
Odczyty okresowe uzbrojonego robota mają budżet odpowiedzi 5 ms i nie czekają
na zajętą magistralę. Dłuższe timeouty pozostały przy konfiguracji serw.
Konsola `status` pokazuje czas ostatniego/najdłuższego obliczenia, przekroczenia
20 ms i pominięte publikacje. Nie wykonano jeszcze pomiaru tych czasów na ESP
po tej zmianie; test na Macu nie zastępuje pomiaru docelowego procesora.

Podstawowa ramka 0x27 aktualizuje tylko własne pola. Nie zeruje już danych
pozy korpusu z 0x31, odometrii ani metadanych trzech łap. Czysty dekoder
`RobotTelemetry.swift` ma test z przeplatanymi ramkami i wersją v1/v2.

## Granice modelu

Jest to model referencyjny o stałej wysokości, małym momencie pędu korpusu
oraz zakładanym kontakcie bez poślizgu. Od wersji 2026-09-24 również stały
skręt uwzględnia transport ramy, przyspieszenie dośrodkowe i składnik Coriolisa.
Szybko zmieniający się twist nadal nie jest pełnym modelem przejściowym.
Nie dowodzi stabilności przy zmianie prędkości, opóźnieniu napędów, ugięciu nóg,
nasyceniu krańców, nierównościach ani błędnym offsetcie CoM.

Trot/Run z duty poniżej 50% zawierają lot, dla którego model stałej wysokości
podpartego wahadła nie obowiązuje. Planner nie wysyła wtedy nowej klatki
ruchu: zgłasza nieosiągalność i częstotliwość 0, zamiast zastępować model
niepowiązanym ruchem. Dotychczasowy zapis profilu nie jest kasowany. Prosty Crawl/Climb nadal korzysta z istniejącej
sekwencji trójpunktowego podparcia.

Pełny MIT MPC optymalizuje siły i momenty. ST3215 przyjmuje cele pozycji, więc
kod nie realizuje sterowania siłowego i nie wyznacza realnego CoM z samego IMU.
Próba fizyczna z operatorem pozostaje konieczna. Podczas przygotowania zmiany
nie wykonywano automatycznego uzbrajania ani ruchów robota.

## Testy

Test robot_body_trajectory sprawdza znane rozwiązanie sinusoidalne, niezależne
różniczkowanie prędkości, ciągłość okresu, wyprzedzenie kontaktu, odcinek podparcia,
ciągłość prędkości stopa–rama oraz nominalny cykl trotu przez IK/FK. Pozostałe
testy chodu i modelu serw pozostają niezależne. To testy matematyczne, nie pomiar
stabilności realnego robota ani pełna symulacja dynamiki.

Test robot_balance odtwarza położenie korpusu z FK podpartych nóg dla obu
przekątnych, obu znaków roll/pitch i sprawdza prześwit przez IK/FK. W tych
przypadkach poprzedni transform daje również cele poniżej podłoża, a nowy
odtwarza zadaną ścieżkę w świecie. Sprawdzane są wszystkie cztery wybory
uniesionej nogi w trybie 3 łap, liczba podpór, domknięcie ścieżki i osiągalność
nominalnego cyklu. To nie model bezwładności, tarcia ani dynamiki ST3215.

Brak poślizgu, poprawna kalibracja, realna osiągalność i terminowe wykonanie
poleceń przez serwa są nadal założeniami. Przełączenie między planowaną pozą
podparcia a estymowaną pozą przenoszenia podlega istniejącemu ograniczeniu
przyspieszeń osi; przy dużym błędzie nadążania nie gwarantuje ciągłego
prześwitu. Bez próby fizycznej nie można stwierdzić, że robot nie będzie
się przewracał. Nie testowano automatycznie ruchów ani uzbrajania.

## Źródła

- Bledt i in., [MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot](https://dspace.mit.edu/entities/publication/b77b4b14-6d3d-43c0-899e-e85ddbba2a8f), §III.C/F, równania (2)–(4), (10)–(15).
- Kajita i in., [Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point](https://people.csail.mit.edu/katiebyl/kb/DW2008/papers_of_tangential_interest/kajita03.pdf), ICRA 2003, równania (6)–(9), §3.1 o rozwiązaniu odwrotnym i okresowym.
- Di Carlo i in., [Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control](https://dspace.mit.edu/entities/publication/bc8c7e1e-5830-443f-a879-787947111fcf), model floating base i siłowe MPC.
- [MIT FootSwingTrajectory](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/Controllers/FootSwingTrajectory.cpp) i [ConvexMPCLocomotion](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp), trajektoria w świecie i transformacja do nogi.

Test `robot_timed_path` przeprowadza po 1500 próbek 50 Hz dla pięciu profili
wprost, po łuku i w obrocie: LIPM → stopy → IK → generator osi → FK. Sprawdza
limity osi, błąd śledzenia i podłoże przy nominalnej, poziomej orientacji.
Obejmuje wspólne funkcje R1 oraz liczbę LED-ów. **Nie** symuluje fizycznego
serwa, tarcia, bezwładności ani narastającego przechyłu. Wyniku nie należy
przedstawiać jako dowodu równowagi robota. Wszystkie testy bez sprzętu:
`python3 tests/run_host_tests.py`.

Dodatkowe źródło transformacji sztywnych: Murray, Li, Sastry,
[A Mathematical Introduction to Robotic Manipulation](https://www.cds.caltech.edu/~murray/mlswiki/images/mlswiki/0/02/Mls94-complete.pdf),
rozdział o wykładnikach ruchu sztywnego. Filtr wejścia polecenia odpowiada
współczynnikowi 0.1 w `ConvexMPCLocomotion::_SetupCommand`, zapisany jako
stała czasowa dla okresu 20 ms; nie filtruje przycisków pada.

## Stawianie stóp — Cheetah 3, równanie (6) (2026-09-26)

`p_step = p_hip + T_st/2·v_des + sqrt(z0/g)·(v − v_des)`. Dwa pierwsze człony to
istniejąca, zaplanowana trajektoria stopy. Trzeci (capture point, Pratt i in.)
liczy `robot_foot_placement.c` z żyroskopu: przy podpartych stopach nieplanowany
obrót korpusu to przewracanie wokół linii podparcia, więc środek masy na wysokości
z0 porusza się z v = z0·ω. Do tego dryf `z0·(przechył − referencja − korekcja)`,
bo w równaniu p_hip jest rzeczywistym położeniem biodra. Szczegóły i testy:
[kinematics-review.md](kinematics-review.md).
