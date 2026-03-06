# BLDC Motor Driver — ESP32

Sterownik silnika BLDC (bezszczotkowego prądu stałego) na bazie ESP32 z mostkami IR2103 (3 fazy).  
Obsługiwane metody sterowania: **komutacja blokowa (6-step / trapezoidalna)** oraz **komutacja sinusoidalna**.  
Planowane rozszerzenia: FOC (Field Oriented Control).

---

## Spis treści

1. [Sprzęt](#sprzęt)
2. [Schemat połączeń — pinout](#schemat-połączeń--pinout)
3. [Układy pomiarowe](#układy-pomiarowe)
4. [Architektura oprogramowania](#architektura-oprogramowania)
5. [Komutacja blokowa — zasada działania](#komutacja-blokowa--zasada-działania)
6. [Timer sprzętowy i ISR](#timer-sprzętowy-i-isr)
7. [Wyświetlacz S866 — protokół 2](#wyświetlacz-s866--protokół-2)
8. [Pomiar prędkości](#pomiar-prędkości)
9. [Obliczanie mocy](#obliczanie-mocy)
10. [Hamowanie regeneracyjne](#hamowanie-regeneracyjne)
11. [Przepustnica i poziomy wspomagania](#przepustnica-i-poziomy-wspomagania)
12. [Rampa rozpędzania](#rampa-rozpędzania)
13. [Sterowanie przez UART/Serial](#sterowanie-przez-uartserial)
14. [Kalibracja prądów](#kalibracja-prądów)
15. [Konfiguracja NVS](#konfiguracja-nvs)
16. [Znane problemy i uwagi](#znane-problemy-i-uwagi)
17. [Rozbudowa projektu](#rozbudowa-projektu)

---

## Sprzęt

| Komponent         | Model/wartość              | Uwagi |
|-------------------|----------------------------|-------|
| Mikrokontroler    | ESP32-D0WDQ6 (WROOM-32)    | 240 MHz, dual-core |
| Sterowniki mostu  | IR2103 × 3                 | Po jednym na każdą fazę A, B, C |
| Tranzystory       | Faza high: IRF877 DPAK (P-ch?) / Faza low: IRF877 ADPFB (N-ch) | Sprawdzić typ z schematu |
| Pomiar prądu      | INA180A2 × 3               | Gain = 50 V/V, shunt = 2 mΩ |
| Shunty            | 2 mΩ (low-side)            | Jeden na każdą fazę |
| Czujniki Halla    | 3 × czujnik cyfrowy        | Wbudowane w silnik |
| Level shifter     | TXB0102DCU                 | 3.3V ↔ 5V, enable GPIO17 |
| Wyświetlacz       | S866 (protokół 2)          | Serial2: GPIO4(TX)/GPIO16(RX), 9600 baud |
| Napięcie baterii  | Dzielnik 1 MΩ / 33 kΩ      | Mierzone na GPIO36 (VP) |
| Przepustnica      | Potencjometr/czujnik 0–3.3 V | GPIO2, ADC zakres 400–2600 |
| Temperatura silnika | Termistor / czujnik analogowy | GPIO15, ADC |

---

## Schemat połączeń — pinout

Wszystkie definicje pinów znajdują się w `include/pinout.h`.

### Sterowanie mostkami (IR2103)

| Pin ESP32 | GPIO | Funkcja              | IR2103 |
|-----------|------|----------------------|--------|
| GPIO32    | 32   | Faza A — HIGH side   | HIN_A  |
| GPIO33    | 33   | Faza A — LOW side    | LIN_A  |
| GPIO25    | 25   | Faza B — HIGH side   | HIN_B  |
| GPIO26    | 26   | Faza B — LOW side    | LIN_B  |
| GPIO27    | 27   | Faza C — HIGH side   | HIN_C  |
| GPIO14    | 14   | Faza C — LOW side    | LIN_C  |

> **Ważne — logika IR2103:**  
> - `HIN = HIGH` → tranzystor high-side **WŁĄCZONY**  
> - `LIN = LOW`  → tranzystor low-side  **WŁĄCZONY** (wejście odwrócone!)  
> - `LIN = HIGH` → tranzystor low-side  **WYŁĄCZONY**  
> W kodzie: PWM na HIN steruje prądem przez uzwojenie; LIN=0 (LEDC duty=0) włącza low-side.

### Wejścia analogowe (ADC)

| GPIO | Funkcja               | ADC kanał |
|------|-----------------------|-----------|
| 36   | Napięcie baterii VBAT | ADC1_CH0  |
| 39   | Prąd fazy A           | ADC1_CH3  |
| 34   | Prąd fazy B           | ADC1_CH6  |
| 35   | Prąd fazy C           | ADC1_CH7  |
| 2    | Przepustnica          | ADC2_CH2  |
| 15   | Temperatura silnika   | ADC2_CH3  |
| 12   | Temperatura FET       | ADC2_CH5  |

> **Uwaga GPIO12 (MTDI):**  
> GPIO12 jest pinem strap ESP32. Jeśli ma pull-up przy starcie, zmienia napięcie VDD_SDIO z 3.3 V na 1.8 V, co uniemożliwia programowanie flash. **Nie podłączać pull-up do GPIO12**; stosować pull-down lub pozostawić floating.

### Czujniki Halla

| GPIO | Funkcja       | Tryb |
|------|---------------|------|
| 5    | Hall A        | INPUT_PULLUP |
| 18   | Hall B        | INPUT_PULLUP |
| 19   | Hall C        | INPUT_PULLUP |

### Wejścia cyfrowe

| GPIO | Funkcja  | Aktywny stan |
|------|----------|--------------|
| 22   | PAS      | LOW (INPUT_PULLUP) |
| 23   | Hamulec  | LOW (INPUT_PULLUP) |

### Pozostałe

| GPIO | Funkcja              |
|------|----------------------|
| 1    | UART0 TX (debug)     |
| 3    | UART0 RX (debug)     |
| 4    | UART2 TX (S866 wyświetlacz) |
| 16   | UART2 RX (S866 wyświetlacz) |
| 17   | UART Enable (TXB0102DCU) |
| 21   | Wejście prędkości (czujnik ext. przy P07≤1) |
| 0    | EXT1 (boot pin!)     |

---

## Układy pomiarowe

### Napięcie baterii

Dzielnik napięciowy z rezystorów:
- R_top = 1 MΩ (między VBAT a pinem ADC)
- R_bottom = 33 kΩ (między pinem ADC a GND)

Wzór przeliczenia:

```
VBAT = V_ADC × (R_top + R_bottom) / R_bottom
     = V_ADC × (1 000 000 + 33 000) / 33 000
     ≈ V_ADC × 31.3
```

W kodzie użyto zmierzonych wartości rzeczywistych rezystorów:
- `kVbatRTop = 1 130 000` Ω
- `kVbatRBottom = 31 700` Ω

### Pomiar prądu (INA180A2 + shunt 2 mΩ)

Układ: prąd płynie przez shunt → INA180A2 wzmacnia napięcie × 50 → ESP32 ADC.

```
V_shunt = I × R_shunt = I × 0.002
V_out   = V_shunt × Gain = I × 0.002 × 50 = I × 0.1
I [A]   = V_out / 0.1 = V_ADC / 0.1
```

W kodzie:
```cpp
kCurrentScale = 1.0 / (0.002 × 50) = 10.0  [A/V]
I = V_ADC_fazy × kCurrentScale
```

Zakres pomiarowy:
- ADC ESP32 = 0–3.3 V
- Maksymalny prąd = 3.3 / 0.1 = 33 A

> **Autokalibracja offset:**  
> Gdy silnik jest wyłączony lub duty = 0, filtr EMA (α = 0.02) uśrednia wartość ADC jako offset zera (dryf wzmacniacza, offsety ADC). Przy pomiarach ten offset jest odejmowany.

### Temperatura silnika

Surowa wartość ADC (0–4095) bez przeliczenia na °C. Wymaga dopasowania krzywej do użytego czujnika (termistor NTC/PTC lub IC).  
PIn GPIO15 ma pull-up 10 kΩ na PCB — przy braku czujnika daje stałą wartość ~4095.

---

## Architektura oprogramowania

### Pliki projektu

```
platformio.ini      — konfiguracja PlatformIO (platforma, prędkość, partycje)
include/
  pinout.h          — wszystkie definicje GPIO, kanały LEDC, stałe PWM
  bldc_types.h      — typy danych, struktury, enumeracje
  display_s866.h    — definicje protokołu wyświetlacza S866, struktury ramek
src/
  main.cpp          — cała logika aplikacji (komutacja, regen, pomiar prędkości/mocy)
  display_s866.cpp  — implementacja protokołu S866 (parsowanie RX, wysyłanie TX)
```

### Globalna struktura stanu

`bldc_state_t g_bldc_state` — jedna struktura przechowująca cały stan systemu:
- tryb pracy (`mode`)
- kierunek (`direction`)
- wypełnienie PWM (`duty_cycle`)
- surowa wartość przepustnicy (`throttle_raw`)
- napięcie baterii (`battery_voltage`)
- prądy fazowe A, B, C (`phase_current[3]`)
- temperatura silnika i FET
- stan Halla (`hall_state`)
- prędkość obrotowa (`rpm`) i czas obrotu koła (`wheeltime_ms`)
- okres między przejściami Halla (`hall_period_us`)
- moc pobierana (`power_watts`) i oddawana (`regen_power_watts`)
- docelowe duty z przepustnicy (`duty_target`) i czas rampy (`ramp_time_ms`)
- flagi: `brake_active`, `pas_active`, `regen_enabled`, `regen_active`, `fault`

### Przepływ wykonania

```
setup()
  │
  ├── initGPIO()           — konfiguracja pinów, bezpieczny stan MOSFETów
  ├── initPWM()            — konfiguracja 6 kanałów LEDC (3 fazy × 2)
  ├── allMosfetsOff()      — wyłączenie wszystkich tranzystorów
  └── initCommutationTimer() — uruchomienie przerwania 20 kHz

loop() [~wolna pętla, ~kilka kHz]
  ├── readAnalogInputs()   — ADC: VBAT, prądy, przepustnica, temp
  ├── readHallSensors()    — stan 3 czujników Halla
  ├── readDigitalInputs()  — hamulec (hw + symulacja), PAS
  ├── obliczenie duty_target z przepustnicy (proporcjonalnie do assist level)
  ├── rampa rozpędzania (duty_target → duty_cycle, max Δ z ramp_time_ms)
  ├── aktualizacja zmiennych volatile dla ISR
  ├── obliczanie mocy (P = Vbat × Imax)
  ├── logika regen (hamulec + warunki → aktywacja)
  ├── obsługa wyświetlacza S866 (wheeltime, TX, service)
  ├── processSerialCommands() — obsługa komend UART
  └── auto-status co 1s (jeśli włączony)

onCommutationTimer() [ISR, 20 kHz = co 50 µs]
  ├── odczyt Halli z GPIO.in (ZAWSZE — pomiar prędkości)
  ├── pomiar czasu między przejściami Halla → hall_period_us
  ├── hamulec aktywny + regen → regenCommutateISR()
  ├── hamulec aktywny bez regen → allMosfetsOff() (coast)
  ├── sprawdzenie g_motor_enabled
  ├── dispatch trybu sterowania (g_mode_isr → BLOCK / SINUS / FOC)
  └── ustawienie 6 kanałów LEDC zgodnie z tabelą komutacji danego trybu
```

### Separacja loop/ISR

Kluczowy element projektowy: **komutacja działa w ISR**, nie w `loop()`.  
Dzięki temu wypisywanie diagnostyki przez Serial (które może trwać 10–50 ms) nie zakłóca pracy silnika.

Zmienne współdzielone między `loop()` a `onCommutationTimer()` są `volatile`:
- `g_hall_isr` — stan Halla (odczytywany też bezpośrednio w ISR z GPIO)
- `g_duty_isr` — aktualne wypełnienie PWM
- `g_motor_enabled` — czy silnik ma się kręcić
- `g_brake_isr` — czy hamulec aktywny
- `g_direction_isr` — kierunek CW/CCW
- `g_mode_isr` — aktualny tryb sterowania (BLOCK/SINUS/FOC/DISABLED)

---

## Komutacja blokowa — zasada działania

Silnik BLDC 3-fazowy jest sterowany przez 3 mostki H (każdy z jednym IR2103).  
W każdej chwili **jedna faza dostaje PWM (high-side ON)**, **jedna faza jest zwarta do GND (low-side ON)**, **jedna faza pływa (oba tranzystory OFF)**.

Sekwencja komutacji zależy od pozycji rotora odczytanej z czujników Halla.  
Hall encoder daje 3-bitowy kod (wartości 1–6, 0 i 7 są błędem).

### Tabela komutacji CW (Hall [C:B:A]):

| Hall (CBA) | Faza A    | Faza B    | Faza C    | Opis     |
|------------|-----------|-----------|-----------|----------|
| 001 = 1    | PWM HIGH  | LOW (GND) | FLOAT     | A+ → B−  |
| 011 = 3    | PWM HIGH  | FLOAT     | LOW (GND) | A+ → C−  |
| 010 = 2    | FLOAT     | PWM HIGH  | LOW (GND) | B+ → C−  |
| 110 = 6    | LOW (GND) | PWM HIGH  | FLOAT     | B+ → A−  |
| 100 = 4    | LOW (GND) | FLOAT     | PWM HIGH  | C+ → A−  |
| 101 = 5    | FLOAT     | LOW (GND) | PWM HIGH  | C+ → B−  |

Dla **CCW** tabela jest lustrzanym odbiciem (zamienione fazy HIGH i LOW).

> **Uwaga do dopasowania tabeli:**  
> Kolejność Hall→uzwojenie zależy od fizycznego montażu czujników w silniku.  
> Jeśli silnik drga zamiast się kręcić — należy cyklicznie przesunąć wpisy w tabeli  
> (np. zamiast 1→3→2→6→4→5, zmienić na 3→2→6→4→5→1).  
> Jeśli kręci się w złym kierunku — użyć komendy `r` lub zamienić dowolne dwie fazy silnika.

### PWM na high-side

- Częstotliwość: **20 kHz** (powyżej słyszalności)
- Rozdzielczość: **10 bitów** (0–1023)
- `duty_cycle = 0` → silnik wyłączony
- `duty_cycle = 1023` → 100% (pełne napięcie)

---

## Komutacja sinusoidalna — zasada działania

Tryb `DRIVE_MODE_SINUS` generuje trójfazowe sinusoidalne napięcia PWM zamiast
prostokątnych przełączeń blokowych, co daje:
- Płynniejszą pracę silnika, zwłaszcza na niskich obrotach
- Mniejsze pulsacje momentu obrotowego
- Cichszą pracę (brak szarpania 6× na obrót elektryczny)

Algorytm jest portem 1:1 ze sprawdzonego projektu STM32 (`bldc_driver_v2`).

### Tablica sinusów (LUT)

Tablica `g_sine_table[97]` zawiera 96 wpisów (= 360° elekt.) + 1 guard entry
(wrap-around bez `% 96`). Wartości: `round(sin(i × 360°/96) × 1024)`,
zakres −1024..+1024. Umieszczona w DRAM (`DRAM_ATTR`) — ESP32 nie pozwala na
byte-access do IRAM (LoadStoreError).

```
indeks:   0 →     0  (0°,   sin = 0)
indeks:  24 → +1024  (90°,  sin = +1)
indeks:  48 →     0  (180°, sin = 0)
indeks:  72 → −1024  (270°, sin = −1)
indeks:  96 →     0  (guard = [0])
```

### Mapowanie Hall → sektor

Tablica `g_hall_to_sector[8]` mapuje stan Halla (1–6) na indeks sektora 0–5.
Kolejność sektorów odpowiada sekwencji CW z komutacji blokowej: 1→3→2→6→4→5.

| Hall (CW) | Sektor | Block commutation |
|-----------|--------|-------------------|
| 1 (001)   |    0   | A→B               |
| 3 (011)   |    1   | A→C−              |
| 2 (010)   |    2   | B→C−              |
| 6 (110)   |    3   | B→A−              |
| 4 (100)   |    4   | C→A−              |
| 5 (101)   |    5   | C→B−              |

### Ciągłe śledzenie kąta (angle tracking)

W przeciwieństwie do prostego snap-to-Hall, kąt jest śledzony ciągle w Q16
fixed-point (0..96<<16). Co tick ISR (50 µs):

```
angle += speed_q16  (lub −= dla CCW)
```

Prędkość `speed_q16` = (16 × 50) << 16 / hall_period_us = 52428800 / hall_period_us,
obliczana w loop() (dzielenie 32-bit w ISR na ESP32 powoduje LoadStoreError).

Na przejściu Halla kąt jest KORYGOWANY (nie nadpisywany) — o 1/8 różnicy między
oczekiwanym a aktualnym. To daje płynne śledzenie bez skoków.

### Start sinusoidalny (bez block startup)

Tryb SINUS startuje **natychmiast** — bez fazy komutacji blokowej. Po wydaniu
komendy `S` (lub `m2`):
1. Kąt jest ustawiany (snap) na środek aktualnego sektora Halla
2. `g_sine_running = 1` — ISR od razu generuje sinusoidalne PWM
3. Silnik rusza w trybie **crawl** (otwartopętlowa minimalna prędkość)
4. Pierwsze przejście Halla daje realną prędkość i crawl wyłącza się

### Tryb crawl (rozruch z miejsca)

Gdy `g_sine_speed_q16 == 0` (brak danych o prędkości — np. start z miejsca lub po
stall fallback), ISR używa minimalnej prędkości otwartopętlowej:

| Stała | Wartość | Opis |
|---|---|---|
| `SINE_CRAWL_SPEED_Q16` | 315 | ≈ 1 obrót elektr./s (52428800/166666) |

Pole magnetyczne wolno się obraca, rotor zaczyna podążać, i pierwsze przejście
Halla daje realną prędkość — crawl wyłącza się automatycznie.

### Coast threshold (próg coastingu)

Center-aligned PWM z bazą 512 przy małej amplitudzie generuje ~50% switching na
wszystkich 6 FETach = **aktywne hamowanie elektromagnetyczne** (w odróżnieniu od
trybu BLOCK, gdzie mały duty = tiny PWM na 1 fazie, reszta float).

Aby temu zapobiec, gdy `amplitude < SINE_MIN_AMPLITUDE` wszystkie FETy są wyłączane
(coast — motor biegnie swobodnie):

| Stała | Wartość | Opis |
|---|---|---|
| `SINE_MIN_AMPLITUDE` | 15 | Poniżej tego: coast zamiast sinus |

### Stall freeze

Jeśli brak przejść Halla > 200 ms → kąt nie jest avansowany (zamrożony). Zapobiega to
ucieczce kąta przy zatrzymanym silniku, co powodowałoby oscylacje i prąd zwarcia.

**Wyjątek: tryb crawl** — gdy `speed == 0`, stall freeze jest wyłączony. Crawl obraca
pole z prędkością ~1 obr.elekt./s, co nie generuje niebezpiecznych prądów, a jest
konieczne do ruszenia silnika z miejsca. Bez tego wyjątku crawl byłby permanentnie
zablokowany (deadlock).

### Safety fallback (zabezpieczenie przed utratą Halli)

Gdy brak przejść Halla przez dłuższy czas (timeout dynamiczny: `max(400ms, 4×hall_period + 20ms)`):
1. Prędkość jest zerowana (`g_sine_speed_q16 = 0`)
2. Timestamp Halla jest resetowany (`g_sine_last_hall_ms = now`)
3. ISR przechodzi w tryb crawl — wolno obraca pole, czekając na przejście Halla

Reset timestampu (punkt 2) jest kluczowy — bez niego crawl byłby natychmiast
zablokowany przez stall freeze (ponieważ stary timestamp wskazywałby >200ms).

### Generacja PWM 3 faz (center-aligned complementary)

Mapowanie faz (dopasowane do tabeli komutacji blokowej CW):
- **Faza A**: `sin(θ)` — referencyjna (peak w sektorach 0,1)
- **Faza B**: `sin(θ + 64)` (64/96 × 360° = 240°, peak w sektorach 2,3)
- **Faza C**: `sin(θ + 32)` (32/96 × 360° = 120°, peak w sektorach 4,5)

Dzięki temu sekwencja peaków A→B→C jest zgodna z kierunkiem obrotu blokowego CW.

Dla każdej fazy:
1. Interpolacja liniowa z tabeli: `sin_val = sine_interp_q16(angle + offset)`
2. Duty: `duty = 512 + (sin_val × amplitude) >> 10`
3. HIN i LIN dostają TEN SAM duty → IR2103 z odwróconym LIN tworzy
   naturalnie komplementarne przełączanie z wbudowanym dead-time (~520 ns)

### Strojenie offsetu fazy (Hall phase offset)

Offset `g_sine_hall_phase_offset` koryguje niedopasowanie między pozycją czujników
Halla a optymalnym kątem wyprzedzenia pola magnetycznego. 1 wpis = 3.75° elektr.

**Ręczne strojenie:**
- `so` — pokaż aktualny offset
- `so+` / `so-` — zmień o ±2 wpisy (±7.5°)
- `so:N` — ustaw na wartość N (-48..+48)

**Automatyczne strojenie (komenda `sat`):**

Algorytm przelatuje zakres offsetów (-24..+24, krok 2) przy stałym niskim duty (10%).
Na każdym kroku mierzy średni prąd (400ms stabilizacja + 600ms pomiar).
Optymalny offset = minimum prądu → najlepsza sprawność (najmniej strat cieplnych
przy stałym momencie obrotowym).

Przykład użycia:
```
S            ← włącz tryb SINUS
15           ← ustaw duty 15% (żeby się kręcił)
sat          ← start auto-tune (silnik przechodzi na 10%, sweep ~25s)
             ← wynik: najlepszy offset ustawiony automatycznie
so           ← sprawdź jaki offset został wybrany
```

Opcjonalnie można podać zakres: `sat:-8:8:1` (od -8 do +8, krok 1).
Wysyłane ponownie `sat` podczas pracy anuluje strojenie i przywraca poprzedni offset.

Całkowity czas: ~25 kroków × 1s = ~25 sekund (domyślne).

### Stałe sinusoidalne

| Stała | Wartość | Opis |
|---|---|---|
| `SINE_STALL_FREEZE_MS` | 200 ms | Brak Halla → zamrożenie kąta (z wyjątkiem crawl) |
| `SINE_CRAWL_SPEED_Q16` | 315 | Minimalna prędkość crawl ≈ 1 obr.elekt./s |
| `SINE_STALL_FALLBACK_MS` | 400 ms | Minimalny timeout safety fallback |
| `SINE_START_MAX_HALL_US` | 30000 µs | Max okres Halla do wejścia w SINUS |
| `SINE_PHASE_CORR_SHIFT` | 3 | Korekcja kąta: 1/8 błędu na przejście Halla |
| `SINE_SPEED_FILTER_SHIFT` | 2 | Filtr prędkości: 1/4 new + 3/4 old |
| `SINE_SAFE_MAX_DUTY` | 45% PWM_MAX | Limit bezpieczeństwa amplitudy |
| `SINE_MIN_AMPLITUDE` | 15 | Poniżej tego: coast (zapobiega hamowaniu EM) |
| `SINE_TABLE_SIZE` | 96 | Wpisów w tablicy sinusów (= 360° elektr.) |

### Aktywacja

- Komenda `S` (natychmiastowa, bez Enter) lub `m2` + Enter
- Silnik startuje bezpośrednio w trybie sinusoidalnym (bez fazy blokowej)
- Z miejsca: tryb crawl rusza silnik, potem przejście na śledzenie Halla
- Powrót do trybu blokowego: komenda `e`
- Wyłączenie silnika: komenda `d`

---

## Timer sprzętowy i ISR

```cpp
// Timer 0, prescaler 80 → 1 MHz ticks, alarm co 50 µs = 20 kHz
commutationTimer = timerBegin(0, 80, true);
timerAttachInterrupt(commutationTimer, &onCommutationTimer, true);
timerAlarmWrite(commutationTimer, 50, true);
timerAlarmEnable(commutationTimer);
```

ISR jest oznaczona `IRAM_ATTR` — kod ISR jest ładowany do RAM (nie do flash cache), co gwarantuje deterministyczny czas wykonania nawet podczas dostępu flash przez inne operacje.

W ISR odczyt Halli odbywa się bezpośrednio z rejestru hardware:
```cpp
uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
```
To jest szybsze i bezpieczniejsze w ISR niż `digitalRead()`.

---

## Wyświetlacz S866 — protokół 2

Sterownik komunikuje się z wyświetlaczem S866 przez **Serial2** (UART2):
- **TX:** GPIO4, **RX:** GPIO16, **9600 baud**, 8N1
- Level shifter **TXB0102DCU** (3.3V ↔ 5V), włączany przez GPIO17 (UART_EN)
- Wyświetlacz jest **zawsze aktywny** (nie wymaga komendy do uruchomienia)

### Ramka RX (wyświetlacz → sterownik): 20 bajtów

| Bajt | Pole | Opis |
|------|------|------|
| 0    | Header | Zawsze 0x0C |
| 1-14 | Parametry | Dane konfiguracyjne i sterujące |
| 15-18 | Rezerwa | |
| 19   | Checksum | XOR bajtów 0-18 |

Kluczowe parametry z RX:
- **assist_level:** poziom wspomagania (raw: 0, 3, 6, 9, 12, 15 → wyświetlacz: 0-5)
- **headlight:** światło ON/OFF
- **cruise_control:** tempomat ON/OFF

### Ramka TX (sterownik → wyświetlacz): 14 bajtów

Wysyłane dane:
- **error:** kod błędu (0 = OK)
- **brake_active:** hamulec aktywny
- **current_x10:** prąd ×10 [0.1 A]
- **wheeltime_ms:** czas obrotu koła [ms] (wyświetlacz przelicza na km/h)

### Konfiguracja P01-P20

Wyświetlacz S866 ma parametry konfiguracyjne P01-P20. Niektóre są przesyłane w ramce RX, inne są lokalne na wyświetlaczu:

| Parametr | Opis | Źródło |
|----------|------|--------|
| P01-P04  | Jasność, jednostki, napięcie, auto-off | Lokalne |
| P05      | Poziomy wspomagania (3/5/9) | Ramka RX |
| P06      | Rozmiar koła (×10, np. 260=26") | Ramka RX |
| **P07**  | **Liczba impulsów Halla na obrót koła** | Ramka RX |
| P08      | Limit prędkości [km/h] | Ramka RX |
| P09      | Tryb startu (0=od zera, 1=po pedałowaniu) | Ramka RX |
| P10      | Tryb jazdy | Ramka RX |
| P11-P13  | PAS: czułość, start, magnesy | Ramka RX |
| P14      | Limit prądu [A] | Ramka RX |
| P15      | Podnapięcie (×10) [V] | Ramka RX |
| P16-P20  | Komunikacja, tempomat, gaz, power assist, protokół | Lokalne/Ramka |

> **P07 — kluczowy parametr:**
> - P07 > 1: silnik direct-drive (np. P07=90 = 6 przejść Halla × 15 par biegunów)
> - P07 ≤ 1: silnik przekładniowy — użyj czujnika SPEED (GPIO21)

### Poziomy wspomagania

Wyświetlacz wysyła `assist_level` jako wartości surowe: 0, 3, 6, 9, 12, 15.
- Podział przez 3 → numer poziomu 0-5 na wyświetlaczu
- Podział przez 15 → współczynnik max duty (0%, 20%, 40%, 60%, 80%, 100%)

---

## Pomiar prędkości

Dwa tryby pomiaru zależne od parametru P07 z wyświetlacza:

### Tryb 1: Direct-drive (P07 > 1)

Mierzone w ISR timera komutacji (20 kHz) — czas między przejściami stanów Halla:

```
wheeltime_ms = hall_period_us × P07 / 1000
```

Gdzie P07 = 6 × pole_pairs (np. 90 dla silnika 15-biegunowego). ISR mierzy `hall_period_us` **zawsze**, nawet gdy silnik nie jest napędzany (wykrywa toczenie koła).

### Tryb 2: Silnik przekładniowy (P07 ≤ 1)

Zewnętrzny czujnik prędkości na GPIO21 (INPUT_PULLUP, zbocze opadające):

```
wheeltime_ms = speed_period_us / 1000
```

Jeden magnes na kole = jeden impuls na obrót.

### Timeout

- Direct-drive: timeout 2 s od ostatniego przejścia Halla → RPM = 0
- Czujnik SPEED: timeout 3 s od ostatniego impulsu → RPM = 0

### Obliczanie RPM

```
RPM = 60000 / wheeltime_ms
```

---

## Obliczanie mocy

Moc jest obliczana w `loop()` jako:

```
P [W] = V_bat [V] × I_max [A]
```

Gdzie `I_max` = maksimum z prądów trzech faz (Ia, Ib, Ic).

### Tryby pracy

| Stan | `power_watts` | `regen_power_watts` |
|------|---------------|---------------------|
| Motoring (silnik napędzany) | P = Vbat × Imax | 0 |
| Regen (hamowanie regeneracyjne) | 0 | P = Vbat × Imax |
| Wyłączony / coast | 0 | 0 |

Moc jest wyświetlana w statusie jako `P:XX.XW`, a regen jako `RGN:XX.XW`.

---

## Hamowanie regeneracyjne

### Zasada działania

Hamowanie rekuperacyjne wykorzystuje silnik BLDC jako generator — energia kinetyczna jest zamieniana na prąd ładujący baterię. Implementacja bazuje na **low-side boost chopper**:

1. **PWM ON** (LS ON): uzwojenie silnika jest zwarte przez GND → prąd narasta napędzany przez back-EMF, energia gromadzi się w indukcyjności uzwojenia
2. **PWM OFF** (LS OFF): prąd indukcyjny nie może się zatrzymać → napięcie rośnie → prąd płynie przez body diodę high-side FET do V+ → **bateria jest ładowana**

### Tabela komutacji regen (CW)

Transformacja motoring → regen:
- Faza źródłowa (dawniej HS_PWM) → **LS_PWM** (regen)
- Faza sink (dawniej LS_ON) → **LS_ON** (bez zmian)
- Faza float → **float** (bez zmian)

| Hall [CBA] | Motoring | Regen |
|---|---|---|
| 1 (001) | A=HS_PWM, B=LS_ON | A=**LS_PWM**, B=LS_ON, C=float |
| 3 (011) | A=HS_PWM, C=LS_ON | A=**LS_PWM**, B=float, C=LS_ON |
| 2 (010) | B=HS_PWM, C=LS_ON | A=float, B=**LS_PWM**, C=LS_ON |
| 6 (110) | B=HS_PWM, A=LS_ON | A=LS_ON, B=**LS_PWM**, C=float |
| 4 (100) | C=HS_PWM, A=LS_ON | A=LS_ON, B=float, C=**LS_PWM** |
| 5 (101) | C=HS_PWM, B=LS_ON | A=float, B=LS_ON, C=**LS_PWM** |

### Sterowanie duty regen

IR2103 LIN jest odwrócony, więc:
```cpp
ledcWrite(LOW_channel, PWM_MAX_DUTY - regen_duty);
```

| duty | Efekt |
|------|-------|
| 0 | Brak hamowania (coast) |
| 50% | Umiarkowane hamowanie |
| 80% | Mocne hamowanie (REGEN_MAX_DUTY limit) |
| 100% | ⚠️ Zabronione! Brak fazy OFF = brak transferu do baterii |

### Aktywacja

Regen jest włączany komendą `R` (toggle). Gdy aktywny, hamowanie regeneracyjne następuje automatycznie przy naciśnięciu hamulca, pod warunkiem:
- **Vbat < 42V** (VBAT_REGEN_CUTOFF) — ochrona przed przepięciem
- **RPM > 50** (REGEN_MIN_RPM) — poniżej back-EMF za niskie

Jeśli warunki nie są spełnione → coast (allMosfetsOff).

### Zabezpieczenia

- **Przepięcie:** monitor Vbat — regen wyłączany powyżej 42V
- **Duty max 80%:** REGEN_MAX_DUTY — gwarantuje fazę OFF na transfer energii
- **Min RPM:** regen nieefektywny przy niskich obrotach (tylko grzeje)
- **Shoot-through:** w trybie regen WSZYSTKIE high-side FET OFF

> **Uwaga:** INA180A2 jest jednokierunkowy. W trybie regen poprawnie mierzy prąd na fazie z LS_PWM (kierunek drain→source). Na fazie sink (prąd source→drain) widzi ~0V.

---

## Przepustnica i poziomy wspomagania

### Sprzęt

- **Pin:** GPIO2 (ADC2_CH2)
- **Martwa strefa:** wartości ADC < 400 → duty = 0
- **Zakres roboczy:** 400–2600 ADC
- **Filtr EMA:** α = 0.15 (filtr szumów ADC2)

> Zakres 400–2600 został skalibrowany empirycznie dla konkretnej przepustnicy.  
> Jeśli przepustnica ma inny zakres, zmień `THROTTLE_DEAD_ZONE`, `THROTTLE_MIN_RAW`, `THROTTLE_MAX_RAW` w `main.cpp`.

### Filtr EMA przepustnicy

GPIO2 (ADC2) jest podatny na szumy od PWM silnika — bez filtra pojedyncza szpilka
poniżej 400 daje `duty=0` i powoduje stall silnika. Zastosowano filtr EMA
(Exponential Moving Average):

```
thr_ema += α × (thr_raw − thr_ema)
```

| Parametr | Wartość | Opis |
|---|---|---|
| `kThrottleFilterAlpha` | 0.15 | Szybkość reakcji filtra |
| Czas ustalania | ~10 iteracji loop | 63% wartości docelowej |

Filtr eliminuje pojedyncze szpilki szumowe bez zauważalnego opóźnienia reakcji
przepustnicy. Pierwsza próbka inicjalizuje filtr (brak opóźnienia na starcie).

### Mapowanie proporcjonalne (wspólny algorytm BLOCK / SINUS / FOC)

Przepustnica działa **proporcjonalnie** do aktualnego poziomu wspomagania z wyświetlacza S866. Zmiana poziomu zmienia zakres wyjściowy gazu, a nie obcina go — pełen zakres przepustnicy zawsze daje gładkie sterowanie od 0 do maxDuty.

Algorytm (dwie funkcje wyekstrahowane do ponownego użycia we wszystkich trybach):

1. **`getAssistMaxDuty()`** — oblicza maksymalne duty z poziomu wspomagania:

| Wyświetlacz | Assist level | maxDuty |
|---|---|---|
| Nie podłączony | — | 100% (standalone) |
| Podłączony | 0 | 0% (silnik wyłączony) |
| Podłączony | 1 (raw=3) | 20% |
| Podłączony | 2 (raw=6) | 40% |
| Podłączony | 3 (raw=9) | 60% |
| Podłączony | 4 (raw=12) | 80% |
| Podłączony | 5 (raw=15) | 100% |

2. **`mapThrottleToDuty(raw, maxDuty)`** — mapuje pełen zakres ADC przepustnicy na 0–maxDuty:

```
duty_target = map(throttle_raw, 400, 2600, 0, maxDuty)
```

Przykład: przy assist level 3 (maxDuty=60%) pełen gaz daje 60%, połowa gazu daje 30%.  
Przy starym podejściu (clamp): pełen gaz dałby 60%, ale połowa gazu — 50% (powyżej progu = obcięte), co dawało słabą rozdzielczość na niższych poziomach.

Przepustnica jest odczytywana we wszystkich aktywnych trybach sterowania (`mode != DISABLED`).  
Dzięki temu algorytm mapowania działa identycznie dla BLOCK, SINUS i FOC — bez modyfikacji.

Komendy UART (`+`, `-`, liczba%) ustawiają `duty_target` i natychmiast synchronizują `g_duty_ramped` (bez rampy — feedback ręcznego sterowania powinien być natychmiastowy). Przepustnica nadpisuje tę wartość w każdej iteracji `loop()`.

---

## Rampa rozpędzania

Silnik nigdy nie otrzyma natychmiastowego skoku duty z 0% do 100%. Zaimplementowana rampa ogranicza szybkość narastania `duty_cycle` w czasie.

### Zasada działania

```
Przepustnica → duty_target (docelowe)
                    │
              ┌─────┴─────┐
              │   RAMPA   │
              └─────┬─────┘
                    │
              duty_cycle (rzeczywiste) → ISR → silnik
```

- **Wzrost (rozpędzanie):** duty_cycle narasta płynnie, ograniczone czasem rampy
- **Spadek (zwalnianie):** **natychmiastowy** — puszczenie gazu od razu zmniejsza moc (bezpieczeństwo)
- **Hamulec aktywny:** rampa jest **zerowana** (`g_duty_ramped = 0`) — po puszczeniu hamulca silnik startuje od 0 z pełną rampą rozpędzania, bez nagłego skoku mocy

### Parametry

| Parametr | Wartość | Opis |
|---|---|---|
| `ramp_time_ms` | 1200 ms (domyślnie) | Czas przejścia 0→100% duty |
| Krok rampy | obliczany z dt | `max_step = PWM_MAX_DUTY × dt_us / (ramp_time_ms × 1000)` |
| `ramp_time_ms = 0` | — | Rampa wyłączona (natychmiastowy skok) |

Rampa jest oparta na rzeczywistym czasie (`micros()`), więc działa poprawnie niezależnie od szybkości `loop()`.

Rampa jest resetowana do 0 przy:
- komendzie `d` (disable) — silnik wyłączony
- hamulcu aktywnym — zapobiega szarpnięciu po puszczeniu hamulca

Po puszczeniu hamulca, jeśli przepustnica jest wciśnięta, silnik rozpędza się płynnie od 0 z pełną rampą.

### Status

Duty w diagnostyce wyświetla się jako `D:aktualny/docelowy%`, np.:
- `D:35/80%` — rampa w trakcie narastania (35% aktualnie, docelowo 80%)
- `D:80/80%` — rampa osiągnęła cel
- `D:0/0%` — silnik wyłączony

---

## Sterowanie przez UART/Serial

**Prędkość:** 115200 baud  
**Port:** USB-UART (GPIO1 TX, GPIO3 RX) lub zewnętrzny UART przez GPIO17 (EN)

### Tabela komend

| Komenda         | Opis |
|-----------------|------|
| `e`             | Włącz silnik — tryb BLOCK |
| `S`             | Włącz silnik — tryb SINUS |
| `m2` Enter      | Włącz silnik — tryb SINUS (alternatywna) |
| `d`             | Wyłącz silnik, duty = 0 |
| `r`             | Odwróć kierunek CW↔CCW (tylko gdy wyłączony) |
| `+`             | Zwiększ duty o 5% |
| `-`             | Zmniejsz duty o 5% |
| `0`–`100` Enter | Ustaw duty w % (np. `25` + Enter = 25%) |
| `R`             | Regeneracja ON/OFF (hamowanie rekuperacyjne) |
| `b`             | Symulacja hamulca ON/OFF (toggle) |
| `P`             | Pokaż parametry wyświetlacza P01-P20 |
| `s`             | Wyświetl status (jednorazowo) |
| `a`             | Toggle auto-status co 1 s |
| `h`             | Wyświetl pomoc |
| `t` Enter       | Pomoc trybu testowego MOSFET |
| `tAH` Enter     | Test faza A HIGH-side |
| `tAL` Enter     | Test faza A LOW-side |
| `tBH` Enter     | Test faza B HIGH-side |
| `tBL` Enter     | Test faza B LOW-side |
| `tCH` Enter     | Test faza C HIGH-side |
| `tCL` Enter     | Test faza C LOW-side |
| `tp:N` Enter    | Ustaw duty testowe na N% (1-50) |
| `t0` Enter      | Wyłącz test MOSFET (wszystkie OFF) |
| `so`           | Pokaż aktualny sine phase offset |
| `so+` / `so-`  | Offset ±2 wpisy (±7.5°) |
| `so:N`         | Ustaw offset na N (-48..+48) |
| `sat`          | Auto-tune offsetu fazy (sweep ~25s) |
| `sat:M:N`      | Auto-tune zakres M..N |
| `sat:M:N:S`    | Auto-tune zakres M..N krok S |
| `man`          | Manual duty ON/OFF (manetka ignorowana) |
| `gdbg`         | Debug SINUS/BLOCK ON/OFF (co 200ms) |
| `cfg`          | Pokaż konfigurację NVS + runtime |
| `cfg:mode:N` Enter | Tryb boot: 1=BLOCK, 2=SINUS, 3=FOC (zapis NVS) |
| `cfg:ramp:N` Enter | Czas rampy 0-10000 ms (zapis NVS + natychmiast runtime) |
| `cfg:regen:N` Enter | Regeneracja boot: 0=OFF, 1=ON (zapis NVS + natychmiast runtime) |

### Format statusu (jedna linia)

```
BLK CW D:35/80% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1870) RPM:120 WT:500 P:44.6W DISP:OK L3
```

| Pole    | Znaczenie |
|---------|-----------||
| `BLK`   | Tryb: OFF/BLK/SIN/FOC |
| `CW`    | Kierunek: CW/CCW |
| `D:35/80%` | Duty cycle: aktualny (po rampie) / docelowy (z przepustnicy) |
| `V:36.1`| Napięcie baterii [V] |
| `Ia/Ib/Ic` | Prądy fazowe A, B, C [A] |
| `H:101` | Stan Halla [C:B:A] binarnie |
| `T:312` | Surowa wartość ADC temperatury |
| `Thr:45%(1870)` | Przepustnica: % i RAW ADC |
| `RPM:120` | Obroty koła na minutę |
| `WT:500` | Czas obrotu koła [ms] |
| `P:44.6W` | Aktualna moc [W] (pobierana lub oddawana) |
| `RGN:12.5W` | Moc regeneracji [W] (gdy regen aktywny) |
| `RGN:rdy` | Regen włączony, czeka na hamulec |
| `DISP:OK L3` | Wyświetlacz S866 podłączony, poziom wspomagania 3 |
| `DISP:--` | Wyświetlacz nie podłączony |
| `BRK`   | Hamulec aktywny |
| `PAS`   | PAS aktywny |
| `FAULT` | Błąd czujników Halla |

---

## Kalibracja prądów

Autokalibracja offsetu prądu działa podczas gdy silnik jest wyłączony lub duty = 0.  
Używa filtra EMA (Exponential Moving Average):

```cpp
offset = (1 - α) × offset + α × V_ADC
```

- `α = 0.02` — powolny filtr, czas ustalania ≈ 50 iteracji loop
- Po uruchomieniu firmware należy odczekać kilka sekund bez prądu, żeby offset się ustabilizował

Aktualny offset nie jest wyświetlany w statusie. Aby go sprawdzić, można tymczasowo dodać `Serial.printf` w `readAnalogInputs()`.

---

## Konfiguracja NVS

Sterownik zapisuje konfigurację w pamięci nieulotnej ESP32 (NVS — Non-Volatile Storage).
Konfiguracja przetrwa restart i wyłączenie zasilania.

### Struktura `controller_config_t` (64 bajty)

| Pole | Typ | Domyślnie | Opis |
|---|---|---|---|
| `magic` | uint32_t | 0x424C4401 | Sentinel walidacyjny ("BLD\x01") |
| `version` | uint16_t | 1 | Wersja struktury |
| `drive_mode` | uint8_t | 1 (BLOCK) | Domyślny tryb po starcie (1=BLOCK, 2=SINUS, 3=FOC) |
| `ramp_time_ms` | uint16_t | 1200 | Czas rampy rozpędzania 0→100% [ms] |
| `regen_enabled` | uint8_t | 0 | Regeneracja ON/OFF (0/1) |
| `_reserved[54]` | uint8_t[] | 0 | Padding do stałego rozmiaru 64 bajtów |

### Walidacja

Przy starcie firmware sprawdza `magic` i `version` w NVS:
- **Zgodne** → konfiguracja załadowana
- **Niezgodne** (nowa wersja firmware, pusty NVS, uszkodzone dane) → reset do domyślnych + zapis

### Komendy

| Komenda | Opis |
|---|---|
| `cfg` | Wyświetla aktualną konfigurację NVS + parametry runtime |
| `cfg:mode:N` | Zmienia tryb boot (1=BLOCK, 2=SINUS, 3=FOC), zapisuje NVS |
| `cfg:ramp:N` | Zmienia czas rampy 0-10000 ms, **natychmiast aktualizuje runtime**, zapisuje NVS |
| `cfg:regen:N` | Zmienia regen (0/1), **natychmiast aktualizuje runtime**, zapisuje NVS |

### Przykład wyjścia `cfg`

```
========== KONFIGURACJA NVS ==========
drive_mode:    2 (SINUS)
ramp_time_ms:  1200 ms
regen_enabled: 0 (OFF)
magic:         0x424C4401 OK
version:       1
======================================
--- Runtime (bieżące) ---
mode:          SINUS
ramp_time_ms:  1200 ms
regen:         OFF
direction:     CW
sine_offset:   0 (0.0°)
```

> **Uwaga:** Komendy `cfg:ramp:N` i `cfg:regen:N` zmieniają zarówno NVS jak i bieżący
> stan runtime — efekt jest natychmiastowy. Komenda `cfg:mode:N` zmienia tylko tryb
> boot — bieżący tryb sterowania nie jest zmieniany (wymaga restartu lub komendy `S`/`e`).

---

## Diagnostyka MOSFET — tryb testowy

### Cel

Procedura diagnostyczna umożliwiająca przetestowanie **pojedynczych tranzystorów MOSFET** w mostkach IR2103. Przydatna gdy podejrzewamy uszkodzenie (zwarcie, przerwa) jednego lub więcej tranzystorów.

### Zasada działania

1. Silnik jest wyłączany (`DRIVE_MODE_DISABLED`)
2. ISR komutacji **nie nadpisuje kanałów LEDC** (flaga `g_mosfet_test_active`)
3. Wszystkie tranzystory ustawiane w stan bezpieczny (OFF)
4. Na **jednym** wybranym tranzystorze ustawiany jest PWM (domyślnie **10%**, konfigurowalne 1-50%)
5. Pozostałe 5 tranzystorów pozostaje wyłączonych

### Komendy (Serial, wymagają Enter)

| Komenda | Tranzystor | Pin ESP32 | IR2103 |
|---------|-----------|-----------|--------|
| `tAH`   | Faza A HIGH-side | GPIO32 | HIN_A |
| `tAL`   | Faza A LOW-side  | GPIO33 | LIN_A |
| `tBH`   | Faza B HIGH-side | GPIO25 | HIN_B |
| `tBL`   | Faza B LOW-side  | GPIO26 | LIN_B |
| `tCH`   | Faza C HIGH-side | GPIO27 | HIN_C |
| `tCL`   | Faza C LOW-side  | GPIO14 | LIN_C |
| `tp:N`  | Ustaw duty testowe na N% (1-50) | — | — |
| `t0`    | Wyłącz test (wszystkie OFF) | — | — |
| `t`     | Pokaż pomoc testową | — | — |

### Zmiana duty testowego

Domyślne duty testowe to **10%**. Można je zmienić komendą `tp:N` (N = 1-50%):

```
tp:5    → 5% PWM (ostrożne testowanie)
tp:20   → 20% PWM (wyraźniejszy prąd)
tp:50   → 50% PWM (maksimum, używaj ostrożnie!)
```

Zmiana duty jest natychmiastowa — jeśli test jest aktywny, PWM na bieżącym tranzystorze jest od razu aktualizowane (nie trzeba go ponownie wybierać).

### Logika PWM w trybie testowym

```
HIGH-side ON: ledcWrite(HIN_channel, test_duty)
  → HIN = HIGH przez N% okresu → high-side MOSFET przewodzi N%

LOW-side ON:  ledcWrite(LIN_channel, PWM_MAX_DUTY - test_duty)
  → LIN = LOW przez N% okresu → low-side MOSFET przewodzi N%
  (LIN jest odwrócony w IR2103!)
```

### Procedura testowa

1. Wyślij `d` — wyłącz silnik
2. Wyślij `t` + Enter — pokaż pomoc testową
3. (opcjonalnie) `tp:5` + Enter — ustaw niskie duty na początek
4. Wyślij `tAH` + Enter — włącz PWM na high-side fazy A
5. Wyślij `s` — odczytaj prąd fazy A (`Ia`)
6. (opcjonalnie) `tp:20` + Enter — zwiększ duty bez zmiany tranzystora
7. Powtórz dla każdego tranzystora (`tAL`, `tBH`, `tBL`, `tCH`, `tCL`)
8. Wyślij `t0` + Enter — zakończ test (lub `d`)

### Interpretacja wyników

| Obserwacja | Diagnoza |
|-----------|----------|
| Prąd = 0 mimo włączonego testu | Tranzystor otwarty (uszkodzony) lub brak kontaktu |
| Prąd zbyt wysoki (~max) | Tranzystor zwarty (drain-source) |
| Prąd proporcjonalny do duty | Tranzystor sprawny |
| Inne tranzystory wykazują prąd | Zwarcie między fazami lub uszkodzony IR2103 |

> **⚠️ UWAGA BEZPIECZEŃSTWA:**
> - Nigdy nie włączaj HIGH i LOW tej samej fazy jednocześnie (shoot-through = zwarcie V+ do GND!)
> - Procedura testowa zabezpiecza przed tym automatycznie — zawsze włączany jest **tylko jeden** tranzystor
> - Używaj niskiego napięcia zasilania do testów jeśli to możliwe
> - Duty ograniczone do 50% max — zabezpieczenie przed przypadkowym podaniem pełnej mocy
> - Monitoruj temperaturę FET podczas testów (komenda `s` pokazuje odczyt czujnika)

---

## Znane problemy i uwagi

### Problem: Silnik drga/stuka zamiast się kręcić
- Tabela komutacji Hall→fazy nie pasuje do silnika
- Rozwiązanie: przesuń cyklicznie wpisy `case` w `blockCommutate()` i `onCommutationTimer()`

### Problem: Brak uploadu na PCB
- GPIO12 z pull-up zmienia napięcie VDD_SDIO → flash nie odpowiada
- Rozwiązanie: dać pull-down na GPIO12, nie pull-up
- Alternatywa: zmniejszyć `upload_speed` do 115200 w `platformio.ini`

### Problem: Silnik się kręci mimo przepustnicy w pozycji 0
- Martwa strefa `THROTTLE_DEAD_ZONE` za niska
- Aktualnie: 400. Sprawdź wartość RAW w statusie (`Thr:xx%(RAW)`) i dostosuj

### Problem: Prądy pokazują 0 mimo obciążenia
- Offset nie skalibrowany (zbyt mało czasu z wyłączonym silnikiem)
- Błąd w kalibracji dzielnika `kVbatRTop/kVbatRBottom` dla VBAT
- Sprawdź napięcie na wyjściu INA180A2 multimetrem

### Problem: GPIO15 (PIN_MOTOR_TEMP) — wartość ADC = 4095
- GPIO15 ma pull-up 10 kΩ na PCB
- Przy braku podłączonego czujnika daje max wartość ADC
- **GPIO15 z pull-up = bezpieczny strap pin dla ESP32** (nie zmienia VDD_SDIO)

---

## Rozbudowa projektu

### 1. ~~Sterowanie sinusoidalne (DRIVE_MODE_SINUS)~~ — **ZAIMPLEMENTOWANE** ✓
- Komenda `S` (natychmiastowa) lub `m2` + Enter — przełącza na tryb sinusoidalny
- Szczegóły: patrz sekcja **Komutacja sinusoidalna** poniżej

### 2. FOC (Field Oriented Control) (DRIVE_MODE_FOC)
- Transformacja Clarke/Park
- Regulatory PI dla prądu Id/Iq
- Wymaga szybkiego ADC (synchronizacja z PWM) i enkoderu/resolvera
- **Infrastruktura gotowa:** `g_mode_isr` + `g_duty_isr` mogą być reinterpretowane jako amplituda/kąt
- Nowa komenda Serial (np. `e3`) → `mode = DRIVE_MODE_FOC`

### 3. Zabezpieczenia (rozszerzone)
- Overcurrent: porównać `phase_current[i]` z progiem → `allMosfetsOff()` + `fault = true`
- Overtemperature: po skalibrowanym czujniku
- Undervoltage: sprawdzać `battery_voltage` < próg
- TVS dioda na szynie DC jako hardwarowy clamp (regen overvoltage)

### 4. Regulacja siły regen
- Mapowanie siły hamowania z prędkości (szybciej → więcej hamowania)
- Pętla prądowa: INA180A2 → limit prądu regen z P14
- Konfiguracja duty regen z wyświetlacza lub komend Serial

### 5. ~~Konfiguracja przez UART~~ — **CZĘŚCIOWO ZAIMPLEMENTOWANE** ✓
- Komenda `cfg` — wyświetlanie konfiguracji NVS + runtime
- Komendy `cfg:mode:N`, `cfg:ramp:N`, `cfg:regen:N` — zmiana i zapis do NVS
- **Do rozbudowy:** zmiana `THROTTLE_DEAD_ZONE`, `THROTTLE_MAX_RAW` bez rekompilacji

### 6. CAN bus / RS485
- Gotowy pin UART_EN (GPIO17) sugeruje planowany RS485 lub CAN
- Zaimplementować protokół ramki np. `$CMD,VALUE\n`

---

## Konfiguracja PlatformIO

Plik `platformio.ini`:
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600         ; zmniejszyć do 115200 przy problemach z PCB
build_flags =
    -D CORE_DEBUG_LEVEL=3
    -D CONFIG_ARDUHAL_LOG_COLORS=1
board_build.partitions = default.csv
```

---

*Dokumentacja wygenerowana: 2026-03-06*  
*Wersja firmware: 0.2.0*
