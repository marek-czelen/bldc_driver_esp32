# BLDC Motor Driver — ESP32

**Wersja firmware: 2.0.0** | CONFIG_VERSION: 16 | PlatformIO espressif32

Sterownik silnika BLDC (bezszczotkowego prądu stałego) na bazie ESP32 z mostkami IR2103 (3 fazy).  
Obsługiwane metody sterowania: **komutacja blokowa (6-step / trapezoidalna)**, **komutacja sinusoidalna** oraz **FOC (Field Oriented Control)**.  
Wbudowany system **PAS (Pedal Assist Sensor)** z detekcją kierunku, soft-startem i regulacją prędkości docelowej.  
Konfiguracja przez **Serial 115200 baud** oraz **WiFi AP** (responsywny interfejs webowy).

### Zmiany w v2.0.0 (względem v1.0.0)

| Zmiana | v1.0 | v2.0 |
|--------|------|------|
| PWM peryferium | LEDC (6 kanałów) | **MCPWM** (3 operatory × 2 generatory) |
| Tryb PWM | Edge-aligned | **Center-aligned (UP_DOWN counter)** |
| Rozdzielczość PWM | 10-bit (0–1023) | **Period-based (0–500)** |
| Timer ISR | hw_timer_t (osobny timer) | **MCPWM TEZ** (przerwanie w dolinie PWM) |
| ISR safety | IRAM_ATTR | **IRAM_ATTR + ESP_INTR_FLAG_IRAM + direct LL register writes** |
| Komutacja w ISR | `ledcWrite()` (driver API) | **Bezpośredni zapis rejestrów** (~5 ns, bez spinlocków) |
| Shadow compare update | — | **TEZ-only** (symetryczne impulsy center-aligned) |
| Prescaler | Automatyczny (LEDC) | **Jawny** (group=1, timer=8 → 20 MHz, period=500) |
| Dead time | Software (IR2103 wewnętrzny) | **MCPWM bypass** (IR2103 ~520 ns wewnętrzny) |
| CONFIG_VERSION | 11 | **16** (rozszerzona struktura config; aktualny stan kodu) |

---

## Spis treści

1. [Sprzęt](#sprzęt)
2. [Schemat połączeń — pinout](#schemat-połączeń--pinout)
3. [Układy pomiarowe](#układy-pomiarowe)
4. [Architektura oprogramowania](#architektura-oprogramowania)
5. [Komutacja blokowa — zasada działania](#komutacja-blokowa--zasada-działania)
6. [Komutacja sinusoidalna — zasada działania](#komutacja-sinusoidalna--zasada-działania)
7. [Timer sprzętowy i ISR](#timer-sprzętowy-i-isr)
8. [Wyświetlacz S866 — protokół 2](#wyświetlacz-s866--protokół-2)
9. [Pomiar prędkości](#pomiar-prędkości)
10. [Obliczanie mocy](#obliczanie-mocy)
11. [Hamowanie regeneracyjne](#hamowanie-regeneracyjne)
12. [Przepustnica i poziomy wspomagania](#przepustnica-i-poziomy-wspomagania)
13. [Rampa rozpędzania](#rampa-rozpędzania)
14. [PAS — Pedal Assist Sensor](#pas--pedal-assist-sensor)
15. [Sterowanie przez UART/Serial](#sterowanie-przez-uartserial)
16. [Kalibracja prądów](#kalibracja-prądów)
17. [Konfiguracja NVS](#konfiguracja-nvs)
18. [Diagnostyka MOSFET — tryb testowy](#diagnostyka-mosfet--tryb-testowy)
19. [FOC — Field Oriented Control](#foc--field-oriented-control)
20. [WiFi — interfejs konfiguracyjny WWW](#wifi--interfejs-konfiguracyjny-www)
21. [Filtracja wejść — przegląd](#filtracja-wejść--przegląd)
22. [Rozbudowa projektu](#rozbudowa-projektu)
23. [Troubleshooting](#troubleshooting)
24. [Konfiguracja PlatformIO](#konfiguracja-platformio)

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
| Level shifter     | TXB0102DCU                 | 3.3V ↔ 5V, enable GPIO5 |
| Wyświetlacz       | S866 (protokół 2)          | Serial2: GPIO17(TX)/GPIO16(RX), 9600 baud |
| Napięcie baterii  | Dzielnik 1 MΩ / 33 kΩ      | Mierzone na GPIO36 (VP) |
| Przepustnica      | Potencjometr/czujnik 0–3.3 V | GPIO33, ADC zakres 400–2600 |

---

## Schemat połączeń — pinout

Wszystkie definicje pinów znajdują się w `include/pinout.h`.

### Sterowanie mostkami (IR2103)

| Pin ESP32 | GPIO | Funkcja              | IR2103 |
|-----------|------|----------------------|--------|
| GPIO12    | 12   | Faza A — HIGH side   | HIN_A  |
| GPIO13    | 13   | Faza A — LOW side    | LIN_A  |
| GPIO25    | 25   | Faza B — HIGH side   | HIN_B  |
| GPIO26    | 26   | Faza B — LOW side    | LIN_B  |
| GPIO27    | 27   | Faza C — HIGH side   | HIN_C  |
| GPIO14    | 14   | Faza C — LOW side    | LIN_C  |

> **Ważne — logika IR2103:**  
> - `HIN = HIGH` → tranzystor high-side **WŁĄCZONY**  
> - `LIN = LOW`  → tranzystor low-side  **WŁĄCZONY** (wejście odwrócone!)  
> - `LIN = HIGH` → tranzystor low-side  **WYŁĄCZONY**  
> W kodzie: MCPWM gen_A steruje HIN, gen_B steruje LIN. W trybie BLOCK: gen_A=PWM, gen_B=forced.  
> W trybie SINUS/FOC: oba generatory dostają ten sam duty → IR2103 tworzy komplementarne przełączanie z dead-time ~520 ns.

### Wejścia analogowe (ADC)

| GPIO | Funkcja               | ADC kanał |
|------|-----------------------|-----------|
| 36   | Napięcie baterii VBAT | ADC1_CH0  |
| 39   | Prąd fazy A           | ADC1_CH3  |
| 34   | Prąd fazy B           | ADC1_CH6  |
| 35   | Prąd fazy C           | ADC1_CH7  |
| 33   | Przepustnica          | ADC1_CH5  |
| 32   | Temperatura FET       | ADC1_CH4  |

> **Uwaga GPIO12 (MTDI):**  
> GPIO12 jest pinem strap ESP32 (teraz używany jako PIN_PWM_A_HIGH). Jeśli ma pull-up przy starcie, zmienia napięcie VDD_SDIO z 3.3 V na 1.8 V, co uniemożliwia programowanie flash. **Nie podłączać pull-up do GPIO12**; stosować pull-down lub pozostawić floating.

### Czujniki Halla

| GPIO | Funkcja       | Tryb |
|------|---------------|------|
| 4    | Hall A        | INPUT_PULLUP |
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
| 16   | UART RX (S866 wyświetlacz) |
| 17   | UART TX (S866 wyświetlacz) |
| 5    | UART Enable (TXB0102DCU) |
| 21   | Wejście prędkości (czujnik ext. przy P07≤1) |
| 0    | EXT1 (boot pin!)     |
| 2    | EXT2                 |
| 15   | EXT3                 |

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

---

## Architektura oprogramowania

### Pliki projektu

```
platformio.ini      — konfiguracja PlatformIO (platforma, prędkość, partycje)
include/
  pinout.h          — wszystkie definicje GPIO, stałe MCPWM (frequency, period, duty)
  bldc_types.h      — typy danych, struktury, enumeracje
  bldc_config.h     — konfiguracja NVS (controller_config_t, CONFIG_VERSION=16)
  display_s866.h    — definicje protokołu wyświetlacza S866, struktury ramek
src/
  main.cpp          — cała logika aplikacji (komutacja MCPWM, regen, pomiar prędkości/mocy)
  bldc_config.cpp   — implementacja NVS (load/save/defaults)
  display_s866.cpp  — implementacja protokołu S866 (parsowanie RX, wysyłanie TX)
```

### Globalna struktura stanu

`bldc_state_t g_bldc_state` — jedna struktura przechowująca cały stan systemu:
- tryb pracy (`mode`)
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
  ├── initPWM()            — konfiguracja MCPWM (3 operatory × 2 generatory, prescaler, period)
  ├── allMosfetsOff()      — wyłączenie wszystkich tranzystorów (gen_force → FORCE_OFF)
  └── initCommutationTimer() — rejestracja ISR na przerwanie TEZ Timer 0 (20 kHz)

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
  └── ustawienie 3 operatorów MCPWM (gen_force + compare) zgodnie z trybem
```

### Separacja loop/ISR

Kluczowy element projektowy: **komutacja działa w ISR**, nie w `loop()`.  
Dzięki temu wypisywanie diagnostyki przez Serial (które może trwać 10–50 ms) nie zakłóca pracy silnika.

Zmienne współdzielone między `loop()` a `onCommutationTimer()` są `volatile`:
- `g_hall_isr` — stan Halla (odczytywany też bezpośrednio w ISR z GPIO)
- `g_duty_isr` — aktualne wypełnienie PWM
- `g_motor_enabled` — czy silnik ma się kręcić
- `g_brake_isr` — czy hamulec aktywny
- `g_mode_isr` — aktualny tryb sterowania (BLOCK/SINUS/FOC/DISABLED)

---

## Komutacja blokowa — zasada działania

Silnik BLDC 3-fazowy jest sterowany przez 3 mostki H (każdy z jednym IR2103).  
W każdej chwili **jedna faza dostaje PWM (high-side ON)**, **jedna faza jest zwarta do GND (low-side ON)**, **jedna faza pływa (oba tranzystory OFF)**.

Sekwencja komutacji zależy od pozycji rotora odczytanej z czujników Halla.  
Hall encoder daje 3-bitowy kod (wartości 1–6, 0 i 7 są błędem).

### Tabela komutacji (Hall [C:B:A]):

| Hall (CBA) | Faza A    | Faza B    | Faza C    | Opis     |
|------------|-----------|-----------|-----------|----------|
| 001 = 1    | PWM HIGH  | LOW (GND) | FLOAT     | A+ → B−  |
| 011 = 3    | PWM HIGH  | FLOAT     | LOW (GND) | A+ → C−  |
| 010 = 2    | FLOAT     | PWM HIGH  | LOW (GND) | B+ → C−  |
| 110 = 6    | LOW (GND) | PWM HIGH  | FLOAT     | B+ → A−  |
| 100 = 4    | LOW (GND) | FLOAT     | PWM HIGH  | C+ → A−  |
| 101 = 5    | FLOAT     | LOW (GND) | PWM HIGH  | C+ → B−  |


> **Uwaga do dopasowania tabeli:**  
> Kolejność Hall→uzwojenie zależy od fizycznego montażu czujników w silniku.  
> Jeśli silnik drga zamiast się kręcić — należy cyklicznie przesunąć wpisy w tabeli  
> (np. zamiast 1→3→2→6→4→5, zmienić na 3→2→6→4→5→1).  
> Jeśli kręci się w złym kierunku — użyć komendy `r` lub zamienić dowolne dwie fazy silnika.

### PWM — MCPWM center-aligned

- Peryferium: **MCPWM** (Motor Control PWM), MCPWM_UNIT_0
- Tryb licznika: **UP_DOWN** (center-aligned) — symetryczne impulsy
- Częstotliwość: **20 kHz** (powyżej słyszalności)
- Timer: 160 MHz / prescaler_group(1) / prescaler_timer(8) = **20 MHz**, period = **500**
- Rozdzielczość: **0–500** (compare value = peak period)
- `duty = 0` → silnik wyłączony
- `duty = 500` → 100% (pełne napięcie)
- Shadow compare update: **TEZ-only** (symetryczne impulsy, brak asymetrii TEP)
- Dead time: **bypass** (IR2103 wewnętrzny ~520 ns)

**Sterowanie w ISR** odbywa się przez bezpośredni zapis rejestrów MCPWM (LL level),
bez wywołań driver API (które używają spinlocków i nie są ISR-safe):

| Tryb | gen_A (HIN) | gen_B (LIN) | gen_force.val |
|------|-------------|-------------|---------------|
| PWM (BLOCK HS) | PWM action | forced HIGH (LS OFF) | `0x0200` |
| GND (BLOCK LS ON) | forced LOW | forced LOW (LS ON) | `0x0140` |
| OFF (float) | forced LOW | forced HIGH (LS OFF) | `0x0240` |
| Complementary (SIN/FOC) | PWM action | PWM action | `0x0000` |
| Regen (LS PWM) | forced LOW | PWM action (inverted) | `0x0040` |

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
Kolejność sektorów odpowiada sekwencji komutacji blokowej: 1→3→2→6→4→5.

| Hall | Sektor | Block commutation |
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
angle += speed_q16
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

Center-aligned PWM z bazą PWM_MAX_DUTY/2 (=250) przy małej amplitudzie generuje ~50% switching na
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

Mapowanie faz (dopasowane do tabeli komutacji blokowej):
- **Faza A**: `sin(θ)` — referencyjna (peak w sektorach 0,1)
- **Faza B**: `sin(θ + 64)` (64/96 × 360° = 240°, peak w sektorach 2,3)
- **Faza C**: `sin(θ + 32)` (32/96 × 360° = 120°, peak w sektorach 4,5)

Dzięki temu sekwencja peaków A→B→C jest zgodna z kierunkiem obrotu blokowego.

Dla każdej fazy:
1. Interpolacja liniowa z tabeli: `sin_val = sine_interp_q16(angle + offset)`
2. Modulacja: `m = (sin_val × amplitude) >> 10` → zakres -amp..+amp
3. **SVPWM centering:** `duty = offset + m`, gdzie `offset = PWM_MAX_DUTY/2 - (max+min)/2`
4. MCPWM gen_A i gen_B dostają TEN SAM duty (tryb complementary, `gen_force = 0x0000`)
5. IR2103 z odwróconym LIN tworzy komplementarne przełączanie z dead-time ~520 ns

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

Opcja `sat` działa również w trybie **FOC** — offset fazy jest tak samo ważny dla poprawnego mapowania Hall→kąt w transformacie Parka.

Wynik i bieżący offset są automatycznie **zapisywane do NVS** po zakończeniu auto-tune. Komendy `so+`, `so-`, `so:N` również zapisują do NVS.

Całkowity czas: ~25 kroków × 1s = ~25 sekund (domyślne).

### Stałe sinusoidalne

| Stała | Wartość | Opis |
|---|---|---|
| `SINE_STALL_FREEZE_MS` | 200 ms | Brak Halla → zamrożenie kąta (z wyjątkiem crawl) |
| `SINE_CRAWL_SPEED_Q16` | 315 | Minimalna prędkość crawl ≈ 1 obr.elekt./s |
| `SINE_STALL_FALLBACK_MS` | 400 ms | Minimalny timeout safety fallback |
| `SINE_START_MAX_HALL_US` | 30000 µs | Max okres Halla do wejścia w SINUS |
| `SINE_PHASE_CORR_SHIFT` | 2 | Korekcja kąta: 1/4 błędu na przejście Halla |
| `SINE_SPEED_FILTER_SHIFT` | 1 | Filtr prędkości: 1/2 new + 1/2 old |
| `SINE_SAFE_MAX_DUTY` | 75% PWM_MAX (375) | Limit bezpieczeństwa amplitudy (SVPWM) |
| `SINE_MIN_AMPLITUDE` | 8 | Poniżej tego: coast (zapobiega hamowaniu EM) |
| `SINE_TABLE_SIZE` | 96 | Wpisów w tablicy sinusów (= 360° elektr.) |

### Aktywacja

- Komenda `S` (natychmiastowa, bez Enter) lub `m2` + Enter
- Silnik startuje bezpośrednio w trybie sinusoidalnym (bez fazy blokowej)
- Z miejsca: tryb crawl rusza silnik, potem przejście na śledzenie Halla
- Powrót do trybu blokowego: komenda `e`
- Wyłączenie silnika: komenda `d`

---

## Timer sprzętowy i ISR

Komutacja silnika odbywa się w przerwaniu **TEZ (Timer Equals Zero)** peryferium MCPWM.
TEZ = dolina center-aligned PWM — optymalny moment na pomiar prądu (wszystkie low-side ON).

```cpp
// Rejestracja ISR MCPWM z flagą IRAM (kod + dane w RAM, nie flash)
mcpwm_isr_register(MCPWM_UNIT_0, onCommutationTimer, NULL,
                    ESP_INTR_FLAG_IRAM, &g_mcpwm_isr_handle);
// Włączenie przerwania TEZ dla Timer 0
mcpwm_ll_intr_enable_timer_tez(&MCPWM0, 0, true);
```

**Częstotliwość ISR:** 20 kHz (identyczna z PWM — jedno przerwanie na cykl PWM).  
**Wyzwalanie:** TEZ (Timer Equals Zero) = dolina center-aligned PWM.

ISR jest oznaczona `IRAM_ATTR` i zarejestrowana z `ESP_INTR_FLAG_IRAM` —
cały kod ISR i wywoływane funkcje muszą być w RAM (nie flash cache).
Wszystkie helpery komutacji (`phaseX_PWM/Low/Off`, `mcpwm_phase_*`, `allMosfetsOff`,
`hallToSector`, `sine_interp_q16`) mają atrybuty `static inline IRAM_ATTR`.

W ISR odczyt Halli odbywa się bezpośrednio z rejestru hardware:
```cpp
uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
```
To jest szybsze i bezpieczniejsze w ISR niż `digitalRead()`.

Zapis PWM w ISR używa bezpośrednich zapisów rejestrów MCPWM (LL level):
```cpp
// Zapis compare value (~5 ns, bez spinlocków)
mcpwm_ll_operator_set_compare_value(&MCPWM0, op, cmp, duty);
// Zapis trybu generatora (force/PWM)
MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_PWM;  // np. high-side PWM
```

**Dlaczego bezpośrednie rejestry zamiast driver API:**
- `mcpwm_set_duty_type()` i inne funkcje driver używają `portENTER_CRITICAL()` (spinlock)
- Spinlocki w ISR level 3 powodują crash (Guru Meditation: Cache disabled but cached memory region accessed)
- Bezpośrednie zapisy LL są ~100× szybsze i ISR-safe

---

## Wyświetlacz S866 — protokół 2

Sterownik komunikuje się z wyświetlaczem S866 przez **Serial2** (UART2):
- **TX:** GPIO17, **RX:** GPIO16, **9600 baud**, 8N1
- Level shifter **TXB0102DCU** (3.3V ↔ 5V), włączany przez GPIO5 (UART_EN)
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

### Tabela komutacji regen

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

IR2103 LIN jest odwrócony — MCPWM gen_B w trybie regen używa odwróconych akcji
(GEN_B_ACTION_MODE1: UP=HIGH, DOWN=LOW), co daje odwrócone duty na LIN:
```cpp
MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE1;  // gen_B inverted
mcpwm_set_compare_fast(op, 1, regen_duty);                     // compare_B = regen_duty
MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_REGEN;        // gen_A=forceLOW, gen_B=PWM
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

- **Pin:** GPIO33 (ADC1_CH5)
- **Martwa strefa:** wartości ADC < 400 → duty = 0
- **Zakres roboczy:** 400–2600 ADC

> Zakres 400–2600 został skalibrowany empirycznie dla konkretnej przepustnicy.  
> Jeśli przepustnica ma inny zakres, zmień `THROTTLE_DEAD_ZONE`, `THROTTLE_MIN_RAW`, `THROTTLE_MAX_RAW` w `main.cpp`.

### Filtr szumów ADC przepustnicy (burst + outlier rejection + EMA)

GPIO33 (ADC1_CH5) jest podatny na szpilki EMI od PWM silnika. Pojedyncza szpilka
poniżej progu dead-zone dawałaby `duty=0` i stall silnika. Zastosowano
3-stopniowy pipeline filtracji:

```
  analogRead() × N
       │
  ┌────┴────┐
  │ BURST   │  N próbek w szybkim burście (domyślnie 8)
  └────┬────┘
       │
  ┌────┴─────────┐
  │ SORTOWANIE   │  Insertion sort → mediana
  └────┬─────────┘
       │
  ┌────┴──────────────┐
  │ OUTLIER REJECTION │  Odrzucenie próbek > threshold od mediany
  └────┬──────────────┘
       │
  ┌────┴────┐
  │ ŚREDNIA │  Z pozostałych (valid) próbek
  └────┬────┘
       │
  ┌────┴────┐
  │  EMA    │  α = 0.15, wygładzenie czasowe
  └────┬────┘
       │
    thr_ema → mapowanie na duty
```

**Etap 1 — Burst sampling:**  
Odczyt N próbek ADC w szybkim burście (pętla `analogRead()` bez opóźnień).  
Parametr `thr_samples` (NVS, domyślnie 8, zakres 2–16).

**Etap 2 — Insertion sort + mediana:**  
Próbki sortowane (O(N²), ale N≤16 — szybkie). Mediana = element środkowy tablicy.

**Etap 3 — Outlier rejection (odrzucanie szpilek):**  
Każda próbka oddalona od mediany o więcej niż `thr_outlier_thresh` (NVS, domyślnie 150,
zakres 10–2000 jednostek ADC) jest odrzucana. Pozostają tylko próbki „w okolicy" mediany.

**Etap 4 — Średnia z valid próbek:**  
Średnia arytmetyczna próbek, które przeżyły filtr odchyleń. Jeśli żadna nie przeżyła
(skrajny przypadek), zwraca medianę.

**Etap 5 — EMA (Exponential Moving Average):**  
```
thr_ema += α × (burst_avg − thr_ema)
```
α = 0.15. Wygładza wynik w czasie — eliminuje jednorazowe odchyłki między burstami.

| Parametr | Wartość | NVS komenda | Opis |
|---|---|---|---|
| `thr_samples` | 8 | `cfg:thrsamp:N` | Liczba próbek w burście (2–16) |
| `thr_outlier_thresh` | 150 | `cfg:thrdelta:N` | Max odchylenie od mediany (10–2000) |
| `kThrottleFilterAlpha` | 0.15 | — | Stała EMA (w kodzie) |

> **Uwaga:** Przepustnica (GPIO33) i temperatura FET (GPIO32) korzystają teraz z **ADC1** — nie ma konfliktu z WiFi.  
> Odczyty ADC przepustnicy i temperatury FET działają poprawnie niezależnie od stanu WiFi.

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

Silnik nigdy nie otrzyma natychmiastowego skoku duty. Zaimplementowana rampa ogranicza szybkość zmiany `duty_cycle` w czasie — zarówno w górę jak i w dół.

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
- **Spadek (zwalnianie):** duty_cycle maleje płynnie, tą samą rampą (symetrycznie)
- **Hamulec aktywny:** rampa jest **zerowana** (`g_duty_ramped = 0`) — po puszczeniu hamulca silnik startuje od 0 z pełną rampą rozpędzania, bez nagłego skoku mocy
- **Dodatkowy limit:** `duty_max_step_pct` (EEPROM, domyślnie 5%) — max % zmiany duty na jedno wywołanie loop()

### Parametry

| Parametr | Wartość | Opis |
|---|---|---|
| `ramp_time_ms` | 1200 ms (domyślnie) | Czas przejścia 0→100% duty (i 100%→0%) |
| `duty_max_step_pct` | 5% (EEPROM) | Max zmiana duty na wywołanie [% PWM_MAX] (0=brak limitu) |
| Krok rampy | obliczany z dt | `max_step = min(ramp_step, pct_step)` |
| `ramp_time_ms = 0` | — | Rampa wyłączona (duty_max_step_pct dalej aktywny) |

Rampa jest oparta na rzeczywistym czasie (`micros()`), więc działa poprawnie niezależnie od szybkości `loop()`.

Rampa jest resetowana do 0 przy:
- komendzie `d` (disable) — silnik wyłączony
- hamulcu aktywnym — zapobiega szarpnięciu po puszczeniu hamulca

Po puszczeniu hamulca, jeśli przepustnica jest wciśnięta, silnik rozpędza się płynnie od 0 z pełną rampą.

### Status

Duty w diagnostyce wyświetla się jako `D:aktualny/docelowy%`, np.:
- `D:35/80%` — rampa w trakcie narastania (35% aktualnie, docelowo 80%)
- `D:80/30%` — rampa w trakcie spadku (80% aktualnie, docelowo 30%)
- `D:80/80%` — rampa osiągnęła cel
- `D:0/0%` — silnik wyłączony

---

## PAS — Pedal Assist Sensor

System wspomagania pedałowania (PAS) wykrywa kierunek i aktywność pedałowania,
a następnie steruje mocą silnika dążąc do prędkości docelowej zależnej od poziomu
wspomagania ustawionego na wyświetlaczu.

### Sprzęt

- **Pin:** GPIO22 (INPUT_PULLUP)
- **Czujnik:** Hall z asymetrycznym dyskiem magnesów (12 magnesów domyślnie, P13)
- **Sygnał:** stan HIGH i LOW o różnym czasie trwania → detekcja kierunku

### Filtr cyfrowy — timer próbkujący (zamiast przerwania)

Wcześniejsza metoda: przerwanie GPIO na obu zboczach (`CHANGE`). Problem:
szpilki EMI z komutacji silnika (µs do kilku ms) odpalały ISR i fałszowały
pomiary HIGH/LOW time oraz odświeżały timestampy — zwłaszcza przy dużych
prędkościach silnika, gdzie szum EMI jest najintensywniejszy.

**Nowa metoda:** timer `esp_timer` próbkuje pin PAS co **500 µs** (2 kHz)
i stosuje filtr cyfrowy z potwierdzeniem:

```
  Pin PAS (surowy):  ____████_██__████████████████████████___________████████████
                          ↑ szpilki EMI           ↑ realna zmiana stanu
  Próbki (co 500µs): LLLLHHLHLLLHHHHHHHHHHHHHHHHHHHHHHHHLLLLLLLLLLLLLHHHHHHHHHH
  Stan filtrowany:   LLLLLLLLLLLLLLLLLLLLLL→H (po N zgodnych)       LLL→H
```

**Algorytm filtra:**
1. Timer co 500 µs czyta pin PAS (`GPIO.in >> PIN_PAS`)
2. Jeśli próbka == aktualny stan filtrowany → reset licznika (stan potwierdzony)
3. Jeśli próbka != stan filtrowany → inkrementuj licznik
4. Gdy licznik osiągnie **N** (`g_pas_filter_depth`) → akceptuj zmianę stanu
5. Zmiana stanu wywołuje `processPasFilteredEdge()` — logikę detekcji kierunku

**Głębokość filtra:**
```
N = pas_debounce_us / PAS_SAMPLE_INTERVAL_US
```
- Domyślnie: 3000 / 500 = **6 próbek = 3 ms** potwierdzenia
- Szpilka EMI (< 1 ms) nie utrzyma się przez 6 × 500 µs → odfiltrowana
- Burst szpilek (np. co 2–4 ms z komutacji): wystarczy **jedna** próbka zgodna
  ze starym stanem aby zresetować licznik → burst nie przechodzi
- Realny impuls PAS przy 150 RPM / 12 magnesów: min half-period ≈ 17 ms →
  filtr 3 ms nie wpływa na realne pomiary

**Dlaczego lepsze od debounce na przerwaniu:**

| Cecha | Przerwanie + debounce | Timer próbkujący |
|---|---|---|
| Szpilka EMI 0.5 ms | Odfiltrowana (ok) | Odfiltrowana (ok) |
| Burst szpilek co 4 ms | Przechodzą (> debounce 3 ms) | Odfiltrowane (reset licznika) |
| Pomiary HIGH/LOW time | Fałszowane przez szpilki | Czyste (tylko potwierdzone krawędzie) |
| Detekcja kierunku przy dużej prędkości | Zaśmiecona EMI | Stabilna |
| Zaginiony impuls | Szpilka zjada timestamp | Nie występuje |

| Parametr | Wartość | NVS komenda | Opis |
|---|---|---|---|
| `PAS_SAMPLE_INTERVAL_US` | 500 µs | — | Okres próbkowania (stała w kodzie) |
| `pas_debounce_us` | 3000 µs | `pasdbnc:N` | Czas potwierdzenia stanu (500–10000) |
| `g_pas_filter_depth` | 6 | — | Obliczane: debounce_us / 500 |

### Detekcja kierunku (`processPasFilteredEdge`)

Po przejściu filtra cyfrowego, przefiltrowane krawędzie są przetwarzane:

- **RISING edge** → koniec okresu LOW, zapis `low_time`
- **FALLING edge** → koniec okresu HIGH, zapis `high_time`, obliczenie kierunku

Detekcja kierunku z histerezą:
1. Oblicz asymetrię: `diff = |HIGH - LOW|`
2. Jeśli `diff > 5%` okresu → kierunek jednoznaczny
3. **Histereza:** licznik `confidence` (±5) — wymaga kilku zgodnych krawędzi przed zmianą `g_pas_forward`
4. Jeśli `diff ≤ 5%` → magnesy symetryczne, kierunek nie zmieniany
5. `pas_dir_invert` (NVS) — odwraca konwencję HIGH>LOW=forward

Debounce półokresów: krawędzie krótsze niż 5 ms (`PAS_MIN_HALFPERIOD_US`) są odrzucane.

Długi timeout (> 2 s bez krawędzi): reset pomiarów HIGH/LOW — zapobiega mieszaniu
starych i nowych pomiarów po przerwie w pedałowaniu.

### Algorytm sterowania (`calculatePasDuty`)

```
Wejścia:
  isPedalingForward — flaga z ISR (kierunek + histereza)
  Lx                — poziom wspomagania (raw 0–15 z wyświetlacza)
  v_curr            — prędkość koła [km/h] (MA z 8 próbek)
  v_max             — P08 limit prędkości [km/h]

Stany:
  IDLE  → brak impulsów lub reverse → duty = 0
  WAIT  → forward, ale < pas_start_delay_ms → duty = 0
  ACTIVE → forward >= pas_start_delay_ms → sterowanie mocą
```

**Sekwencja w stanie ACTIVE:**

1. **Soft-start:** moc narasta liniowo 0→100% w czasie `pas_ramp_ms` (domyślnie 1500 ms)
2. **V_target:** oblicz prędkość docelową z poziomu wspomagania:
   ```
   V_target = 6 + Lx × (v_max − 6) / 15
   ```
   | Poziom (wyśw.) | Raw Lx | V_target (v_max=25) |
   |---|---|---|
   | L0 | 0 | — (silnik OFF) |
   | L1 | 3 | 9.8 km/h |
   | L2 | 6 | 13.6 km/h |
   | L3 | 9 | 17.4 km/h |
   | L4 | 12 | 21.2 km/h |
   | L5 | 15 | 25.0 km/h |

3. **Speed ramp-down** (strefa 3 km/h):
   - `v_curr < V_target − 3` → pełna moc (100%)
   - `v_curr ∈ [V_target−3, V_target]` → liniowa redukcja 100%→0%
   - `v_curr ≥ V_target` → duty = 0

4. **Slew rate limiter:** duty zmienia się max ±30 PWM na wywołanie (~6% z 500).
   Przy ~2 kHz pętli = max ~60%/s. Zapobiega skokom duty.

5. **Globalny limit P08:** stosowany osobno przez `applyGlobalSpeedLimit()` (dotyczy też manetki).

### Filtracja prędkości (Moving Average)

Surowy odczyt `wheel_speed_kmh` jest niestabilny (1 impuls/obrót → skoki wheeltime).
PAS używa bufora kołowego 8 próbek ze średnią kroczącą:

```
speed_smooth = avg(speed_buf[0..7])
```

Zapobiega to oscylacjom duty w strefie ramp-down.

### Forward holdoff (300 ms)

Gdy ISR zgłosi reverse, PAS nie deaktywuje się natychmiast. Czeka 300 ms
(`PAS_FWD_HOLDOFF_MS`) od ostatniego widzianego forward. Pojedyncze zaszumione
odczyty kierunku nie przerywają wspomagania.

Przy deaktywacji (timeout lub reverse) duty schodzi do zera z ograniczeniem
slew rate — łagodne wygaszenie zamiast twardego odcięcia.

### Wygładzone V_target (smooth transition)

Gdy użytkownik zmienia poziom wspomagania w trakcie jazdy (np. L5→L3),
V_target nie skacze natychmiast z 25 na 17.4 km/h. Zamiast tego przechodzi
płynnie z prędkością ~10 km/h/s (stała `VTARGET_SLEW = 0.005 km/h` na wywołanie).

Bez tego wygładzenia nagły spadek V_target powodował natychmiastowe
zerowanie duty (bo speed > new_v_target), a następnie ponowne rozpędzanie
po spadku prędkości — odczuwalne jako „uderzenie”. Z wygładzeniem duty maleje
stopniowo, a prędkość koła płynnie dostosowuje się do nowej wartości docelowej.

### Kombinacja PAS + Manetka (P10)

Parametr P10 z wyświetlacza S866 określa tryb jazdy:

| P10 | Tryb | Duty |
|-----|------|------|
| 0   | PAS + gaz | `max(throttle_duty, pas_duty)` |
| 1   | Tylko gaz | `throttle_duty` |
| 2   | Tylko PAS | `pas_duty` |

Manetka działa w pełnym zakresie 0–PWM_MAX_DUTY (niezależnie od assist level).
PAS jest ograniczony przez V_target zależny od poziomu wspomagania.

### Zatrzymanie czujnika na magnesie (fix)

Problęm: gdy magnes zatrzyma się tuż przy czujniku Halla, generowane są krótkie szpilki (~17 ms) co kilkanaście sekund. `since` resetuje się do ~17 ms po każdej szpilce, więc timeout (`pas_stop_delay_ms = 1000 ms`) nigdy nie zachodzi — silnik jest nadal napędzany mimo braku pedałowania.

Rozwiązanie: dwa warunki stopu zamiast jednego:
```
timed_out    = (since > stop_delay_us)
period_too_long = (period_us > stop_delay_us)   // NOWE
stop = timed_out || period_too_long
```
Jeśli ostatni zmierzony **okres** jest dłuższy niż stop_delay — to nie jest realne pedałowanie, nawet jeśli ISR widział impulsy.

> **Uwaga:** Przy wolnym pedałowaniu (np. 5 RPM, 1 magnes, period=12s) `stop_delay_ms` musi być ustawiony odpowiednio duże: `passtop:13000`.

### Parametry NVS (konfigurowane komendami Serial)

| Parametr | Domyślnie | Komenda | Opis |
|---|---|---|---|
| `pas_dir_invert` | 0 | `pasdir` | Odwróć konwencję kierunku (toggle) |
| `pas_start_delay_ms` | 2000 | `passtart:N` | Czas ciągłego pedałowania forward do aktywacji [ms] |
| `pas_stop_delay_ms` | 1000 | `passtop:N` | Timeout braku impulsów PAS → wyłącz wspomaganie [ms] |
| `pas_ramp_ms` | 1500 | `pasramp:N` | Soft-start: czas narastania mocy 0→100% [ms] |
| `pas_debounce_us` | 3000 | `pasdbnc:N` | Czas potwierdzenia stanu PAS [µs]. N=debounce/500 próbek filtra |

### Diagnostyka

**Status (auto-status):**
- `PAS:ON 85% vt:14.9 sl:0.85` — aktywny, 85% duty, V_target=14.9 km/h, speed limit factor=0.85
- `PAS:WAIT 850ms/2000ms fw:1 H:45000 L:32000` — w start delay, 850/2000 ms, forward=1, czasy H/L
- `PAS:REV H:28000 L:35000` — kierunek reverse, czasy półokresów

**Status (auto-status)** — linia `[PAS]` wyświetlana zawsze:
```
  [PAS] st:ON edges:3657 since:2ms H:124ms L:83ms asym:20% conf:5 fwd:1 rpm:23 fwd_ms:2350/2000 ped:1 inv:0 d:61% vt:22.8 sl:1.00
```

| Pole | Opis |
|------|------|
| `st:` | Stan: `STOP`/`WAIT`/`ON`/`REV` |
| `edges:` | Licznik krawędzi ISR (róśnie przy pedałowaniu) |
| `since:` | ms od ostatniego impulsu |
| `H:/L:` | Czasy HIGH/LOW w ms |
| `asym:` | Asymetria % (musi być >5% dla detekcji kierunku) |
| `conf:` | Pewność kierunku -5..+5 |
| `fwd:` | Aktualny kierunek (0=wstecz, 1=wprzód) |
| `rpm:` | Wyliczona kadencja [RPM] |
| `fwd_ms:/` | Czas pedałowania forward / wymagany start_delay |
| `ped:` | Czy logika uznała pedałowanie za aktywne |
| `inv:` | Czy kierunek odwrócony (`pasdir`) |
| `d:/vt:/sl:` | (tylko gdy ON) duty%, V_target, speed-limit factor |

**Komenda `pasdbg`:**
```
========== PAS DEBUG ==========
HIGH time:  45123 us
LOW time:   32456 us
Period:     77579 us
Duty H/L:   58% / 42%
Asymetria:  16% (próg: 5%)
g_pas_forward:    1 (confidence: 4)
edge_count:       1234
last_pulse:       15234 us ago
pas_pedaling:     1
fwd_since_ms:     12345
active_since_ms:  10345
pas_dir_invert:   0
pas_start_delay:  2000 ms
pas_stop_delay:   1000 ms
pas_ramp:         1500 ms
P13 magnets:      12
================================
```

### Stałe

| Stała | Wartość | Opis |
|---|---|---|
| `PAS_SAMPLE_INTERVAL_US` | 500 µs | Okres próbkowania timera PAS |
| `PAS_MIN_HALFPERIOD_US` | 5000 µs | Min półokres PAS (debounce krawędzi) |
| `PAS_DIR_MIN_ASYMMETRY` | 5% | Próg asymetrii do detekcji kierunku |
| `PAS_SPEED_MA_SIZE` | 8 | Próbek w buforze Moving Average prędkości |
| `PAS_SLEW_RATE_MAX` | 30 | Max zmiana duty na wywołanie (~3% PWM) |
| `PAS_FWD_HOLDOFF_MS` | 300 ms | Czas podtrzymania forward po szumie reverse |

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
| **FOC** | |
| `F` / `m3`     | Włącz tryb FOC |
| `foc`          | Pokaż status FOC (Vd/Vq, Id/Iq, PI, EMA) |
| `fdbg`         | Debug FOC ON/OFF (co 200ms) |
| `fkp:N.N`      | Ustaw Kp obu osi d/q (0.0–100.0) |
| `fki:N.N`      | Ustaw Ki obu osi d/q (0.0–1000.0) |
| `fkpd:N.N`     | Ustaw Kp tylko osi d (0.0–100.0) |
| `fkid:N.N`     | Ustaw Ki tylko osi d (0.0–1000.0) |
| `fvolt`        | FOC voltage mode ON/OFF (Vq=duty, bez PI) |
| `fpitune`      | Auto-tuning PI metodą relay (Åström-Hägglund, ~5s) |
| **PAS** | |
| `pasdir`       | Odwróć konwencję kierunku PAS (toggle, zapis NVS) |
| `passtart:N`   | Ustaw opóźnienie startu PAS 0-10000 ms (zapis NVS) |
| `passtop:N`    | Ustaw timeout PAS 100-10000 ms (zapis NVS) |
| `pasramp:N`    | Ustaw czas soft-start PAS 0-10000 ms (zapis NVS) |
| `pasdbg`       | Wyświetl pełną diagnostykę PAS (jednorazowo) |
| **Konfiguracja NVS** | |
| `cfg`          | Pokaż konfigurację NVS + runtime |
| `cfg:mode:N` Enter | Tryb boot: 1=BLOCK, 2=SINUS, 3=FOC (zapis NVS) |
| `cfg:ramp:N` Enter | Czas rampy 0-10000 ms (zapis NVS + natychmiast runtime) |
| `cfg:regen:N` Enter | Regeneracja boot: 0=OFF, 1=ON (zapis NVS + natychmiast runtime) |
| `cfg:rev:N` Enter | Kierunek obrotów: 0=CW, 1=CCW (zapis NVS + natychmiast runtime) |
| `cfg:step:N` Enter | Max zmiana duty na krok: 0-100% (0=brak limitu, zapis NVS) |
| `cfg:defaults` | Zaladuj wartości domyślne do runtime (bez zapisu EEPROM) |
| `cfg:save` | Zapisz aktualne wartości runtime do EEPROM |
| `cfg:reload` | Wczytaj wartości z EEPROM i zastosuj do runtime (w tym tryb silnika) |

### Format statusu (jedna linia)

```
BLK D:35/80% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1870) RPM:120 WT:500 P:44.6W DISP:OK L3
```

| Pole    | Znaczenie |
|---------|-----------|
| `BLK`   | Tryb: OFF/BLK/SIN/FOC |
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
| `PAS:ON 85% vt:14.9 sl:0.85` | PAS aktywny: duty%, V_target, speed limit factor |
| `PAS:WAIT 850ms/2000ms fw:1 H:45000 L:32000` | PAS w start delay: upłynęło/wymagane, forward, czasy H/L |
| `PAS:REV H:28000 L:35000` | PAS — pedałowanie do tyłu (czasy półokresów) |
| `FAULT` | Błąd czujników Halla |

Linia `[PAS]` wyświetlana jest zawsze (niezależnie od stanu PAS) — patrz sekcja [PAS — diagnostyka](#pas--pedal-assist-sensor).

Linia `[DBG]` wyświetlana jest w trybach SINUS i FOC:
```
  [DBG] ang:62 spd:37025 hdir:1 sec:3 h:100 hs:4 hp:1500us err:-1 snp:5 cor:26138 fb:1 d:556 vq:58 iq:0.27 tgt:0.50
```

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

## WiFi — interfejs konfiguracyjny WWW

Sterownik udostępnia responsywny interfejs webowy przez WiFi Access Point. Aktywacja przez parametr P17 wyświetlacza S866 (repurposed — nie implementuje tempomatu).

### Włączanie / wyłączanie

| P17 | Stan | Opis |
|-----|------|------|
| **1** | WiFi ON, silnik OFF | Uruchamia AP + HTTP, wyłącza wszystkie tranzystory |
| **0** | WiFi OFF, silnik dostępny | Zatrzymuje AP, wykonuje opcjonalną komendę z kolejki |

Przejście P17: 1→0 jest jedynym momentem kiedy kolejka trybu (`B`, `S`, `F`) jest wykonywana.

### Dane połączenia

| Parametr | Wartość |
|----------|---------|
| SSID | `BLDC_Config` |
| Hasło | `bldc1234` |
| IP | `192.168.4.1` |
| Port | 80 |

### REST API

| Endpoint | Metoda | Opis |
|----------|--------|------|
| `/` | GET | Strona HTML (cały interfejs) |
| `/api/config` | GET | JSON z konfiguracją NVS + stanem runtime |
| `/api/cmd` | POST | Wykonaj komendę Serial natychmiast (pole `cmd`) |
| `/api/queue` | POST | Ustaw komendę do wykonania po WiFi OFF (pole `cmd`) |

> **Ograniczenie `/api/cmd`:** komendy startujące silnik (`B`, `S`, `F`, `e`, `m2`, `m3`) są odrzucane z HTTP 403 gdy WiFi aktywne. Użyj `/api/queue` — komenda wykona się po P17→0.

### ADC a WiFi

W nowej wersji PCB przepustnica (GPIO33) i temperatura FET (GPIO32) korzystają z **ADC1**, który **nie koliduje z WiFi**. Nie ma już potrzeby blokowania odczytów ADC podczas aktywnego WiFi.

Jedyne piny ADC2 nie są używane do pomiarów analogowych — konflikt ADC2/WiFi nie dotyczy tej konfiguracji.

---

## Konfiguracja NVS

Sterownik zapisuje konfigurację w pamięci nieulotnej ESP32 (NVS — Non-Volatile Storage).
Konfiguracja przetrwa restart i wyłączenie zasilania.

### Struktura `controller_config_t` (64 bajty)

| Pole | Typ | Domyślnie | Opis |
|---|---|---|---|
| `magic` | uint32_t | 0x424C4401 | Sentinel walidacyjny ("BLD\x01") |
| `version` | uint16_t | 16 | Wersja struktury (CONFIG_VERSION) |
| `drive_mode` | uint8_t | 1 (BLOCK) | Domyślny tryb po starcie (1=BLOCK, 2=SINUS, 3=FOC) |
| `ramp_time_ms` | uint16_t | 1200 | Czas rampy rozpędzania 0→100% [ms] |
| `regen_enabled` | uint8_t | 0 | Regeneracja ON/OFF (0/1) |
| `pas_dir_invert` | uint8_t | 0 | Odwróć konwencję kierunku PAS (0/1) |
| `pas_start_delay_ms` | uint16_t | 2000 | Opóźnienie startu PAS [ms] |
| `pas_stop_delay_ms` | uint16_t | 1000 | Timeout PAS — brak impulsów [ms] |
| `pas_ramp_ms` | uint16_t | 1500 | Soft-start PAS: czas narastania 0→100% [ms] |
| `duty_max_step_pct` | uint8_t | 5 | Max zmiana duty na wywołanie [% PWM_MAX] |
| `motor_reverse` | uint8_t | 0 | Kierunek obrotów (0=CW, 1=CCW) |
| `sine_hall_offset` | int8_t | 0 | Offset fazy Hall→sinus [-48..+48], wynik `sat`/`so:N` |
| `foc_kp_q` | float | 0.5 | FOC PI Kp osi q [PWM/A] |
| `foc_ki_q` | float | 5.0 | FOC PI Ki osi q [PWM/(A·s)] |
| `foc_kp_d` | float | 0.5 | FOC PI Kp osi d [PWM/A] |
| `foc_ki_d` | float | 5.0 | FOC PI Ki osi d [PWM/(A·s)] |
| `foc_voltage_mode` | uint8_t | 0 | FOC tryb napięciowy (0=PI, 1=Vmode) — diagnostyka |
| `pas_debounce_us` | uint16_t | 3000 | Czas potwierdzenia stanu PAS [µs] (filtr = N×500µs) |
| `display_required` | uint8_t | 1 | Blokada silnika bez wyświetlacza S866 (0/1) |
| `thr_samples` | uint8_t | 8 | Liczba próbek ADC przepustnicy w burście (2–16) |
| `thr_outlier_thresh` | uint16_t | 150 | Max odchylenie próbki od mediany ADC (10–2000) |
| `current_limit_a` | uint8_t | 15 | Limit prądu fazowego [A] (0=brak, nadpisywany przez P14 z display) |
| `_reserved[20]` | uint8_t[] | 0 | Padding do stałego rozmiaru 64 bajtów |

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
| `cfg:rev:N` | Kierunek obrotów: 0=CW, 1=CCW, zapisuje NVS |
| `cfg:step:N` | Max zmiana duty 0-100% na krok, zapisuje NVS |
| `cfg:dispreq:N` | Wymagany wyświetlacz: 0=NIE (standalone), 1=TAK, zapisuje NVS |
| `cfg:thrsamp:N` | Próbki burst przepustnicy: 2-16, zapisuje NVS |
| `cfg:thrdelta:N` | Max odchylenie od mediany ADC: 10-2000, zapisuje NVS |
| `cfg:defaults` | Ładuje wartości domyślne do runtime i config struct **(bez zapisu NVS)** |
| `cfg:save` | Synchronizuje runtime → NVS (zapisuje aktualny stan) |
| `cfg:reload` | Wczytuje NVS → runtime + przełącza tryb silnika jeśli zmieniony |
| `pasdir` | Odwraca konwencję kierunku PAS (toggle), zapisuje NVS |
| `passtart:N` | Opóźnienie startu PAS 0-10000 ms, zapisuje NVS |
| `passtop:N` | Timeout PAS 100-10000 ms, zapisuje NVS |
| `pasramp:N` | Soft-start PAS 0-10000 ms, zapisuje NVS |
| `pasdbnc:N` | Czas potwierdzenia stanu PAS 500-10000 µs (filtr = N/500 próbek), zapisuje NVS |

### Przykład wyjścia `cfg`

```
========== KONFIGURACJA NVS ==========
drive_mode:    3 (FOC)
               Tryb pracy silnika po starcie: 1=BLOCK, 2=SINUS, 3=FOC
ramp_time_ms:  1200 ms
               Czas narastania duty 0->100%. 0=natychmiastowy. Dziala w obu kierunkach.
regen_enabled: 0 (OFF)
               Hamowanie regeneracyjne (odzyskiwanie energii do baterii).
pas_dir_inv:   0 (NORM)
               Inwersja kierunku PAS. Uzyj gdy silnik napedza wstecz.
pas_start_ms:  2000 ms
               Czas ciaglego pedalowania wymagany do aktywacji silnika.
pas_stop_ms:   1000 ms
               Czas bez impulsow PAS po ktorym silnik wylacza wspomaganie.
pas_ramp_ms:   1500 ms
               Soft-start PAS: czas narastania mocy 0->100% po aktywacji.
pas_dbnc_us:   3000 us (filter: 6 probek x 500 us)
               Czas potwierdzenia stanu PAS. Probkowanie co 500us, N zgodnych = zmiana stanu.
disp_req:      1 (TAK)
               Blokada silnika gdy wyswietlacz S866 nie jest polaczony.
thr_samples:   8
               Liczba probek ADC w jednym odczycie gazu (burst). Wiecej=gladszy, wolniejszy.
thr_outlier:   150
               Max odchylenie probki od mediany (ADC). Probki dalsze odrzucane jako szum.
duty_step:     5 %
               Max zmiana duty na krok petli. 0=bez limitu. Ogranicza szarpniecia mocy.
motor_rev:     0 (CW)
               Kierunek obrotow silnika. CW=normalny, CCW=odwrocony.
sine_offset:   -6 (-22.5 deg)
               Przesuniecie fazowe Hall->sinus. Dobierane komenda 'sat' (auto-tune).
foc_kp_q:      0.450
               FOC: wzmocnienie proporcjonalne PI osi Q (moment obrotowy).
foc_ki_q:      12.345
               FOC: wzmocnienie calkujace PI osi Q (moment obrotowy).
foc_kp_d:      0.450
               FOC: wzmocnienie proporcjonalne PI osi D (strumien magnetyczny).
foc_ki_d:      12.345
               FOC: wzmocnienie calkujace PI osi D (strumien magnetyczny).
foc_vmode:     0 (OFF)
               FOC tryb napieciowy: sterowanie napieciem bez PI. Do diagnostyki.
magic:         0x424C4401 OK
version:       12
======================================
--- Runtime (bieżące) ---
mode:          FOC
ramp_time_ms:  1200 ms
regen:         OFF
direction:     CW
sine_offset:   -6 (-22.5 deg)
foc_kp_q:      0.450
foc_ki_q:      12.345
foc_kp_d:      0.450
foc_ki_d:      12.345
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
2. ISR komutacji **nie nadpisuje rejestrów MCPWM** (flaga `g_mosfet_test_active`)
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

W trybie testowym używane są te same helpery MCPWM co w normalnej pracy:
```
HIGH-side ON: mcpwm_phase_pwm(op, test_duty)
  → gen_A=PWM, compare=test_duty, gen_B=forced HIGH (LS OFF)

LOW-side ON:  (gen_force = MCPWM_FORCE_GND na wybranym operatorze)
  → gen_A=forced LOW (HS OFF), gen_B=forced LOW (LS ON)
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

## FOC — Field Oriented Control

Tryb `DRIVE_MODE_FOC` implementuje sterowanie wektorowe (FOC) w ramie obrotowej dq.
Silnik jest sterowany jako wektor napięciowy o składowych Vd (pole) i Vq (moment),
co daje potencjał do precyzyjnej regulacji momentu obrotowego.

### Architektura

```
loop() (~2 kHz)                          ISR (20 kHz, MCPWM TEZ)
───────────────                          ───────────

ADC → EMA filtr                          Read Vd_i, Vq_i (volatile int32)
   ↓                                        ↓
Clarke (Ia,Ib,Ic → Iα,Iβ)              Inverse Park (Vd,Vq → Vα,Vβ)
   ↓                                        ↓
Park (Iα,Iβ → Id,Iq)                  Inverse Clarke (Vα,Vβ → Va,Vb,Vc)
   ↓                                        ↓
PI(Id) + PI(Iq)                          SVPWM min-max centering
   ↓                                        ↓
Vd = ff_d + korekta_d                    MCPWM register write (3 operatory)
Vq = ff_q + korekta_q
   ↓
Zapisz Vd_i, Vq_i (volatile int32)
```

**KRYTYCZNE:** ISR timera (level 3 interrupt) na ESP32 **nie zapisuje kontekstu FPU**.
Użycie `float` w ISR korumpuje rejestry koprocesora → crash `LoadProhibited`.
Dlatego cała arytmetyka ISR używa `int32_t` (Q10 fixed-point).

### Feedforward + PI

FOC używa architektury **feedforward + korekta PI**:

```
Vq = feedforward(duty) + PI_korekta(±100 max)
     └─ natychmiastowe      └─ mała korekta prądowa
         napięcie z duty          (gdy ADC złapie prąd)
```

- **Feedforward:** `Vq_ff = duty_cycle` (po rampie). Silnik otrzymuje napięcie
  natychmiast, identycznie jak w trybie SINUS. Rampa rozpędzania obowiązuje.
- **PI korekta:** `±FOC_PI_CORR_LIMIT` (domyślnie ±100 PWM) wokół feedforward.
  Regulator koryguje Vq w górę/dół na podstawie błędu prądowego Id/Iq.
- **Vd feedforward = 0** (cel: Id = 0, MTPA — max torque per amp)

Bez feedforward PI integrowało od zera → silnik rozpędzał się bardzo wolno
(Ki=5, err=1.5A → ~7.5 PWM/s → 20s do 153 PWM). Z feedforward reakcja jest
natychmiastowa.

### Kąt i inverse Park

Kąt θ jest współdzielony z trybem SINUS (Hall tracking + interpolacja Q16).
Inverse Park w ISR używa konwencji znaków dopasowanej do mapowania faz
A=0°, B=240°, C=120°:

```
Vα = (Vd·cos + Vq·sin) >> 10
Vβ = (Vd·sin - Vq·cos) >> 10
```

Forward Park w loop() ma odpowiadające znaki:
```
Id =  Iα·cos + Iβ·sin
Iq =  Iα·sin - Iβ·cos
```

### Pomiar prądów i filtr EMA

Czujniki INA180A2 są **jednokierunkowe** (klipują prąd ujemny do ~0V).
`analogRead()` nie jest zsynchronizowany z PWM → ~50% odczytów trafia
w czas gdy low-side jest OFF (prąd = 0).

Aby uzyskać stabilny feedback dla PI:

1. **EMA filtr** (α = 0.05, τ ≈ 10ms):
   ```
   g_foc_ia_ema += α × (ia_raw - g_foc_ia_ema)
   ```
   Uśrednia sporadyczne odczyty do ciągłego sygnału.

2. **Rekonstrukcja Kirchhoffa:** Faza z najmniejszym odczytem = -(suma dwóch pozostałych).
   Pozwala odtworzyć prąd ujemny z jednokierunkowych czujników.

### Tryb napięciowy (`fvolt`)

Komenda `fvolt` przełącza FOC w **tryb napięciowy** (open-loop):
- `Vq = duty` (bezpośrednio), `Vd = 0`
- Brak PI — identyczne zachowanie jak SINUS ale w ramie dq
- Pozwala porównać jakość pracy z trybem SINUS i wykluczyć PI jako źródło problemów
- Debug: `[FOC-V]` vs `[FOC-PI]`

### Stałe FOC

| Stała | Wartość | Opis |
|---|---|---|
| `FOC_KP_DEFAULT` | 0.5 | Domyślne Kp regulatorów PI d/q [PWM/A] |
| `FOC_KI_DEFAULT` | 5.0 | Domyślne Ki regulatorów PI d/q [PWM/(A·s)] |
| `FOC_INTEGRAL_LIMIT` | SINE_SAFE_MAX_DUTY (375) | Absolutny max integrala |
| `FOC_PI_CORR_LIMIT` | 100 | Max korekta PI wokół feedforward [±PWM] |
| `FOC_IQ_MAX` | 10.0 | Maksymalny prąd Iq target [A] |
| `FOC_LOOP_DT` | 0.0005 s | Przybliżony dt pętli prądowej (~2kHz) |
| `FOC_CURRENT_EMA_ALPHA` | 0.05 | EMA α dla filtrowania prądów |

### Komendy Serial

| Komenda | Opis |
|---------|------|
| `F` / `m3` | Włącz tryb FOC |
| `foc` | Status FOC: Kp, Ki, Limit, Vd/Vq, Id/Iq, EMA, tryb |
| `fdbg` | Debug ON/OFF (co 200ms): Vd, Vq, Id, Iq, tgt, lim, ff, EMA prądy |
| `fkp:N.N` | Ustaw Kp obu osi d/q (0.0–100.0) |
| `fki:N.N` | Ustaw Ki obu osi d/q (0.0–1000.0) |
| `fkpd:N.N` | Ustaw Kp tylko osi d (0.0–100.0) |
| `fkid:N.N` | Ustaw Ki tylko osi d (0.0–1000.0) |
| `fvolt` | Tryb napięciowy ON/OFF (Vq=duty, bez PI) |
| `fpitune` | Auto-tuning PI metodą relay feedback (~5s) |

### Regulator PI — co to jest Kp i Ki

Regulator **PI (Proportional-Integral)** steruje prądem w osiach d i q silnika BLDC.
Wyjście regulatora = **feedforward** (Vq = duty, natychmiastowe napięcie) **+ korekta PI** (±100 PWM max).

| Parametr | Pełna nazwa | Rola | Efekt za małej wartości | Efekt za dużej wartości | Domyślnie |
|----------|-------------|------|-------------------------|-------------------------|-----------|
| **Kp** | Wzmocnienie proporcjonalne | Natychmiastowa reakcja na błąd prądu (Iq_target − Iq_meas) | Wolna reakcja, błąd ustalony | Oscylacje, niestabilność | 0.5 |
| **Ki** | Wzmocnienie całkujące | Kumuluje błąd w czasie, eliminuje błąd ustalony | Wolne dochodzenie do celu | Przeregulowanie (overshoot), oscylacje | 5.0 |

**Jednostki:** Kp [PWM/A], Ki [PWM/(A·s)].  
**Anti-windup:** Całka ograniczona do ±pi_limit (= min(FOC_PI_CORR_LIMIT, feedforward)).

### Auto-tuning PI (`fpitune`)

Komenda `fpitune` implementuje automatyczne strojenie parametrów PI **metodą relay feedback
(Åström-Hägglund, 1984)**. Jest to standardowa metoda strojenia PID w przemyśle.

**Algorytm:**
1. Zamiast regulatora PI, wyjście jest przełączane (relay): Vq = feedforward ± 30 PWM
2. Gdy błąd Iq > 0 → wyjście = feedforward + relay_amp
3. Gdy błąd Iq < 0 → wyjście = feedforward − relay_amp
4. Mierzone są oscylacje prądu Iq (amplituda `a` i okres `Tu`)
5. Z oscylacji obliczany jest **ultimate gain** Ku = 4·d / (π·a)
6. Parametry PI wg **reguł Zieglera-Nicholsa**: Kp = 0.45·Ku, Ki = 0.54·Ku/Tu

**Wymagania:**
- Tryb FOC aktywny (komenda `F`)
- Silnik pracuje (duty > 0)
- Voltage mode wyłączony (`fvolt` = OFF)

**Użycie:**
```
F              ← włącz FOC
50             ← ustaw duty 50%
fpitune        ← start auto-tune (~5s)
               ← czekaj na wynik...
fkp:X.XXX      ← zastosuj sugerowane Kp
fki:X.XXX      ← zastosuj sugerowane Ki
```

**Uwaga:** Podczas testu silnik może lekko oscylować — to normalne zachowanie metody relay.
Test trwa 5 sekund i kończy się automatycznie, wypisując sugerowane wartości Kp/Ki.

### Format debug (`fdbg`)

```
[FOC-PI] Vd=  0.0 Vq=153.0 | Id= 0.01 Iq= 0.03 | tgt= 1.5 lim=100 ff=153 | ia= 0.12 ib= 0.08 ic= 0.05
         │        │           │         │           │         │    │      └─ EMA prądy faz [A]
         │        │           │         │           │         │    └─ feedforward [PWM]
         │        │           │         │           │         └─ PI limit [±PWM]
         │        │           │         │           └─ Iq target z przepustnicy [A]
         │        │           │         └─ Zmierzony Iq [A] (Park)
         │        │           └─ Zmierzony Id [A]
         │        └─ Napięcie q (torque) = ff + korekta [PWM]
         └─ Napięcie d (field) = 0 + korekta [PWM]
```

### Procedura strojenia

**1. Sprawdź tryb napięciowy (powinien brzmieć jak SINUS):**
```
F              ← włącz FOC
fvolt          ← tryb napięciowy (bez PI)
fdbg           ← debug ON
```
Jeśli brzmi jak SINUS — ISR działa poprawnie.

**2. Przełącz na PI i dostrajaj:**
```
fvolt          ← wyłącz voltage mode (wróć na PI)
fkp:0          ← wyłącz Kp
fki:1.0        ← niskie Ki (korekta wolno narasta)
```

**3. Zwiększaj Ki stopniowo:**
```
fki:2.0        ← cicho? →
fki:5.0        ← hałas? → cofnij do 2.0
```

**4. Dodaj Kp (reakcja na zmiany):**
```
fkp:0.1        ← obserwuj
fkp:0.5        ← oscylacja? → cofnij
```

### Na co patrzeć w debug

| Symptom | Przyczyna | Rozwiązanie |
|---------|-----------|-------------|
| `Vq` skacze ±20% | Oscylacja PI (Ki za duże) | `fki:1.0` |
| `Vq` stałe = ff | PI nasycony (OK, quasi-open-loop) | — |
| `ia/ib/ic` ≈ 0.00 | ADC nie łapie prądów | Obniż Ki (działa open-loop) |
| `Id` duże (>0.5A) | Zły kąt fazy | `so+`/`so-` strojenie |
| `Iq` bliskie `tgt` | Zamknięta pętla działa! | Zwiększ Ki/Kp |
| Silnik wibruje stojąc | Kąt daleko od Halla | `so+` kilka razy |

### Ograniczenia sprzętowe

Pełne FOC (zamknięta pętla prądowa) wymaga:
- **Dwukierunkowego czujnika prądu** (np. INA240 z Vref=1.65V) — INA180A2 klipuje prąd ujemny
- **ADC zsynchronizowanego z PWM** (sample w centrum impulsu) — `analogRead()` trafia losowo
- **Szybkiego ADC** (<5µs/kanał) — ESP32 `analogRead()` ≈ 100µs

Z obecnym sprzętem FOC działa jako **"sinus z feedforward + lekkie korekty PI"**.
Optymalne ustawienie: niskie Ki (1–3), Kp ≈ 0–0.5, co daje zachowanie zbliżone do SINUS
ale z ramą dq.

### Aktywacja

- Komenda `F` (natychmiastowa, bez Enter) lub `m3` + Enter
- Silnik startuje identycznie jak w trybie SINUS (snap do sektora, crawl, rampa)
- Do FOC-V: `fvolt` po włączeniu FOC
- Powrót: `S` (SINUS), `e` (BLOCK), `d` (wyłącz)

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

---

## Rozbudowa projektu

### Zaimplementowane w wersji 2.0.0

| # | Funkcjonalność | Opis |
|---|----------------|------|
| 1 | **MCPWM center-aligned PWM** | Migracja z LEDC na MCPWM: 3 operatory × 2 generatory, UP_DOWN counter, bezpośrednie rejestry LL w ISR |
| 2 | **ISR w przerwaniu TEZ MCPWM** | Eliminacja osobnego hw_timer — ISR wyzwalana w dolinie PWM (optymalny pomiar prądu) |
| 3 | **Komutacja sinusoidalna** | Komenda `S` / `m2` — tryb sinusoidalny z interpolacją kąta Hall, SVPWM |
| 4 | **FOC (Field Oriented Control)** | Komenda `F` / `m3` — feedforward + PI, Park/Clarke, SVPWM, tryb napięciowy i PI |
| 5 | **PAS — Pedal Assist Sensor** | Timer sampling 2 kHz, filtr cyfrowy, detekcja kierunku, soft-start, maszyna stanów |
| 6 | **WiFi — interfejs WWW** | AP mode, responsive UI, REST API, kolejka komend trybu |
| 7 | **Konfiguracja NVS (EEPROM)** | 22 parametry, CONFIG_VERSION=16, Serial `cfg` + Web UI |
| 8 | **Filtracja przepustnicy** | Ring buffer + median + odrzucanie outlierów (odporność na EMI) |
| 9 | **Filtracja PAS** | Timer sampling 500 µs, N kolejnych zgodnych próbek (odporność na EMI) |
| 10 | **Wyświetlacz S866** | Protokół 2, prędkość, moc, poziomy wspomagania, flaga `display_required` |

### Planowane rozszerzenia

#### 1. Zabezpieczenia (rozszerzone)
- Overcurrent: porównać `phase_current[i]` z progiem → `allMosfetsOff()` + `fault = true`
- Overtemperature: po skalibrowanym czujniku
- Undervoltage: sprawdzać `battery_voltage` < próg
- TVS dioda na szynie DC jako hardwarowy clamp (regen overvoltage)

#### 2. Regulacja siły regen
- Mapowanie siły hamowania z prędkości (szybciej → więcej hamowania)
- Pętla prądowa: INA180A2 → limit prądu regen z P14
- Konfiguracja duty regen z wyświetlacza lub komend Serial

#### 3. Konfiguracja UART — rozszerzenie
- Zmiana `THROTTLE_DEAD_ZONE`, `THROTTLE_MAX_RAW` bez rekompilacji

#### 4. CAN bus / RS485
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

*Dokumentacja wygenerowana: 2026-03-22*  
*Wersja firmware: 2.0.0 (MCPWM center-aligned + BLOCK + SINUS + FOC + PAS + WiFi + NVS)*  
*CONFIG_VERSION: 16 | controller_config_t: 64 bajty*
