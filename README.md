# BLDC Motor Driver — ESP32

Sterownik silnika BLDC (bezszczotkowego prądu stałego) na bazie ESP32 z mostkami IR2103 (3 fazy).  
Obsługiwana metoda sterowania: **komutacja blokowa (6-step / trapezoidalna)**.  
Planowane rozszerzenia: sterowanie sinusoidalne, FOC.

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
15. [Znane problemy i uwagi](#znane-problemy-i-uwagi)
16. [Rozbudowa projektu](#rozbudowa-projektu)

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

> Zakres 400–2600 został skalibrowany empirycznie dla konkretnej przepustnicy.  
> Jeśli przepustnica ma inny zakres, zmień `THROTTLE_DEAD_ZONE`, `THROTTLE_MIN_RAW`, `THROTTLE_MAX_RAW` w `main.cpp`.

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

### 1. Sterowanie sinusoidalne (DRIVE_MODE_SINUS)
- Zamiast włączyć/wyłączyć fazę: generować sinusoidalne PWM na wszystkich 3 fazach
- Eliminuje szarpanie na niskich obrotach
- Wymaga: tabeli sinusoid + mapowania kąta Halla na fazę
- **Infrastruktura gotowa:** ISR ma dispatch `g_mode_isr` — wystarczy dodać `sinusCommutateISR()`
- Elementy niezależne od trybu (nie wymagają zmian): przepustnica, rampa, assist levels,
  regen, wyświetlacz, pomiar prędkości, obliczanie mocy, komendy Serial

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

### 5. Konfiguracja przez UART
- Możliwość zmiany `THROTTLE_DEAD_ZONE`, `THROTTLE_MAX_RAW` bez rekompilacji
- Zapis do NVS (Non-Volatile Storage) ESP32

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

*Dokumentacja wygenerowana: 2026-02-23*  
*Wersja firmware: 0.1.0*
