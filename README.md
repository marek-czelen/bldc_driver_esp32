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
7. [Przepustnica](#przepustnica)
8. [Sterowanie przez UART/Serial](#sterowanie-przez-uartserial)
9. [Kalibracja prądów](#kalibracja-prądów)
10. [Znane problemy i uwagi](#znane-problemy-i-uwagi)
11. [Rozbudowa projektu](#rozbudowa-projektu)

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

| GPIO | Funkcja         |
|------|-----------------|
| 1    | UART0 TX        |
| 3    | UART0 RX        |
| 17   | UART Enable     |
| 21   | Wyjście prędkości |
| 0    | EXT1 (boot pin!) |
| 4    | EXT2            |
| 16   | EXT3            |

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
src/
  main.cpp          — cała logika aplikacji
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
- prędkość obrotowa w RPM (do implementacji)
- flagi: `brake_active`, `pas_active`, `fault`

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
  ├── readDigitalInputs()  — hamulec, PAS
  ├── obliczenie duty z przepustnicy (jeśli BLOCK mode)
  ├── aktualizacja zmiennych volatile dla ISR
  ├── processSerialCommands() — obsługa komend UART
  └── auto-status co 1s (jeśli włączony)

onCommutationTimer() [ISR, 20 kHz = co 50 µs]
  ├── sprawdzenie hamulca → wyłącz wszystko
  ├── sprawdzenie g_motor_enabled
  ├── odczyt Halli bezpośrednio z rejestrów GPIO.in
  └── ustawienie 6 kanałów LEDC zgodnie z tabelą komutacji
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

## Przepustnica

- **Pin:** GPIO2 (ADC2_CH2)
- **Martwa strefa:** wartości ADC < 400 → duty = 0
- **Zakres roboczy:** 400–2600 ADC → 0–100% duty
- **Mapowanie:** `map(thr, 400, 2600, 0, PWM_MAX_DUTY)`

> Zakres 400–2600 został skalibrowany empirycznie dla konkretnej przepustnicy.  
> Jeśli przepustnica ma inny zakres, zmień `THROTTLE_DEAD_ZONE`, `THROTTLE_MIN_RAW`, `THROTTLE_MAX_RAW` w `main.cpp`.

Przepustnica jest odczytywana tylko gdy `mode == DRIVE_MODE_BLOCK`.  
Komendy UART (`+`, `-`, liczba%) również ustawiają `duty_cycle`, ale przepustnica nadpisuje tę wartość w każdej iteracji `loop()`.

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
| `s`             | Wyświetl status (jednorazowo) |
| `a`             | Toggle auto-status co 1 s |
| `h`             | Wyświetl pomoc |

### Format statusu (jedna linia)

```
BLK CW D:45% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1870)
```

| Pole    | Znaczenie |
|---------|-----------|
| `BLK`   | Tryb: OFF/BLK/SIN/FOC |
| `CW`    | Kierunek: CW/CCW |
| `D:45%` | Duty cycle PWM |
| `V:36.1`| Napięcie baterii [V] |
| `Ia/Ib/Ic` | Prądy fazowe A, B, C [A] |
| `H:101` | Stan Halla [C:B:A] binarnie |
| `T:312` | Surowa wartość ADC temperatury |
| `Thr:45%(1870)` | Przepustnica: % i RAW ADC |
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

### 2. FOC (Field Oriented Control) (DRIVE_MODE_FOC)
- Transformacja Clarke/Park
- Regulatory PI dla prądu Id/Iq
- Wymaga szybkiego ADC (synchronizacja z PWM) i enkoderu/resolvera

### 3. Pomiar RPM
- Liczyć przejścia stanów Halla: 6 przejść = 1 obrót elektryczny
- `g_bldc_state.rpm` jest zarezerwowane w strukturze

### 4. Zabezpieczenia
- Overcurrent: porównać `phase_current[i]` z progiem → `allMosfetsOff()` + `fault = true`
- Overtemperature: po skalibrowanym czujniku
- Undervoltage: sprawdzać `battery_voltage` < próg

### 5. Konfiguracja przez UART
- Możliwość zmiany `THROTTLE_DEAD_ZONE`, `THROTTLE_MAX_RAW` bez rekompilacji
- Zapis do NVS (Non-Volatile Storage) ESP32

### 6. CAN bus / UART protokół
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
