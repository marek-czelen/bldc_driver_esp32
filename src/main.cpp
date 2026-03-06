/**
 * @file main.cpp
 * @brief BLDC Motor Driver — ESP32
 * @version 0.1.0
 *
 * Sterownik silnika BLDC (bezszczotkowego) 3-fazowego na ESP32.
 * Aktualnie obsługiwana metoda: komutacja blokowa (6-step / trapezoidalna).
 *
 * ## Architektura
 * Komutacja odbywa się w ISR timera sprzętowego (20 kHz), niezależnie od loop().
 * Dzięki temu Serial.printf() i inne wolne operacje w loop() nie powodują
 * zakłóceń w sterowaniu silnikiem.
 *
 * ## Sterowniki mostu
 * IR2103: HIN=active HIGH (high-side ON gdy HIGH),
 *         LIN=active LOW (low-side ON gdy LOW — logika odwrócona!).
 * W LEDC: duty=0 na LIN = LOW = low-side ON,
 *         duty=PWM_MAX_DUTY na LIN = HIGH = low-side OFF.
 *
 * ## Przepływ danych
 * loop() czyta ADC/Hall/GPIO → aktualizuje zmienne volatile → ISR odczytuje je
 * w każdym przerwaniu i ustawia odpowiednie kanały LEDC.
 *
 * ## Hardware
 * - MCU: ESP32-D0WDQ6, 240 MHz, Arduino via PlatformIO
 * - Gate drivers: 3x IR2103
 * - Pomiar prądu: INA180A2 (gain=50 V/V) + shunt 2 mΩ
 * - Czujniki Halla: GPIO5(A)/18(B)/19(C), INPUT_PULLUP
 * - VBAT dzielnik: 1.13 MΩ / 31.7 kΩ
 * - Przepustnica: GPIO2, ADC 400-2600 RAW → 0-100%
 */

#include <Arduino.h>
#include "pinout.h"
#include "bldc_types.h"
#include "display_s866.h"
#include "bldc_config.h"

// ============================================================================
// Zmienne globalne
// ============================================================================

bldc_state_t g_bldc_state;

// Timer sprzętowy do komutacji
hw_timer_t *commutationTimer = NULL;
volatile uint8_t g_hall_isr = 0;
volatile uint16_t g_duty_isr = 0;
volatile bool g_motor_enabled = false;
volatile bool g_brake_isr = false;
volatile motor_direction_t g_direction_isr = DIRECTION_CW;
volatile drive_mode_t g_mode_isr = DRIVE_MODE_DISABLED;   ///< Tryb sterowania (do ISR)

// Pomiar RPM z przejść Halla (aktualizowane w ISR timera)
volatile uint8_t g_hall_prev_isr = 0;         ///< Poprzedni stan Halla w ISR
volatile uint32_t g_hall_last_change_us = 0;  ///< micros() ostatniego przejścia
volatile uint32_t g_hall_period_us = 0;       ///< Okres między przejściami Halla [µs]

// Sinusoidal commutation state — ported from bldc_driver_v2 (STM32, proven algorithm)
// Continuous angle tracking with Hall correction, NOT snap-to-hall approach.
volatile uint32_t g_sine_angle_q16 = 0;       ///< Kąt elektr. w wpisach tabeli (Q16, 0..96<<16)
volatile uint32_t g_sine_speed_q16 = 0;       ///< Prędkość: wpisów tabeli na tick ISR (Q16)
volatile uint8_t  g_sine_running = 0;          ///< 1 = tryb sinusoidalny aktywny, 0 = block startup
volatile uint8_t  g_sine_startup_count = 0;    ///< Licznik komutacji blokowych przed przejściem na sinus
volatile int8_t   g_sine_last_hall_idx = -1;   ///< Ostatni indeks Halla w sekwencji (0-5, -1=unknown)
volatile int8_t   g_sine_dir = 1;              ///< Kierunek z przejść Halla: +1 forward, -1 reverse
volatile uint32_t g_sine_last_hall_ms = 0;     ///< HAL tick ostatniego przejścia Halla (stall detection)
volatile int8_t   g_sine_hall_phase_offset = 0; ///< Runtime-tunable Hall phase offset (-48..+48 entries, 1 entry = 3.75°)

// Debug sterowania sinusoidalnego (snapshot + liczniki zdarzeń ISR)
volatile uint32_t g_dbg_hall_edges = 0;
volatile uint32_t g_dbg_sine_enter_count = 0;
volatile uint32_t g_dbg_sine_fallback_count = 0;
volatile uint32_t g_dbg_sine_start_reject_count = 0;
volatile uint8_t  g_dbg_last_hall = 0;
volatile uint32_t g_dbg_last_sine_enter_ms = 0;
volatile uint32_t g_dbg_last_fallback_ms = 0;
volatile uint16_t g_dbg_last_amp = 0;
volatile int16_t  g_dbg_last_ma = 0;
volatile int16_t  g_dbg_last_mb = 0;
volatile int16_t  g_dbg_last_mc = 0;

// Pomiar RPM z pinu SPEED (GPIO ISR, dla silników przekładniowych, P07==1)
volatile uint32_t g_speed_last_pulse_us = 0;  ///< micros() ostatniego impulsu SPEED
volatile uint32_t g_speed_period_us = 0;      ///< Okres między impulsami SPEED [µs]

// PAS (Pedal Assist Sensor) — pomiar kadencji i kierunku z przerwania GPIO
volatile uint32_t g_pas_last_pulse_us = 0;    ///< micros() ostatniej krawędzi PAS
volatile uint32_t g_pas_period_us = 0;        ///< Pełny okres (HIGH+LOW) PAS [µs]
volatile uint32_t g_pas_rising_us = 0;        ///< Czas ostatniego RISING edge
volatile uint32_t g_pas_falling_us = 0;       ///< Czas ostatniego FALLING edge
volatile uint32_t g_pas_high_time_us = 0;     ///< Czas trwania stanu HIGH [µs]
volatile uint32_t g_pas_low_time_us = 0;      ///< Czas trwania stanu LOW [µs]
volatile bool g_pas_forward = true;           ///< Kierunek pedałowania (asymetria duty cycle)

/// Timeout PAS: jeśli >1.5s bez impulsu → kadencja = 0 (użytkownik przestał pedałować)
#define PAS_TIMEOUT_US          1500000
/// Minimalny półokres PAS: odrzucaj krawędzie szybsze niż 5ms (debounce)
#define PAS_MIN_HALFPERIOD_US   5000
/// Maksymalna kadencja [RPM] zanim obetniemy (ochrona przed szumem)
#define PAS_MAX_CADENCE_RPM     150
/// Minimalna różnica duty cycle do detekcji kierunku (5% = 0.05)
/// Jeśli |HIGH-LOW| < 5% okresu → magnesy symetryczne, zakładamy forward
#define PAS_DIR_MIN_ASYMMETRY   5
/// Maksymalna kadencja (prędkość pedałowania) mapowana na pełną prędkość koła [RPM]
/// Typowa kadencja sprawneg rowerzysty to 60-90 RPM
#define PAS_CADENCE_MAX_RPM     80

// Regeneracja — zmienne volatile dla ISR
volatile bool g_regen_active_isr = false;     ///< Tryb regen aktywny (do ISR)
volatile uint16_t g_regen_duty_isr = 0;       ///< Siła hamowania regen 0-PWM_MAX_DUTY

// Tryb testowy MOSFETów — diagnostyka uszkodzonych tranzystorów
volatile bool g_mosfet_test_active = false;    ///< Tryb testu MOSFET aktywny (ISR nie rusza LEDC)
static uint16_t g_mosfet_test_duty = PWM_MAX_DUTY * 10 / 100;  ///< Duty testowe (domyślnie 10%)
static char g_mosfet_test_phase = 0;           ///< Aktualnie testowana faza ('A','B','C') lub 0
static char g_mosfet_test_side  = 0;           ///< Aktualnie testowana strona ('H','L') lub 0

// ============================================================================
// Sterowanie sinusoidalne — port z bldc_driver_v2 (STM32, sprawdzony algorytm)
// ============================================================================
//
// Algorytm źródłowy: bldc_driver_v2/src/bldc.c, TIM1_UP_IRQHandler()
// Kluczowe cechy:
//   1. Ciągłe śledzenie kąta (angle_q16 += speed_q16 co tick ISR)
//   2. Hall KORYGUJE kąt (1/8 błędu), NIE narzuca go
//   3. Block startup: 6 komutacji blokowych buduje dane o prędkości
//   4. Stall freeze: brak Halla >200ms → zamrożenie kąta
//   5. Center-aligned complementary PWM: duty = center + sine * amp
//
// Tablica: 97 elementów (96 + guard entry), wartości -1024..+1024
// 96 wpisy = 360° elektrycznych, 16 wpisów na sektor (60°)
// Rozdzielczość: 3.75° na wpis

/**
 * @brief Tablica sinusa: 96 wpisów + 1 guard (wrap-around).
 * Wartości: round(sin(i × 360°/96) × 1024), zakres -1024..+1024.
 * Guard entry [96] = [0] = 0 dla bezpiecznej interpolacji.
 * DRAM_ATTR: uint8/int16 load z IRAM → LoadStoreError na ESP32.
 */
static const DRAM_ATTR int16_t g_sine_table[97] = {
       0,   67,  134,  200,  265,  329,  392,  453,
     512,  569,  623,  675,  724,  770,  812,  851,
     887,  918,  946,  970,  989, 1004, 1015, 1022,
    1024, 1022, 1015, 1004,  989,  970,  946,  918,
     887,  851,  812,  770,  724,  675,  623,  569,
     512,  453,  392,  329,  265,  200,  134,   67,
       0,  -67, -134, -200, -265, -329, -392, -453,
    -512, -569, -623, -675, -724, -770, -812, -851,
    -887, -918, -946, -970, -989,-1004,-1015,-1022,
   -1024,-1022,-1015,-1004, -989, -970, -946, -918,
    -887, -851, -812, -770, -724, -675, -623, -569,
    -512, -453, -392, -329, -265, -200, -134,  -67,
       0   // guard entry [96] = entry [0]
};

/**
 * @brief Mapowanie Hall→indeks sektora (0-5) dla sekwencji CW.
 *
 * Sekwencja CW z komutacji blokowej: 1→3→2→6→4→5
 * Sektor 0 = Hall 1, Sektor 1 = Hall 3, ... Sektor 5 = Hall 5
 * Wartość -1 = nieprawidłowy stan Halla (0 lub 7).
 */
static const DRAM_ATTR int8_t g_hall_to_sector[8] = {
    -1,     // 0 = invalid
     0,     // 1 (001) → sector 0  (block: A→B)
     2,     // 2 (010) → sector 2  (block: B→C−)
     1,     // 3 (011) → sector 1  (block: A→C−)
     4,     // 4 (100) → sector 4  (block: C→A−)
     5,     // 5 (101) → sector 5  (block: C→B−)
     3,     // 6 (110) → sector 3  (block: B→A−)
    -1      // 7 = invalid
};

// Stałe sinusoidalne (identyczne z bldc_driver_v2)
#define SINE_TABLE_SIZE         96
#define SINE_TABLE_Q16_FULL     (96UL << 16)   // 6291456
#define SINE_SECTOR_ENTRIES     16              // 96 / 6
#define SINE_SECTOR_CENTER      8               // środek sektora
// SINE_HALL_PHASE_OFFSET: teraz runtime variable g_sine_hall_phase_offset (komendy so+/so-/so:N)
// Domyślnie 0; strojenie: 1 wpis = 3.75° elektr.
// Offsety fazowe: dopasowane do tabeli komutacji blokowej CW (1→3→2→6→4→5)
// Faza A = referencyjna (peak w sektorach 0,1)
// Faza B = +240° (peak w sektorach 2,3)
// Faza C = +120° (peak w sektorach 4,5)
#define SINE_PHASE_A_OFFSET     0               // faza referencyjna
#define SINE_PHASE_B_OFFSET     64              // 96*2/3 = 240°
#define SINE_PHASE_C_OFFSET     32              // 96/3 = 120°
#define SINE_STARTUP_COMMUT     12              // komutacji blokowych przed sinus
#define SINE_STALL_FREEZE_MS    200             // ms bez Halla → zamrożenie kąta
#define SINE_CRAWL_SPEED_Q16    315             // minimalna prędkość startowa ≈ 1 obr.elekt./s (52428800/166666)
#define SINE_STALL_FALLBACK_MS  400             // minimalny timeout fallback (histereza, anty-szarpanie)
#define SINE_START_MAX_HALL_US  30000           // max okres Halla (min prędkość) do wejścia w SINUS
#define SINE_PHASE_CORR_SHIFT   2               // korekcja 1/4 błędu na przejście Halla
#define SINE_SPEED_FILTER_SHIFT 1               // filtr prędkości: 1/2 new + 1/2 old
#define SINE_SNAP_THRESHOLD     (24 << 16)       // błąd > 1/4 obrotu elektr. → pełny snap kąta
#define SINE_SAFE_MAX_DUTY      (PWM_MAX_DUTY * 75 / 100)  // SVPWM: liniowy do 58%, overmod do 75%, powyżej szkodliwe harmoniczne
#define SINE_MIN_AMPLITUDE      15              // poniżej tego coast (center-aligned 50% = hamowanie)

/// Debounce czujników Halla: minimalna przerwa między przejściami [us].
/// W trybie SINUS 6 FETów przekłądają jednocześnie (center-aligned PWM),
/// generując znacznie więcej EMI niż BLOCK (2 FETy). Szumy sprzegają się
/// w linie Halla i tworzą fałszywe przejścia (dt ~50us = 1 tick ISR).
/// Bez debounce: hall_period_us = 50us → speed_q16 = 1M → kąt ucieka → desync.
/// 200us = 4 ticki ISR, bezpieczne do ~50k eRPM (daleko poza realnym motorem).
#define HALL_MIN_PERIOD_US      200
#define DEFAULT_P07_STANDALONE  90              // domyślne P07 gdy brak wyświetlacza (6 × 15 par biegunów)

/// Limit duty regen — 80% max (musi zostać czas OFF na transfer energii do baterii)
#define REGEN_MAX_DUTY  (PWM_MAX_DUTY * 80 / 100)
/// Minimalne RPM poniżej którego regen jest nieefektywny (tylko grzeje)
#define REGEN_MIN_RPM   50
/// Napięcie odcięcia regen [V] — powyżej tego progu regen wyłączony (ochrona baterii)
#define VBAT_REGEN_CUTOFF  42.0f
/// Domyślne duty regen (50% — umiarkowane hamowanie)
#define REGEN_DEFAULT_DUTY  (PWM_MAX_DUTY / 2)

// ============================================================================
// Prototypy funkcji
// ============================================================================

void initGPIO();
void initPWM();
void initCommutationTimer();
void allMosfetsOff();
void readAnalogInputs();
void readHallSensors();
void readDigitalInputs();
void printDiagnostics();
void printDisplayConfig();
void blockCommutate(uint8_t hallState, uint16_t duty);
void processSerialCommands();
static void IRAM_ATTR regenCommutateISR(uint8_t hall, uint16_t regen_duty, motor_direction_t dir);
static void IRAM_ATTR sinusCommutateISR(uint8_t hall, uint16_t amplitude);
static void printSineDebug();
static String executeCommand(const String& cmd);
static String mosfetTestSet(const String& which);
static void mosfetTestPrintHelp();

// Dzielnik napięcia VBAT: 1M (góra) / 33k (dół)
static const float kVbatRTop = 1130000.0f;
static const float kVbatRBottom = 31700.0f;
static const float kVbatDividerGain = (kVbatRTop + kVbatRBottom) / kVbatRBottom;

// Pomiar prądu: shunt 2 mOhm + INA180A2 (gain 50 V/V)
static const float kShuntOhms = 0.002f;
static const float kInaGain = 50.0f;
static const float kCurrentScale = 1.0f / (kShuntOhms * kInaGain);
static const float kCurrentOffsetAlpha = 0.02f;  // filtr do autokalibracji zera
static float g_currentOffsetV[3] = {0.0f, 0.0f, 0.0f};

// Krok zmiany duty dla komend +/-
static const uint16_t DUTY_STEP = PWM_MAX_DUTY / 20;  // 5% kroku

// Przepustnica - próg martwej strefy i zakres
static const uint16_t THROTTLE_DEAD_ZONE = 400;
static const uint16_t THROTTLE_MIN_RAW   = 400;   // 0% duty
static const uint16_t THROTTLE_MAX_RAW   = 2600;  // 100% duty
static const float kThrottleFilterAlpha  = 0.15f;  ///< EMA α dla przepustnicy (filtr szumów ADC2)
                                                    //   α=0.15 @ 2kHz loop → τ≈3ms, 99% settling≈14ms
                                                    //   Szpilka 1870→0 → po filtrze: 1590 (>400 dead zone)

// Auto-status
static bool g_autoStatus = false;
static unsigned long g_lastAutoStatusMs = 0;
static const unsigned long AUTO_STATUS_INTERVAL_MS = 1000;
static bool g_debugSine = false;
static unsigned long g_lastDebugSineMs = 0;
static const unsigned long DEBUG_SINE_INTERVAL_MS = 200;

// Bufor na komendy numeryczne
static String serialBuffer = "";

// Symulacja hamulca komendą Serial
static bool g_brake_simulated = false;

// Rampa rozpędzania silnika
static uint16_t g_duty_ramped = 0;                   ///< Aktualny duty po rampie
static unsigned long g_ramp_last_us = 0;             ///< Timestamp ostatniego kroku rampy [µs]
static bool g_manual_duty_override = false;          ///< true = duty z komendy serial, manetka ignorowana

// Auto-tune fazy sinusoidalnej (komenda 'sat')
enum AutoTuneState : uint8_t {
    ATUNE_IDLE = 0,
    ATUNE_INIT,
    ATUNE_SETTLE,
    ATUNE_MEASURE,
    ATUNE_NEXT,
    ATUNE_DONE
};
static AutoTuneState g_atune_state = ATUNE_IDLE;
static int8_t   g_atune_offset_min   = -24;      ///< Początek zakresu sweep
static int8_t   g_atune_offset_max   = 24;       ///< Koniec zakresu sweep
static int8_t   g_atune_offset_step  = 2;        ///< Krok sweep (wpisy tabeli)
static int8_t   g_atune_current_ofs  = 0;        ///< Aktualnie testowany offset
static int8_t   g_atune_best_ofs     = 0;        ///< Najlepszy offset (min prąd)
static float    g_atune_best_current = 1e9f;     ///< Najniższy średni prąd [A]
static float    g_atune_sum_current  = 0.0f;     ///< Akumulator prądu w fazie MEASURE
static uint32_t g_atune_sample_count = 0;        ///< Liczba próbek w fazie MEASURE
static unsigned long g_atune_phase_start_ms = 0; ///< millis() startu aktualnej fazy
static int8_t   g_atune_saved_offset = 0;        ///< Zapisany offset przed auto-tune
static uint16_t g_atune_test_duty    = 0;        ///< Duty testowe (10% domyślnie)
static const unsigned long ATUNE_SETTLE_MS  = 400;  ///< Czas stabilizacji [ms]
static const unsigned long ATUNE_MEASURE_MS = 600;  ///< Czas pomiaru [ms]

// Wyświetlacz S866 (zawsze aktywny na Serial2)
static s866_display_t g_display;

// ============================================================================
// Algorytm przepustnicy — wspólny dla BLOCK / SINUS / FOC
// ============================================================================

/**
 * @brief Oblicza max duty na podstawie poziomu wspomagania (assist level).
 *
 * Algorytm wspólny dla wszystkich trybów sterowania (BLOCK, SINUS, FOC).
 * Przepustnica mapuje zakres RAW bezpośrednio na 0–maxDuty (proporcjonalnie).
 *
 * @return Maksymalne duty 0–PWM_MAX_DUTY:
 *   - Wyświetlacz podłączony, level>0: proporcjonalnie 20/40/60/80/100%
 *   - Wyświetlacz podłączony, level=0:  0 (silnik wyłączony)
 *   - Wyświetlacz nie podłączony:       PWM_MAX_DUTY (tryb standalone)
 */
static uint16_t getAssistMaxDuty() {
    if (!g_display.connected) {
        return PWM_MAX_DUTY;  // brak wyświetlacza → pełna moc (standalone)
    }
    if (g_display.rx.assist_level == 0) {
        return 0;  // assist level 0 → silnik wyłączony
    }
    // Raw: 3=20%, 6=40%, 9=60%, 12=80%, 15=100%
    uint16_t maxDuty = (uint16_t)((uint32_t)g_display.rx.assist_level * PWM_MAX_DUTY / 15);
    if (maxDuty > PWM_MAX_DUTY) maxDuty = PWM_MAX_DUTY;
    return maxDuty;
}

/**
 * @brief Mapuje wartość RAW przepustnicy na duty cycle 0–maxDuty.
 *
 * Pełen zakres przepustnicy (THROTTLE_MIN_RAW–THROTTLE_MAX_RAW) jest mapowany
 * proporcjonalnie na 0–maxDuty. Dzięki temu zmiana poziomu wspomagania
 * zmienia zakres wyjściowy, a nie obcina go (lepsza rozdzielczość sterowania).
 *
 * @param throttle_raw Surowa wartość ADC przepustnicy.
 * @param maxDuty      Maksymalne duty z getAssistMaxDuty().
 * @return duty 0–maxDuty
 */
static uint16_t mapThrottleToDuty(uint16_t throttle_raw, uint16_t maxDuty) {
    if (maxDuty == 0) return 0;
    if (throttle_raw < THROTTLE_DEAD_ZONE) return 0;

    uint16_t thr = throttle_raw;
    if (thr > THROTTLE_MAX_RAW) thr = THROTTLE_MAX_RAW;
    if (thr < THROTTLE_MIN_RAW) thr = THROTTLE_MIN_RAW;

    return (uint16_t)map(thr, THROTTLE_MIN_RAW, THROTTLE_MAX_RAW, 0, maxDuty);
}

// ============================================================================
// Algorytm PAS (Pedal Assist Sensor)
// ============================================================================

/**
 * @brief Oblicza kadencję pedałowania na podstawie impulsów z czujnika PAS.
 *
 * Kadencja [RPM] = 60 000 000 / (period_us × P13_magnets).
 * Timeout: jeśli > PAS_TIMEOUT_US od ostatniego impulsu → kadencja = 0.
 * Kierunek: jeśli pedałowanie do tyłu (g_pas_forward == false) → kadencja = 0.
 *
 * @return Kadencja w RPM (0 = nie pedałuje / timeout / pedałowanie wstecz)
 */
static uint16_t calculatePasCadence() {
    uint32_t period = g_pas_period_us;       // volatile → local copy
    uint32_t last   = g_pas_last_pulse_us;
    uint32_t now_us = (uint32_t)esp_timer_get_time();

    // Timeout: brak impulsu → nie pedałuje
    if (last == 0 || period == 0 || (now_us - last) > PAS_TIMEOUT_US) {
        return 0;
    }

    // Kierunek: pedałowanie wstecz → kadencja 0 (brak wspomagania)
    if (!g_pas_forward) {
        return 0;
    }

    // Liczba magnesów PAS (z wyświetlacza P13, domyślnie 12)
    uint8_t magnets = g_display.config.p13_pas_magnets;
    if (magnets == 0) magnets = 12;  // fallback

    // cadence_rpm = 60_000_000 / (period_us × magnets)
    uint32_t cadence = 60000000UL / (period * (uint32_t)magnets);
    if (cadence > PAS_MAX_CADENCE_RPM) cadence = PAS_MAX_CADENCE_RPM;

    return (uint16_t)cadence;
}

/**
 * @brief Oblicza prędkość koła [km/h] z wheeltime_ms i rozmiaru koła P06.
 *
 * Formuła: speed_kmh = (obwód_koła_m × 3600000) / (wheeltime_ms × 1000)
 * obwód = P06_inch_x10 / 10 × 0.0254 × π  [m]
 * Uproszczenie: speed_kmh = P06 × 0.028727 / wheeltime_ms  (stała = 0.0254×π×3600/10)
 *
 * @return Prędkość w km/h (0.0 jeśli stoi)
 */
static float calculateWheelSpeedKmh() {
    uint16_t wt_ms = g_bldc_state.wheeltime_ms;
    if (wt_ms == 0) return 0.0f;

    uint16_t p06 = g_display.config.p06_wheel_size_x10;
    if (p06 == 0) p06 = 260;  // fallback: 26" koło

    // Obwód koła [m] = (P06/10) × 0.0254 × π = P06 × 0.007980
    // v [km/h] = obwód [m] × 3600000 / (wt_ms × 1000) = P06 × 28.727 / wt_ms
    return (float)p06 * 28.727f / (float)wt_ms;
}

/**
 * @brief Oblicza duty PAS na podstawie kadencji, prędkości koła i wspomagania.
 *
 * ## Algorytm: kadencja → prędkość docelowa → duty feedback
 *
 * Problem: stary algorytm dawał pełne duty gdy kadencja > próg, więc koło
 * zawsze rozpędzało się do max niezależnie od szybkości kręcenia pedałami.
 *
 * Nowy schemat ("Cadence PAS" z ebikes.ca):
 *
 * 1. **Kadencja → prędkość docelowa** (target speed):
 *    target_kmh = (cadence / PAS_CADENCE_MAX_RPM) × speed_limit
 *    Wolne pedałowanie → niska prędkość docelowa.
 *    Szybkie pedałowanie → wyższa prędkość docelowa.
 *    P11 (czułość) przesuwa krzywe — wyższa czułość = więcej prędkości
 *    przy mniejszej kadencji.
 *
 * 2. **Speed feedback loop** (duty modulowane prędkością koła):
 *    - speed < target: duty = maxDuty (pełne przyspieszenie)
 *    - speed w strefie 80-100% target: liniowy spadek duty
 *    - speed >= target: duty = 0 (utrzymanie prędkości naturalnie)
 *    Efekt: koło przyspiesza do prędkości docelowej i stabilizuje się.
 *
 * 3. **Start Strength (P12)**: minimalne duty przy bardzo niskiej kadencji.
 *
 * @param maxDuty  Maksymalne duty z poziomu wspomagania (getAssistMaxDuty)
 * @return duty PAS 0–maxDuty
 */
static uint16_t calculatePasDuty(uint16_t maxDuty) {
    if (maxDuty == 0) return 0;

    uint16_t cadence = g_bldc_state.pas_cadence_rpm;
    if (cadence == 0) return 0;

    // --- 1. Prędkość docelowa z kadencji ---
    // P11: czułość PAS 1-24 (domyślnie 12)
    // Wyższa czułość = niższa kadencja potrzebna do osiągnięcia danej prędkości
    uint8_t p11 = g_display.config.p11_pas_sensitivity;
    if (p11 == 0) p11 = 12;
    if (p11 > 24) p11 = 24;

    // Kadencja dla pełnej prędkości (RPM):
    //   P11=1 (niska czułość)  → 100 RPM (trzeba bardzo szybko kręcić)
    //   P11=12 (domyślna)      → 62 RPM
    //   P11=24 (wysoka czułość) → 25 RPM (lekkie kręcenie = pełna prędkość)
    uint16_t cadence_for_full = (uint16_t)map((long)p11, 1, 24, 100, 25);
    if (cadence_for_full < 15) cadence_for_full = 15;

    // Limit prędkości [km/h] (P08, fallback 25 km/h)
    uint8_t speed_limit_kmh = g_display.config.p08_speed_limit;
    if (speed_limit_kmh == 0) speed_limit_kmh = 25;

    // Target speed = (cadence / cadence_for_full) × speed_limit
    // Nie ograniczamy do 1.0 — jeśli ktoś kręci szybciej niż cadence_for_full,
    // target może przekroczyć speed_limit, ale speed feedback i tak uzetnie.
    float cadence_ratio = (float)cadence / (float)cadence_for_full;
    if (cadence_ratio > 1.0f) cadence_ratio = 1.0f;
    float target_kmh = cadence_ratio * (float)speed_limit_kmh;
    g_bldc_state.pas_target_kmh = target_kmh;

    // --- 2. Speed feedback: modulacja duty na podstawie aktualnej prędkości ---
    float speed_kmh = g_bldc_state.wheel_speed_kmh;
    float speed_factor;

    if (target_kmh <= 0.5f) {
        // Bardzo niska prędkość docelowa — użyj liniowego skalowania kadencji
        speed_factor = cadence_ratio;
    } else if (speed_kmh >= target_kmh) {
        // Osiągnięta lub przekroczona prędkość docelowa → wyłącz moc
        speed_factor = 0.0f;
    } else {
        // Strefa ramp-down: 80% do 100% prędkości docelowej
        float ramp_start = target_kmh * 0.8f;
        if (speed_kmh <= ramp_start) {
            speed_factor = 1.0f;  // pełne przyspieszenie
        } else {
            // Liniowy spadek od 1.0 (80% target) do 0.0 (100% target)
            speed_factor = (target_kmh - speed_kmh) / (target_kmh - ramp_start);
        }
    }

    // --- 3. Oblicz duty PAS ---
    float pas_duty_f = speed_factor * (float)maxDuty;

    // --- 4. Start Strength (P12) — minimalne duty na starcie ---
    // P12=0 → 0%, P12=1 → 4%, ..., P12=5 → 20% maxDuty
    // Aktywne tylko przy niskiej prędkości (start z miejsca)
    uint8_t p12 = g_display.config.p12_pas_start_strength;
    if (p12 > 5) p12 = 5;
    if (p12 > 0 && cadence > 0 && speed_kmh < 5.0f) {
        float min_duty_f = (float)p12 * (float)maxDuty / 25.0f;
        if (pas_duty_f < min_duty_f) {
            pas_duty_f = min_duty_f;
        }
    }

    uint16_t result = (uint16_t)pas_duty_f;
    if (result > maxDuty) result = maxDuty;
    return result;
}

// ============================================================================
// ISR czujnika prędkości (GPIO, pin SPEED)
// ============================================================================

/**
 * @brief ISR przerwania GPIO na pinie SPEED (FALLING edge).
 *
 * Dla silników przekładniowych (P07==1): jeden magnes na kole generuje
 * jeden impuls na obrót. Mierzymy czas między impulsami.
 *
 * @note Używany tylko gdy P07 <= 1 (czujnik zewnętrzny).
 */
void IRAM_ATTR onSpeedPulse() {
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (g_speed_last_pulse_us > 0) {
        g_speed_period_us = now_us - g_speed_last_pulse_us;
    }
    g_speed_last_pulse_us = now_us;
}

/**
 * @brief ISR przerwania GPIO na pinie PAS (oba zbocza — CHANGE).
 *
 * Detekcja kierunku pedałowania:
 * Większość czujników PAS ma asymetryczny dysk magnesów — magnesy N i S
 * mają różną szerokość. Efekt: sygnał z Halla ma różne czasy HIGH i LOW.
 * Przy pedałowaniu DO PRZODU: HIGH > LOW (lub odwrotnie, zależy od montażu).
 * Przy pedałowaniu DO TYŁU: proporcje się odwracają.
 *
 * Mierzymy oba półokresy (HIGH time i LOW time) i porównujemy:
 *   HIGH > LOW → forward (pedałowanie do przodu)
 *   LOW > HIGH → backward (wstecz)
 *
 * Jeśli magnesy są symetryczne (różnica < PAS_DIR_MIN_ASYMMETRY%),
 * zakładamy forward — nie można jednoznacznie określić kierunku.
 */
void IRAM_ATTR onPasPulse() {
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    bool pin_high = (GPIO.in >> PIN_PAS) & 1;  // szybki odczyt GPIO

    if (pin_high) {
        // RISING edge — koniec okresu LOW
        if (g_pas_falling_us > 0) {
            uint32_t low_time = now_us - g_pas_falling_us;
            if (low_time >= PAS_MIN_HALFPERIOD_US) {
                g_pas_low_time_us = low_time;
            }
        }
        g_pas_rising_us = now_us;
    } else {
        // FALLING edge — koniec okresu HIGH
        if (g_pas_rising_us > 0) {
            uint32_t high_time = now_us - g_pas_rising_us;
            if (high_time >= PAS_MIN_HALFPERIOD_US) {
                g_pas_high_time_us = high_time;
            }
        }
        g_pas_falling_us = now_us;

        // Oba półokresy zmierzone → oblicz okres i kierunek
        if (g_pas_high_time_us > 0 && g_pas_low_time_us > 0) {
            uint32_t period = g_pas_high_time_us + g_pas_low_time_us;
            g_pas_period_us = period;

            // Detekcja kierunku: porównaj HIGH vs LOW time
            // Asymetria musi przekraczać próg (symetryczne magnesy → forward)
            uint32_t diff = (g_pas_high_time_us > g_pas_low_time_us)
                          ? (g_pas_high_time_us - g_pas_low_time_us)
                          : (g_pas_low_time_us - g_pas_high_time_us);
            uint32_t threshold = period * PAS_DIR_MIN_ASYMMETRY / 100;
            if (diff > threshold) {
                g_pas_forward = (g_pas_high_time_us > g_pas_low_time_us);
            }
            // Jeśli diff <= threshold: magnesy symetryczne, nie zmieniaj g_pas_forward
        }
    }
    g_pas_last_pulse_us = now_us;
}

// ============================================================================
// Setup
// ============================================================================

/**
 * @brief Inicjalizacja systemu — wywoływana raz przy starcie.
 *
 * Kolejność inicjalizacji ma znaczenie:
 * 1. GPIO muszą być skonfigurowane przed PWM (LEDC attaches to pin)
 * 2. allMosfetsOff() bezpieczny stan przed uruchomieniem timera ISR
 * 3. Timer uruchamiany jako ostatni — od tego momentu ISR działa
 */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("==========================================");
    Serial.println("  BLDC Motor Driver - ESP32");
    Serial.println("  Wersja: 0.2.0");
    Serial.println("==========================================");
    Serial.println();

    // Konfiguracja z NVS (EEPROM) — musi być PRZED użyciem parametrów
    config_init();
    controller_config_t& cfg = config_get();

    // Inicjalizacja stanu
    memset(&g_bldc_state, 0, sizeof(bldc_state_t));
    g_bldc_state.mode = DRIVE_MODE_DISABLED;  // tymczasowo — tryb docelowy ustawiony po init HW
    g_bldc_state.ramp_time_ms = cfg.ramp_time_ms;
    g_bldc_state.regen_enabled = (cfg.regen_enabled != 0);

    // Inicjalizacja GPIO
    initGPIO();
    Serial.println("[OK] GPIO zainicjalizowane");

    // Przerwanie na pinie SPEED (czujnik zewnętrzny — aktywne przy P07<=1)
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED), onSpeedPulse, FALLING);

    // PAS (Pedal Assist Sensor) — przerwanie na obu zboczach (detekcja kierunku)
    attachInterrupt(digitalPinToInterrupt(PIN_PAS), onPasPulse, CHANGE);
    Serial.println("[OK] Przerwanie SPEED (GPIO21) gotowe");

    // Inicjalizacja PWM
    initPWM();
    Serial.println("[OK] PWM zainicjalizowane");

    // Upewnienie się, że wszystkie MOSFETy są wyłączone
    allMosfetsOff();
    Serial.println("[OK] Wszystkie MOSFETy wyłączone (stan bezpieczny)");

    // Timer sprzętowy do komutacji (co 50 us = 20 kHz)
    initCommutationTimer();
    Serial.println("[OK] Timer komutacji uruchomiony (20 kHz)");
    Serial.printf("[OK] Rampa rozpędzania: %d ms (0→100%%)\n", g_bldc_state.ramp_time_ms);

    // Inicjalizacja wyświetlacza S866 na Serial2 (GPIO4/GPIO16)
    memset(&g_display, 0, sizeof(g_display));
    g_display.last_valid_ms = millis();
    s866_init();
    Serial.println("[OK] Wyświetlacz S866 uruchomiony (Serial2: GPIO4/GPIO16, 9600 baud)");

    // Automatyczne włączenie trybu jazdy z konfiguracji NVS
    {
        drive_mode_t boot_mode = (drive_mode_t)cfg.drive_mode;
        if (boot_mode >= DRIVE_MODE_BLOCK && boot_mode <= DRIVE_MODE_FOC) {
            g_bldc_state.mode = boot_mode;
            g_bldc_state.fault = false;
            const char* modeNames[] = {"OFF", "BLOCK", "SINUS", "FOC"};
            Serial.printf("[OK] Tryb jazdy z NVS: %s\n", modeNames[boot_mode]);
        } else {
            Serial.println("[OK] Tryb jazdy: DISABLED (nieprawidłowy w NVS)");
        }
    }

    Serial.printf("[OK] Rampa: %d ms | Regen: %s\n",
                  cfg.ramp_time_ms,
                  cfg.regen_enabled ? "ON" : "OFF");

    Serial.println();
    Serial.println("Komendy Serial: h=pomoc");
    Serial.println("==========================================");
    Serial.println();
}

// ============================================================================
// Auto-tune fazy sinusoidalnej
// ============================================================================

/**
 * @brief Maszyna stanów auto-strojenia offsetu fazy sinusoidalnej.
 *
 * Algorytm: przy stałym niskim duty (10%) przelatuje zakres offsetów
 * g_sine_hall_phase_offset od -24 do +24 (co 2 wpisy = 7.5° elektr.).
 * Na każdym kroku: 400ms stabilizacja + 600ms pomiar średniego prądu.
 * Optymalny offset = minimum prądu (najlepsza sprawność, najmniej strat).
 *
 * Uruchamiany komendą 'sat'. Wymaga trybu SINUS i działającego silnika.
 * Podczas strojenia przepustnica jest ignorowana (g_manual_duty_override).
 *
 * Całkowity czas: ~25 kroków × 1s = ~25 sekund.
 */
static void autoTuneStep() {
    if (g_atune_state == ATUNE_IDLE) return;

    unsigned long now = millis();

    switch (g_atune_state) {
    case ATUNE_INIT: {
        // Zapisz stan, ustaw stałe duty testowe
        g_atune_saved_offset = g_sine_hall_phase_offset;
        g_atune_current_ofs = g_atune_offset_min;
        g_atune_best_ofs = 0;
        g_atune_best_current = 1e9f;

        // Ustaw duty testowe (10%) i override manetki
        g_atune_test_duty = (uint16_t)((uint32_t)PWM_MAX_DUTY * 10 / 100);
        g_manual_duty_override = true;
        g_bldc_state.duty_target = g_atune_test_duty;
        g_duty_ramped = g_atune_test_duty;

        // Ustaw pierwszy offset
        g_sine_hall_phase_offset = g_atune_current_ofs;

        Serial.println("[SAT] Auto-tune start: offset " + String((int)g_atune_offset_min)
                       + ".." + String((int)g_atune_offset_max)
                       + " krok " + String((int)g_atune_offset_step)
                       + ", duty=10%");
        Serial.println("[SAT]  ofs |  avg_I [A] | *=best");

        g_atune_phase_start_ms = now;
        g_atune_state = ATUNE_SETTLE;
        break;
    }

    case ATUNE_SETTLE: {
        // Czekaj na stabilizację prądu po zmianie offsetu
        if (now - g_atune_phase_start_ms >= ATUNE_SETTLE_MS) {
            g_atune_sum_current = 0.0f;
            g_atune_sample_count = 0;
            g_atune_phase_start_ms = now;
            g_atune_state = ATUNE_MEASURE;
        }
        break;
    }

    case ATUNE_MEASURE: {
        // Akumuluj próbki prądu (suma 3 faz)
        float i_sum = g_bldc_state.phase_current[0]
                    + g_bldc_state.phase_current[1]
                    + g_bldc_state.phase_current[2];
        g_atune_sum_current += i_sum;
        g_atune_sample_count++;

        if (now - g_atune_phase_start_ms >= ATUNE_MEASURE_MS) {
            // Oblicz średni prąd
            float avg = (g_atune_sample_count > 0)
                        ? (g_atune_sum_current / (float)g_atune_sample_count)
                        : 999.0f;

            bool is_best = (avg < g_atune_best_current);
            if (is_best) {
                g_atune_best_current = avg;
                g_atune_best_ofs = g_atune_current_ofs;
            }

            // Wypisz wynik kroku
            char buf[64];
            snprintf(buf, sizeof(buf), "[SAT] %+4d | %6.3f A   %s",
                     (int)g_atune_current_ofs, avg, is_best ? "*" : "");
            Serial.println(buf);

            g_atune_state = ATUNE_NEXT;
        }
        break;
    }

    case ATUNE_NEXT: {
        // Przejdź do następnego offsetu lub zakończ
        int next = (int)g_atune_current_ofs + (int)g_atune_offset_step;
        if (next > (int)g_atune_offset_max) {
            g_atune_state = ATUNE_DONE;
        } else {
            g_atune_current_ofs = (int8_t)next;
            g_sine_hall_phase_offset = g_atune_current_ofs;
            // Utrzymaj stałe duty testowe
            g_bldc_state.duty_target = g_atune_test_duty;
            g_duty_ramped = g_atune_test_duty;
            g_atune_phase_start_ms = millis();
            g_atune_state = ATUNE_SETTLE;
        }
        break;
    }

    case ATUNE_DONE: {
        // Zastosuj najlepszy offset
        g_sine_hall_phase_offset = g_atune_best_ofs;

        Serial.println("[SAT] ==============================");
        Serial.println("[SAT] Najlepszy offset: " + String((int)g_atune_best_ofs)
                       + " (" + String((float)g_atune_best_ofs * 3.75f, 1) + "°)"
                       + "  avg_I=" + String(g_atune_best_current, 3) + " A");
        Serial.println("[SAT] Poprzedni offset: " + String((int)g_atune_saved_offset)
                       + " (" + String((float)g_atune_saved_offset * 3.75f, 1) + "°)");
        Serial.println("[SAT] Auto-tune zakończony. Offset zastosowany.");
        Serial.println("[SAT] Użyj so/so+/so- do ręcznej korekty.");

        // Przywróć duty — manetka przejmie kontrolę
        g_manual_duty_override = false;
        g_atune_state = ATUNE_IDLE;
        break;
    }

    default:
        g_atune_state = ATUNE_IDLE;
        break;
    }
}

// ============================================================================
// Loop
// ============================================================================

/**
 * @brief Główna pętla aplikacji — wykonywana ciągle, ~kilka kHz.
 *
 * Odpowiada za wolne operacje (ADC, Serial, diagnostyka).
 * NIE wykonuje komutacji — robi to ISR onCommutationTimer().
 *
 * Przepływ:
 * 1. Odczyt ADC (napięcie, prądy, przepustnica, temp)
 * 2. Odczyt Halli i wejść cyfrowych (hamulec, PAS)
 * 3. Mapowanie przepustnicy → duty (wszystkie aktywne tryby sterowania)
 * 3a. Rampa rozpędzania (duty_cycle narasta płynnie w kierunku duty_target)
 * 3b. Hamulec aktywny → zerowanie rampy (płynny rozruch po puszczeniu)
 * 4. Zapis stanu do zmiennych volatile (dla ISR, w tym g_mode_isr)
 * 5. Obsługa komend Serial
 * 6. Auto-status (jeśli włączony)
 *
 * @note Zapis do g_duty_isr / g_motor_enabled nie jest atomowy na ESP32,
 * ale przy 32-bitowych typach i braku zależności kolejności zapis jest
 * wystarczająco bezpieczny dla tej aplikacji. Przy FOC użyć portENTER_CRITICAL.
 */
void loop() {
    // Odczyt wejść (wolna ścieżka)
    readAnalogInputs();
    readHallSensors();
    readDigitalInputs();

    // Przepustnica sprzętowa + PAS → duty target
    // Algorytm wspólny dla BLOCK / SINUS / FOC:
    //   maxDuty = f(assist_level)    — zakres wyjściowy zależy od poziomu
    //   throttle_duty = map(throttle, 0, maxDuty)
    //   pas_duty = f(kadencja, P11_czułość, P12_start, P08_speed_limit)
    //
    // Kombinacja PAS + Throttle zależy od P10 (drive mode z wyświetlacza):
    //   P10=0: PAS + gaz → duty = max(throttle_duty, pas_duty)
    //   P10=1: tylko gaz → duty = throttle_duty
    //   P10=2: tylko PAS → duty = pas_duty
    if (g_bldc_state.mode != DRIVE_MODE_DISABLED && !g_manual_duty_override) {
        uint16_t maxDuty = getAssistMaxDuty();

        // --- Oblicz kadencję PAS i prędkość koła ---
        g_bldc_state.pas_cadence_rpm = calculatePasCadence();
        g_bldc_state.wheel_speed_kmh = calculateWheelSpeedKmh();

        // --- Throttle duty ---
        uint16_t throttle_duty = mapThrottleToDuty(g_bldc_state.throttle_raw, maxDuty);

        // --- PAS duty ---
        uint16_t pas_duty = calculatePasDuty(maxDuty);
        g_bldc_state.pas_duty = pas_duty;

        // --- Kombinacja P10 ---
        uint8_t p10 = g_display.config.p10_drive_mode;
        switch (p10) {
            case 1:  // Tylko gaz (throttle only)
                g_bldc_state.duty_target = throttle_duty;
                break;
            case 2:  // Tylko PAS (pedal assist only)
                g_bldc_state.duty_target = pas_duty;
                break;
            default: // 0 lub nierozpoznany: PAS + gaz (wyższy wygrywa)
                g_bldc_state.duty_target = (throttle_duty > pas_duty) ? throttle_duty : pas_duty;
                break;
        }
    }

    // Rampa rozpędzania: duty_cycle narasta płynnie w kierunku duty_target
    // Czas rampy = ramp_time_ms dla przejścia 0→100%.
    // Spadek (hamowanie przepustnicą) jest natychmiastowy — rampa działa tylko w górę.
    {
        uint16_t target = g_bldc_state.duty_target;
        unsigned long now_us = micros();
        unsigned long dt_us = now_us - g_ramp_last_us;
        g_ramp_last_us = now_us;

        if (g_bldc_state.brake_active) {
            // Hamulec → zeruj rampę (po puszczeniu hamulca silnik startuje płynnie od 0)
            g_duty_ramped = 0;
        } else if (target <= g_duty_ramped) {
            // Spadek — natychmiastowy (bezpieczeństwo: szybkie zwolnienie gazu)
            g_duty_ramped = target;
        } else if (g_bldc_state.ramp_time_ms > 0) {
            // Wzrost — ograniczony rampą
            // max_step = PWM_MAX_DUTY × dt_us / (ramp_time_ms × 1000)
            uint32_t max_step = (uint32_t)PWM_MAX_DUTY * dt_us
                                / ((uint32_t)g_bldc_state.ramp_time_ms * 1000UL);
            if (max_step < 1) max_step = 1;
            uint16_t diff = target - g_duty_ramped;
            if (diff > max_step) {
                g_duty_ramped += (uint16_t)max_step;
            } else {
                g_duty_ramped = target;
            }
        } else {
            // ramp_time_ms == 0 → rampa wyłączona
            g_duty_ramped = target;
        }
        g_bldc_state.duty_cycle = g_duty_ramped;
    }

    // Aktualizacja zmiennych ISR z głównego stanu
    g_hall_isr = g_bldc_state.hall_state;
    g_duty_isr = g_bldc_state.duty_cycle;
    g_motor_enabled = (g_bldc_state.mode != DRIVE_MODE_DISABLED) && (g_bldc_state.duty_cycle > 0);
    g_brake_isr = g_bldc_state.brake_active;
    g_direction_isr = g_bldc_state.direction;
    g_mode_isr = g_bldc_state.mode;

    // Prędkość sinusoidalna (speed_q16) jest teraz obliczana bezpośrednio
    // w ISR (onCommutationTimer) na przejściu Halla — bez opóźnienia loop().
    // Tu nie ma nic do roboty — zostawione jako komentarz dla czytelności.

    // Obliczanie mocy: P = Vbat × max(Ia, Ib, Ic)
    {
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        float power = g_bldc_state.battery_voltage * maxI;

        if (g_bldc_state.regen_active) {
            // W trybie regen: prąd płynie do baterii → moc ujemna (oddawana)
            g_bldc_state.regen_power_watts = power;
            g_bldc_state.power_watts = 0.0f;
        } else if (g_motor_enabled) {
            // W trybie motoring: moc pobierana z baterii
            g_bldc_state.power_watts = power;
            g_bldc_state.regen_power_watts = 0.0f;
        } else {
            g_bldc_state.power_watts = 0.0f;
            g_bldc_state.regen_power_watts = 0.0f;
        }
    }

    // Regeneracja — logika aktywacji (hamulec + regen_enabled + warunki bezpieczeństwa)
    if (g_bldc_state.brake_active && g_bldc_state.regen_enabled) {
        bool vbat_ok = g_bldc_state.battery_voltage < VBAT_REGEN_CUTOFF;
        bool rpm_ok = g_bldc_state.rpm > REGEN_MIN_RPM;

        if (vbat_ok && rpm_ok) {
            g_bldc_state.regen_active = true;
            uint16_t regen_d = REGEN_DEFAULT_DUTY;
            if (regen_d > REGEN_MAX_DUTY) regen_d = REGEN_MAX_DUTY;
            g_regen_active_isr = true;
            g_regen_duty_isr = regen_d;
        } else {
            // Warunki niespełnione — wyłącz regen (coast)
            g_bldc_state.regen_active = false;
            g_regen_active_isr = false;
            g_regen_duty_isr = 0;
        }
    } else {
        g_bldc_state.regen_active = false;
        g_regen_active_isr = false;
        g_regen_duty_isr = 0;
    }

    // Obsługa wyświetlacza S866 (Serial2 — zawsze aktywny)
    {
        // Aktualizuj dane TX dla wyświetlacza
        g_display.tx.error = g_bldc_state.fault ? 1 : 0;
        g_display.tx.brake_active = g_bldc_state.brake_active ? 1 : 0;

        // Prąd: maksimum z 3 faz, w jednostkach 0.1A
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        g_display.tx.current_x10 = (uint16_t)(maxI * 10.0f);

        // Wheeltime [ms] — źródło zależy od P07:
        //   P07 > 1  → silnik direct-drive, P07 = liczba impulsów Halla na obrót koła
        //              (= 6 transitions/erev × pole_pairs, np. 6×15=90)
        //              wheeltime = hall_period_us × P07 / 1000
        //              NIE mnożymy dodatkowo ×6, bo P07 już to zawiera!
        //   P07 == 1 → silnik przekładniowy, użyj zewnętrznego czujnika SPEED
        //              wheeltime = speed_period_us / 1000 (1 impuls na obrót)
        //   P07 == 0 → brak konfiguracji, użyj Halli z domyślnym P07=1 (SPEED)
        uint8_t p07 = g_display.config.p07_speed_magnets;
        uint32_t wt_us = 0;

        // Gdy brak wyświetlacza i p07==0 → fallback na Halle z domyślnym P07
        if (p07 == 0 && !g_display.connected) {
            p07 = DEFAULT_P07_STANDALONE;
        }

        if (p07 <= 1) {
            // P07==1: czujnik zewnętrzny SPEED (1 magnes na koło)
            uint32_t sp = g_speed_period_us;  // volatile → local copy
            uint32_t last = g_speed_last_pulse_us;
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            // Timeout: jeśli >3s od ostatniego impulsu → koło stoi
            if (sp > 0 && sp < 10000000 && last > 0 && (now_us - last) < 3000000) {
                wt_us = sp;  // 1 impuls = 1 obrót koła
            }
        } else {
            // P07 > 1: direct-drive hub, P07 = hall transitions per wheel revolution
            // P07 = 6 × pole_pairs (np. 90 = 6×15 par biegunów)
            uint32_t hp = g_hall_period_us;  // volatile → local copy
            uint32_t last = g_hall_last_change_us;
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            // Timeout: jeśli >2s od ostatniego przejścia Halla → silnik stoi
            if (hp > 0 && hp < 2000000 && last > 0 && (now_us - last) < 2000000) {
                wt_us = (uint32_t)hp * (uint32_t)p07;  // bez ×6! P07 już zawiera 6×pole_pairs
            }
        }

        if (wt_us > 0) {
            uint32_t wt_ms = wt_us / 1000UL;
            if (wt_ms > 65000) wt_ms = 65000;
            if (wt_ms == 0) wt_ms = 1;  // minimum 1 ms
            g_display.tx.wheeltime_ms = (uint16_t)wt_ms;
            g_bldc_state.wheeltime_ms = (uint16_t)wt_ms;
            g_bldc_state.rpm = (wt_ms > 0) ? (60000UL / wt_ms) : 0;
        } else {
            g_display.tx.wheeltime_ms = 0;
            g_bldc_state.wheeltime_ms = 0;
            g_bldc_state.rpm = 0;
        }

        // Obsługa protokołu wyswietlacza
        s866_service(&g_display);
    }

    // Obsługa komend USB Serial (zawsze aktywna)
    processSerialCommands();

    // Auto-status co 1s
    if (g_autoStatus && (millis() - g_lastAutoStatusMs >= AUTO_STATUS_INTERVAL_MS)) {
        g_lastAutoStatusMs = millis();
        printDiagnostics();
    }

    // Debug sinus co 200ms (krok po kroku)
    if (g_debugSine && (millis() - g_lastDebugSineMs >= DEBUG_SINE_INTERVAL_MS)) {
        g_lastDebugSineMs = millis();
        printSineDebug();
    }

    // Auto-tune fazy sinusoidalnej (maszyna stanów)
    autoTuneStep();
}

// ============================================================================
// Inicjalizacja GPIO
// ============================================================================

/**
 * @brief Konfiguracja wszystkich pinów GPIO.
 *
 * Piny PWM (mostki) są ustawiane w bezpieczny stan (wszystkie tranzystory OFF)
 * PRZED przełączeniem trybu na OUTPUT — zapobiega to impulsowi przy starcie.
 *
 * @warning GPIO12 (PIN_FET_TEMP) jest pinem STRAP ESP32.
 * Pull-up na GPIO12 during boot przestawia VDD_SDIO na 1.8V → brak uploadu do flash.
 * Na PCB GPIO12 nie powinien mieć pull-up; tu nie konfigurujemy go jako OUTPUT.
 *
 * @warning GPIO0 (PIN_EXT_1) jest pinem BOOT. LOW przy resecie = tryb programowania.
 * Używać ostrożnie.
 */
void initGPIO() {
    // --- Wyjścia PWM (sterowanie IR2103) ---
    // Najpierw ustawiamy bezpieczny stan, potem tryb OUTPUT
    
    // Faza A
    digitalWrite(PIN_PWM_A_HIGH, IR2103_HIN_OFF);   // High-side OFF
    digitalWrite(PIN_PWM_A_LOW, IR2103_LIN_OFF);     // Low-side OFF (LIN=HIGH bo odwrócone)
    pinMode(PIN_PWM_A_HIGH, OUTPUT);
    pinMode(PIN_PWM_A_LOW, OUTPUT);

    // Faza B
    digitalWrite(PIN_PWM_B_HIGH, IR2103_HIN_OFF);
    digitalWrite(PIN_PWM_B_LOW, IR2103_LIN_OFF);
    pinMode(PIN_PWM_B_HIGH, OUTPUT);
    pinMode(PIN_PWM_B_LOW, OUTPUT);

    // Faza C
    digitalWrite(PIN_PWM_C_HIGH, IR2103_HIN_OFF);
    digitalWrite(PIN_PWM_C_LOW, IR2103_LIN_OFF);
    pinMode(PIN_PWM_C_HIGH, OUTPUT);
    pinMode(PIN_PWM_C_LOW, OUTPUT);

    // --- Wejścia analogowe (ADC) ---
    // GPIO 34, 35, 36 - tylko wejście (input-only), nie wymagają pinMode
    // ale ustawiamy dla czytelności
    pinMode(PIN_BATTERY_VOLTAGE, INPUT);
    pinMode(PIN_PHASE_B_CURRENT, INPUT);
    pinMode(PIN_PHASE_C_CURRENT, INPUT);

    // --- Czujniki temperatury ---
    pinMode(PIN_MOTOR_TEMP, INPUT);
    // PIN_FET_TEMP (GPIO12)

    // --- Przepustnica ---
    pinMode(PIN_THROTTLE, INPUT);

    // --- Czujniki Halla ---
    pinMode(PIN_HALL_SENSOR_A, INPUT_PULLUP);
    pinMode(PIN_HALL_SENSOR_B, INPUT_PULLUP);
    pinMode(PIN_HALL_SENSOR_C, INPUT_PULLUP);

    // --- PAS ---
    pinMode(PIN_PAS, INPUT_PULLUP);

    // --- Hamulec ---
    pinMode(PIN_BRAKE, INPUT_PULLUP);

    // --- Prędkość (wejście czujnika zewnętrznego — aktywne przy P07==1) ---
    pinMode(PIN_SPEED, INPUT_PULLUP);

    // --- UART Enable ---
    pinMode(PIN_UART_EN, OUTPUT);
    digitalWrite(PIN_UART_EN, LOW);

    // --- Rozszerzenia ---
    // Domyślnie jako wejścia
    // GPIO0 - uwaga: boot pin!
    pinMode(PIN_EXT_1, INPUT_PULLUP);
    pinMode(PIN_EXT_2, INPUT);
    pinMode(PIN_EXT_3, INPUT);
}

// ============================================================================
// Inicjalizacja PWM (LEDC)
// ============================================================================

/**
 * @brief Konfiguracja 6 kanałów LEDC dla sterowania mostkami IR2103.
 *
 * Każda z 3 faz (A, B, C) ma dwa kanały LEDC:
 * - HIGH: steruje wejściem HIN (high-side) — duty=0 = OFF, duty=d = PWM
 * - LOW:  steruje wejściem LIN (low-side) — LOGIKA ODWRÓCONA!
 *   - duty=PWM_MAX_DUTY → LIN=HIGH → low-side OFF (bezpieczny stan domyślny)
 *   - duty=0            → LIN=LOW  → low-side ON (przewodzi prąd do GND)
 *
 * Parametry PWM:
 * - Częstotliwość: PWM_FREQUENCY = 20 kHz (powyżej słyszalności)
 * - Rozdzielczość: PWM_RESOLUTION = 10 bit (wartości 0–1023)
 *
 * @note Po initPWM() wszystkie kanały LOW mają duty=PWM_MAX_DUTY (stan OFF).
 * allMosfetsOff() wywołuje to samo, ale jest idempotentna.
 */
void initPWM() {
    // Konfiguracja kanałów LEDC dla sterowania PWM mostków
    // Na razie tylko konfiguracja - PWM nieaktywne (duty=0)
    
    // Faza A - High-side
    ledcSetup(PWM_CHANNEL_A_HIGH, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_A_HIGH, PWM_CHANNEL_A_HIGH);
    ledcWrite(PWM_CHANNEL_A_HIGH, 0);

    // Faza A - Low-side (pamiętaj: IR2103 LIN jest odwrócony!)
    // Duty=PWM_MAX_DUTY oznacza LIN=HIGH czyli low-side OFF
    ledcSetup(PWM_CHANNEL_A_LOW, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_A_LOW, PWM_CHANNEL_A_LOW);
    ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);  // LIN=HIGH -> low-side OFF

    // Faza B - High-side
    ledcSetup(PWM_CHANNEL_B_HIGH, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_B_HIGH, PWM_CHANNEL_B_HIGH);
    ledcWrite(PWM_CHANNEL_B_HIGH, 0);

    // Faza B - Low-side
    ledcSetup(PWM_CHANNEL_B_LOW, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_B_LOW, PWM_CHANNEL_B_LOW);
    ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);

    // Faza C - High-side
    ledcSetup(PWM_CHANNEL_C_HIGH, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_C_HIGH, PWM_CHANNEL_C_HIGH);
    ledcWrite(PWM_CHANNEL_C_HIGH, 0);

    // Faza C - Low-side
    ledcSetup(PWM_CHANNEL_C_LOW, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_C_LOW, PWM_CHANNEL_C_LOW);
    ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
}

// ============================================================================
// Wyłączenie wszystkich MOSFETów (stan bezpieczny)
// ============================================================================

/**
 * @brief Przełącza wszystkie tranzystory w stan OFF (stan bezpieczny).
 *
 * IR2103: high-side OFF = HIN=LOW (duty=0),
 *         low-side OFF  = LIN=HIGH (duty=PWM_MAX_DUTY, bo logika odwrócona).
 *
 * Wywoływana:
 * - po initPWM() podczas startu
 * - przy komendzie 'd' (disable)
 * - przy hamulcu w ISR
 * - przy błędnym stanie Halla (0 lub 7)
 *
 * @note Bezpieczna do wywołania z loop() i z ISR.
 */
void allMosfetsOff() {
    // High-side OFF: HIN=LOW (duty=0)
    ledcWrite(PWM_CHANNEL_A_HIGH, 0);
    ledcWrite(PWM_CHANNEL_B_HIGH, 0);
    ledcWrite(PWM_CHANNEL_C_HIGH, 0);

    // Low-side OFF: LIN=HIGH (duty=MAX, bo LIN jest odwrócony w IR2103)
    ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
    ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
    ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
}

// ============================================================================
// Odczyt wejść analogowych
// ============================================================================

/**
 * @brief Odczytuje wszystkie wejścia analogowe i przelicza na wartości fizyczne.
 *
 * Wykonywane obliczenia:
 * 1. ADC raw → napięcie [V] (mnożnik 3.3/4095)
 * 2. VBAT: V_ADC × kVbatDividerGain = V_ADC × (R_top+R_bot)/R_bot
 * 3. Prądy: autokalibracja offsetu (EMA, α=0.02) gdy silnik off lub duty=0
 *    Prąd [A] = (V_ADC - offset) × kCurrentScale
 *    gdzie kCurrentScale = 1/(R_shunt × INA_gain) = 1/(0.002 × 50) = 10 A/V
 * 4. Przepustnica: raw ADC do g_bldc_state.throttle_raw (mapowanie w loop())
 * 5. Temperatura silnika: raw ADC (bez przeliczenia, czekamy na specyfikację czujnika)
 *
 * @note Prądy < 0 są clampowane do 0 (nie ma ujemnego prądu przez shunty low-side).
 * @note Autokalibracja offsetu prądu wymaga kilku sekund z wyłączonym silnikiem
 *       przy uruchomieniu firmware (filtr EMA stabilizuje się po ~50 iteracjach).
 */
void readAnalogInputs() {
    // Odczyt napięcia baterii
    uint16_t batteryRaw = analogRead(PIN_BATTERY_VOLTAGE);
    // Odczyt prądu fazy A (GPIO39)
    uint16_t phaseA_raw = analogRead(PIN_PHASE_A_CURRENT);

    // Przepustnica: odczyt + filtr EMA (ADC2/GPIO2 jest podatny na szumy
    // od PWM silnika — bez filtra pojedyncza szpilka <400 daje duty=0 i stall)
    {
        uint16_t thr_raw = analogRead(PIN_THROTTLE);
        static float thr_ema = 0.0f;
        static bool  thr_init = false;
        if (!thr_init) {
            thr_ema = (float)thr_raw;
            thr_init = true;
        } else {
            thr_ema += kThrottleFilterAlpha * ((float)thr_raw - thr_ema);
        }
        g_bldc_state.throttle_raw = (uint16_t)(thr_ema + 0.5f);
    }

    // Odczyt prądów fazowych
    uint16_t phaseB_raw = analogRead(PIN_PHASE_B_CURRENT);
    uint16_t phaseC_raw = analogRead(PIN_PHASE_C_CURRENT);

    // Odczyt temperatur
    uint16_t motorTempRaw = analogRead(PIN_MOTOR_TEMP);

    // Surowe wartości ADC
    const float batteryAdcV = batteryRaw * (3.3f / 4095.0f);
    const float phaseA_V = phaseA_raw * (3.3f / 4095.0f);
    const float phaseB_V = phaseB_raw * (3.3f / 4095.0f);
    const float phaseC_V = phaseC_raw * (3.3f / 4095.0f);

    // Autokalibracja zera prądu gdy silnik nie pracuje
    if (g_bldc_state.mode == DRIVE_MODE_DISABLED || g_bldc_state.duty_cycle == 0) {
        g_currentOffsetV[0] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[0] + kCurrentOffsetAlpha * phaseA_V;
        g_currentOffsetV[1] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[1] + kCurrentOffsetAlpha * phaseB_V;
        g_currentOffsetV[2] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[2] + kCurrentOffsetAlpha * phaseC_V;
    }

    g_bldc_state.battery_voltage = batteryAdcV * kVbatDividerGain;

    float ia = (phaseA_V - g_currentOffsetV[0]) * kCurrentScale;
    float ib = (phaseB_V - g_currentOffsetV[1]) * kCurrentScale;
    float ic = (phaseC_V - g_currentOffsetV[2]) * kCurrentScale;
    if (ia < 0.0f) ia = 0.0f;
    if (ib < 0.0f) ib = 0.0f;
    if (ic < 0.0f) ic = 0.0f;

    g_bldc_state.phase_current[0] = ia;
    g_bldc_state.phase_current[1] = ib;
    g_bldc_state.phase_current[2] = ic;
    g_bldc_state.motor_temperature = motorTempRaw;                   // Surowa wartość
}

// ============================================================================
// Odczyt czujników Halla
// ============================================================================

/**
 * @brief Odczytuje 3 czujniki Halla i zapisuje 3-bitowy kod do g_bldc_state.hall_state.
 *
 * Format: hall_state = [C:B:A] gdzie bit0=HallA, bit1=HallB, bit2=HallC.
 * Czujniki są INPUT_PULLUP (aktywny LOW: logika odwrócona przez hardware).
 * digitalRead() zwraca już poprawną wartość logiczną po pull-up.
 *
 * Poprawne stany: 1, 2, 3, 4, 5, 6 (6 pozycji elektrycznych rotora).
 * Stany 0 i 7 oznaczają błąd czujników (zwarcie lub przerwa).
 *
 * @note W ISR Halle są czytane szybciej bezpośrednio z rejestru GPIO.in
 *       (bez narzutu czasowego digitalRead). Ta funkcja jest tylko dla loop().
 */
void readHallSensors() {
    uint8_t hallA = digitalRead(PIN_HALL_SENSOR_A) ? 1 : 0;
    uint8_t hallB = digitalRead(PIN_HALL_SENSOR_B) ? 1 : 0;
    uint8_t hallC = digitalRead(PIN_HALL_SENSOR_C) ? 1 : 0;

    // Stan Halla: 3 bity [C:B:A]
    g_bldc_state.hall_state = (hallC << 2) | (hallB << 1) | hallA;
}

// ============================================================================
// Odczyt wejść cyfrowych
// ============================================================================

/**
 * @brief Odczytuje wejścia cyfrowe: hamulec i PAS.
 *
 * Hamulec: INPUT_PULLUP — aktywny sygnał = LOW (przycisk do GND).
 * PAS: pas_active wyznaczany z kadencji (nie z raw stanu pinu),
 *      bo czujnik Halla generuje impulsy — stan chwilowy nic nie mówi.
 */
void readDigitalInputs() {
    g_bldc_state.brake_active = (digitalRead(PIN_BRAKE) == LOW) || g_brake_simulated;
    // pas_active = ktoś pedałuje (kadencja > 0), ustalane w loop() przez calculatePasCadence()
    g_bldc_state.pas_active = (g_bldc_state.pas_cadence_rpm > 0);
    g_bldc_state.pas_forward = g_pas_forward;  // volatile → state
}

// ============================================================================
// Wyświetlanie diagnostyki
// ============================================================================

/**
 * @brief Wypisuje pojedynczą linię statusu na Serial.
 *
 * Format: `MODE DIR D:duty% V:Vbat Ia:X.XX Ib:X.XX Ic:X.XX H:CBA T:temp Thr:thr%(raw) [flagi]`
 *
 * Przykład:
 * @code
 * BLK CW D:45% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1850)
 * @endcode
 *
 * Kolumny:
 * - MODE:    OFF/BLK/SIN/FOC (tryb sterowania)
 * - DIR:     CW/CCW (kierunek)
 * - D:       duty cycle PWM [%]
 * - V:       napięcie baterii [V]
 * - Ia/Ib/Ic: prądy fazowe [A]
 * - H:       stan Halla [C:B:A] jako 3 bity
 * - T:       surowa wartość ADC temperatury silnika
 * - Thr:     przepustnica [%] i (RAW ADC)
 * - Opcjonalne flagi: BRK (hamulec), PAS, FAULT
 *
 * @note Wywołanie Serial.printf() może zablokować loop() na kilkanaście ms.
 *       Komutacja jest w ISR i nie jest tym zakłócana.
 */
void printDiagnostics() {
    const char* modeNames[] = {"OFF", "BLK", "SIN", "FOC"};
    int dutyPct = (int)((uint32_t)g_bldc_state.duty_cycle * 100 / PWM_MAX_DUTY);
    int targetPct = (int)((uint32_t)g_bldc_state.duty_target * 100 / PWM_MAX_DUTY);
    int thrPct = 0;
    if (g_bldc_state.throttle_raw > THROTTLE_DEAD_ZONE) {
        uint16_t thr = g_bldc_state.throttle_raw;
        if (thr > THROTTLE_MAX_RAW) thr = THROTTLE_MAX_RAW;
        thrPct = (int)((uint32_t)(thr - THROTTLE_DEAD_ZONE) * 100 / (THROTTLE_MAX_RAW - THROTTLE_DEAD_ZONE));
        if (thrPct > 100) thrPct = 100;
    }

    Serial.printf("%s %s D:%d/%d%% V:%.1f Ia:%.2f Ib:%.2f Ic:%.2f H:%d%d%d T:%d Thr:%d%%(%d) RPM:%lu WT:%u P:%.1fW",
        modeNames[g_bldc_state.mode],
        g_bldc_state.direction == DIRECTION_CW ? "CW" : "CCW",
        dutyPct, targetPct,
        g_bldc_state.battery_voltage,
        g_bldc_state.phase_current[0],
        g_bldc_state.phase_current[1],
        g_bldc_state.phase_current[2],
        (g_bldc_state.hall_state >> 2) & 1,
        (g_bldc_state.hall_state >> 1) & 1,
        g_bldc_state.hall_state & 1,
        (int)g_bldc_state.motor_temperature,
        thrPct,
        g_bldc_state.throttle_raw,
        (unsigned long)g_bldc_state.rpm,
        g_bldc_state.wheeltime_ms,
        g_bldc_state.regen_active ? g_bldc_state.regen_power_watts : g_bldc_state.power_watts);

    // Informacje o regeneracji
    if (g_bldc_state.regen_enabled) {
        if (g_bldc_state.regen_active) {
            Serial.printf(" RGN:%.1fW", g_bldc_state.regen_power_watts);
        } else {
            Serial.print(" RGN:rdy");
        }
    }

    // Informacja o aktywnej regeneracji
    if (g_bldc_state.regen_active) {
        Serial.print(" >>REGEN<<");
    }

    Serial.printf(" %s%s%s",
        g_bldc_state.brake_active ? "BRK " : "",
        g_bldc_state.pas_active ? "PAS " : "",
        g_bldc_state.fault ? "FAULT " : "");

    // PAS: kadencja, kierunek i prędkość docelowa
    if (g_bldc_state.pas_cadence_rpm > 0) {
        int pasDutyPct = (int)((uint32_t)g_bldc_state.pas_duty * 100 / PWM_MAX_DUTY);
        Serial.printf("CAD:%u/%d%% T:%.0f ", g_bldc_state.pas_cadence_rpm, pasDutyPct,
                      g_bldc_state.pas_target_kmh);
    } else if (!g_pas_forward) {
        Serial.print("PAS:REV ");
    }

    // Prędkość koła [km/h]
    if (g_bldc_state.wheel_speed_kmh > 0.5f) {
        Serial.printf("%.1fkm/h ", g_bldc_state.wheel_speed_kmh);
    }

    // Informacje z wyświetlacza S866
    if (g_display.connected) {
        Serial.printf("DISP:OK L%d %s%s",
            g_display.rx.assist_level / 3,
            g_display.rx.headlight ? "HL " : "",
            g_display.rx.cruise_control ? "CC " : "");
    } else {
        Serial.print("DISP:-- ");
    }
    Serial.println();
}

/**
 * @brief Diagnostyka krokowa trybu SINUS/BLOCK.
 *
 * Linie [SDBG] pokazują sekwencję:
 * Hall edges -> startup -> wejście SINUS -> fallback -> ponowny startup.
 */
static void printSineDebug() {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    int32_t angle_entry = (int32_t)(g_sine_angle_q16 >> 16);
    int32_t angle_frac = (int32_t)(g_sine_angle_q16 & 0xFFFF);

    Serial.printf("[SDBG] mode:%s run:%u st:%u hall:%u hp:%luus since:%lums ",
        (g_bldc_state.mode == DRIVE_MODE_SINUS) ? "SIN" : ((g_bldc_state.mode == DRIVE_MODE_BLOCK) ? "BLK" : "OTH"),
        (unsigned)g_sine_running,
        (unsigned)g_sine_startup_count,
        (unsigned)g_dbg_last_hall,
        (unsigned long)g_hall_period_us,
        (unsigned long)since_hall);

    Serial.printf("spd:%lu ang:%ld.%04ld dir:%s amp:%u m:%d/%d/%d ",
        (unsigned long)g_sine_speed_q16,
        (long)angle_entry,
        (long)((angle_frac * 10000L) >> 16),
        (g_direction_isr == DIRECTION_CW) ? "CW" : "CCW",
        (unsigned)g_dbg_last_amp,
        (int)g_dbg_last_ma, (int)g_dbg_last_mb, (int)g_dbg_last_mc);

    Serial.printf("ofs:%d ev[h:%lu en:%lu rej:%lu fb:%lu lastEn:%lums lastFb:%lums]\n",
        (int)g_sine_hall_phase_offset,
        (unsigned long)g_dbg_hall_edges,
        (unsigned long)g_dbg_sine_enter_count,
        (unsigned long)g_dbg_sine_start_reject_count,
        (unsigned long)g_dbg_sine_fallback_count,
        (unsigned long)g_dbg_last_sine_enter_ms,
        (unsigned long)g_dbg_last_fallback_ms);
}

// ============================================================================
// Timer ISR - komutacja w przerwaniu (niezależna od loop)
// ============================================================================

/**
 * @brief ISR timera sprzętowego — wykonywana co 50 µs (20 kHz).
 *
 * To jest SERCE sterownika. Wywoływana niezależnie od loop().
 * Czyta stan z volatile zmiennych globalnych i ustawia kanały LEDC.
 *
 * ## Priorytety obsługi (kolejność sprawdzania):
 * 1. Hamulec aktywny (g_brake_isr) → natychmiast allMosfetsOff()
 * 2. Silnik wyłączony (!g_motor_enabled) → allMosfetsOff()
 * 3. Odczyt Halli z rejestru GPIO.in (szybkie, bez przerwań)
 * 4. Wywołanie tabeli komutacji dla aktualnego kierunku
 *
 * ## Dlaczego IRAM_ATTR?
 * Kod ISR musi być w RAM, nie w flash. Bez IRAM_ATTR, jeśli cache flash
 * jest zajęty (np. przez OTA lub SPIFFS), ISR może wywołać cache miss
 * i zawiesić się na dziesiątki mikrosekund → zakłócenia komutacji.
 *
 * ## Odczyt GPIO.in zamiast digitalRead()
 * `GPIO.in` to bezpośredni rejestr hardware GPIO0-31.
 * Bit N = stan GPIO N. Czytanie rejestru trwa ~5 ns vs ~1 µs dla digitalRead().
 *
 * @warning Nie wolno tu używać: malloc, Serial, delay, mutex, nor F() string.
 * @warning ledcWrite() jest bezpieczne z ISR (operuje na rejestrach LEDC).
 */
void IRAM_ATTR onCommutationTimer() {
    // Odczyt Halli ZAWSZE — pomiar prędkości nawet gdy silnik wyłączony
    uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
    uint8_t hb = (GPIO.in >> PIN_HALL_SENSOR_B) & 1;
    uint8_t hc = (GPIO.in >> PIN_HALL_SENSOR_C) & 1;
    uint8_t hall = (hc << 2) | (hb << 1) | ha;

    // Pomiar czasu między przejściami Halla → RPM (niezależnie od stanu silnika)
    // DEBOUNCE: ignoruj przejścia szybsze niż HALL_MIN_PERIOD_US.
    // W trybie SINUS 6 FETów na center-aligned PWM generuje silne EMI,
    // które sprzega się w linie Halla tworząc fałszywe przejścia.
    // Bez debounce: hall_period = 50us → speed ucieka → kąt ucieka → desync.
    if (hall != g_hall_prev_isr) {
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        uint32_t dt_us = now_us - g_hall_last_change_us;

        // Debounce: akceptuj przejście tylko jeśli minęło wystarczająco dużo czasu
        // od ostatniego POTWIERDZONEGO przejścia (lub pierwszy pomiar)
        if (g_hall_last_change_us == 0 || dt_us >= HALL_MIN_PERIOD_US) {
            g_dbg_hall_edges++;
            g_dbg_last_hall = hall;
            if (g_hall_last_change_us > 0) {
                g_hall_period_us = dt_us;
            }
            g_hall_last_change_us = now_us;
            g_hall_prev_isr = hall;

            // ── Sine mode: przetwarzanie przejścia Halla ──
            // Port z bldc_driver_v2: bldc_hall_interrupt()
            if (g_mode_isr == DRIVE_MODE_SINUS) {
                g_sine_last_hall_ms = (uint32_t)(now_us / 1000);  // stall detection timestamp

                // --- Aktualizacja prędkości NATYCHMIAST w ISR ---
                // Krytyczne: obliczanie w loop() dawało opóźnienie 1-50ms
                // (Serial.printf), co przy 500 RPM = 20-1000 ISR ticków
                // z nieaktualną prędkością → kąt ucieka → desync.
                // 32-bit dzielenie na ESP32 Xtensa jest bezpieczne w ISR
                // (LoadStoreError dotyczy tylko byte-access do IRAM, nie ALU).
                if (dt_us >= HALL_MIN_PERIOD_US && dt_us < 500000) {
                    uint32_t new_speed = 52428800UL / dt_us;
                    uint32_t old_speed = g_sine_speed_q16;
                    if (old_speed == 0) {
                        g_sine_speed_q16 = new_speed;
                    } else {
                        g_sine_speed_q16 = (old_speed + new_speed) >> SINE_SPEED_FILTER_SHIFT;
                    }
                }

                int8_t new_idx = g_hall_to_sector[hall];
                if (new_idx >= 0) {
                    int8_t old_idx = g_sine_last_hall_idx;

                    // Detekcja kierunku z sekwencji przejść Halla
                    if (old_idx >= 0) {
                        int8_t fwd = (old_idx + 1) % 6;
                        int8_t rev = (new_idx + 1) % 6;
                        if (new_idx == fwd) {
                            g_sine_dir = 1;   // forward (CW)
                        } else if (old_idx == rev) {
                            g_sine_dir = -1;  // reverse (CCW)
                        }
                    }

                    // Korekcja kąta na przejściu Halla
                    // Oczekiwany kąt = środek sektora + offset fazowy Halla
                    int32_t expected_entry = (int32_t)new_idx * SINE_SECTOR_ENTRIES + SINE_SECTOR_CENTER + g_sine_hall_phase_offset;
                    if (expected_entry < 0) expected_entry += SINE_TABLE_SIZE;
                    if (expected_entry >= SINE_TABLE_SIZE) expected_entry -= SINE_TABLE_SIZE;
                    int32_t expected = expected_entry << 16;
                    int32_t current = (int32_t)g_sine_angle_q16;
                    int32_t err = expected - current;
                    // Wrap error to [-48<<16, +48<<16] (half revolution)
                    if (err > (int32_t)(SINE_TABLE_Q16_FULL >> 1)) err -= (int32_t)SINE_TABLE_Q16_FULL;
                    if (err < -(int32_t)(SINE_TABLE_Q16_FULL >> 1)) err += (int32_t)SINE_TABLE_Q16_FULL;

                    int32_t abs_err = (err >= 0) ? err : -err;

                    if (g_sine_speed_q16 == 0) {
                        // Przy starcie/crawl: pełny snap kąta na środek sektora
                        g_sine_angle_q16 = (uint32_t)expected;
                    } else if (abs_err > SINE_SNAP_THRESHOLD) {
                        // Duży błąd (>90° elektr.) = desync → pełny snap
                        // Zapobiega pozytywnej pętli zwrotnej: błąd→mniej momentu→stall
                        g_sine_angle_q16 = (uint32_t)expected;
                    } else {
                        // Normalna praca: łagodna korekcja 1/4 błędu
                        g_sine_angle_q16 = (uint32_t)(current + (err >> SINE_PHASE_CORR_SHIFT));
                    }
                    // Wrap angle
                    if (g_sine_angle_q16 >= SINE_TABLE_Q16_FULL) {
                        g_sine_angle_q16 -= SINE_TABLE_Q16_FULL;
                    }

                    g_sine_last_hall_idx = new_idx;

                    // Licznik przejść Halla (statystyka, nie blokuje startu sinusa)
                    if (g_sine_startup_count < 255) {
                        g_sine_startup_count++;
                    }
                }
            }
        }
        // else: szum EMI — ignoruj (nie aktualizuj prev ani timestamp)
    }

    // Tryb testu MOSFET — ISR nie dotyka kanałów LEDC, tylko mierzy Hall/prędkość
    if (g_mosfet_test_active) {
        return;
    }

    if (g_brake_isr) {
        // Hamulec aktywny — regen braking jeśli włączony, inaczej coast
        if (g_regen_active_isr && g_regen_duty_isr > 0) {
            regenCommutateISR(hall, g_regen_duty_isr, g_direction_isr);
        } else {
            ledcWrite(PWM_CHANNEL_A_HIGH, 0);
            ledcWrite(PWM_CHANNEL_B_HIGH, 0);
            ledcWrite(PWM_CHANNEL_C_HIGH, 0);
            ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
            ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
            ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
        }
        return;
    }

    if (!g_motor_enabled) {
        ledcWrite(PWM_CHANNEL_A_HIGH, 0);
        ledcWrite(PWM_CHANNEL_B_HIGH, 0);
        ledcWrite(PWM_CHANNEL_C_HIGH, 0);
        ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
        ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
        ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
        return;
    }

    uint16_t d = g_duty_isr;

    // SINUS safety fallback: gdy brak przejść Halla przez dłuższy czas → zeruj prędkość.
    // Timeout dynamiczny: max(400ms, 4*ostatni_hall_period + 20ms), aby uniknąć
    // oscylacji przy chwilowym spadku prędkości pod obciążeniem.
    // Po zerowaniu prędkości, crawl w sinusCommutateISR wolno obraca pole,
    // aż silnik da przejście Halla i prędkość zostanie przywrócona.
    if (g_mode_isr == DRIVE_MODE_SINUS) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t hp_us = g_hall_period_us;
        uint32_t dyn_ms = SINE_STALL_FALLBACK_MS;
        if (hp_us > 0 && hp_us < 500000) {
            uint32_t est_ms = ((hp_us * 4U) / 1000U) + 20U;
            if (est_ms > dyn_ms) dyn_ms = est_ms;
        }
        if ((now_ms - g_sine_last_hall_ms) > dyn_ms) {
            if (g_sine_speed_q16 != 0) {
                g_sine_speed_q16 = 0;
                g_dbg_sine_fallback_count++;
                g_dbg_last_fallback_ms = now_ms;
            }
            // Reset timestampu — pozwala crawl w sinusCommutateISR działać
            // kolejne STALL_FREEZE_MS ms zanim stall freeze znów zadziała.
            // Bez tego crawl byłby permanentnie zablokowany (deadlock).
            g_sine_last_hall_ms = now_ms;
        }
    }

    // Dispatch trybu sterowania
    switch (g_mode_isr) {
        case DRIVE_MODE_SINUS:
            // Sinus mode — zawsze sinusoidalny (bez block startup)
            sinusCommutateISR(hall, d);
            return;
        case DRIVE_MODE_BLOCK:
            break;  // kontynuuj do komutacji blokowej poniżej
        default:
            // Tryb niezaimplementowany (FOC) lub DISABLED → bezpieczny stan
            ledcWrite(PWM_CHANNEL_A_HIGH, 0); ledcWrite(PWM_CHANNEL_B_HIGH, 0); ledcWrite(PWM_CHANNEL_C_HIGH, 0);
            ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
            return;
    }

    if (g_direction_isr == DIRECTION_CW) {
        switch (hall) {
            case 1:
                ledcWrite(PWM_CHANNEL_A_HIGH, d);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, 0);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 3:
                ledcWrite(PWM_CHANNEL_A_HIGH, d);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, 0);
                break;
            case 2:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, d);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, 0);
                break;
            case 6:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, 0);
                ledcWrite(PWM_CHANNEL_B_HIGH, d);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 4:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, 0);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, d);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 5:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, 0);
                ledcWrite(PWM_CHANNEL_C_HIGH, d);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            default:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0); ledcWrite(PWM_CHANNEL_B_HIGH, 0); ledcWrite(PWM_CHANNEL_C_HIGH, 0);
                ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
        }
    } else {
        switch (hall) {
            case 1:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, 0);
                ledcWrite(PWM_CHANNEL_B_HIGH, d);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 3:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, 0);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, d);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 2:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, 0);
                ledcWrite(PWM_CHANNEL_C_HIGH, d);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 6:
                ledcWrite(PWM_CHANNEL_A_HIGH, d);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, 0);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
            case 4:
                ledcWrite(PWM_CHANNEL_A_HIGH, d);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, 0);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, 0);
                break;
            case 5:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0);   ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_B_HIGH, d);   ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
                ledcWrite(PWM_CHANNEL_C_HIGH, 0);   ledcWrite(PWM_CHANNEL_C_LOW, 0);
                break;
            default:
                ledcWrite(PWM_CHANNEL_A_HIGH, 0); ledcWrite(PWM_CHANNEL_B_HIGH, 0); ledcWrite(PWM_CHANNEL_C_HIGH, 0);
                ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY); ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
                break;
        }
    }
}

// ============================================================================
// Inicjalizacja timera komutacji
// ============================================================================

/**
 * @brief Konfiguruje i uruchamia timer sprzętowy dla ISR komutacji.
 *
 * Konfiguracja:
 * - Timer 0 (z 4 dostępnych: 0, 1, 2, 3)
 * - Prescaler = 80 → tick = 1 µs (80 MHz APB / 80 = 1 MHz)
 * - Auto-reload = true (powtarza się co alarm)
 * - Alarm = 50 µs → częstotliwość ISR = 1/50µs = 20 kHz
 * - Edge interrupt = true
 *
 * @note Timer APB clock = 80 MHz (stały, niezależny od CPU frequency)
 * @note Po tej funkcji ISR onCommutationTimer() będzie wywoływana od razu.
 *       Dlatego allMosfetsOff() musi być wywołana PRZED initCommutationTimer().
 */
void initCommutationTimer() {
    // Timer 0, prescaler 80 -> 1 MHz (1 us tick), alarm co 50 us = 20 kHz
    commutationTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(commutationTimer, &onCommutationTimer, true);
    timerAlarmWrite(commutationTimer, 50, true);
    timerAlarmEnable(commutationTimer);
}

// ============================================================================
// Komutacja sinusoidalna — ISR (port z bldc_driver_v2 TIM1_UP_IRQHandler)
// ============================================================================

/**
 * @brief Interpolacja liniowa sinusa z tablicy 97-elementowej (Q16).
 *
 * Identyczna logika jak sine_interp_q16() z bldc_driver_v2/src/bldc.c.
 * Tablica ma 97 wpisów (96+guard) — guard entry [96]=[0] eliminuje % 96.
 *
 * @param angle_q16  Kąt w wpisach tablicy (Q16), musi być 0..SINE_TABLE_Q16_FULL-1
 * @return Wartość sinusa -1024..+1024
 */
static inline int32_t IRAM_ATTR sine_interp_q16(uint32_t angle_q16) {
    uint32_t idx  = angle_q16 >> 16;        // indeks całkowity 0..95
    uint32_t frac = angle_q16 & 0xFFFF;     // część ułamkowa Q16
    int32_t s0 = (int32_t)g_sine_table[idx];
    int32_t s1 = (int32_t)g_sine_table[idx + 1];  // guard entry [96] = entry [0]
    return s0 + (((s1 - s0) * (int32_t)frac) >> 16);
}

/**
 * @brief Komutacja sinusoidalna — ciągłe śledzenie kąta.
 *
 * ## Algorytm (port z bldc_driver_v2 TIM1_UP_IRQHandler)
 *
 * 1. Advance angle: angle += speed_q16 (lub -= dla CCW)
 * 2. Stall freeze: brak przejść Halla > 200ms → zamrożenie kąta
 * 3. Oblicz 3 kąty fazowe: C=base, A=base+32, B=base+64 (×120°)
 * 4. Interpolacja sinusa z tabeli + obliczenie duty
 * 5. Center-aligned complementary: HIN=LIN=ten sam duty
 *
 * ## Mapowanie faz (identyczne z STM32)
 *   Phase C = sin(θ)           — faza referencyjna
 *   Phase A = sin(θ + 120°)    — offset 32 wpisów (96/3)
 *   Phase B = sin(θ + 240°)    — offset 64 wpisów (96×2/3)
 *
 * ## PWM center-aligned complementary
 *   duty = 512 + (sine_val × amplitude) >> 10
 *   HIN = LIN = duty → IR2103 produkuje komplementarne switching z dead-time
 *
 * @param hall      Aktualny stan Halla [C:B:A] 1-6
 * @param amplitude Duty z przepustnicy/rampy 0-PWM_MAX_DUTY
 */
static void IRAM_ATTR sinusCommutateISR(uint8_t hall, uint16_t amplitude) {
    // Walidacja Halla
    if (hall == 0 || hall == 7) {
        allMosfetsOff();
        return;
    }

    // Gdy amplitude < SINE_MIN_AMPLITUDE → coast (wszystkie FETy OFF).
    // Center-aligned PWM z bazą 512 przy małej amplitudzie = ~50% switching
    // na wszystkich fazach = aktywne hamowanie elektromagnetyczne.
    // W block mode mały duty = niemal 0 prądu (tiny PWM na 1 fazie).
    if (amplitude < SINE_MIN_AMPLITUDE) {
        allMosfetsOff();
        return;
    }

    // ── 1. Stall freeze: brak przejścia Halla > 200ms → nie avansuj kąta ──
    // WYJĄTEK: w trybie crawl (speed==0) zawsze avansuj — crawl jest wolny
    // (~1 obr.elekt./s) i nie generuje niebezpiecznych prądów.
    // Bez tego wyjątku: stall freeze blokuje crawl → silnik nigdy nie ruszy → deadlock.
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    uint32_t real_speed = g_sine_speed_q16;
    bool in_crawl = (real_speed == 0);
    bool stalled = !in_crawl && (since_hall > SINE_STALL_FREEZE_MS);

    // ── 2. Advance angle ──
    if (!stalled) {
        uint32_t spd = real_speed;
        // Gdy speed==0 (startup/crawl), użyj minimalnej prędkości otwartopętlowej
        // aby pole magnetyczne wolno się obracało i rotor zaczął podążać.
        // Pierwsze przejście Halla da realną prędkość i crawl się wyłączy.
        if (spd == 0) spd = SINE_CRAWL_SPEED_Q16;
        {
            bool forward = (g_direction_isr == DIRECTION_CW);
            if (forward) {
                g_sine_angle_q16 += spd;
                if (g_sine_angle_q16 >= SINE_TABLE_Q16_FULL) {
                    g_sine_angle_q16 -= SINE_TABLE_Q16_FULL;
                }
            } else {
                if (g_sine_angle_q16 >= spd) {
                    g_sine_angle_q16 -= spd;
                } else {
                    g_sine_angle_q16 = SINE_TABLE_Q16_FULL - (spd - g_sine_angle_q16);
                }
            }
        }
    }

    // ── 3. Oblicz 3 kąty fazowe z offsetami ──
    // Mapowanie faz dopasowane do tabeli komutacji blokowej CW:
    //   Phase A = sin(θ)           — faza referencyjna (peak w sektorach 0,1)
    //   Phase B = sin(θ + 240°)    — offset 64 wpisów (peak w sektorach 2,3)
    //   Phase C = sin(θ + 120°)    — offset 32 wpisy (peak w sektorach 4,5)
    uint32_t angle = g_sine_angle_q16;
    uint32_t angle_a = angle;  // A = reference
    uint32_t angle_b = angle + ((uint32_t)SINE_PHASE_B_OFFSET << 16);
    uint32_t angle_c = angle + ((uint32_t)SINE_PHASE_C_OFFSET << 16);

    // Wrap to valid range (subtraction, no modulo)
    if (angle_b >= SINE_TABLE_Q16_FULL) angle_b -= SINE_TABLE_Q16_FULL;
    if (angle_c >= SINE_TABLE_Q16_FULL) angle_c -= SINE_TABLE_Q16_FULL;

    // ── 4. Interpolacja sinusa + obliczenie duty ──
    int32_t sin_a = sine_interp_q16(angle_a);  // -1024..+1024
    int32_t sin_b = sine_interp_q16(angle_b);
    int32_t sin_c = sine_interp_q16(angle_c);

    // amplitude: 0..PWM_MAX_DUTY(1023), z ograniczeniem bezpieczeństwa
    int32_t amp = (int32_t)((amplitude > SINE_SAFE_MAX_DUTY) ? SINE_SAFE_MAX_DUTY : amplitude);
    g_dbg_last_amp = (uint16_t)amp;

    // Modulacja fazy: -amp..+amp
    int32_t ma = (sin_a * amp) >> 10;
    int32_t mb = (sin_b * amp) >> 10;
    int32_t mc = (sin_c * amp) >> 10;
    g_dbg_last_ma = (int16_t)ma;
    g_dbg_last_mb = (int16_t)mb;
    g_dbg_last_mc = (int16_t)mc;

    // ── 5. Write LEDC — SVPWM (Space Vector PWM / min-max centering) ──
    // Zamiast stałej bazy 512, przesuwamy wszystkie 3 modulacje razem tak,
    // żeby mieściły się w zakresie 0..PWM_MAX_DUTY bez klipowania.
    // Offset = 512 - (max+min)/2 to zero-sequence (common-mode) składowa,
    // która nie wpływa na napięcie linia-linia, ale zwiększa zakres liniowy
    // o 15.5% (z Vbus*√3/2 do Vbus) — identycznie jak SVPWM.
    //
    // Bez SVPWM: amp > 512 → klipowanie → brak wzrostu napięcia fundamentalnego
    // Z SVPWM:   amp do ~591 → pełny Vbus bez zniekształceń
    //
    // IR2103: HIN i LIN dostają TEN SAM duty → komplementarne przełączanie
    // z wbudowanym dead-time ~520ns.
    {
        // Min-max centering (SVPWM)
        int32_t mn = ma;
        if (mb < mn) mn = mb;
        if (mc < mn) mn = mc;
        int32_t mx = ma;
        if (mb > mx) mx = mb;
        if (mc > mx) mx = mc;
        int32_t offset = 512 - ((mx + mn) >> 1);

        int32_t da = offset + ma;
        if (da < 0) da = 0;
        if (da > (int32_t)PWM_MAX_DUTY) da = (int32_t)PWM_MAX_DUTY;
        ledcWrite(PWM_CHANNEL_A_HIGH, (uint32_t)da);
        ledcWrite(PWM_CHANNEL_A_LOW,  (uint32_t)da);

        int32_t db = offset + mb;
        if (db < 0) db = 0;
        if (db > (int32_t)PWM_MAX_DUTY) db = (int32_t)PWM_MAX_DUTY;
        ledcWrite(PWM_CHANNEL_B_HIGH, (uint32_t)db);
        ledcWrite(PWM_CHANNEL_B_LOW,  (uint32_t)db);

        int32_t dc = offset + mc;
        if (dc < 0) dc = 0;
        if (dc > (int32_t)PWM_MAX_DUTY) dc = (int32_t)PWM_MAX_DUTY;
        ledcWrite(PWM_CHANNEL_C_HIGH, (uint32_t)dc);
        ledcWrite(PWM_CHANNEL_C_LOW,  (uint32_t)dc);
    }
}

// ============================================================================
// Komutacja blokowa (trapezoidalna / 6-step)
// ============================================================================
//
// Tabela komutacji dla kierunku CW (Hall [C:B:A]):
//
//   Hall | Faza A      | Faza B      | Faza C
//   -----+-------------+-------------+------------
//    1   | PWM (high)  | LOW (low-on)| OFF (float)
//    3   | PWM (high)  | OFF (float) | LOW (low-on)
//    2   | OFF (float) | PWM (high)  | LOW (low-on)
//    6   | LOW (low-on)| PWM (high)  | OFF (float)
//    4   | LOW (low-on)| OFF (float) | PWM (high)
//    5   | OFF (float) | LOW (low-on)| PWM (high)
//
// Dla CCW zamienione high/low.

// Pomocnicze inline do ustawiania stanu fazy
/**
 * @brief Faza A: wyjście PWM (high-side ON z modulacją, low-side OFF).
 * @param duty Wypełnienie PWM 0–PWM_MAX_DUTY.
 */
static inline void phaseA_PWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_A_HIGH, duty);         // HIN = PWM
    ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);  // LIN = HIGH -> low-side OFF
}
/** @brief Faza A: podłączona do GND (high-side OFF, low-side ON). */
static inline void phaseA_Low() {
    ledcWrite(PWM_CHANNEL_A_HIGH, 0);             // HIN = LOW -> high-side OFF
    ledcWrite(PWM_CHANNEL_A_LOW, 0);              // LIN = LOW -> low-side ON
}
/** @brief Faza A: pływająca (oba tranzystory OFF). */
static inline void phaseA_Off() {
    ledcWrite(PWM_CHANNEL_A_HIGH, 0);             // HIN = LOW -> high-side OFF
    ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);   // LIN = HIGH -> low-side OFF
}

/** @brief Faza B: PWM (high-side ON, low-side OFF). @param duty 0–PWM_MAX_DUTY. */
static inline void phaseB_PWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_B_HIGH, duty);
    ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
}
/** @brief Faza B: GND (high-side OFF, low-side ON). */
static inline void phaseB_Low() {
    ledcWrite(PWM_CHANNEL_B_HIGH, 0);
    ledcWrite(PWM_CHANNEL_B_LOW, 0);
}
/** @brief Faza B: pływająca (oba OFF). */
static inline void phaseB_Off() {
    ledcWrite(PWM_CHANNEL_B_HIGH, 0);
    ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
}

/** @brief Faza C: PWM (high-side ON, low-side OFF). @param duty 0–PWM_MAX_DUTY. */
static inline void phaseC_PWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_C_HIGH, duty);
    ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
}
/** @brief Faza C: GND (high-side OFF, low-side ON). */
static inline void phaseC_Low() {
    ledcWrite(PWM_CHANNEL_C_HIGH, 0);
    ledcWrite(PWM_CHANNEL_C_LOW, 0);
}
/** @brief Faza C: pływająca (oba OFF). */
static inline void phaseC_Off() {
    ledcWrite(PWM_CHANNEL_C_HIGH, 0);
    ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
}

// ============================================================================
// Funkcje pomocnicze regen — Low-side PWM (HS OFF, LS modulowany)
// ============================================================================

/**
 * @brief Faza A: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY (0=brak zwarcia, MAX=pełne zwarcie).
 *
 * IR2103 LIN odwrócony: duty=0 → LIN=LOW → LS ON,
 * więc LEDC duty = PWM_MAX_DUTY - regen_duty.
 * Przy PWM OFF (LS wyłączony) prąd indukcyjny płynie przez body diodę HS do V+.
 */
static inline void IRAM_ATTR phaseA_RegenPWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_A_HIGH, 0);                          // HIN=LOW → HS OFF
    ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY - duty);         // LS PWM (odwrócone)
}

/**
 * @brief Faza B: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseB_RegenPWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_B_HIGH, 0);
    ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY - duty);
}

/**
 * @brief Faza C: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseC_RegenPWM(uint16_t duty) {
    ledcWrite(PWM_CHANNEL_C_HIGH, 0);
    ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY - duty);
}

// ============================================================================
// ISR regen — komutacja regeneracyjna
// ============================================================================

/**
 * @brief Komutacja regeneracyjna w ISR (low-side boost chopper).
 *
 * Zasada działania:
 * - Faza, która w motoring miała HS_PWM (źródło) → teraz LS_PWM (regen)
 * - Faza, która w motoring miała LS_ON (sink) → LS_ON (bez zmian)
 * - Trzecia faza → float (bez zmian)
 *
 * Cykl PWM regen:
 * 1. PWM ON (LS ON): uzwojenie zwarte przez GND, prąd narasta (L ładuje się)
 * 2. PWM OFF (LS OFF): prąd indukcyjny płynie przez body diodę HS → Vbat (ładuje baterię)
 *
 * @param hall  Stan Halla [C:B:A] 1-6
 * @param regen_duty Siła hamowania 0–PWM_MAX_DUTY
 * @param dir   Kierunek obrotu
 *
 * @warning Wszystkie high-side FETy MUSZĄ być OFF! Shoot-through = uszkodzenie.
 * @warning Nigdy duty 100% — brak fazy OFF = brak transferu do baterii (tylko ciepło).
 */
static void IRAM_ATTR regenCommutateISR(uint8_t hall, uint16_t regen_duty, motor_direction_t dir) {
    if (dir == DIRECTION_CW) {
        switch (hall) {
            case 1:  // motoring: A+ B-  →  regen: A=LS_PWM, B=LS_ON, C=float
                phaseA_RegenPWM(regen_duty);
                phaseB_Low();
                phaseC_Off();
                break;
            case 3:  // motoring: A+ C-  →  regen: A=LS_PWM, B=float, C=LS_ON
                phaseA_RegenPWM(regen_duty);
                phaseB_Off();
                phaseC_Low();
                break;
            case 2:  // motoring: B+ C-  →  regen: A=float, B=LS_PWM, C=LS_ON
                phaseA_Off();
                phaseB_RegenPWM(regen_duty);
                phaseC_Low();
                break;
            case 6:  // motoring: B+ A-  →  regen: A=LS_ON, B=LS_PWM, C=float
                phaseA_Low();
                phaseB_RegenPWM(regen_duty);
                phaseC_Off();
                break;
            case 4:  // motoring: C+ A-  →  regen: A=LS_ON, B=float, C=LS_PWM
                phaseA_Low();
                phaseB_Off();
                phaseC_RegenPWM(regen_duty);
                break;
            case 5:  // motoring: C+ B-  →  regen: A=float, B=LS_ON, C=LS_PWM
                phaseA_Off();
                phaseB_Low();
                phaseC_RegenPWM(regen_duty);
                break;
            default:
                allMosfetsOff();
                break;
        }
    } else {  // CCW
        switch (hall) {
            case 1:  // motoring CCW: B+ A-  →  regen: A=LS_ON, B=LS_PWM, C=float
                phaseA_Low();
                phaseB_RegenPWM(regen_duty);
                phaseC_Off();
                break;
            case 3:  // motoring CCW: C+ A-  →  regen: A=LS_ON, B=float, C=LS_PWM
                phaseA_Low();
                phaseB_Off();
                phaseC_RegenPWM(regen_duty);
                break;
            case 2:  // motoring CCW: C+ B-  →  regen: A=float, B=LS_ON, C=LS_PWM
                phaseA_Off();
                phaseB_Low();
                phaseC_RegenPWM(regen_duty);
                break;
            case 6:  // motoring CCW: A+ B-  →  regen: A=LS_PWM, B=LS_ON, C=float
                phaseA_RegenPWM(regen_duty);
                phaseB_Low();
                phaseC_Off();
                break;
            case 4:  // motoring CCW: A+ C-  →  regen: A=LS_PWM, B=float, C=LS_ON
                phaseA_RegenPWM(regen_duty);
                phaseB_Off();
                phaseC_Low();
                break;
            case 5:  // motoring CCW: B+ C-  →  regen: A=float, B=LS_PWM, C=LS_ON
                phaseA_Off();
                phaseB_RegenPWM(regen_duty);
                phaseC_Low();
                break;
            default:
                allMosfetsOff();
                break;
        }
    }
}

/**
 * @brief Komutacja blokowa 6-step (backup path — wywoływana z loop()).
 *
 * Ustawia stany faz A/B/C na podstawie stanu Halla i kierunku.
 * Ta funkcja NIE jest używana w normalnej pracy (ISR obsługuje komutację).
 * Zachowana jako fallback / do debugowania bez timera.
 *
 * @param hallState 3-bitowy stan Halla [C:B:A], wartości 1–6
 * @param duty      Wypełnienie PWM 0–PWM_MAX_DUTY
 *
 * @note Jeśli hallState == 0 lub 7 (błąd czujników) → allMosfetsOff() + fault=true.
 * @note Używa pomocniczych funkcji phaseX_PWM/Low/Off() zamiast bezpośrednich ledcWrite().
 */
void blockCommutate(uint8_t hallState, uint16_t duty) {
    // Walidacja stanu Halla
    if (hallState == HALL_STATE_INVALID_0 || hallState == HALL_STATE_INVALID_7) {
        allMosfetsOff();
        g_bldc_state.fault = true;
        return;
    }

    if (g_bldc_state.direction == DIRECTION_CW) {
        // Komutacja CW
        switch (hallState) {
            case 1: // Hall 001: A+ B-
                phaseA_PWM(duty);
                phaseB_Low();
                phaseC_Off();
                break;
            case 3: // Hall 011: A+ C-
                phaseA_PWM(duty);
                phaseB_Off();
                phaseC_Low();
                break;
            case 2: // Hall 010: B+ C-
                phaseA_Off();
                phaseB_PWM(duty);
                phaseC_Low();
                break;
            case 6: // Hall 110: B+ A-
                phaseA_Low();
                phaseB_PWM(duty);
                phaseC_Off();
                break;
            case 4: // Hall 100: C+ A-
                phaseA_Low();
                phaseB_Off();
                phaseC_PWM(duty);
                break;
            case 5: // Hall 101: C+ B-
                phaseA_Off();
                phaseB_Low();
                phaseC_PWM(duty);
                break;
            default:
                allMosfetsOff();
                break;
        }
    } else {
        // Komutacja CCW - zamienione high-side i low-side
        switch (hallState) {
            case 1: // Hall 001: B+ A-
                phaseA_Low();
                phaseB_PWM(duty);
                phaseC_Off();
                break;
            case 3: // Hall 011: C+ A-
                phaseA_Low();
                phaseB_Off();
                phaseC_PWM(duty);
                break;
            case 2: // Hall 010: C+ B-
                phaseA_Off();
                phaseB_Low();
                phaseC_PWM(duty);
                break;
            case 6: // Hall 110: A+ B-
                phaseA_PWM(duty);
                phaseB_Low();
                phaseC_Off();
                break;
            case 4: // Hall 100: A+ C-
                phaseA_PWM(duty);
                phaseB_Off();
                phaseC_Low();
                break;
            case 5: // Hall 101: B+ C-
                phaseA_Off();
                phaseB_PWM(duty);
                phaseC_Low();
                break;
            default:
                allMosfetsOff();
                break;
        }
    }
}

// ============================================================================
// Obsługa komend Serial
// ============================================================================

/** @brief Wypisuje tabelę dostępnych komend Serial na konsole. */
void printHelp() {
    Serial.println();
    Serial.println("========== KOMENDY (wszystkie wymagają Enter) ==========");
    Serial.println("e          Włącz tryb BLOCK sterowania");
    Serial.println("S          Włącz tryb SINUS sterowania");
    Serial.println("m2         Włącz tryb SINUS (alternatywna)");
    Serial.println("d          Wyłącz silnik (duty=0)");
    Serial.println("r          Odwróć kierunek (CW/CCW) - tylko gdy wyłączony");
    Serial.println("+/-        Zmień duty o ±5%");
    Serial.println("0-100      Ustaw duty w procentach");
    Serial.println("R          Regeneracja ON/OFF (hamowanie rekuperacyjne)");
    Serial.println("b          Symulacja hamulca ON/OFF");
    Serial.println("P          Pokaż parametry wyświetlacza P01-P20");
    Serial.println("s          Pokaż status");
    Serial.println("a          Auto-status co 1s ON/OFF");
    Serial.println("man        Manual duty ON/OFF (manetka ignorowana gdy ON)");
    Serial.println("gdbg       Debug SINUS/BLOCK ON/OFF (co 200ms)");
    Serial.println("so         Pokaż aktualny sine phase offset");
    Serial.println("so+        Offset +2 wpisy (+7.5°)");
    Serial.println("so-        Offset -2 wpisy (-7.5°)");
    Serial.println("so:N       Ustaw offset na N (-48..+48, 1=3.75°)");
    Serial.println("sat        Auto-tune offsetu fazy (sweep -24..+24, ~25s)");
    Serial.println("sat:M:N    Auto-tune zakres M..N  (np. sat:-16:16)");
    Serial.println("sat:M:N:S  Auto-tune zakres M..N krok S (np. sat:-8:8:1)");
    Serial.println("h          Pokaż tę pomoc");
    Serial.println("---------- Test MOSFET (diagnostyka) ----------");
    Serial.println("tAH        Test faza A HIGH-side");
    Serial.println("tAL        Test faza A LOW-side");
    Serial.println("tBH        Test faza B HIGH-side");
    Serial.println("tBL        Test faza B LOW-side");
    Serial.println("tCH        Test faza C HIGH-side");
    Serial.println("tCL        Test faza C LOW-side");
    Serial.println("tp:N       Ustaw duty testowe na N% (1-50)");
    Serial.println("t0         Wyłącz test MOSFET (wszystkie OFF)");
    Serial.println("t          Pokaż pomoc trybu testowego");
    Serial.println("---------- Konfiguracja NVS ----------");
    Serial.println("cfg            Poka\u017c aktualn\u0105 konfiguracj\u0119 NVS");
    Serial.println("cfg:mode:N    Tryb boot (1=BLOCK 2=SIN 3=FOC)");
    Serial.println("cfg:ramp:N    Czas rampy [ms] 0-10000");
    Serial.println("cfg:regen:N   Regeneracja boot (0/1)");
    Serial.println("===========================");
    Serial.println();
}

/**
 * @brief Wypisuje parametry konfiguracyjne wyświetlacza P01-P20.
 */
void printDisplayConfig() {
    Serial.println();
    if (!g_display.connected) {
        Serial.println("[S866] Wyświetlacz nie podłączony — brak parametrów");
        return;
    }
    const s866_config_t& c = g_display.config;
    Serial.println("========== PARAMETRY WYŚWIETLACZA S866 ==========");
    Serial.println("--- Parametry lokalne wyświetlacza (nie w ramce) ---");
    Serial.printf("P01  Jasność podświetlenia:   [local]\n");
    Serial.printf("P02  Jednostki prędkości:     [local]\n");
    Serial.printf("P03  Napięcie systemu:        [local]\n");
    Serial.printf("P04  Auto-wyłączenie:         [local]\n");
    Serial.println("--- Parametry z ramki RX ---");
    Serial.printf("P05  Poziomy wspomagania:     %d\n",        c.p05_assist_levels);
    Serial.printf("P06  Rozmiar koła:            %d.%d\"\n",   c.p06_wheel_size_x10 / 10, c.p06_wheel_size_x10 % 10);
    Serial.printf("P07  Pole pairs / magnesy:    %d\n",        c.p07_speed_magnets);
    Serial.printf("P08  Limit prędkości:         %d km/h\n",   c.p08_speed_limit);
    Serial.printf("P09  Tryb startu:             %s\n",       c.p09_start_mode ? "po pedałowaniu" : "od zera");
    Serial.printf("P10  Tryb jazdy:              %d\n",        c.p10_drive_mode);
    Serial.printf("P11  Czułość PAS:             %d\n",        c.p11_pas_sensitivity);
    Serial.printf("P12  Intensywność startu PAS: %d\n",        c.p12_pas_start_strength);
    Serial.printf("P13  Magnesy PAS:             %d\n",        c.p13_pas_magnets);
    Serial.printf("P14  Limit prądu:             %d A\n",      c.p14_current_limit_a);
    Serial.printf("P15  Podna pięcie:            %.1f V\n",    c.p15_undervoltage_x10 / 10.0f);
    Serial.println("--- Parametry lokalne wyświetlacza (nie w ramce) ---");
    Serial.printf("P16  Tryb komunikacji:        [local]\n");
    Serial.printf("P17  Tempomat:                %s\n",       c.p17_cruise_control ? "ON" : "OFF");
    Serial.printf("P18  Tryb gazu:               [local]\n");
    Serial.printf("P19  Power Assist:            [local]\n");
    Serial.printf("P20  Protokół:                [local]\n");
    Serial.println("=================================================");
    uint8_t p07 = g_display.config.p07_speed_magnets;
    if (p07 <= 1) {
        Serial.println("Tryb prędkości: czujnik SPEED (silnik przekładniowy)");
    } else {
        Serial.printf("Tryb prędkości: Hall × %d pole_pairs (direct-drive)\n", p07);
    }
    Serial.println();
}

// ============================================================================
// Test MOSFET — diagnostyka pojedynczych tranzystorów
// ============================================================================

/**
 * @brief Wyświetla pomoc trybu testowego MOSFET.
 */
static void mosfetTestPrintHelp() {
    Serial.println();
    Serial.println("========== TEST MOSFET (diagnostyka) ==========");
    Serial.println("Procedura testu uszkodzonych tranzystorow MOSFET.");
    Serial.println("Wystawia 10% PWM na POJEDYNCZY wskazany tranzystor.");
    Serial.println("Pozostale tranzystory sa WYLACZONE.");
    Serial.println();
    Serial.println("!!! UWAGA: silnik musi byc WYLACZONY (komenda 'd') !!!");
    Serial.println("!!! NIGDY nie wlaczaj HIGH+LOW tej samej fazy !!!");
    Serial.println("!!! (shoot-through = zwarcie zasilania)        !!!");
    Serial.println();
    Serial.println("Komendy (wyslij + Enter):");
    Serial.println("  tAH   Faza A HIGH-side  (GPIO32, IR2103 HIN_A)");
    Serial.println("  tAL   Faza A LOW-side   (GPIO33, IR2103 LIN_A)");
    Serial.println("  tBH   Faza B HIGH-side  (GPIO25, IR2103 HIN_B)");
    Serial.println("  tBL   Faza B LOW-side   (GPIO26, IR2103 LIN_B)");
    Serial.println("  tCH   Faza C HIGH-side  (GPIO27, IR2103 HIN_C)");
    Serial.println("  tCL   Faza C LOW-side   (GPIO14, IR2103 LIN_C)");
    Serial.println("  t0    Wylacz test (wszystkie OFF)");
    Serial.println("  tp:N  Ustaw duty testowe na N% (1-50, np. tp:20)");
    Serial.println();
    Serial.println("Po wlaczeniu testu uzyj 's' aby odczytac prady fazowe.");
    Serial.printf( "Duty testowe: %d/%d (%.0f%%)\n",
                   g_mosfet_test_duty, PWM_MAX_DUTY,
                   100.0f * g_mosfet_test_duty / PWM_MAX_DUTY);
    Serial.println("================================================");
    Serial.println();
}

/**
 * @brief Ustawia 10% PWM na wybranym tranzystorze MOSFET (diagnostyka).
 *
 * Wyłącza silnik, aktywuje tryb testowy (ISR nie nadpisuje LEDC),
 * ustawia allMosfetsOff() a potem włącza TYLKO wybrany tranzystor.
 *
 * Logika IR2103:
 *   HIGH-side ON: ledcWrite(HIN_channel, g_mosfet_test_duty)
 *   LOW-side  ON: ledcWrite(LIN_channel, PWM_MAX_DUTY - g_mosfet_test_duty)
 *                 — LIN=LOW przez X% (wejście odwrócone IR2103)
 *
 * @param cmd Komenda "tXY" gdzie X={A,B,C} Y={H,L}
 * @return Opis wyniku
 */
static String mosfetTestSet(const String& cmd) {
    if (cmd.length() != 3) return "Format: tXY (np. tAH)";

    char phase = cmd[1];  // A, B, C
    char side  = cmd[2];  // H (high-side), L (low-side)

    // Walidacja
    if (phase != 'A' && phase != 'a' && phase != 'B' && phase != 'b' && phase != 'C' && phase != 'c') {
        return "Bledna faza! Uzyj A, B lub C";
    }
    if (side != 'H' && side != 'h' && side != 'L' && side != 'l') {
        return "Bledna strona! Uzyj H (high) lub L (low)";
    }

    // Normalizacja na wielkie litery
    phase = toupper(phase);
    side  = toupper(side);

    // Wyłącz silnik i włącz tryb testowy
    g_bldc_state.mode = DRIVE_MODE_DISABLED;
    g_bldc_state.duty_cycle = 0;
    g_bldc_state.duty_target = 0;
    g_duty_ramped = 0;
    g_motor_enabled = false;
    g_mosfet_test_active = true;  // ISR nie będzie nadpisywać LEDC
    g_mosfet_test_phase = phase;
    g_mosfet_test_side  = side;

    // Bezpieczny stan — wszystko OFF
    allMosfetsOff();

    // Wybór kanału i ustawienie PWM
    uint16_t duty = g_mosfet_test_duty;
    const char* pinInfo = "";
    if (phase == 'A' && side == 'H') {
        ledcWrite(PWM_CHANNEL_A_HIGH, duty);
        pinInfo = "GPIO32 HIN_A";
    } else if (phase == 'A' && side == 'L') {
        ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY - duty);
        pinInfo = "GPIO33 LIN_A";
    } else if (phase == 'B' && side == 'H') {
        ledcWrite(PWM_CHANNEL_B_HIGH, duty);
        pinInfo = "GPIO25 HIN_B";
    } else if (phase == 'B' && side == 'L') {
        ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY - duty);
        pinInfo = "GPIO26 LIN_B";
    } else if (phase == 'C' && side == 'H') {
        ledcWrite(PWM_CHANNEL_C_HIGH, duty);
        pinInfo = "GPIO27 HIN_C";
    } else if (phase == 'C' && side == 'L') {
        ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY - duty);
        pinInfo = "GPIO14 LIN_C";
    }

    // Komunikat z informacji diagnostycznych
    int pct = (int)((uint32_t)duty * 100 / PWM_MAX_DUTY);
    char buf[128];
    snprintf(buf, sizeof(buf),
             "TEST: Faza %c %s-side ON (%d%% PWM) [%s]",
             phase, (side == 'H') ? "HIGH" : "LOW", pct, pinInfo);
    return String(buf);
}

/**
 * @brief Zmienia duty testowe i aktualizuje aktywny test (jeśli trwa).
 *
 * @param pct Procent PWM (1-50)
 * @return Opis wyniku
 */
static String mosfetTestSetDuty(int pct) {
    if (pct < 1)  pct = 1;
    if (pct > 50) pct = 50;
    g_mosfet_test_duty = (uint16_t)((uint32_t)pct * PWM_MAX_DUTY / 100);

    // Jeśli test jest aktywny — natychmiast zaktualizuj PWM na bieżącym tranzystorze
    if (g_mosfet_test_active && g_mosfet_test_phase != 0) {
        // Ponowne ustawienie tego samego tranzystora z nowym duty
        allMosfetsOff();
        uint16_t duty = g_mosfet_test_duty;
        char phase = g_mosfet_test_phase;
        char side  = g_mosfet_test_side;
        if (phase == 'A' && side == 'H') ledcWrite(PWM_CHANNEL_A_HIGH, duty);
        else if (phase == 'A' && side == 'L') ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY - duty);
        else if (phase == 'B' && side == 'H') ledcWrite(PWM_CHANNEL_B_HIGH, duty);
        else if (phase == 'B' && side == 'L') ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY - duty);
        else if (phase == 'C' && side == 'H') ledcWrite(PWM_CHANNEL_C_HIGH, duty);
        else if (phase == 'C' && side == 'L') ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY - duty);

        char buf[96];
        snprintf(buf, sizeof(buf), "Test duty: %d%% — zaktualizowano Faza %c %s-side",
                 pct, phase, (side == 'H') ? "HIGH" : "LOW");
        return String(buf);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Test duty: %d%% (aktywuj komenda tXY)", pct);
    return String(buf);
}

/**
 * @brief Przetwarza wszystkie dostępne bajty z bufora Serial.
 *
 * Wszystkie komendy wymagają Enter. Znaki buforowane do '\n'/'\r',
 * wtedy przekazywane do executeCommand().
 */
void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                String result = executeCommand(serialBuffer);
                if (result.length() > 0) {
                    Serial.printf("[CMD] %s\n", result.c_str());
                }
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
        }
    }
}

// ============================================================================
// Wspólna obsługa komend Serial
// ============================================================================

/**
 * @brief Wykonuje komendę — wspólna logika.
 *
 * Komendy jednoznakowe + komendy konfiguracyjne cfg:param:value.
 *
 * @param cmd Komenda jako String
 * @return Opis wyniku
 */
static String executeCommand(const String& cmd) {
    if (cmd == "e") {
        g_mosfet_test_active = false;
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_BLOCK;
        g_bldc_state.fault = false;
        return "BLOCK ON";
    }
    if (cmd == "m2" || cmd == "S") {
        g_mosfet_test_active = false;
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_SINUS;
        g_bldc_state.fault = false;
        // Reset stanu sinusoidalnego — natychmiastowy start sinusa
        g_sine_startup_count = 0;
        g_sine_last_hall_idx = -1;
        g_sine_speed_q16 = 0;
        g_sine_dir = 1;
        g_sine_last_hall_ms = (uint32_t)(esp_timer_get_time() / 1000);
        // Snap kąta do środka aktualnego sektora Halla
        uint8_t hall_now = g_bldc_state.hall_state;
        int8_t sector = (hall_now >= 1 && hall_now <= 6) ? g_hall_to_sector[hall_now] : 0;
        int32_t init_entry = (int32_t)sector * SINE_SECTOR_ENTRIES + SINE_SECTOR_CENTER + g_sine_hall_phase_offset;
        if (init_entry < 0) init_entry += SINE_TABLE_SIZE;
        if (init_entry >= SINE_TABLE_SIZE) init_entry -= SINE_TABLE_SIZE;
        g_sine_angle_q16 = (uint32_t)init_entry << 16;
        g_sine_running = 1;  // od razu sinus, bez block startup
        return "SINUS ON";
    }
    if (cmd == "d") {
        g_mosfet_test_active = false;  // Wyłącz tryb testowy MOSFET
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_DISABLED;
        g_bldc_state.duty_cycle = 0;
        g_bldc_state.duty_target = 0;
        g_duty_ramped = 0;
        allMosfetsOff();
        return "Silnik OFF";
    }
    if (cmd == "r") {
        if (g_bldc_state.mode == DRIVE_MODE_DISABLED) {
            g_bldc_state.direction = (g_bldc_state.direction == DIRECTION_CW)
                ? DIRECTION_CCW : DIRECTION_CW;
            return g_bldc_state.direction == DIRECTION_CW ? "Kierunek: CW" : "Kierunek: CCW";
        }
        return "Najpierw wylacz silnik!";
    }
    if (cmd == "+") {
        g_manual_duty_override = true;
        if (g_bldc_state.duty_target <= PWM_MAX_DUTY - DUTY_STEP)
            g_bldc_state.duty_target += DUTY_STEP;
        else
            g_bldc_state.duty_target = PWM_MAX_DUTY;
        g_duty_ramped = g_bldc_state.duty_target;
        return "Duty: " + String((int)((uint32_t)g_bldc_state.duty_target * 100 / PWM_MAX_DUTY)) + "%";
    }
    if (cmd == "-") {
        g_manual_duty_override = true;
        if (g_bldc_state.duty_target >= DUTY_STEP)
            g_bldc_state.duty_target -= DUTY_STEP;
        else
            g_bldc_state.duty_target = 0;
        g_duty_ramped = g_bldc_state.duty_target;
        return "Duty: " + String((int)((uint32_t)g_bldc_state.duty_target * 100 / PWM_MAX_DUTY)) + "%";
    }
    if (cmd == "R") {
        g_bldc_state.regen_enabled = !g_bldc_state.regen_enabled;
        if (!g_bldc_state.regen_enabled) {
            g_bldc_state.regen_active = false;
            g_regen_active_isr = false;
            g_regen_duty_isr = 0;
        }
        config_get().regen_enabled = g_bldc_state.regen_enabled ? 1 : 0;
        config_save();
        return g_bldc_state.regen_enabled ? "Regen: ON" : "Regen: OFF";
    }
    if (cmd == "b") {
        g_brake_simulated = !g_brake_simulated;
        return g_brake_simulated ? "Hamulec: ON" : "Hamulec: OFF";
    }
    if (cmd == "s") {
        printDiagnostics();
        return "";
    }
    if (cmd == "a") {
        g_autoStatus = !g_autoStatus;
        g_lastAutoStatusMs = millis();
        return g_autoStatus ? "Auto-status: ON" : "Auto-status: OFF";
    }
    if (cmd == "gdbg") {
        g_debugSine = !g_debugSine;
        g_lastDebugSineMs = millis();
        return g_debugSine ? "Debug SINUS: ON" : "Debug SINUS: OFF";
    }
    if (cmd == "man") {
        g_manual_duty_override = !g_manual_duty_override;
        return g_manual_duty_override ? "Manual duty: ON (manetka ignorowana)" : "Manual duty: OFF (manetka aktywna)";
    }
    // Strojenie SINE_HALL_PHASE_OFFSET w runtime (1 wpis = 3.75° elektr.)
    if (cmd == "so") {
        return "Sine offset: " + String((int)g_sine_hall_phase_offset) + " (" + String((float)g_sine_hall_phase_offset * 3.75f, 1) + "°)";
    }
    if (cmd == "so+") {
        int8_t o = g_sine_hall_phase_offset;
        if (o < 47) o += 2;
        g_sine_hall_phase_offset = o;
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "°)";
    }
    if (cmd == "so-") {
        int8_t o = g_sine_hall_phase_offset;
        if (o > -47) o -= 2;
        g_sine_hall_phase_offset = o;
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "°)";
    }
    if (cmd.startsWith("so:")) {
        int val = cmd.substring(3).toInt();
        if (val < -48 || val > 48) return "Zakres: -48..+48 (1 wpis = 3.75°)";
        g_sine_hall_phase_offset = (int8_t)val;
        return "Sine offset: " + String(val) + " (" + String((float)val * 3.75f, 1) + "°)";
    }
    // Auto-tune fazy sinusoidalnej
    if (cmd == "sat" || cmd.startsWith("sat:")) {
        if (g_atune_state != ATUNE_IDLE) {
            g_atune_state = ATUNE_IDLE;
            g_sine_hall_phase_offset = g_atune_saved_offset;
            g_manual_duty_override = false;
            return "[SAT] Anulowano. Offset przywrócony: " + String((int)g_atune_saved_offset);
        }
        if (g_bldc_state.mode != DRIVE_MODE_SINUS) {
            return "[SAT] Wymaga trybu SINUS! Użyj S lub m2 aby włączyć.";
        }
        // Opcjonalnie: sat:MIN:MAX:STEP  np. sat:-16:16:4
        if (cmd.startsWith("sat:")) {
            // Parsuj parametry sat:min:max[:step]
            String params = cmd.substring(4);
            int c1 = params.indexOf(':');
            if (c1 > 0) {
                int mn = params.substring(0, c1).toInt();
                String rest = params.substring(c1 + 1);
                int c2 = rest.indexOf(':');
                int mx, st = 2;
                if (c2 > 0) {
                    mx = rest.substring(0, c2).toInt();
                    st = rest.substring(c2 + 1).toInt();
                } else {
                    mx = rest.toInt();
                }
                if (mn < -48) mn = -48;
                if (mx > 48) mx = 48;
                if (st < 1) st = 1;
                if (st > 16) st = 16;
                if (mn >= mx) return "[SAT] Błąd: min >= max";
                g_atune_offset_min = (int8_t)mn;
                g_atune_offset_max = (int8_t)mx;
                g_atune_offset_step = (int8_t)st;
            }
        } else {
            // Domyślne: -24..+24 krok 2
            g_atune_offset_min = -24;
            g_atune_offset_max = 24;
            g_atune_offset_step = 2;
        }
        g_atune_state = ATUNE_INIT;
        return "";
    }
    if (cmd == "P") {
        printDisplayConfig();
        return "";
    }
    if (cmd == "h") {
        printHelp();
        return "";
    }

    // --- Test MOSFET: komendy tXX ---
    if (cmd == "t") {
        mosfetTestPrintHelp();
        return "";
    }
    if (cmd == "t0") {
        g_mosfet_test_active = false;
        allMosfetsOff();
        g_bldc_state.mode = DRIVE_MODE_DISABLED;
        return "Test MOSFET: OFF — wszystkie tranzystory wyłączone";
    }
    if (cmd.startsWith("tp:")) {
        int pct = cmd.substring(3).toInt();
        return mosfetTestSetDuty(pct);
    }
    if (cmd.startsWith("t") && cmd.length() == 3) {
        return mosfetTestSet(cmd);
    }

    // Komendy konfiguracyjne: cfg / cfg:param:value
    if (cmd == "cfg") {
        controller_config_t& cfg = config_get();
        const char* modeNames[] = {"DISABLED", "BLOCK", "SINUS", "FOC"};
        const char* modeName = (cfg.drive_mode <= DRIVE_MODE_FOC) ? modeNames[cfg.drive_mode] : "???";
        Serial.println();
        Serial.println("========== KONFIGURACJA NVS ==========");
        Serial.printf("drive_mode:    %d (%s)\n", cfg.drive_mode, modeName);
        Serial.printf("ramp_time_ms:  %d ms\n",   cfg.ramp_time_ms);
        Serial.printf("regen_enabled: %d (%s)\n", cfg.regen_enabled, cfg.regen_enabled ? "ON" : "OFF");
        Serial.printf("magic:         0x%08X %s\n", cfg.magic, (cfg.magic == CONFIG_MAGIC) ? "OK" : "BAD!");
        Serial.printf("version:       %d\n",      cfg.version);
        Serial.println("======================================");
        // Runtime (bie\u017c\u0105ce, mog\u0105 r\u00f3\u017cni\u0107 si\u0119 od NVS):
        Serial.println("--- Runtime (bie\u017c\u0105ce) ---");
        const char* rtMode = (g_bldc_state.mode <= DRIVE_MODE_FOC) ? modeNames[g_bldc_state.mode] : "???";
        Serial.printf("mode:          %s\n", rtMode);
        Serial.printf("ramp_time_ms:  %d ms\n", g_bldc_state.ramp_time_ms);
        Serial.printf("regen:         %s\n", g_bldc_state.regen_enabled ? "ON" : "OFF");
        Serial.printf("direction:     %s\n", (g_bldc_state.direction == DIRECTION_CW) ? "CW" : "CCW");
        Serial.printf("sine_offset:   %d (%.1f\u00b0)\n", (int)g_sine_hall_phase_offset, (float)g_sine_hall_phase_offset * 3.75f);
        Serial.println();
        return "";
    }
    if (cmd.startsWith("cfg:")) {
        controller_config_t& cfg = config_get();
        if (cmd.startsWith("cfg:mode:")) {
            int val = cmd.substring(9).toInt();
            if (val >= DRIVE_MODE_BLOCK && val <= DRIVE_MODE_FOC) {
                cfg.drive_mode = (uint8_t)val;
                config_save();
                const char* names[] = {"OFF", "BLOCK", "SINUS", "FOC"};
                return String("Tryb boot: ") + names[val];
            }
            return "Bledna wartosc trybu";
        }
        if (cmd.startsWith("cfg:ramp:")) {
            int val = cmd.substring(9).toInt();
            if (val >= 0 && val <= 10000) {
                cfg.ramp_time_ms = (uint16_t)val;
                g_bldc_state.ramp_time_ms = (uint16_t)val;
                config_save();
                return "Rampa: " + String(val) + " ms";
            }
            return "Zakres 0-10000 ms";
        }
        if (cmd.startsWith("cfg:regen:")) {
            int val = cmd.substring(10).toInt();
            cfg.regen_enabled = val ? 1 : 0;
            g_bldc_state.regen_enabled = (val != 0);
            if (!g_bldc_state.regen_enabled) {
                g_bldc_state.regen_active = false;
                g_regen_active_isr = false;
                g_regen_duty_isr = 0;
            }
            config_save();
            return val ? "Regen: ON" : "Regen: OFF";
        }
        return "Nieznany parametr cfg";
    }

    // Numeryczna wartość duty w %
    bool isNum = true;
    for (unsigned int i = 0; i < cmd.length(); i++) {
        if (cmd[i] < '0' || cmd[i] > '9') { isNum = false; break; }
    }
    if (isNum && cmd.length() > 0) {
        int pct = cmd.toInt();
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        g_manual_duty_override = true;
        g_bldc_state.duty_target = (uint16_t)((uint32_t)pct * PWM_MAX_DUTY / 100);
        g_duty_ramped = g_bldc_state.duty_target;
        return "Duty: " + String(pct) + "%";
    }

    return "Nieznana komenda: " + cmd;
}


