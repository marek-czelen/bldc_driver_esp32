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

// Pomiar RPM z pinu SPEED (GPIO ISR, dla silników przekładniowych, P07==1)
volatile uint32_t g_speed_last_pulse_us = 0;  ///< micros() ostatniego impulsu SPEED
volatile uint32_t g_speed_period_us = 0;      ///< Okres między impulsami SPEED [µs]

// PAS (Pedal Assist Sensor) — pomiar kadencji z przerwania GPIO
volatile uint32_t g_pas_last_pulse_us = 0;    ///< micros() ostatniego impulsu PAS
volatile uint32_t g_pas_period_us = 0;        ///< Okres między impulsami PAS [µs]

/// Timeout PAS: jeśli >1.5s bez impulsu → kadencja = 0 (użytkownik przestał pedałować)
#define PAS_TIMEOUT_US          1500000
/// Minimalny okres PAS: odrzucaj impulsy szybsze niż 200 RPM × max magnesów
/// (200 RPM × 24 magnesy = 4800 pulsów/min = 80 Hz → min 12.5ms)
#define PAS_MIN_PERIOD_US       10000
/// Maksymalna kadencja [RPM] zanim obetniemy (ochrona przed szumem)
#define PAS_MAX_CADENCE_RPM     150

// Regeneracja — zmienne volatile dla ISR
volatile bool g_regen_active_isr = false;     ///< Tryb regen aktywny (do ISR)
volatile uint16_t g_regen_duty_isr = 0;       ///< Siła hamowania regen 0-PWM_MAX_DUTY

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
static String executeCommand(const String& cmd);

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

// Auto-status
static bool g_autoStatus = false;
static unsigned long g_lastAutoStatusMs = 0;
static const unsigned long AUTO_STATUS_INTERVAL_MS = 1000;

// Bufor na komendy numeryczne
static String serialBuffer = "";

// Symulacja hamulca komendą Serial
static bool g_brake_simulated = false;

// Rampa rozpędzania silnika
static uint16_t g_duty_ramped = 0;                   ///< Aktualny duty po rampie
static unsigned long g_ramp_last_us = 0;             ///< Timestamp ostatniego kroku rampy [µs]

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
 *
 * @return Kadencja w RPM (0 = nie pedałuje / timeout)
 */
static uint16_t calculatePasCadence() {
    uint32_t period = g_pas_period_us;       // volatile → local copy
    uint32_t last   = g_pas_last_pulse_us;
    uint32_t now_us = (uint32_t)esp_timer_get_time();

    // Timeout: brak impulsu → nie pedałuje
    if (last == 0 || period == 0 || (now_us - last) > PAS_TIMEOUT_US) {
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
 * @brief Oblicza duty PAS na podstawie kadencji, czułości, prędkości i wspomagania.
 *
 * Algorytm łączy 3 czynniki (każdy 0.0–1.0):
 *
 * 1. **Cadence Factor**: Jak szybko użytkownik pedałuje vs próg z P11 (sensitivity).
 *    P11=1 (niska czułość) → potrzebujesz wysokiej kadencji dla pełnej mocy.
 *    P11=24 (wysoka czułość) → niska kadencja daje pełną moc.
 *    Próg kadencji = map(P11, 1→24, 80→15 RPM).
 *    Factor = min(1.0, cadence/threshold).
 *
 * 2. **Speed Limiter**: Redukuje moc przy zbliżaniu się do limitu prędkości (P08).
 *    Od 90% limitu zaczyna ściąganie, na 100% → 0%.
 *    Factor = 1.0 poniżej 90%, liniowy spadek do 0.0 na limicie.
 *
 * 3. **Start Strength (P12)**: Minimalne duty na starcie pedałowania.
 *    P12=0 → brak boost, P12=5 → do 20% min duty.
 *    Przyspieszenie startu kiedy kadencja jest niska ale > 0.
 *
 * Wynik: pas_duty = cadence_factor × speed_factor × maxDuty  (min clamped by P12)
 *
 * @param maxDuty  Maksymalne duty z poziomu wspomagania (getAssistMaxDuty)
 * @return duty PAS 0–maxDuty
 */
static uint16_t calculatePasDuty(uint16_t maxDuty) {
    if (maxDuty == 0) return 0;

    uint16_t cadence = g_bldc_state.pas_cadence_rpm;
    if (cadence == 0) return 0;

    // --- 1. Cadence Factor (0.0 – 1.0) ---
    // P11: czułość PAS 1-24 (domyślnie 12)
    uint8_t p11 = g_display.config.p11_pas_sensitivity;
    if (p11 == 0) p11 = 12;  // fallback
    if (p11 > 24) p11 = 24;

    // Próg kadencji [RPM] dla pełnej mocy:
    //   P11=1  → 80 RPM (trzeba szybko kręcić do pełnej mocy)
    //   P11=12 → ~47 RPM (średnia czułość)
    //   P11=24 → 15 RPM (bardzo lekkie pedałowanie = pełna moc)
    uint16_t cadence_threshold = (uint16_t)map((long)p11, 1, 24, 80, 15);
    if (cadence_threshold < 10) cadence_threshold = 10;

    float cadence_factor;
    if (cadence >= cadence_threshold) {
        cadence_factor = 1.0f;
    } else {
        cadence_factor = (float)cadence / (float)cadence_threshold;
    }

    // --- 2. Speed Limiter (0.0 – 1.0) ---
    float speed_factor = 1.0f;
    uint8_t speed_limit_kmh = g_display.config.p08_speed_limit;
    if (speed_limit_kmh > 0) {
        float speed_kmh = g_bldc_state.wheel_speed_kmh;
        float limit_90pct = (float)speed_limit_kmh * 0.9f;
        if (speed_kmh >= (float)speed_limit_kmh) {
            speed_factor = 0.0f;
        } else if (speed_kmh > limit_90pct) {
            speed_factor = ((float)speed_limit_kmh - speed_kmh)
                         / ((float)speed_limit_kmh - limit_90pct);
        }
    }

    // --- 3. Oblicz duty PAS ---
    float pas_duty_f = cadence_factor * speed_factor * (float)maxDuty;

    // --- 4. Start Strength (P12) — minimalne duty na starcie ---
    // P12=0 → 0%, P12=1 → 4%, ..., P12=5 → 20% maxDuty
    uint8_t p12 = g_display.config.p12_pas_start_strength;
    if (p12 > 5) p12 = 5;
    if (p12 > 0 && cadence > 0) {
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
 * @brief ISR przerwania GPIO na pinie PAS (FALLING edge).
 *
 * Czujnik PAS (Hall sensor) generuje impulsy w czasie pedałowania.
 * Liczba impulsów na obrót korby = P13 (magnesy PAS).
 * Mierzymy okres między impulsami, z tego obliczamy kadencję.
 *
 * Filtr: ignoruj impulsy szybsze niż PAS_MIN_PERIOD_US (debounce + szum).
 */
void IRAM_ATTR onPasPulse() {
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (g_pas_last_pulse_us > 0) {
        uint32_t period = now_us - g_pas_last_pulse_us;
        if (period >= PAS_MIN_PERIOD_US) {
            g_pas_period_us = period;
            g_pas_last_pulse_us = now_us;
        }
        // Impulsy szybsze niż PAS_MIN_PERIOD_US → ignoruj (szum/bouncing)
    } else {
        g_pas_last_pulse_us = now_us;
    }
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

    // PAS (Pedal Assist Sensor) — przerwanie na FALLING edge
    attachInterrupt(digitalPinToInterrupt(PIN_PAS), onPasPulse, FALLING);
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
    if (g_bldc_state.mode != DRIVE_MODE_DISABLED) {
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

        if (p07 <= 1) {
            // P07==0 lub P07==1: czujnik zewnętrzny SPEED (1 magnes na koło)
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
    g_bldc_state.throttle_raw = analogRead(PIN_THROTTLE);

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

    // PAS: kadencja i duty
    if (g_bldc_state.pas_cadence_rpm > 0) {
        int pasDutyPct = (int)((uint32_t)g_bldc_state.pas_duty * 100 / PWM_MAX_DUTY);
        Serial.printf("CAD:%u/%d%% ", g_bldc_state.pas_cadence_rpm, pasDutyPct);
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
    if (hall != g_hall_prev_isr) {
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        if (g_hall_last_change_us > 0) {
            g_hall_period_us = now_us - g_hall_last_change_us;
        }
        g_hall_last_change_us = now_us;
        g_hall_prev_isr = hall;
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

    // Dispatch trybu sterowania — obecnie zaimplementowany tylko BLOCK
    // Aby dodać SINUS/FOC: zamień poniższy if na switch(g_mode_isr) z nowymi case'ami
    if (g_mode_isr != DRIVE_MODE_BLOCK) {
        // Tryb niezaimplementowany → bezpieczny stan (wszystkie FETy OFF)
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
    Serial.println("========== KOMENDY ==========");
    Serial.println("e         Włącz tryb BLOCK sterowania");
    Serial.println("d         Wyłącz silnik (duty=0)");
    Serial.println("r         Odwróć kierunek (CW/CCW) - tylko gdy wyłączony");
    Serial.println("+/-       Zmień duty o ±5%");
    Serial.println("0-100     Ustaw duty w procentach (wpisz liczbę i Enter)");
    Serial.println("R         Regeneracja ON/OFF (hamowanie rekuperacyjne)");
    Serial.println("b         Symulacja hamulca ON/OFF");
    Serial.println("P         Pokaż parametry wyświetlacza P01-P20");
    Serial.println("s         Pokaż status");
    Serial.println("a         Auto-status co 1s ON/OFF");
    Serial.println("h         Pokaż tę pomoc");
    Serial.println("---------- Konfiguracja NVS ----------");
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

/**
 * @brief Przetwarza wszystkie dostępne bajty z bufora Serial.
 *
 * Komendy jednoznakowe (e, d, r, +, -, s, a, h, R, b, P) są obsługiwane natychmiast
 * (bez Enter). Pozostałe znaki buforowane do Enter — wtedy przekazywane do
 * wspólnej executeCommand() (obsługuje też cfg:param:value i numeryczne duty).
 *
 * Komendy:
 * | Znak   | Akcja |
 * |--------|-------|
 * | e      | Włącz BLOCK mode |
 * | d      | Wyłącz silnik, duty=0 |
 * | r      | Odwróć kierunek (tylko gdy off) |
 * | +/-    | Duty ±5% |
 * | 0-100  | Ustaw duty w % (Enter) |
 * | R      | Regeneracja ON/OFF |
 * | b      | Symulacja hamulca ON/OFF |
 * | P      | Pokaż parametry wyświetlacza P01-P20 |
 * | s      | Jednorazowy status |
 * | a      | Toggle auto-status co 1 s |
 * | h      | Pomoc |
 * | cfg:.. | Komendy konfiguracyjne (Enter) |
 */
void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        // Komendy jednoznakowe — natychmiast, bez Enter
        const char* immediateCmds = "edrs+-haRbP";
        bool isImmediate = false;
        for (const char* p = immediateCmds; *p; p++) {
            if (c == *p) { isImmediate = true; break; }
        }

        if (isImmediate && serialBuffer.length() == 0) {
            String result = executeCommand(String(c));
            if (result.length() > 0) {
                Serial.printf("[CMD] %s\n", result.c_str());
            }
            continue;
        }

        // Buforowanie do Enter
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
        g_bldc_state.mode = DRIVE_MODE_BLOCK;
        g_bldc_state.fault = false;
        return "BLOCK ON";
    }
    if (cmd == "d") {
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
        if (g_bldc_state.duty_target <= PWM_MAX_DUTY - DUTY_STEP)
            g_bldc_state.duty_target += DUTY_STEP;
        else
            g_bldc_state.duty_target = PWM_MAX_DUTY;
        g_duty_ramped = g_bldc_state.duty_target;
        return "Duty: " + String((int)((uint32_t)g_bldc_state.duty_target * 100 / PWM_MAX_DUTY)) + "%";
    }
    if (cmd == "-") {
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
    if (cmd == "P") {
        printDisplayConfig();
        return "";
    }
    if (cmd == "h") {
        printHelp();
        return "";
    }

    // Komendy konfiguracyjne: cfg:param:value
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
        g_bldc_state.duty_target = (uint16_t)((uint32_t)pct * PWM_MAX_DUTY / 100);
        g_duty_ramped = g_bldc_state.duty_target;
        return "Duty: " + String(pct) + "%";
    }

    return "Nieznana komenda: " + cmd;
}


