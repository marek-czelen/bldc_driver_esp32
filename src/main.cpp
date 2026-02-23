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
void blockCommutate(uint8_t hallState, uint16_t duty);
void processSerialCommands();

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

// Wyświetlacz S866
static s866_display_t g_display;
static bool g_displayEnabled = false;

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
    Serial.println("  Wersja: 0.1.0 (Hardware Test)");
    Serial.println("==========================================");
    Serial.println();

    // Inicjalizacja stanu
    memset(&g_bldc_state, 0, sizeof(bldc_state_t));
    g_bldc_state.mode = DRIVE_MODE_DISABLED;

    // Inicjalizacja GPIO
    initGPIO();
    Serial.println("[OK] GPIO zainicjalizowane");

    // Inicjalizacja PWM
    initPWM();
    Serial.println("[OK] PWM zainicjalizowane");

    // Upewnienie się, że wszystkie MOSFETy są wyłączone
    allMosfetsOff();
    Serial.println("[OK] Wszystkie MOSFETy wyłączone (stan bezpieczny)");

    // Timer sprzętowy do komutacji (co 50 us = 20 kHz)
    initCommutationTimer();
    Serial.println("[OK] Timer komutacji uruchomiony (20 kHz)");

    Serial.println();
    Serial.println("Komendy Serial:");
    Serial.println("  e     - włącz tryb BLOCK");
    Serial.println("  d     - wyłącz silnik");
    Serial.println("  r     - odwróć kierunek");
    Serial.println("  +/-   - duty +/- 5%");
    Serial.println("  0-100 - duty w % (Enter)");
    Serial.println("  X     - włącz/wyłącz wyświetlacz S866");
    Serial.println("  s     - pokaż status");
    Serial.println("  a     - auto-status co 1s ON/OFF");
    Serial.println("  h     - pomoc");
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
 * 3. Mapowanie przepustnicy → duty (gdy BLOCK mode)
 * 4. Zapis stanu do zmiennych volatile (dla ISR)
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

    // Przepustnica sprzętowa -> duty
    if (g_bldc_state.mode == DRIVE_MODE_BLOCK) {
        uint16_t thr = g_bldc_state.throttle_raw;
        if (thr < THROTTLE_DEAD_ZONE) {
            g_bldc_state.duty_cycle = 0;
        } else {
            if (thr > THROTTLE_MAX_RAW) thr = THROTTLE_MAX_RAW;
            if (thr < THROTTLE_MIN_RAW) thr = THROTTLE_MIN_RAW;
            g_bldc_state.duty_cycle = map(thr, THROTTLE_MIN_RAW, THROTTLE_MAX_RAW, 0, PWM_MAX_DUTY);
        }

        // Wyświetlacz S866: assist level limituje max duty
        if (g_displayEnabled && g_display.connected) {
            uint16_t maxDuty = 0;
            if (g_display.rx.assist_level > 0) {
                // Poziomy 1-5 → 20/40/60/80/100% max duty (>5 = 100%)
                maxDuty = (uint16_t)((uint32_t)g_display.rx.assist_level * PWM_MAX_DUTY / 5);
                if (maxDuty > PWM_MAX_DUTY) maxDuty = PWM_MAX_DUTY;
            }
            if (g_bldc_state.duty_cycle > maxDuty) {
                g_bldc_state.duty_cycle = maxDuty;
            }
        }
    }

    // Aktualizacja zmiennych ISR z głównego stanu
    g_hall_isr = g_bldc_state.hall_state;
    g_duty_isr = g_bldc_state.duty_cycle;
    g_motor_enabled = (g_bldc_state.mode == DRIVE_MODE_BLOCK) && (g_bldc_state.duty_cycle > 0);
    g_brake_isr = g_bldc_state.brake_active;
    g_direction_isr = g_bldc_state.direction;

    // Obsługa wyświetlacza S866 (Serial2 — niezależny od USB)
    if (g_displayEnabled) {
        // Aktualizuj dane TX dla wyświetlacza
        g_display.tx.error = g_bldc_state.fault ? 1 : 0;
        g_display.tx.brake_active = g_bldc_state.brake_active ? 1 : 0;

        // Prąd: maksimum z 3 faz, w jednostkach 0.1A
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        g_display.tx.current_x10 = (uint16_t)(maxI * 10.0f);

        // Czas obrotu koła [ms] — wymaga pomiaru RPM (na razie 0 = stoi)
        g_display.tx.wheeltime_ms = 0;
        if (g_bldc_state.rpm > 0) {
            // wheeltime = 60000 / RPM (ms na obrót)
            g_display.tx.wheeltime_ms = (uint16_t)(60000UL / g_bldc_state.rpm);
        }

        // Obsługa protokołu wyswietlacza
        s866_service(&g_display);

        // Auto-powrót jeśli wyświetlacz się rozłączył
        if (!g_display.connected && (millis() - g_display.last_valid_ms > S866_TIMEOUT_MS * 2)) {
            g_displayEnabled = false;
            s866_deinit();
            Serial.println("[S866] Wyświetlacz rozłączony — wyłączam Serial2");
        }
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

    // --- Prędkość ---
    pinMode(PIN_SPEED, OUTPUT);
    digitalWrite(PIN_SPEED, LOW);

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
 * Oba piny są INPUT_PULLUP — aktywny sygnał = LOW (przycisk do GND).
 * Hamulec ma priorytet — jest też sprawdzany w ISR (g_brake_isr).
 */
void readDigitalInputs() {
    g_bldc_state.brake_active = (digitalRead(PIN_BRAKE) == LOW);
    g_bldc_state.pas_active = (digitalRead(PIN_PAS) == LOW);
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
    int thrPct = 0;
    if (g_bldc_state.throttle_raw > THROTTLE_DEAD_ZONE) {
        uint16_t thr = g_bldc_state.throttle_raw;
        if (thr > THROTTLE_MAX_RAW) thr = THROTTLE_MAX_RAW;
        thrPct = (int)((uint32_t)(thr - THROTTLE_DEAD_ZONE) * 100 / (THROTTLE_MAX_RAW - THROTTLE_DEAD_ZONE));
        if (thrPct > 100) thrPct = 100;
    }
    Serial.printf("%s %s D:%d%% V:%.1f Ia:%.2f Ib:%.2f Ic:%.2f H:%d%d%d T:%d Thr:%d%%(%d) %s%s%s",
        modeNames[g_bldc_state.mode],
        g_bldc_state.direction == DIRECTION_CW ? "CW" : "CCW",
        dutyPct,
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
        g_bldc_state.brake_active ? "BRK " : "",
        g_bldc_state.pas_active ? "PAS " : "",
        g_bldc_state.fault ? "FAULT " : "");

    // Informacje z wyświetlacza S866
    Serial.print(g_displayEnabled ? "DISP:EN " : "DISP:OFF ");
    if (g_displayEnabled) {
        if (g_display.connected) {
            Serial.printf("L%d %s%s",
                g_display.rx.assist_level,
                g_display.rx.headlight ? "HL " : "",
                g_display.rx.cruise_control ? "CC " : "");
        } else {
            Serial.print("L-- ");
        }
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
    if (g_brake_isr) {
        // Hamulec - natychmiast wyłącz
        ledcWrite(PWM_CHANNEL_A_HIGH, 0);
        ledcWrite(PWM_CHANNEL_B_HIGH, 0);
        ledcWrite(PWM_CHANNEL_C_HIGH, 0);
        ledcWrite(PWM_CHANNEL_A_LOW, PWM_MAX_DUTY);
        ledcWrite(PWM_CHANNEL_B_LOW, PWM_MAX_DUTY);
        ledcWrite(PWM_CHANNEL_C_LOW, PWM_MAX_DUTY);
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

    // Odczyt Halli bezpośrednio w ISR (szybkie GPIO read)
    uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
    uint8_t hb = (GPIO.in >> PIN_HALL_SENSOR_B) & 1;
    uint8_t hc = (GPIO.in >> PIN_HALL_SENSOR_C) & 1;
    uint8_t hall = (hc << 2) | (hb << 1) | ha;

    uint16_t d = g_duty_isr;

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
    Serial.println("X         Włącz/wyłącz wyświetlacz S866 (9600 baud)");
    Serial.println("s         Pokaż status");
    Serial.println("a         Auto-status co 1s ON/OFF");
    Serial.println("h         Pokaż tę pomoc");
    Serial.println("===========================");
    Serial.println();
}

/**
 * @brief Przetwarza wszystkie dostępne bajty z bufora Serial.
 *
 * Komendy jednoznakowe (e, d, r, +, -, s, a, h) są obsługiwane natychmiast.
 * Liczby (0–100) są buforowane w serialBuffer aż do naciśnięcia Enter (\n/\r),
 * po czym ustawiają duty_cycle w procentach.
 *
 * Wbudowany throttle (ADC) nadpisuje duty ustawione przez UART w każdym loop(),
 * więc komendy +/- i numeryczne działają tylko gdy przepustnica jest w pozycji 0
 * lub gdy tryb to DRIVE_MODE_DISABLED.
 *
 * Komendy:
 * | Znak   | Akcja |
 * |--------|-------|
 * | e      | Włącz BLOCK mode |
 * | d      | Wyłącz silnik, duty=0 |
 * | r      | Odwróć kierunek (tylko gdy off) |
 * | +/-    | Duty ±5% |
 * | 0-100  | Ustaw duty w % (Enter) |
 * | s      | Jednorazowy status |
 * | a      | Toggle auto-status co 1 s |
 * | h      | Pomoc |
 */
void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        // Komendy jednoznakowe (natychmiast)
        switch (c) {
            case 'e':
                g_bldc_state.mode = DRIVE_MODE_BLOCK;
                g_bldc_state.fault = false;
                Serial.printf("[CMD] BLOCK ON  duty=%d%%\n",
                    (int)((uint32_t)g_bldc_state.duty_cycle * 100 / PWM_MAX_DUTY));
                serialBuffer = "";
                continue;
            case 'd':
                g_bldc_state.mode = DRIVE_MODE_DISABLED;
                g_bldc_state.duty_cycle = 0;
                allMosfetsOff();
                Serial.println("[CMD] Silnik OFF");
                serialBuffer = "";
                continue;
            case 'r':
                if (g_bldc_state.mode == DRIVE_MODE_DISABLED) {
                    g_bldc_state.direction = (g_bldc_state.direction == DIRECTION_CW)
                        ? DIRECTION_CCW : DIRECTION_CW;
                    Serial.printf("[CMD] Kierunek: %s\n",
                        g_bldc_state.direction == DIRECTION_CW ? "CW" : "CCW");
                } else {
                    Serial.println("[CMD] Najpierw 'd' przed zmianą kierunku!");
                }
                serialBuffer = "";
                continue;
            case '+':
                if (g_bldc_state.duty_cycle <= PWM_MAX_DUTY - DUTY_STEP)
                    g_bldc_state.duty_cycle += DUTY_STEP;
                else
                    g_bldc_state.duty_cycle = PWM_MAX_DUTY;
                Serial.printf("[CMD] Duty: %d%%\n",
                    (int)((uint32_t)g_bldc_state.duty_cycle * 100 / PWM_MAX_DUTY));
                serialBuffer = "";
                continue;
            case '-':
                if (g_bldc_state.duty_cycle >= DUTY_STEP)
                    g_bldc_state.duty_cycle -= DUTY_STEP;
                else
                    g_bldc_state.duty_cycle = 0;
                Serial.printf("[CMD] Duty: %d%%\n",
                    (int)((uint32_t)g_bldc_state.duty_cycle * 100 / PWM_MAX_DUTY));
                serialBuffer = "";
                continue;
            case 's':
                printDiagnostics();
                serialBuffer = "";
                continue;
            case 'h':
                printHelp();
                serialBuffer = "";
                continue;
            case 'a':
                g_autoStatus = !g_autoStatus;
                g_lastAutoStatusMs = millis();
                Serial.printf("[CMD] Auto-status: %s\n", g_autoStatus ? "ON" : "OFF");
                serialBuffer = "";
                continue;
            case 'X':
                if (!g_displayEnabled) {
                    g_displayEnabled = true;
                    memset(&g_display, 0, sizeof(g_display));
                    g_display.last_valid_ms = millis();  // grace period na pierwszą ramkę
                    g_bldc_state.mode = DRIVE_MODE_BLOCK;
                    g_bldc_state.fault = false;
                    Serial.println("[CMD] Wyświetlacz S866 ON (Serial2: GPIO4/GPIO16, 9600 baud)");
                    s866_init();
                } else {
                    g_displayEnabled = false;
                    s866_deinit();
                    Serial.println("[CMD] Wyświetlacz S866 OFF");
                }
                serialBuffer = "";
                continue;
            default:
                break;
        }

        // Buforowanie cyfr + Enter -> ustawienie duty w %
        if (c >= '0' && c <= '9') {
            serialBuffer += c;
        } else if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                int pct = serialBuffer.toInt();
                if (pct < 0) pct = 0;
                if (pct > 100) pct = 100;
                g_bldc_state.duty_cycle = (uint16_t)((uint32_t)pct * PWM_MAX_DUTY / 100);
                Serial.printf("[CMD] Duty: %d%% (%d/%d)\n",
                    pct, g_bldc_state.duty_cycle, PWM_MAX_DUTY);
                serialBuffer = "";
            }
        } else {
            serialBuffer = "";
        }
    }
}
