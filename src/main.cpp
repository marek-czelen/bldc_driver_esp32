/**
 * @file main.cpp
 * @brief BLDC Motor Driver — ESP32
 * @version 2.0.0
 *
 * Sterownik silnika BLDC (bezszczotkowego) 3-fazowego na ESP32.
 * Obsługiwane metody sterowania:
 *   - Komutacja blokowa (6-step / trapezoidalna)
 *   - Komutacja sinusoidalna (ciągłe śledzenie kąta z LUT 97-wpisów)
 *   - FOC (Field Oriented Control) — inverse Park/Clarke + SVPWM
 *
 * ## Architektura
 * Komutacja odbywa się w ISR MCPWM TEZ (Timer Equals Zero, 20 kHz),
 * niezależnie od loop(). Center-aligned PWM (UP_DOWN counter) zapewnia
 * synchroniczny pomiar prądu w dolinie PWM (wszystkie low-side ON).
 * Dzięki temu Serial.printf() i inne wolne operacje w loop() nie powodują
 * zakłóceń w sterowaniu silnikiem.
 *
 * ## Filtracja wejść
 * - PAS: timer próbkujący co 500µs + filtr cyfrowy (N zgodnych próbek)
 * - Przepustnica: bufor kołowy 1 próbka/loop + mediana + outlier rejection
 * - Hamulec: debounce z licznikiem kolejnych odczytów
 * - Halle silnika: debounce w ISR komutacji (HALL_MIN_PERIOD_US)
 *
 * ## PWM: MCPWM center-aligned (v2.0)
 * Migracja z LEDC (edge-aligned, 10-bit) na MCPWM (center-aligned, period=500).
 * Trzy operatory × 2 generatory (gen_A=HIN, gen_B=LIN) sterowane bezpośrednio
 * przez zapisy rejestrów LL w ISR (~5 ns, bez spinlocków driver API).
 * Prescaler: group=1, timer=8 → 20 MHz, period=500 → dokładnie 20 kHz.
 * Shadow compare: TEZ-only → symetryczne impulsy center-aligned.
 * Dead time: bypass (IR2103 wewnętrzny ~520 ns).
 *
 * ## Sterowniki mostu
 * IR2103: HIN=active HIGH (high-side ON gdy HIGH),
 *         LIN=active LOW (low-side ON gdy LOW — logika odwrócona!).
 * MCPWM center-aligned: gen_A(HIN) i gen_B(LIN) sterowane niezależnie.
 * SINUS/FOC: oba generatory = ten sam duty (IR2103 complementary + dead time).
 * BLOCK: gen_A=PWM, gen_B=force HIGH/LOW zależnie od stanu fazy.
 *
 * ## Przepływ danych
 * loop() czyta ADC/Hall/GPIO → aktualizuje zmienne volatile → ISR MCPWM odczytuje je
 * w każdym przerwaniu TEZ i ustawia odpowiednie generatory MCPWM.
 *
 * ## Konfiguracja
 * NVS (EEPROM): 64-bajtowa struktura controller_config_t (CONFIG_VERSION=16).
 * Interfejs: Serial 115200 baud + WiFi AP (P17=1) z responsywnym UI.
 *
 * ## Hardware
 * - MCU: ESP32-D0WDQ6, 240 MHz, Arduino via PlatformIO
 * - Gate drivers: 3x IR2103
 * - Pomiar prądu: INA180A2 (gain=50 V/V) + shunt 2 mΩ
 * - Czujniki Halla: GPIO5(A)/18(B)/19(C), INPUT_PULLUP
 * - VBAT dzielnik: 1.13 MΩ / 31.7 kΩ
 * - Przepustnica: GPIO2, ADC 400-2600 RAW → 0-100%
 * - Wyświetlacz: S866 protokół 2, Serial2 9600 baud
 * - PAS: GPIO22, timer sampling 2 kHz + filtr cyfrowy
 */

#include <Arduino.h>
#include "pinout.h"
#include "bldc_types.h"
#include "display_s866.h"
#include "bldc_config.h"
#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "hal/mcpwm_ll.h"
#include "driver/adc.h"           // adc1_config_width/channel_atten
#include "soc/sens_struct.h"      // SENS — direct register access for ISR-safe ADC

// ============================================================================
// Zmienne globalne
// ============================================================================

bldc_state_t g_bldc_state;

// MCPWM ISR handle (zastępuje hw_timer — komutacja w przerwaniu TEZ center-aligned)
static intr_handle_t g_mcpwm_isr_handle = NULL;
/// Flaga: ISR MCPWM przeczytał ADC prądu w dolinie PWM (center)
volatile bool g_adc_ready_isr = false;
/// Akumulator surowych odczytów ADC prądu z ISR (uśrednianie wielu cykli PWM)
volatile uint32_t g_phase_adc_sum_isr[3] = {0, 0, 0};
volatile uint16_t g_phase_adc_cnt_isr = 0;
/// Ostatni surowy odczyt ADC (do diagnostyki idbg)
volatile uint16_t g_phase_adc_raw_isr[3] = {0, 0, 0};
/// Ostatni dobry odczyt ADC per kanał (do zastąpienia outlierów)
volatile uint16_t g_phase_adc_last_good[3] = {0, 0, 0};
/// Spinlock synchronizacji ISR ↔ loop dla akumulatora ADC prądu
static portMUX_TYPE g_adc_mux = portMUX_INITIALIZER_UNLOCKED;
/// Flaga: ISR może czytać ADC1 (false=loop() czyta ADC1, ISR pomija)
volatile bool g_adc_isr_active = false;
/// Licznik timeoutów ADC w ISR (diagnostyka)
volatile uint32_t g_adc_timeout_count = 0;
volatile uint8_t g_hall_isr = 0;
volatile uint16_t g_duty_isr = 0;
volatile bool g_motor_enabled = false;
volatile bool g_brake_isr = false;
volatile drive_mode_t g_mode_isr = DRIVE_MODE_DISABLED;   ///< Tryb sterowania (do ISR)
static uint16_t g_pwm_freq_hz = PWM_FREQUENCY;           ///< Aktualna częstotliwość PWM [Hz]
static uint16_t applyPwmFrequency(uint16_t freq_hz);     ///< Forward declaration

// Debug Hall — śledzenie komutacji w ISR
volatile uint8_t  g_dbg_hall_raw_isr = 0;       ///< Ostatni surowy odczyt Hall z GPIO
volatile uint8_t  g_dbg_hall_filt_isr = 0;      ///< Ostatni przefiltrowany Hall użyty do komutacji
volatile uint8_t  g_dbg_commut_path = 0;        ///< 0=off, 1=block, 2=sinus, 3=foc, 4=block_startup

// Pomiar RPM z przejść Halla (aktualizowane w ISR timera)
volatile uint8_t g_hall_prev_isr = 0;         ///< Poprzedni POTWIERDZONY stan Halla w ISR
volatile uint32_t g_hall_last_change_us = 0;  ///< micros() ostatniego potwierdzonego przejścia
volatile uint32_t g_hall_period_us = 0;       ///< Okres między przejściami Halla [µs]

// Multi-sample Hall confirmation (anty-EMI)
// Nowy stan Halla musi się utrzymać przez HALL_CONFIRM_COUNT kolejnych
// odczytów ISR (N×50µs przy 20kHz) zanim zostanie zaakceptowany.
// Chroni przed glitchami EMI z PWM sprzęgającymi się w linie Halla.
#define HALL_CONFIRM_COUNT      3              ///< Wymagane kolejne zgodne odczyty
static uint8_t  g_hall_candidate = 0;          ///< Kandydat na nowy stan Halla
static uint8_t  g_hall_confirm_cnt = 0;        ///< Ile razy z rzędu widzieliśmy kandydata

// Sinusoidal commutation state — ported from bldc_driver_v2 (STM32, proven algorithm)
// Continuous angle tracking with Hall correction, NOT snap-to-hall approach.
volatile uint32_t g_sine_angle_q16 = 0;       ///< Kąt elektr. w wpisach tabeli (Q16, 0..96<<16)
volatile uint32_t g_sine_speed_q16 = 0;       ///< Prędkość: wpisów tabeli na tick ISR (Q16)
volatile uint32_t g_sine_sector_speed[6] = {0}; ///< Prędkość per-sector (kompensacja nierównych Halli)
volatile uint8_t  g_sine_running = 0;          ///< 1 = tryb sinusoidalny aktywny, 0 = block startup
volatile uint8_t  g_sine_startup_count = 0;    ///< Licznik komutacji blokowych przed przejściem na sinus
volatile int8_t   g_sine_last_hall_idx = -1;   ///< Ostatni indeks Halla w sekwencji (0-5, -1=unknown)
volatile int8_t   g_sine_dir = 1;              ///< Kierunek z przejść Halla: +1 forward, -1 reverse
volatile uint32_t g_sine_last_hall_ms = 0;     ///< HAL tick ostatniego przejścia Halla (stall detection)
volatile int8_t   g_sine_hall_phase_offset = 0; ///< Runtime-tunable Hall phase offset (-48..+48 entries, 1 entry = 3.75°)

// FOC state — regulatory PI i wektory napięciowe
struct foc_pi_t {
    float kp;
    float ki;
    float integral;
    float limit;
};
// Inicjalizacja g_foc_pi_d/q: wartości domyślne ustawiane w setup()
// (SINE_SAFE_MAX_DUTY jeszcze nie zdefiniowany w tym momencie pliku)
static foc_pi_t g_foc_pi_d = {0};
static foc_pi_t g_foc_pi_q = {0};
volatile int32_t g_foc_vd_i = 0;          ///< Napięcie d do ISR (jednostki PWM: -SAFE_MAX..+SAFE_MAX)
volatile int32_t g_foc_vq_i = 0;          ///< Napięcie q do ISR (jednostki PWM)
volatile float g_foc_iq_target = 0.0f;    ///< Docelowy prąd Iq [A] (torque)
// Debug/pomiar FOC (zapisywane w loop, czytane w diagnostyce)
static float g_foc_id_meas = 0.0f;       ///< Zmierzony Id [A]
static float g_foc_iq_meas = 0.0f;       ///< Zmierzony Iq [A]
static float g_foc_vd_dbg = 0.0f;        ///< Kopia Vd do debugowania (float, non-volatile)
static float g_foc_vq_dbg = 0.0f;        ///< Kopia Vq do debugowania (float, non-volatile)
static float g_foc_ia_signed = 0.0f;     ///< Prąd fazy A [A] (surowy, przed klipowaniem)
static float g_foc_ib_signed = 0.0f;     ///< Prąd fazy B [A]
static float g_foc_ic_signed = 0.0f;     ///< Prąd fazy C [A]
// EMA-filtrowane prądy dla FOC (wygładzenie sporadycznych odczytów ADC)
static float g_foc_ia_ema = 0.0f;        ///< EMA Ia [A]
static float g_foc_ib_ema = 0.0f;        ///< EMA Ib [A]
static float g_foc_ic_ema = 0.0f;        ///< EMA Ic [A]
static bool g_foc_debug = false;          ///< Debug FOC (komenda fdbg)
static bool g_foc_voltage_mode = false;   ///< Tryb napięciowy: Vq = duty wprost, bez PI
static unsigned long g_foc_last_debug_ms = 0;
static unsigned long g_foc_last_loop_us = 0;  ///< Timestamp ostatniej iteracji FOC loop

// ── PI Auto-tune (metoda relay / Åström-Hägglund) ──
// Relay feedback: zamiast PI, wyjście przełączane +/-relay_amp gdy błąd >/<0.
// Mierzy oscylacje Iq → oblicza ultimate gain Ku, period Tu → Kp, Ki (Z-N PI).
static bool     g_foc_at_active = false;     ///< Auto-tune w toku
static float    g_foc_at_relay_amp = 30.0f;  ///< Amplituda relay [PWM] (± wokół feedforward)
static uint32_t g_foc_at_start_ms = 0;       ///< Timestamp startu [ms]
static uint32_t g_foc_at_duration_ms = 5000; ///< Czas trwania testu [ms]
static float    g_foc_at_err_prev = 0.0f;    ///< Poprzedni błąd Iq (detekcja zero-crossing)
static uint16_t g_foc_at_crossings = 0;      ///< Liczba przejść przez zero błędu
static uint32_t g_foc_at_first_cross_ms = 0; ///< Timestamp pierwszego zero-crossing
static uint32_t g_foc_at_last_cross_ms = 0;  ///< Timestamp ostatniego zero-crossing
static float    g_foc_at_err_max = 0.0f;     ///< Max błąd w bieżącym pół-cyklu
static float    g_foc_at_err_min = 0.0f;     ///< Min błąd w bieżącym pół-cyklu
static float    g_foc_at_amp_sum = 0.0f;     ///< Suma amplitud oscylacji (do średniej)
static uint16_t g_foc_at_amp_count = 0;      ///< Liczba zmierzonych amplitud

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
volatile int32_t  g_dbg_last_hall_err = 0;     ///< Ostatni błąd korekcji kąta Halla (Q16)
volatile uint32_t g_dbg_snap_count = 0;        ///< Licznik pełnych snapów kąta (desync/crawl)
volatile uint32_t g_dbg_corr_count = 0;        ///< Licznik łagodnych korekcji kąta
volatile uint32_t g_dbg_hall_invalid = 0;      ///< Odrzucone stany 0/7
volatile uint32_t g_dbg_hall_glitch = 0;       ///< Glitche EMI (kandydat zmieniony przed potwierdzeniem)
volatile uint32_t g_dbg_hall_seq_reject = 0;   ///< Odrzucone (zła sekwencja sektora w SINUS/FOC)

// ── Rozszerzona diagnostyka SINUS/FOC (running min/max resetowane co debug print) ──
// Zbierane w ISR, wyświetlane i resetowane w printSineDebug()
volatile int32_t  g_dbg_err_min_q16 = INT32_MAX; ///< Min błąd kąta na Hall edge (Q16) w oknie
volatile int32_t  g_dbg_err_max_q16 = INT32_MIN; ///< Max błąd kąta na Hall edge (Q16) w oknie
volatile uint32_t g_dbg_speed_min = 0xFFFFFFFF; ///< Min prędkość q16 w oknie
volatile uint32_t g_dbg_speed_max = 0;          ///< Max prędkość q16 w oknie
volatile uint32_t g_dbg_dt_min = 0xFFFFFFFF;    ///< Min okres Hall [µs] w oknie
volatile uint32_t g_dbg_dt_max = 0;             ///< Max okres Hall [µs] w oknie
volatile uint32_t g_dbg_snap_window = 0;        ///< Snapy w bieżącym oknie
volatile uint32_t g_dbg_corr_window = 0;        ///< Korekcje w bieżącym oknie
volatile int32_t  g_dbg_angle_at_hall = 0;      ///< Kąt (entry Q16) w momencie Hall (przed korekcją)
volatile int32_t  g_dbg_expected_at_hall = 0;   ///< Oczekiwany kąt (entry Q16) wg tabeli sektorów

// ── Event-driven debug: ring buffer zdarzeń ISR → loop() ──
// ISR wypełnia na zmianę Halla / komutacji, loop() drukuje
#define DBG_EVT_RING_SIZE 64
struct dbg_evt_t {
    uint32_t ts_us;           ///< Timestamp zdarzenia [µs]
    uint8_t  hall_raw;        ///< Surowy Hall [C:B:A] z GPIO.in
    uint8_t  hall_filt;       ///< Przefiltrowany Hall użyty do komutacji
    uint8_t  mode;            ///< drive_mode_t (0=OFF,1=BLK,2=SIN,3=FOC)
    int8_t   sector_old;      ///< Poprzedni sektor (0-5, -1=unknown)
    int8_t   sector_new;      ///< Nowy sektor
    int8_t   dir;             ///< Kierunek: +1 forward, -1 reverse
    uint8_t  startup_cnt;     ///< g_sine_startup_count
    uint32_t angle_q16;       ///< Kąt po korekcji
    uint32_t speed_q16;       ///< Prędkość Q16
    uint32_t hall_period_us;  ///< Okres między edge Halla [µs]
    uint16_t duty;            ///< Duty z przepustnicy/rampy
    int16_t  da, db, dc;      ///< Duty zastosowane na fazach A/B/C
    uint16_t adc_a, adc_b, adc_c; ///< Surowy ADC prądu faz
    int32_t  foc_vd, foc_vq;  ///< FOC: napięcia Vd/Vq (tylko tryb FOC)
    uint8_t  flags;           ///< b0=stalled, b1=snap, b2=seq_reject, b3=reverse
};
volatile dbg_evt_t g_dbg_evt_ring[DBG_EVT_RING_SIZE];
volatile uint8_t g_dbg_evt_wr = 0;      ///< Indeks zapisu (ISR)
volatile uint8_t g_dbg_evt_rd = 0;      ///< Indeks odczytu (loop)
volatile uint32_t g_dbg_evt_overflow = 0; ///< Licznik utraconych zdarzeń
static bool g_debugCommutation = false;   ///< Włączone komendą 'cdbg'

// Duty zastosowane na fazach (ustawiane w sinusCommutateISR/focCommutateISR/block)
volatile int16_t g_dbg_last_da = 0;
volatile int16_t g_dbg_last_db = 0;
volatile int16_t g_dbg_last_dc = 0;

// Pomiar RPM z pinu SPEED (GPIO ISR, dla silników przekładniowych, P07==1)
volatile uint32_t g_speed_last_pulse_us = 0;  ///< micros() ostatniego impulsu SPEED
volatile uint32_t g_speed_period_us = 0;      ///< Okres między impulsami SPEED [µs]
volatile uint32_t g_speed_pulse_count = 0;    ///< Licznik zaakceptowanych impulsów SPEED
volatile uint32_t g_speed_reject_count = 0;   ///< Licznik odrzuconych impulsów (debounce)
#define SPEED_DEBOUNCE_US  30000              ///< Min okres SPEED [µs] (~250 km/h @26")

// Mediana z 3 ostatnich pomiarów okresu SPEED (filtr outlier)
static uint32_t g_speed_period_buf[3] = {0, 0, 0};
static uint8_t  g_speed_period_idx = 0;
static uint8_t  g_speed_period_valid = 0;  ///< Ile pomiarów w buforze (0..3)
static uint32_t g_speed_outlier_count = 0; ///< Ile pomiarów odrzuconych jako outlier (>3x lub <1/3 mediany)
static uint8_t  g_speed_pulses_per_rev = 1; ///< Impulsy SPEED na obrót koła (z NVS, kalibracja: spdcal)
static uint32_t g_speed_last_processed_sp = 0;  ///< Ostatni przetworzony period ISR (skip duplikatów)
static uint32_t g_speed_last_accepted_us = 0;   ///< Timestamp ostatniego zaakceptowanego pomiaru do mediany

// PAS (Pedal Assist Sensor) — pomiar kadencji i kierunku z przerwania GPIO
volatile uint32_t g_pas_last_pulse_us = 0;    ///< micros() ostatniej krawędzi PAS
volatile uint32_t g_pas_period_us = 0;        ///< Pełny okres (HIGH+LOW) PAS [µs]
volatile uint32_t g_pas_rising_us = 0;        ///< Czas ostatniego RISING edge
volatile uint32_t g_pas_falling_us = 0;       ///< Czas ostatniego FALLING edge
volatile uint32_t g_pas_high_time_us = 0;     ///< Czas trwania stanu HIGH [µs]
volatile uint32_t g_pas_low_time_us = 0;      ///< Czas trwania stanu LOW [µs]
volatile bool g_pas_forward = true;           ///< Kierunek pedałowania (asymetria duty cycle)
volatile uint32_t g_pas_last_fwd_pulse_us = 0; ///< micros() ostatniej krawędzi PAS w kierunku FORWARD (aktualizowane tylko w ISR)
volatile int8_t g_pas_dir_confidence = 0;     ///< Licznik pewności kierunku: >0=fwd, <0=rev (histereza ISR)
volatile bool g_pas_dir_invert_isr = false;   ///< Kopia pas_dir_invert dla ISR (ustawiana w setup/cmd)
volatile uint32_t g_pas_debounce_us_isr = 3000; ///< Kopia pas_debounce_us dla ISR (ustawiana z config)
volatile uint32_t g_pas_edge_count = 0;       ///< Licznik krawędzi PAS (ISR inkrementuje)

/// PAS sampling: timer co PAS_SAMPLE_INTERVAL_US próbkuje pin zamiast przerwania.
/// Stan zmieni się dopiero po g_pas_filter_depth kolejnych zgodnych próbkach.
/// Eliminuje szpilki EMI z silnika (<1ms) które nie utrzymają się N×500µs.
#define PAS_SAMPLE_INTERVAL_US  500   ///< Okres próbkowania PAS [µs] (2 kHz)
#define PAS_DEBOUNCE_US_DEFAULT 3000  ///< Domyślny czas potwierdzenia stanu [µs]
volatile uint8_t  g_pas_filter_count = 0;     ///< Ile kolejnych próbek różni się od stanu filtrowanego
volatile bool     g_pas_filtered_state = false; ///< Aktualny przefiltrowany stan pinu PAS
volatile uint8_t  g_pas_filter_depth = 6;     ///< Ile zgodnych próbek do zmiany stanu (debounce_us/500)
static esp_timer_handle_t g_pas_sample_timer = nullptr;

/// Minimalny półokres PAS: domyślnie 5ms — nadpisywany z NVS (pas_min_halfperiod_ms)
static uint32_t g_pas_min_halfperiod_us = 5000;
/// Minimalna różnica duty cycle do detekcji kierunku (%)
/// Domyślnie 5% — nadpisywany z NVS (pas_dir_asymmetry_pct)
static uint8_t g_pas_dir_min_asymmetry = 5;

// Regeneracja — zmienne volatile dla ISR
volatile bool g_regen_active_isr = false;     ///< Tryb regen aktywny (do ISR)
volatile uint16_t g_regen_duty_isr = 0;       ///< Siła hamowania regen 0-PWM_MAX_DUTY

// Limit prądowy (P14 / NVS current_limit_a)
static float g_current_limit_factor = 1.0f;   ///< Mnożnik duty z limitera prądowego (0.0–1.0)
static bool  g_overcurrent_fault = false;      ///< Hard cutoff: prąd > 150% limitu → blokada
static unsigned long g_overcurrent_fault_ms = 0; ///< Czas wejścia w fault [ms]
static float g_ilim_current_ema = 0.0f;        ///< EMA-filtrowany max prąd fazowy dla limitera [A]
static uint8_t g_ilim_hard_count = 0;          ///< Licznik kolejnych próbek powyżej hard cutoff
#define OVERCURRENT_HARD_MULT  1.5f            ///< Mnożnik: hard cutoff przy 150% limitu
#define OVERCURRENT_FAULT_MS   500             ///< Czas blokady po hard cutoff [ms]
#define OVERCURRENT_CONSEC     3               ///< Wymagane kolejne próbki powyżej progu hard cutoff
#define ILIMIT_KP_DOWN         0.05f           ///< Proporcjonalny spadek przy przekroczeniu [/A]
#define ILIMIT_RECOVER_RATE    0.5f            ///< Szybkość odzyskiwania [1/s]
#define ILIMIT_EMA_ALPHA       0.15f           ///< EMA α filtru prądu dla limitera (szybszy niż FOC)

// Tryb testowy MOSFETów — diagnostyka uszkodzonych tranzystorów
volatile bool g_mosfet_test_active = false;    ///< Tryb testu MOSFET aktywny (ISR nie rusza LEDC)
static uint16_t g_mosfet_test_duty = PWM_MAX_DUTY * 10 / 100;  ///< Duty testowe (domyślnie 10%)
static char g_mosfet_test_phase = 0;           ///< Aktualnie testowana faza ('A','B','C') lub 0
static char g_mosfet_test_side  = 0;           ///< Aktualnie testowana strona ('H','L') lub 0

// Odwrócenie kierunku obrotów (software switch CW/CCW)
volatile bool g_reverse_isr = false;           ///< Kierunek: false=CW (domyślny), true=CCW

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

/**
 * @brief Mapowanie Hall→Hall dla odwróconego kierunku (CCW).
 *
 * Zamiana źródła i ujścia w komutacji blokowej: 1↔6, 3↔4, 2↔5.
 * Wartości 0 i 7 (nieprawidłowe) bez zmian.
 */
static const DRAM_ATTR uint8_t g_hall_reverse_map[8] = {
    0,  // 0 = invalid → invalid
    6,  // 1 (A+B-) → 6 (B+A-)
    5,  // 2 (B+C-) → 5 (C+B-)
    4,  // 3 (A+C-) → 4 (C+A-)
    3,  // 4 (C+A-) → 3 (A+C-)
    2,  // 5 (C+B-) → 2 (B+C-)
    1,  // 6 (B+A-) → 1 (A+B-)
    7   // 7 = invalid → invalid
};

/**
 * @brief Mapowanie Hall→sektor dla CCW (SINUS/FOC), bez zamiany faz.
 *
 * Wyprowadzenie z pierwszych zasad (poprawne):
 *   Bez zamiany B↔C: kąt pola = θ° - 90°
 *   CW empirycznie: Hall=001 wejście przy wirniku=0°, θ=8 → pole=300° = wirnik-60° → moment CW
 *   CCW wejście w Hall=001 następuje przy wirniku=60° (nie 0°)!
 *   Dla momentu CCW: pole = wirnik+60° = 60°+60° = 120° → θ=210°=56 wpisów = sektor 3
 *   Reguła: CCW_sector = (CW_sector + 3) mod 6  — pole odwrócone o 180° = odwrócony moment
 *   CW tabela: {0,2,1,4,5,3} dla hall 1-6
 *   CCW = CW+3 mod 6: {3,5,4,1,2,0} dla hall 1-6
 *   Sekwencja CCW: 001→101→100→110→010→011 → sektory 3→2→1→0→5→4 (maleje -1 ✓)
 */
static const DRAM_ATTR int8_t g_hall_to_sector_ccw[8] = {
    -1,     // 0 = invalid
     3,     // 1 (001) → sector 3  (θ snap=56)
     5,     // 2 (010) → sector 5  (θ snap=88)
     4,     // 3 (011) → sector 4  (θ snap=72)
     1,     // 4 (100) → sector 1  (θ snap=24)
     2,     // 5 (101) → sector 2  (θ snap=40)
     0,     // 6 (110) → sector 0  (θ snap=8)
    -1      // 7 = invalid
};

/**
 * @brief Zwraca sektor (0-5) dla danego stanu Halla, z uwzględnieniem kierunku.
 * W CCW używa dedykowanej tabeli dla fizycznej sekwencji CCW rotora.
 */
static inline int8_t IRAM_ATTR hallToSector(uint8_t hall) {
    if (hall == 0 || hall == 7) return -1;
    return g_reverse_isr ? g_hall_to_sector_ccw[hall] : g_hall_to_sector[hall];
}

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
#define SINE_STARTUP_COMMUT     18              // przejść Halla w BLOCK przed przejściem na SINUS/FOC (3 obroty el.)
#define SINE_BLOCK_SPEED_THRESHOLD  10000       // Q16 speed poniżej tego → BLOCK zamiast SINUS/FOC (stall/oscylacja)
#define SINE_STALL_FREEZE_MS    200             // ms bez Halla → zamrożenie kąta
#define SINE_CRAWL_SPEED_Q16    315             // minimalna prędkość startowa ≈ 1 obr.elekt./s (52428800/166666)
#define SINE_STALL_FALLBACK_MS  400             // minimalny timeout fallback (histereza, anty-szarpanie)
#define SINE_START_MAX_HALL_US  30000           // max okres Halla (min prędkość) do wejścia w SINUS
#define SINE_PHASE_CORR_SHIFT   0               // korekcja pełnego błędu na przejście Halla
                                                // Hall jest wiarygodne → nie filtruj o połowę
#define SINE_SPEED_CORR_ENABLE  1               // PLL: korekcja prędkości na przejściu Halla
                                                // err>0 = kąt za wolny → zwiększ prędkość
                                                // err<0 = kąt za szybki → zmniejsz prędkość
#define SINE_SPEED_FILTER_SHIFT 1               // filtr prędkości (unused, rate-limiter zamiast EMA)
#define SINE_SPEED_MAX_CHANGE_PCT 30            // max zmiana prędkości na Hall edge [%]

// Block crossfade: płynne przejście PWM między fazami przy komutacji
// Stara faza PWM wygasza duty do 0, nowa faza PWM rozjaśnia od 0 do d
// Eliminuje skokowy przeskok prądu (główne źródło terkotu blokowego)
#define BLOCK_CROSS_TICKS       8               // tiki crossfade (8×50µs = 400µs)
static volatile uint8_t  g_block_cross_cnt = 0; ///< Tiki pozostałe crossfade (0 = normalny tryb)
static volatile uint8_t  g_block_old_bh = 0;    ///< Remapowany Hall PRZED przejściem

// Tabela stanów faz per remapowany Hall: 0=Off, 1=Low, 2=PWM
static const DRAM_ATTR uint8_t g_block_phase_tbl[7][3] = {
    {0,0,0},  // 0: invalid
    {2,1,0},  // 1: A=PWM B=Low  C=Off
    {0,2,1},  // 2: A=Off  B=PWM C=Low
    {2,0,1},  // 3: A=PWM B=Off  C=Low
    {1,0,2},  // 4: A=Low  B=Off  C=PWM
    {0,1,2},  // 5: A=Off  B=Low  C=PWM
    {1,2,0},  // 6: A=Low  B=PWM C=Off
};

/**
 * @brief Resetuje tracker kąta SINUS/FOC do środka aktualnego sektora Halla.
 */
static void resetSineTracking(uint8_t hall_state) {
    int8_t sector = hallToSector(hall_state);
    if (sector < 0) sector = 0;
    int32_t init_entry = (int32_t)sector * SINE_SECTOR_ENTRIES + SINE_SECTOR_CENTER + g_sine_hall_phase_offset;
    if (init_entry < 0) init_entry += SINE_TABLE_SIZE;
    if (init_entry >= SINE_TABLE_SIZE) init_entry -= SINE_TABLE_SIZE;

    g_sine_startup_count = 0;
    g_sine_last_hall_idx = -1;
    g_sine_speed_q16 = 0;
    for (int i = 0; i < 6; i++) g_sine_sector_speed[i] = 0;
    g_sine_dir = 1;
    g_sine_last_hall_ms = (uint32_t)esp_timer_get_time() / 1000;
    g_sine_angle_q16 = (uint32_t)init_entry << 16;
    g_sine_running = 1;
}
#define SINE_SNAP_THRESHOLD     (24 << 16)       // błąd > 1/4 obrotu elektr. → pełny snap kąta
#define SINE_SAFE_MAX_DUTY      (PWM_MAX_DUTY * 75 / 100)  // SVPWM: liniowy do 58%, overmod do 75%, powyżej szkodliwe harmoniczne
#define SINE_MIN_AMPLITUDE      8               // poniżej tego coast (~1.5% PWM_MAX_DUTY)

/// Debounce czujników Halla: minimalna przerwa między przejściami [us].
/// W trybie SINUS 6 FETów przekłądają jednocześnie (center-aligned PWM),
/// generując znacznie więcej EMI niż BLOCK (2 FETy). Szumy sprzegają się
/// w linie Halla i tworzą fałszywe przejścia (dt ~50us = 1 tick ISR).
/// Bez debounce: hall_period_us = 50us → speed_q16 = 1M → kąt ucieka → desync.
/// 200us = 4 ticki ISR, bezpieczne do ~50k eRPM (daleko poza realnym motorem).
#define HALL_MIN_PERIOD_US      200
// Przy starcie/postoju SINUS/FOC szpilki EMI potrafią wygenerować szybkie pseudo-przejścia
// Hall, które kręcą wektor pola mimo zablokowanego koła. Podnosimy wtedy minimalny okres.
#define HALL_MIN_PERIOD_STARTUP_US 1000
// Reverse-step lock tylko przy niskim duty (manetka lekko otwarta), żeby nie blokować
// rzeczywistego cofania przy mocnym hamowaniu/odwrotnym momencie.
#define SINE_DIR_LOCK_DUTY_PCT  20
#define DEFAULT_P07_STANDALONE  90              // domyślne P07 gdy brak wyświetlacza (6 × 15 par biegunów)

// ── FOC (Field Oriented Control) ──
// Architektura: pętla prądowa w loop() (~2kHz), modulacja SVPWM w ISR (20kHz).
// loop(): czyta prądy ADC → Clarke → Park → PI(Id,Iq) → zapisuje Vd,Vq (volatile)
// ISR: czyta Vd,Vq → InvPark(θ) → InvClarke → SVPWM → MCPWM
// Kąt θ: współdzielony z SINUS (Hall tracking + interpolacja Q16)
#define FOC_KP_DEFAULT      0.5f    ///< Domyślne Kp regulatora PI d/q (PWM/A)
#define FOC_KI_DEFAULT      5.0f    ///< Domyślne Ki regulatora PI d/q (PWM/(A·s))
#define FOC_INTEGRAL_LIMIT  ((float)SINE_SAFE_MAX_DUTY)  ///< Absolutny max integrala (anti-windup)
#define FOC_PI_CORR_LIMIT   100.0f  ///< Max korekta PI wokół feedforward [±PWM]
#define FOC_IQ_MAX          10.0f   ///< Maksymalny prąd Iq target [A]
#define FOC_LOOP_DT         0.0005f ///< Przybliżony dt pętli prądowej [s] (~2kHz loop)
#define FOC_INV_SQRT3       0.57735026919f  ///< 1/√3
#define FOC_SQRT3           1.73205080757f  ///< √3
#define FOC_CURRENT_EMA_ALPHA  0.05f ///< EMA α dla prądów FOC (~2kHz → τ≈10ms)
                                     //   Uśrednia sporadyczne odczyty ADC (analogRead
                                     //   nie jest zsynchr. z PWM, INA180A2 widzi prąd
                                     //   tylko gdy low-side ON → ~50% odczytów = 0)

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
void IRAM_ATTR allMosfetsOff();
void readAnalogInputs();
void readHallSensors();
void readDigitalInputs();
void printDiagnostics();
void printDisplayConfig();
void blockCommutate(uint8_t hallState, uint16_t duty);
void processSerialCommands();
static void IRAM_ATTR regenCommutateISR(uint8_t hall, uint16_t regen_duty);
static void IRAM_ATTR sinusCommutateISR(uint8_t hall, uint16_t amplitude);
static void IRAM_ATTR focCommutateISR(uint8_t hall, uint16_t amplitude);
static inline int32_t IRAM_ATTR sine_interp_q16(uint32_t angle_q16);
// Prototypy phase helpers (definicje poniżej ISR)
static inline void IRAM_ATTR phaseA_PWM(uint16_t duty);
static inline void IRAM_ATTR phaseA_Low();
static inline void IRAM_ATTR phaseA_Off();
static inline void IRAM_ATTR phaseB_PWM(uint16_t duty);
static inline void IRAM_ATTR phaseB_Low();
static inline void IRAM_ATTR phaseB_Off();
static inline void IRAM_ATTR phaseC_PWM(uint16_t duty);
static inline void IRAM_ATTR phaseC_Low();
static inline void IRAM_ATTR phaseC_Off();
static inline void IRAM_ATTR phaseA_RegenPWM(uint16_t duty);
static inline void IRAM_ATTR phaseB_RegenPWM(uint16_t duty);
static inline void IRAM_ATTR phaseC_RegenPWM(uint16_t duty);
static void printSineDebug();
static String executeCommand(const String& cmd);
static String mosfetTestSet(const String& which);
static void mosfetTestPrintHelp();
static void webConfigInit();
static void webConfigStop();
static void webConfigHandle();

// ============================================================================
// MCPWM inline helpers — ISR-safe, direct register writes
// ============================================================================
//
// Architektura: MCPWM_UNIT_0, 3 operatory (po jednym na fazę), UP_DOWN counter.
// Każdy operator: gen_A → HIN (high-side), gen_B → LIN (low-side).
// Oba generatory sterowane niezależnie (dead time bypass — IR2103 ma wewnętrzny).
//
// Mapowanie:
//   Operator 0 = Faza A: gen_A → GPIO32 (HIN_A), gen_B → GPIO33 (LIN_A)
//   Operator 1 = Faza B: gen_A → GPIO25 (HIN_B), gen_B → GPIO26 (LIN_B)
//   Operator 2 = Faza C: gen_A → GPIO27 (HIN_C), gen_B → GPIO14 (LIN_C)
//
// IR2103: HIN=active HIGH (HIGH → HS ON), LIN=active LOW (LOW → LS ON)
// Kontrola za pomocą bezpośrednich zapisów do rejestrów gen_force i generator action (ISR-safe).
//
// Sekwencja stanów:
//   PWM:   gen_A = PWM(duty), gen_B = forced HIGH (LIN=HIGH → LS OFF)
//   GND:   gen_A = forced LOW (HS OFF), gen_B = forced LOW (LIN=LOW → LS ON)
//   OFF:   gen_A = forced LOW (HS OFF), gen_B = forced HIGH (LIN=HIGH → LS OFF)
//   SINUS/FOC: gen_A = gen_B = PWM(same_duty) — IR2103 robi complementary + dead time
//   REGEN: gen_A = forced LOW (HS OFF), gen_B = PWM(inverted_duty: 0% = LS ON, 100% = LS OFF)

// ── ISR-safe MCPWM register helpers (bezpośredni zapis do rejestrów, ~5ns) ──
//
// Wszystkie operacje poniżej omijają driver API (mcpwm_set_duty_type,
// mcpwm_set_signal_high/low) które używają spinlocków i nie są ISR-safe.
// Zamiast tego operujemy bezpośrednio na rejestrach MCPWM0.

// ── Precomputed gen_force.val values ──
// gen_cntuforce_upmethod (bits [5:0]) = 0 → update immediately
// gen_a_cntuforce_mode (bits [7:6]): 0=disabled(PWM), 1=forceLOW, 2=forceHIGH
// gen_b_cntuforce_mode (bits [9:8]): 0=disabled(PWM), 1=forceLOW, 2=forceHIGH
#define MCPWM_FORCE_PWM     0x0200  // gen_A=PWM(0), gen_B=forceHIGH(2) → LS OFF
#define MCPWM_FORCE_GND     0x0140  // gen_A=forceLOW(1), gen_B=forceLOW(1) → LS ON
#define MCPWM_FORCE_OFF     0x0240  // gen_A=forceLOW(1), gen_B=forceHIGH(2) → float
#define MCPWM_FORCE_COMPL   0x0000  // gen_A=PWM(0), gen_B=PWM(0) → complementary
#define MCPWM_FORCE_REGEN   0x0040  // gen_A=forceLOW(1), gen_B=PWM(0) → regen

// ── Precomputed generator action register values ──
// Dla UP_DOWN counter (center-aligned PWM):
//   utea/uteb = action on compare match, counting UP
//   dtea/dteb = action on compare match, counting DOWN
//   action: 0=no change, 1=low, 2=high, 3=toggle
//
// UP_DOWN: czas HIGH = 2×C, okres = 2×P → duty = C/P
// Aby duty=0→0% i duty=P→100% (jak LEDC), potrzebujemy:
//   counting UP + compare match → LOW  (koniec fazy HIGH)
//   counting DOWN + compare match → HIGH (początek fazy HIGH)
//
// gen_A (generator[0]) reacts to compare_A:     MODE_0: utea=LOW(1)  dtea=HIGH(2)
// gen_B (generator[1]) reacts to compare_B:     MODE_0: uteb=LOW(1)  dteb=HIGH(2)
//                                    (active-low) MODE_1: uteb=HIGH(2) dteb=LOW(1)
#define GEN_A_ACTION_MODE0  0x00020010  // utea=1 @bits[5:4], dtea=2 @bits[17:16]
#define GEN_B_ACTION_MODE0  0x00080040  // uteb=1 @bits[7:6], dteb=2 @bits[19:18]
#define GEN_B_ACTION_MODE1  0x00040080  // uteb=2 @bits[7:6], dteb=1 @bits[19:18]

// Szybki zapis compare value (ISR-safe, ~5ns)
static inline void IRAM_ATTR mcpwm_set_compare_fast(int op, int cmp, uint32_t val) {
    mcpwm_ll_operator_set_compare_value(&MCPWM0, op, cmp, val);
}

/**
 * @brief Ustawia fazę na PWM (high-side modulowany, low-side OFF).
 * gen_A = PWM(duty, MODE_0), gen_B = forced HIGH (LIN=HIGH → LS OFF)
 * ISR-safe: bezpośredni zapis do rejestrów, bez spinlocków.
 */
static inline void IRAM_ATTR mcpwm_phase_pwm(int op, uint16_t duty) {
    MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
    mcpwm_set_compare_fast(op, 0, duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_PWM;
}

/**
 * @brief Ustawia fazę na GND (high-side OFF, low-side ON).
 * gen_A = forced LOW (HS OFF), gen_B = forced LOW (LIN=LOW → LS ON)
 */
static inline void IRAM_ATTR mcpwm_phase_gnd(int op) {
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_GND;
}

/**
 * @brief Ustawia fazę na float (oba OFF).
 * gen_A = forced LOW (HS OFF), gen_B = forced HIGH (LIN=HIGH → LS OFF)
 */
static inline void IRAM_ATTR mcpwm_phase_off(int op) {
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_OFF;
}

/**
 * @brief Ustawia fazę na SINUS/FOC (oba generatory = ten sam duty, komplementarny przez IR2103).
 * gen_A = PWM(duty, MODE_0), gen_B = PWM(duty, MODE_0) — IR2103 tworzy komplementarne + dead time.
 * ISR-safe: bezpośredni zapis do rejestrów.
 */
static inline void IRAM_ATTR mcpwm_phase_complementary(int op, uint32_t duty) {
    MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
    MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE0;
    mcpwm_set_compare_fast(op, 0, duty);
    mcpwm_set_compare_fast(op, 1, duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_COMPL;
}

/**
 * @brief Ustawia fazę na regen PWM (HS OFF, LS modulowany).
 * gen_A = forced LOW (HS OFF)
 * gen_B = PWM inverted (MODE_1): duty=0→LS ON, duty=MAX→LS OFF
 * Przy PWM OFF: prąd indukcyjny przez body diodę HS → Vbat (regeneracja)
 * ISR-safe: bezpośredni zapis do rejestrów.
 */
static inline void IRAM_ATTR mcpwm_phase_regen(int op, uint16_t regen_duty) {
    MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE1;
    mcpwm_set_compare_fast(op, 1, regen_duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_REGEN;
}

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
static const unsigned long AUTO_STATUS_INTERVAL_MS = 500;
static bool g_debugSine = false;
static unsigned long g_lastDebugSineMs = 0;
static const unsigned long DEBUG_SINE_INTERVAL_MS = 200;
static bool g_debugCurrent = false;
static unsigned long g_lastDebugCurrentMs = 0;
static const unsigned long DEBUG_CURRENT_INTERVAL_MS = 500;
static bool g_debugHall = false;
static unsigned long g_lastDebugHallMs = 0;
static const unsigned long DEBUG_HALL_INTERVAL_MS = 200;

// Bufor na komendy numeryczne
static String serialBuffer = "";

// Symulacja hamulca komendą Serial
static bool g_brake_simulated = false;
static uint8_t g_brake_debounce_count = 0;   ///< Licznik debounce hamulca
#define BRAKE_DEBOUNCE_THRESHOLD  5           ///< Ile kolejnych LOW wymagane (~2.5ms przy 2kHz loop)

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

// PAS Auto-tune (komenda 'pasat')
// Zbiera statystyki sygnału PAS przez PASAT_MEASURE_S sekund,
// potem oblicza i ustawia optymalne parametry filtra PAS.
enum PasAtState : uint8_t {
    PASAT_IDLE = 0,
    PASAT_INIT,
    PASAT_MEASURE,
    PASAT_DONE
};
static PasAtState g_pasat_state = PASAT_IDLE;
static unsigned long g_pasat_start_ms        = 0;
static uint32_t g_pasat_edge_start           = 0;     ///< g_pas_edge_count na starcie
static uint32_t g_pasat_min_halfperiod_us    = UINT32_MAX;
static uint32_t g_pasat_max_halfperiod_us    = 0;
static uint64_t g_pasat_sum_halfperiod_us    = 0;
static uint32_t g_pasat_halfperiod_count     = 0;
static uint64_t g_pasat_sum_asymmetry_x100   = 0;     ///< Suma asymetrii * 100
static uint32_t g_pasat_asymmetry_count      = 0;
static uint32_t g_pasat_max_asymmetry_x100   = 0;     ///< Max zmierzona asymetria * 100
static uint32_t g_pasat_prev_high_us         = 0;     ///< Poprzedni odczyt g_pas_high_time_us
static uint32_t g_pasat_prev_low_us          = 0;     ///< Poprzedni odczyt g_pas_low_time_us
static const unsigned long PASAT_MEASURE_S   = 10;    ///< Czas pomiaru [s]

// SPEED Calibration (komenda 'spdcal')
// Kalibracja: użytkownik kręci kołem 3 pełne obroty w 15 s,
// firmware zlicza impulsy i oblicza pulses_per_rev.
enum SpdCalState : uint8_t {
    SPDCAL_IDLE = 0,
    SPDCAL_INIT,
    SPDCAL_MEASURE,
    SPDCAL_DONE
};
static SpdCalState g_spdcal_state       = SPDCAL_IDLE;
static unsigned long g_spdcal_start_ms  = 0;
static uint32_t g_spdcal_pulse_start    = 0;    ///< g_speed_pulse_count na starcie
static const unsigned long SPDCAL_MEASURE_S = 15;  ///< Czas pomiaru [s]

// Wyświetlacz S866 (zawsze aktywny na Serial2)
static s866_display_t g_display;

// ── Serwer WWW konfiguracji WiFi (aktywny gdy P17=1) ──
static WebServer* g_web_server     = nullptr;  ///< Instancja serwera HTTP (nullptr gdy wy\u0142\u0105czony)
static String     g_web_queued_cmd = "";        ///< Komenda silnika do wykonania po wy\u0142\u0105czeniu WiFi
static bool       g_wifi_active    = false;     ///< Flaga: WiFi AP aktywne

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
        if (config_get().display_required) {
            return 0;  // wyświetlacz wymagany ale niepodłączony → silnik wyłączony
        }
        return PWM_MAX_DUTY;  // brak wyświetlacza → pełna moc (standalone)
    }
    if (g_display.rx.assist_level == 0) {
        return 0;  // assist level 0 → silnik wyłączony
    }
    // Raw assist_level z wyświetlacza S866 jest zawsze 0-15 (bajt 4, bity 0-3),
    // niezależnie od ustawienia P05 (które definiuje ile kroków widzi użytkownik).
    uint16_t maxDuty = (uint16_t)((uint32_t)g_display.rx.assist_level * PWM_MAX_DUTY / 15);
    if (maxDuty > PWM_MAX_DUTY) maxDuty = PWM_MAX_DUTY;
    return maxDuty;
}

/**
 * @brief Pobiera limit prędkości [km/h] z parametru P08.
 * @return Limit prędkości w km/h. 0 = brak limitu (wyłączony).
 */
static uint8_t getSpeedLimitKmh() {
    return g_display.config.p08_speed_limit;
}

/**
 * @brief Efektywny limit prądu fazowego [A].
 *
 * Priorytet: P14 z wyświetlacza (jeśli podłączony i P14>0),
 * w przeciwnym razie current_limit_a z NVS.
 * Wartość 0 oznacza brak limitu (bypass).
 *
 * @return Limit prądu [A], 0 = brak limitu.
 */
static uint8_t getEffectiveCurrentLimit() {
    if (g_display.connected && g_display.config.p14_current_limit_a > 0) {
        return g_display.config.p14_current_limit_a;
    }
    return config_get().current_limit_a;
}

/**
 * @brief Globalny limit prędkości z power fade (filtrowany EMA).
 *
 * Redukuje duty_target w miarę zbliżania się do prędkości maksymalnej (P08).
 * Strefa fade: 70%..100% limitu. Powyżej limitu: duty = 0 (coast).
 *
 * Współczynnik mocy (0.0..1.0) jest filtrowany filtrem EMA aby uniknąć
 * oscylacji duty spowodowanych zaszumioną prędkością koła (1 impuls/obrót).
 * Bez filtra: speed bounces around limit → duty bounces → speed bounces → ...
 *
 * Dotyczy WSZYSTKICH trybów (BLOCK, SINUS, FOC) i źródeł duty (manetka, PAS).
 *
 * @param duty_in  Duty przed limitowaniem.
 * @param speed_kmh Aktualna prędkość koła [km/h].
 * @return Duty po limitowaniu.
 */
static float g_speed_limit_factor = 1.0f;           ///< EMA-filtrowany współczynnik mocy 0..1
#define SPEED_LIMIT_ALPHA_DOWN    0.03f              ///< EMA α spadek (over-speed → szybka redukcja duty)
#define SPEED_LIMIT_ALPHA_UP      0.02f              ///< EMA α wzrost (under-speed → narastanie duty)
#define SPEED_LIMIT_FADE_BAND_KMH 3.0f              ///< Strefa liniowego fade przed limitem [km/h]

static uint16_t applyGlobalSpeedLimit(uint16_t duty_in, float speed_kmh) {
    if (duty_in == 0) {
        // Przy zerowym duty nie modyfikuj filtra — silnik nie jedzie
        return 0;
    }
    uint8_t limit_kmh = getSpeedLimitKmh();
    if (limit_kmh == 0) {
        // P08==0: brak limitu prędkości — pełna moc bez ograniczeń
        g_speed_limit_factor = 1.0f;
        return duty_in;
    }
    float limit_f = (float)limit_kmh;

    // Oblicz surowy współczynnik mocy (0.0 .. 1.0)
    // Fade zaczyna się SPEED_LIMIT_FADE_BAND_KMH km/h przed limitem (stałe okno, niezależne od limitu)
    float raw_factor;
    if (speed_kmh >= limit_f) {
        raw_factor = 0.0f;  // powyżej limitu → zero mocy
    } else {
        float fade_start = limit_f - SPEED_LIMIT_FADE_BAND_KMH;
        if (fade_start < 0.0f) fade_start = 0.0f;
        if (speed_kmh <= fade_start) {
            raw_factor = 1.0f;  // poniżej strefy fade → pełna moc
        } else {
            // Liniowy spadek 1.0→0.0 w strefie fade (ostatnie 3 km/h przed limitem)
            raw_factor = (limit_f - speed_kmh) / SPEED_LIMIT_FADE_BAND_KMH;
        }
    }

    // Asymetryczny filtr EMA:
    // - Over-speed (raw < sl): szybka redukcja duty → zapobiega przekroczeniu limitu
    // - Under-speed (raw > sl): wolne narastanie → zapobiega oscylacji bang-bang
    float alpha = (raw_factor < g_speed_limit_factor) ? SPEED_LIMIT_ALPHA_DOWN : SPEED_LIMIT_ALPHA_UP;
    g_speed_limit_factor += alpha * (raw_factor - g_speed_limit_factor);

    // Clamp do 0..1 (bezpieczeństwo numeryczne)
    if (g_speed_limit_factor < 0.0f) g_speed_limit_factor = 0.0f;
    if (g_speed_limit_factor > 1.0f) g_speed_limit_factor = 1.0f;

    // Twardy clamp: jeśli prędkość > limit+2km/h → natychmiast zero (bezpieczeństwo)
    if (speed_kmh > limit_f + 2.0f) {
        g_speed_limit_factor = 0.0f;
    }

    uint16_t result = (uint16_t)((float)duty_in * g_speed_limit_factor);
    return result;
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
    uint32_t stop_us = (uint32_t)config_get().pas_stop_delay_ms * 1000UL;
    if (last == 0 || period == 0 || (now_us - last) > stop_us) {
        return 0;
    }

    // Liczba magnesów PAS (z wyświetlacza P13, domyślnie 12)
    uint8_t magnets = g_display.config.p13_pas_magnets;
    if (magnets == 0) magnets = 12;  // fallback

    // cadence_rpm = 60_000_000 / (period_us × magnets)
    uint32_t cadence = 60000000UL / (period * (uint32_t)magnets);
    if (cadence > 150) cadence = 150;

    return (uint16_t)cadence;
}

// === Stan PAS ===
/// Timestamp [ms] od kiedy PAS kręci się w kierunku forward (0 = nie kręci)
static uint32_t g_pas_fwd_since_ms = 0;
/// Timestamp [ms] momentu aktywacji PAS (przejścia z WAIT do ON) — soft-start
static uint32_t g_pas_active_since_ms = 0;
/// Flaga: PAS aktywnie wspomaga (przeszedł start delay i kręci się forward)
static bool g_pas_pedaling = false;
/// Poprzednie duty PAS — do slew rate limiter
static uint16_t g_pas_prev_duty = 0;
/// Timestamp [ms] ostatniego widzianego g_pas_forward==true — holdoff reverse
static uint32_t g_pas_last_fwd_ms = 0;
/// Wygładzona prędkość docelowa PAS — zapobiega szarpnięciom przy zmianie poziomu
static float g_pas_vtarget_smooth = 0.0f;

// --- Bufor średniej kroczącej prędkości koła (Moving Average) ---
#define PAS_SPEED_MA_SIZE  8
static float g_pas_speed_ma_buf[PAS_SPEED_MA_SIZE] = {0};
static uint8_t g_pas_speed_ma_idx = 0;
static bool g_pas_speed_ma_full = false;

/// Maksymalna zmiana duty PAS na jedno wywołanie (slew rate)
/// Domyślnie 30 (~6% PWM_MAX_DUTY). Nadpisywany z NVS (pas_slew_rate).
static uint16_t g_pas_slew_rate_max = 30;
/// Czas podtrzymania stanu forward po krótkim "reverse" [ms]
/// Domyślnie 300. Nadpisywany z NVS (pas_fwd_holdoff_ms).
static uint16_t g_pas_fwd_holdoff_ms = 300;

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
 * @brief Oblicza duty PAS z V_target, soft-start, speed ramp-down, slew rate i MA.
 *
 * Algorytm:
 *   1. assist_level == 0 → duty = 0
 *   2. Brak impulsów PAS przez pas_stop_delay_ms → duty = 0
 *   3. Kierunek reverse (z holdoff 300ms) → duty = 0
 *   4. Forward < pas_start_delay_ms → duty = 0
 *   5. Forward >= pas_start_delay_ms → ACTIVE:
 *      a. soft_start: moc narasta 0→100% w czasie pas_ramp_ms
 *      b. V_target = 6 + Lx * (v_max - 6) / 15
 *      c. Prędkość wygładzona średnią kroczącą (8 próbek)
 *      d. v_smooth < v_target-3 → pełna moc
 *      e. v_smooth w [v_target-3, v_target] → redukcja liniowa 100%→0%
 *      f. v_smooth >= v_target → duty = 0
 *   6. Slew rate limit: duty zmienia się max ±PAS_SLEW_RATE_MAX na wywołanie
 *   7. Globalny limit P08 stosowany osobno przez applyGlobalSpeedLimit().
 *
 * @param maxDuty  Używane do detekcji assist=0 (motor OFF)
 * @return duty PAS 0–PWM_MAX_DUTY
 */
static uint16_t calculatePasDuty(uint16_t maxDuty) {
    // Assist level 0 → PAS wyłączony
    if (maxDuty == 0) {
        g_pas_pedaling = false;
        g_pas_fwd_since_ms = 0;
        g_pas_active_since_ms = 0;
        g_pas_prev_duty = 0;
        g_pas_vtarget_smooth = 0.0f;
        return 0;
    }

    uint32_t now_us = (uint32_t)esp_timer_get_time();
    uint32_t now_ms = (uint32_t)(now_us / 1000);
    uint32_t last_fwd_pulse = g_pas_last_fwd_pulse_us;  // snapshot volatile — ostatni FORWARD impuls z ISR

    // Parametry z EEPROM
    controller_config_t& cfg = config_get();
    uint32_t stop_delay_us = (uint32_t)cfg.pas_stop_delay_ms * 1000UL;
    uint32_t start_delay_ms = (uint32_t)cfg.pas_start_delay_ms;
    uint32_t ramp_ms = (uint32_t)cfg.pas_ramp_ms;

    // --- Timeout: brak PAS FORWARD przez stop_delay → zatrzymaj wspomaganie ---
    // g_pas_last_fwd_pulse_us jest odświeżane w ISR tylko przy g_pas_forward==true.
    // PAS stoi  → ISR nie odpala       → timestamp starzeje się → timed_out ✓
    // PAS wstecz → ISR odpala, fwd=false → timestamp nie odświeżany → timed_out ✓
    // PAS forward → ISR odpala, fwd=true  → timestamp odświeżany  → timed_out ✗
    //
    // Stale dane HIGH/LOW po długiej przerwie: ISR sam resetuje pomiary
    // gdy wykryje przerwę >2s (patrz początek onPasPulse).
    //
    // period_too_long: magnesy zatrzymane generują sporadyczne szpilki,
    // które odnawiają last_pulse. Zmierzony okres H+L >> stop_delay → wykrycie.
    uint32_t period_us = g_pas_period_us;
    bool timed_out = (last_fwd_pulse == 0) || ((now_us - last_fwd_pulse) >= stop_delay_us);
    bool period_too_long = (period_us > 0) && (period_us > stop_delay_us);
    if (timed_out || period_too_long) {
        g_pas_pedaling = false;
        g_pas_fwd_since_ms = 0;
        g_pas_active_since_ms = 0;
        g_pas_vtarget_smooth = 0.0f;
        // Slew rate w dół: łagodne wygaszenie zamiast twardego 0
        if (g_pas_prev_duty > g_pas_slew_rate_max) {
            g_pas_prev_duty -= g_pas_slew_rate_max;
            return g_pas_prev_duty;
        }
        g_pas_prev_duty = 0;
        return 0;
    }

    // --- Kierunek: reverse z holdoff ---
    // g_pas_last_fwd_ms: timestamp ostatniego forward — dla pomiaru holdoff.
    // Aktualizowany tutaj (po bloku timed_out) — działa poprawnie, bo timed_out już sprawdzony.
    if (g_pas_forward) {
        g_pas_last_fwd_ms = now_ms;
    }
    bool effective_reverse = !g_pas_forward
        && (g_pas_last_fwd_ms == 0 || (now_ms - g_pas_last_fwd_ms) > g_pas_fwd_holdoff_ms);

    if (effective_reverse) {
        g_pas_pedaling = false;
        g_pas_fwd_since_ms = 0;
        g_pas_active_since_ms = 0;
        g_pas_vtarget_smooth = 0.0f;
        if (g_pas_prev_duty > g_pas_slew_rate_max) {
            g_pas_prev_duty -= g_pas_slew_rate_max;
            return g_pas_prev_duty;
        }
        g_pas_prev_duty = 0;
        return 0;
    }

    // --- Kierunek forward: liczymy czas ---
    if (g_pas_fwd_since_ms == 0) {
        g_pas_fwd_since_ms = now_ms;
    }

    // Jeszcze w start delay — nie wspomagaj
    uint32_t fwd_duration_ms = now_ms - g_pas_fwd_since_ms;
    if (fwd_duration_ms < start_delay_ms) {
        return 0;
    }

    // === ACTIVE: pedałowanie potwierdzone ===
    if (!g_pas_pedaling) {
        g_pas_pedaling = true;
        g_pas_active_since_ms = now_ms;  // start soft-start
    }

    // --- Soft-start: moc narasta 0→100% w czasie ramp_ms ---
    float soft_start_factor = 1.0f;
    if (ramp_ms > 0 && g_pas_active_since_ms > 0) {
        uint32_t active_ms = now_ms - g_pas_active_since_ms;
        if (active_ms < ramp_ms) {
            soft_start_factor = (float)active_ms / (float)ramp_ms;
        }
    }

    // --- V_target z poziomu wspomagania (z wygładzeniem) ---
    uint8_t raw_assist = g_display.rx.assist_level;     // 0..15 (raw, niezależne od P05)
    uint8_t v_max = g_display.config.p08_speed_limit;
    // P08==0: brak limitu prędkości → PAS zawsze pełna moc (speed_factor=1)
    bool pas_speed_unlimited = (v_max == 0);
    if (v_max == 0) v_max = 25;  // fallback dla obliczeń v_target (nie ogranicza)
    float v_target_raw = 6.0f + (float)raw_assist * ((float)v_max - 6.0f) / 15.0f;

    // Smooth V_target: zapobiega szarpnięciom przy zmianie assist level w trakcie jazdy.
    // Slew rate ~10 km/h/s (0.005 km/h na wywołanie przy ~2kHz loop).
    // Zmiana L5→L3 (25→17.4 km/h) trwa ~0.76s zamiast natychmiast.
    if (g_pas_vtarget_smooth <= 0.0f) {
        g_pas_vtarget_smooth = v_target_raw;  // pierwszy start: snap
    } else {
        const float VTARGET_SLEW = 0.005f;  // km/h na wywołanie (~10 km/h/s)
        float vdiff = v_target_raw - g_pas_vtarget_smooth;
        if (vdiff > VTARGET_SLEW) vdiff = VTARGET_SLEW;
        if (vdiff < -VTARGET_SLEW) vdiff = -VTARGET_SLEW;
        g_pas_vtarget_smooth += vdiff;
    }
    float v_target = g_pas_vtarget_smooth;

    // --- Średnia krocząca prędkości (Moving Average, 8 próbek) ---
    float raw_speed = g_bldc_state.wheel_speed_kmh;
    g_pas_speed_ma_buf[g_pas_speed_ma_idx] = raw_speed;
    g_pas_speed_ma_idx = (g_pas_speed_ma_idx + 1) % PAS_SPEED_MA_SIZE;
    if (!g_pas_speed_ma_full && g_pas_speed_ma_idx == 0) g_pas_speed_ma_full = true;

    uint8_t ma_count = g_pas_speed_ma_full ? PAS_SPEED_MA_SIZE : g_pas_speed_ma_idx;
    float speed_sum = 0.0f;
    for (uint8_t i = 0; i < ma_count; i++) speed_sum += g_pas_speed_ma_buf[i];
    float speed_kmh = (ma_count > 0) ? (speed_sum / (float)ma_count) : 0.0f;

    // --- Speed ramp-down (strefa 3 km/h) ---
    float speed_factor;
    if (pas_speed_unlimited) {
        speed_factor = 1.0f;  // P08==0: brak limitu → zawsze pełna moc
    } else if (speed_kmh >= v_target) {
        speed_factor = 0.0f;
    } else if (speed_kmh > v_target - 3.0f) {
        // Liniowa redukcja: v_target-3 = 100%, v_target = 0%
        speed_factor = (v_target - speed_kmh) / 3.0f;
    } else {
        speed_factor = 1.0f;
    }

    // --- Wynikowe duty (przed slew rate) ---
    float factor = soft_start_factor * speed_factor;
    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;

    uint16_t target_duty = (uint16_t)(factor * (float)PWM_MAX_DUTY);
    if (target_duty > PWM_MAX_DUTY) target_duty = PWM_MAX_DUTY;

    // --- Slew rate limiter: max ±g_pas_slew_rate_max na wywołanie ---
    uint16_t result;
    if (target_duty > g_pas_prev_duty) {
        uint16_t step = target_duty - g_pas_prev_duty;
        result = (step > g_pas_slew_rate_max)
            ? (g_pas_prev_duty + g_pas_slew_rate_max)
            : target_duty;
    } else {
        uint16_t step = g_pas_prev_duty - target_duty;
        result = (step > g_pas_slew_rate_max)
            ? (g_pas_prev_duty - g_pas_slew_rate_max)
            : target_duty;
    }
    g_pas_prev_duty = result;
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
        uint32_t dt = now_us - g_speed_last_pulse_us;
        if (dt < SPEED_DEBOUNCE_US) {
            g_speed_reject_count++;
            return;  // bounce — za krótki interwał
        }
        g_speed_period_us = dt;
        g_speed_pulse_count++;
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
/**
 * @brief Przetwarzanie krawędzi PAS po przejściu filtra cyfrowego.
 *
 * Wywoływane z timera próbkującego gdy przefiltrowany stan pinu się zmienił.
 * Zawiera logikę detekcji kierunku (asymetria HIGH/LOW) i aktualizację timestampów.
 */
static void IRAM_ATTR processPasFilteredEdge(bool pin_high, uint32_t now_us) {
    // --- Detekcja długiej przerwy → reset pomiarów HIGH/LOW ---
    if (g_pas_last_pulse_us > 0 && (now_us - g_pas_last_pulse_us) > 2000000UL) {
        g_pas_high_time_us   = 0;
        g_pas_low_time_us    = 0;
        g_pas_period_us      = 0;
        g_pas_rising_us      = 0;
        g_pas_falling_us     = 0;
        g_pas_dir_confidence = 0;
        g_pas_forward        = true;
    }

    if (pin_high) {
        // RISING edge — koniec okresu LOW
        if (g_pas_falling_us > 0) {
            uint32_t low_time = now_us - g_pas_falling_us;
            if (low_time >= g_pas_min_halfperiod_us) {
                g_pas_low_time_us = low_time;
            }
        }
        g_pas_rising_us = now_us;
    } else {
        // FALLING edge — koniec okresu HIGH
        if (g_pas_rising_us > 0) {
            uint32_t high_time = now_us - g_pas_rising_us;
            if (high_time >= g_pas_min_halfperiod_us) {
                g_pas_high_time_us = high_time;
            }
        }
        g_pas_falling_us = now_us;

        // Oba półokresy zmierzone → oblicz okres i kierunek
        if (g_pas_high_time_us > 0 && g_pas_low_time_us > 0) {
            uint32_t period = g_pas_high_time_us + g_pas_low_time_us;
            g_pas_period_us = period;

            uint32_t diff = (g_pas_high_time_us > g_pas_low_time_us)
                          ? (g_pas_high_time_us - g_pas_low_time_us)
                          : (g_pas_low_time_us - g_pas_high_time_us);
            uint32_t threshold = period * g_pas_dir_min_asymmetry / 100;
            if (diff > threshold) {
                bool should_fwd = (g_pas_high_time_us > g_pas_low_time_us);
                if (g_pas_dir_invert_isr) should_fwd = !should_fwd;
                if (should_fwd) {
                    if (g_pas_dir_confidence < 5) g_pas_dir_confidence++;
                } else {
                    if (g_pas_dir_confidence > -5) g_pas_dir_confidence--;
                }
                g_pas_forward = (g_pas_dir_confidence > 0);
            }
        }
    }

    g_pas_last_pulse_us = now_us;

    if (g_pas_forward) {
        g_pas_last_fwd_pulse_us = now_us;
    }

    g_pas_edge_count++;
}

/**
 * @brief Timer ISR: próbkowanie pinu PAS co PAS_SAMPLE_INTERVAL_US (500µs = 2kHz).
 *
 * Zamiast reagować na każde zbocze (w tym szpilki EMI), próbkujemy pin
 * z ustaloną częstotliwością i wymagamy N kolejnych zgodnych próbek
 * przed uznaniem zmiany stanu. Szpilki EMI (<1ms) nie utrzymają się
 * przez N×500µs i zostaną odfiltrowane.
 *
 * g_pas_filter_depth = pas_debounce_us / PAS_SAMPLE_INTERVAL_US
 * Np. debounce=3000µs → depth=6 → zmiana stanu wymaga 3ms ciągłego sygnału.
 */
void IRAM_ATTR onPasSampleTimer(void* arg) {
    bool raw = (GPIO.in >> PIN_PAS) & 1;

    if (raw == g_pas_filtered_state) {
        // Próbka zgodna z aktualnym stanem — reset licznika
        g_pas_filter_count = 0;
        return;
    }

    // Próbka różni się od przefiltrowanego stanu
    if (++g_pas_filter_count >= g_pas_filter_depth) {
        // N kolejnych próbek potwierdziło nowy stan → akceptuj zmianę
        g_pas_filter_count = 0;
        g_pas_filtered_state = raw;

        // Przefiltrowana krawędź — przetwórz jak dawne przerwanie
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        processPasFilteredEdge(raw, now_us);
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
    Serial.begin(921600);
    delay(1000);
    
    Serial.println("==========================================");
    Serial.println("  BLDC Motor Driver - ESP32");
    Serial.println("  Wersja: 1.0.0  build:ADC_SYNC_BLKSTART");
    Serial.println("  BLOCK / SINUS / FOC | PAS | WiFi");
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
    g_pas_dir_invert_isr = (cfg.pas_dir_invert != 0);
    g_pas_debounce_us_isr = (uint32_t)cfg.pas_debounce_us;
    g_pas_min_halfperiod_us = (uint32_t)cfg.pas_min_halfperiod_ms * 1000;
    g_pas_dir_min_asymmetry = cfg.pas_dir_asymmetry_pct;
    g_pas_slew_rate_max     = cfg.pas_slew_rate;
    g_pas_fwd_holdoff_ms    = cfg.pas_fwd_holdoff_ms;
    g_speed_pulses_per_rev  = cfg.speed_pulses_per_rev;
    if (g_speed_pulses_per_rev < 1) g_speed_pulses_per_rev = 1;
    g_reverse_isr = (cfg.motor_reverse != 0);

    // Inicjalizacja regulatorów PI dla FOC — z wartościami zapisanymi w NVS
    g_sine_hall_phase_offset = cfg.sine_hall_offset;
    g_foc_pi_d = {cfg.foc_kp_d, cfg.foc_ki_d, 0.0f, FOC_INTEGRAL_LIMIT};
    g_foc_pi_q = {cfg.foc_kp_q, cfg.foc_ki_q, 0.0f, FOC_INTEGRAL_LIMIT};
    g_foc_voltage_mode = (cfg.foc_voltage_mode != 0);

    // Inicjalizacja GPIO
    initGPIO();
    Serial.println("[OK] GPIO zainicjalizowane");

    // Konfiguracja ADC1 dla pomiaru prądu fazowego w ISR (direct register access)
    // analogRead() konfiguruje kanały przy pierwszym wywołaniu, ale ISR omija sterownik
    // i czyta rejestry bezpośrednio → trzeba skonfigurować kanały jawnie.
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);  // Phase A (GPIO39)
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);  // Phase B (GPIO34)
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);  // Phase C (GPIO35)
    Serial.println("[OK] ADC1 skonfigurowany (prądy fazowe, 12-bit, 11dB atten)");

    // Przerwanie na pinie SPEED (czujnik zewnętrzny — aktywne przy P07<=1)
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED), onSpeedPulse, FALLING);

    // PAS (Pedal Assist Sensor) — timer próbkujący zamiast przerwania.
    // Próbkowanie co 500µs (2kHz), filtr cyfrowy: N zgodnych próbek do zmiany stanu.
    // Odporność na szpilki EMI z silnika — szpilka musi trwać N×500µs żeby przejść.
    g_pas_filter_depth = (uint8_t)(g_pas_debounce_us_isr / PAS_SAMPLE_INTERVAL_US);
    if (g_pas_filter_depth < 2) g_pas_filter_depth = 2;
    g_pas_filtered_state = (GPIO.in >> PIN_PAS) & 1;  // stan początkowy
    {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = onPasSampleTimer;
        timer_args.name = "pas_sample";
        timer_args.dispatch_method = ESP_TIMER_TASK;  // callback z task (nie ISR)
        esp_timer_create(&timer_args, &g_pas_sample_timer);
        esp_timer_start_periodic(g_pas_sample_timer, PAS_SAMPLE_INTERVAL_US);
    }
    Serial.printf("[OK] PAS sampling timer: %d us, filter depth: %d (debounce %lu us)\n",
                  PAS_SAMPLE_INTERVAL_US, g_pas_filter_depth, (unsigned long)g_pas_debounce_us_isr);

    // Inicjalizacja PWM
    initPWM();
    Serial.println("[OK] PWM zainicjalizowane");

    // Diagnostyka rejestrów MCPWM po initPWM
    for (int op = 0; op < 3; op++) {
        Serial.printf("[MCPWM] Op%d: genA=0x%08X genB=0x%08X force=0x%08X stmp=0x%08X cfg0=0x%08X dt=0x%08X\n",
            op,
            MCPWM0.operators[op].generator[0].val,
            MCPWM0.operators[op].generator[1].val,
            MCPWM0.operators[op].gen_force.val,
            MCPWM0.operators[op].gen_stmp_cfg.val,
            MCPWM0.operators[op].gen_cfg0.val,
            MCPWM0.operators[op].dt_cfg.val);
    }
    Serial.printf("[MCPWM] Timer0: cfg0=0x%08X period=%d\n",
        MCPWM0.timer[0].timer_cfg0.val,
        MCPWM0.timer[0].timer_cfg0.timer_period);
    // Pokaż faktyczną częstotliwość PWM
    {
        uint32_t grp_pre = mcpwm_ll_group_get_clock_prescale(&MCPWM0);
        uint32_t tmr_pre = mcpwm_ll_timer_get_clock_prescale(&MCPWM0, 0);
        uint32_t period  = MCPWM0.timer[0].timer_cfg0.timer_period;
        uint32_t timer_clk = 160000000UL / grp_pre / tmr_pre;
        uint32_t pwm_hz = (period > 0) ? timer_clk / (2 * period) : 0;
        Serial.printf("[MCPWM] group_pre=%u timer_pre=%u timer_clk=%u Hz  PWM=%u Hz\n",
            grp_pre, tmr_pre, timer_clk, pwm_hz);
    }

    // Upewnienie się, że wszystkie MOSFETy są wyłączone
    allMosfetsOff();
    Serial.println("[OK] Wszystkie MOSFETy wyłączone (stan bezpieczny)");

    // Timer sprzętowy do komutacji (co 50 us = 20 kHz)
    initCommutationTimer();
    g_adc_isr_active = true;  // włącz odczyt ADC prądu w ISR po pełnej konfiguracji
    Serial.println("[OK] Timer komutacji uruchomiony (20 kHz) + ADC prądu w ISR TEZ");

    // Zastosuj częstotliwość PWM z NVS (jeśli inna niż domyślna 20 kHz)
    if (cfg.pwm_freq_hz >= 8000 && cfg.pwm_freq_hz <= 32000 && cfg.pwm_freq_hz != PWM_FREQUENCY) {
        uint16_t actual = applyPwmFrequency(cfg.pwm_freq_hz);
        Serial.printf("[OK] PWM freq z NVS: %u Hz (żądane: %u Hz)\n", actual, cfg.pwm_freq_hz);
    } else {
        g_pwm_freq_hz = PWM_FREQUENCY;
    }

    Serial.printf("[OK] Rampa rozpędzania: %d ms (0→100%%)\n", g_bldc_state.ramp_time_ms);

    // Inicjalizacja wyświetlacza S866 na Serial2 (GPIO16/GPIO17)
    memset(&g_display, 0, sizeof(g_display));
    g_display.last_valid_ms = millis();
    s866_init();
    Serial.println("[OK] Wyświetlacz S866 uruchomiony (Serial2: GPIO16/GPIO17, 9600 baud)");

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
 * Uruchamiany komendą 'sat'. Wymaga trybu SINUS lub FOC i działającego silnika.
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

        // Zapisz do NVS
        config_get().sine_hall_offset = g_atune_best_ofs;
        config_save();

        Serial.println("[SAT] ==============================");
        Serial.println("[SAT] Najlepszy offset: " + String((int)g_atune_best_ofs)
                       + " (" + String((float)g_atune_best_ofs * 3.75f, 1) + "°)"
                       + "  avg_I=" + String(g_atune_best_current, 3) + " A");
        Serial.println("[SAT] Poprzedni offset: " + String((int)g_atune_saved_offset)
                       + " (" + String((float)g_atune_saved_offset * 3.75f, 1) + "°)");
        Serial.println("[SAT] Auto-tune zakończony. Offset zapisany do EEPROM.");
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

/**
 * @brief PAS auto-tune — krok maszyny stanów (wywoływany z updateMotorState).
 *
 * Zbiera statystyki sygnału PAS przez PASAT_MEASURE_S sekund:
 * - min/max/avg półokres (HIGH time, LOW time)
 * - średnia/max asymetria H/L
 * - liczba krawędzi
 *
 * Na koniec oblicza i ustawia optymalne parametry filtra PAS:
 * - pas_min_halfperiod_ms: 50% minimalnego zmierzonego półokresu
 * - pas_dir_asymmetry_pct: 60% średniej zmierzonej asymetrii (min 3%)
 * - pas_debounce_us: 30% minimalnego półokresu (min 500µs)
 */
static void pasAutoTuneStep() {
    if (g_pasat_state == PASAT_IDLE) return;

    unsigned long now = millis();

    switch (g_pasat_state) {
    case PASAT_INIT: {
        g_pasat_start_ms = now;
        g_pasat_edge_start = g_pas_edge_count;
        g_pasat_min_halfperiod_us = UINT32_MAX;
        g_pasat_max_halfperiod_us = 0;
        g_pasat_sum_halfperiod_us = 0;
        g_pasat_halfperiod_count = 0;
        g_pasat_sum_asymmetry_x100 = 0;
        g_pasat_asymmetry_count = 0;
        g_pasat_max_asymmetry_x100 = 0;
        g_pasat_prev_high_us = 0;
        g_pasat_prev_low_us = 0;
        Serial.println("[PASAT] === PAS Auto-tune start ===");
        Serial.printf("[PASAT] Pedaluj rownomiernie przez %lu sekund...\n", PASAT_MEASURE_S);
        g_pasat_state = PASAT_MEASURE;
        break;
    }

    case PASAT_MEASURE: {
        // Odczytaj bieżące pomiary PAS (snapshot volatile)
        uint32_t ht = g_pas_high_time_us;
        uint32_t lt = g_pas_low_time_us;

        // Nowy pomiar HIGH — różny od poprzedniego snapshot
        if (ht > 0 && ht != g_pasat_prev_high_us) {
            g_pasat_prev_high_us = ht;
            if (ht < g_pasat_min_halfperiod_us) g_pasat_min_halfperiod_us = ht;
            if (ht > g_pasat_max_halfperiod_us) g_pasat_max_halfperiod_us = ht;
            g_pasat_sum_halfperiod_us += ht;
            g_pasat_halfperiod_count++;
        }
        // Nowy pomiar LOW
        if (lt > 0 && lt != g_pasat_prev_low_us) {
            g_pasat_prev_low_us = lt;
            if (lt < g_pasat_min_halfperiod_us) g_pasat_min_halfperiod_us = lt;
            if (lt > g_pasat_max_halfperiod_us) g_pasat_max_halfperiod_us = lt;
            g_pasat_sum_halfperiod_us += lt;
            g_pasat_halfperiod_count++;
        }

        // Asymetria (gdy oba dostępne)
        if (ht > 0 && lt > 0) {
            uint32_t sum = ht + lt;
            uint32_t diff = (ht > lt) ? (ht - lt) : (lt - ht);
            uint32_t asym_x100 = diff * 100 / sum;  // asymetria w %
            // Zbieraj unikalne pomiary (zmiana któregokolwiek)
            if (ht != g_pasat_prev_high_us || lt != g_pasat_prev_low_us) {
                // Pomiary już zaktualizowane powyżej, ale asymetrię liczymy osobno
            }
            // Zawsze aktualizuj max asymetrię
            if (asym_x100 > g_pasat_max_asymmetry_x100) g_pasat_max_asymmetry_x100 = asym_x100;
            // Zbieraj średnią asymetrii co ~100ms
            static unsigned long s_last_asym_ms = 0;
            if (now - s_last_asym_ms >= 100) {
                s_last_asym_ms = now;
                g_pasat_sum_asymmetry_x100 += asym_x100;
                g_pasat_asymmetry_count++;
            }
        }

        // Progress co 2s
        unsigned long elapsed_s = (now - g_pasat_start_ms) / 1000;
        static unsigned long s_last_progress_s = UINT32_MAX;
        if (elapsed_s != s_last_progress_s && (elapsed_s % 2) == 0 && elapsed_s > 0) {
            s_last_progress_s = elapsed_s;
            uint32_t edges = g_pas_edge_count - g_pasat_edge_start;
            Serial.printf("[PASAT] %lu/%lus: %lu krawedzi, %lu pomiarow halfperiod\n",
                          elapsed_s, PASAT_MEASURE_S, (unsigned long)edges,
                          (unsigned long)g_pasat_halfperiod_count);
        }

        // Koniec pomiaru
        if ((now - g_pasat_start_ms) >= PASAT_MEASURE_S * 1000UL) {
            g_pasat_state = PASAT_DONE;
        }
        break;
    }

    case PASAT_DONE: {
        uint32_t total_edges = g_pas_edge_count - g_pasat_edge_start;

        Serial.println("[PASAT] === Wyniki pomiaru ===");
        Serial.printf("[PASAT] Krawedzi PAS:       %lu\n", (unsigned long)total_edges);
        Serial.printf("[PASAT] Pomiarow halfperiod: %lu\n", (unsigned long)g_pasat_halfperiod_count);

        if (g_pasat_halfperiod_count < 10) {
            Serial.println("[PASAT] BLAD: Za malo danych! Pedaluj szybciej/dluzej.");
            Serial.println("[PASAT] Parametry NIE zmienione.");
            g_pasat_state = PASAT_IDLE;
            break;
        }

        uint32_t avg_hp_us = (uint32_t)(g_pasat_sum_halfperiod_us / g_pasat_halfperiod_count);
        uint32_t avg_asym = (g_pasat_asymmetry_count > 0)
            ? (uint32_t)(g_pasat_sum_asymmetry_x100 / g_pasat_asymmetry_count)
            : 0;

        Serial.printf("[PASAT] Halfperiod min:     %lu us\n", (unsigned long)g_pasat_min_halfperiod_us);
        Serial.printf("[PASAT] Halfperiod max:     %lu us\n", (unsigned long)g_pasat_max_halfperiod_us);
        Serial.printf("[PASAT] Halfperiod avg:     %lu us\n", (unsigned long)avg_hp_us);
        Serial.printf("[PASAT] Asymetria avg:      %lu %%\n", (unsigned long)avg_asym);
        Serial.printf("[PASAT] Asymetria max:      %lu %%\n", (unsigned long)g_pasat_max_asymmetry_x100);

        // === Oblicz optymalne parametry ===
        // min_halfperiod: 50% minimalnego zmierzonego (margines na szybsze pedałowanie)
        uint32_t new_halfperiod_ms = (g_pasat_min_halfperiod_us / 2) / 1000;
        if (new_halfperiod_ms < 1) new_halfperiod_ms = 1;
        if (new_halfperiod_ms > 200) new_halfperiod_ms = 200;

        // debounce: 30% minimalnego półokresu (wystarczający do odfiltrowania szumów)
        uint32_t new_debounce_us = g_pasat_min_halfperiod_us * 30 / 100;
        if (new_debounce_us < 500) new_debounce_us = 500;
        if (new_debounce_us > 10000) new_debounce_us = 10000;
        // Zaokrąglij do wielokrotności PAS_SAMPLE_INTERVAL_US
        new_debounce_us = (new_debounce_us / PAS_SAMPLE_INTERVAL_US) * PAS_SAMPLE_INTERVAL_US;
        if (new_debounce_us < 500) new_debounce_us = 500;

        // asymetria: 60% średniej (z marginesem) ale minimum 3%
        uint32_t new_asymmetry = avg_asym * 60 / 100;
        if (new_asymmetry < 3) new_asymmetry = 3;
        if (new_asymmetry > 50) new_asymmetry = 50;

        Serial.println("[PASAT] === Nowe parametry ===");
        Serial.printf("[PASAT] pas_min_halfperiod_ms: %lu (bylo: %u)\n",
                      (unsigned long)new_halfperiod_ms, (unsigned)config_get().pas_min_halfperiod_ms);
        Serial.printf("[PASAT] pas_debounce_us:       %lu (bylo: %u)\n",
                      (unsigned long)new_debounce_us, (unsigned)config_get().pas_debounce_us);
        Serial.printf("[PASAT] pas_dir_asymmetry_pct: %lu (bylo: %u)\n",
                      (unsigned long)new_asymmetry, (unsigned)config_get().pas_dir_asymmetry_pct);

        // Zastosuj parametry
        controller_config_t& cfg = config_get();
        cfg.pas_min_halfperiod_ms = (uint8_t)new_halfperiod_ms;
        g_pas_min_halfperiod_us = new_halfperiod_ms * 1000;

        cfg.pas_debounce_us = (uint16_t)new_debounce_us;
        g_pas_debounce_us_isr = new_debounce_us;
        uint8_t depth = (uint8_t)(new_debounce_us / PAS_SAMPLE_INTERVAL_US);
        if (depth < 2) depth = 2;
        g_pas_filter_depth = depth;

        cfg.pas_dir_asymmetry_pct = (uint8_t)new_asymmetry;
        g_pas_dir_min_asymmetry = (uint8_t)new_asymmetry;

        config_save();

        Serial.println("[PASAT] Parametry zapisane do NVS.");
        Serial.println("[PASAT] Slew rate i holdoff nie zmienione (ustaw recznie: passlew:N, pashold:N).");
        Serial.println("[PASAT] === Auto-tune zakonczony ===");

        g_pasat_state = PASAT_IDLE;
        break;
    }

    default:
        g_pasat_state = PASAT_IDLE;
        break;
    }
}

/**
 * @brief Krok maszyny stanów kalibracji czujnika SPEED.
 *
 * Procedura: użytkownik kręci kołem dokładnie 3 pełne obroty w 15 sekund.
 * Firmware zlicza impulsy SPEED i oblicza pulses_per_rev = total / 3.
 * Wynik zapisywany do NVS.
 */
static void spdCalStep() {
    if (g_spdcal_state == SPDCAL_IDLE) return;

    unsigned long now = millis();

    switch (g_spdcal_state) {
    case SPDCAL_INIT: {
        g_spdcal_start_ms = now;
        g_spdcal_pulse_start = g_speed_pulse_count;
        Serial.println("[SPDCAL] === Kalibracja SPEED start ===");
        Serial.printf("[SPDCAL] Obroc kolo dokladnie 3 razy w ciagu %lu sekund.\n", SPDCAL_MEASURE_S);
        Serial.println("[SPDCAL] Silnik musi byc wylaczony! Krec reka.");
        g_spdcal_state = SPDCAL_MEASURE;
        break;
    }

    case SPDCAL_MEASURE: {
        unsigned long elapsed_s = (now - g_spdcal_start_ms) / 1000;
        static unsigned long s_last_progress_s = UINT32_MAX;
        if (elapsed_s != s_last_progress_s && (elapsed_s % 3) == 0 && elapsed_s > 0) {
            s_last_progress_s = elapsed_s;
            uint32_t pulses = g_speed_pulse_count - g_spdcal_pulse_start;
            Serial.printf("[SPDCAL] %lu/%lus: %lu impulsow\n",
                          elapsed_s, SPDCAL_MEASURE_S, (unsigned long)pulses);
        }
        if ((now - g_spdcal_start_ms) >= SPDCAL_MEASURE_S * 1000UL) {
            g_spdcal_state = SPDCAL_DONE;
        }
        break;
    }

    case SPDCAL_DONE: {
        uint32_t total_pulses = g_speed_pulse_count - g_spdcal_pulse_start;

        Serial.println("[SPDCAL] === Wyniki kalibracji ===");
        Serial.printf("[SPDCAL] Impulsy SPEED:     %lu\n", (unsigned long)total_pulses);

        if (total_pulses < 6) {
            Serial.println("[SPDCAL] BLAD: Za malo impulsow (<6)! Krec szybciej/sprawdz czujnik.");
            Serial.println("[SPDCAL] Parametry NIE zmienione.");
            g_spdcal_state = SPDCAL_IDLE;
            break;
        }

        // Zaokrąglij do najbliższej: total_pulses / 3 obrotów
        uint8_t ppr = (uint8_t)((total_pulses + 1) / 3);
        if (ppr < 1) ppr = 1;
        if (ppr > 20) ppr = 20;

        Serial.printf("[SPDCAL] Obliczony pulses_per_rev: %d (bylo: %d)\n",
                      (int)ppr, (int)g_speed_pulses_per_rev);

        // Zastosuj i zapisz
        g_speed_pulses_per_rev = ppr;
        config_get().speed_pulses_per_rev = ppr;
        config_save();

        // Reset filtra mediany (nowe parametry → czysta historia)
        g_speed_period_valid = 0;
        g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
        g_speed_last_accepted_us = 0;
        g_speed_outlier_count = 0;

        Serial.println("[SPDCAL] Zapisano do NVS.");
        Serial.printf("[SPDCAL] Wheeltime teraz = period_us * %d\n", (int)ppr);
        Serial.println("[SPDCAL] === Kalibracja zakonczona ===");

        g_spdcal_state = SPDCAL_IDLE;
        break;
    }

    default:
        g_spdcal_state = SPDCAL_IDLE;
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
        // wheel_speed_kmh obliczana po aktualizacji wheeltime_ms (w display block poniżej)
        float speed_kmh = g_bldc_state.wheel_speed_kmh;

        // --- Throttle duty ---
        // Manetka ZAWSZE działa w pełnym zakresie mocy (0..PWM_MAX_DUTY),
        // niezależnie od ustawionego poziomu wspomagania.
        // Assist level ogranicza tylko PAS (wspomaganie pedałowania).
        // Gdy assist_level == 0 → manetka też wyłączona (silnik OFF).
        uint16_t throttle_duty = (maxDuty > 0)
            ? mapThrottleToDuty(g_bldc_state.throttle_raw, PWM_MAX_DUTY)
            : 0;

        // --- PAS duty (ograniczone assist level + wbudowany speed fade P08) ---
        uint16_t pas_duty = calculatePasDuty(maxDuty);
        g_bldc_state.pas_duty = pas_duty;

        // --- Kombinacja P10 ---
        uint16_t combined_duty;
        uint8_t p10 = g_display.config.p10_drive_mode;
        switch (p10) {
            case 1:  // Tylko gaz (throttle only)
                combined_duty = throttle_duty;
                break;
            case 2:  // Tylko PAS (pedal assist only)
                combined_duty = pas_duty;
                break;
            default: // 0 lub nierozpoznany: PAS + gaz (wyższy wygrywa)
                combined_duty = (throttle_duty > pas_duty) ? throttle_duty : pas_duty;
                break;
        }

        // --- Globalny limit prędkości P08 z power fade ---
        // Dotyczy WSZYSTKICH źródeł duty (manetka + PAS).
        // Moc maleje w strefie 80%..100% limitu, powyżej = 0 (coast).
        // PAS ma własny speed fade wbudowany, ale globalny limit jest nadrzędny
        // i dotyczy też manetki.
        combined_duty = applyGlobalSpeedLimit(combined_duty, speed_kmh);

        // --- Freewheel: jeśli rowerzysta jedzie szybciej niż max silnika ---
        // Sprawdzamy efektywny limit prędkości dla aktualnego źródła:
        //   Manetka: P08 (max speed) — zawsze pełny zakres
        //   PAS: PAS target speed (zależy od assist level i kadencji)
        // Jeśli prędkość koła >= efektywny limit → duty = 0 (coast)
        // Motor zostanie ponownie włączony gdy prędkość spadnie < 80% limitu.
        // (to jest już obsłużone przez applyGlobalSpeedLimit powyżej)

        g_bldc_state.duty_target = combined_duty;
    }

    // Rampa dwukierunkowa: duty zmienia się płynnie w OBU kierunkach.
    // Czas rampy = ramp_time_ms (0→100% i 100%→0%).
    // Dodatkowy limit: duty_max_step_pct (max % zmiany na wywołanie, EEPROM).
    // Hamulec = natychmiastowe zerowanie (safety override).
    {
        uint16_t target = g_bldc_state.duty_target;
        unsigned long now_us = micros();
        unsigned long dt_us = now_us - g_ramp_last_us;
        g_ramp_last_us = now_us;

        if (g_bldc_state.brake_active) {
            // Hamulec → zeruj rampę (po puszczeniu hamulca silnik startuje płynnie od 0)
            g_duty_ramped = 0;
        } else if (target != g_duty_ramped) {
            // Oblicz max krok z ramp_time_ms (time-based)
            uint32_t ramp_step;
            if (g_bldc_state.ramp_time_ms > 0) {
                ramp_step = (uint32_t)PWM_MAX_DUTY * dt_us
                            / ((uint32_t)g_bldc_state.ramp_time_ms * 1000UL);
                if (ramp_step < 1) ramp_step = 1;
            } else {
                ramp_step = PWM_MAX_DUTY;  // ramp wyłączony
            }

            // Oblicz max krok z duty_max_step_pct (per-call clamp, EEPROM)
            uint8_t pct = config_get().duty_max_step_pct;
            uint32_t pct_step = (pct > 0 && pct <= 100)
                ? ((uint32_t)PWM_MAX_DUTY * pct / 100)
                : PWM_MAX_DUTY;  // 0 lub >100 → brak limitu

            // Użyj bardziej restrykcyjnego limitu
            uint32_t max_step = (ramp_step < pct_step) ? ramp_step : pct_step;
            if (max_step < 1) max_step = 1;

            if (target > g_duty_ramped) {
                uint16_t diff = target - g_duty_ramped;
                g_duty_ramped += (diff > max_step) ? (uint16_t)max_step : diff;
            } else {
                uint16_t diff = g_duty_ramped - target;
                g_duty_ramped -= (diff > max_step) ? (uint16_t)max_step : diff;
            }
        }
        g_bldc_state.duty_cycle = g_duty_ramped;
    }

    // ============================================================================
    // FOC: pętla prądowa (Clarke → Park → PI → Vd/Vq)
    // ============================================================================
    // Działa z częstotliwością loop() (~2kHz). ISR (20kHz) odczytuje Vd/Vq
    // i generuje SVPWM z aktualnym kątem θ (inverse Park + inverse Clarke).
    if (g_bldc_state.mode == DRIVE_MODE_FOC) {
        if (g_bldc_state.duty_cycle > 0) {
            // ── Tryb napięciowy (fvolt): Vq = duty, Vd = 0, bez PI ──
            // Działa identycznie jak SINUS ale w ramie dq.
            // Pozwala porównać "sinus vs FOC" i wykluczyć PI jako źródło hałasu.
            if (g_foc_voltage_mode) {
                float vq = (float)g_bldc_state.duty_cycle;
                if (vq > (float)SINE_SAFE_MAX_DUTY) vq = (float)SINE_SAFE_MAX_DUTY;
                g_foc_vd_i = 0;
                g_foc_vq_i = (int32_t)vq;
                g_foc_vd_dbg = 0.0f;
                g_foc_vq_dbg = vq;
                g_foc_iq_target = vq;  // dla debug wyświetlania
                g_foc_id_meas = 0.0f;
                g_foc_iq_meas = 0.0f;
            } else {
            // ── Tryb PI (normalny FOC) ──

            // 1. Mapowanie duty → Iq target (torque)
            {
                float iq_max = FOC_IQ_MAX;
                uint8_t ilim = getEffectiveCurrentLimit();
                if (ilim > 0 && (float)ilim < iq_max) {
                    iq_max = (float)ilim;  // Warstwa 2: clamp Iq do P14/NVS
                }
                g_foc_iq_target = (float)g_bldc_state.duty_cycle * iq_max / (float)PWM_MAX_DUTY;
            }

            // 2. Feedforward napięciowy: Vq_ff = duty (natychmiastowe napięcie)
            // PI dodaje tylko małą korektę wokół tego punktu pracy.
            // Bez feedforward: PI integruje od zera do limitu
            // → przy Ki=5, err=1.5A → ~7.5 PWM/s → 20s do 153 PWM.
            // Z feedforward: Vq = duty natychmiast + PI(±korekta).
            float ff_vq = (float)g_bldc_state.duty_cycle;
            if (ff_vq > (float)SINE_SAFE_MAX_DUTY) ff_vq = (float)SINE_SAFE_MAX_DUTY;
            // Feedforward dla Vd = 0 (chcemy Id = 0)
            float ff_vd = 0.0f;

            // Globalny limit napięcia (bezpieczeństwo)
            float amp_limit = fabsf(ff_vq);  // max = |feedforward| (duty)

            // PI limit = mała korekta wokół feedforward
            float pi_limit = FOC_PI_CORR_LIMIT;
            if (pi_limit > amp_limit) pi_limit = amp_limit;  // korekta nie większa niż FF
            g_foc_pi_d.limit = pi_limit;
            g_foc_pi_q.limit = pi_limit;

            // 3. Rekonstrukcja prądów ze znakiem (INA180A2 jest jednokierunkowy)
            // Użyj EMA-filtrowanych prądów (wygładzone z niesynchr. ADC).
            // Czujnik klipuje prąd ujemny do ~0V. Kirchhoff: Ia+Ib+Ic=0,
            // więc faza z najmniejszym odczytem = -(suma pozostałych dwóch).
            // ADC mierzy fizyczne fazy. Brak zamiany B↔C – Park działa
            // bezpośrednio na pomiarach fizycznych w obu kierunkach.
            float ia = g_foc_ia_ema;
            float ib = g_foc_ib_ema;
            float ic = g_foc_ic_ema;

            if (ia <= ib && ia <= ic) {
                ia = -(ib + ic);
            } else if (ib <= ia && ib <= ic) {
                ib = -(ia + ic);
            } else {
                ic = -(ia + ib);
            }

            // 4. Clarke: Ia,Ib,Ic → Iα,Iβ
            float i_alpha = ia;
            float i_beta  = (ia + 2.0f * ib) * FOC_INV_SQRT3;

            // 5. Park: Iα,Iβ → Id,Iq (używając aktualnego kąta z ISR)
            // Znaki dopasowane do konwencji inverse Park (zob. focCommutateISR):
            //   Id =  Iα·cos + Iβ·sin
            //   Iq =  Iα·sin - Iβ·cos
            uint32_t angle = g_sine_angle_q16;  // volatile → local copy
            int32_t sin_val = sine_interp_q16(angle);
            uint32_t cos_angle = angle + (24UL << 16);  // +90°
            if (cos_angle >= SINE_TABLE_Q16_FULL) cos_angle -= SINE_TABLE_Q16_FULL;
            int32_t cos_val = sine_interp_q16(cos_angle);
            float sin_f = (float)sin_val * (1.0f / 1024.0f);
            float cos_f = (float)cos_val * (1.0f / 1024.0f);

            float id =  i_alpha * cos_f + i_beta * sin_f;
            float iq =  i_alpha * sin_f - i_beta * cos_f;
            g_foc_id_meas = id;
            g_foc_iq_meas = iq;

            // 6. Oblicz dt (czas od ostatniej iteracji)
            unsigned long now_foc_us = micros();
            float dt = (float)(now_foc_us - g_foc_last_loop_us) / 1000000.0f;
            g_foc_last_loop_us = now_foc_us;
            if (dt <= 0.0f || dt > 0.05f) dt = FOC_LOOP_DT;  // clamp (max 50ms)

            float vd, vq;

            if (g_foc_at_active) {
                // ── PI Auto-tune: relay feedback (Åström-Hägglund) ──
                // Vd = 0 (jak normalnie), relay tylko na osi q.
                vd = ff_vd;

                float err_q = g_foc_iq_target - iq;
                uint32_t now_at_ms = (uint32_t)(millis());

                // Detekcja przejścia przez zero błędu
                if (g_foc_at_crossings > 0 || (g_foc_at_err_prev != 0.0f)) {
                    if ((err_q > 0.0f && g_foc_at_err_prev <= 0.0f) ||
                        (err_q < 0.0f && g_foc_at_err_prev >= 0.0f)) {
                        // Zapisz amplitudę z zakończonego pół-cyklu
                        if (g_foc_at_crossings > 0) {
                            float amp = (g_foc_at_err_max - g_foc_at_err_min) / 2.0f;
                            if (amp > 0.001f) {
                                g_foc_at_amp_sum += amp;
                                g_foc_at_amp_count++;
                            }
                        }
                        g_foc_at_err_max = err_q;
                        g_foc_at_err_min = err_q;
                        g_foc_at_crossings++;
                        if (g_foc_at_crossings == 1) g_foc_at_first_cross_ms = now_at_ms;
                        g_foc_at_last_cross_ms = now_at_ms;
                    }
                }
                // Śledzenie min/max błędu w bieżącym pół-cyklu
                if (err_q > g_foc_at_err_max) g_foc_at_err_max = err_q;
                if (err_q < g_foc_at_err_min) g_foc_at_err_min = err_q;
                g_foc_at_err_prev = err_q;

                // Wyjście relay: +/- relay_amp wokół feedforward
                float relay_out = (err_q > 0.0f) ? g_foc_at_relay_amp : -g_foc_at_relay_amp;
                vq = ff_vq + relay_out;

                // Sprawdzenie zakończenia testu
                if ((now_at_ms - g_foc_at_start_ms) >= g_foc_at_duration_ms) {
                    g_foc_at_active = false;
                    // Reset PI integratorów po relay
                    g_foc_pi_d.integral = 0.0f;
                    g_foc_pi_q.integral = 0.0f;

                    // Oblicz wyniki
                    Serial.println();
                    Serial.println("[PI Auto-tune] Zakończony.");
                    if (g_foc_at_crossings >= 4 && g_foc_at_amp_count > 0) {
                        // Średnia amplituda oscylacji Iq [A]
                        float avg_amp = g_foc_at_amp_sum / (float)g_foc_at_amp_count;
                        // Okres oscylacji Tu [s]
                        float tu_s = (float)(g_foc_at_last_cross_ms - g_foc_at_first_cross_ms)
                                     / (float)(g_foc_at_crossings - 1) * 2.0f / 1000.0f;
                        // Ultimate gain: Ku = 4*d/(π*a)
                        float ku = 4.0f * g_foc_at_relay_amp / (3.14159f * avg_amp);

                        // Konserwatywne PI (Tyreus-Luyben, bezpieczniejsze niż Z-N):
                        // Kp = 0.30*Ku (Z-N: 0.45), Ki = Kp/(2.2*Tu)  (Z-N: Kp*1.2/Tu)
                        float kp_new = 0.30f * ku;
                        float ki_new = (tu_s > 0.001f) ? (kp_new / (2.2f * tu_s)) : 0.0f;

                        // Sanity clamp — zapobiegaj absurdalnym wartościom
                        if (kp_new > 5.0f) kp_new = 5.0f;
                        if (ki_new > 50.0f) ki_new = 50.0f;
                        if (kp_new < 0.01f) kp_new = FOC_KP_DEFAULT;
                        if (ki_new < 0.01f) ki_new = FOC_KI_DEFAULT;

                        Serial.printf("  Oscylacje: %d przejsc, amplituda=%.3f A, Tu=%.4f s\n",
                                      g_foc_at_crossings, avg_amp, tu_s);
                        Serial.printf("  Ku=%.3f, Tu=%.4f s\n", ku, tu_s);
                        Serial.printf("  >> Zastosowane: Kp=%.3f  Ki=%.3f\n", kp_new, ki_new);

                        // Zastosuj obliczone wartości i zapisz do EEPROM
                        g_foc_pi_d.kp = kp_new;  g_foc_pi_d.ki = ki_new;
                        g_foc_pi_q.kp = kp_new;  g_foc_pi_q.ki = ki_new;
                        config_get().foc_kp_q = kp_new;  config_get().foc_ki_q = ki_new;
                        config_get().foc_kp_d = kp_new;  config_get().foc_ki_d = ki_new;
                        config_save();
                        Serial.println("  [PI Auto-tune] Wartości zapisane do EEPROM.");
                    } else {
                        Serial.printf("  Za malo oscylacji (%d crossings). Zwieksz duty lub relay_amp.\n",
                                      g_foc_at_crossings);
                    }
                }
            } else {
                // ── Normalny tryb PI ──

            // 7. PI regulator osi d: target Id = 0 (MTPA — max torque per amp)
            float err_d = 0.0f - id;
            g_foc_pi_d.integral += g_foc_pi_d.ki * err_d * dt;
            if (g_foc_pi_d.integral >  pi_limit) g_foc_pi_d.integral =  pi_limit;
            if (g_foc_pi_d.integral < -pi_limit) g_foc_pi_d.integral = -pi_limit;
            float corr_d = g_foc_pi_d.kp * err_d + g_foc_pi_d.integral;
            if (corr_d >  pi_limit) corr_d =  pi_limit;
            if (corr_d < -pi_limit) corr_d = -pi_limit;
            vd = ff_vd + corr_d;  // feedforward + korekta

            // 8. PI regulator osi q: target Iq z przepustnicy
            float err_q = g_foc_iq_target - iq;
            g_foc_pi_q.integral += g_foc_pi_q.ki * err_q * dt;
            if (g_foc_pi_q.integral >  pi_limit) g_foc_pi_q.integral =  pi_limit;
            if (g_foc_pi_q.integral < -pi_limit) g_foc_pi_q.integral = -pi_limit;
            float corr_q = g_foc_pi_q.kp * err_q + g_foc_pi_q.integral;
            if (corr_q >  pi_limit) corr_q =  pi_limit;
            if (corr_q < -pi_limit) corr_q = -pi_limit;
            vq = ff_vq + corr_q;  // feedforward + korekta
            }  // koniec if/else (autotune / normalny PI)

            // 9. Clamp globalny: Vd,Vq do bezpiecznego zakresu
            float v_max = (float)SINE_SAFE_MAX_DUTY;
            if (vd >  v_max) vd =  v_max;
            if (vd < -v_max) vd = -v_max;
            if (vq >  v_max) vq =  v_max;
            if (vq < -v_max) vq = -v_max;

            // 10. Clamp wektora napięciowego: |V| ≤ SINE_SAFE_MAX_DUTY
            float v_mag_sq = vd * vd + vq * vq;
            float v_lim_sq = v_max * v_max;
            if (v_mag_sq > v_lim_sq && v_mag_sq > 0.01f) {
                float scale = v_max / sqrtf(v_mag_sq);
                vd *= scale;
                vq *= scale;
                g_foc_pi_d.integral *= scale;
                g_foc_pi_q.integral *= scale;
            }

            // 11. Zapisz Vd/Vq dla ISR (inverse Park + SVPWM) jako int32_t
            // ISR NIE może używać float (brak zapisu kontekstu FPU w timer ISR).
            g_foc_vd_i = (int32_t)vd;
            g_foc_vq_i = (int32_t)vq;
            g_foc_vd_dbg = vd;  // kopia float do debugowania
            g_foc_vq_dbg = vq;
            }  // koniec else (tryb PI)
        } else {
            // Duty = 0 → reset PI integratorów
            g_foc_pi_d.integral = 0.0f;
            g_foc_pi_q.integral = 0.0f;
            g_foc_vd_i = 0;
            g_foc_vq_i = 0;
            g_foc_vd_dbg = 0.0f;
            g_foc_vq_dbg = 0.0f;
            g_foc_iq_target = 0.0f;
        }
    }

    // Aktualizacja zmiennych ISR z głównego stanu
    g_hall_isr = g_bldc_state.hall_state;
    {
        uint16_t raw_duty = g_bldc_state.duty_cycle;
        uint16_t min_duty = ((uint16_t)config_get().duty_min_pct * PWM_MAX_DUTY) / 100;
        if (raw_duty > 0 && raw_duty < min_duty) raw_duty = 0;
        g_duty_isr = raw_duty;
    }
    g_motor_enabled = (g_bldc_state.mode != DRIVE_MODE_DISABLED) &&
        (g_bldc_state.duty_cycle > 0 || g_bldc_state.mode == DRIVE_MODE_SINUS || g_bldc_state.mode == DRIVE_MODE_FOC);
    g_brake_isr = g_bldc_state.brake_active;
    g_mode_isr = g_bldc_state.mode;

    // Sync kierunku obrotów z konfiguracji NVS do ISR
    g_reverse_isr = (config_get().motor_reverse != 0);

    // Prędkość sinusoidalna (speed_q16) jest teraz obliczana bezpośrednio
    // w ISR (onCommutationTimer) na przejściu Halla — bez opóźnienia loop().
    // Tu nie ma nic do roboty — zostawione jako komentarz dla czytelności.

    // Obliczanie mocy: P = Vbat × max(Ia, Ib, Ic)
    float g_maxI_now = 0.0f;  // max prąd fazowy — surowy (do wyświetlacza/mocy)
    float g_maxI_filtered;    // EMA-filtrowany max prąd (do limitera prądowego)
    {
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        g_maxI_now = maxI;

        // EMA filtr prądu dla limitera — wygładza szum ADC.
        // ADC zsynchronizowany z PWM (ISR TEZ) → odczyty stabilne.
        // EMA α=0.15 → τ≈3ms przy loop ~1kHz.
        g_ilim_current_ema += ILIMIT_EMA_ALPHA * (maxI - g_ilim_current_ema);
        if (g_ilim_current_ema < 0.0f) g_ilim_current_ema = 0.0f;
        g_maxI_filtered = g_ilim_current_ema;

        float power = g_bldc_state.battery_voltage * g_maxI_filtered;

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

    // ====================================================================
    // Limit prądowy (3 warstwy: soft P-regulator, Iq clamp, hard cutoff)
    // ====================================================================
    // Warstwa 1 (soft): g_current_limit_factor (0..1) — mnożnik duty.
    //   Gdy maxI > limit → szybko obcina duty (Kp_down × error).
    //   Gdy maxI < limit → wolno wraca do 1.0 (recovery rate).
    // Warstwa 2 (FOC): Iq target obcinany do limitu (w sekcji FOC powyżej).
    // Warstwa 3 (hard): maxI > 150% → natychmiastowy duty=0 na 500ms.
    {
        uint8_t limit_a = getEffectiveCurrentLimit();

        if (limit_a > 0 && g_motor_enabled) {
            float flimit = (float)limit_a;

            // --- Warstwa 3: hard cutoff (ochrona MOSFET/silnika) ---
            // Wymaga OVERCURRENT_CONSEC kolejnych próbek (filtrowanego prądu)
            // powyżej progu. Pojedyncza szpilka ADC nie wywoła cutoff.
            if (g_maxI_filtered > flimit * OVERCURRENT_HARD_MULT) {
                g_ilim_hard_count++;
                if (g_ilim_hard_count >= OVERCURRENT_CONSEC) {
                    g_overcurrent_fault = true;
                    g_overcurrent_fault_ms = millis();
                    g_duty_isr = 0;
                    g_bldc_state.duty_cycle = 0;
                    g_duty_ramped = 0;
                    Serial.printf("[ILIM] HARD CUTOFF! I_filt=%.1fA > %.0fA (150%% of %dA)\n",
                                  g_maxI_filtered, flimit * OVERCURRENT_HARD_MULT, limit_a);
                    g_ilim_hard_count = 0;
                }
            } else {
                g_ilim_hard_count = 0;
            }

            // Blokada po hard cutoff
            if (g_overcurrent_fault) {
                if (millis() - g_overcurrent_fault_ms < OVERCURRENT_FAULT_MS) {
                    g_duty_isr = 0;
                    g_current_limit_factor = 0.0f;
                } else {
                    g_overcurrent_fault = false;
                    g_current_limit_factor = 0.1f; // restart od 10%
                }
            } else {
                // --- Warstwa 1: soft P-regulator ---
                float error = g_maxI_filtered - flimit;
                if (error > 0.0f) {
                    // Przekroczenie → szybka redukcja
                    g_current_limit_factor -= ILIMIT_KP_DOWN * error;
                    if (g_current_limit_factor < 0.05f) g_current_limit_factor = 0.05f;
                } else {
                    // Pod limitem → wolne odzyskiwanie
                    // dt ≈ 0.5-1ms (loop ~1-2kHz)
                    g_current_limit_factor += ILIMIT_RECOVER_RATE * 0.001f;
                    if (g_current_limit_factor > 1.0f) g_current_limit_factor = 1.0f;
                }

                // Zastosuj mnożnik do duty_isr (BLOCK + SINUS + FOC backup)
                if (g_current_limit_factor < 1.0f) {
                    uint16_t limited = (uint16_t)((float)g_duty_isr * g_current_limit_factor);
                    g_duty_isr = limited;
                }
            }
        } else {
            // Brak limitu lub silnik wyłączony → pełna moc
            g_current_limit_factor = 1.0f;
            g_overcurrent_fault = false;
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
                // Przetwarzaj każdy nowy impuls ISR tylko raz (skip duplikatów)
                if (sp != g_speed_last_processed_sp) {
                    g_speed_last_processed_sp = sp;

                    // Stale median reset: jeśli mediana ważna ale żaden impuls
                    // nie zaakceptowany >3s, warunki się zmieniły — reset filtra
                    if (g_speed_period_valid >= 3 && g_speed_last_accepted_us > 0
                        && (now_us - g_speed_last_accepted_us) > 3000000) {
                        g_speed_period_valid = 0;
                        g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
                    }

                    // Outlier rejection: nowy pomiar akceptowany tylko gdy
                    // jest w zakresie 1/3..3x aktualnej mediany (gdy mediana dostępna).
                    bool accept = true;
                    if (g_speed_period_valid >= 3) {
                        uint32_t a = g_speed_period_buf[0];
                        uint32_t b = g_speed_period_buf[1];
                        uint32_t c = g_speed_period_buf[2];
                        if (a > b) { uint32_t t = a; a = b; b = t; }
                        if (b > c) { uint32_t t = b; b = c; c = t; }
                        if (a > b) { uint32_t t = a; a = b; b = t; }
                        uint32_t median = b;
                        if (sp < median / 3 || sp > median * 3) {
                            accept = false;
                            g_speed_outlier_count++;
                        }
                    }
                    if (accept) {
                        g_speed_period_buf[g_speed_period_idx] = sp;
                        g_speed_period_idx = (g_speed_period_idx + 1) % 3;
                        if (g_speed_period_valid < 3) g_speed_period_valid++;
                        g_speed_last_accepted_us = now_us;
                    }
                }
                // Oblicz medianę (lub surowy) do wheeltime
                if (g_speed_period_valid >= 3) {
                    uint32_t a = g_speed_period_buf[0];
                    uint32_t b = g_speed_period_buf[1];
                    uint32_t c = g_speed_period_buf[2];
                    if (a > b) { uint32_t t = a; a = b; b = t; }
                    if (b > c) { uint32_t t = b; b = c; c = t; }
                    if (a > b) { uint32_t t = a; a = b; b = t; }
                    wt_us = b;  // mediana
                } else if (g_speed_period_valid > 0) {
                    wt_us = g_speed_period_buf[(g_speed_period_idx + 2) % 3]; // ostatni zaakceptowany
                } else {
                    wt_us = sp;
                }
            } else if ((now_us - last) >= 3000000) {
                // Koło stanęło — reset mediany (włącznie z zawartością bufora)
                g_speed_period_valid = 0;
                g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
                g_speed_last_accepted_us = 0;
            }
            // Przelicz: period_jednego_impulsu × impulsy_na_obrót = czas_obrotu_koła
            if (wt_us > 0 && g_speed_pulses_per_rev > 1) {
                wt_us *= g_speed_pulses_per_rev;
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

        // Prędkość koła — oblicz tutaj, po aktualizacji wheeltime_ms
        g_bldc_state.wheel_speed_kmh = calculateWheelSpeedKmh();

        // Obsługa protokołu wyswietlacza
        s866_service(&g_display);
    }

    // Obsługa komend USB Serial (zawsze aktywna)
    processSerialCommands();

    // Serwer WWW konfiguracji: P17=1 → WiFi+HTTP ON, silnik OFF; P17=0 → WiFi OFF, wykonaj kolejkę
    // UWAGA: P17 przetwarzamy TYLKO gdy display podłączony. Przy odłączonym display
    // szum EMI (szczególnie z SINUS/FOC) generuje fałszywe ramki S866 na Serial2
    // (checksum XOR = 1/256 kolizji) → P17 "migocze" → fałszywy WiFi start/stop.
    {
        static uint8_t s_prev_p17 = 0;  // 0 = domyślne, nie triggeruje fałszywej zmiany
        if (g_display.connected) {
            const uint8_t p17 = g_display.config.p17_cruise_control;
            if (p17 != s_prev_p17) {
                s_prev_p17 = p17;
                if (p17 != 0) {
                    webConfigInit();
                } else {
                    webConfigStop();
                    // Zawsze przeladuj config z EEPROM po wyjsciu z WiFi —
                    // przywraca tryb silnika, wszystkie parametry runtime.
                    executeCommand("cfg:reload");
                    // Dopiero teraz wykonaj skoljkowana komende (silnik jest juz w trybie z EEPROM)
                    if (g_web_queued_cmd.length() > 0) {
                        String qr = executeCommand(g_web_queued_cmd);
                        Serial.printf("[WEB-Q] '%s' -> %s\n", g_web_queued_cmd.c_str(), qr.c_str());
                        g_web_queued_cmd = "";
                    }
                }
            }
        } else {
            // Display odłączony — wyłącz WiFi jeśli było aktywne
            if (g_wifi_active) {
                webConfigStop();
            }
            s_prev_p17 = 0;  // reset trackera
        }
        if (g_wifi_active) {
            webConfigHandle();
        }
    }

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

    // Debug prądów co 500ms
    if (g_debugCurrent && (millis() - g_lastDebugCurrentMs >= DEBUG_CURRENT_INTERVAL_MS)) {
        g_lastDebugCurrentMs = millis();
        float maxI = fmaxf(g_bldc_state.phase_current[0],
                           fmaxf(g_bldc_state.phase_current[1], g_bldc_state.phase_current[2]));
        float dutyPct = (float)g_bldc_state.duty_cycle * 100.0f / PWM_MAX_DUTY;
        // Idc ≈ Ifaz × duty (przybliżenie prądu DC bus z zasilacza)
        float idcEst = g_ilim_current_ema * dutyPct / 100.0f;
        Serial.printf("[I] A=%5.2f B=%5.2f C=%5.2f | max=%5.2f ema=%5.2f Idc~%.1f | d=%3.0f%% lim=%d fct=%.2f | raw=%u/%u/%u to=%lu\n",
            g_bldc_state.phase_current[0],
            g_bldc_state.phase_current[1],
            g_bldc_state.phase_current[2],
            maxI, g_ilim_current_ema, idcEst,
            dutyPct, (int)getEffectiveCurrentLimit(),
            g_current_limit_factor,
            (unsigned)g_phase_adc_raw_isr[0], (unsigned)g_phase_adc_raw_isr[1], (unsigned)g_phase_adc_raw_isr[2],
            (unsigned long)g_adc_timeout_count);
    }

    // Debug Hall co 200ms — śledzenie komutacji
    if (g_debugHall && (millis() - g_lastDebugHallMs >= DEBUG_HALL_INTERVAL_MS)) {
        g_lastDebugHallMs = millis();
        const char* pathNames[] = {"OFF", "BLK", "SIN", "FOC", "B-S"};
        uint8_t p = g_dbg_commut_path;
        if (p > 4) p = 0;
        Serial.printf("[H] raw=%d filt=%d sec=%d | spd=%lu stup=%u dir=%d | path=%s d=%u | edges=%lu glitch=%lu inv=%lu seqrej=%lu\n",
            (int)g_dbg_hall_raw_isr,
            (int)g_dbg_hall_filt_isr,
            (int)g_sine_last_hall_idx,
            (unsigned long)g_sine_speed_q16,
            (unsigned)g_sine_startup_count,
            (int)g_sine_dir,
            pathNames[p],
            (unsigned)g_duty_isr,
            (unsigned long)g_dbg_hall_edges,
            (unsigned long)g_dbg_hall_glitch,
            (unsigned long)g_dbg_hall_invalid,
            (unsigned long)g_dbg_hall_seq_reject);
    }

    // ── Event-driven debug: drukuj zdarzenia z ring buffera ISR ──
    if (g_debugCommutation) {
        static const char* modeTag[] = {"OFF", "BLK", "SIN", "FOC"};
        uint8_t printed = 0;
        while (g_dbg_evt_rd != g_dbg_evt_wr && printed < 16) {
            volatile dbg_evt_t* ev = &g_dbg_evt_ring[g_dbg_evt_rd];
            uint8_t m = ev->mode;
            if (m > 3) m = 0;
            float angDeg = (float)ev->angle_q16 / 65536.0f * 3.75f;
            Serial.printf("[CE] t=%lu H:%d>%d s:%d>%d dir:%+d spd:%lu ang:%.1f dt:%luus "
                          "d:%u dA:%d dB:%d dC:%d Ia:%u Ib:%u Ic:%u %s stup:%u%s%s%s",
                (unsigned long)ev->ts_us,
                (int)ev->hall_raw, (int)ev->hall_filt,
                (int)ev->sector_old, (int)ev->sector_new,
                (int)ev->dir,
                (unsigned long)ev->speed_q16,
                angDeg,
                (unsigned long)ev->hall_period_us,
                (unsigned)ev->duty,
                (int)ev->da, (int)ev->db, (int)ev->dc,
                (unsigned)ev->adc_a, (unsigned)ev->adc_b, (unsigned)ev->adc_c,
                modeTag[m],
                (unsigned)ev->startup_cnt,
                (ev->flags & 2) ? " SNAP" : "",
                (ev->flags & 4) ? " SEQREJ" : "",
                (ev->flags & 8) ? " REV" : "");
            if (m == 3) {
                Serial.printf(" Vd:%ld Vq:%ld", (long)ev->foc_vd, (long)ev->foc_vq);
            }
            Serial.println();
            g_dbg_evt_rd = (g_dbg_evt_rd + 1) & (DBG_EVT_RING_SIZE - 1);
            printed++;
        }
        // Raportuj overflow (utracone zdarzenia)
        uint32_t ovf = g_dbg_evt_overflow;
        if (ovf > 0) {
            g_dbg_evt_overflow = 0;
            Serial.printf("[CE] OVERFLOW: %lu events lost\n", (unsigned long)ovf);
        }
    }

    // Debug FOC co 200ms
    if (g_foc_debug && g_bldc_state.mode == DRIVE_MODE_FOC
        && (millis() - g_foc_last_debug_ms >= 200)) {
        g_foc_last_debug_ms = millis();
        Serial.printf("[FOC%s] Vd=%5.1f Vq=%5.1f | Id=%5.2f Iq=%5.2f | tgt=%4.1f lim=%3.0f ff=%3.0f | ia=%5.2f ib=%5.2f ic=%5.2f\n",
            g_foc_voltage_mode ? "-V" : "-PI",
            g_foc_vd_dbg, g_foc_vq_dbg, g_foc_id_meas, g_foc_iq_meas,
            g_foc_iq_target, g_foc_pi_q.limit,
            (float)g_bldc_state.duty_cycle,  // feedforward value
            g_foc_ia_ema, g_foc_ib_ema, g_foc_ic_ema);
    }

    // Auto-tune fazy sinusoidalnej (maszyna stanów)
    autoTuneStep();

    // PAS auto-tune (maszyna stanów)
    pasAutoTuneStep();

    // SPEED calibration (maszyna stanów)
    spdCalStep();
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
 * @warning GPIO12 (PIN_PWM_A_HIGH) jest pinem STRAP ESP32.
 * Pull-up na GPIO12 during boot przestawia VDD_SDIO na 1.8V → brak uploadu do flash.
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

    // --- Czujnik temperatury FET (GPIO32/ADC1_CH4) ---
    // PIN_FET_TEMP — nie wymaga pinMode, input-only nie dotyczy, ale ustawiamy
    pinMode(PIN_FET_TEMP, INPUT);

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
// Zmiana częstotliwości PWM w runtime
// ============================================================================

/**
 * @brief Zmienia częstotliwość PWM przez modyfikację prescalera timera MCPWM.
 *
 * Period=500 (PWM_MAX_DUTY) nie zmienia się — duty 0-500 nadal odpowiada 0-100%.
 * Zmienia się tylko prescaler timera: timer_clk = 160MHz / prescaler.
 * freq = timer_clk / (2 × 500) = 160000 / prescaler
 * prescaler = round(160000 / freq_hz)
 *
 * Dead-time ticks skalowany do ~500ns: ticks = timer_clk * 500ns = 160M/pre * 500e-9
 *
 * @param freq_hz  Żądana częstotliwość [Hz], zakres 8000-32000
 * @return Faktyczna częstotliwość [Hz] po zaokrągleniu prescalera
 */
static uint16_t applyPwmFrequency(uint16_t freq_hz) {
    if (freq_hz < 8000) freq_hz = 8000;
    if (freq_hz > 32000) freq_hz = 32000;

    // prescaler = round(160000 / freq)
    uint8_t prescaler = (uint8_t)((160000UL + freq_hz / 2) / freq_hz);
    if (prescaler < 5) prescaler = 5;    // max 32kHz
    if (prescaler > 20) prescaler = 20;  // min 8kHz

    uint16_t actual_freq = (uint16_t)(160000UL / prescaler);

    // Dead-time: ~500ns = timer_clk * 500e-9 = (160M/pre) * 500e-9 = 80/pre
    uint8_t dt_ticks = (uint8_t)(80 / prescaler);
    if (dt_ticks < 2) dt_ticks = 2;

    // Wyłącz silnik na czas zmiany prescalera
    allMosfetsOff();

    // Zmień prescaler i dead-time dla wszystkich 3 timerów
    for (int t = 0; t < 3; t++) {
        mcpwm_ll_timer_set_clock_prescale(&MCPWM0, t, prescaler);
        // Period bez zmian (500) — PWM_MAX_DUTY zachowane
    }
    // Dead-time update (bypass mode — IR2103 robi own dead-time)
    // Nie potrzebujemy zmieniać DT bo IR2103 ma wbudowany

    g_pwm_freq_hz = actual_freq;
    return actual_freq;
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
 * Parametry PWM (MCPWM center-aligned):
 * - Częstotliwość: PWM_FREQUENCY = 20 kHz (UP_DOWN counter → symmetric)
 * - Rozdzielczość timera: 20 MHz → period = 500 counts (0..500)
 * - Dead time: bypass (IR2103 ma wewnętrzny ~520ns)
 * - Synchronizacja: Timer 1,2 zsynchronizowane z Timer 0 (TEZ)
 *
 * @note Po initPWM() wszystkie fazy są w stanie float (OFF).
 * allMosfetsOff() wywołuje to samo, ale jest idempotentna.
 */
void initPWM() {
    // GPIO init — przypisanie pinów do MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM_A_HIGH);  // Op0 gen_A → HIN_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_PWM_A_LOW);   // Op0 gen_B → LIN_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_PWM_B_HIGH);  // Op1 gen_A → HIN_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_PWM_B_LOW);   // Op1 gen_B → LIN_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_PWM_C_HIGH);  // Op2 gen_A → HIN_C
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PIN_PWM_C_LOW);   // Op2 gen_B → LIN_C

    // ── Konfiguracja timerów: UP_DOWN (center-aligned), 20 kHz ──
    // mcpwm_init() ustawia GPIO matrix, operator binding, counter mode itp.
    // Częstotliwość podana tutaj NIE jest dokładna — prescalery i period
    // wymuszamy ręcznie poniżej, żeby uzyskać DOKŁADNIE 20 kHz z period=500.
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;     // orientacyjna, nadpisana poniżej
    pwm_config.cmpr_a = 0.0f;               // duty A = 0%
    pwm_config.cmpr_b = 0.0f;               // duty B = 0%
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // active high
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;  // center-aligned!

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // ── Wymuszenie prescalerów + period dla DOKŁADNIE 20 kHz PWM ──
    // Timer clock = 160 MHz / group_pre / timer_pre = MCPWM_TIMER_RESOLUTION (20 MHz)
    // PWM freq (UP_DOWN) = timer_clk / (2 × period) = 20M / (2×500) = 20 kHz
    // BEZ TEGO: mcpwm_init() dobiera prescaler pod swoją period (np. 200),
    //           a wymuszenie period=500 obniża freq do ~16 kHz (SŁYSZALNE!)
    mcpwm_ll_group_set_clock_prescale(&MCPWM0, 1);  // group_clk = 160 MHz
    for (int t = 0; t < 3; t++) {
        mcpwm_ll_timer_set_clock_prescale(&MCPWM0, t, 160000000UL / MCPWM_TIMER_RESOLUTION);
        mcpwm_ll_timer_set_peak(&MCPWM0, t, MCPWM_TIMER_PERIOD, true);
    }

    // ── Synchronizacja timerów: Timer 1,2 syncowane z Timer 0 (TEZ) ──
    // Timer 0: master — generuje sync na TEZ (Timer Equals Zero)
    mcpwm_sync_config_t sync_master = {};
    sync_master.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    sync_master.timer_val = 0;
    sync_master.count_direction = MCPWM_TIMER_DIRECTION_UP;

    // Timer 0 generuje sync przy TEZ
    mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
    // Timer 1,2 syncują się z Timer 0
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_master);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_2, &sync_master);

    // ── Dead time: bypass (IR2103 ma wewnętrzny ~520ns dead time) ──
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_DEADTIME_BYPASS, 0, 0);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_DEADTIME_BYPASS, 0, 0);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_DEADTIME_BYPASS, 0, 0);

    // Startuj timery
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

    // ── Konfiguracja generatorów: action registers + force upmethod ──
    // mcpwm_init() ustawia gen_cntuforce_upmethod=32 (disable update).
    // Nasze ISR helpery zapisują gen_force.val bezpośrednio, więc potrzebujemy
    // upmethod=0 (immediate). Zapisz generator action registers (MODE_0)
    // i ustaw force upmethod na immediate dla wszystkich 3 operatorów.
    for (int op = 0; op < 3; op++) {
        MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
        MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE0;
        // force upmethod = 0 (immediate), force modes = 0 (disabled/PWM)
        MCPWM0.operators[op].gen_force.val = 0;

        // ── Compare shadow: update TYLKO na TEZ (nie TEZ+TEP!) ──
        // mcpwm_init() ustawia TEZ+TEP (stmp_cfg=0x33). W center-aligned PWM
        // to powoduje ASYMETRYCZNE impulsy: rampa UP używa starych wartości
        // compare, a DOWN nowych (po TEP shadow transfer). Szum!
        // TEZ-only → obie rampy używają tej samej wartości → symetryczny PWM.
        mcpwm_ll_operator_enable_update_compare_on_tez(&MCPWM0, op, 0, true);   // cmpr_a: TEZ ✓
        mcpwm_ll_operator_enable_update_compare_on_tep(&MCPWM0, op, 0, false);  // cmpr_a: !TEP
        mcpwm_ll_operator_enable_update_compare_on_tez(&MCPWM0, op, 1, true);   // cmpr_b: TEZ ✓
        mcpwm_ll_operator_enable_update_compare_on_tep(&MCPWM0, op, 1, false);  // cmpr_b: !TEP

        // Action register update: immediate (ważne przy przejściach BLOCK↔SIN/FOC)
        mcpwm_ll_operator_update_action_at_once(&MCPWM0, op);
    }

    // Inicjalny stan: wszystkie fazy OFF (float)
    allMosfetsOff();
}

// ============================================================================
// Wyłączenie wszystkich MOSFETów (stan bezpieczny)
// ============================================================================

/**
 * @brief Przełącza wszystkie tranzystory w stan OFF (stan bezpieczny / float).
 *
 * MCPWM: gen_A = forced LOW (HIN=LOW → HS OFF),
 *         gen_B = forced HIGH (LIN=HIGH → LS OFF, bo IR2103 LIN odwrócony).
 *
 * Wywoływana:
 * - po initPWM() podczas startu
 * - przy komendzie 'd' (disable)
 * - przy hamulcu w ISR
 * - przy błędnym stanie Halla (0 lub 7)
 *
 * @note Bezpieczna do wywołania z loop() i z ISR.
 */
void IRAM_ATTR allMosfetsOff() {
    mcpwm_phase_off(0);  // Faza A
    mcpwm_phase_off(1);  // Faza B
    mcpwm_phase_off(2);  // Faza C
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
    // Odczyt napięcia baterii — ADC1_CH0 (GPIO36)
    // Guard: wyłącz ISR ADC na czas odczytu, bo ISR też używa ADC1.
    g_adc_isr_active = false;
    uint16_t batteryRaw = analogRead(PIN_BATTERY_VOLTAGE);
    g_adc_isr_active = true;

    // ── Odczyt prądów fazowych: zsynchronizowany z PWM (ISR TEZ) ──
    // ISR onCommutationTimer czyta ADC prądu w dolinie center-aligned PWM
    // (counter=0, wszystkie low-side ON) → odczyt stabilny, bez szpilek.
    // Gdy ISR nie dostarczył danych (np. przed uruchomieniem timera),
    // fallback na analogRead() z medianą (stare zachowanie).
    uint16_t phaseA_raw, phaseB_raw, phaseC_raw;
    if (g_adc_ready_isr) {
        // Pobierz uśrednione odczyty z akumulatora ISR (wiele cykli PWM)
        portENTER_CRITICAL(&g_adc_mux);
        uint32_t sa = g_phase_adc_sum_isr[0];
        uint32_t sb = g_phase_adc_sum_isr[1];
        uint32_t sc = g_phase_adc_sum_isr[2];
        uint16_t cnt = g_phase_adc_cnt_isr;
        g_phase_adc_sum_isr[0] = 0;
        g_phase_adc_sum_isr[1] = 0;
        g_phase_adc_sum_isr[2] = 0;
        g_phase_adc_cnt_isr = 0;
        g_adc_ready_isr = false;
        portEXIT_CRITICAL(&g_adc_mux);
        if (cnt > 0) {
            phaseA_raw = (uint16_t)(sa / cnt);
            phaseB_raw = (uint16_t)(sb / cnt);
            phaseC_raw = (uint16_t)(sc / cnt);
        } else {
            phaseA_raw = phaseB_raw = phaseC_raw = 0;
        }
    } else {
        // Fallback: bezpośredni odczyt (przed init ISR lub ISR nieaktywny)
        g_adc_isr_active = false;
        phaseA_raw = analogRead(PIN_PHASE_A_CURRENT);
        phaseB_raw = analogRead(PIN_PHASE_B_CURRENT);
        phaseC_raw = analogRead(PIN_PHASE_C_CURRENT);
        g_adc_isr_active = true;
    }

    // Przepustnica: bufor kołowy N próbek (1 na iterację loop) + mediana
    // GPIO33/ADC1_CH5 — podatny na szpilki EMI od PWM silnika.
    // Zamiast burst (N próbek w ~100µs) — 1 próbka na loop() (~0.5ms).
    // Próbki rozłożone w czasie: szpilka EMI trwająca <N×0.5ms
    // zanieczyszcza tylko część bufora, mediana ją odrzuca.
    {
        static uint16_t thr_ring[16] = {0};
        static uint8_t  thr_ring_idx = 0;
        static bool     thr_ring_init = false;

        controller_config_t& tcfg = config_get();
        uint8_t n_samples = tcfg.thr_samples;
        if (n_samples < 2) n_samples = 2;
        if (n_samples > 16) n_samples = 16;
        uint16_t thresh = tcfg.thr_outlier_thresh;

        // 1 próbka ADC na wywołanie (rozłożone w czasie, nie burst)
        uint16_t raw = analogRead(PIN_THROTTLE);
        thr_ring[thr_ring_idx] = raw;
        thr_ring_idx = (thr_ring_idx + 1) % n_samples;

        // Inicjalizacja: wypełnij bufor pierwszą próbką
        if (!thr_ring_init) {
            for (uint8_t i = 0; i < 16; i++) thr_ring[i] = raw;
            thr_ring_init = true;
        }

        // Kopia do sortowania (nie modyfikujemy bufora kołowego)
        uint16_t sorted[16];
        for (uint8_t i = 0; i < n_samples; i++) sorted[i] = thr_ring[i];

        // Insertion sort
        for (uint8_t i = 1; i < n_samples; i++) {
            uint16_t key = sorted[i];
            int8_t j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        // Mediana — odporna na szpilki (do 50% bufora może być zaśmiecone)
        uint16_t median = sorted[n_samples / 2];

        // Średnia z próbek bliskich medianie (dodatkowe wygładzenie)
        uint32_t sum = 0;
        uint8_t count = 0;
        for (uint8_t i = 0; i < n_samples; i++) {
            int32_t diff = (int32_t)sorted[i] - (int32_t)median;
            if (diff < 0) diff = -diff;
            if ((uint16_t)diff <= thresh) {
                sum += sorted[i];
                count++;
            }
        }
        g_bldc_state.throttle_raw = (count > 0) ? (uint16_t)(sum / count) : median;
    }

    // Odczyt temperatury FET (ADC1_CH4/GPIO32) — PIN_FET_TEMP
    // Aktualnie PIN_FET_TEMP NIE jest czytany (czujnik niepodłączony).

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

    // FOC: zapisz prądy przed klipowaniem (Clarke/Park potrzebuje wartości ze znakiem)
    g_foc_ia_signed = ia;
    g_foc_ib_signed = ib;
    g_foc_ic_signed = ic;

    // FOC: EMA filtr prądów — wygładza szum ADC.
    // ADC zsynchronizowany z PWM (ISR TEZ) → odczyty stabilne, bez szpilek.
    // EMA wygładza niewielki szum kwantyzacji SAR ADC.
    if (g_bldc_state.mode == DRIVE_MODE_FOC) {
        g_foc_ia_ema += FOC_CURRENT_EMA_ALPHA * (ia - g_foc_ia_ema);
        g_foc_ib_ema += FOC_CURRENT_EMA_ALPHA * (ib - g_foc_ib_ema);
        g_foc_ic_ema += FOC_CURRENT_EMA_ALPHA * (ic - g_foc_ic_ema);
    }

    if (ia < 0.0f) ia = 0.0f;
    if (ib < 0.0f) ib = 0.0f;
    if (ic < 0.0f) ic = 0.0f;

    g_bldc_state.phase_current[0] = ia;
    g_bldc_state.phase_current[1] = ib;
    g_bldc_state.phase_current[2] = ic;
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
 * Debounce: wymagaj BRAKE_DEBOUNCE_THRESHOLD kolejnych odczytów LOW
 * zanim uzna hamulec za aktywny. Filtruje spike EMI z przełączania
 * FETów (szczególnie w SINUS/FOC, gdzie 6 FETów przełącza jednocześnie).
 * Bez debounce: spike EMI → brake_active=true → g_duty_ramped=0 →
 * silnik gwałtownie zwalnia i rozpędza się ponownie.
 */
void readDigitalInputs() {
    // Debounce hamulca
    bool brake_raw = (digitalRead(PIN_BRAKE) == LOW) || g_brake_simulated;
    if (brake_raw) {
        if (g_brake_debounce_count < BRAKE_DEBOUNCE_THRESHOLD) {
            g_brake_debounce_count++;
        }
    } else {
        g_brake_debounce_count = 0;
    }
    g_bldc_state.brake_active = (g_brake_debounce_count >= BRAKE_DEBOUNCE_THRESHOLD);
    // pas_active = pedałowanie (potwierdzony PAS albo okno init)
    g_bldc_state.pas_active = g_pas_pedaling;
    g_bldc_state.pas_forward = g_pas_forward;  // volatile → state
}

// ============================================================================
// Wyświetlanie diagnostyki
// ============================================================================

/**
 * @brief Wypisuje pojedynczą linię statusu na Serial.
 *
 * Format: `MODE D:duty% V:Vbat Ia:X.XX Ib:X.XX Ic:X.XX H:CBA T:temp Thr:thr%(raw) [flagi]`
 *
 * Przykład:
 * @code
 * BLK D:45% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1850)
 * @endcode
 *
 * Kolumny:
 * - MODE:    OFF/BLK/SIN/FOC (tryb sterowania)
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

    Serial.printf("%s%s D:%d/%d%% d_isr:%u CLF:%.2f V:%.1f Ia:%.2f Ib:%.2f Ic:%.2f Imax:%.2f H:%d%d%d Thr:%d%%(%d) RPM:%lu WT:%u P:%.1fW",
        modeNames[g_bldc_state.mode],
        g_reverse_isr ? "<" : ">",
        dutyPct, targetPct,
        (unsigned)g_duty_isr,
        g_current_limit_factor,
        g_bldc_state.battery_voltage,
        g_bldc_state.phase_current[0],
        g_bldc_state.phase_current[1],
        g_bldc_state.phase_current[2],
        g_ilim_current_ema,
        (g_bldc_state.hall_state >> 2) & 1,
        (g_bldc_state.hall_state >> 1) & 1,
        g_bldc_state.hall_state & 1,
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

    // PAS: linia diagnostyczna — zawsze wyświetlana
    {
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        uint32_t since_us = (g_pas_last_pulse_us > 0) ? (now_us - g_pas_last_pulse_us) : 0xFFFFFFFFUL;
        uint32_t ht = g_pas_high_time_us;
        uint32_t lt = g_pas_low_time_us;
        uint32_t period = ht + lt;

        // Asymetria [%]
        uint8_t asym_pct = 0;
        if (period > 0) {
            uint32_t diff = (ht > lt) ? (ht - lt) : (lt - ht);
            asym_pct = (uint8_t)(diff * 100UL / period);
        }

        // Kadencja [RPM] z okresu PAS i liczby magnesów
        float cadence_rpm = 0.0f;
        uint8_t magnets = g_display.config.p13_pas_magnets;
        if (magnets == 0) magnets = 1;
        if (period > 0 && since_us < 2000000UL) {
            cadence_rpm = 60000000.0f / ((float)period * (float)magnets);
        }

        // Stan słowny
        const char* pas_state;
        if (since_us > (uint32_t)config_get().pas_stop_delay_ms * 1000UL || since_us == 0xFFFFFFFFUL) {
            pas_state = "STOP";
        } else if (g_pas_pedaling) {
            pas_state = "ON";
        } else if (!g_pas_forward && g_pas_fwd_since_ms == 0) {
            pas_state = "REV";
        } else {
            pas_state = "WAIT";
        }

        // Czas od ostatniego impulsu [ms]
        uint32_t since_ms = (since_us == 0xFFFFFFFFUL) ? 9999 : (since_us / 1000);
        if (since_ms > 9999) since_ms = 9999;

        // Czas w trybie forward / do aktywacji
        uint32_t fwd_elapsed_ms = 0;
        if (g_pas_fwd_since_ms > 0) {
            fwd_elapsed_ms = (uint32_t)(esp_timer_get_time() / 1000) - g_pas_fwd_since_ms;
        }

        Serial.printf(
            "\n  [PAS] st:%s edges:%lu since:%lums H:%lums L:%lums asym:%d%% conf:%d fwd:%d "
            "rpm:%.0f fwd_ms:%lu/%u ped:%d inv:%d pin:%d",
            pas_state,
            (unsigned long)g_pas_edge_count,
            (unsigned long)since_ms,
            (unsigned long)(ht / 1000),
            (unsigned long)(lt / 1000),
            (int)asym_pct,
            (int)g_pas_dir_confidence,
            (int)g_pas_forward,
            cadence_rpm,
            (unsigned long)fwd_elapsed_ms,
            (unsigned)config_get().pas_start_delay_ms,
            (int)g_pas_pedaling,
            (int)g_pas_dir_invert_isr,
            digitalRead(PIN_PAS));

        // Jeśli PAS aktywny — pokaż duty i V_target
        if (g_pas_pedaling) {
            int pasDutyPct = (int)((uint32_t)g_bldc_state.pas_duty * 100 / PWM_MAX_DUTY);
            uint8_t ra = g_display.rx.assist_level;
            uint8_t vm = g_display.config.p08_speed_limit;
            if (vm == 0) vm = 25;
            float vt = 6.0f + (float)ra * ((float)vm - 6.0f) / 15.0f;
            Serial.printf(" d:%d%% vt:%.1f sl:%.2f", pasDutyPct, vt, g_speed_limit_factor);
        }
    }

    // Prędkość koła [km/h] + debug SPEED
    if (g_bldc_state.wheel_speed_kmh > 0.5f || g_speed_pulse_count > 0) {
        uint32_t age_us = 0;
        if (g_speed_last_pulse_us > 0) {
            age_us = (uint32_t)esp_timer_get_time() - g_speed_last_pulse_us;
        }
        Serial.printf("\n  [SPD] %.1fkm/h per:%lums wt:%u ppr:%d pls:%lu rej:%lu out:%lu age:%lums med:[%lu,%lu,%lu]",
            g_bldc_state.wheel_speed_kmh,
            (unsigned long)(g_speed_period_us / 1000),
            (unsigned)g_bldc_state.wheeltime_ms,
            (int)g_speed_pulses_per_rev,
            (unsigned long)g_speed_pulse_count,
            (unsigned long)g_speed_reject_count,
            (unsigned long)g_speed_outlier_count,
            (unsigned long)(age_us / 1000),
            (unsigned long)(g_speed_period_buf[0] / 1000),
            (unsigned long)(g_speed_period_buf[1] / 1000),
            (unsigned long)(g_speed_period_buf[2] / 1000));
    }

    // Debug SINUS/FOC: parametry śledzenia kąta
    if (g_bldc_state.mode == DRIVE_MODE_SINUS || g_bldc_state.mode == DRIVE_MODE_FOC) {
        uint32_t ang = g_sine_angle_q16;
        int32_t ang_entry = (int32_t)(ang >> 16);
        int32_t hall_err_entry = g_dbg_last_hall_err >> 16;  // w wpisach tabeli (1=3.75°)
        uint8_t hall_raw = g_bldc_state.hall_state;
        int8_t hall_sec = hallToSector(hall_raw);
        Serial.printf("\n  [DBG] ang:%ld spd:%lu hdir:%d sec:%d h:%d%d%d hs:%d hp:%luus err:%ld snp:%lu cor:%lu fb:%lu d:%u",
            (long)ang_entry,
            (unsigned long)g_sine_speed_q16,
            (int)g_sine_dir,
            (int)g_sine_last_hall_idx,
            (hall_raw >> 2) & 1,
            (hall_raw >> 1) & 1,
            hall_raw & 1,
            (int)hall_sec,
            (unsigned long)g_hall_period_us,
            (long)hall_err_entry,
            (unsigned long)g_dbg_snap_count,
            (unsigned long)g_dbg_corr_count,
            (unsigned long)g_dbg_sine_fallback_count,
            (unsigned)g_bldc_state.duty_cycle);
        if (g_bldc_state.mode == DRIVE_MODE_FOC) {
            Serial.printf(" vq:%ld iq:%.2f tgt:%.2f",
                (long)g_foc_vq_i,
                g_foc_iq_meas,
                g_foc_iq_target);
        }
    }

    // Informacje z wyświetlacza S866
    if (g_display.connected) {
        Serial.printf("DISP:OK L%d(r%d) %s%s",
            g_display.rx.assist_level / 3,
            g_display.rx.assist_level,
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

    Serial.printf("spd:%lu ang:%ld.%04ld amp:%u m:%d/%d/%d ",
        (unsigned long)g_sine_speed_q16,
        (long)angle_entry,
        (long)((angle_frac * 10000L) >> 16),
        (unsigned)g_dbg_last_amp,
        (int)g_dbg_last_ma, (int)g_dbg_last_mb, (int)g_dbg_last_mc);

    Serial.printf("ofs:%d ev[h:%lu gli:%lu seq:%lu snp:%lu cor:%lu]\n",
        (int)g_sine_hall_phase_offset,
        (unsigned long)g_dbg_hall_edges,
        (unsigned long)g_dbg_hall_glitch,
        (unsigned long)g_dbg_hall_seq_reject,
        (unsigned long)g_dbg_snap_count,
        (unsigned long)g_dbg_corr_count);

    // ── Rozszerzona diagnostyka (running stats z okna ~200ms) ──
    int32_t err_min_ent = g_dbg_err_min_q16 >> 16;
    int32_t err_max_ent = g_dbg_err_max_q16 >> 16;
    Serial.printf("  err[%ld..%ld] spd[%lu..%lu] dt[%lu..%lu]us snp/cor:%lu/%lu\n",
        (long)err_min_ent, (long)err_max_ent,
        (unsigned long)g_dbg_speed_min, (unsigned long)g_dbg_speed_max,
        (unsigned long)g_dbg_dt_min, (unsigned long)g_dbg_dt_max,
        (unsigned long)g_dbg_snap_window, (unsigned long)g_dbg_corr_window);
    // Kąt obserwera vs oczekiwany kąt (z ostatniego Hall edge w oknie)
    Serial.printf("  angH:%ld exp:%ld sec:%d dir:%d rev:%d\n",
        (long)(g_dbg_angle_at_hall >> 16),
        (long)(g_dbg_expected_at_hall >> 16),
        (int)g_sine_last_hall_idx,
        (int)g_sine_dir,
        (int)g_reverse_isr);

    // Reset running stats dla następnego okna
    g_dbg_err_min_q16 = INT32_MAX;
    g_dbg_err_max_q16 = INT32_MIN;
    g_dbg_speed_min = 0xFFFFFFFF;
    g_dbg_speed_max = 0;
    g_dbg_dt_min = 0xFFFFFFFF;
    g_dbg_dt_max = 0;
    g_dbg_snap_window = 0;
    g_dbg_corr_window = 0;
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
 * @warning Funkcje MCPWM force/duty są ISR-safe (operują na rejestrach sprzętowych).
 */

// ── ISR-safe ADC1 read — bezpośredni dostęp do rejestrów SAR ADC ──
// analogRead() używa mutex → nie wolno w ISR.
// Ta funkcja omija sterownik i czyta SAR ADC1 bezpośrednio (~3µs).
// Wymaga wcześniejszej konfiguracji kanału (adc1_config_channel_atten w setup).
// Timeout: max ~5µs (1200 cykli @240MHz) — zapobiega blokowaniu ISR
// gdy ADC jest zajęty przez analogRead() z loop().
static uint16_t IRAM_ATTR adc1_read_isr(uint8_t channel) {
    // Upewnij się, że SAR ADC1 jest w trybie RTC (software trigger)
    SENS.sar_meas_start1.meas1_start_force = 1;  // force start by software
    SENS.sar_read_ctrl.sar1_dig_force = 0;        // SAR1 controlled by RTC, not DIG
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    // Timeout: ~1200 iteracji ≈ 5µs @240MHz. Normalnie konwersja trwa ~2µs.
    uint32_t timeout = 1200;
    while (!SENS.sar_meas_start1.meas1_done_sar && --timeout) {}
    if (timeout == 0) { g_adc_timeout_count++; return 0; }  // ADC busy — zwróć 0
    return SENS.sar_meas_start1.meas1_data_sar;
}

void IRAM_ATTR onCommutationTimer(void *arg) {
    // Skasuj flagę przerwania TEZ Timer 0
    mcpwm_ll_intr_clear_timer_tez_status(&MCPWM0, 1 << 0);

    // ── Odczyt ADC prądu w dolinie PWM (TEZ) ──
    // TEZ = counter=0 w center-aligned: wszystkie low-side ON → prąd shuntu stabilny.
    // Czytaj NATYCHMIAST po wejściu do ISR, zanim okno zero-vector się skończy.
    // Trzy odczyty ~9µs — mieści się w oknie zero-vector (≥6µs przy 75% duty).
    // Flaga g_adc_isr_active=false gdy loop() czyta ADC1 (battery) → unikamy konfliktu HW.
    if (g_adc_isr_active) {
        uint16_t a = adc1_read_isr(ADC1_CHANNEL_3);  // Phase A (GPIO39)
        uint16_t b = adc1_read_isr(ADC1_CHANNEL_6);  // Phase B (GPIO34)
        uint16_t c = adc1_read_isr(ADC1_CHANNEL_7);  // Phase C (GPIO35)
        // Zachowaj ostatni raw do diagnostyki (idbg)
        g_phase_adc_raw_isr[0] = a;
        g_phase_adc_raw_isr[1] = b;
        g_phase_adc_raw_isr[2] = c;
        // Akumuluj do uśredniania w loop() — odrzucaj szpilki ADC per kanał.
        // Próbki > 3800 (~30A) to artefakty przełączania MOSFET, nie prawdziwy prąd.
        // Każdy kanał filtrowany niezależnie — spike na fazie A nie odrzuca fazy B/C.
        portENTER_CRITICAL_ISR(&g_adc_mux);
        if (a < 3800) { g_phase_adc_sum_isr[0] += a; } else { g_phase_adc_sum_isr[0] += g_phase_adc_last_good[0]; }
        if (b < 3800) { g_phase_adc_sum_isr[1] += b; g_phase_adc_last_good[1] = b; } else { g_phase_adc_sum_isr[1] += g_phase_adc_last_good[1]; }
        if (c < 3800) { g_phase_adc_sum_isr[2] += c; g_phase_adc_last_good[2] = c; } else { g_phase_adc_sum_isr[2] += g_phase_adc_last_good[2]; }
        if (a < 3800) { g_phase_adc_last_good[0] = a; }
        g_phase_adc_cnt_isr++;
        portEXIT_CRITICAL_ISR(&g_adc_mux);
        g_adc_ready_isr = true;
    }

    // Odczyt Halli ZAWSZE — pomiar prędkości nawet gdy silnik wyłączony
    uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
    uint8_t hb = (GPIO.in >> PIN_HALL_SENSOR_B) & 1;
    uint8_t hc = (GPIO.in >> PIN_HALL_SENSOR_C) & 1;
    uint8_t hall = (hc << 2) | (hb << 1) | ha;

    // ── Filtr Hall z wielopróbkowym potwierdzeniem (anty-EMI) ──
    // 3 warstwy:
    //   1. Odrzucenie nieprawidłowych stanów (0, 7)
    //   2. Multi-sample: nowy stan musi utrzymać się HALL_CONFIRM_COUNT ticków ISR
    //   3. Debounce czasowy: HALL_MIN_PERIOD_US od ostatniego potwierdzonego przejścia
    // Pod obciążeniem PWM generuje silne EMI sprzęgające się w linie Halla,
    // tworząc fałszywe przejścia. Glitch <150µs jest odrzucany.

    // Warstwa 1: odrzuć nieprawidłowe stany (brak/zwarcie czujnika)
    if (hall == 0 || hall == 7) {
        g_dbg_hall_invalid++;
        g_hall_confirm_cnt = 0;  // reset potwierdzenia — przerwany ciąg
    }
    else if (hall != g_hall_prev_isr) {
        // Warstwa 2: multi-sample confirmation
        if (hall == g_hall_candidate) {
            g_hall_confirm_cnt++;
        } else {
            // Kandydat się zmienił przed potwierdzeniem → prawdziwy glitch EMI
            if (g_hall_confirm_cnt > 0) g_dbg_hall_glitch++;
            g_hall_candidate = hall;
            g_hall_confirm_cnt = 1;
        }

        if (g_hall_confirm_cnt >= HALL_CONFIRM_COUNT) {
            // Potwierdzony! Warstwa 3: debounce czasowy
            g_hall_confirm_cnt = 0;
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            uint32_t dt_us = now_us - g_hall_last_change_us;

            uint32_t min_hall_period_us = HALL_MIN_PERIOD_US;
            if ((g_mode_isr == DRIVE_MODE_SINUS || g_mode_isr == DRIVE_MODE_FOC) &&
                g_sine_speed_q16 < SINE_BLOCK_SPEED_THRESHOLD) {
                min_hall_period_us = HALL_MIN_PERIOD_STARTUP_US;
            }

            if (g_hall_last_change_us == 0 || dt_us >= min_hall_period_us) {
                g_dbg_hall_edges++;
                g_dbg_last_hall = hall;
                if (g_hall_last_change_us > 0) {
                    g_hall_period_us = dt_us;
                }
                g_hall_last_change_us = now_us;

                // Block crossfade: zapisz stary remapowany Hall PRZED aktualizacją
                if (g_mode_isr == DRIVE_MODE_BLOCK && g_hall_prev_isr >= 1 && g_hall_prev_isr <= 6) {
                    g_block_old_bh = g_reverse_isr ? g_hall_reverse_map[g_hall_prev_isr] : g_hall_prev_isr;
                    g_block_cross_cnt = BLOCK_CROSS_TICKS;
                }
                g_hall_prev_isr = hall;

                // ── Sine/FOC mode: przetwarzanie przejścia Halla ──
                if (g_mode_isr == DRIVE_MODE_SINUS || g_mode_isr == DRIVE_MODE_FOC) {
                    int8_t new_idx = hallToSector(hall);
                    if (new_idx >= 0) {
                        int8_t old_idx = g_sine_last_hall_idx;
                        bool is_snap = false;

                        // Walidacja sekwencji: akceptuj tylko ±1 sektor
                        // Przeskok >1 sektora = prawdopodobnie glitch EMI który
                        // przeszedł multi-sample (np. stabilne pole EMI pod dużym obciążeniem).
                        // W takim przypadku: aktualizuj timestamp/idx ale NIE koryguj kąta/prędkości.
                        bool seq_valid = true;
                        if (old_idx >= 0) {
                            int8_t fwd = (old_idx + 1) % 6;
                            int8_t rev = (old_idx + 5) % 6;
                            if (new_idx == fwd) {
                                g_sine_dir = 1;   // prawidłowy kierunek
                            } else if (new_idx == rev) {
                                // Przy zablokowanym kole i małym duty szum Hall często wygląda jak
                                // naprzemienne ±1 sektor. Blokujemy takie "cofanie" na starcie.
                                uint16_t duty_lock_th = (uint16_t)(PWM_MAX_DUTY * SINE_DIR_LOCK_DUTY_PCT / 100);
                                bool startup_lock =
                                    (g_sine_speed_q16 < SINE_BLOCK_SPEED_THRESHOLD) &&
                                    (g_duty_isr <= duty_lock_th);
                                if (startup_lock) {
                                    g_dbg_hall_seq_reject++;
                                    seq_valid = false;
                                } else {
                                    g_sine_dir = -1;  // cofanie się
                                }
                            } else {
                                // Przeskok >1 sektora — podejrzany
                                g_dbg_hall_seq_reject++;
                                seq_valid = false;
                            }
                        }

                        // Stall detection timestamp — zawsze aktualizuj
                        g_sine_last_hall_ms = (uint32_t)(now_us / 1000);

                        // Running stats: Hall period min/max
                        if (dt_us < g_dbg_dt_min) g_dbg_dt_min = dt_us;
                        if (dt_us > g_dbg_dt_max) g_dbg_dt_max = dt_us;

                        if (seq_valid) {
                            // --- Aktualizacja prędkości per-sector ---
                            // Halle mogą być nierówno rozmieszczone (30-40%),
                            // globalna prędkość nie działa → zapamiętaj czas
                            // każdego sektora osobno.
                            if (dt_us >= min_hall_period_us && dt_us < 500000) {
                                uint32_t new_speed = 52428800UL / dt_us;
                                // old_idx = sektor WYCHODZĄCY (właśnie zmierzony)
                                if (old_idx >= 0 && old_idx < 6) {
                                    uint32_t prev = g_sine_sector_speed[old_idx];
                                    if (prev == 0) {
                                        g_sine_sector_speed[old_idx] = new_speed;
                                    } else {
                                        // Lekki filtr: 3/4 new + 1/4 old
                                        g_sine_sector_speed[old_idx] = (new_speed * 3 + prev) >> 2;
                                    }
                                }
                                // Prędkość interpolacji = prędkość sektora WCHODZĄCEGO
                                if (new_idx >= 0 && new_idx < 6 && g_sine_sector_speed[new_idx] != 0) {
                                    g_sine_speed_q16 = g_sine_sector_speed[new_idx];
                                } else {
                                    // Fallback: użyj właśnie zmierzonej
                                    g_sine_speed_q16 = new_speed;
                                }
                            }

                            // Korekcja kąta na przejściu Halla
                            // Snap point = środek sektora + offset.
                            int32_t expected_entry = (int32_t)new_idx * SINE_SECTOR_ENTRIES + SINE_SECTOR_CENTER + g_sine_hall_phase_offset;
                            if (expected_entry < 0) expected_entry += SINE_TABLE_SIZE;
                            if (expected_entry >= SINE_TABLE_SIZE) expected_entry -= SINE_TABLE_SIZE;
                            int32_t expected = expected_entry << 16;
                            int32_t current = (int32_t)g_sine_angle_q16;
                            int32_t err = expected - current;
                            if (err > (int32_t)(SINE_TABLE_Q16_FULL >> 1)) err -= (int32_t)SINE_TABLE_Q16_FULL;
                            if (err < -(int32_t)(SINE_TABLE_Q16_FULL >> 1)) err += (int32_t)SINE_TABLE_Q16_FULL;

                            int32_t abs_err = (err >= 0) ? err : -err;
                            g_dbg_last_hall_err = err;
                            g_dbg_angle_at_hall = current;  // kąt PRZED korekcją
                            g_dbg_expected_at_hall = expected;  // oczekiwany kąt wg tabeli

                            // Running stats: err min/max, speed min/max
                            if (err < g_dbg_err_min_q16) g_dbg_err_min_q16 = err;
                            if (err > g_dbg_err_max_q16) g_dbg_err_max_q16 = err;
                            { uint32_t spd = g_sine_speed_q16;
                              if (spd < g_dbg_speed_min) g_dbg_speed_min = spd;
                              if (spd > g_dbg_speed_max) g_dbg_speed_max = spd; }

                            if (g_sine_speed_q16 == 0) {
                                g_sine_angle_q16 = (uint32_t)expected;
                                g_dbg_snap_count++;
                                g_dbg_snap_window++;
                                is_snap = true;
                            } else if (abs_err > SINE_SNAP_THRESHOLD) {
                                g_sine_angle_q16 = (uint32_t)expected;
                                g_dbg_snap_count++;
                                g_dbg_snap_window++;
                                is_snap = true;
                            } else {
                                int32_t new_angle = current + (err >> SINE_PHASE_CORR_SHIFT);
                                if (new_angle < 0) new_angle += (int32_t)SINE_TABLE_Q16_FULL;
                                if (new_angle >= (int32_t)SINE_TABLE_Q16_FULL) new_angle -= (int32_t)SINE_TABLE_Q16_FULL;
                                g_sine_angle_q16 = (uint32_t)new_angle;
                                g_dbg_corr_count++;
                                g_dbg_corr_window++;

#if SINE_SPEED_CORR_ENABLE
                                // Korekcja prędkości PLL.
                                // err>0 = kąt za wolny (za mały/za duży wg kierunku)
                                // Dla dir:+1 (kąt rośnie): err>0 → zwiększ prędkość (+)
                                // Dla dir:-1 (kąt maleje): err<0 → kąt za wysoki = za wolny
                                //   → potrzeba WIĘKSZEJ prędkości (więcej odejmowania)
                                //   → ale err<0 dałby ujemną korektę → ODWRÓĆ znak!
                                int32_t err_s = err >> 8;
                                if (g_sine_dir < 0) err_s = -err_s;
                                int32_t spd_corr = (err_s * (int32_t)g_sine_speed_q16) >> 13;
                                int32_t new_spd = (int32_t)g_sine_speed_q16 + spd_corr;
                                if (new_spd < 0) new_spd = 0;
                                g_sine_speed_q16 = (uint32_t)new_spd;
#endif
                            }
                        }

                        g_sine_last_hall_idx = new_idx;

                        if (g_sine_startup_count < 255) {
                            g_sine_startup_count++;
                        }

                        // ── Push debug event: Hall change w trybie SINUS/FOC ──
                        if (g_debugCommutation) {
                            uint8_t wr = g_dbg_evt_wr;
                            uint8_t next = (wr + 1) & (DBG_EVT_RING_SIZE - 1);
                            if (next != g_dbg_evt_rd) {
                                volatile dbg_evt_t* ev = &g_dbg_evt_ring[wr];
                                ev->ts_us = now_us;
                                ev->hall_raw = hall;
                                ev->hall_filt = g_hall_prev_isr;
                                ev->mode = (uint8_t)g_mode_isr;
                                ev->sector_old = old_idx;
                                ev->sector_new = new_idx;
                                ev->dir = g_sine_dir;
                                ev->startup_cnt = g_sine_startup_count;
                                ev->angle_q16 = g_sine_angle_q16;
                                ev->speed_q16 = g_sine_speed_q16;
                                ev->hall_period_us = dt_us;
                                ev->duty = g_duty_isr;
                                ev->da = g_dbg_last_da;
                                ev->db = g_dbg_last_db;
                                ev->dc = g_dbg_last_dc;
                                ev->adc_a = g_phase_adc_raw_isr[0];
                                ev->adc_b = g_phase_adc_raw_isr[1];
                                ev->adc_c = g_phase_adc_raw_isr[2];
                                ev->foc_vd = g_foc_vd_i;
                                ev->foc_vq = g_foc_vq_i;
                                ev->flags = (is_snap ? 2 : 0)
                                          | (!seq_valid ? 4 : 0)
                                          | (g_reverse_isr ? 8 : 0);
                                g_dbg_evt_wr = next;
                            } else {
                                g_dbg_evt_overflow++;
                            }
                        }
                    }
                }
                // ── Push debug event: Hall change w trybie BLOCK ──
                else if (g_debugCommutation && g_mode_isr == DRIVE_MODE_BLOCK) {
                    uint8_t wr = g_dbg_evt_wr;
                    uint8_t next = (wr + 1) & (DBG_EVT_RING_SIZE - 1);
                    if (next != g_dbg_evt_rd) {
                        volatile dbg_evt_t* ev = &g_dbg_evt_ring[wr];
                        ev->ts_us = now_us;
                        ev->hall_raw = hall;
                        ev->hall_filt = g_hall_prev_isr;
                        ev->mode = (uint8_t)g_mode_isr;
                        ev->sector_old = -1;
                        ev->sector_new = -1;
                        ev->dir = g_reverse_isr ? -1 : 1;
                        ev->startup_cnt = 0;
                        ev->angle_q16 = 0;
                        ev->speed_q16 = 0;
                        ev->hall_period_us = dt_us;
                        ev->duty = g_duty_isr;
                        ev->da = g_dbg_last_da;
                        ev->db = g_dbg_last_db;
                        ev->dc = g_dbg_last_dc;
                        ev->adc_a = g_phase_adc_raw_isr[0];
                        ev->adc_b = g_phase_adc_raw_isr[1];
                        ev->adc_c = g_phase_adc_raw_isr[2];
                        ev->foc_vd = 0;
                        ev->foc_vq = 0;
                        ev->flags = g_reverse_isr ? 8 : 0;
                        g_dbg_evt_wr = next;
                    } else {
                        g_dbg_evt_overflow++;
                    }
                }
            }
        } // else: potwierdzanie w toku (normalny przebieg)
    }

    // ── Od tego momentu: używaj PRZEFILTROWANEGO Halla do komutacji ──
    // Surowy `hall` z GPIO.in może mieć glitche EMI (transient na 1 tick ISR).
    // g_hall_prev_isr = potwierdzony stan (3 zgodne odczyty + debounce 200µs).
    // Komutacja BLOCK/SINUS/FOC musi używać stabilnego stanu.
    g_dbg_hall_raw_isr = hall;  // diagnostyka: surowy vs filtrowany
    hall = g_hall_prev_isr;
    g_dbg_hall_filt_isr = hall;

    // Tryb testu MOSFET — ISR nie dotyka kanałów LEDC, tylko mierzy Hall/prędkość
    if (g_mosfet_test_active) {
        return;
    }

    if (g_brake_isr) {
        // Hamulec aktywny — regen braking jeśli włączony, inaczej coast
        if (g_regen_active_isr && g_regen_duty_isr > 0) {
            regenCommutateISR(hall, g_regen_duty_isr);
        } else {
            allMosfetsOff();
        }
        return;
    }

    if (!g_motor_enabled) {
        allMosfetsOff();
        return;
    }

    // SINUS/FOC safety fallback: PRZED guardem d==0, żeby prędkość
    // była zerowana nawet gdy duty=0 (motor coastuje po puszczeniu manetki).
    // Bez tego: spd zostaje zamrożone na starej wartości → przy ponownym
    // duty kąt startuje z błędną prędkością → snap → szarpnięcie.
    if (g_mode_isr == DRIVE_MODE_SINUS || g_mode_isr == DRIVE_MODE_FOC) {
        uint32_t now_ms = (uint32_t)esp_timer_get_time() / 1000;  // identycznie jak w Hall ISR
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
            // Stall → powrót do BLOCK startup (reaktywna komutacja z Halli)
            // Bez tego: prędkość=0 + startup_count>=6 → sinusCommutateISR z crawl
            // → pole leci dookoła → silnik się "miota".
            g_sine_startup_count = 0;
            g_sine_last_hall_ms = now_ms;
        }
    }

    uint16_t d = g_duty_isr;

    // ── Explicit freewheel: duty == 0 → wolnobieg ──
    // BLOCK: natychmiast allMosfetsOff (bo d=0 + LS ON = hamowanie EM).
    // SINUS/FOC: kontynuuj do sinusCommutateISR/focCommutateISR —
    // angle advance musi działać nawet przy d=0, inaczej po ponownym
    // włączeniu duty kąt jest stary → duży err → snap → szarpnięcie.
    // sinusCommutateISR/focCommutateISR same wyłączą MOSFETy (amplitude < MIN).
    if (d == 0 && g_mode_isr != DRIVE_MODE_SINUS && g_mode_isr != DRIVE_MODE_FOC) {
        allMosfetsOff();
        return;
    }

    // Dispatch trybu sterowania
    switch (g_mode_isr) {
        case DRIVE_MODE_SINUS:
        case DRIVE_MODE_FOC:
            // ── Max Torque Startup: snap kąt do optymalnej pozycji Hall ──
            // Gdy prędkość < threshold: rotor stoi lub oscyluje.
            // Snap kąt do hall_sector_center + skalibrowany offset (sat) →
            // pole statorowe prostopadłe do rotora (max Iq, max moment).
            // SINUS/FOC SVPWM generuje gładkie sinusoidalne prądy pod tym kątem.
            // Nie używamy BLOCK — SINUS daje precyzyjniejszy wektor momentu.
            if (g_sine_speed_q16 < SINE_BLOCK_SPEED_THRESHOLD) {
                int8_t sec = hallToSector(hall);
                if (sec >= 0) {
                    int32_t entry = (int32_t)sec * SINE_SECTOR_ENTRIES + SINE_SECTOR_CENTER + g_sine_hall_phase_offset;
                    if (entry < 0) entry += SINE_TABLE_SIZE;
                    if (entry >= SINE_TABLE_SIZE) entry -= SINE_TABLE_SIZE;
                    g_sine_angle_q16 = (uint32_t)entry << 16;
                }
            }
            if (g_mode_isr == DRIVE_MODE_SINUS) {
                g_dbg_commut_path = 2;  // SINUS
                sinusCommutateISR(hall, d);
            } else {
                g_dbg_commut_path = 3;  // FOC
                focCommutateISR(hall, d);
            }
            return;
        case DRIVE_MODE_BLOCK:
            g_dbg_commut_path = 1;  // BLOCK
            break;  // kontynuuj do komutacji blokowej poniżej
        default:
            // Tryb DISABLED → bezpieczny stan
            allMosfetsOff();
            return;
    }

    // Odwrócenie kierunku: remap Hall przed tabelą komutacji blokowej
    uint8_t bh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;

    // Block crossfade: płynne przejście duty między starą a nową fazą PWM
    if (g_block_cross_cnt > 0 && g_block_old_bh >= 1 && g_block_old_bh <= 6 && bh >= 1 && bh <= 6) {
        uint8_t remaining = g_block_cross_cnt;
        uint8_t elapsed = BLOCK_CROSS_TICKS - remaining;
        g_block_cross_cnt--;

        const uint8_t *os = g_block_phase_tbl[g_block_old_bh];
        const uint8_t *ns = g_block_phase_tbl[bh];

        // Faza A
        uint8_t sa = ns[0]; uint16_t da = d;
        if (os[0] == 2 && ns[0] == 0) { sa = 2; da = (uint16_t)((uint32_t)d * remaining / BLOCK_CROSS_TICKS); }
        else if (os[0] == 0 && ns[0] == 2) { da = (uint16_t)((uint32_t)d * (elapsed + 1) / BLOCK_CROSS_TICKS); }
        if (sa == 2) phaseA_PWM(da); else if (sa == 1) phaseA_Low(); else phaseA_Off();

        // Faza B
        uint8_t sb = ns[1]; uint16_t db = d;
        if (os[1] == 2 && ns[1] == 0) { sb = 2; db = (uint16_t)((uint32_t)d * remaining / BLOCK_CROSS_TICKS); }
        else if (os[1] == 0 && ns[1] == 2) { db = (uint16_t)((uint32_t)d * (elapsed + 1) / BLOCK_CROSS_TICKS); }
        if (sb == 2) phaseB_PWM(db); else if (sb == 1) phaseB_Low(); else phaseB_Off();

        // Faza C
        uint8_t sc = ns[2]; uint16_t dc = d;
        if (os[2] == 2 && ns[2] == 0) { sc = 2; dc = (uint16_t)((uint32_t)d * remaining / BLOCK_CROSS_TICKS); }
        else if (os[2] == 0 && ns[2] == 2) { dc = (uint16_t)((uint32_t)d * (elapsed + 1) / BLOCK_CROSS_TICKS); }
        if (sc == 2) phaseC_PWM(dc); else if (sc == 1) phaseC_Low(); else phaseC_Off();
        // Debug: aktualizuj dA/dB/dC dla cdbg (crossfade)
        g_dbg_last_da = (sa == 2) ? da : 0;
        g_dbg_last_db = (sb == 2) ? db : 0;
        g_dbg_last_dc = (sc == 2) ? dc : 0;
    } else {
        // Normalny block (bez crossfade)
        switch (bh) {
            case 1: phaseA_PWM(d); phaseB_Low(); phaseC_Off(); break;
            case 3: phaseA_PWM(d); phaseB_Off(); phaseC_Low(); break;
            case 2: phaseA_Off(); phaseB_PWM(d); phaseC_Low(); break;
            case 6: phaseA_Low(); phaseB_PWM(d); phaseC_Off(); break;
            case 4: phaseA_Low(); phaseB_Off(); phaseC_PWM(d); break;
            case 5: phaseA_Off(); phaseB_Low(); phaseC_PWM(d); break;
            default: allMosfetsOff(); break;
        }
        // Debug: aktualizuj dA/dB/dC dla cdbg
        switch (bh) {
            case 1: g_dbg_last_da = d; g_dbg_last_db = 0; g_dbg_last_dc = 0; break;
            case 3: g_dbg_last_da = d; g_dbg_last_db = 0; g_dbg_last_dc = 0; break;
            case 2: g_dbg_last_da = 0; g_dbg_last_db = d; g_dbg_last_dc = 0; break;
            case 6: g_dbg_last_da = 0; g_dbg_last_db = d; g_dbg_last_dc = 0; break;
            case 4: g_dbg_last_da = 0; g_dbg_last_db = 0; g_dbg_last_dc = d; break;
            case 5: g_dbg_last_da = 0; g_dbg_last_db = 0; g_dbg_last_dc = d; break;
            default: g_dbg_last_da = g_dbg_last_db = g_dbg_last_dc = 0; break;
        }
    }
}

// ============================================================================
// Inicjalizacja timera komutacji
// ============================================================================

/**
 * @brief Konfiguruje i uruchamia ISR komutacji na przerwaniu MCPWM TEZ.
 *
 * Konfiguracja:
 * - MCPWM_UNIT_0, Timer 0 TEZ (Timer Equals Zero) interrupt
 * - Częstotliwość ISR = 20 kHz (= częstotliwość PWM, center-aligned)
 * - ISR wywoływana w dolinie PWM (counter=0) — wszystkie low-side ON
 *   → optymalny moment na odczyt ADC prądu (INA180A2 mierzy gdy LS przewodzi)
 *
 * @note allMosfetsOff() musi być wywołana PRZED initCommutationTimer().
 */
void initCommutationTimer() {
    // Rejestruj globalny ISR MCPWM
    mcpwm_isr_register(MCPWM_UNIT_0, onCommutationTimer, NULL,
                        ESP_INTR_FLAG_IRAM,
                        &g_mcpwm_isr_handle);
    // Włącz przerwanie TEZ dla Timer 0 (= dolina center-aligned PWM)
    mcpwm_ll_intr_enable_timer_tez(&MCPWM0, 0, true);
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
 * 1. Advance angle: angle += speed_q16
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
 *   duty = PWM_MAX_DUTY/2 + (sine_val × amplitude) >> 10
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

    // ── 1. Stall freeze: brak przejścia Halla > 200ms → nie avansuj kąta ──
    // Gdy speed==0 → nie avansuj (dispatch już użył BLOCK, tu nie powinniśmy trafić).
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    uint32_t real_speed = g_sine_speed_q16;
    bool stalled = (real_speed < SINE_BLOCK_SPEED_THRESHOLD) || (since_hall > SINE_STALL_FREEZE_MS);

    // ── 2. Advance angle ──
    // Kierunek avansowania kąta: z detekcji Halli (g_sine_dir), nie config.
    if (!stalled) {
        uint32_t spd = real_speed;
        bool dir_reverse = (g_sine_dir < 0);
        if (dir_reverse) {
            if (g_sine_angle_q16 >= spd) {
                g_sine_angle_q16 -= spd;
            } else {
                g_sine_angle_q16 = g_sine_angle_q16 + SINE_TABLE_Q16_FULL - spd;
            }
        } else {
            g_sine_angle_q16 += spd;
            if (g_sine_angle_q16 >= SINE_TABLE_Q16_FULL) {
                g_sine_angle_q16 -= SINE_TABLE_Q16_FULL;
            }
        }
    }

    // ── Amplitude guard (PO angle advance!) ──
    if (amplitude < SINE_MIN_AMPLITUDE) {
        allMosfetsOff();
        return;
    }

    // ── 3. Oblicz 3 kąty fazowe z offsetami ──
    //   Phase A = sin(θ)           — faza referencyjna
    //   Phase B = sin(θ + 240°)    — offset 64 wpisów
    //   Phase C = sin(θ + 120°)    — offset 32 wpisów
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

    // amplitude: 0..PWM_MAX_DUTY, z ograniczeniem bezpieczeństwa
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
    // Zamiast stałej bazy PWM_MAX_DUTY/2, przesuwamy wszystkie 3 modulacje razem tak,
    // żeby mieściły się w zakresie 0..PWM_MAX_DUTY bez klipowania.
    // Offset = PWM_MAX_DUTY/2 - (max+min)/2 to zero-sequence (common-mode) składowa,
    // która nie wpływa na napięcie linia-linia, ale zwiększa zakres liniowy
    // o 15.5% (z Vbus*√3/2 do Vbus) — identycznie jak SVPWM.
    //
    // Bez SVPWM: amp > PWM_MAX_DUTY/2 → klipowanie → brak wzrostu napięcia fundamentalnego
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
        int32_t offset = (PWM_MAX_DUTY / 2) - ((mx + mn) >> 1);

        int32_t da = offset + ma;
        if (da < 0) da = 0;
        if (da > (int32_t)PWM_MAX_DUTY) da = (int32_t)PWM_MAX_DUTY;

        int32_t db = offset + mb;
        if (db < 0) db = 0;
        if (db > (int32_t)PWM_MAX_DUTY) db = (int32_t)PWM_MAX_DUTY;

        int32_t dc = offset + mc;
        if (dc < 0) dc = 0;
        if (dc > (int32_t)PWM_MAX_DUTY) dc = (int32_t)PWM_MAX_DUTY;

        g_dbg_last_da = (int16_t)da;
        g_dbg_last_db = (int16_t)db;
        g_dbg_last_dc = (int16_t)dc;

        mcpwm_phase_complementary(0, (uint32_t)da);
        mcpwm_phase_complementary(1, (uint32_t)db);
        mcpwm_phase_complementary(2, (uint32_t)dc);
    }
}

// ============================================================================
// Komutacja FOC — ISR (inverse Park + inverse Clarke + SVPWM)
// ============================================================================

/**
 * @brief Komutacja FOC — ISR generuje SVPWM z Vd/Vq przygotowanych w loop().
 *
 * ## KRYTYCZNE: INTEGER-ONLY MATH
 * Timer ISR na ESP32 (level 3 interrupt) NIE zapisuje kontekst FPU.
 * Użycie float w ISR korumpuje rejestry koprocesora → crash (LoadProhibited).
 * Cała arytmetyka używa int32_t, identycznie jak sinusCommutateISR.
 *
 * ## Algorytm ISR (arytmetyka identyczna z sinusCommutateISR)
 * 1. Advance angle (Hall tracking + interpolacja Q16)
 * 2. Inverse Park: Vα = (Vd·cos - Vq·sin) >> 10, Vβ = (Vd·sin + Vq·cos) >> 10
 * 3. Inverse Clarke: Va=Vα, Vb=(-Vα+√3·Vβ)/2, Vc=(-Vα-√3·Vβ)/2
 *    (√3 ≈ 1774/1024 w Q10)
 * 4. SVPWM min-max centering
 * 5. MCPWM set duty
 *
 * @param hall      Aktualny stan Halla [C:B:A] 1-6
 * @param amplitude Duty z przepustnicy/rampy — używane tylko jako guard (>SINE_MIN_AMPLITUDE)
 */
static void IRAM_ATTR focCommutateISR(uint8_t hall, uint16_t amplitude) {
    // Walidacja Halla
    if (hall == 0 || hall == 7) {
        allMosfetsOff();
        return;
    }

    // ── 1. Stall freeze (identycznie jak SINUS) ──
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    uint32_t real_speed = g_sine_speed_q16;
    bool stalled = (real_speed < SINE_BLOCK_SPEED_THRESHOLD) || (since_hall > SINE_STALL_FREEZE_MS);

    // ── 2. Advance angle (identycznie jak SINUS) ──
    if (!stalled) {
        uint32_t spd = real_speed;
        bool dir_reverse = (g_sine_dir < 0);
        if (dir_reverse) {
            if (g_sine_angle_q16 >= spd) {
                g_sine_angle_q16 -= spd;
            } else {
                g_sine_angle_q16 = g_sine_angle_q16 + SINE_TABLE_Q16_FULL - spd;
            }
        } else {
            g_sine_angle_q16 += spd;
            if (g_sine_angle_q16 >= SINE_TABLE_Q16_FULL) {
                g_sine_angle_q16 -= SINE_TABLE_Q16_FULL;
            }
        }
    }

    // ── Amplitude guard (PO angle advance!) ──
    if (amplitude < SINE_MIN_AMPLITUDE) {
        allMosfetsOff();
        return;
    }

    // ── 3. Read Vd, Vq from loop() PI controller (int32_t, duty units) ──
    int32_t vd_i = g_foc_vd_i;
    int32_t vq_i = g_foc_vq_i;

    // ── 4. Inverse Park transform (INTEGER-ONLY) ──
    // sin_val, cos_val: -1024..+1024 (Q10)
    //
    // UWAGA: znaki dopasowane do konwencji kąta sinusCommutateISR.
    // Standardowy inverse Park to: Vα = Vd·cos - Vq·sin, Vβ = Vd·sin + Vq·cos
    // ale z naszą tabelą sinusa i mapowaniem faz A=0°, B=240°, C=120°
    // to daje wektor obracający się w przeciwnym kierunku niż SINUS.
    // Weryfikacja numeryczna przy θ=0°:
    //   SINUS: A=512 B=468 C=556
    //   Poprawiony FOC: A=512 B=468 C=556 ✔
    //
    // Vα = (Vd·cos + Vq·sin) >> 10
    // Vβ = (Vd·sin - Vq·cos) >> 10
    uint32_t angle = g_sine_angle_q16;
    // FOC: surowy θ (bez offsetu 180°!) — musi być spójny z forward Park w loop().
    int32_t sin_val = sine_interp_q16(angle);  // -1024..+1024
    uint32_t cos_angle = angle + (24UL << 16); // +90° (24/96 entries = 90°)
    if (cos_angle >= SINE_TABLE_Q16_FULL) cos_angle -= SINE_TABLE_Q16_FULL;
    int32_t cos_val = sine_interp_q16(cos_angle);

    int32_t v_alpha = (vd_i * cos_val + vq_i * sin_val) >> 10;
    int32_t v_beta  = (vd_i * sin_val - vq_i * cos_val) >> 10;

    // ── 5. Inverse Clarke → 3 napięcia fazowe (INTEGER-ONLY) ──
    // Va = Vα
    // Vb = (-Vα + √3·Vβ) / 2    [√3 ≈ 1774/1024 w Q10]
    // Vc = (-Vα - √3·Vβ) / 2
    int32_t va = v_alpha;
    int32_t sqrt3_vbeta = (1774 * v_beta) >> 10;  // √3·Vβ
    int32_t vb = (-v_alpha + sqrt3_vbeta) >> 1;
    int32_t vc = (-v_alpha - sqrt3_vbeta) >> 1;

    // ── 6. SVPWM min-max centering (identycznie jak SINUS) ──
    {
        int32_t mn = va;
        if (vb < mn) mn = vb;
        if (vc < mn) mn = vc;
        int32_t mx = va;
        if (vb > mx) mx = vb;
        if (vc > mx) mx = vc;
        int32_t offset = (PWM_MAX_DUTY / 2) - ((mx + mn) >> 1);

        int32_t da = offset + va;
        if (da < 0) da = 0;
        if (da > (int32_t)PWM_MAX_DUTY) da = (int32_t)PWM_MAX_DUTY;

        int32_t db = offset + vb;
        if (db < 0) db = 0;
        if (db > (int32_t)PWM_MAX_DUTY) db = (int32_t)PWM_MAX_DUTY;

        int32_t dc_duty = offset + vc;
        if (dc_duty < 0) dc_duty = 0;
        if (dc_duty > (int32_t)PWM_MAX_DUTY) dc_duty = (int32_t)PWM_MAX_DUTY;

        g_dbg_last_da = (int16_t)da;
        g_dbg_last_db = (int16_t)db;
        g_dbg_last_dc = (int16_t)dc_duty;

        mcpwm_phase_complementary(0, (uint32_t)da);
        mcpwm_phase_complementary(1, (uint32_t)db);
        mcpwm_phase_complementary(2, (uint32_t)dc_duty);
    }
}

// ============================================================================
// Komutacja blokowa (trapezoidalna / 6-step)
// ============================================================================
//
// Tabela komutacji (Hall [C:B:A]):
//
//   Hall | Faza A      | Faza B      | Faza C
//   -----+-------------+-------------+------------
//    1   | PWM (high)  | LOW (low-on)| OFF (float)
//    3   | PWM (high)  | OFF (float) | LOW (low-on)
//    2   | OFF (float) | PWM (high)  | LOW (low-on)
//    6   | LOW (low-on)| PWM (high)  | OFF (float)
//    4   | LOW (low-on)| OFF (float) | PWM (high)
//    5   | OFF (float) | LOW (low-on)| PWM (high)

// Pomocnicze inline do ustawiania stanu fazy — wrapper na MCPWM helpers
/**
 * @brief Faza A: wyjście PWM (high-side ON z modulacją, low-side OFF).
 * @param duty Wypełnienie PWM 0–PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseA_PWM(uint16_t duty) {
    mcpwm_phase_pwm(0, duty);
}
/** @brief Faza A: podłączona do GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseA_Low() {
    mcpwm_phase_gnd(0);
}
/** @brief Faza A: pływająca (oba tranzystory OFF). */
static inline void IRAM_ATTR phaseA_Off() {
    mcpwm_phase_off(0);
}

/** @brief Faza B: PWM (high-side ON, low-side OFF). @param duty 0–PWM_MAX_DUTY. */
static inline void IRAM_ATTR phaseB_PWM(uint16_t duty) {
    mcpwm_phase_pwm(1, duty);
}
/** @brief Faza B: GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseB_Low() {
    mcpwm_phase_gnd(1);
}
/** @brief Faza B: pływająca (oba OFF). */
static inline void IRAM_ATTR phaseB_Off() {
    mcpwm_phase_off(1);
}

/** @brief Faza C: PWM (high-side ON, low-side OFF). @param duty 0–PWM_MAX_DUTY. */
static inline void IRAM_ATTR phaseC_PWM(uint16_t duty) {
    mcpwm_phase_pwm(2, duty);
}
/** @brief Faza C: GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseC_Low() {
    mcpwm_phase_gnd(2);
}
/** @brief Faza C: pływająca (oba OFF). */
static inline void IRAM_ATTR phaseC_Off() {
    mcpwm_phase_off(2);
}

// ============================================================================
// Funkcje pomocnicze regen — Low-side PWM (HS OFF, LS modulowany)
// ============================================================================

/**
 * @brief Faza A: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY (0=brak zwarcia, MAX=pełne zwarcie).
 * MCPWM: gen_A forced LOW (HS OFF), gen_B = active-low PWM (LIN=LOW → LS ON).
 */
static inline void IRAM_ATTR phaseA_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(0, duty);
}

/**
 * @brief Faza B: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseB_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(1, duty);
}

/**
 * @brief Faza C: regen PWM (high-side OFF, low-side PWM).
 * @param duty Siła hamowania 0–PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseC_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(2, duty);
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
 *
 * @warning Wszystkie high-side FETy MUSZĄ być OFF! Shoot-through = uszkodzenie.
 * @warning Nigdy duty 100% — brak fazy OFF = brak transferu do baterii (tylko ciepło).
 */
static void IRAM_ATTR regenCommutateISR(uint8_t hall, uint16_t regen_duty) {
    // Odwrócenie kierunku: remap Hall
    uint8_t rh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;

    switch (rh) {
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
}

/**
 * @brief Komutacja blokowa 6-step (backup path — wywoływana z loop()).
 *
 * Ustawia stany faz A/B/C na podstawie stanu Halla.
 * Ta funkcja NIE jest używana w normalnej pracy (ISR obsługuje komutację).
 * Zachowana jako fallback / do debugowania bez timera.
 *
 * @param hallState 3-bitowy stan Halla [C:B:A], wartości 1–6
 * @param duty      Wypełnienie PWM 0–PWM_MAX_DUTY
 *
 * @note Jeśli hallState == 0 lub 7 (błąd czujników) → allMosfetsOff() + fault=true.
 * @note Używa pomocniczych funkcji phaseX_PWM/Low/Off() (MCPWM wrappers).
 */
void blockCommutate(uint8_t hallState, uint16_t duty) {
    // Walidacja stanu Halla
    if (hallState == HALL_STATE_INVALID_0 || hallState == HALL_STATE_INVALID_7) {
        allMosfetsOff();
        g_bldc_state.fault = true;
        return;
    }

    if (g_bldc_state.mode == DRIVE_MODE_DISABLED) return;  // safety

    // Odwrócenie kierunku: remap Hall
    uint8_t h = g_reverse_isr ? g_hall_reverse_map[hallState] : hallState;

    // Komutacja
    switch (h) {
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
}

// ============================================================================
// Obsługa komend Serial
// ============================================================================

/** @brief Wypisuje tabelę dostępnych komend Serial na konsole. */
void printHelp() {
    Serial.println();
    Serial.println("==================== KOMENDY (wszystkie wymagaja Enter) ====================");
    Serial.println();
    Serial.println("---------- Tryby sterowania silnika ----------");
    Serial.println("e          Wlacz tryb BLOCK (6-krokowa komutacja trapezowa)");
    Serial.println("             Prosty, niezawodny. Sygnaly Hall -> 6 stanow przelaczania.");
    Serial.println("             Szum slyszalny, ale niskie wymagania obliczeniowe.");
    Serial.println("S / m2     Wlacz tryb SINUS (plynna komutacja sinusoidalna)");
    Serial.println("             Tablica sinusow 96 wpisow, interpolacja miedzy Hallami.");
    Serial.println("             Cichszy niz BLOCK, plynniejszy moment obrotowy.");
    Serial.println("F / m3     Wlacz tryb FOC (Field Oriented Control - sterowanie wektorowe)");
    Serial.println("             Transformacja Clarke+Park, regulator PI osi d/q.");
    Serial.println("             Feedforward + korekcja pradowa. Najlepsza sprawnosc.");
    Serial.println("d          Wylacz silnik (duty=0, wszystkie MOSFET OFF = wolnobieg)");
    Serial.println();
    Serial.println("---------- Sterowanie moca (duty cycle) ----------");
    Serial.println("+          Zwieksz duty o 5% (wzgledem PWM_MAX_DUTY)");
    Serial.println("-          Zmniejsz duty o 5%");
    Serial.println("0-100      Ustaw duty bezposrednio [%]. Np. '50' = 50% mocy.");
    Serial.println("             0=wylaczony, 100=pelna moc. Dziala w trybie manual.");
    Serial.println("man        Manual duty ON/OFF. Gdy ON: manetka (przepustnica) ignorowana,");
    Serial.println("             duty sterowane tylko komendami +/-/0-100 z konsoli.");
    Serial.println("R          Regeneracja ON/OFF (hamowanie rekuperacyjne)");
    Serial.println("             Gdy ON: przetwornica zwraca energie do baterii przy hamowaniu.");
    Serial.println("             Wymaga predkosci > 50 RPM, Vbat < 42V (ochrona LiPo).");
    Serial.println("rev        Przelacz kierunek obrotow CW/CCW (zapisuje do NVS).");
    Serial.println("             Softwarowa zamiana faz — dziala we wszystkich trybach.");
    Serial.println("b          Symulacja hamulca ON/OFF (natychmiastowe duty=0, nadrzedny)");
    Serial.println();
    Serial.println("---------- Status i diagnostyka ----------");
    Serial.println("s          Pokaz pelny status: tryb, duty, predkosc, napiecie, prady,");
    Serial.println("             PAS, assist level, throttle RAW, temperatura (jesli dostepna).");
    Serial.println("a          Auto-status co 1s ON/OFF (cykliczne wypisywanie statusu)");
    Serial.println("P          Pokaz parametry wyswietlacza S866 P01-P20:");
    Serial.println("             P05=poziomy assist, P06=rozmiar kola, P07=magnesy/polpary,");
    Serial.println("             P08=limit predkosci [km/h], P10=tryb jazdy, P13=magnesy PAS.");
    Serial.println("gdbg       Debug SINUS/BLOCK ON/OFF (co 200ms: hall idx, predkosc, duty)");
    Serial.println("idbg       Debug pradow fazowych ON/OFF (co 500ms: Ia,Ib,Ic, max, EMA, limit, factor)");
    Serial.println("hdbg       Debug Hall ON/OFF (co 200ms: raw/filtered hall, sector, speed, path, edges)");
    Serial.println("cdbg       Debug Commutation ON/OFF (event-driven: Hall change + duty/angle/speed)");
    Serial.println();
    Serial.println("---------- Offset fazy sinusa (SINUS/FOC) ----------");
    Serial.println("so         Pokaz aktualny offset fazy [-48..+48], 1 wpis = 3.75 deg el.");
    Serial.println("so+        Offset +2 wpisy (+7.5 deg) - wiecej mocy w jednym kierunku");
    Serial.println("so-        Offset -2 wpisy (-7.5 deg)");
    Serial.println("so:N       Ustaw offset na N (zakres -48..+48). Dostraja fazowanie");
    Serial.println("             miedzy Hallami a uzwojeniami. Zly offset = slaba moc/szarpanie.");
    Serial.println("sat        Auto-tune offsetu fazy: sweep -24..+24 krok 2, mierzy prad.");
    Serial.println("             Najlepszy offset = najwyzszy prad przy stalym duty (~25s).");
    Serial.println("sat:M:N    Auto-tune zakres M..N (np. sat:-16:16)");
    Serial.println("sat:M:N:S  Auto-tune zakres M..N krok S (np. sat:-8:8:1)");
    Serial.println();
    Serial.println("---------- FOC (Field Oriented Control) ----------");
    Serial.println("  Regulator PI (Proportional-Integral) steruje pradem w osiach d/q.");
    Serial.println("  Iq (os q) = moment obrotowy, Id (os d) = 0 (MTPA).");
    Serial.println("  Wyjscie PI = feedforward (Vq=duty) + korekcja PI (+/-100 PWM max).");
    Serial.println();
    Serial.println("  Kp (proporcjonalny): natychmiastowa reakcja na blad pradu [PWM/A].");
    Serial.println("    Za niskie Kp = wolna reakcja, blad ustalonego stanu.");
    Serial.println("    Za wysokie Kp = oscylacje, niestabilnosc. Domyslnie: 0.5");
    Serial.println("  Ki (calkujacy): eliminuje blad ustalony, kumuluje blad w czasie [PWM/(A*s)].");
    Serial.println("    Za niskie Ki = wolne dochodzenie do celu, pozostaly blad.");
    Serial.println("    Za wysokie Ki = przeregulowanie, oscylacje. Domyslnie: 5.0");
    Serial.println("    Anti-windup: calka ograniczona do +/-pi_limit.");
    Serial.println();
    Serial.println("foc        Pokaz status FOC: Vd/Vq [PWM], Id/Iq [A], Kp/Ki, integral");
    Serial.println("fdbg       Debug FOC ON/OFF (co 200ms: prady, napiecia, blad PI)");
    Serial.println("fkp:N.N    Ustaw Kp obu osi d i q (zakres 0.0-100.0). Np. fkp:0.8");
    Serial.println("fki:N.N    Ustaw Ki obu osi d i q (zakres 0.0-1000.0). Np. fki:10.0");
    Serial.println("fkpd:N.N   Ustaw Kp TYLKO osi d (zakres 0.0-100.0)");
    Serial.println("fkid:N.N   Ustaw Ki TYLKO osi d (zakres 0.0-1000.0)");
    Serial.println("fvolt      FOC voltage mode ON/OFF: Vq=duty wprost, bez regulatora PI.");
    Serial.println("             Do diagnostyki: porownanie SINUS vs FOC bez wplywu PI.");
    Serial.println("fpitune    Auto-tuning PI metoda Astrom-Hagglund (relay feedback).");
    Serial.println("             Wymaga: tryb FOC, silnik pracuje (duty>0). Czas: ~5s.");
    Serial.println("             Przejsciowo zastepuje PI sterowaniem ON/OFF (relay),");
    Serial.println("             mierzy oscylacje pradu Iq, oblicza optymalne Kp/Ki.");
    Serial.println();
    Serial.println("---------- Test MOSFET (diagnostyka) ----------");
    Serial.println("t          Pokaz pomoc trybu testowego");
    Serial.println("tAH/tBH/tCH  Wlacz tranzystor HIGH-side fazy A/B/C (PWM testowe)");
    Serial.println("tAL/tBL/tCL  Wlacz tranzystor LOW-side fazy A/B/C");
    Serial.println("tp:N       Ustaw duty testowe na N% (zakres 1-50, bezpieczenstwo)");
    Serial.println("t0         Wylacz test MOSFET (wszystkie tranzystory OFF)");
    Serial.println();
    Serial.println("---------- Konfiguracja NVS (EEPROM, zachowana po restarcie) ----------");
    Serial.println("cfg        Pokaz aktualna konfiguracje NVS + wartosc runtime");
    Serial.println("cfg:mode:N Tryb po restarcie: 0=wylaczony, 1=BLOCK, 2=SINUS, 3=FOC");
    Serial.println("cfg:ramp:N Czas rampy duty 0->100% [ms], zakres 0-10000. 0=natychmiastowy.");
    Serial.println("             Rampa dziala w OBU kierunkach (przyspieszanie i zwalnianie).");
    Serial.println("cfg:regen:N  Regeneracja po restarcie: 0=wyl, 1=wl");
    Serial.println("cfg:step:N Max zmiana duty na krok petli glownej [%], zakres 1-100.");
    Serial.println("             0=bez limitu. Domyslnie 5%. Ogranicza szarpniecia mocy.");
    Serial.println("             Dziala razem z rampa (bardziej restrykcyjny wygrywa).");
    Serial.println("cfg:rev:N  Kierunek obrotow po restarcie: 0=CW (normalny), 1=CCW (odwrocony)");
    Serial.println("pwmfreq    Pokaz aktualna czestotliwosc PWM [Hz]");
    Serial.println("pwmfreq:N  Ustaw czestotliwosc PWM: 8000-32000 Hz (domyslnie 20000)");
    Serial.println("cfg:defaults  Zaladuj wartosci domyslne do runtime (bez zapisu EEPROM)");
    Serial.println("cfg:save      Zapisz aktualne wartosci runtime do EEPROM");
    Serial.println("cfg:reload    Wczytaj wartosci z EEPROM i zastosuj do runtime");
    Serial.println();
    Serial.println("---------- PAS (Pedal Assist Sensor) ----------");
    Serial.println("  Czujnik na GPIO22, dysk magnesow na wale korbowym.");
    Serial.println("  Detekcja kierunku: asymetria HIGH/LOW (histereza +/-5 impulsow).");
    Serial.println("  V_target z poziomu assist: 6 + Lx*(Vmax-6)/15 km/h, wygladzony.");
    Serial.println("  Freewheel: gdy Vkola >= Vtarget, duty=0 (motor nie przeszkadza).");
    Serial.println();
    Serial.println("---------- Diagnostyka predkosci (SPEED) ----------");
    Serial.println("spddbg       Debug SPEED: okres, impulsy, odrzucone, mediana, pin state.");
    Serial.println("spddbg0      Zerowanie licznikow SPEED (pulse/reject count).");
    Serial.println("spdcal       Kalibracja SPEED: obroc kolo 3x w 15s -> oblicza pulses/rev.");
    Serial.println("               Ponowne wpisanie spdcal anuluje trwajaca kalibracje.");
    Serial.println("spdppr:N     Reczne ustawienie impulsow SPEED na obrot (1-20, domyslnie 1).");
    Serial.println();
    Serial.println("pasdir       Przelacz kierunek PAS normal/odwrocony (zapisuje do NVS).");
    Serial.println("               Jesli silnik krecisie wstecz przy pedalowaniu, uzyj tego.");
    Serial.println("passtart:N   Opoznienie startu [ms] (zakres 0-10000, domyslnie 2000).");
    Serial.println("               Czas ciaglego pedalowania do przodu wymagany do aktywacji.");
    Serial.println("               Zapobiega przypadkowemu wlaczeniu silnika.");
    Serial.println("passtop:N    Timeout stopu [ms] (zakres 100-10000, domyslnie 1000).");
    Serial.println("               Czas bez impulsow PAS po ktorym silnik wylacza wspomaganie.");
    Serial.println("               Nizsza wartosc = szybsza reakcja na stop pedalowania.");
    Serial.println("pasramp:N    Soft-start [ms] (zakres 0-10000, domyslnie 1500).");
    Serial.println("               Czas narastania mocy 0->100% po aktywacji PAS.");
    Serial.println("               0=natychmiastowy start, 1500=lagodne narastanie ~1.5s.");
    Serial.println("pasdbg       Debug PAS ON/OFF: kierunek, kadencja RPM, V_target,");
    Serial.println("               V_smooth, speed_MA, duty, soft-start, stan automatu.");
    Serial.println("pashalf:N    Min polokres PAS [ms], zakres 1-200 (domyslnie 5).");
    Serial.println("               Impulsy krotsze odrzucane jako szum.");
    Serial.println("pasasym:N    Prog asymetrii kierunku [%], zakres 1-50 (domyslnie 5).");
    Serial.println("               Wieksza = mniej czuly detektor fwd/rev.");
    Serial.println("passlew:N    Slew rate PAS duty, zakres 1-100 (domyslnie 30).");
    Serial.println("               Max zmiana duty na iteracje loop. Mniejszy = lagodniej.");
    Serial.println("pashold:N    Forward holdoff [ms], zakres 50-2000 (domyslnie 300).");
    Serial.println("               Czas pedalowania fwd wymagany do uznania kierunku.");
    Serial.println("pasat        PAS auto-tune: mierzy sygnal 10s, ustawia optymalne");
    Serial.println("               debounce, min polokres i asymetrie. Pedaluj rownomiernie!");
    Serial.println();
    Serial.println("---------- Limit pradowy ----------");
    Serial.println("ilim:N       Limit pradu fazowego [A], zakres 0-50. 0=brak limitu.");
    Serial.println("               Zapisuje do NVS. P14 z display nadpisuje (jesli>0).");
    Serial.println("               3 warstwy ochrony: soft P-regulator (duty reduction),");
    Serial.println("               FOC Iq clamp, hard cutoff przy 150% limitu.");
    Serial.println();
    Serial.println("h          Pokaz te pomoc");
    Serial.println("==========================================================================");
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
    Serial.printf("Assist level:  %d (raw: %d)\n", g_display.rx.assist_level / 3, g_display.rx.assist_level);
    Serial.println("--- Parametry lokalne wyświetlacza (nie w ramce) ---");
    Serial.printf("P01  Jasność podświetlenia:   [local]\n");
    Serial.printf("P02  Jednostki prędkości:     [local]\n");
    Serial.printf("P03  Napięcie systemu:        [local]\n");
    Serial.printf("P04  Auto-wyłączenie:         [local]\n");
    Serial.printf("P05  Poziomy wspomagania:     [local]\n");
    Serial.println("--- Parametry z ramki RX ---");
    Serial.printf("P06* Rozmiar koła:            %d.%d\"\n",   c.p06_wheel_size_x10 / 10, c.p06_wheel_size_x10 % 10);
    Serial.printf("P07* Pole pairs / magnesy:    %d\n",        c.p07_speed_magnets);
    Serial.printf("P08* Limit prędkości:         %d km/h\n",   c.p08_speed_limit);
    Serial.printf("P09  Tryb startu:             %s\n",        c.p09_start_mode ? "po pedałowaniu" : "od zera");
    Serial.printf("P10* Tryb jazdy:              %d (%s)\n",   c.p10_drive_mode,
        c.p10_drive_mode == 0 ? "PAS+gaz" : c.p10_drive_mode == 1 ? "tylko gaz" : "tylko PAS");
    Serial.printf("P11  Czułość PAS:             %d\n",        c.p11_pas_sensitivity);
    Serial.printf("P12  Intensywność startu PAS: %d\n",        c.p12_pas_start_strength);
    Serial.printf("P13* Magnesy PAS:             %d\n",        c.p13_pas_magnets);
    Serial.printf("P14* Limit prądu:             %d A\n",      c.p14_current_limit_a);
    Serial.printf("P15  Napięcie odcięcia:       %.1f V\n",    c.p15_undervoltage_x10 / 10.0f);
    Serial.println("--- Parametry lokalne wyświetlacza (nie w ramce) ---");
    Serial.printf("P16  Tryb komunikacji:        [local]\n");
    Serial.printf("P17* WiFi config:             %d (%s)\n",   c.p17_cruise_control, c.p17_cruise_control ? "WiFi ON" : "WiFi OFF");
    Serial.printf("P18  Tryb gazu:               [local]\n");
    Serial.printf("P19  Power Assist:            [local]\n");
    Serial.printf("P20  Protokół:                [local]\n");
    Serial.println("     * = parametr używany w logice sterownika");
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
 *   HIGH-side ON: mcpwm_phase_pwm(op, g_mosfet_test_duty)
 *   LOW-side  ON: mcpwm_phase_regen(op, g_mosfet_test_duty)
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

    // Wybór kanału i ustawienie PWM (MCPWM)
    // HIGH-side: gen_A = PWM(duty), gen_B = forced HIGH (LS OFF)
    // LOW-side: gen_A = forced LOW (HS OFF), gen_B = active-low PWM (LS ON)
    uint16_t duty = g_mosfet_test_duty;
    const char* pinInfo = "";
    if (phase == 'A' && side == 'H') {
        mcpwm_phase_pwm(0, duty);
        pinInfo = "GPIO32 HIN_A";
    } else if (phase == 'A' && side == 'L') {
        mcpwm_phase_regen(0, duty);
        pinInfo = "GPIO33 LIN_A";
    } else if (phase == 'B' && side == 'H') {
        mcpwm_phase_pwm(1, duty);
        pinInfo = "GPIO25 HIN_B";
    } else if (phase == 'B' && side == 'L') {
        mcpwm_phase_regen(1, duty);
        pinInfo = "GPIO26 LIN_B";
    } else if (phase == 'C' && side == 'H') {
        mcpwm_phase_pwm(2, duty);
        pinInfo = "GPIO27 HIN_C";
    } else if (phase == 'C' && side == 'L') {
        mcpwm_phase_regen(2, duty);
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
        if (phase == 'A' && side == 'H') mcpwm_phase_pwm(0, duty);
        else if (phase == 'A' && side == 'L') mcpwm_phase_regen(0, duty);
        else if (phase == 'B' && side == 'H') mcpwm_phase_pwm(1, duty);
        else if (phase == 'B' && side == 'L') mcpwm_phase_regen(1, duty);
        else if (phase == 'C' && side == 'H') mcpwm_phase_pwm(2, duty);
        else if (phase == 'C' && side == 'L') mcpwm_phase_regen(2, duty);

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
        resetSineTracking(g_bldc_state.hall_state);
        return "SINUS ON";
    }
    if (cmd == "F" || cmd == "m3") {
        g_mosfet_test_active = false;
        // NIE resetuj g_manual_duty_override — użytkownik może chcieć
        // ręcznie sterować duty w FOC (man + d:60).
        g_bldc_state.mode = DRIVE_MODE_FOC;
        g_bldc_state.fault = false;
        // Reset FOC PI
        g_foc_pi_d.integral = 0.0f;
        g_foc_pi_q.integral = 0.0f;
        g_foc_vd_i = 0;
        g_foc_vq_i = 0;
        g_foc_vd_dbg = 0.0f;
        g_foc_vq_dbg = 0.0f;
        g_foc_iq_target = 0.0f;
        g_foc_last_loop_us = micros();
        // Reset EMA prądów
        g_foc_ia_ema = 0.0f;
        g_foc_ib_ema = 0.0f;
        g_foc_ic_ema = 0.0f;
        resetSineTracking(g_bldc_state.hall_state);
        return "FOC ON";
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
    if (cmd == "rev") {
        controller_config_t& cfg = config_get();
        cfg.motor_reverse = cfg.motor_reverse ? 0 : 1;
        g_reverse_isr = (cfg.motor_reverse != 0);
        if (g_bldc_state.mode == DRIVE_MODE_SINUS || g_bldc_state.mode == DRIVE_MODE_FOC) {
            resetSineTracking(g_bldc_state.hall_state);
            g_foc_pi_d.integral = 0.0f;
            g_foc_pi_q.integral = 0.0f;
        }
        config_save();
        return cfg.motor_reverse ? "Kierunek: CCW (odwrocony)" : "Kierunek: CW (normalny)";
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
    if (cmd == "idbg") {
        g_debugCurrent = !g_debugCurrent;
        g_lastDebugCurrentMs = millis();
        return g_debugCurrent ? "Debug prądów: ON (co 500ms)" : "Debug prądów: OFF";
    }
    if (cmd == "hdbg") {
        g_debugHall = !g_debugHall;
        g_lastDebugHallMs = millis();
        return g_debugHall ? "Debug Hall: ON (co 200ms)" : "Debug Hall: OFF";
    }
    if (cmd == "cdbg") {
        g_debugCommutation = !g_debugCommutation;
        if (g_debugCommutation) {
            // Reset ring buffer
            g_dbg_evt_rd = g_dbg_evt_wr;
            g_dbg_evt_overflow = 0;
        }
        return g_debugCommutation ? "Debug Commutation: ON (event-driven)" : "Debug Commutation: OFF";
    }
    // ── FOC: strojenie i diagnostyka ──
    if (cmd == "fdbg") {
        g_foc_debug = !g_foc_debug;
        g_foc_last_debug_ms = millis();
        return g_foc_debug ? "Debug FOC: ON (co 200ms)" : "Debug FOC: OFF";
    }
    if (cmd == "foc") {
        Serial.println();
        Serial.println("========== STATUS FOC ==========");
        Serial.printf("Kp=%.2f  Ki=%.2f  Limit=%.0f\n", g_foc_pi_q.kp, g_foc_pi_q.ki, g_foc_pi_q.limit);
        Serial.printf("Vd=%.1f  Vq=%.1f\n", g_foc_vd_dbg, g_foc_vq_dbg);
        Serial.printf("Id=%.2fA  Iq=%.2fA  Iq_tgt=%.2fA\n", g_foc_id_meas, g_foc_iq_meas, g_foc_iq_target);
        Serial.printf("IntD=%.1f  IntQ=%.1f\n", g_foc_pi_d.integral, g_foc_pi_q.integral);
        Serial.printf("Ia=%.2f  Ib=%.2f  Ic=%.2f (signed)\n", g_foc_ia_signed, g_foc_ib_signed, g_foc_ic_signed);
        Serial.printf("EMA: %.2f  %.2f  %.2f  (alpha=%.3f)\n", g_foc_ia_ema, g_foc_ib_ema, g_foc_ic_ema, FOC_CURRENT_EMA_ALPHA);
        Serial.printf("Mode: %s\n", g_foc_voltage_mode ? "VOLTAGE (open-loop)" : "PI (closed-loop)");
        Serial.println("================================");
        return "";
    }
    if (cmd.startsWith("fkp:")) {
        float val = cmd.substring(4).toFloat();
        if (val >= 0.0f && val <= 100.0f) {
            g_foc_pi_d.kp = val;
            g_foc_pi_q.kp = val;
            config_get().foc_kp_q = val;
            config_get().foc_kp_d = val;
            config_save();
            return "FOC Kp: " + String(val, 2) + " (zapisano)";
        }
        return "Zakres: 0.0-100.0";
    }
    if (cmd.startsWith("fki:")) {
        float val = cmd.substring(4).toFloat();
        if (val >= 0.0f && val <= 1000.0f) {
            g_foc_pi_d.ki = val;
            g_foc_pi_q.ki = val;
            config_get().foc_ki_q = val;
            config_get().foc_ki_d = val;
            config_save();
            return "FOC Ki: " + String(val, 1) + " (zapisano)";
        }
        return "Zakres: 0.0-1000.0";
    }
    if (cmd.startsWith("fkpd:")) {
        float val = cmd.substring(5).toFloat();
        if (val >= 0.0f && val <= 100.0f) {
            g_foc_pi_d.kp = val;
            config_get().foc_kp_d = val;
            config_save();
            return "FOC Kp_d: " + String(val, 2) + " (zapisano)";
        }
        return "Zakres: 0.0-100.0";
    }
    if (cmd.startsWith("fkid:")) {
        float val = cmd.substring(5).toFloat();
        if (val >= 0.0f && val <= 1000.0f) {
            g_foc_pi_d.ki = val;
            config_get().foc_ki_d = val;
            config_save();
            return "FOC Ki_d: " + String(val, 1) + " (zapisano)";
        }
        return "Zakres: 0.0-1000.0";
    }
    if (cmd == "fvolt") {
        g_foc_voltage_mode = !g_foc_voltage_mode;
        // Reset PI przy zmianie trybu
        g_foc_pi_d.integral = 0.0f;
        g_foc_pi_q.integral = 0.0f;
        g_foc_vd_i = 0;
        g_foc_vq_i = 0;
        config_get().foc_voltage_mode = g_foc_voltage_mode ? 1 : 0;
        config_save();
        return g_foc_voltage_mode
            ? "FOC Voltage mode: ON (Vq=duty, bez PI) [zapisano]"
            : "FOC Voltage mode: OFF (PI closed-loop) [zapisano]";
    }
    if (cmd == "fpitune") {
        if (g_bldc_state.mode != DRIVE_MODE_FOC) {
            return "Blad: wymaga trybu FOC (wpisz F)";
        }
        if (g_bldc_state.duty_cycle == 0) {
            return "Blad: silnik musi pracowac (duty > 0)";
        }
        if (g_foc_voltage_mode) {
            return "Blad: wylacz voltage mode (fvolt) przed auto-tune";
        }
        if (g_foc_at_active) {
            return "Auto-tune juz w toku...";
        }
        // Inicjalizacja stanu auto-tune
        // Relay amplitude: 15% aktualnego duty (proporcjonalny do punktu pracy).
        // Stała 30 PWM przy niskim duty (np. 25=5%) dawała ±120% oscylację
        // → silnik saturował → tiny amplituda → kosmiczne Ku → absurdalne Kp/Ki.
        float ff = (float)g_bldc_state.duty_cycle;
        g_foc_at_relay_amp = ff * 0.15f;
        if (g_foc_at_relay_amp < 5.0f) g_foc_at_relay_amp = 5.0f;
        if (g_foc_at_relay_amp > 50.0f) g_foc_at_relay_amp = 50.0f;
        g_foc_at_start_ms = (uint32_t)millis();
        g_foc_at_crossings = 0;
        g_foc_at_first_cross_ms = 0;
        g_foc_at_last_cross_ms = 0;
        g_foc_at_err_prev = 0.0f;
        g_foc_at_err_max = 0.0f;
        g_foc_at_err_min = 0.0f;
        g_foc_at_amp_sum = 0.0f;
        g_foc_at_amp_count = 0;
        g_foc_pi_d.integral = 0.0f;
        g_foc_pi_q.integral = 0.0f;
        g_foc_at_active = true;
        return "[PI Auto-tune] START — relay feedback 5s. Silnik moze oscylowac.";
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
        config_get().sine_hall_offset = o;
        config_save();
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "°) (zapisano)";
    }
    if (cmd == "so-") {
        int8_t o = g_sine_hall_phase_offset;
        if (o > -47) o -= 2;
        g_sine_hall_phase_offset = o;
        config_get().sine_hall_offset = o;
        config_save();
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "°) (zapisano)";
    }
    if (cmd.startsWith("so:")) {
        int val = cmd.substring(3).toInt();
        if (val < -48 || val > 48) return "Zakres: -48..+48 (1 wpis = 3.75°)";
        g_sine_hall_phase_offset = (int8_t)val;
        config_get().sine_hall_offset = (int8_t)val;
        config_save();
        return "Sine offset: " + String(val) + " (" + String((float)val * 3.75f, 1) + "°) (zapisano)";
    }
    // Auto-tune fazy sinusoidalnej
    if (cmd == "sat" || cmd.startsWith("sat:")) {
        if (g_atune_state != ATUNE_IDLE) {
            g_atune_state = ATUNE_IDLE;
            g_sine_hall_phase_offset = g_atune_saved_offset;
            g_manual_duty_override = false;
            return "[SAT] Anulowano. Offset przywrócony: " + String((int)g_atune_saved_offset);
        }
        if (g_bldc_state.mode != DRIVE_MODE_SINUS && g_bldc_state.mode != DRIVE_MODE_FOC) {
            return "[SAT] Wymaga trybu SINUS lub FOC! Użyj S/F lub m2/m3 aby włączyć.";
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
    if (cmd == "pasdir") {
        controller_config_t& cfg = config_get();
        cfg.pas_dir_invert = cfg.pas_dir_invert ? 0 : 1;
        g_pas_dir_invert_isr = (cfg.pas_dir_invert != 0);
        config_save();
        return cfg.pas_dir_invert ? "PAS kierunek: ODWROCONY (H<L=forward)" : "PAS kierunek: NORMALNY (H>L=forward)";
    }
    if (cmd == "spddbg") {
        Serial.println();
        Serial.println("========== SPEED DEBUG ==========");
        uint32_t sp = g_speed_period_us;
        uint32_t last = g_speed_last_pulse_us;
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        uint32_t age_us = (last > 0) ? (now_us - last) : 0;
        Serial.printf("period_us (raw):  %lu\n", (unsigned long)sp);
        Serial.printf("last_pulse:       %lu us ago\n", (unsigned long)age_us);
        Serial.printf("pulse_count:      %lu (accepted)\n", (unsigned long)g_speed_pulse_count);
        Serial.printf("reject_count:     %lu (debounce < %d us)\n", (unsigned long)g_speed_reject_count, SPEED_DEBOUNCE_US);
        Serial.printf("outlier_count:    %lu (outside 1/3..3x median)\n", (unsigned long)g_speed_outlier_count);
        Serial.printf("pulses_per_rev:   %d (kalibracja: spdcal, reczne: spdppr:N)\n", (int)g_speed_pulses_per_rev);
        Serial.printf("median_buf[]:     %lu, %lu, %lu us\n",
                      (unsigned long)g_speed_period_buf[0],
                      (unsigned long)g_speed_period_buf[1],
                      (unsigned long)g_speed_period_buf[2]);
        Serial.printf("median_valid:     %d/3\n", (int)g_speed_period_valid);
        Serial.printf("wheeltime_ms:     %u\n", (unsigned)g_bldc_state.wheeltime_ms);
        Serial.printf("wheel_speed:      %.1f km/h\n", g_bldc_state.wheel_speed_kmh);
        Serial.printf("P06 (wheel):      %d.%d\"\n",
                      g_display.config.p06_wheel_size_x10 / 10,
                      g_display.config.p06_wheel_size_x10 % 10);
        Serial.printf("P07 (magnets):    %d\n", (int)g_display.config.p07_speed_magnets);
        Serial.printf("P08 (speed lim):  %d km/h%s\n",
                      (int)g_display.config.p08_speed_limit,
                      g_display.config.p08_speed_limit == 0 ? " (brak limitu)" : "");
        Serial.printf("PIN_SPEED state:  %d\n", digitalRead(PIN_SPEED));
        Serial.println("=================================");
        return "";
    }
    if (cmd == "spddbg0") {
        g_speed_pulse_count = 0;
        g_speed_reject_count = 0;
        g_speed_outlier_count = 0;
        g_speed_period_valid = 0;
        g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
        g_speed_last_accepted_us = 0;
        return "SPEED liczniki wyzerowane";
    }
    if (cmd == "spdcal") {
        if (g_spdcal_state != SPDCAL_IDLE) {
            g_spdcal_state = SPDCAL_IDLE;
            return "[SPDCAL] Anulowano.";
        }
        g_spdcal_state = SPDCAL_INIT;
        return "";
    }
    if (cmd.startsWith("spdppr:")) {
        int val = cmd.substring(7).toInt();
        if (val >= 1 && val <= 20) {
            g_speed_pulses_per_rev = (uint8_t)val;
            config_get().speed_pulses_per_rev = (uint8_t)val;
            config_save();
            g_speed_period_valid = 0;  // reset mediany
            g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
            g_speed_last_accepted_us = 0;
            return "SPEED pulses_per_rev: " + String(val);
        }
        return "Zakres 1-20";
    }
    if (cmd.startsWith("pwmfreq:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 8000 && val <= 32000) {
            uint16_t actual = applyPwmFrequency((uint16_t)val);
            config_get().pwm_freq_hz = actual;
            config_save();
            return "PWM freq: " + String(actual) + " Hz";
        }
        return "Zakres 8000-32000 Hz";
    }
    if (cmd == "pwmfreq") {
        return "PWM freq: " + String(g_pwm_freq_hz) + " Hz";
    }
    if (cmd == "pasdbg") {
        Serial.println();
        Serial.println("========== PAS DEBUG ==========");
        Serial.printf("HIGH time:  %lu us\n", (unsigned long)g_pas_high_time_us);
        Serial.printf("LOW time:   %lu us\n", (unsigned long)g_pas_low_time_us);
        Serial.printf("Period:     %lu us\n", (unsigned long)g_pas_period_us);
        uint32_t ht = g_pas_high_time_us;
        uint32_t lt = g_pas_low_time_us;
        uint32_t sum = ht + lt;
        if (sum > 0) {
            Serial.printf("Duty H/L:   %lu%% / %lu%%\n", (unsigned long)(ht * 100 / sum), (unsigned long)(lt * 100 / sum));
            uint32_t diff = (ht > lt) ? (ht - lt) : (lt - ht);
            Serial.printf("Asymetria:  %lu%% (pr\xF3g: %d%%)\n", (unsigned long)(diff * 100 / sum), (int)g_pas_dir_min_asymmetry);
        }
        Serial.printf("g_pas_forward:    %d (confidence: %d)\n", (int)g_pas_forward, (int)g_pas_dir_confidence);
        Serial.printf("edge_count:       %lu\n", (unsigned long)g_pas_edge_count);
        Serial.printf("last_pulse:       %lu us ago\n", (unsigned long)((uint32_t)esp_timer_get_time() - g_pas_last_pulse_us));
        Serial.printf("pas_pedaling:     %d\n", (int)g_pas_pedaling);
        Serial.printf("fwd_since_ms:     %lu\n", (unsigned long)g_pas_fwd_since_ms);
        Serial.printf("active_since_ms:  %lu\n", (unsigned long)g_pas_active_since_ms);
        Serial.printf("pas_dir_invert:   %d\n", (int)config_get().pas_dir_invert);
        Serial.printf("pas_start_delay:  %u ms\n", (unsigned)config_get().pas_start_delay_ms);
        Serial.printf("pas_stop_delay:   %u ms\n", (unsigned)config_get().pas_stop_delay_ms);
        Serial.printf("pas_ramp:         %u ms\n", (unsigned)config_get().pas_ramp_ms);
        Serial.printf("pas_debounce:     %lu us (filter depth: %d, sample: %d us)\n",
                      (unsigned long)g_pas_debounce_us_isr, (int)g_pas_filter_depth, PAS_SAMPLE_INTERVAL_US);
        Serial.printf("min_halfperiod:   %lu us (%u ms)\n", (unsigned long)g_pas_min_halfperiod_us, (unsigned)config_get().pas_min_halfperiod_ms);
        Serial.printf("dir_asymmetry:    %d %%\n", (int)g_pas_dir_min_asymmetry);
        Serial.printf("slew_rate_max:    %d\n", (int)g_pas_slew_rate_max);
        Serial.printf("fwd_holdoff:      %u ms\n", (unsigned)g_pas_fwd_holdoff_ms);
        Serial.printf("P13 magnets:      %d\n", (int)g_display.config.p13_pas_magnets);
        Serial.println("================================");
        return "";
    }
    if (cmd == "pasat") {
        if (g_pasat_state != PASAT_IDLE) {
            g_pasat_state = PASAT_IDLE;
            return "[PASAT] Anulowano.";
        }
        g_pasat_state = PASAT_INIT;
        return "";
    }
    if (cmd.startsWith("passtart:")) {
        int val = cmd.substring(9).toInt();
        if (val >= 0 && val <= 10000) {
            config_get().pas_start_delay_ms = (uint16_t)val;
            config_save();
            return "PAS start delay: " + String(val) + " ms";
        }
        return "Zakres 0-10000 ms";
    }
    if (cmd.startsWith("passtop:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 100 && val <= 10000) {
            config_get().pas_stop_delay_ms = (uint16_t)val;
            config_save();
            return "PAS stop delay: " + String(val) + " ms";
        }
        return "Zakres 100-10000 ms";
    }
    if (cmd.startsWith("pasramp:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 0 && val <= 10000) {
            config_get().pas_ramp_ms = (uint16_t)val;
            config_save();
            return "PAS ramp (soft-start): " + String(val) + " ms";
        }
        return "Zakres 0-10000 ms";
    }
    if (cmd.startsWith("pasdbnc:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 500 && val <= 10000) {
            config_get().pas_debounce_us = (uint16_t)val;
            g_pas_debounce_us_isr = (uint32_t)val;
            uint8_t depth = (uint8_t)(val / PAS_SAMPLE_INTERVAL_US);
            if (depth < 2) depth = 2;
            g_pas_filter_depth = depth;
            config_save();
            return "PAS debounce: " + String(val) + " us (filter depth: " + String(depth) + ")";
        }
        return "Zakres 500-10000 us";
    }
    if (cmd.startsWith("pashalf:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 1 && val <= 200) {
            config_get().pas_min_halfperiod_ms = (uint8_t)val;
            g_pas_min_halfperiod_us = (uint32_t)val * 1000;
            config_save();
            return "PAS min half-period: " + String(val) + " ms (" + String(val * 1000) + " us)";
        }
        return "Zakres 1-200 ms";
    }
    if (cmd.startsWith("pasasym:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 1 && val <= 50) {
            config_get().pas_dir_asymmetry_pct = (uint8_t)val;
            g_pas_dir_min_asymmetry = (uint8_t)val;
            config_save();
            return "PAS asymetria kierunku: " + String(val) + " %";
        }
        return "Zakres 1-50 %";
    }
    if (cmd.startsWith("passlew:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 1 && val <= 100) {
            config_get().pas_slew_rate = (uint8_t)val;
            g_pas_slew_rate_max = (uint16_t)val;
            config_save();
            return "PAS slew rate max: " + String(val);
        }
        return "Zakres 1-100";
    }
    if (cmd.startsWith("pashold:")) {
        int val = cmd.substring(8).toInt();
        if (val >= 50 && val <= 2000) {
            config_get().pas_fwd_holdoff_ms = (uint16_t)val;
            g_pas_fwd_holdoff_ms = (uint16_t)val;
            config_save();
            return "PAS forward holdoff: " + String(val) + " ms";
        }
        return "Zakres 50-2000 ms";
    }
    if (cmd.startsWith("ilim:")) {
        int val = cmd.substring(5).toInt();
        if (val >= 0 && val <= 50) {
            config_get().current_limit_a = (uint8_t)val;
            config_save();
            g_current_limit_factor = 1.0f;
            g_overcurrent_fault = false;
            if (val == 0)
                return "Limit pradu: WYLACZONY (brak limitu)";
            return "Limit pradu: " + String(val) + " A";
        }
        return "Zakres 0-50 A (0=brak limitu)";
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
        Serial.println("               Tryb pracy silnika po starcie: 1=BLOCK, 2=SINUS, 3=FOC");
        Serial.printf("ramp_time_ms:  %d ms\n",   cfg.ramp_time_ms);
        Serial.println("               Czas narastania duty 0->100%. 0=natychmiastowy. Dziala w obu kierunkach.");
        Serial.printf("regen_enabled: %d (%s)\n", cfg.regen_enabled, cfg.regen_enabled ? "ON" : "OFF");
        Serial.println("               Hamowanie regeneracyjne (odzyskiwanie energii do baterii).");
        Serial.printf("pas_dir_inv:   %d (%s)\n", cfg.pas_dir_invert, cfg.pas_dir_invert ? "ODWR" : "NORM");
        Serial.println("               Inwersja kierunku PAS. Uzyj gdy silnik napedza wstecz.");
        Serial.printf("pas_start_ms:  %u ms\n",  (unsigned)cfg.pas_start_delay_ms);
        Serial.println("               Czas ciaglego pedalowania wymagany do aktywacji silnika.");
        Serial.printf("pas_stop_ms:   %u ms\n",  (unsigned)cfg.pas_stop_delay_ms);
        Serial.println("               Czas bez impulsow PAS po ktorym silnik wylacza wspomaganie.");
        Serial.printf("pas_ramp_ms:   %u ms\n",  (unsigned)cfg.pas_ramp_ms);
        Serial.println("               Soft-start PAS: czas narastania mocy 0->100% po aktywacji.");
        Serial.printf("pas_dbnc_us:   %u us (filter: %d probek x %d us)\n",
                      (unsigned)cfg.pas_debounce_us, (int)g_pas_filter_depth, PAS_SAMPLE_INTERVAL_US);
        Serial.println("               Czas potwierdzenia stanu PAS. Probkowanie co 500us, N zgodnych = zmiana stanu.");
        Serial.printf("pas_halfp_ms:  %u ms (%lu us)\n", (unsigned)cfg.pas_min_halfperiod_ms, (unsigned long)g_pas_min_halfperiod_us);
        Serial.println("               Min polokres PAS uznawany za poprawny impuls. Krotsze = szum. Komenda: pashalf:N");
        Serial.printf("pas_asym_pct:  %u %%\n", (unsigned)cfg.pas_dir_asymmetry_pct);
        Serial.println("               Prog asymetrii H/L do detekcji kierunku. Komenda: pasasym:N");
        Serial.printf("pas_slew:      %u\n", (unsigned)cfg.pas_slew_rate);
        Serial.println("               Maks. zmiana duty PAS na iteracje loop. Komenda: passlew:N");
        Serial.printf("pas_holdoff:   %u ms\n", (unsigned)cfg.pas_fwd_holdoff_ms);
        Serial.println("               Czas pedalowania fwd wymagany do stabilnego kierunku. Komenda: pashold:N");
        Serial.printf("disp_req:      %d (%s)\n", cfg.display_required, cfg.display_required ? "TAK" : "NIE");
        Serial.println("               Blokada silnika gdy wyswietlacz S866 nie jest polaczony.");
        Serial.printf("current_lim:   %d A (%s)\n", cfg.current_limit_a, cfg.current_limit_a ? "AKTYWNY" : "WYLACZONY");
        Serial.println("               Limit pradu fazowego. 0=brak limitu. P14 z display nadpisuje (jesli>0). Komenda: ilim:N");
        Serial.printf("thr_samples:   %d\n", cfg.thr_samples);
        Serial.println("               Glebokosc bufora kolowego gazu. 1 probka/loop, mediana z N. Wiecej=gladszy.");
        Serial.printf("thr_outlier:   %d\n", cfg.thr_outlier_thresh);
        Serial.println("               Max odchylenie probki od mediany (ADC). Probki dalsze odrzucane jako szum.");
        Serial.printf("duty_step:     %d %%\n",   cfg.duty_max_step_pct);
        Serial.println("               Max zmiana duty na krok petli. 0=bez limitu. Ogranicza szarpniecia mocy.");
        Serial.printf("motor_rev:     %d (%s)\n", cfg.motor_reverse, cfg.motor_reverse ? "CCW" : "CW");
        Serial.println("               Kierunek obrotow silnika. CW=normalny, CCW=odwrocony.");
        Serial.printf("sine_offset:   %d (%.1f deg)\n", (int)cfg.sine_hall_offset, (float)cfg.sine_hall_offset * 3.75f);
        Serial.println("               Przesuniecie fazowe Hall->sinus. Dobierane komenda 'sat' (auto-tune).");
        Serial.printf("foc_kp_q:      %.3f\n", cfg.foc_kp_q);
        Serial.println("               FOC: wzmocnienie proporcjonalne PI osi Q (moment obrotowy).");
        Serial.printf("foc_ki_q:      %.3f\n", cfg.foc_ki_q);
        Serial.println("               FOC: wzmocnienie calkujace PI osi Q (moment obrotowy).");
        Serial.printf("foc_kp_d:      %.3f\n", cfg.foc_kp_d);
        Serial.println("               FOC: wzmocnienie proporcjonalne PI osi D (strumien magnetyczny).");
        Serial.printf("foc_ki_d:      %.3f\n", cfg.foc_ki_d);
        Serial.println("               FOC: wzmocnienie calkujace PI osi D (strumien magnetyczny).");
        Serial.printf("foc_vmode:     %d (%s)\n", cfg.foc_voltage_mode, cfg.foc_voltage_mode ? "ON" : "OFF");
        Serial.println("               FOC tryb napieciowy: sterowanie napieciem bez PI. Do diagnostyki.");
        Serial.printf("pwm_freq_hz:   %u Hz (runtime: %u Hz)\n", (unsigned)cfg.pwm_freq_hz, (unsigned)g_pwm_freq_hz);
        Serial.println("               Czestotliwosc PWM. 8000-32000 Hz. Komenda: pwmfreq:N");
        Serial.printf("magic:         0x%08X %s\n", cfg.magic, (cfg.magic == CONFIG_MAGIC) ? "OK" : "BAD!");
        Serial.printf("version:       %d\n",      cfg.version);
        Serial.println("======================================");
        // Runtime (bie\u017c\u0105ce, mog\u0105 r\u00f3\u017cni\u0107 si\u0119 od NVS):
        Serial.println("--- Runtime (bie\u017c\u0105ce) ---");
        const char* rtMode = (g_bldc_state.mode <= DRIVE_MODE_FOC) ? modeNames[g_bldc_state.mode] : "???";
        Serial.printf("mode:          %s\n", rtMode);
        Serial.printf("ramp_time_ms:  %d ms\n", g_bldc_state.ramp_time_ms);
        Serial.printf("regen:         %s\n", g_bldc_state.regen_enabled ? "ON" : "OFF");
        Serial.printf("direction:     %s\n", g_reverse_isr ? "CCW" : "CW");
        Serial.printf("sine_offset:   %d (%.1f deg)\n", (int)g_sine_hall_phase_offset, (float)g_sine_hall_phase_offset * 3.75f);
        Serial.printf("foc_kp_q:      %.3f\n", g_foc_pi_q.kp);
        Serial.printf("foc_ki_q:      %.3f\n", g_foc_pi_q.ki);
        Serial.printf("foc_kp_d:      %.3f\n", g_foc_pi_d.kp);
        Serial.printf("foc_ki_d:      %.3f\n", g_foc_pi_d.ki);
        Serial.printf("ilim_factor:   %.2f\n", g_current_limit_factor);
        Serial.printf("ilim_I_filt:   %.1f A (raw max: %.1f A)\n",
                      g_ilim_current_ema,
                      fmaxf(fmaxf(g_bldc_state.phase_current[0], g_bldc_state.phase_current[1]), g_bldc_state.phase_current[2]));
        Serial.printf("ilim_eff:      %d A (P14=%d, NVS=%d)\n",
                      (int)getEffectiveCurrentLimit(),
                      (int)g_display.config.p14_current_limit_a,
                      (int)config_get().current_limit_a);
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
        if (cmd.startsWith("cfg:step:")) {
            int val = cmd.substring(9).toInt();
            if (val >= 0 && val <= 100) {
                cfg.duty_max_step_pct = (uint8_t)val;
                config_save();
                return "Duty max step: " + String(val) + "%";
            }
            return "Zakres 0-100 %";
        }
        if (cmd.startsWith("cfg:rev:")) {
            int val = cmd.substring(8).toInt();
            cfg.motor_reverse = val ? 1 : 0;
            g_reverse_isr = (cfg.motor_reverse != 0);
            if (g_bldc_state.mode == DRIVE_MODE_SINUS || g_bldc_state.mode == DRIVE_MODE_FOC) {
                resetSineTracking(g_bldc_state.hall_state);
                g_foc_pi_d.integral = 0.0f;
                g_foc_pi_q.integral = 0.0f;
            }
            config_save();
            return cfg.motor_reverse ? "Kierunek boot: CCW (odwrocony)" : "Kierunek boot: CW (normalny)";
        }
        // cfg:defaults — załaduj wartości domyślne do runtime (BEZ zapisu do EEPROM)
        if (cmd == "cfg:defaults") {
            cfg.drive_mode          = (uint8_t)DRIVE_MODE_BLOCK;
            cfg.ramp_time_ms        = 1200;
            cfg.regen_enabled       = 0;
            cfg.pas_dir_invert      = 0;
            cfg.pas_start_delay_ms  = 2000;
            cfg.pas_stop_delay_ms   = 1000;
            cfg.pas_ramp_ms         = 1500;
            cfg.duty_max_step_pct   = 5;
            cfg.motor_reverse       = 0;
            cfg.sine_hall_offset    = 0;
            cfg.foc_kp_q = cfg.foc_kp_d = FOC_KP_DEFAULT;
            cfg.foc_ki_q = cfg.foc_ki_d = FOC_KI_DEFAULT;
            cfg.foc_voltage_mode    = 0;
            cfg.pas_debounce_us     = PAS_DEBOUNCE_US_DEFAULT;
            cfg.pas_min_halfperiod_ms  = 5;
            cfg.pas_dir_asymmetry_pct  = 5;
            cfg.pas_slew_rate          = 30;
            cfg.pas_fwd_holdoff_ms     = 300;
            cfg.display_required    = 1;
            cfg.thr_samples         = 8;
            cfg.thr_outlier_thresh  = 150;
            cfg.speed_pulses_per_rev = 1;
            cfg.pwm_freq_hz         = 20000;
            // Zastosuj do runtime
            g_bldc_state.ramp_time_ms   = cfg.ramp_time_ms;
            g_bldc_state.regen_enabled  = false;
            g_pas_dir_invert_isr        = false;
            g_pas_debounce_us_isr       = PAS_DEBOUNCE_US_DEFAULT;
            { uint8_t d = (uint8_t)(PAS_DEBOUNCE_US_DEFAULT / PAS_SAMPLE_INTERVAL_US); g_pas_filter_depth = (d < 2) ? 2 : d; }
            g_pas_min_halfperiod_us     = 5000;
            g_pas_dir_min_asymmetry     = 5;
            g_pas_slew_rate_max         = 30;
            g_pas_fwd_holdoff_ms        = 300;
            g_speed_pulses_per_rev      = 1;
            g_speed_period_valid        = 0;
            g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
            g_speed_last_accepted_us    = 0;
            applyPwmFrequency(20000);
            g_reverse_isr               = false;
            g_sine_hall_phase_offset    = 0;
            g_foc_pi_d.kp = g_foc_pi_q.kp = FOC_KP_DEFAULT;
            g_foc_pi_d.ki = g_foc_pi_q.ki = FOC_KI_DEFAULT;
            g_foc_pi_d.integral = g_foc_pi_q.integral = 0.0f;
            g_foc_voltage_mode      = false;
            Serial.println("[CFG] Wartosci domyslne zaladowane do runtime (nie zapisano do EEPROM).");
            Serial.println("[CFG] Uzyj cfg:save aby zapisac do EEPROM.");
            return "";
        }
        // cfg:save — zsynchronizuj wartości runtime → config i zapisz do EEPROM
        if (cmd == "cfg:save") {
            cfg.ramp_time_ms       = g_bldc_state.ramp_time_ms;
            cfg.regen_enabled      = g_bldc_state.regen_enabled ? 1 : 0;
            cfg.pas_dir_invert     = g_pas_dir_invert_isr ? 1 : 0;
            cfg.motor_reverse      = g_reverse_isr ? 1 : 0;
            cfg.sine_hall_offset   = g_sine_hall_phase_offset;
            cfg.foc_kp_q           = g_foc_pi_q.kp;
            cfg.foc_ki_q           = g_foc_pi_q.ki;
            cfg.foc_kp_d           = g_foc_pi_d.kp;
            cfg.foc_ki_d           = g_foc_pi_d.ki;
            cfg.foc_voltage_mode   = g_foc_voltage_mode ? 1 : 0;
            cfg.pas_debounce_us    = (uint16_t)g_pas_debounce_us_isr;
            cfg.pas_min_halfperiod_ms  = (uint8_t)(g_pas_min_halfperiod_us / 1000);
            cfg.pas_dir_asymmetry_pct  = g_pas_dir_min_asymmetry;
            cfg.pas_slew_rate          = (uint8_t)g_pas_slew_rate_max;
            cfg.pas_fwd_holdoff_ms     = g_pas_fwd_holdoff_ms;
            cfg.speed_pulses_per_rev   = g_speed_pulses_per_rev;
            cfg.pwm_freq_hz            = g_pwm_freq_hz;
            // display_required nie ma runtime var — zapisuje się bezpośrednio w cfg
            config_save();
            return "[CFG] Aktualne wartosci runtime zapisane do EEPROM.";
        }
        // cfg:reload — wczytaj config z EEPROM i zastosuj do runtime
        if (cmd == "cfg:reload") {
            config_init();  // wczytuje NVS do g_config (lub reset do domyslnych)
            controller_config_t& c = config_get();
            g_bldc_state.ramp_time_ms   = c.ramp_time_ms;
            g_bldc_state.regen_enabled  = (c.regen_enabled != 0);
            g_pas_dir_invert_isr        = (c.pas_dir_invert != 0);
            g_reverse_isr               = (c.motor_reverse != 0);
            g_sine_hall_phase_offset    = c.sine_hall_offset;
            g_foc_pi_d.kp = c.foc_kp_d;  g_foc_pi_d.ki = c.foc_ki_d;
            g_foc_pi_q.kp = c.foc_kp_q;  g_foc_pi_q.ki = c.foc_ki_q;
            g_foc_pi_d.integral = g_foc_pi_q.integral = 0.0f;
            g_foc_voltage_mode = (c.foc_voltage_mode != 0);
            g_pas_debounce_us_isr = (uint32_t)c.pas_debounce_us;
            { uint8_t d = (uint8_t)(g_pas_debounce_us_isr / PAS_SAMPLE_INTERVAL_US); g_pas_filter_depth = (d < 2) ? 2 : d; }
            g_pas_min_halfperiod_us = (uint32_t)c.pas_min_halfperiod_ms * 1000;
            g_pas_dir_min_asymmetry = c.pas_dir_asymmetry_pct;
            g_pas_slew_rate_max     = c.pas_slew_rate;
            g_pas_fwd_holdoff_ms    = c.pas_fwd_holdoff_ms;
            g_speed_pulses_per_rev  = c.speed_pulses_per_rev;
            if (g_speed_pulses_per_rev < 1) g_speed_pulses_per_rev = 1;
            g_speed_period_valid    = 0;  // reset mediany po reload
            g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
            g_speed_last_accepted_us = 0;
            if (c.pwm_freq_hz >= 8000 && c.pwm_freq_hz <= 32000)
                applyPwmFrequency(c.pwm_freq_hz);
            // display_required nie wymaga sync — czytany bezpośrednio z config_get()
            // Przełącz tryb silnika jeśli skonfigurowany tryb rozni się od bieżącego
            drive_mode_t target_mode = (drive_mode_t)c.drive_mode;
            if (target_mode >= DRIVE_MODE_BLOCK && target_mode <= DRIVE_MODE_FOC
                && target_mode != g_bldc_state.mode) {
                g_bldc_state.mode  = target_mode;
                g_bldc_state.fault = false;
                g_foc_vd_i = 0; g_foc_vq_i = 0;
                g_foc_vd_dbg = 0.0f; g_foc_vq_dbg = 0.0f;
                g_foc_iq_target = 0.0f;
                g_foc_ia_ema = g_foc_ib_ema = g_foc_ic_ema = 0.0f;
                g_foc_last_loop_us = micros();
                resetSineTracking(g_bldc_state.hall_state);
                const char* mnames[] = {"DISABLED","BLOCK","SINUS","FOC"};
                Serial.printf("[CFG] Tryb przełączony na: %s\n", mnames[target_mode]);
            }
            return "[CFG] Wartosci z EEPROM zaladowane do runtime.";
        }
        if (cmd.startsWith("cfg:dispreq:")) {
            int val = cmd.substring(12).toInt();
            cfg.display_required = val ? 1 : 0;
            config_save();
            return val ? "Wyswietlacz WYMAGANY (silnik tylko z display)" : "Wyswietlacz OPCJONALNY (standalone OK)";
        }
        if (cmd.startsWith("cfg:ilim:")) {
            int val = cmd.substring(9).toInt();
            if (val >= 0 && val <= 50) {
                cfg.current_limit_a = (uint8_t)val;
                config_save();
                g_current_limit_factor = 1.0f;
                g_overcurrent_fault = false;
                if (val == 0)
                    return "Limit pradu NVS: WYLACZONY (brak limitu)";
                return "Limit pradu NVS: " + String(val) + " A";
            }
            return "Zakres 0-50 A (0=brak limitu)";
        }
        if (cmd.startsWith("cfg:thrsamp:")) {
            int val = cmd.substring(12).toInt();
            if (val >= 2 && val <= 16) {
                cfg.thr_samples = (uint8_t)val;
                config_save();
                return "Throttle samples: " + String(val);
            }
            return "Zakres 2-16";
        }
        if (cmd.startsWith("cfg:thrdelta:")) {
            int val = cmd.substring(13).toInt();
            if (val >= 10 && val <= 2000) {
                cfg.thr_outlier_thresh = (uint16_t)val;
                config_save();
                return "Throttle outlier thresh: " + String(val);
            }
            return "Zakres 10-2000";
        }
        if (cmd.startsWith("cfg:dutymin:")) {
            int val = cmd.substring(12).toInt();
            if (val >= 0 && val <= 50) {
                cfg.duty_min_pct = (uint8_t)val;
                config_save();
                return "Min duty: " + String(val) + "% (ponizej = 0)";
            }
            return "Zakres 0-50 %";
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

// ============================================================================
// Serwer WWW konfiguracji przez WiFi
// Aktywny gdy P17=1 w menu wyswietlacza S866.
// WiFi AP: SSID="BLDC_Config", haslo="bldc1234", IP=192.168.4.1
// Silnik wylaczony gdy WiFi wlaczone (ADC2 koliduje z WiFi).
// ============================================================================

static const char BLDC_WIFI_SSID[] = "BLDC_Config";
static const char BLDC_WIFI_PASS[] = "bldc1234";

// Strona HTML w pamieci flash (PROGMEM)
static const char BLDC_WEB_HTML[] PROGMEM = R"bldc_html(<!DOCTYPE html><html lang="pl"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BLDC Config</title>
<style>*{box-sizing:border-box;margin:0;padding:0}body{font-family:system-ui,sans-serif;background:#0d1117;color:#c9d1d9;padding-bottom:52px}h1{padding:10px 14px;background:#161b22;font-size:.92em;border-bottom:2px solid #238636;line-height:1.5}details{border-bottom:1px solid #21262d}summary{padding:9px 14px;cursor:pointer;list-style:none;background:#161b22;font-size:.88em;font-weight:600;color:#e6edf3}summary::-webkit-details-marker{display:none}.card{padding:10px 14px}.row{display:flex;align-items:center;gap:6px;margin:5px 0;flex-wrap:wrap}label{min-width:195px;font-size:.8em;color:#8b949e}input[type=number],input[type=text],select{flex:1;min-width:70px;padding:5px 7px;background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;font-size:.85em}button{padding:5px 11px;background:#238636;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:.8em;white-space:nowrap}button:hover{filter:brightness(1.15)}button.w{background:#5a5a60}button.r{background:#b62324}.sg{display:grid;grid-template-columns:auto 1fr;gap:3px 10px;font-size:.82em}.sl{color:#8b949e}.sv{font-weight:600;color:#58a6ff}hr{border:0;border-top:1px solid #21262d;margin:8px 0}p.hint{font-size:.78em;color:#8b949e;margin:0 0 8px}#notif{position:fixed;bottom:0;left:0;right:0;padding:8px 14px;background:#161b22;font-size:.78em;color:#3fb950;border-top:1px solid #238636;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}</style>
</head><body>
<h1>&#9889; BLDC ESP32 Konfigurator &#8212; WiFi: <b>BLDC_Config</b> / bldc1234 &#8212; 192.168.4.1</h1>
<details open><summary>&#128202; Stan systemu</summary><div class="card">
<div class="sg"><span class="sl">Tryb:</span><span class="sv" id="s_mode">-</span><span class="sl">Pr&#281;dko&#347;&#263;:</span><span class="sv" id="s_spd">-</span><span class="sl">Bateria:</span><span class="sv" id="s_vbat">-</span><span class="sl">Duty:</span><span class="sv" id="s_dty">-</span><span class="sl">Fault:</span><span class="sv" id="s_flt">-</span><span class="sl">FOC Vmode (RT):</span><span class="sv" id="s_fvolt">-</span><span class="sl">Offset Hall (RT):</span><span class="sv" id="s_soff">-</span><span class="sl">Kolejka:</span><span class="sv" id="s_q" style="color:#f0883e">brak</span></div>
<div class="row" style="margin-top:8px"><button onclick="loadCfg()">&#128260; Od&#347;wie&#380;</button></div>
</div></details>
<details open><summary>&#9881; Silnik</summary><div class="card">
<div class="row"><label>Tryb startu (boot)</label><select id="drv_m"><option value="1">BLOCK</option><option value="2">SINUS</option><option value="3">FOC</option></select><button onclick="cmd('cfg:mode:'+v('drv_m'))">Ustaw</button></div>
<p class="hint">Tryb pracy silnika po w&#322;&#261;czeniu zasilania. BLOCK=komutacja trapezowa, SINUS=sinusoidalna, FOC=wektorowe sterowanie polem.</p>
<div class="row"><label>Rampa 0&#8594;100% (ms)</label><input type="number" id="ramp_ms" min="0" max="10000" step="100"><button onclick="cmd('cfg:ramp:'+v('ramp_ms'))">Ustaw</button></div>
<p class="hint">Czas narastania mocy od 0 do 100%. Dzia&#322;a w obu kierunkach (przyspieszanie i zwalnianie). 0=natychmiastowy.</p>
<div class="row"><label>Regen hamowanie</label><select id="regen_e"><option value="0">OFF</option><option value="1">ON</option></select><button onclick="cmd('cfg:regen:'+v('regen_e'))">Ustaw</button></div>
<p class="hint">Hamowanie regeneracyjne &#8212; odzyskiwanie energii do baterii przy zwalnianiu.</p>
<div class="row"><label>Max krok duty (%)</label><input type="number" id="duty_s" min="0" max="100"><button onclick="cmd('cfg:step:'+v('duty_s'))">Ustaw</button></div>
<p class="hint">Maksymalna zmiana duty na jeden cykl p&#281;tli. Ogranicza szarpni&#281;cia mocy. 0=bez limitu, domy&#347;lnie 5%.</p>
<div class="row"><label>Kierunek silnika</label><select id="mot_rev"><option value="0">CW (normalny)</option><option value="1">CCW (odwr&#243;cony)</option></select><button onclick="cmd('cfg:rev:'+v('mot_rev'))">Ustaw</button></div>
<p class="hint">Kierunek obrot&#243;w silnika. Zmie&#324; je&#347;li ko&#322;o kr&#281;ci si&#281; w z&#322;&#261; stron&#281;.</p>
<div class="row"><label>Offset fazy Hall</label><select id="sin_off"></select><button onclick="cmd('so:'+v('sin_off'))">Ustaw</button></div>
<p class="hint">Przesuni&#281;cie fazowe Hall&#8594;sinus. Dobierane automatycznie komend&#261; 'sat' (auto-tune) lub r&#281;cznie.</p>
</div></details>
<details><summary>&#128690; Asystent PAS</summary><div class="card">
<div class="row"><label>Op&#243;&#378;nienie startu (ms)</label><input type="number" id="pas_s" min="0" max="10000" step="100"><button onclick="cmd('passtart:'+v('pas_s'))">Ustaw</button></div>
<p class="hint">Czas ci&#261;g&#322;ego peda&#322;owania do przodu wymagany do aktywacji silnika. Zapobiega przypadkowemu w&#322;&#261;czeniu.</p>
<div class="row"><label>Op&#243;&#378;nienie stopu (ms)</label><input type="number" id="pas_t" min="100" max="10000" step="100"><button onclick="cmd('passtop:'+v('pas_t'))">Ustaw</button></div>
<p class="hint">Czas bez impuls&#243;w PAS po kt&#243;rym silnik wy&#322;&#261;cza wspomaganie. Ni&#380;sza warto&#347;&#263; = szybsza reakcja na stop.</p>
<div class="row"><label>Rampa PAS (ms)</label><input type="number" id="pas_r" min="0" max="10000" step="100"><button onclick="cmd('pasramp:'+v('pas_r'))">Ustaw</button></div>
<p class="hint">Soft-start PAS: czas narastania mocy 0&#8594;100% po aktywacji. 0=natychmiastowy, 1500=&#322;agodne ~1.5s.</p>
<div class="row"><label>Kierunek PAS</label><select id="pas_d"><option value="0">Normalny</option><option value="1">Odwr&#243;cony</option></select><button onclick="setPasDir()">Ustaw</button></div>
<p class="hint">Inwersja kierunku czujnika PAS. U&#380;yj gdy silnik nap&#281;dza przy peda&#322;owaniu do ty&#322;u.</p>
<div class="row"><label>Debounce PAS (&#181;s)</label><input type="number" id="pas_db" min="500" max="10000" step="100"><button onclick="cmd('pasdbnc:'+v('pas_db'))">Ustaw</button></div>
<p class="hint">Filtr cyfrowy PAS: pin pr&#243;bkowany co 500&#181;s, zmiana stanu wymaga N kolejnych zgodnych pr&#243;bek (N=debounce/500). Szpilki EMI &lt;N&#215;500&#181;s s&#261; odfiltrowane.</p>
<div class="row"><label>Min p&#243;&#322;okres PAS (ms)</label><input type="number" id="pas_hp" min="1" max="200" step="1"><button onclick="cmd('pashalf:'+v('pas_hp'))">Ustaw</button></div>
<p class="hint">Minimalny czas p&#243;&#322;okresu (HIGH lub LOW) uznawany za poprawny impuls PAS. Kr&#243;tsze impulsy odrzucane jako szum.</p>
<div class="row"><label>Asymetria kierunku (%)</label><input type="number" id="pas_as" min="1" max="50" step="1"><button onclick="cmd('pasasym:'+v('pas_as'))">Ustaw</button></div>
<p class="hint">Pr&#243;g asymetrii HIGH/LOW do detekcji kierunku. Wi&#281;kszy=mniej czu&#322;y ale stabilniejszy. Je&#347;li |H-L|/(H+L)&gt;pr&#243;g &rarr; kierunek rozpoznany.</p>
<div class="row"><label>Slew rate PAS</label><input type="number" id="pas_sl" min="1" max="100" step="1"><button onclick="cmd('passlew:'+v('pas_sl'))">Ustaw</button></div>
<p class="hint">Maks. zmiana wype&#322;nienia PAS na iteracj&#281; p&#281;tli g&#322;&#243;wnej (loop). Mniejszy=&#322;agodniejsze zmiany mocy.</p>
<div class="row"><label>Forward holdoff (ms)</label><input type="number" id="pas_fh" min="50" max="2000" step="50"><button onclick="cmd('pashold:'+v('pas_fh'))">Ustaw</button></div>
<p class="hint">Czas peda&#322;owania do przodu wymagany do uznania kierunku &quot;forward&quot; za stabilny (od momentu detekcji asymetrii).</p>
</div></details>
<details><summary>&#128295; Ustawienia systemowe</summary><div class="card">
<div class="row"><label>Wymagany wy&#347;wietlacz</label><select id="disp_rq"><option value="1">TAK (silnik tylko z display)</option><option value="0">NIE (standalone OK)</option></select><button onclick="cmd('cfg:dispreq:'+v('disp_rq'))">Ustaw</button></div>
<p class="hint">Blokada silnika gdy wy&#347;wietlacz S866 nie jest pod&#322;&#261;czony. TAK=silnik wymaga display, NIE=dzia&#322;a samodzielnie.</p>
<div class="row"><label>Limit pr&#261;du [A]</label><input type="number" id="ilim" min="0" max="50" step="1"><button onclick="cmd('cfg:ilim:'+v('ilim'))">Ustaw</button></div>
<p class="hint">Limit pr&#261;du fazowego (ochrona silnika/MOSFET). 0=brak limitu. P14 z wy&#347;wietlacza nadpisuje gdy &gt;0. Komenda Serial: ilim:N</p>
<div class="row"><label>Throttle bufor pr&#243;bek</label><input type="number" id="thr_n" min="2" max="16" step="1"><button onclick="cmd('cfg:thrsamp:'+v('thr_n'))">Ustaw</button></div>
<p class="hint">G&#322;&#281;boko&#347;&#263; bufora ko&#322;owego gazu: 1 pr&#243;bka ADC na iteracj&#281; loop(), mediana z N. Wi&#281;cej = g&#322;adszy sygna&#322;. Zakres 2-16.</p>
<div class="row"><label>Throttle max odchylenie</label><input type="number" id="thr_d" min="10" max="2000" step="10"><button onclick="cmd('cfg:thrdelta:'+v('thr_d'))">Ustaw</button></div>
<p class="hint">Pr&#243;bki ADC z bufora ko&#322;owego oddalone od mediany o wi&#281;cej ni&#380; ta warto&#347;&#263; s&#261; odrzucane jako szum EMI. Zakres 10-2000.</p>
</div></details>
<details><summary>&#128260; Parametry FOC</summary><div class="card">
<div class="row"><label>Tryb napi&#281;ciowy (fvolt)</label><span id="fvolt_st" class="sv">-</span><button onclick="doFvolt()">Prze&#322;&#261;cz fvolt</button><button class="w" onclick="queueCmd('fpitune')">&#10141; Kolejka: fpitune</button></div>
<p class="hint">Tryb napi&#281;ciowy: bezpo&#347;rednie sterowanie napi&#281;ciem (bez regulatora PI). Do diagnostyki i por&#243;wnania SINUS vs FOC.</p>
<div class="row"><label>Kp o&#347; Q (torque) + D</label><input type="number" id="fkp_q" min="0" max="100" step="0.01"><button onclick="cmd('fkp:'+v('fkp_q'))">Ustaw Q+D</button></div>
<p class="hint">Wzmocnienie proporcjonalne PI osi Q (moment obrotowy). Wi&#281;ksze = szybsza reakcja, za du&#380;e = oscylacje.</p>
<div class="row"><label>Ki o&#347; Q (torque) + D</label><input type="number" id="fki_q" min="0" max="1000" step="0.1"><button onclick="cmd('fki:'+v('fki_q'))">Ustaw Q+D</button></div>
<p class="hint">Wzmocnienie ca&#322;kuj&#261;ce PI osi Q. Eliminuje b&#322;&#261;d ustalony. Za du&#380;e = wolne oscylacje (wind-up).</p>
<div class="row"><label>Kp_d o&#347; D (flux)</label><input type="number" id="fkp_d" min="0" max="100" step="0.01"><button onclick="cmd('fkpd:'+v('fkp_d'))">Ustaw D</button></div>
<p class="hint">Wzmocnienie proporcjonalne PI osi D (strumie&#324; magnetyczny). Utrzymuje Id&#8776;0 (brak rozmagnesowania).</p>
<div class="row"><label>Ki_d o&#347; D (flux)</label><input type="number" id="fki_d" min="0" max="1000" step="0.1"><button onclick="cmd('fkid:'+v('fki_d'))">Ustaw D</button></div>
<p class="hint">Wzmocnienie ca&#322;kuj&#261;ce PI osi D. Eliminuje b&#322;&#261;d ustalony strumienia magnetycznego.</p>
</div></details>
<details><summary>&#128190; EEPROM / NVS</summary><div class="card">
<div class="row"><button onclick="sendR('cfg:save')">&#128190; Zapisz do EEPROM</button><button class="w" onclick="sendR('cfg:reload')">&#128194; Wczytaj z EEPROM</button><button class="r" onclick="cfgDef()">&#9888; Domy&#347;lne</button></div>
<hr><p class="hint">Warto&#347;ci zapisane w NVS (EEPROM):</p>
<div class="sg"><span class="sl">Tryb boot:</span><span class="sv" id="n_dm">-</span><span class="sl">Rampa:</span><span class="sv" id="n_ramp">-</span><span class="sl">Regen:</span><span class="sv" id="n_regen">-</span><span class="sl">Max krok duty:</span><span class="sv" id="n_step">-</span><span class="sl">Kierunek:</span><span class="sv" id="n_rev">-</span><span class="sl">Offset Hall:</span><span class="sv" id="n_soff">-</span><span class="sl">FOC Vmode:</span><span class="sv" id="n_fvolt">-</span><span class="sl">Kp_q / Ki_q:</span><span class="sv" id="n_focq">-</span><span class="sl">Kp_d / Ki_d:</span><span class="sv" id="n_focd">-</span><span class="sl">PAS start:</span><span class="sv" id="n_pstart">-</span><span class="sl">PAS stop:</span><span class="sv" id="n_pstop">-</span><span class="sl">PAS rampa:</span><span class="sv" id="n_pramp">-</span><span class="sl">PAS kierunek:</span><span class="sv" id="n_pdir">-</span><span class="sl">PAS debounce:</span><span class="sv" id="n_pdbnc">-</span><span class="sl">PAS min p&#243;&#322;okres:</span><span class="sv" id="n_phalf">-</span><span class="sl">PAS asymetria:</span><span class="sv" id="n_pasym">-</span><span class="sl">PAS slew rate:</span><span class="sv" id="n_pslew">-</span><span class="sl">PAS holdoff:</span><span class="sv" id="n_phold">-</span><span class="sl">Limit pr&#261;du:</span><span class="sv" id="n_ilim">-</span><span class="sl">Wym. wy&#347;wietlacz:</span><span class="sv" id="n_dreq">-</span><span class="sl">Thr pr&#243;bki:</span><span class="sv" id="n_thrn">-</span><span class="sl">Thr odchylenie:</span><span class="sv" id="n_thrd">-</span></div>
</div></details>
<details><summary>&#127919; Kolejka &#183; Komendy</summary><div class="card">
<p class="hint">Komenda wykonana gdy P17&#8594;0 (WiFi wy&#322;&#261;czone, silnik dost&#281;pny)</p>
<div class="row"><select id="q_m"><option value="">-- brak zmiany --</option><option value="d">&#9940; OFF (d)</option><option value="B">&#128309; BLOCK</option><option value="S">&#126; SINUS</option><option value="F">&#127744; FOC</option><option value="sat">&#128269; SAT (auto-tune offsetu fazy)</option><option value="fpitune">&#10024; FOC PI Auto-tune</option><option value="man">&#9757; man (manual duty toggle)</option></select><button onclick="qMode()">Do kolejki</button><button class="r" onclick="clrQ()">Wyczy&#347;&#263;</button></div>
<hr>
<p class="hint">Dowolna komenda Serial (natychmiastowa, dzia&#322;a podczas WiFi):</p>
<div class="row"><input type="text" id="dcmd" placeholder="np. cfg, foc, so:4, pasdbg..." onkeydown="if(event.key==='Enter')doDirect()"><button onclick="doDirect()">Wy&#347;lij</button></div>
</div></details>
<div id="notif">&#8987; &#321;adowanie konfiguracji...</div>
<script>
const v=id=>document.getElementById(id).value;
const el=id=>document.getElementById(id);
const MODES=['DISABLED','BLOCK','SINUS','FOC'];
let _c={};
(function(){const s=el('sin_off');for(let i=-48;i<=48;i+=2){const o=document.createElement('option');o.value=i;o.textContent=i+' ('+(i*3.75).toFixed(1)+'\u00b0)';s.appendChild(o);}})();
function notif(m,e){const n=el('notif');n.textContent=m;n.style.color=e?'#f85149':'#3fb950';n.style.borderTopColor=e?'#b62324':'#238636';}
async function cmd(c){
  try{const fd=new FormData();fd.append('cmd',c);const r=await fetch('/api/cmd',{method:'POST',body:fd});const t=await r.text();notif((t||'OK')+' \u2190 '+c);return t;}
  catch(e){notif('B\u0142\u0105d: '+e,1);}
}
async function sendR(c){await cmd(c);loadCfg();}
async function queueCmd(c){
  try{const fd=new FormData();fd.append('cmd',c);await fetch('/api/queue',{method:'POST',body:fd});el('s_q').textContent=c||'brak';notif(c?'Kolejka: '+c:'Kolejka wyczyszczona');}
  catch(e){notif('B\u0142\u0105d: '+e,1);}
}
function qMode(){const c=v('q_m');if(!c){notif('Wybierz komend\u0119');return;}queueCmd(c);}
function clrQ(){queueCmd('');}
function cfgDef(){if(confirm('Za\u0142adowa\u0107 warto\u015bci domy\u015blne?\n(nie zapisuje do EEPROM)'))sendR('cfg:defaults');}
function setPasDir(){const d=parseInt(v('pas_d'));const cur=(_c.pas_dir_invert||0);if(d!==cur)cmd('pasdir');else notif('Kierunek PAS bez zmian');}
function doDirect(){const i=el('dcmd');const c=i.value.trim();if(!c)return;cmd(c).then(()=>{i.value='';});}
async function doFvolt(){await cmd('fvolt');loadCfg();}
async function loadCfg(){
  try{
    const r=await fetch('/api/config');_c=await r.json();
    el('s_mode').textContent=MODES[_c.rt_mode]||String(_c.rt_mode);
    el('s_spd').textContent=_c.rt_speed.toFixed(1)+' km/h';
    el('s_vbat').textContent=_c.rt_vbat.toFixed(1)+' V';
    el('s_dty').textContent=_c.rt_duty+'%';
    el('s_flt').textContent=_c.rt_fault?'\u26a0\ufe0f FAULT':'OK';
    el('s_flt').style.color=_c.rt_fault?'#f85149':'#3fb950';
    const fvOn=_c.rt_fvolt;
    el('s_fvolt').textContent=fvOn?'ON (Vmode)':'OFF (PI)';
    el('s_fvolt').style.color=fvOn?'#f0883e':'#3fb950';
    el('s_soff').textContent=_c.rt_sine_offset+' ('+(_c.rt_sine_offset*3.75).toFixed(1)+'\u00b0)';
    el('s_q').textContent=_c.queued_cmd||'brak';
    el('drv_m').value=_c.drive_mode;el('ramp_ms').value=_c.ramp_time_ms;
    el('regen_e').value=_c.regen_enabled;el('duty_s').value=_c.duty_max_step_pct;
    el('mot_rev').value=_c.motor_reverse;el('sin_off').value=_c.sine_hall_offset;
    el('pas_s').value=_c.pas_start_delay_ms;el('pas_t').value=_c.pas_stop_delay_ms;
    el('pas_r').value=_c.pas_ramp_ms;el('pas_d').value=_c.pas_dir_invert;
    el('pas_db').value=_c.pas_debounce_us||3000;
    el('pas_hp').value=_c.pas_min_halfperiod_ms||5;
    el('pas_as').value=_c.pas_dir_asymmetry_pct||5;
    el('pas_sl').value=_c.pas_slew_rate||30;
    el('pas_fh').value=_c.pas_fwd_holdoff_ms||300;
    el('disp_rq').value=_c.display_required||0;
    el('ilim').value=_c.current_limit_a||0;
    el('thr_n').value=_c.thr_samples||8;
    el('thr_d').value=_c.thr_outlier_thresh||150;
    el('fkp_q').value=(_c.foc_kp_q||0).toFixed(3);el('fki_q').value=(_c.foc_ki_q||0).toFixed(3);
    el('fkp_d').value=(_c.foc_kp_d||0).toFixed(3);el('fki_d').value=(_c.foc_ki_d||0).toFixed(3);
    const fvSt=el('fvolt_st');fvSt.textContent=fvOn?'ON (fvolt)':'OFF (PI)';fvSt.style.color=fvOn?'#f0883e':'#3fb950';
    const DM=['?','BLOCK','SINUS','FOC'];
    el('n_dm').textContent=DM[_c.drive_mode]||String(_c.drive_mode);
    el('n_ramp').textContent=_c.ramp_time_ms+' ms';
    el('n_regen').textContent=_c.regen_enabled?'ON':'OFF';
    el('n_step').textContent=_c.duty_max_step_pct+'%';
    el('n_rev').textContent=_c.motor_reverse?'CCW':'CW';
    el('n_soff').textContent=_c.sine_hall_offset+' ('+(_c.sine_hall_offset*3.75).toFixed(1)+'\u00b0)';
    el('n_fvolt').textContent=_c.foc_voltage_mode?'ON':'OFF';el('n_fvolt').style.color=_c.foc_voltage_mode?'#f0883e':'#3fb950';
    el('n_focq').textContent=(_c.foc_kp_q||0).toFixed(3)+' / '+(_c.foc_ki_q||0).toFixed(3);
    el('n_focd').textContent=(_c.foc_kp_d||0).toFixed(3)+' / '+(_c.foc_ki_d||0).toFixed(3);
    el('n_pstart').textContent=(_c.pas_start_delay_ms||0)+' ms';
    el('n_pstop').textContent=(_c.pas_stop_delay_ms||0)+' ms';
    el('n_pramp').textContent=(_c.pas_ramp_ms||0)+' ms';
    el('n_pdir').textContent=_c.pas_dir_invert?'Odwr\u00f3cony':'Normalny';el('n_pdir').style.color=_c.pas_dir_invert?'#f0883e':'#3fb950';
    el('n_pdbnc').textContent=(_c.pas_debounce_us||3000)+' \u00b5s';
    el('n_phalf').textContent=(_c.pas_min_halfperiod_ms||5)+' ms';
    el('n_pasym').textContent=(_c.pas_dir_asymmetry_pct||5)+' %';
    el('n_pslew').textContent=(_c.pas_slew_rate||30);
    el('n_phold').textContent=(_c.pas_fwd_holdoff_ms||300)+' ms';
    el('n_ilim').textContent=(_c.current_limit_a||0)+' A';el('n_ilim').style.color=(_c.current_limit_a>0)?'#3fb950':'#f0883e';
    el('n_dreq').textContent=_c.display_required?'TAK':'NIE';el('n_dreq').style.color=_c.display_required?'#3fb950':'#f0883e';
    el('n_thrn').textContent=_c.thr_samples||8;
    el('n_thrd').textContent=_c.thr_outlier_thresh||150;
    notif('OK \u2013 '+new Date().toLocaleTimeString());
  }catch(e){notif('B\u0142\u0105d ładowania: '+e,1);}
}
setInterval(loadCfg,5000);loadCfg();
</script></body></html>)bldc_html";

// --- Handlery HTTP ---

static void webHandleRoot() {
    if (g_web_server) g_web_server->send_P(200, "text/html", BLDC_WEB_HTML);
}

static void webHandleApiConfig() {
    if (!g_web_server) return;
    controller_config_t& cfg = config_get();
    char buf[1536];
    snprintf(buf, sizeof(buf),
        "{\"drive_mode\":%d,\"ramp_time_ms\":%d,\"regen_enabled\":%d,"
        "\"pas_dir_invert\":%d,\"pas_start_delay_ms\":%d,"
        "\"pas_stop_delay_ms\":%d,\"pas_ramp_ms\":%d,"
        "\"duty_max_step_pct\":%d,\"motor_reverse\":%d,"
        "\"sine_hall_offset\":%d,\"foc_voltage_mode\":%d,"
        "\"foc_kp_q\":%.4f,\"foc_ki_q\":%.4f,"
        "\"foc_kp_d\":%.4f,\"foc_ki_d\":%.4f,"
        "\"pas_debounce_us\":%d,"
        "\"pas_min_halfperiod_ms\":%d,"
        "\"pas_dir_asymmetry_pct\":%d,"
        "\"pas_slew_rate\":%d,"
        "\"pas_fwd_holdoff_ms\":%d,"
        "\"display_required\":%d,"
        "\"current_limit_a\":%d,"
        "\"thr_samples\":%d,\"thr_outlier_thresh\":%d,"
        "\"rt_mode\":%d,\"rt_speed\":%.2f,\"rt_vbat\":%.2f,"
        "\"rt_duty\":%u,\"rt_fault\":%s,"
        "\"rt_fvolt\":%s,\"rt_sine_offset\":%d,"
        "\"queued_cmd\":\"%s\"}",
        (int)cfg.drive_mode, (int)cfg.ramp_time_ms, (int)cfg.regen_enabled,
        (int)cfg.pas_dir_invert, (int)cfg.pas_start_delay_ms,
        (int)cfg.pas_stop_delay_ms, (int)cfg.pas_ramp_ms,
        (int)cfg.duty_max_step_pct, (int)cfg.motor_reverse,
        (int)cfg.sine_hall_offset, (int)cfg.foc_voltage_mode,
        cfg.foc_kp_q, cfg.foc_ki_q, cfg.foc_kp_d, cfg.foc_ki_d,
        (int)cfg.pas_debounce_us,
        (int)cfg.pas_min_halfperiod_ms,
        (int)cfg.pas_dir_asymmetry_pct,
        (int)cfg.pas_slew_rate,
        (int)cfg.pas_fwd_holdoff_ms,
        (int)cfg.display_required,
        (int)cfg.current_limit_a,
        (int)cfg.thr_samples, (int)cfg.thr_outlier_thresh,
        (int)g_bldc_state.mode,
        g_bldc_state.wheel_speed_kmh,
        g_bldc_state.battery_voltage,
        (unsigned int)((uint32_t)g_bldc_state.duty_cycle * 100u / PWM_MAX_DUTY),
        g_bldc_state.fault ? "true" : "false",
        g_foc_voltage_mode ? "true" : "false",
        (int)g_sine_hall_phase_offset,
        g_web_queued_cmd.c_str()
    );
    g_web_server->send(200, "application/json", buf);
}

static void webHandleApiCmd() {
    if (!g_web_server) return;
    if (!g_web_server->hasArg("cmd")) {
        g_web_server->send(400, "text/plain", "Brak cmd");
        return;
    }
    String c = g_web_server->arg("cmd");
    c.trim();
    if (c.length() == 0) {
        g_web_server->send(400, "text/plain", "Pusta komenda");
        return;
    }
    // Blokuj komendy wlaczajace silnik gdy WiFi aktywne (ADC2 konflikt)
    if (c == "B" || c == "e" || c == "S" || c == "m2" || c == "F" || c == "m3") {
        g_web_server->send(403, "text/plain",
            "Silnik zablokowany (WiFi aktywne). Uzyj kolejki lub wylacz WiFi (P17=0).");
        return;
    }
    String result = executeCommand(c);
    g_web_server->send(200, "text/plain", result.length() > 0 ? result : "OK");
}

static void webHandleApiQueue() {
    if (!g_web_server) return;
    if (!g_web_server->hasArg("cmd")) {
        g_web_server->send(400, "text/plain", "Brak cmd");
        return;
    }
    g_web_queued_cmd = g_web_server->arg("cmd");
    g_web_queued_cmd.trim();
    Serial.printf("[WEB] Kolejka: '%s'\n", g_web_queued_cmd.c_str());
    g_web_server->send(200, "text/plain",
        g_web_queued_cmd.length() > 0 ? "OK: " + g_web_queued_cmd : "Kolejka wyczyszczona");
}

static void webHandleNotFound() {
    if (g_web_server) g_web_server->send(404, "text/plain", "404 Not Found");
}

/**
 * @brief Uruchamia WiFi AP i serwer HTTP na porcie 80.
 * Wylacza silnik przed wlaczeniem WiFi (ADC2 - przepustnica/temp koliduje z WiFi).
 */
static void webConfigInit() {
    if (g_wifi_active) return;

    // Wylacz silnik — ADC2 (przepustnica, temperatury) koliduje z WiFi
    g_bldc_state.mode       = DRIVE_MODE_DISABLED;
    g_bldc_state.duty_cycle  = 0;
    g_bldc_state.duty_target = 0;
    g_duty_ramped            = 0;
    allMosfetsOff();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(BLDC_WIFI_SSID, BLDC_WIFI_PASS);
    Serial.printf("[WiFi] AP '%s' uruchomiony, IP: %s\n",
                  BLDC_WIFI_SSID, WiFi.softAPIP().toString().c_str());

    g_web_server = new WebServer(80);
    g_web_server->on("/",           HTTP_GET,  webHandleRoot);
    g_web_server->on("/api/config", HTTP_GET,  webHandleApiConfig);
    g_web_server->on("/api/cmd",    HTTP_POST, webHandleApiCmd);
    g_web_server->on("/api/queue",  HTTP_POST, webHandleApiQueue);
    g_web_server->onNotFound(webHandleNotFound);
    g_web_server->begin();
    g_wifi_active = true;
    Serial.printf("[WiFi] HTTP port 80. Polacz z '%s' (haslo: '%s'), otworz http://192.168.4.1\n",
                   BLDC_WIFI_SSID, BLDC_WIFI_PASS);
}

/**
 * @brief Zatrzymuje serwer HTTP i wylacza WiFi (ADC2 dostepne po powrocie).
 */
static void webConfigStop() {
    if (!g_wifi_active) return;
    if (g_web_server) {
        g_web_server->stop();
        delete g_web_server;
        g_web_server = nullptr;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    g_wifi_active = false;
    Serial.println("[WiFi] AP zatrzymany, WiFi OFF.");
}

/**
 * @brief Wywolywany w loop() gdy WiFi aktywne — obsluguje klientow HTTP.
 */
static void webConfigHandle() {
    if (g_web_server) g_web_server->handleClient();
}


