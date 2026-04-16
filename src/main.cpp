/**
 * @file main.cpp
 * @brief BLDC Motor Driver â€” ESP32
 * @version 2.0.0
 *
 * Sterownik silnika BLDC (bezszczotkowego) 3-fazowego na ESP32.
 * ObsĹ‚ugiwane metody sterowania:
 *   - Komutacja blokowa (6-step / trapezoidalna)
 *   - Komutacja sinusoidalna (ciÄ…gĹ‚e Ĺ›ledzenie kÄ…ta z LUT 97-wpisĂłw)
 *   - FOC (Field Oriented Control) â€” inverse Park/Clarke + SVPWM
 *
 * ## Architektura
 * Komutacja odbywa siÄ™ w ISR MCPWM TEZ (Timer Equals Zero, 20 kHz),
 * niezaleĹĽnie od loop(). Center-aligned PWM (UP_DOWN counter) zapewnia
 * synchroniczny pomiar prÄ…du w dolinie PWM (wszystkie low-side ON).
 * DziÄ™ki temu Serial.printf() i inne wolne operacje w loop() nie powodujÄ…
 * zakĹ‚ĂłceĹ„ w sterowaniu silnikiem.
 *
 * ## Filtracja wejĹ›Ä‡
 * - PAS: timer prĂłbkujÄ…cy co 500Âµs + filtr cyfrowy (N zgodnych prĂłbek)
 * - Przepustnica: bufor koĹ‚owy 1 prĂłbka/loop + mediana + outlier rejection
 * - Hamulec: debounce z licznikiem kolejnych odczytĂłw
 * - Halle silnika: debounce w ISR komutacji (HALL_MIN_PERIOD_US)
 *
 * ## PWM: MCPWM center-aligned (v2.0)
 * Migracja z LEDC (edge-aligned, 10-bit) na MCPWM (center-aligned, period=500).
 * Trzy operatory Ă— 2 generatory (gen_A=HIN, gen_B=LIN) sterowane bezpoĹ›rednio
 * przez zapisy rejestrĂłw LL w ISR (~5 ns, bez spinlockĂłw driver API).
 * Prescaler: group=1, timer=8 â†’ 20 MHz, period=500 â†’ dokĹ‚adnie 20 kHz.
 * Shadow compare: TEZ-only â†’ symetryczne impulsy center-aligned.
 * Dead time: bypass (IR2103 wewnÄ™trzny ~520 ns).
 *
 * ## Sterowniki mostu
 * IR2103: HIN=active HIGH (high-side ON gdy HIGH),
 *         LIN=active LOW (low-side ON gdy LOW â€” logika odwrĂłcona!).
 * MCPWM center-aligned: gen_A(HIN) i gen_B(LIN) sterowane niezaleĹĽnie.
 * SINUS/FOC: oba generatory = ten sam duty (IR2103 complementary + dead time).
 * BLOCK: gen_A=PWM, gen_B=force HIGH/LOW zaleĹĽnie od stanu fazy.
 *
 * ## PrzepĹ‚yw danych
 * loop() czyta ADC/Hall/GPIO â†’ aktualizuje zmienne volatile â†’ ISR MCPWM odczytuje je
 * w kaĹĽdym przerwaniu TEZ i ustawia odpowiednie generatory MCPWM.
 *
 * ## Konfiguracja
 * NVS (EEPROM): 64-bajtowa struktura controller_config_t (CONFIG_VERSION=16).
 * Interfejs: Serial 115200 baud + WiFi AP (P17=1) z responsywnym UI.
 *
 * ## Hardware
 * - MCU: ESP32-D0WDQ6, 240 MHz, Arduino via PlatformIO
 * - Gate drivers: 3x IR2103
 * - Pomiar prÄ…du: INA180A2 (gain=50 V/V) + shunt 2 mÎ©
 * - Czujniki Halla: GPIO5(A)/18(B)/19(C), INPUT_PULLUP
 * - VBAT dzielnik: 1.13 MÎ© / 31.7 kÎ©
 * - Przepustnica: GPIO2, ADC 400-2600 RAW â†’ 0-100%
 * - WyĹ›wietlacz: S866 protokĂłĹ‚ 2, Serial2 9600 baud
 * - PAS: GPIO22, timer sampling 2 kHz + filtr cyfrowy
 */

#include <Arduino.h>
#include "pinout.h"
#include "bldc_types.h"
#include "display_s866.h"
#include "bldc_config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "hal/mcpwm_ll.h"
#include "driver/adc.h"           // adc1_config_width/channel_atten
#include "soc/sens_struct.h"      // SENS â€” direct register access for ISR-safe ADC

// ============================================================================
// Zmienne globalne
// ============================================================================

bldc_state_t g_bldc_state;

// MCPWM ISR handle (zastÄ™puje hw_timer â€” komutacja w przerwaniu TEZ center-aligned)
static intr_handle_t g_mcpwm_isr_handle = NULL;
/// Flaga: ISR MCPWM przeczytaĹ‚ pierwszy odczyt ADC prÄ…du
volatile bool g_adc_ready_isr = false;
/// CiÄ…gĹ‚a EMA prÄ…du per kanaĹ‚ w ISR (fixed-point Q8: wartoĹ›Ä‡ Ă— 256)
/// Aktualizowana co tick ISR (20kHz), spike > ADC_SPIKE_THRESHOLD â†’ pominiÄ™ty.
/// SHIFT=4 â†’ alpha=1/16, tau=16/20000=0.8ms. Odczyt w loop(): ema>>8.
volatile uint32_t g_phase_adc_ema_q8[3] = {0, 0, 0};
/// Ostatni surowy odczyt ADC (do diagnostyki idbg/cdbg)
volatile uint16_t g_phase_adc_raw_isr[3] = {0, 0, 0};
/// PrĂłg odrzucania szpilek ADC (~30A przy INA180+2mÎ© shunt)
#define ADC_SPIKE_THRESHOLD  3800
/// StaĹ‚a EMA w ISR: shift=4 â†’ alpha=1/16, tauâ‰0.8ms @ 20kHz
#define ADC_EMA_SHIFT        4
/// Flaga: ISR moĹĽe czytaÄ‡ ADC1 (false=loop() czyta ADC1, ISR pomija)
volatile bool g_adc_isr_active = false;
/// Licznik timeoutĂłw ADC w ISR (diagnostyka)
volatile uint32_t g_adc_timeout_count = 0;
volatile uint8_t g_hall_isr = 0;
volatile uint16_t g_duty_isr = 0;
volatile bool g_motor_enabled = false;
volatile bool g_brake_isr = false;
volatile drive_mode_t g_mode_isr = DRIVE_MODE_DISABLED;   ///< Tryb sterowania (do ISR)
static uint16_t g_pwm_freq_hz = PWM_FREQUENCY;           ///< Aktualna czÄ™stotliwoĹ›Ä‡ PWM [Hz]
static uint16_t applyPwmFrequency(uint16_t freq_hz);     ///< Forward declaration

// Debug Hall â€” Ĺ›ledzenie komutacji w ISR
volatile uint8_t  g_dbg_hall_raw_isr = 0;       ///< Ostatni surowy odczyt Hall z GPIO
volatile uint8_t  g_dbg_hall_filt_isr = 0;      ///< Ostatni przefiltrowany Hall uĹĽyty do komutacji
volatile uint8_t  g_dbg_commut_path = 0;        ///< 0=off, 1=block, 2=sinus, 3=foc, 4=block_startup, 5=block12

// Pomiar RPM z przejĹ›Ä‡ Halla (aktualizowane w ISR timera)
volatile uint8_t g_hall_prev_isr = 0;         ///< Poprzedni POTWIERDZONY stan Halla w ISR
volatile uint32_t g_hall_last_change_us = 0;  ///< micros() ostatniego potwierdzonego przejĹ›cia
volatile uint32_t g_hall_period_us = 0;       ///< Okres miÄ™dzy przejĹ›ciami Halla [Âµs]

// Multi-sample Hall confirmation (anty-EMI)
// Nowy stan Halla musi siÄ™ utrzymaÄ‡ przez HALL_CONFIRM_COUNT kolejnych
// odczytĂłw ISR (NĂ—50Âµs przy 20kHz) zanim zostanie zaakceptowany.
// Chroni przed glitchami EMI z PWM sprzÄ™gajÄ…cymi siÄ™ w linie Halla.
#define HALL_CONFIRM_COUNT      3              ///< Wymagane kolejne zgodne odczyty
static uint8_t  g_hall_candidate = 0;          ///< Kandydat na nowy stan Halla
static uint8_t  g_hall_confirm_cnt = 0;        ///< Ile razy z rzÄ™du widzieliĹ›my kandydata

// Sinusoidal commutation state â€” ported from bldc_driver_v2 (STM32, proven algorithm)
// Continuous angle tracking with Hall correction, NOT snap-to-hall approach.
volatile uint32_t g_sine_angle_q16 = 0;       ///< KÄ…t elektr. w wpisach tabeli (Q16, 0..96<<16)
volatile uint32_t g_sine_speed_q16 = 0;       ///< PrÄ™dkoĹ›Ä‡: wpisĂłw tabeli na tick ISR (Q16)
volatile uint32_t g_sine_sector_speed[6] = {0}; ///< PrÄ™dkoĹ›Ä‡ per-sector (kompensacja nierĂłwnych Halli)
volatile uint8_t  g_sine_running = 0;          ///< 1 = tryb sinusoidalny aktywny, 0 = block startup
volatile uint8_t  g_sine_startup_count = 0;    ///< Licznik komutacji blokowych przed przejĹ›ciem na sinus
volatile int8_t   g_sine_last_hall_idx = -1;   ///< Ostatni indeks Halla w sekwencji (0-5, -1=unknown)
volatile int8_t   g_sine_dir = 1;              ///< Kierunek z przejĹ›Ä‡ Halla: +1 forward, -1 reverse
volatile uint32_t g_sine_last_hall_ms = 0;     ///< HAL tick ostatniego przejĹ›cia Halla (stall detection)
volatile int8_t   g_sine_hall_phase_offset = 0; ///< Runtime-tunable Hall phase offset (-48..+48 entries, 1 entry = 3.75Â°)
static int8_t     g_web_assist_override = -1;   ///< -1=auto(display), 0..15=override z WWW

/// Startup state machine for SINUS/FOC: 0=IDLE, 1=ALIGN, 2=RUN
volatile uint8_t  g_startup_state = 0;
volatile uint32_t g_startup_align_start_us = 0; ///< Timestamp rozpoczÄ™cia alignmentu [us]
volatile uint8_t  g_startup_align_hall = 0;     ///< Hall zapisany na poczÄ…tku alignmentu

// FOC state â€” regulatory PI i wektory napiÄ™ciowe
struct foc_pi_t {
    float kp;
    float ki;
    float integral;
    float limit;
};
// Inicjalizacja g_foc_pi_d/q: wartoĹ›ci domyĹ›lne ustawiane w setup()
// (SINE_SAFE_MAX_DUTY jeszcze nie zdefiniowany w tym momencie pliku)
static foc_pi_t g_foc_pi_d = {0};
static foc_pi_t g_foc_pi_q = {0};
volatile int32_t g_foc_vd_i = 0;          ///< NapiÄ™cie d do ISR (jednostki PWM: -SAFE_MAX..+SAFE_MAX)
volatile int32_t g_foc_vq_i = 0;          ///< NapiÄ™cie q do ISR (jednostki PWM)
volatile float g_foc_iq_target = 0.0f;    ///< Docelowy prÄ…d Iq [A] (torque)
// Debug/pomiar FOC (zapisywane w loop, czytane w diagnostyce)
static float g_foc_id_meas = 0.0f;       ///< Zmierzony Id [A]
static float g_foc_iq_meas = 0.0f;       ///< Zmierzony Iq [A]
static float g_foc_vd_dbg = 0.0f;        ///< Kopia Vd do debugowania (float, non-volatile)
static float g_foc_vq_dbg = 0.0f;        ///< Kopia Vq do debugowania (float, non-volatile)
static float g_foc_ia_signed = 0.0f;     ///< PrÄ…d fazy A [A] (surowy, przed klipowaniem)
static float g_foc_ib_signed = 0.0f;     ///< PrÄ…d fazy B [A]
static float g_foc_ic_signed = 0.0f;     ///< PrÄ…d fazy C [A]
// EMA-filtrowane prÄ…dy dla FOC (zachowane do diagnostyki/debugowania)
static float g_foc_ia_ema = 0.0f;        ///< EMA Ia [A]
static float g_foc_ib_ema = 0.0f;        ///< EMA Ib [A]
static float g_foc_ic_ema = 0.0f;        ///< EMA Ic [A]
// EMA-filtrowane Id/Iq (po transformacie Parka â€” sygnaĹ‚y DC)
static float g_foc_id_ema = 0.0f;        ///< EMA Id [A] (po Park, bez phase lag)
static float g_foc_iq_ema = 0.0f;        ///< EMA Iq [A] (po Park, bez phase lag)
static bool g_foc_debug = false;          ///< Debug FOC (komenda fdbg)
static bool g_foc_voltage_mode = false;   ///< Tryb napiÄ™ciowy: Vq = duty wprost, bez PI
static unsigned long g_foc_last_debug_ms = 0;
static unsigned long g_foc_last_loop_us = 0;  ///< Timestamp ostatniej iteracji FOC loop

// â”€â”€ PI Auto-tune (metoda relay / Ă…strĂ¶m-HĂ¤gglund) â”€â”€
// Relay feedback: zamiast PI, wyjĹ›cie przeĹ‚Ä…czane +/-relay_amp gdy bĹ‚Ä…d >/<0.
// Mierzy oscylacje Iq â†’ oblicza ultimate gain Ku, period Tu â†’ Kp, Ki (Z-N PI).
static bool     g_foc_at_active = false;     ///< Auto-tune w toku
static float    g_foc_at_relay_amp = 30.0f;  ///< Amplituda relay [PWM] (Â± wokĂłĹ‚ feedforward)
static uint32_t g_foc_at_start_ms = 0;       ///< Timestamp startu [ms]
static uint32_t g_foc_at_duration_ms = 5000; ///< Czas trwania testu [ms]
static float    g_foc_at_err_prev = 0.0f;    ///< Poprzedni bĹ‚Ä…d Iq (detekcja zero-crossing)
static uint16_t g_foc_at_crossings = 0;      ///< Liczba przejĹ›Ä‡ przez zero bĹ‚Ä™du
static uint32_t g_foc_at_first_cross_ms = 0; ///< Timestamp pierwszego zero-crossing
static uint32_t g_foc_at_last_cross_ms = 0;  ///< Timestamp ostatniego zero-crossing
static float    g_foc_at_err_max = 0.0f;     ///< Max bĹ‚Ä…d w bieĹĽÄ…cym pĂłĹ‚-cyklu
static float    g_foc_at_err_min = 0.0f;     ///< Min bĹ‚Ä…d w bieĹĽÄ…cym pĂłĹ‚-cyklu
static float    g_foc_at_amp_sum = 0.0f;     ///< Suma amplitud oscylacji (do Ĺ›redniej)
static uint16_t g_foc_at_amp_count = 0;      ///< Liczba zmierzonych amplitud

// Debug sterowania sinusoidalnego (snapshot + liczniki zdarzeĹ„ ISR)
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
volatile int32_t  g_dbg_last_hall_err = 0;     ///< Ostatni bĹ‚Ä…d korekcji kÄ…ta Halla (Q16)
volatile uint32_t g_dbg_snap_count = 0;        ///< Licznik peĹ‚nych snapĂłw kÄ…ta (desync/crawl)
volatile uint32_t g_dbg_corr_count = 0;        ///< Licznik Ĺ‚agodnych korekcji kÄ…ta
volatile uint32_t g_dbg_hall_invalid = 0;      ///< Odrzucone stany 0/7
volatile uint32_t g_dbg_hall_glitch = 0;       ///< Glitche EMI (kandydat zmieniony przed potwierdzeniem)
volatile uint32_t g_dbg_hall_seq_reject = 0;   ///< Odrzucone (zĹ‚a sekwencja sektora w SINUS/FOC)

// â”€â”€ Rozszerzona diagnostyka SINUS/FOC (running min/max resetowane co debug print) â”€â”€
// Zbierane w ISR, wyĹ›wietlane i resetowane w printSineDebug()
volatile int32_t  g_dbg_err_min_q16 = INT32_MAX; ///< Min bĹ‚Ä…d kÄ…ta na Hall edge (Q16) w oknie
volatile int32_t  g_dbg_err_max_q16 = INT32_MIN; ///< Max bĹ‚Ä…d kÄ…ta na Hall edge (Q16) w oknie
volatile uint32_t g_dbg_speed_min = 0xFFFFFFFF; ///< Min prÄ™dkoĹ›Ä‡ q16 w oknie
volatile uint32_t g_dbg_speed_max = 0;          ///< Max prÄ™dkoĹ›Ä‡ q16 w oknie
volatile uint32_t g_dbg_dt_min = 0xFFFFFFFF;    ///< Min okres Hall [Âµs] w oknie
volatile uint32_t g_dbg_dt_max = 0;             ///< Max okres Hall [Âµs] w oknie
volatile uint32_t g_dbg_snap_window = 0;        ///< Snapy w bieĹĽÄ…cym oknie
volatile uint32_t g_dbg_corr_window = 0;        ///< Korekcje w bieĹĽÄ…cym oknie
volatile int32_t  g_dbg_angle_at_hall = 0;      ///< KÄ…t (entry Q16) w momencie Hall (przed korekcjÄ…)
volatile int32_t  g_dbg_expected_at_hall = 0;   ///< Oczekiwany kÄ…t (entry Q16) wg tabeli sektorĂłw

// â”€â”€ Event-driven debug: ring buffer zdarzeĹ„ ISR â†’ loop() â”€â”€
// ISR wypeĹ‚nia na zmianÄ™ Halla / komutacji, loop() drukuje
#define DBG_EVT_RING_SIZE 64
struct dbg_evt_t {
    uint32_t ts_us;           ///< Timestamp zdarzenia [Âµs]
    uint8_t  hall_raw;        ///< Surowy Hall [C:B:A] z GPIO.in
    uint8_t  hall_filt;       ///< Przefiltrowany Hall uĹĽyty do komutacji
    uint8_t  mode;            ///< drive_mode_t (0=OFF,1=BLK,2=SIN,3=FOC)
    int8_t   sector_old;      ///< Poprzedni sektor (0-5, -1=unknown)
    int8_t   sector_new;      ///< Nowy sektor
    int8_t   dir;             ///< Kierunek: +1 forward, -1 reverse
    uint8_t  startup_cnt;     ///< g_sine_startup_count
    uint32_t angle_q16;       ///< KÄ…t po korekcji
    uint32_t speed_q16;       ///< PrÄ™dkoĹ›Ä‡ Q16
    uint32_t hall_period_us;  ///< Okres miÄ™dzy edge Halla [Âµs]
    uint16_t duty;            ///< Duty z przepustnicy/rampy
    int16_t  da, db, dc;      ///< Duty zastosowane na fazach A/B/C
    uint16_t adc_a, adc_b, adc_c; ///< Surowy ADC prÄ…du faz
    int32_t  foc_vd, foc_vq;  ///< FOC: napiÄ™cia Vd/Vq (tylko tryb FOC)
    uint8_t  flags;           ///< b0=stalled, b1=snap, b2=seq_reject, b3=reverse
};
volatile dbg_evt_t g_dbg_evt_ring[DBG_EVT_RING_SIZE];
volatile uint8_t g_dbg_evt_wr = 0;      ///< Indeks zapisu (ISR)
volatile uint8_t g_dbg_evt_rd = 0;      ///< Indeks odczytu (loop)
volatile uint32_t g_dbg_evt_overflow = 0; ///< Licznik utraconych zdarzeĹ„
static bool g_debugCommutation = false;   ///< WĹ‚Ä…czone komendÄ… 'cdbg'

// Duty zastosowane na fazach (ustawiane w sinusCommutateISR/focCommutateISR/block)
volatile int16_t g_dbg_last_da = 0;
volatile int16_t g_dbg_last_db = 0;
volatile int16_t g_dbg_last_dc = 0;

// Pomiar RPM z pinu SPEED (GPIO ISR, dla silnikĂłw przekĹ‚adniowych, P07==1)
volatile uint32_t g_speed_last_pulse_us = 0;  ///< micros() ostatniego impulsu SPEED
volatile uint32_t g_speed_period_us = 0;      ///< Okres miÄ™dzy impulsami SPEED [Âµs]
volatile uint32_t g_speed_pulse_count = 0;    ///< Licznik zaakceptowanych impulsĂłw SPEED
volatile uint32_t g_speed_reject_count = 0;   ///< Licznik odrzuconych impulsĂłw (debounce)
#define SPEED_DEBOUNCE_US  30000              ///< Min okres SPEED [Âµs] (~250 km/h @26")

// Mediana z 3 ostatnich pomiarĂłw okresu SPEED (filtr outlier)
static uint32_t g_speed_period_buf[3] = {0, 0, 0};
static uint8_t  g_speed_period_idx = 0;
static uint8_t  g_speed_period_valid = 0;  ///< Ile pomiarĂłw w buforze (0..3)
static uint32_t g_speed_outlier_count = 0; ///< Ile pomiarĂłw odrzuconych jako outlier (>3x lub <1/3 mediany)
static uint8_t  g_speed_pulses_per_rev = 1; ///< Impulsy SPEED na obrĂłt koĹ‚a (z NVS, kalibracja: spdcal)
static uint32_t g_speed_last_processed_sp = 0;  ///< Ostatni przetworzony period ISR (skip duplikatĂłw)
static uint32_t g_speed_last_accepted_us = 0;   ///< Timestamp ostatniego zaakceptowanego pomiaru do mediany

// PAS (Pedal Assist Sensor) â€” pomiar kadencji i kierunku z przerwania GPIO
volatile uint32_t g_pas_last_pulse_us = 0;    ///< micros() ostatniej krawÄ™dzi PAS
volatile uint32_t g_pas_period_us = 0;        ///< PeĹ‚ny okres (HIGH+LOW) PAS [Âµs]
volatile uint32_t g_pas_rising_us = 0;        ///< Czas ostatniego RISING edge
volatile uint32_t g_pas_falling_us = 0;       ///< Czas ostatniego FALLING edge
volatile uint32_t g_pas_high_time_us = 0;     ///< Czas trwania stanu HIGH [Âµs]
volatile uint32_t g_pas_low_time_us = 0;      ///< Czas trwania stanu LOW [Âµs]
volatile bool g_pas_forward = true;           ///< Kierunek pedaĹ‚owania (asymetria duty cycle)
volatile uint32_t g_pas_last_fwd_pulse_us = 0; ///< micros() ostatniej krawÄ™dzi PAS w kierunku FORWARD (aktualizowane tylko w ISR)
volatile int8_t g_pas_dir_confidence = 0;     ///< Licznik pewnoĹ›ci kierunku: >0=fwd, <0=rev (histereza ISR)
volatile bool g_pas_dir_invert_isr = false;   ///< Kopia pas_dir_invert dla ISR (ustawiana w setup/cmd)
volatile uint32_t g_pas_debounce_us_isr = 3000; ///< Kopia pas_debounce_us dla ISR (ustawiana z config)
volatile uint32_t g_pas_edge_count = 0;       ///< Licznik krawÄ™dzi PAS (ISR inkrementuje)

/// PAS sampling: timer co PAS_SAMPLE_INTERVAL_US prĂłbkuje pin zamiast przerwania.
/// Stan zmieni siÄ™ dopiero po g_pas_filter_depth kolejnych zgodnych prĂłbkach.
/// Eliminuje szpilki EMI z silnika (<1ms) ktĂłre nie utrzymajÄ… siÄ™ NĂ—500Âµs.
#define PAS_SAMPLE_INTERVAL_US  500   ///< Okres prĂłbkowania PAS [Âµs] (2 kHz)
#define PAS_DEBOUNCE_US_DEFAULT 3000  ///< DomyĹ›lny czas potwierdzenia stanu [Âµs]
volatile uint8_t  g_pas_filter_count = 0;     ///< Ile kolejnych prĂłbek rĂłĹĽni siÄ™ od stanu filtrowanego
volatile bool     g_pas_filtered_state = false; ///< Aktualny przefiltrowany stan pinu PAS
volatile uint8_t  g_pas_filter_depth = 6;     ///< Ile zgodnych prĂłbek do zmiany stanu (debounce_us/500)
static esp_timer_handle_t g_pas_sample_timer = nullptr;

/// Minimalny pĂłĹ‚okres PAS: domyĹ›lnie 5ms â€” nadpisywany z NVS (pas_min_halfperiod_ms)
static uint32_t g_pas_min_halfperiod_us = 5000;
/// Minimalna rĂłĹĽnica duty cycle do detekcji kierunku (%)
/// DomyĹ›lnie 5% â€” nadpisywany z NVS (pas_dir_asymmetry_pct)
static uint8_t g_pas_dir_min_asymmetry = 5;

// Regeneracja â€” zmienne volatile dla ISR
volatile bool g_regen_active_isr = false;     ///< Tryb regen aktywny (do ISR)
volatile uint16_t g_regen_duty_isr = 0;       ///< SiĹ‚a hamowania regen 0-PWM_MAX_DUTY

// Limit prÄ…dowy (P14 / NVS current_limit_a)
static float g_current_limit_factor = 1.0f;   ///< MnoĹĽnik duty z limitera prÄ…dowego (0.0â€“1.0)
static bool  g_overcurrent_fault = false;      ///< Hard cutoff: prÄ…d > 150% limitu â†’ blokada
static unsigned long g_overcurrent_fault_ms = 0; ///< Czas wejĹ›cia w fault [ms]
static float g_ilim_current_ema = 0.0f;        ///< EMA-filtrowany max prÄ…d fazowy dla limitera [A]
static uint8_t g_ilim_hard_count = 0;          ///< Licznik kolejnych prĂłbek powyĹĽej hard cutoff
#define OVERCURRENT_HARD_MULT  1.5f            ///< MnoĹĽnik: hard cutoff przy 150% limitu
#define OVERCURRENT_FAULT_MS   500             ///< Czas blokady po hard cutoff [ms]
#define OVERCURRENT_CONSEC     3               ///< Wymagane kolejne prĂłbki powyĹĽej progu hard cutoff
#define ILIMIT_KP_DOWN         0.05f           ///< Proporcjonalny spadek przy przekroczeniu [/A]
#define ILIMIT_RECOVER_RATE    0.5f            ///< SzybkoĹ›Ä‡ odzyskiwania [1/s]
#define ILIMIT_EMA_ALPHA       0.15f           ///< EMA Î± filtru prÄ…du dla limitera (szybszy niĹĽ FOC)
#define ILIMIT_STARTUP_GRACE_MS 2000            ///< Grace period: ignoruj ILIM przez 2s po starcie
static unsigned long g_startup_ms = 0;          ///< Timestamp startu firmware (millis)

// Tryb testowy MOSFETĂłw â€” diagnostyka uszkodzonych tranzystorĂłw
volatile bool g_mosfet_test_active = false;    ///< Tryb testu MOSFET aktywny (ISR nie rusza LEDC)
static uint16_t g_mosfet_test_duty = PWM_MAX_DUTY * 10 / 100;  ///< Duty testowe (domyĹ›lnie 10%)
static char g_mosfet_test_phase = 0;           ///< Aktualnie testowana faza ('A','B','C') lub 0
static char g_mosfet_test_side  = 0;           ///< Aktualnie testowana strona ('H','L') lub 0

// OdwrĂłcenie kierunku obrotĂłw (software switch CW/CCW)
volatile bool g_reverse_isr = false;           ///< Kierunek: false=CW (domyĹ›lny), true=CCW

// ============================================================================
// Sterowanie sinusoidalne â€” port z bldc_driver_v2 (STM32, sprawdzony algorytm)
// ============================================================================
//
// Algorytm ĹşrĂłdĹ‚owy: bldc_driver_v2/src/bldc.c, TIM1_UP_IRQHandler()
// Kluczowe cechy:
//   1. CiÄ…gĹ‚e Ĺ›ledzenie kÄ…ta (angle_q16 += speed_q16 co tick ISR)
//   2. Hall KORYGUJE kÄ…t (1/8 bĹ‚Ä™du), NIE narzuca go
//   3. Block startup: 6 komutacji blokowych buduje dane o prÄ™dkoĹ›ci
//   4. Stall freeze: brak Halla >200ms â†’ zamroĹĽenie kÄ…ta
//   5. Center-aligned complementary PWM: duty = center + sine * amp
//
// Tablica: 97 elementĂłw (96 + guard entry), wartoĹ›ci -1024..+1024
// 96 wpisy = 360Â° elektrycznych, 16 wpisĂłw na sektor (60Â°)
// RozdzielczoĹ›Ä‡: 3.75Â° na wpis

/**
 * @brief Tablica sinusa: 96 wpisĂłw + 1 guard (wrap-around).
 * WartoĹ›ci: round(sin(i Ă— 360Â°/96) Ă— 1024), zakres -1024..+1024.
 * Guard entry [96] = [0] = 0 dla bezpiecznej interpolacji.
 * DRAM_ATTR: uint8/int16 load z IRAM â†’ LoadStoreError na ESP32.
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
 * @brief Mapowanie Hallâ†’indeks sektora (0-5) dla sekwencji CW.
 *
 * Sekwencja CW z komutacji blokowej: 1â†’3â†’2â†’6â†’4â†’5
 * Sektor 0 = Hall 1, Sektor 1 = Hall 3, ... Sektor 5 = Hall 5
 * WartoĹ›Ä‡ -1 = nieprawidĹ‚owy stan Halla (0 lub 7).
 */
static const DRAM_ATTR int8_t g_hall_to_sector[8] = {
    -1,     // 0 = invalid
     0,     // 1 (001) â†’ sector 0  (block: Aâ†’B)
     2,     // 2 (010) â†’ sector 2  (block: Bâ†’Câ’)
     1,     // 3 (011) â†’ sector 1  (block: Aâ†’Câ’)
     4,     // 4 (100) â†’ sector 4  (block: Câ†’Aâ’)
     5,     // 5 (101) â†’ sector 5  (block: Câ†’Bâ’)
     3,     // 6 (110) â†’ sector 3  (block: Bâ†’Aâ’)
    -1      // 7 = invalid
};

static const DRAM_ATTR uint8_t g_block_hall_seq[6] = {1, 3, 2, 6, 4, 5};

static inline uint8_t IRAM_ATTR blockHallNextCw(uint8_t bh) {
    int8_t sector = g_hall_to_sector[bh];
    if (sector < 0) return bh;
    return g_block_hall_seq[(sector + 1) % 6];
}

static inline const char* driveModeName(drive_mode_t mode) {
    switch (mode) {
        case DRIVE_MODE_DISABLED: return "DISABLED";
        case DRIVE_MODE_BLOCK: return "BLOCK";
        case DRIVE_MODE_SINUS: return "SINUS";
        case DRIVE_MODE_FOC: return "FOC";
        case DRIVE_MODE_BLOCK12: return "BLOCK12";
        default: return "???";
    }
}

/**
 * @brief Mapowanie Hallâ†’Hall dla odwrĂłconego kierunku (CCW).
 *
 * Zamiana ĹşrĂłdĹ‚a i ujĹ›cia w komutacji blokowej: 1â†”6, 3â†”4, 2â†”5.
 * WartoĹ›ci 0 i 7 (nieprawidĹ‚owe) bez zmian.
 */
static const DRAM_ATTR uint8_t g_hall_reverse_map[8] = {
    0,  // 0 = invalid â†’ invalid
    6,  // 1 (A+B-) â†’ 6 (B+A-)
    5,  // 2 (B+C-) â†’ 5 (C+B-)
    4,  // 3 (A+C-) â†’ 4 (C+A-)
    3,  // 4 (C+A-) â†’ 3 (A+C-)
    2,  // 5 (C+B-) â†’ 2 (B+C-)
    1,  // 6 (B+A-) â†’ 1 (A+B-)
    7   // 7 = invalid â†’ invalid
};

/**
 * @brief Mapowanie Hallâ†’sektor dla CCW (SINUS/FOC), bez zamiany faz.
 *
 * Wyprowadzenie z pierwszych zasad (poprawne):
 *   Bez zamiany Bâ†”C: kÄ…t pola = Î¸Â° - 90Â°
 *   CW empirycznie: Hall=001 wejĹ›cie przy wirniku=0Â°, Î¸=8 â†’ pole=300Â° = wirnik-60Â° â†’ moment CW
 *   CCW wejĹ›cie w Hall=001 nastÄ™puje przy wirniku=60Â° (nie 0Â°)!
 *   Dla momentu CCW: pole = wirnik+60Â° = 60Â°+60Â° = 120Â° â†’ Î¸=210Â°=56 wpisĂłw = sektor 3
 *   ReguĹ‚a: CCW_sector = (CW_sector + 3) mod 6  â€” pole odwrĂłcone o 180Â° = odwrĂłcony moment
 *   CW tabela: {0,2,1,4,5,3} dla hall 1-6
 *   CCW = CW+3 mod 6: {3,5,4,1,2,0} dla hall 1-6
 *   Sekwencja CCW: 001â†’101â†’100â†’110â†’010â†’011 â†’ sektory 3â†’2â†’1â†’0â†’5â†’4 (maleje -1 âś“)
 */
static const DRAM_ATTR int8_t g_hall_to_sector_ccw[8] = {
    -1,     // 0 = invalid
     3,     // 1 (001) â†’ sector 3  (Î¸ snap=56)
     5,     // 2 (010) â†’ sector 5  (Î¸ snap=88)
     4,     // 3 (011) â†’ sector 4  (Î¸ snap=72)
     1,     // 4 (100) â†’ sector 1  (Î¸ snap=24)
     2,     // 5 (101) â†’ sector 2  (Î¸ snap=40)
     0,     // 6 (110) â†’ sector 0  (Î¸ snap=8)
    -1      // 7 = invalid
};

/**
 * @brief Zwraca sektor (0-5) dla danego stanu Halla, z uwzglÄ™dnieniem kierunku.
 * W CCW uĹĽywa dedykowanej tabeli dla fizycznej sekwencji CCW rotora.
 */
static inline int8_t IRAM_ATTR hallToSector(uint8_t hall) {
    if (hall == 0 || hall == 7) return -1;
    return g_reverse_isr ? g_hall_to_sector_ccw[hall] : g_hall_to_sector[hall];
}

// StaĹ‚e sinusoidalne (identyczne z bldc_driver_v2)
#define SINE_TABLE_SIZE         96
#define SINE_TABLE_Q16_FULL     (96UL << 16)   // 6291456
#define SINE_SECTOR_ENTRIES     16              // 96 / 6
#define SINE_SECTOR_CENTER      8               // Ĺ›rodek sektora
// SINE_HALL_PHASE_OFFSET: teraz runtime variable g_sine_hall_phase_offset (komendy so+/so-/so:N)
// DomyĹ›lnie 0; strojenie: 1 wpis = 3.75Â° elektr.
// Offsety fazowe: dopasowane do tabeli komutacji blokowej CW (1â†’3â†’2â†’6â†’4â†’5)
// Faza A = referencyjna (peak w sektorach 0,1)
// Faza B = +240Â° (peak w sektorach 2,3)
// Faza C = +120Â° (peak w sektorach 4,5)
#define SINE_PHASE_A_OFFSET     0               // faza referencyjna
#define SINE_PHASE_B_OFFSET     64              // 96*2/3 = 240Â°
#define SINE_PHASE_C_OFFSET     32              // 96/3 = 120Â°
#define SINE_STARTUP_COMMUT     18              // przejĹ›Ä‡ Halla w BLOCK przed przejĹ›ciem na SINUS/FOC (3 obroty el.)
#define STARTUP_ALIGN_MS        150             // czas wyrĂłwnania rotora w BLOCK [ms]
#define STARTUP_ALIGN_DUTY_PCT  100             // duty wyrĂłwnania [% PWM_MAX_DUTY]
#define SINE_BLOCK_SPEED_THRESHOLD  10000       // Q16 speed poniĹĽej tego â†’ BLOCK zamiast SINUS/FOC (stall/oscylacja)
#define SINE_STALL_FREEZE_MS    200             // ms bez Halla â†’ zamroĹĽenie kÄ…ta
#define SINE_CRAWL_SPEED_Q16    315             // minimalna prÄ™dkoĹ›Ä‡ startowa â‰ 1 obr.elekt./s (52428800/166666)
#define SINE_STALL_FALLBACK_MS  400             // minimalny timeout fallback (histereza, anty-szarpanie)
#define SINE_START_MAX_HALL_US  30000           // max okres Halla (min prÄ™dkoĹ›Ä‡) do wejĹ›cia w SINUS
#define SINE_PHASE_CORR_SHIFT   0               // korekcja peĹ‚nego bĹ‚Ä™du na przejĹ›cie Halla
                                                // Hall jest wiarygodne â†’ nie filtruj o poĹ‚owÄ™
#define SINE_SPEED_CORR_ENABLE  1               // PLL: korekcja prÄ™dkoĹ›ci na przejĹ›ciu Halla
                                                // err>0 = kÄ…t za wolny â†’ zwiÄ™ksz prÄ™dkoĹ›Ä‡
                                                // err<0 = kÄ…t za szybki â†’ zmniejsz prÄ™dkoĹ›Ä‡
#define SINE_SPEED_FILTER_SHIFT 1               // filtr prÄ™dkoĹ›ci (unused, rate-limiter zamiast EMA)
#define SINE_SPEED_MAX_CHANGE_PCT 30            // max zmiana prÄ™dkoĹ›ci na Hall edge [%]

// Block crossfade: pĹ‚ynne przejĹ›cie PWM miÄ™dzy fazami przy komutacji
// Stara faza PWM wygasza duty do 0, nowa faza PWM rozjaĹ›nia od 0 do d
// Eliminuje skokowy przeskok prÄ…du (gĹ‚Ăłwne ĹşrĂłdĹ‚o terkotu blokowego)
#define BLOCK_CROSS_TICKS       8               // tiki crossfade (8Ă—50Âµs = 400Âµs)
static volatile uint8_t  g_block_cross_cnt = 0; ///< Tiki pozostaĹ‚e crossfade (0 = normalny tryb)
static volatile uint8_t  g_block_old_bh = 0;    ///< Remapowany Hall PRZED przejĹ›ciem

// Tabela stanĂłw faz per remapowany Hall: 0=Off, 1=Low, 2=PWM
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
 * @brief Resetuje tracker kÄ…ta SINUS/FOC do Ĺ›rodka aktualnego sektora Halla.
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
#define SINE_SNAP_THRESHOLD     (24 << 16)       // bĹ‚Ä…d > 1/4 obrotu elektr. â†’ peĹ‚ny snap kÄ…ta
#define SINE_SAFE_MAX_DUTY      (PWM_MAX_DUTY * 75 / 100)  // SVPWM: liniowy do 58%, overmod do 75%, powyĹĽej szkodliwe harmoniczne
#define SINE_MIN_AMPLITUDE      8               // poniĹĽej tego coast (~1.5% PWM_MAX_DUTY)

/// Debounce czujnikĂłw Halla: minimalna przerwa miÄ™dzy przejĹ›ciami [us].
/// W trybie SINUS 6 FETĂłw przekĹ‚Ä…dajÄ… jednoczeĹ›nie (center-aligned PWM),
/// generujÄ…c znacznie wiÄ™cej EMI niĹĽ BLOCK (2 FETy). Szumy sprzegajÄ… siÄ™
/// w linie Halla i tworzÄ… faĹ‚szywe przejĹ›cia (dt ~50us = 1 tick ISR).
/// Bez debounce: hall_period_us = 50us â†’ speed_q16 = 1M â†’ kÄ…t ucieka â†’ desync.
/// 200us = 4 ticki ISR, bezpieczne do ~50k eRPM (daleko poza realnym motorem).
#define HALL_MIN_PERIOD_US      200
// Przy starcie/postoju SINUS/FOC szpilki EMI potrafiÄ… wygenerowaÄ‡ szybkie pseudo-przejĹ›cia
// Hall, ktĂłre krÄ™cÄ… wektor pola mimo zablokowanego koĹ‚a. Podnosimy wtedy minimalny okres.
#define HALL_MIN_PERIOD_STARTUP_US 1000
// Reverse-step lock tylko przy niskim duty (manetka lekko otwarta), ĹĽeby nie blokowaÄ‡
// rzeczywistego cofania przy mocnym hamowaniu/odwrotnym momencie.
#define SINE_DIR_LOCK_DUTY_PCT  20
#define DEFAULT_P07_STANDALONE  90              // domyĹ›lne P07 gdy brak wyĹ›wietlacza (6 Ă— 15 par biegunĂłw)

// â”€â”€ FOC (Field Oriented Control) â”€â”€
// Architektura: pÄ™tla prÄ…dowa w loop() (~2kHz), modulacja SVPWM w ISR (20kHz).
// loop(): czyta prÄ…dy ADC â†’ Clarke â†’ Park â†’ PI(Id,Iq) â†’ zapisuje Vd,Vq (volatile)
// ISR: czyta Vd,Vq â†’ InvPark(Î¸) â†’ InvClarke â†’ SVPWM â†’ MCPWM
// KÄ…t Î¸: wspĂłĹ‚dzielony z SINUS (Hall tracking + interpolacja Q16)
#define FOC_KP_DEFAULT      0.5f    ///< DomyĹ›lne Kp regulatora PI d/q (PWM/A)
#define FOC_KI_DEFAULT      5.0f    ///< DomyĹ›lne Ki regulatora PI d/q (PWM/(AÂ·s))
#define FOC_INTEGRAL_LIMIT  ((float)SINE_SAFE_MAX_DUTY)  ///< Absolutny max integrala (anti-windup)
#define FOC_PI_CORR_LIMIT   100.0f  ///< Max korekta PI wokĂłĹ‚ feedforward [Â±PWM]
#define FOC_IQ_MAX          10.0f   ///< Maksymalny prÄ…d Iq target [A]
#define FOC_LOOP_DT         0.0005f ///< PrzybliĹĽony dt pÄ™tli prÄ…dowej [s] (~2kHz loop)
#define FOC_INV_SQRT3       0.57735026919f  ///< 1/âš3
#define FOC_SQRT3           1.73205080757f  ///< âš3
#define FOC_CURRENT_EMA_ALPHA  0.05f ///< EMA Î± dla prÄ…dĂłw FOC (~2kHz â†’ Ď„â‰10ms)
#define FOC_DQ_EMA_ALPHA       0.15f ///< EMA Î± dla Id/Iq po Park (Ď„â‰3ms @ 2kHz, DC signal)
                                     //   UĹ›rednia sporadyczne odczyty ADC (analogRead
                                     //   nie jest zsynchr. z PWM, INA180A2 widzi prÄ…d
                                     //   tylko gdy low-side ON â†’ ~50% odczytĂłw = 0)

/// Limit duty regen â€” 80% max (musi zostaÄ‡ czas OFF na transfer energii do baterii)
#define REGEN_MAX_DUTY  (PWM_MAX_DUTY * 80 / 100)
/// Minimalne RPM poniĹĽej ktĂłrego regen jest nieefektywny (tylko grzeje)
#define REGEN_MIN_RPM   50
/// NapiÄ™cie odciÄ™cia regen [V] â€” powyĹĽej tego progu regen wyĹ‚Ä…czony (ochrona baterii)
#define VBAT_REGEN_CUTOFF  42.0f
/// DomyĹ›lne duty regen (50% â€” umiarkowane hamowanie)
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
// Prototypy phase helpers (definicje poniĹĽej ISR)
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
// MCPWM inline helpers â€” ISR-safe, direct register writes
// ============================================================================
//
// Architektura: MCPWM_UNIT_0, 3 operatory (po jednym na fazÄ™), UP_DOWN counter.
// KaĹĽdy operator: gen_A â†’ HIN (high-side), gen_B â†’ LIN (low-side).
// Oba generatory sterowane niezaleĹĽnie (dead time bypass â€” IR2103 ma wewnÄ™trzny).
//
// Mapowanie:
//   Operator 0 = Faza A: gen_A â†’ GPIO32 (HIN_A), gen_B â†’ GPIO33 (LIN_A)
//   Operator 1 = Faza B: gen_A â†’ GPIO25 (HIN_B), gen_B â†’ GPIO26 (LIN_B)
//   Operator 2 = Faza C: gen_A â†’ GPIO27 (HIN_C), gen_B â†’ GPIO14 (LIN_C)
//
// IR2103: HIN=active HIGH (HIGH â†’ HS ON), LIN=active LOW (LOW â†’ LS ON)
// Kontrola za pomocÄ… bezpoĹ›rednich zapisĂłw do rejestrĂłw gen_force i generator action (ISR-safe).
//
// Sekwencja stanĂłw:
//   PWM:   gen_A = PWM(duty), gen_B = forced HIGH (LIN=HIGH â†’ LS OFF)
//   GND:   gen_A = forced LOW (HS OFF), gen_B = forced LOW (LIN=LOW â†’ LS ON)
//   OFF:   gen_A = forced LOW (HS OFF), gen_B = forced HIGH (LIN=HIGH â†’ LS OFF)
//   SINUS/FOC: gen_A = gen_B = PWM(same_duty) â€” IR2103 robi complementary + dead time
//   REGEN: gen_A = forced LOW (HS OFF), gen_B = PWM(inverted_duty: 0% = LS ON, 100% = LS OFF)

// â”€â”€ ISR-safe MCPWM register helpers (bezpoĹ›redni zapis do rejestrĂłw, ~5ns) â”€â”€
//
// Wszystkie operacje poniĹĽej omijajÄ… driver API (mcpwm_set_duty_type,
// mcpwm_set_signal_high/low) ktĂłre uĹĽywajÄ… spinlockĂłw i nie sÄ… ISR-safe.
// Zamiast tego operujemy bezpoĹ›rednio na rejestrach MCPWM0.

// â”€â”€ Precomputed gen_force.val values â”€â”€
// gen_cntuforce_upmethod (bits [5:0]) = 0 â†’ update immediately
// gen_a_cntuforce_mode (bits [7:6]): 0=disabled(PWM), 1=forceLOW, 2=forceHIGH
// gen_b_cntuforce_mode (bits [9:8]): 0=disabled(PWM), 1=forceLOW, 2=forceHIGH
#define MCPWM_FORCE_PWM     0x0200  // gen_A=PWM(0), gen_B=forceHIGH(2) â†’ LS OFF
#define MCPWM_FORCE_GND     0x0140  // gen_A=forceLOW(1), gen_B=forceLOW(1) â†’ LS ON
#define MCPWM_FORCE_OFF     0x0240  // gen_A=forceLOW(1), gen_B=forceHIGH(2) â†’ float
#define MCPWM_FORCE_COMPL   0x0000  // gen_A=PWM(0), gen_B=PWM(0) â†’ complementary
#define MCPWM_FORCE_REGEN   0x0040  // gen_A=forceLOW(1), gen_B=PWM(0) â†’ regen

// â”€â”€ Precomputed generator action register values â”€â”€
// Dla UP_DOWN counter (center-aligned PWM):
//   utea/uteb = action on compare match, counting UP
//   dtea/dteb = action on compare match, counting DOWN
//   action: 0=no change, 1=low, 2=high, 3=toggle
//
// UP_DOWN: czas HIGH = 2Ă—C, okres = 2Ă—P â†’ duty = C/P
// Aby duty=0â†’0% i duty=Pâ†’100% (jak LEDC), potrzebujemy:
//   counting UP + compare match â†’ LOW  (koniec fazy HIGH)
//   counting DOWN + compare match â†’ HIGH (poczÄ…tek fazy HIGH)
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
 * @brief Ustawia fazÄ™ na PWM (high-side modulowany, low-side OFF).
 * gen_A = PWM(duty, MODE_0), gen_B = forced HIGH (LIN=HIGH â†’ LS OFF)
 * ISR-safe: bezpoĹ›redni zapis do rejestrĂłw, bez spinlockĂłw.
 */
static inline void IRAM_ATTR mcpwm_phase_pwm(int op, uint16_t duty) {
    MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
    mcpwm_set_compare_fast(op, 0, duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_PWM;
}

/**
 * @brief Ustawia fazÄ™ na GND (high-side OFF, low-side ON).
 * gen_A = forced LOW (HS OFF), gen_B = forced LOW (LIN=LOW â†’ LS ON)
 */
static inline void IRAM_ATTR mcpwm_phase_gnd(int op) {
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_GND;
}

/**
 * @brief Ustawia fazÄ™ na float (oba OFF).
 * gen_A = forced LOW (HS OFF), gen_B = forced HIGH (LIN=HIGH â†’ LS OFF)
 */
static inline void IRAM_ATTR mcpwm_phase_off(int op) {
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_OFF;
}

/**
 * @brief Ustawia fazÄ™ na SINUS/FOC (oba generatory = ten sam duty, komplementarny przez IR2103).
 * gen_A = PWM(duty, MODE_0), gen_B = PWM(duty, MODE_0) â€” IR2103 tworzy komplementarne + dead time.
 * ISR-safe: bezpoĹ›redni zapis do rejestrĂłw.
 */
static inline void IRAM_ATTR mcpwm_phase_complementary(int op, uint32_t duty) {
    MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
    MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE0;
    mcpwm_set_compare_fast(op, 0, duty);
    mcpwm_set_compare_fast(op, 1, duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_COMPL;
}

/**
 * @brief Ustawia fazÄ™ na regen PWM (HS OFF, LS modulowany).
 * gen_A = forced LOW (HS OFF)
 * gen_B = PWM inverted (MODE_1): duty=0â†’LS ON, duty=MAXâ†’LS OFF
 * Przy PWM OFF: prÄ…d indukcyjny przez body diodÄ™ HS â†’ Vbat (regeneracja)
 * ISR-safe: bezpoĹ›redni zapis do rejestrĂłw.
 */
static inline void IRAM_ATTR mcpwm_phase_regen(int op, uint16_t regen_duty) {
    MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE1;
    mcpwm_set_compare_fast(op, 1, regen_duty);
    MCPWM0.operators[op].gen_force.val = MCPWM_FORCE_REGEN;
}

static inline void IRAM_ATTR applyBlockState(uint8_t bh, uint16_t duty) {
    switch (bh) {
        case 1: phaseA_PWM(duty); phaseB_Low(); phaseC_Off(); break;
        case 3: phaseA_PWM(duty); phaseB_Off(); phaseC_Low(); break;
        case 2: phaseA_Off(); phaseB_PWM(duty); phaseC_Low(); break;
        case 6: phaseA_Low(); phaseB_PWM(duty); phaseC_Off(); break;
        case 4: phaseA_Low(); phaseB_Off(); phaseC_PWM(duty); break;
        case 5: phaseA_Off(); phaseB_Low(); phaseC_PWM(duty); break;
        default: allMosfetsOff(); break;
    }

    switch (bh) {
        case 1:
        case 3:
            g_dbg_last_da = duty; g_dbg_last_db = 0;    g_dbg_last_dc = 0;    break;
        case 2:
        case 6:
            g_dbg_last_da = 0;    g_dbg_last_db = duty; g_dbg_last_dc = 0;    break;
        case 4:
        case 5:
            g_dbg_last_da = 0;    g_dbg_last_db = 0;    g_dbg_last_dc = duty; break;
        default:
            g_dbg_last_da = 0;    g_dbg_last_db = 0;    g_dbg_last_dc = 0;    break;
    }
}

// Dzielnik napiÄ™cia VBAT: 1M (gĂłra) / 33k (dĂłĹ‚)
static const float kVbatRTop = 1130000.0f;
static const float kVbatRBottom = 31700.0f;
static const float kVbatDividerGain = (kVbatRTop + kVbatRBottom) / kVbatRBottom;

// Pomiar prÄ…du: shunt 2 mOhm + INA180A2 (gain 50 V/V)
static const float kShuntOhms = 0.002f;
static const float kInaGain = 50.0f;
static const float kCurrentScale = 1.0f / (kShuntOhms * kInaGain);
static const float kCurrentOffsetAlpha = 0.02f;  // filtr do autokalibracji zera
static float g_currentOffsetV[3] = {0.0f, 0.0f, 0.0f};

// Krok zmiany duty dla komend +/-
static const uint16_t DUTY_STEP = PWM_MAX_DUTY / 20;  // 5% kroku

// Przepustnica - prĂłg martwej strefy i zakres
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

// Symulacja hamulca komendÄ… Serial
static bool g_brake_simulated = false;
static uint8_t g_brake_debounce_count = 0;   ///< Licznik debounce hamulca
#define BRAKE_DEBOUNCE_THRESHOLD  5           ///< Ile kolejnych LOW wymagane (~2.5ms przy 2kHz loop)

// Rampa rozpÄ™dzania silnika
static uint16_t g_duty_ramped = 0;                   ///< Aktualny duty po rampie
static unsigned long g_ramp_last_us = 0;             ///< Timestamp ostatniego kroku rampy [Âµs]
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
static int8_t   g_atune_offset_min   = -24;      ///< PoczÄ…tek zakresu sweep
static int8_t   g_atune_offset_max   = 24;       ///< Koniec zakresu sweep
static int8_t   g_atune_offset_step  = 2;        ///< Krok sweep (wpisy tabeli)
static int8_t   g_atune_current_ofs  = 0;        ///< Aktualnie testowany offset
static int8_t   g_atune_best_ofs     = 0;        ///< Najlepszy offset (min prÄ…d)
static float    g_atune_best_current = 1e9f;     ///< NajniĹĽszy Ĺ›redni prÄ…d [A]
static float    g_atune_sum_current  = 0.0f;     ///< Akumulator prÄ…du w fazie MEASURE
static uint32_t g_atune_sample_count = 0;        ///< Liczba prĂłbek w fazie MEASURE
static unsigned long g_atune_phase_start_ms = 0; ///< millis() startu aktualnej fazy
static int8_t   g_atune_saved_offset = 0;        ///< Zapisany offset przed auto-tune
static uint16_t g_atune_test_duty    = 0;        ///< Duty testowe (10% domyĹ›lnie)
static const unsigned long ATUNE_SETTLE_MS  = 400;  ///< Czas stabilizacji [ms]
static const unsigned long ATUNE_MEASURE_MS = 600;  ///< Czas pomiaru [ms]

// PAS Auto-tune (komenda 'pasat')
// Zbiera statystyki sygnaĹ‚u PAS przez PASAT_MEASURE_S sekund,
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
// Kalibracja: uĹĽytkownik krÄ™ci koĹ‚em 3 peĹ‚ne obroty w 15 s,
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

// WyĹ›wietlacz S866 (zawsze aktywny na Serial2)
static s866_display_t g_display;
static inline uint8_t getEffectiveAssistRaw() {
    if (g_web_assist_override >= 0) {
        return (uint8_t)g_web_assist_override;
    }
    if (g_display.connected) {
        return g_display.rx.assist_level;
    }
    if (config_get().display_required) {
        return 0;
    }
    return 15;
}

// â”€â”€ Serwer WWW konfiguracji WiFi (aktywny gdy P17=1) â”€â”€
static WebServer*  g_web_server     = nullptr;  ///< Instancja serwera HTTP (nullptr gdy wy\u0142\u0105czony)
static DNSServer*  g_dns_server     = nullptr;  ///< DNS captive-portal (wszystkie domeny -> 192.168.4.1)
static String      g_web_queued_cmd = "";        ///< Komenda silnika do wykonania po wy\u0142\u0105czeniu WiFi
static bool        g_wifi_active    = false;     ///< Flaga: WiFi AP aktywne

// ============================================================================
// Algorytm przepustnicy â€” wspĂłlny dla BLOCK / SINUS / FOC
// ============================================================================

/**
 * @brief Oblicza max duty na podstawie poziomu wspomagania (assist level).
 *
 * Algorytm wspĂłlny dla wszystkich trybĂłw sterowania (BLOCK, SINUS, FOC).
 * Przepustnica mapuje zakres RAW bezpoĹ›rednio na 0â€“maxDuty (proporcjonalnie).
 *
 * @return Maksymalne duty 0â€“PWM_MAX_DUTY:
 *   - WyĹ›wietlacz podĹ‚Ä…czony, level>0: proporcjonalnie 20/40/60/80/100%
 *   - WyĹ›wietlacz podĹ‚Ä…czony, level=0:  0 (silnik wyĹ‚Ä…czony)
 *   - WyĹ›wietlacz nie podĹ‚Ä…czony:       PWM_MAX_DUTY (tryb standalone)
 */
static uint16_t getAssistMaxDuty() {
    uint8_t assist_raw = getEffectiveAssistRaw();
    if (!g_display.connected && g_web_assist_override < 0) {
        if (config_get().display_required) {
            return 0;  // wyĹ›wietlacz wymagany ale niepodĹ‚Ä…czony â†’ silnik wyĹ‚Ä…czony
        }
        return PWM_MAX_DUTY;  // brak wyĹ›wietlacza â†’ peĹ‚na moc (standalone)
    }
    if (assist_raw == 0) {
        return 0;  // assist level 0 â†’ silnik wyĹ‚Ä…czony
    }
    // Raw assist_level z wyĹ›wietlacza S866 jest zawsze 0-15 (bajt 4, bity 0-3),
    // niezaleĹĽnie od ustawienia P05 (ktĂłre definiuje ile krokĂłw widzi uĹĽytkownik).
    uint16_t maxDuty = (uint16_t)((uint32_t)assist_raw * PWM_MAX_DUTY / 15);
    if (maxDuty > PWM_MAX_DUTY) maxDuty = PWM_MAX_DUTY;
    return maxDuty;
}

/**
 * @brief Pobiera limit prÄ™dkoĹ›ci [km/h] z parametru P08.
 * @return Limit prÄ™dkoĹ›ci w km/h. 0 = brak limitu (wyĹ‚Ä…czony).
 */
static uint8_t getSpeedLimitKmh() {
    return g_display.config.p08_speed_limit;
}

/**
 * @brief Efektywny limit prÄ…du fazowego [A].
 *
 * Priorytet: P14 z wyĹ›wietlacza (jeĹ›li podĹ‚Ä…czony i P14>0),
 * w przeciwnym razie current_limit_a z NVS.
 * WartoĹ›Ä‡ 0 oznacza brak limitu (bypass).
 *
 * @return Limit prÄ…du [A], 0 = brak limitu.
 */
static uint8_t getEffectiveCurrentLimit() {
    if (g_display.connected && g_display.config.p14_current_limit_a > 0) {
        return g_display.config.p14_current_limit_a;
    }
    return config_get().current_limit_a;
}

/**
 * @brief Globalny limit prÄ™dkoĹ›ci z power fade (filtrowany EMA).
 *
 * Redukuje duty_target w miarÄ™ zbliĹĽania siÄ™ do prÄ™dkoĹ›ci maksymalnej (P08).
 * Strefa fade: 70%..100% limitu. PowyĹĽej limitu: duty = 0 (coast).
 *
 * WspĂłĹ‚czynnik mocy (0.0..1.0) jest filtrowany filtrem EMA aby uniknÄ…Ä‡
 * oscylacji duty spowodowanych zaszumionÄ… prÄ™dkoĹ›ciÄ… koĹ‚a (1 impuls/obrĂłt).
 * Bez filtra: speed bounces around limit â†’ duty bounces â†’ speed bounces â†’ ...
 *
 * Dotyczy WSZYSTKICH trybĂłw (BLOCK, SINUS, FOC) i ĹşrĂłdeĹ‚ duty (manetka, PAS).
 *
 * @param duty_in  Duty przed limitowaniem.
 * @param speed_kmh Aktualna prÄ™dkoĹ›Ä‡ koĹ‚a [km/h].
 * @return Duty po limitowaniu.
 */
static float g_speed_limit_factor = 1.0f;           ///< EMA-filtrowany wspĂłĹ‚czynnik mocy 0..1
#define SPEED_LIMIT_ALPHA_DOWN    0.03f              ///< EMA Î± spadek (over-speed â†’ szybka redukcja duty)
#define SPEED_LIMIT_ALPHA_UP      0.02f              ///< EMA Î± wzrost (under-speed â†’ narastanie duty)
#define SPEED_LIMIT_FADE_BAND_KMH 3.0f              ///< Strefa liniowego fade przed limitem [km/h]

static uint16_t applyGlobalSpeedLimit(uint16_t duty_in, float speed_kmh) {
    if (duty_in == 0) {
        // Przy zerowym duty nie modyfikuj filtra â€” silnik nie jedzie
        return 0;
    }
    uint8_t limit_kmh = getSpeedLimitKmh();
    if (limit_kmh == 0) {
        // P08==0: brak limitu prÄ™dkoĹ›ci â€” peĹ‚na moc bez ograniczeĹ„
        g_speed_limit_factor = 1.0f;
        return duty_in;
    }
    float limit_f = (float)limit_kmh;

    // Oblicz surowy wspĂłĹ‚czynnik mocy (0.0 .. 1.0)
    // Fade zaczyna siÄ™ SPEED_LIMIT_FADE_BAND_KMH km/h przed limitem (staĹ‚e okno, niezaleĹĽne od limitu)
    float raw_factor;
    if (speed_kmh >= limit_f) {
        raw_factor = 0.0f;  // powyĹĽej limitu â†’ zero mocy
    } else {
        float fade_start = limit_f - SPEED_LIMIT_FADE_BAND_KMH;
        if (fade_start < 0.0f) fade_start = 0.0f;
        if (speed_kmh <= fade_start) {
            raw_factor = 1.0f;  // poniĹĽej strefy fade â†’ peĹ‚na moc
        } else {
            // Liniowy spadek 1.0â†’0.0 w strefie fade (ostatnie 3 km/h przed limitem)
            raw_factor = (limit_f - speed_kmh) / SPEED_LIMIT_FADE_BAND_KMH;
        }
    }

    // Asymetryczny filtr EMA:
    // - Over-speed (raw < sl): szybka redukcja duty â†’ zapobiega przekroczeniu limitu
    // - Under-speed (raw > sl): wolne narastanie â†’ zapobiega oscylacji bang-bang
    float alpha = (raw_factor < g_speed_limit_factor) ? SPEED_LIMIT_ALPHA_DOWN : SPEED_LIMIT_ALPHA_UP;
    g_speed_limit_factor += alpha * (raw_factor - g_speed_limit_factor);

    // Clamp do 0..1 (bezpieczeĹ„stwo numeryczne)
    if (g_speed_limit_factor < 0.0f) g_speed_limit_factor = 0.0f;
    if (g_speed_limit_factor > 1.0f) g_speed_limit_factor = 1.0f;

    // Twardy clamp: jeĹ›li prÄ™dkoĹ›Ä‡ > limit+2km/h â†’ natychmiast zero (bezpieczeĹ„stwo)
    if (speed_kmh > limit_f + 2.0f) {
        g_speed_limit_factor = 0.0f;
    }

    uint16_t result = (uint16_t)((float)duty_in * g_speed_limit_factor);
    return result;
}

/**
 * @brief Mapuje wartoĹ›Ä‡ RAW przepustnicy na duty cycle 0â€“maxDuty.
 *
 * PeĹ‚en zakres przepustnicy (THROTTLE_MIN_RAWâ€“THROTTLE_MAX_RAW) jest mapowany
 * proporcjonalnie na 0â€“maxDuty. DziÄ™ki temu zmiana poziomu wspomagania
 * zmienia zakres wyjĹ›ciowy, a nie obcina go (lepsza rozdzielczoĹ›Ä‡ sterowania).
 *
 * @param throttle_raw Surowa wartoĹ›Ä‡ ADC przepustnicy.
 * @param maxDuty      Maksymalne duty z getAssistMaxDuty().
 * @return duty 0â€“maxDuty
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
 * @brief Oblicza kadencjÄ™ pedaĹ‚owania na podstawie impulsĂłw z czujnika PAS.
 *
 * Kadencja [RPM] = 60 000 000 / (period_us Ă— P13_magnets).
 * Timeout: jeĹ›li > PAS_TIMEOUT_US od ostatniego impulsu â†’ kadencja = 0.
 *
 * @return Kadencja w RPM (0 = nie pedaĹ‚uje / timeout)
 */
static uint16_t calculatePasCadence() {
    uint32_t period = g_pas_period_us;       // volatile â†’ local copy
    uint32_t last   = g_pas_last_pulse_us;
    uint32_t now_us = (uint32_t)esp_timer_get_time();

    // Timeout: brak impulsu â†’ nie pedaĹ‚uje
    uint32_t stop_us = (uint32_t)config_get().pas_stop_delay_ms * 1000UL;
    if (last == 0 || period == 0 || (now_us - last) > stop_us) {
        return 0;
    }

    // Liczba magnesĂłw PAS (z wyĹ›wietlacza P13, domyĹ›lnie 12)
    uint8_t magnets = g_display.config.p13_pas_magnets;
    if (magnets == 0) magnets = 12;  // fallback

    // cadence_rpm = 60_000_000 / (period_us Ă— magnets)
    uint32_t cadence = 60000000UL / (period * (uint32_t)magnets);
    if (cadence > 150) cadence = 150;

    return (uint16_t)cadence;
}

// === Stan PAS ===
/// Timestamp [ms] od kiedy PAS krÄ™ci siÄ™ w kierunku forward (0 = nie krÄ™ci)
static uint32_t g_pas_fwd_since_ms = 0;
/// Timestamp [ms] momentu aktywacji PAS (przejĹ›cia z WAIT do ON) â€” soft-start
static uint32_t g_pas_active_since_ms = 0;
/// Flaga: PAS aktywnie wspomaga (przeszedĹ‚ start delay i krÄ™ci siÄ™ forward)
static bool g_pas_pedaling = false;
/// Poprzednie duty PAS â€” do slew rate limiter
static uint16_t g_pas_prev_duty = 0;
/// Timestamp [ms] ostatniego widzianego g_pas_forward==true â€” holdoff reverse
static uint32_t g_pas_last_fwd_ms = 0;
/// WygĹ‚adzona prÄ™dkoĹ›Ä‡ docelowa PAS â€” zapobiega szarpniÄ™ciom przy zmianie poziomu
static float g_pas_vtarget_smooth = 0.0f;

// --- Bufor Ĺ›redniej kroczÄ…cej prÄ™dkoĹ›ci koĹ‚a (Moving Average) ---
#define PAS_SPEED_MA_SIZE  8
static float g_pas_speed_ma_buf[PAS_SPEED_MA_SIZE] = {0};
static uint8_t g_pas_speed_ma_idx = 0;
static bool g_pas_speed_ma_full = false;

/// Maksymalna zmiana duty PAS na jedno wywoĹ‚anie (slew rate)
/// DomyĹ›lnie 30 (~6% PWM_MAX_DUTY). Nadpisywany z NVS (pas_slew_rate).
static uint16_t g_pas_slew_rate_max = 30;
/// Czas podtrzymania stanu forward po krĂłtkim "reverse" [ms]
/// DomyĹ›lnie 300. Nadpisywany z NVS (pas_fwd_holdoff_ms).
static uint16_t g_pas_fwd_holdoff_ms = 300;

/**
 * @brief Oblicza prÄ™dkoĹ›Ä‡ koĹ‚a [km/h] z wheeltime_ms i rozmiaru koĹ‚a P06.
 *
 * FormuĹ‚a: speed_kmh = (obwĂłd_koĹ‚a_m Ă— 3600000) / (wheeltime_ms Ă— 1000)
 * obwĂłd = P06_inch_x10 / 10 Ă— 0.0254 Ă— Ď€  [m]
 * Uproszczenie: speed_kmh = P06 Ă— 0.028727 / wheeltime_ms  (staĹ‚a = 0.0254Ă—Ď€Ă—3600/10)
 *
 * @return PrÄ™dkoĹ›Ä‡ w km/h (0.0 jeĹ›li stoi)
 */
static float calculateWheelSpeedKmh() {
    uint16_t wt_ms = g_bldc_state.wheeltime_ms;
    if (wt_ms == 0) return 0.0f;

    uint16_t p06 = g_display.config.p06_wheel_size_x10;
    if (p06 == 0) p06 = 260;  // fallback: 26" koĹ‚o

    // ObwĂłd koĹ‚a [m] = (P06/10) Ă— 0.0254 Ă— Ď€ = P06 Ă— 0.007980
    // v [km/h] = obwĂłd [m] Ă— 3600000 / (wt_ms Ă— 1000) = P06 Ă— 28.727 / wt_ms
    return (float)p06 * 28.727f / (float)wt_ms;
}

/**
 * @brief Oblicza duty PAS z V_target, soft-start, speed ramp-down, slew rate i MA.
 *
 * Algorytm:
 *   1. assist_level == 0 â†’ duty = 0
 *   2. Brak impulsĂłw PAS przez pas_stop_delay_ms â†’ duty = 0
 *   3. Kierunek reverse (z holdoff 300ms) â†’ duty = 0
 *   4. Forward < pas_start_delay_ms â†’ duty = 0
 *   5. Forward >= pas_start_delay_ms â†’ ACTIVE:
 *      a. soft_start: moc narasta 0â†’100% w czasie pas_ramp_ms
 *      b. V_target = 6 + Lx * (v_max - 6) / 15
 *      c. PrÄ™dkoĹ›Ä‡ wygĹ‚adzona Ĺ›redniÄ… kroczÄ…cÄ… (8 prĂłbek)
 *      d. v_smooth < v_target-3 â†’ peĹ‚na moc
 *      e. v_smooth w [v_target-3, v_target] â†’ redukcja liniowa 100%â†’0%
 *      f. v_smooth >= v_target â†’ duty = 0
 *   6. Slew rate limit: duty zmienia siÄ™ max Â±PAS_SLEW_RATE_MAX na wywoĹ‚anie
 *   7. Globalny limit P08 stosowany osobno przez applyGlobalSpeedLimit().
 *
 * @param maxDuty  UĹĽywane do detekcji assist=0 (motor OFF)
 * @return duty PAS 0â€“PWM_MAX_DUTY
 */
static uint16_t calculatePasDuty(uint16_t maxDuty) {
    // Assist level 0 â†’ PAS wyĹ‚Ä…czony
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
    uint32_t last_fwd_pulse = g_pas_last_fwd_pulse_us;  // snapshot volatile â€” ostatni FORWARD impuls z ISR

    // Parametry z EEPROM
    controller_config_t& cfg = config_get();
    uint32_t stop_delay_us = (uint32_t)cfg.pas_stop_delay_ms * 1000UL;
    uint32_t start_delay_ms = (uint32_t)cfg.pas_start_delay_ms;
    uint32_t ramp_ms = (uint32_t)cfg.pas_ramp_ms;

    // --- Timeout: brak PAS FORWARD przez stop_delay â†’ zatrzymaj wspomaganie ---
    // g_pas_last_fwd_pulse_us jest odĹ›wieĹĽane w ISR tylko przy g_pas_forward==true.
    // PAS stoi  â†’ ISR nie odpala       â†’ timestamp starzeje siÄ™ â†’ timed_out âś“
    // PAS wstecz â†’ ISR odpala, fwd=false â†’ timestamp nie odĹ›wieĹĽany â†’ timed_out âś“
    // PAS forward â†’ ISR odpala, fwd=true  â†’ timestamp odĹ›wieĹĽany  â†’ timed_out âś—
    //
    // Stale dane HIGH/LOW po dĹ‚ugiej przerwie: ISR sam resetuje pomiary
    // gdy wykryje przerwÄ™ >2s (patrz poczÄ…tek onPasPulse).
    //
    // period_too_long: magnesy zatrzymane generujÄ… sporadyczne szpilki,
    // ktĂłre odnawiajÄ… last_pulse. Zmierzony okres H+L >> stop_delay â†’ wykrycie.
    uint32_t period_us = g_pas_period_us;
    bool timed_out = (last_fwd_pulse == 0) || ((now_us - last_fwd_pulse) >= stop_delay_us);
    bool period_too_long = (period_us > 0) && (period_us > stop_delay_us);
    if (timed_out || period_too_long) {
        g_pas_pedaling = false;
        g_pas_fwd_since_ms = 0;
        g_pas_active_since_ms = 0;
        g_pas_vtarget_smooth = 0.0f;
        // Slew rate w dĂłĹ‚: Ĺ‚agodne wygaszenie zamiast twardego 0
        if (g_pas_prev_duty > g_pas_slew_rate_max) {
            g_pas_prev_duty -= g_pas_slew_rate_max;
            return g_pas_prev_duty;
        }
        g_pas_prev_duty = 0;
        return 0;
    }

    // --- Kierunek: reverse z holdoff ---
    // g_pas_last_fwd_ms: timestamp ostatniego forward â€” dla pomiaru holdoff.
    // Aktualizowany tutaj (po bloku timed_out) â€” dziaĹ‚a poprawnie, bo timed_out juĹĽ sprawdzony.
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

    // Jeszcze w start delay â€” nie wspomagaj
    uint32_t fwd_duration_ms = now_ms - g_pas_fwd_since_ms;
    if (fwd_duration_ms < start_delay_ms) {
        return 0;
    }

    // === ACTIVE: pedaĹ‚owanie potwierdzone ===
    if (!g_pas_pedaling) {
        g_pas_pedaling = true;
        g_pas_active_since_ms = now_ms;  // start soft-start
    }

    // --- Soft-start: moc narasta 0â†’100% w czasie ramp_ms ---
    float soft_start_factor = 1.0f;
    if (ramp_ms > 0 && g_pas_active_since_ms > 0) {
        uint32_t active_ms = now_ms - g_pas_active_since_ms;
        if (active_ms < ramp_ms) {
            soft_start_factor = (float)active_ms / (float)ramp_ms;
        }
    }

    // --- V_target z poziomu wspomagania (z wygĹ‚adzeniem) ---
    uint8_t raw_assist = getEffectiveAssistRaw();       // 0..15 (display lub override WWW)
    uint8_t v_max = g_display.config.p08_speed_limit;
    // P08==0: brak limitu prÄ™dkoĹ›ci â†’ PAS zawsze peĹ‚na moc (speed_factor=1)
    bool pas_speed_unlimited = (v_max == 0);
    if (v_max == 0) v_max = 25;  // fallback dla obliczeĹ„ v_target (nie ogranicza)
    float v_target_raw = 6.0f + (float)raw_assist * ((float)v_max - 6.0f) / 15.0f;

    // Smooth V_target: zapobiega szarpniÄ™ciom przy zmianie assist level w trakcie jazdy.
    // Slew rate ~10 km/h/s (0.005 km/h na wywoĹ‚anie przy ~2kHz loop).
    // Zmiana L5â†’L3 (25â†’17.4 km/h) trwa ~0.76s zamiast natychmiast.
    if (g_pas_vtarget_smooth <= 0.0f) {
        g_pas_vtarget_smooth = v_target_raw;  // pierwszy start: snap
    } else {
        const float VTARGET_SLEW = 0.005f;  // km/h na wywoĹ‚anie (~10 km/h/s)
        float vdiff = v_target_raw - g_pas_vtarget_smooth;
        if (vdiff > VTARGET_SLEW) vdiff = VTARGET_SLEW;
        if (vdiff < -VTARGET_SLEW) vdiff = -VTARGET_SLEW;
        g_pas_vtarget_smooth += vdiff;
    }
    float v_target = g_pas_vtarget_smooth;

    // --- Ĺšrednia kroczÄ…ca prÄ™dkoĹ›ci (Moving Average, 8 prĂłbek) ---
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
        speed_factor = 1.0f;  // P08==0: brak limitu â†’ zawsze peĹ‚na moc
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

    // --- Slew rate limiter: max Â±g_pas_slew_rate_max na wywoĹ‚anie ---
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
// ISR czujnika prÄ™dkoĹ›ci (GPIO, pin SPEED)
// ============================================================================

/**
 * @brief ISR przerwania GPIO na pinie SPEED (FALLING edge).
 *
 * Dla silnikĂłw przekĹ‚adniowych (P07==1): jeden magnes na kole generuje
 * jeden impuls na obrĂłt. Mierzymy czas miÄ™dzy impulsami.
 *
 * @note UĹĽywany tylko gdy P07 <= 1 (czujnik zewnÄ™trzny).
 */
void IRAM_ATTR onSpeedPulse() {
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (g_speed_last_pulse_us > 0) {
        uint32_t dt = now_us - g_speed_last_pulse_us;
        if (dt < SPEED_DEBOUNCE_US) {
            g_speed_reject_count++;
            return;  // bounce â€” za krĂłtki interwaĹ‚
        }
        g_speed_period_us = dt;
        g_speed_pulse_count++;
    }
    g_speed_last_pulse_us = now_us;
}

/**
 * @brief ISR przerwania GPIO na pinie PAS (oba zbocza â€” CHANGE).
 *
 * Detekcja kierunku pedaĹ‚owania:
 * WiÄ™kszoĹ›Ä‡ czujnikĂłw PAS ma asymetryczny dysk magnesĂłw â€” magnesy N i S
 * majÄ… rĂłĹĽnÄ… szerokoĹ›Ä‡. Efekt: sygnaĹ‚ z Halla ma rĂłĹĽne czasy HIGH i LOW.
 * Przy pedaĹ‚owaniu DO PRZODU: HIGH > LOW (lub odwrotnie, zaleĹĽy od montaĹĽu).
 * Przy pedaĹ‚owaniu DO TYĹU: proporcje siÄ™ odwracajÄ….
 *
 * Mierzymy oba pĂłĹ‚okresy (HIGH time i LOW time) i porĂłwnujemy:
 *   HIGH > LOW â†’ forward (pedaĹ‚owanie do przodu)
 *   LOW > HIGH â†’ backward (wstecz)
 *
 * JeĹ›li magnesy sÄ… symetryczne (rĂłĹĽnica < PAS_DIR_MIN_ASYMMETRY%),
 * zakĹ‚adamy forward â€” nie moĹĽna jednoznacznie okreĹ›liÄ‡ kierunku.
 */
/**
 * @brief Przetwarzanie krawÄ™dzi PAS po przejĹ›ciu filtra cyfrowego.
 *
 * WywoĹ‚ywane z timera prĂłbkujÄ…cego gdy przefiltrowany stan pinu siÄ™ zmieniĹ‚.
 * Zawiera logikÄ™ detekcji kierunku (asymetria HIGH/LOW) i aktualizacjÄ™ timestampĂłw.
 */
static void IRAM_ATTR processPasFilteredEdge(bool pin_high, uint32_t now_us) {
    // --- Detekcja dĹ‚ugiej przerwy â†’ reset pomiarĂłw HIGH/LOW ---
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
        // RISING edge â€” koniec okresu LOW
        if (g_pas_falling_us > 0) {
            uint32_t low_time = now_us - g_pas_falling_us;
            if (low_time >= g_pas_min_halfperiod_us) {
                g_pas_low_time_us = low_time;
            }
        }
        g_pas_rising_us = now_us;
    } else {
        // FALLING edge â€” koniec okresu HIGH
        if (g_pas_rising_us > 0) {
            uint32_t high_time = now_us - g_pas_rising_us;
            if (high_time >= g_pas_min_halfperiod_us) {
                g_pas_high_time_us = high_time;
            }
        }
        g_pas_falling_us = now_us;

        // Oba pĂłĹ‚okresy zmierzone â†’ oblicz okres i kierunek
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
 * @brief Timer ISR: prĂłbkowanie pinu PAS co PAS_SAMPLE_INTERVAL_US (500Âµs = 2kHz).
 *
 * Zamiast reagowaÄ‡ na kaĹĽde zbocze (w tym szpilki EMI), prĂłbkujemy pin
 * z ustalonÄ… czÄ™stotliwoĹ›ciÄ… i wymagamy N kolejnych zgodnych prĂłbek
 * przed uznaniem zmiany stanu. Szpilki EMI (<1ms) nie utrzymajÄ… siÄ™
 * przez NĂ—500Âµs i zostanÄ… odfiltrowane.
 *
 * g_pas_filter_depth = pas_debounce_us / PAS_SAMPLE_INTERVAL_US
 * Np. debounce=3000Âµs â†’ depth=6 â†’ zmiana stanu wymaga 3ms ciÄ…gĹ‚ego sygnaĹ‚u.
 */
void IRAM_ATTR onPasSampleTimer(void* arg) {
    bool raw = (GPIO.in >> PIN_PAS) & 1;

    if (raw == g_pas_filtered_state) {
        // PrĂłbka zgodna z aktualnym stanem â€” reset licznika
        g_pas_filter_count = 0;
        return;
    }

    // PrĂłbka rĂłĹĽni siÄ™ od przefiltrowanego stanu
    if (++g_pas_filter_count >= g_pas_filter_depth) {
        // N kolejnych prĂłbek potwierdziĹ‚o nowy stan â†’ akceptuj zmianÄ™
        g_pas_filter_count = 0;
        g_pas_filtered_state = raw;

        // Przefiltrowana krawÄ™dĹş â€” przetwĂłrz jak dawne przerwanie
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        processPasFilteredEdge(raw, now_us);
    }
}

// ============================================================================
// Setup
// ============================================================================

/**
 * @brief Inicjalizacja systemu â€” wywoĹ‚ywana raz przy starcie.
 *
 * KolejnoĹ›Ä‡ inicjalizacji ma znaczenie:
 * 1. GPIO muszÄ… byÄ‡ skonfigurowane przed PWM (LEDC attaches to pin)
 * 2. allMosfetsOff() bezpieczny stan przed uruchomieniem timera ISR
 * 3. Timer uruchamiany jako ostatni â€” od tego momentu ISR dziaĹ‚a
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

    // Konfiguracja z NVS (EEPROM) â€” musi byÄ‡ PRZED uĹĽyciem parametrĂłw
    config_init();
    controller_config_t& cfg = config_get();

    // Inicjalizacja stanu
    memset(&g_bldc_state, 0, sizeof(bldc_state_t));
    g_bldc_state.mode = DRIVE_MODE_DISABLED;  // tymczasowo â€” tryb docelowy ustawiony po init HW
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

    // Inicjalizacja regulatorĂłw PI dla FOC â€” z wartoĹ›ciami zapisanymi w NVS
    g_sine_hall_phase_offset = cfg.sine_hall_offset;
    g_foc_pi_d = {cfg.foc_kp_d, cfg.foc_ki_d, 0.0f, FOC_INTEGRAL_LIMIT};
    g_foc_pi_q = {cfg.foc_kp_q, cfg.foc_ki_q, 0.0f, FOC_INTEGRAL_LIMIT};
    g_foc_voltage_mode = (cfg.foc_voltage_mode != 0);

    // Inicjalizacja GPIO
    initGPIO();
    Serial.println("[OK] GPIO zainicjalizowane");

    // Konfiguracja ADC1 dla pomiaru prÄ…du fazowego w ISR (direct register access)
    // analogRead() konfiguruje kanaĹ‚y przy pierwszym wywoĹ‚aniu, ale ISR omija sterownik
    // i czyta rejestry bezpoĹ›rednio â†’ trzeba skonfigurowaÄ‡ kanaĹ‚y jawnie.
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);  // Phase A (GPIO39)
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);  // Phase B (GPIO34)
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);  // Phase C (GPIO35)
    Serial.println("[OK] ADC1 skonfigurowany (prÄ…dy fazowe, 12-bit, 11dB atten)");

    // Pre-kalibracja offsetu pradu: 64 odczytow + seed ISR EMA
    {
        uint32_t sumA = 0, sumB = 0, sumC = 0;
        const int N_CAL = 64;
        for (int i = 0; i < N_CAL; i++) {
            sumA += analogRead(PIN_PHASE_A_CURRENT);
            sumB += analogRead(PIN_PHASE_B_CURRENT);
            sumC += analogRead(PIN_PHASE_C_CURRENT);
            delayMicroseconds(200);
        }
        uint16_t avgA = sumA / N_CAL, avgB = sumB / N_CAL, avgC = sumC / N_CAL;
        g_currentOffsetV[0] = (float)avgA * (3.3f / 4095.0f);
        g_currentOffsetV[1] = (float)avgB * (3.3f / 4095.0f);
        g_currentOffsetV[2] = (float)avgC * (3.3f / 4095.0f);
        // Seed ISR EMA z prawdziwym offsetem (nie od zera!)
        g_phase_adc_ema_q8[0] = (uint32_t)avgA << 8;
        g_phase_adc_ema_q8[1] = (uint32_t)avgB << 8;
        g_phase_adc_ema_q8[2] = (uint32_t)avgC << 8;
        g_adc_ready_isr = true;
        Serial.printf("[OK] Offset pradu: A=%.4fV B=%.4fV C=%.4fV (raw %u %u %u)\n",
                      g_currentOffsetV[0], g_currentOffsetV[1], g_currentOffsetV[2], avgA, avgB, avgC);
    }

    // Przerwanie na pinie SPEED (czujnik zewnÄ™trzny â€” aktywne przy P07<=1)
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED), onSpeedPulse, FALLING);

    // PAS (Pedal Assist Sensor) â€” timer prĂłbkujÄ…cy zamiast przerwania.
    // PrĂłbkowanie co 500Âµs (2kHz), filtr cyfrowy: N zgodnych prĂłbek do zmiany stanu.
    // OdpornoĹ›Ä‡ na szpilki EMI z silnika â€” szpilka musi trwaÄ‡ NĂ—500Âµs ĹĽeby przejĹ›Ä‡.
    g_pas_filter_depth = (uint8_t)(g_pas_debounce_us_isr / PAS_SAMPLE_INTERVAL_US);
    if (g_pas_filter_depth < 2) g_pas_filter_depth = 2;
    g_pas_filtered_state = (GPIO.in >> PIN_PAS) & 1;  // stan poczÄ…tkowy
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

    // Diagnostyka rejestrĂłw MCPWM po initPWM
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
    // PokaĹĽ faktycznÄ… czÄ™stotliwoĹ›Ä‡ PWM
    {
        uint32_t grp_pre = mcpwm_ll_group_get_clock_prescale(&MCPWM0);
        uint32_t tmr_pre = mcpwm_ll_timer_get_clock_prescale(&MCPWM0, 0);
        uint32_t period  = MCPWM0.timer[0].timer_cfg0.timer_period;
        uint32_t timer_clk = 160000000UL / grp_pre / tmr_pre;
        uint32_t pwm_hz = (period > 0) ? timer_clk / (2 * period) : 0;
        Serial.printf("[MCPWM] group_pre=%u timer_pre=%u timer_clk=%u Hz  PWM=%u Hz\n",
            grp_pre, tmr_pre, timer_clk, pwm_hz);
    }

    // Upewnienie siÄ™, ĹĽe wszystkie MOSFETy sÄ… wyĹ‚Ä…czone
    allMosfetsOff();
    Serial.println("[OK] Wszystkie MOSFETy wyĹ‚Ä…czone (stan bezpieczny)");

    // Timer sprzÄ™towy do komutacji (co 50 us = 20 kHz)
    initCommutationTimer();
    g_adc_isr_active = true;  // wĹ‚Ä…cz odczyt ADC prÄ…du w ISR po peĹ‚nej konfiguracji
    Serial.println("[OK] Timer komutacji uruchomiony (20 kHz) + ADC prÄ…du w ISR TEZ");

    // Zastosuj czÄ™stotliwoĹ›Ä‡ PWM z NVS (jeĹ›li inna niĹĽ domyĹ›lna 20 kHz)
    if (cfg.pwm_freq_hz >= 8000 && cfg.pwm_freq_hz <= 32000 && cfg.pwm_freq_hz != PWM_FREQUENCY) {
        uint16_t actual = applyPwmFrequency(cfg.pwm_freq_hz);
        Serial.printf("[OK] PWM freq z NVS: %u Hz (ĹĽÄ…dane: %u Hz)\n", actual, cfg.pwm_freq_hz);
    } else {
        g_pwm_freq_hz = PWM_FREQUENCY;
    }

    Serial.printf("[OK] Rampa rozpÄ™dzania: %d ms (0â†’100%%)\n", g_bldc_state.ramp_time_ms);

    // Inicjalizacja wyĹ›wietlacza S866 na Serial2 (GPIO16/GPIO17)
    memset(&g_display, 0, sizeof(g_display));
    g_display.last_valid_ms = millis();
    s866_init();
    Serial.println("[OK] WyĹ›wietlacz S866 uruchomiony (Serial2: GPIO16/GPIO17, 9600 baud)");

    // Automatyczne wĹ‚Ä…czenie trybu jazdy z konfiguracji NVS
    {
        drive_mode_t boot_mode = (drive_mode_t)cfg.drive_mode;
        if (boot_mode >= DRIVE_MODE_BLOCK && boot_mode <= DRIVE_MODE_BLOCK12) {
            g_bldc_state.mode = boot_mode;
            g_bldc_state.fault = false;
            Serial.printf("[OK] Tryb jazdy z NVS: %s\n", driveModeName(boot_mode));
        } else {
            Serial.println("[OK] Tryb jazdy: DISABLED (nieprawidĹ‚owy w NVS)");
        }
    }

    Serial.printf("[OK] Rampa: %d ms | Regen: %s\n",
                  cfg.ramp_time_ms,
                  cfg.regen_enabled ? "ON" : "OFF");

    Serial.println();
    Serial.println("Komendy Serial: h=pomoc");
    Serial.println("==========================================");
    Serial.println();

    webConfigInit();

    g_startup_ms = millis();
}

// ============================================================================
// Auto-tune fazy sinusoidalnej
// ============================================================================

/**
 * @brief Maszyna stanĂłw auto-strojenia offsetu fazy sinusoidalnej.
 *
 * Algorytm: przy staĹ‚ym niskim duty (10%) przelatuje zakres offsetĂłw
 * g_sine_hall_phase_offset od -24 do +24 (co 2 wpisy = 7.5Â° elektr.).
 * Na kaĹĽdym kroku: 400ms stabilizacja + 600ms pomiar Ĺ›redniego prÄ…du.
 * Optymalny offset = minimum prÄ…du (najlepsza sprawnoĹ›Ä‡, najmniej strat).
 *
 * Uruchamiany komendÄ… 'sat'. Wymaga trybu SINUS lub FOC i dziaĹ‚ajÄ…cego silnika.
 * Podczas strojenia przepustnica jest ignorowana (g_manual_duty_override).
 *
 * CaĹ‚kowity czas: ~25 krokĂłw Ă— 1s = ~25 sekund.
 */
static void autoTuneStep() {
    if (g_atune_state == ATUNE_IDLE) return;

    unsigned long now = millis();

    switch (g_atune_state) {
    case ATUNE_INIT: {
        // Zapisz stan, ustaw staĹ‚e duty testowe
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
        // Czekaj na stabilizacjÄ™ prÄ…du po zmianie offsetu
        if (now - g_atune_phase_start_ms >= ATUNE_SETTLE_MS) {
            g_atune_sum_current = 0.0f;
            g_atune_sample_count = 0;
            g_atune_phase_start_ms = now;
            g_atune_state = ATUNE_MEASURE;
        }
        break;
    }

    case ATUNE_MEASURE: {
        // Akumuluj prĂłbki prÄ…du (suma 3 faz)
        float i_sum = g_bldc_state.phase_current[0]
                    + g_bldc_state.phase_current[1]
                    + g_bldc_state.phase_current[2];
        g_atune_sum_current += i_sum;
        g_atune_sample_count++;

        if (now - g_atune_phase_start_ms >= ATUNE_MEASURE_MS) {
            // Oblicz Ĺ›redni prÄ…d
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
        // PrzejdĹş do nastÄ™pnego offsetu lub zakoĹ„cz
        int next = (int)g_atune_current_ofs + (int)g_atune_offset_step;
        if (next > (int)g_atune_offset_max) {
            g_atune_state = ATUNE_DONE;
        } else {
            g_atune_current_ofs = (int8_t)next;
            g_sine_hall_phase_offset = g_atune_current_ofs;
            // Utrzymaj staĹ‚e duty testowe
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
                       + " (" + String((float)g_atune_best_ofs * 3.75f, 1) + "Â°)"
                       + "  avg_I=" + String(g_atune_best_current, 3) + " A");
        Serial.println("[SAT] Poprzedni offset: " + String((int)g_atune_saved_offset)
                       + " (" + String((float)g_atune_saved_offset * 3.75f, 1) + "Â°)");
        Serial.println("[SAT] Auto-tune zakoĹ„czony. Offset zapisany do EEPROM.");
        Serial.println("[SAT] UĹĽyj so/so+/so- do rÄ™cznej korekty.");

        // PrzywrĂłÄ‡ duty â€” manetka przejmie kontrolÄ™
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
 * @brief PAS auto-tune â€” krok maszyny stanĂłw (wywoĹ‚ywany z updateMotorState).
 *
 * Zbiera statystyki sygnaĹ‚u PAS przez PASAT_MEASURE_S sekund:
 * - min/max/avg pĂłĹ‚okres (HIGH time, LOW time)
 * - Ĺ›rednia/max asymetria H/L
 * - liczba krawÄ™dzi
 *
 * Na koniec oblicza i ustawia optymalne parametry filtra PAS:
 * - pas_min_halfperiod_ms: 50% minimalnego zmierzonego pĂłĹ‚okresu
 * - pas_dir_asymmetry_pct: 60% Ĺ›redniej zmierzonej asymetrii (min 3%)
 * - pas_debounce_us: 30% minimalnego pĂłĹ‚okresu (min 500Âµs)
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
        // Odczytaj bieĹĽÄ…ce pomiary PAS (snapshot volatile)
        uint32_t ht = g_pas_high_time_us;
        uint32_t lt = g_pas_low_time_us;

        // Nowy pomiar HIGH â€” rĂłĹĽny od poprzedniego snapshot
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

        // Asymetria (gdy oba dostÄ™pne)
        if (ht > 0 && lt > 0) {
            uint32_t sum = ht + lt;
            uint32_t diff = (ht > lt) ? (ht - lt) : (lt - ht);
            uint32_t asym_x100 = diff * 100 / sum;  // asymetria w %
            // Zbieraj unikalne pomiary (zmiana ktĂłregokolwiek)
            if (ht != g_pasat_prev_high_us || lt != g_pasat_prev_low_us) {
                // Pomiary juĹĽ zaktualizowane powyĹĽej, ale asymetriÄ™ liczymy osobno
            }
            // Zawsze aktualizuj max asymetriÄ™
            if (asym_x100 > g_pasat_max_asymmetry_x100) g_pasat_max_asymmetry_x100 = asym_x100;
            // Zbieraj Ĺ›redniÄ… asymetrii co ~100ms
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
        // min_halfperiod: 50% minimalnego zmierzonego (margines na szybsze pedaĹ‚owanie)
        uint32_t new_halfperiod_ms = (g_pasat_min_halfperiod_us / 2) / 1000;
        if (new_halfperiod_ms < 1) new_halfperiod_ms = 1;
        if (new_halfperiod_ms > 200) new_halfperiod_ms = 200;

        // debounce: 30% minimalnego pĂłĹ‚okresu (wystarczajÄ…cy do odfiltrowania szumĂłw)
        uint32_t new_debounce_us = g_pasat_min_halfperiod_us * 30 / 100;
        if (new_debounce_us < 500) new_debounce_us = 500;
        if (new_debounce_us > 10000) new_debounce_us = 10000;
        // ZaokrÄ…glij do wielokrotnoĹ›ci PAS_SAMPLE_INTERVAL_US
        new_debounce_us = (new_debounce_us / PAS_SAMPLE_INTERVAL_US) * PAS_SAMPLE_INTERVAL_US;
        if (new_debounce_us < 500) new_debounce_us = 500;

        // asymetria: 60% Ĺ›redniej (z marginesem) ale minimum 3%
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
 * @brief Krok maszyny stanĂłw kalibracji czujnika SPEED.
 *
 * Procedura: uĹĽytkownik krÄ™ci koĹ‚em dokĹ‚adnie 3 peĹ‚ne obroty w 15 sekund.
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

        // ZaokrÄ…glij do najbliĹĽszej: total_pulses / 3 obrotĂłw
        uint8_t ppr = (uint8_t)((total_pulses + 1) / 3);
        if (ppr < 1) ppr = 1;
        if (ppr > 20) ppr = 20;

        Serial.printf("[SPDCAL] Obliczony pulses_per_rev: %d (bylo: %d)\n",
                      (int)ppr, (int)g_speed_pulses_per_rev);

        // Zastosuj i zapisz
        g_speed_pulses_per_rev = ppr;
        config_get().speed_pulses_per_rev = ppr;
        config_save();

        // Reset filtra mediany (nowe parametry â†’ czysta historia)
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
 * @brief GĹ‚Ăłwna pÄ™tla aplikacji â€” wykonywana ciÄ…gle, ~kilka kHz.
 *
 * Odpowiada za wolne operacje (ADC, Serial, diagnostyka).
 * NIE wykonuje komutacji â€” robi to ISR onCommutationTimer().
 *
 * PrzepĹ‚yw:
 * 1. Odczyt ADC (napiÄ™cie, prÄ…dy, przepustnica, temp)
 * 2. Odczyt Halli i wejĹ›Ä‡ cyfrowych (hamulec, PAS)
 * 3. Mapowanie przepustnicy â†’ duty (wszystkie aktywne tryby sterowania)
 * 3a. Rampa rozpÄ™dzania (duty_cycle narasta pĹ‚ynnie w kierunku duty_target)
 * 3b. Hamulec aktywny â†’ zerowanie rampy (pĹ‚ynny rozruch po puszczeniu)
 * 4. Zapis stanu do zmiennych volatile (dla ISR, w tym g_mode_isr)
 * 5. ObsĹ‚uga komend Serial
 * 6. Auto-status (jeĹ›li wĹ‚Ä…czony)
 *
 * @note Zapis do g_duty_isr / g_motor_enabled nie jest atomowy na ESP32,
 * ale przy 32-bitowych typach i braku zaleĹĽnoĹ›ci kolejnoĹ›ci zapis jest
 * wystarczajÄ…co bezpieczny dla tej aplikacji. Przy FOC uĹĽyÄ‡ portENTER_CRITICAL.
 */
void loop() {
    // Odczyt wejĹ›Ä‡ (wolna Ĺ›cieĹĽka)
    readAnalogInputs();
    readHallSensors();
    readDigitalInputs();

    // Przepustnica sprzÄ™towa + PAS â†’ duty target
    // Algorytm wspĂłlny dla BLOCK / SINUS / FOC:
    //   maxDuty = f(assist_level)    â€” zakres wyjĹ›ciowy zaleĹĽy od poziomu
    //   throttle_duty = map(throttle, 0, maxDuty)
    //   pas_duty = f(kadencja, P11_czuĹ‚oĹ›Ä‡, P12_start, P08_speed_limit)
    //
    // Kombinacja PAS + Throttle zaleĹĽy od P10 (drive mode z wyĹ›wietlacza):
    //   P10=0: PAS + gaz â†’ duty = max(throttle_duty, pas_duty)
    //   P10=1: tylko gaz â†’ duty = throttle_duty
    //   P10=2: tylko PAS â†’ duty = pas_duty
    if (g_bldc_state.mode != DRIVE_MODE_DISABLED && !g_manual_duty_override) {
        uint16_t maxDuty = getAssistMaxDuty();

        // --- Oblicz kadencjÄ™ PAS i prÄ™dkoĹ›Ä‡ koĹ‚a ---
        g_bldc_state.pas_cadence_rpm = calculatePasCadence();
        // wheel_speed_kmh obliczana po aktualizacji wheeltime_ms (w display block poniĹĽej)
        float speed_kmh = g_bldc_state.wheel_speed_kmh;

        // --- Throttle duty ---
        // Manetka ZAWSZE dziaĹ‚a w peĹ‚nym zakresie mocy (0..PWM_MAX_DUTY),
        // niezaleĹĽnie od ustawionego poziomu wspomagania.
        // Assist level ogranicza tylko PAS (wspomaganie pedaĹ‚owania).
        // Gdy assist_level == 0 â†’ manetka teĹĽ wyĹ‚Ä…czona (silnik OFF).
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
            default: // 0 lub nierozpoznany: PAS + gaz (wyĹĽszy wygrywa)
                combined_duty = (throttle_duty > pas_duty) ? throttle_duty : pas_duty;
                break;
        }

        // --- Globalny limit prÄ™dkoĹ›ci P08 z power fade ---
        // Dotyczy WSZYSTKICH ĹşrĂłdeĹ‚ duty (manetka + PAS).
        // Moc maleje w strefie 80%..100% limitu, powyĹĽej = 0 (coast).
        // PAS ma wĹ‚asny speed fade wbudowany, ale globalny limit jest nadrzÄ™dny
        // i dotyczy teĹĽ manetki.
        combined_duty = applyGlobalSpeedLimit(combined_duty, speed_kmh);

        // --- Freewheel: jeĹ›li rowerzysta jedzie szybciej niĹĽ max silnika ---
        // Sprawdzamy efektywny limit prÄ™dkoĹ›ci dla aktualnego ĹşrĂłdĹ‚a:
        //   Manetka: P08 (max speed) â€” zawsze peĹ‚ny zakres
        //   PAS: PAS target speed (zaleĹĽy od assist level i kadencji)
        // JeĹ›li prÄ™dkoĹ›Ä‡ koĹ‚a >= efektywny limit â†’ duty = 0 (coast)
        // Motor zostanie ponownie wĹ‚Ä…czony gdy prÄ™dkoĹ›Ä‡ spadnie < 80% limitu.
        // (to jest juĹĽ obsĹ‚uĹĽone przez applyGlobalSpeedLimit powyĹĽej)

        g_bldc_state.duty_target = combined_duty;
    }

    // Rampa dwukierunkowa: duty zmienia siÄ™ pĹ‚ynnie w OBU kierunkach.
    // Czas rampy = ramp_time_ms (0â†’100% i 100%â†’0%).
    // Dodatkowy limit: duty_max_step_pct (max % zmiany na wywoĹ‚anie, EEPROM).
    // Hamulec = natychmiastowe zerowanie (safety override).
    {
        uint16_t target = g_bldc_state.duty_target;
        unsigned long now_us = micros();
        unsigned long dt_us = now_us - g_ramp_last_us;
        g_ramp_last_us = now_us;

        if (g_bldc_state.brake_active) {
            // Hamulec â†’ zeruj rampÄ™ (po puszczeniu hamulca silnik startuje pĹ‚ynnie od 0)
            g_duty_ramped = 0;
        } else if (target != g_duty_ramped) {
            // Oblicz max krok z ramp_time_ms (time-based)
            uint32_t ramp_step;
            if (g_bldc_state.ramp_time_ms > 0) {
                ramp_step = (uint32_t)PWM_MAX_DUTY * dt_us
                            / ((uint32_t)g_bldc_state.ramp_time_ms * 1000UL);
                if (ramp_step < 1) ramp_step = 1;
            } else {
                ramp_step = PWM_MAX_DUTY;  // ramp wyĹ‚Ä…czony
            }

            // Oblicz max krok z duty_max_step_pct (per-call clamp, EEPROM)
            uint8_t pct = config_get().duty_max_step_pct;
            uint32_t pct_step = (pct > 0 && pct <= 100)
                ? ((uint32_t)PWM_MAX_DUTY * pct / 100)
                : PWM_MAX_DUTY;  // 0 lub >100 â†’ brak limitu

            // UĹĽyj bardziej restrykcyjnego limitu
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
    // FOC: pÄ™tla prÄ…dowa (Clarke â†’ Park â†’ PI â†’ Vd/Vq)
    // ============================================================================
    // DziaĹ‚a z czÄ™stotliwoĹ›ciÄ… loop() (~2kHz). ISR (20kHz) odczytuje Vd/Vq
    // i generuje SVPWM z aktualnym kÄ…tem Î¸ (inverse Park + inverse Clarke).
    if (g_bldc_state.mode == DRIVE_MODE_FOC) {
        if (g_bldc_state.duty_cycle > 0) {
            // â”€â”€ Tryb napiÄ™ciowy (fvolt): Vq = duty, Vd = 0, bez PI â”€â”€
            // DziaĹ‚a identycznie jak SINUS ale w ramie dq.
            // Pozwala porĂłwnaÄ‡ "sinus vs FOC" i wykluczyÄ‡ PI jako ĹşrĂłdĹ‚o haĹ‚asu.
            if (g_foc_voltage_mode) {
                float vq = (float)g_bldc_state.duty_cycle;
                if (vq > (float)SINE_SAFE_MAX_DUTY) vq = (float)SINE_SAFE_MAX_DUTY;
                g_foc_vd_i = 0;
                g_foc_vq_i = (int32_t)vq;
                g_foc_vd_dbg = 0.0f;
                g_foc_vq_dbg = vq;
                g_foc_iq_target = vq;  // dla debug wyĹ›wietlania
                g_foc_id_meas = 0.0f;
                g_foc_iq_meas = 0.0f;
            } else {
            // â”€â”€ Tryb PI (normalny FOC) â”€â”€

            // 1. Mapowanie duty â†’ Iq target (torque)
            {
                float iq_max = FOC_IQ_MAX;
                uint8_t ilim = getEffectiveCurrentLimit();
                if (ilim > 0 && (float)ilim < iq_max) {
                    iq_max = (float)ilim;  // Warstwa 2: clamp Iq do P14/NVS
                }
                g_foc_iq_target = (float)g_bldc_state.duty_cycle * iq_max / (float)PWM_MAX_DUTY;
            }

            // 2. Feedforward napiÄ™ciowy: Vq_ff = duty (natychmiastowe napiÄ™cie)
            // PI dodaje tylko maĹ‚Ä… korektÄ™ wokĂłĹ‚ tego punktu pracy.
            // Bez feedforward: PI integruje od zera do limitu
            // â†’ przy Ki=5, err=1.5A â†’ ~7.5 PWM/s â†’ 20s do 153 PWM.
            // Z feedforward: Vq = duty natychmiast + PI(Â±korekta).
            float ff_vq = (float)g_bldc_state.duty_cycle;
            if (ff_vq > (float)SINE_SAFE_MAX_DUTY) ff_vq = (float)SINE_SAFE_MAX_DUTY;
            // Feedforward dla Vd = 0 (chcemy Id = 0)
            float ff_vd = 0.0f;

            // Globalny limit napiÄ™cia (bezpieczeĹ„stwo)
            float amp_limit = fabsf(ff_vq);  // max = |feedforward| (duty)

            // PI limit = maĹ‚a korekta wokĂłĹ‚ feedforward
            float pi_limit = FOC_PI_CORR_LIMIT;
            if (pi_limit > amp_limit) pi_limit = amp_limit;  // korekta nie wiÄ™ksza niĹĽ FF
            g_foc_pi_d.limit = pi_limit;
            g_foc_pi_q.limit = pi_limit;

            // 3. Rekonstrukcja prÄ…dĂłw ze znakiem (INA180A2 jest jednokierunkowy)
            // INA180 mierzy tylko prÄ…d w jednym kierunku. PrÄ…d w drugÄ… stronÄ™ â†’ 0V.
            // Zamiast zgadywaÄ‡ min() (niestabilne na szumie), uĹĽywamy kÄ…ta elektrycznego
            // do deterministycznego wyboru fazy z prÄ…dem ujemnym (Kirchhoff: Ia+Ib+Ic=0).
            //
            // Fazy: A=sin(Î¸), B=sin(Î¸+240Â°), C=sin(Î¸+120Â°)
            // Tabela 96 wpisĂłw = 360Â° elektr, 16 wpisĂłw = 60Â° = 1 sektor
            // Sektor z kÄ…ta: entry = angle>>16, sector = entry/16
            //
            // Faza z prÄ…dem ujemnym per sektor (wynika z sinusĂłw 3-fazowych):
            //   Sektor 0 (0-60Â°):    B ujemne (sin(240Â°..300Â°) < 0)
            //   Sektor 1 (60-120Â°):  B ujemne (sin(300Â°..360Â°) â‰¤ 0)
            //   Sektor 2 (120-180Â°): A ujemne (sin(120Â°..180Â°) â†’ crossing)
            //   Sektor 3 (180-240Â°): A ujemne (sin(180Â°..240Â°) < 0)
            //   Sektor 4 (240-300Â°): C ujemne (sin(360Â°..420Â°â†’60Â°) â†’ crossing)
            //   Sektor 5 (300-360Â°): C ujemne (sin(60Â°..120Â°â†’420Â°) depends)
            //
            // Uproszczenie: faza z najniĹĽszym napiÄ™ciem SVPWM = ujemny prÄ…d.
            // KÄ…t jest znany â†’ deterministyczny wybĂłr bez porĂłwnywania szumnych ADC.
            float ia = g_foc_ia_signed;
            float ib = g_foc_ib_signed;
            float ic = g_foc_ic_signed;

            {
                // Oblicz napiÄ™cia sinusoidalne z aktualnego kÄ…ta (identycznie jak SVPWM)
                uint32_t a_angle = g_sine_angle_q16;
                uint32_t b_angle = a_angle + ((uint32_t)SINE_PHASE_B_OFFSET << 16);
                uint32_t c_angle = a_angle + ((uint32_t)SINE_PHASE_C_OFFSET << 16);
                if (b_angle >= SINE_TABLE_Q16_FULL) b_angle -= SINE_TABLE_Q16_FULL;
                if (c_angle >= SINE_TABLE_Q16_FULL) c_angle -= SINE_TABLE_Q16_FULL;
                int32_t sa = sine_interp_q16(a_angle);
                int32_t sb = sine_interp_q16(b_angle);
                int32_t sc = sine_interp_q16(c_angle);

                // Faza z najniĹĽszym sinusem â†’ ma prÄ…d ujemny â†’ rekonstruuj z Kirchhoffa
                if (sa <= sb && sa <= sc) {
                    ia = -(ib + ic);
                } else if (sb <= sa && sb <= sc) {
                    ib = -(ia + ic);
                } else {
                    ic = -(ia + ib);
                }
            }

            // 4. Clarke: Ia,Ib,Ic â†’ IÎ±,IÎ˛
            float i_alpha = ia;
            float i_beta  = (ia + 2.0f * ib) * FOC_INV_SQRT3;

            // 5. Park: IÎ±,IÎ˛ â†’ Id,Iq (uĹĽywajÄ…c aktualnego kÄ…ta z ISR)
            // Znaki dopasowane do konwencji inverse Park (zob. focCommutateISR):
            //   Id =  IÎ±Â·cos + IÎ˛Â·sin
            //   Iq =  IÎ±Â·sin - IÎ˛Â·cos
            uint32_t angle = g_sine_angle_q16;  // volatile â†’ local copy
            int32_t sin_val = sine_interp_q16(angle);
            uint32_t cos_angle = angle + (24UL << 16);  // +90Â°
            if (cos_angle >= SINE_TABLE_Q16_FULL) cos_angle -= SINE_TABLE_Q16_FULL;
            int32_t cos_val = sine_interp_q16(cos_angle);
            float sin_f = (float)sin_val * (1.0f / 1024.0f);
            float cos_f = (float)cos_val * (1.0f / 1024.0f);

            float id =  i_alpha * cos_f + i_beta * sin_f;
            float iq =  i_alpha * sin_f - i_beta * cos_f;

            // EMA filtr na Id/Iq (sygnaĹ‚y DC po Park â€” bez phase lag!)
            g_foc_id_ema += FOC_DQ_EMA_ALPHA * (id - g_foc_id_ema);
            g_foc_iq_ema += FOC_DQ_EMA_ALPHA * (iq - g_foc_iq_ema);
            id = g_foc_id_ema;
            iq = g_foc_iq_ema;

            g_foc_id_meas = id;
            g_foc_iq_meas = iq;

            // 6. Oblicz dt (czas od ostatniej iteracji)
            unsigned long now_foc_us = micros();
            float dt = (float)(now_foc_us - g_foc_last_loop_us) / 1000000.0f;
            g_foc_last_loop_us = now_foc_us;
            if (dt <= 0.0f || dt > 0.05f) dt = FOC_LOOP_DT;  // clamp (max 50ms)

            float vd, vq;

            if (g_foc_at_active) {
                // â”€â”€ PI Auto-tune: relay feedback (Ă…strĂ¶m-HĂ¤gglund) â”€â”€
                // Vd = 0 (jak normalnie), relay tylko na osi q.
                vd = ff_vd;

                float err_q = g_foc_iq_target - iq;
                uint32_t now_at_ms = (uint32_t)(millis());

                // Detekcja przejĹ›cia przez zero bĹ‚Ä™du
                if (g_foc_at_crossings > 0 || (g_foc_at_err_prev != 0.0f)) {
                    if ((err_q > 0.0f && g_foc_at_err_prev <= 0.0f) ||
                        (err_q < 0.0f && g_foc_at_err_prev >= 0.0f)) {
                        // Zapisz amplitudÄ™ z zakoĹ„czonego pĂłĹ‚-cyklu
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
                // Ĺšledzenie min/max bĹ‚Ä™du w bieĹĽÄ…cym pĂłĹ‚-cyklu
                if (err_q > g_foc_at_err_max) g_foc_at_err_max = err_q;
                if (err_q < g_foc_at_err_min) g_foc_at_err_min = err_q;
                g_foc_at_err_prev = err_q;

                // WyjĹ›cie relay: +/- relay_amp wokĂłĹ‚ feedforward
                float relay_out = (err_q > 0.0f) ? g_foc_at_relay_amp : -g_foc_at_relay_amp;
                vq = ff_vq + relay_out;

                // Sprawdzenie zakoĹ„czenia testu
                if ((now_at_ms - g_foc_at_start_ms) >= g_foc_at_duration_ms) {
                    g_foc_at_active = false;
                    // Reset PI integratorĂłw po relay
                    g_foc_pi_d.integral = 0.0f;
                    g_foc_pi_q.integral = 0.0f;

                    // Oblicz wyniki
                    Serial.println();
                    Serial.println("[PI Auto-tune] ZakoĹ„czony.");
                    if (g_foc_at_crossings >= 4 && g_foc_at_amp_count > 0) {
                        // Ĺšrednia amplituda oscylacji Iq [A]
                        float avg_amp = g_foc_at_amp_sum / (float)g_foc_at_amp_count;
                        // Okres oscylacji Tu [s]
                        float tu_s = (float)(g_foc_at_last_cross_ms - g_foc_at_first_cross_ms)
                                     / (float)(g_foc_at_crossings - 1) * 2.0f / 1000.0f;
                        // Ultimate gain: Ku = 4*d/(Ď€*a)
                        float ku = 4.0f * g_foc_at_relay_amp / (3.14159f * avg_amp);

                        // Konserwatywne PI (Tyreus-Luyben, bezpieczniejsze niĹĽ Z-N):
                        // Kp = 0.30*Ku (Z-N: 0.45), Ki = Kp/(2.2*Tu)  (Z-N: Kp*1.2/Tu)
                        float kp_new = 0.30f * ku;
                        float ki_new = (tu_s > 0.001f) ? (kp_new / (2.2f * tu_s)) : 0.0f;

                        // Sanity clamp â€” zapobiegaj absurdalnym wartoĹ›ciom
                        if (kp_new > 5.0f) kp_new = 5.0f;
                        if (ki_new > 50.0f) ki_new = 50.0f;
                        if (kp_new < 0.01f) kp_new = FOC_KP_DEFAULT;
                        if (ki_new < 0.01f) ki_new = FOC_KI_DEFAULT;

                        Serial.printf("  Oscylacje: %d przejsc, amplituda=%.3f A, Tu=%.4f s\n",
                                      g_foc_at_crossings, avg_amp, tu_s);
                        Serial.printf("  Ku=%.3f, Tu=%.4f s\n", ku, tu_s);
                        Serial.printf("  >> Zastosowane: Kp=%.3f  Ki=%.3f\n", kp_new, ki_new);

                        // Zastosuj obliczone wartoĹ›ci i zapisz do EEPROM
                        g_foc_pi_d.kp = kp_new;  g_foc_pi_d.ki = ki_new;
                        g_foc_pi_q.kp = kp_new;  g_foc_pi_q.ki = ki_new;
                        config_get().foc_kp_q = kp_new;  config_get().foc_ki_q = ki_new;
                        config_get().foc_kp_d = kp_new;  config_get().foc_ki_d = ki_new;
                        config_save();
                        Serial.println("  [PI Auto-tune] WartoĹ›ci zapisane do EEPROM.");
                    } else {
                        Serial.printf("  Za malo oscylacji (%d crossings). Zwieksz duty lub relay_amp.\n",
                                      g_foc_at_crossings);
                    }
                }
            } else {
                // â”€â”€ Normalny tryb PI â”€â”€

            // 7. PI regulator osi d: target Id = 0 (MTPA â€” max torque per amp)
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

            // 10. Clamp wektora napiÄ™ciowego: |V| â‰¤ SINE_SAFE_MAX_DUTY
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
            // ISR NIE moĹĽe uĹĽywaÄ‡ float (brak zapisu kontekstu FPU w timer ISR).
            g_foc_vd_i = (int32_t)vd;
            g_foc_vq_i = (int32_t)vq;
            g_foc_vd_dbg = vd;  // kopia float do debugowania
            g_foc_vq_dbg = vq;
            }  // koniec else (tryb PI)
        } else {
            // Duty = 0 â†’ reset PI integratorĂłw i EMA
            g_foc_pi_d.integral = 0.0f;
            g_foc_pi_q.integral = 0.0f;
            g_foc_vd_i = 0;
            g_foc_vq_i = 0;
            g_foc_vd_dbg = 0.0f;
            g_foc_vq_dbg = 0.0f;
            g_foc_iq_target = 0.0f;
            g_foc_id_ema = 0.0f;
            g_foc_iq_ema = 0.0f;
        }
    }

    // Aktualizacja zmiennych ISR z gĹ‚Ăłwnego stanu
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

    // Sync kierunku obrotĂłw z konfiguracji NVS do ISR
    g_reverse_isr = (config_get().motor_reverse != 0);

    // PrÄ™dkoĹ›Ä‡ sinusoidalna (speed_q16) jest teraz obliczana bezpoĹ›rednio
    // w ISR (onCommutationTimer) na przejĹ›ciu Halla â€” bez opĂłĹşnienia loop().
    // Tu nie ma nic do roboty â€” zostawione jako komentarz dla czytelnoĹ›ci.

    // Obliczanie mocy: P = Vbat Ă— max(Ia, Ib, Ic)
    float g_maxI_now = 0.0f;  // max prÄ…d fazowy â€” surowy (do wyĹ›wietlacza/mocy)
    float g_maxI_filtered;    // EMA-filtrowany max prÄ…d (do limitera prÄ…dowego)
    {
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        g_maxI_now = maxI;

        // EMA filtr prÄ…du dla limitera â€” wygĹ‚adza szum ADC.
        // ADC zsynchronizowany z PWM (ISR TEZ) â†’ odczyty stabilne.
        // EMA Î±=0.15 â†’ Ď„â‰3ms przy loop ~1kHz.
        g_ilim_current_ema += ILIMIT_EMA_ALPHA * (maxI - g_ilim_current_ema);
        if (g_ilim_current_ema < 0.0f) g_ilim_current_ema = 0.0f;
        g_maxI_filtered = g_ilim_current_ema;

        float power = g_bldc_state.battery_voltage * g_maxI_filtered;

        if (g_bldc_state.regen_active) {
            // W trybie regen: prÄ…d pĹ‚ynie do baterii â†’ moc ujemna (oddawana)
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
    // Limit prÄ…dowy (3 warstwy: soft P-regulator, Iq clamp, hard cutoff)
    // ====================================================================
    // Warstwa 1 (soft): g_current_limit_factor (0..1) â€” mnoĹĽnik duty.
    //   Gdy maxI > limit â†’ szybko obcina duty (Kp_down Ă— error).
    //   Gdy maxI < limit â†’ wolno wraca do 1.0 (recovery rate).
    // Warstwa 2 (FOC): Iq target obcinany do limitu (w sekcji FOC powyĹĽej).
    // Warstwa 3 (hard): maxI > 150% â†’ natychmiastowy duty=0 na 500ms.
    {
        uint8_t limit_a = getEffectiveCurrentLimit();

        if (limit_a > 0 && g_motor_enabled) {
            // Grace period: ignoruj limiter przez 2s po starcie (offset ADC się kalibruje)
            if ((millis() - g_startup_ms) < ILIMIT_STARTUP_GRACE_MS) {
                g_current_limit_factor = 1.0f;
                g_ilim_current_ema = 0.0f;
            } else {
            float flimit = (float)limit_a;

            // --- Warstwa 3: hard cutoff (ochrona MOSFET/silnika) ---
            // Wymaga OVERCURRENT_CONSEC kolejnych prĂłbek (filtrowanego prÄ…du)
            // powyĹĽej progu. Pojedyncza szpilka ADC nie wywoĹ‚a cutoff.
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
                    // Przekroczenie â†’ szybka redukcja
                    g_current_limit_factor -= ILIMIT_KP_DOWN * error;
                    if (g_current_limit_factor < 0.05f) g_current_limit_factor = 0.05f;
                } else {
                    // Pod limitem â†’ wolne odzyskiwanie
                    // dt â‰ 0.5-1ms (loop ~1-2kHz)
                    g_current_limit_factor += ILIMIT_RECOVER_RATE * 0.001f;
                    if (g_current_limit_factor > 1.0f) g_current_limit_factor = 1.0f;
                }

                // Zastosuj mnoĹĽnik do duty_isr (BLOCK + SINUS + FOC backup)
                if (g_current_limit_factor < 1.0f) {
                    uint16_t limited = (uint16_t)((float)g_duty_isr * g_current_limit_factor);
                    g_duty_isr = limited;
                }
            }
            } // end grace period else
        } else {
            // Brak limitu lub silnik wyĹ‚Ä…czony â†’ peĹ‚na moc
            g_current_limit_factor = 1.0f;
            g_overcurrent_fault = false;
            g_ilim_current_ema = 0.0f;
            g_ilim_hard_count = 0;
        }
    }

    // Regeneracja â€” logika aktywacji (hamulec + regen_enabled + warunki bezpieczeĹ„stwa)
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
            // Warunki niespeĹ‚nione â€” wyĹ‚Ä…cz regen (coast)
            g_bldc_state.regen_active = false;
            g_regen_active_isr = false;
            g_regen_duty_isr = 0;
        }
    } else {
        g_bldc_state.regen_active = false;
        g_regen_active_isr = false;
        g_regen_duty_isr = 0;
    }

    // ObsĹ‚uga wyĹ›wietlacza S866 (Serial2 â€” zawsze aktywny)
    {
        // Aktualizuj dane TX dla wyĹ›wietlacza
        g_display.tx.error = g_bldc_state.fault ? 1 : 0;
        g_display.tx.brake_active = g_bldc_state.brake_active ? 1 : 0;

        // PrÄ…d: maksimum z 3 faz, w jednostkach 0.1A
        float maxI = g_bldc_state.phase_current[0];
        if (g_bldc_state.phase_current[1] > maxI) maxI = g_bldc_state.phase_current[1];
        if (g_bldc_state.phase_current[2] > maxI) maxI = g_bldc_state.phase_current[2];
        g_display.tx.current_x10 = (uint16_t)(maxI * 10.0f);

        // Wheeltime [ms] â€” ĹşrĂłdĹ‚o zaleĹĽy od P07:
        //   P07 > 1  â†’ silnik direct-drive, P07 = liczba impulsĂłw Halla na obrĂłt koĹ‚a
        //              (= 6 transitions/erev Ă— pole_pairs, np. 6Ă—15=90)
        //              wheeltime = hall_period_us Ă— P07 / 1000
        //              NIE mnoĹĽymy dodatkowo Ă—6, bo P07 juĹĽ to zawiera!
        //   P07 == 1 â†’ silnik przekĹ‚adniowy, uĹĽyj zewnÄ™trznego czujnika SPEED
        //              wheeltime = speed_period_us / 1000 (1 impuls na obrĂłt)
        //   P07 == 0 â†’ brak konfiguracji, uĹĽyj Halli z domyĹ›lnym P07=1 (SPEED)
        uint8_t p07 = g_display.config.p07_speed_magnets;
        uint32_t wt_us = 0;

        // Gdy brak wyĹ›wietlacza i p07==0 â†’ fallback na Halle z domyĹ›lnym P07
        if (p07 == 0 && !g_display.connected) {
            p07 = DEFAULT_P07_STANDALONE;
        }

        if (p07 <= 1) {
            // P07==1: czujnik zewnÄ™trzny SPEED (1 magnes na koĹ‚o)
            uint32_t sp = g_speed_period_us;  // volatile â†’ local copy
            uint32_t last = g_speed_last_pulse_us;
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            // Timeout: jeĹ›li >3s od ostatniego impulsu â†’ koĹ‚o stoi
            if (sp > 0 && sp < 10000000 && last > 0 && (now_us - last) < 3000000) {
                // Przetwarzaj kaĹĽdy nowy impuls ISR tylko raz (skip duplikatĂłw)
                if (sp != g_speed_last_processed_sp) {
                    g_speed_last_processed_sp = sp;

                    // Stale median reset: jeĹ›li mediana waĹĽna ale ĹĽaden impuls
                    // nie zaakceptowany >3s, warunki siÄ™ zmieniĹ‚y â€” reset filtra
                    if (g_speed_period_valid >= 3 && g_speed_last_accepted_us > 0
                        && (now_us - g_speed_last_accepted_us) > 3000000) {
                        g_speed_period_valid = 0;
                        g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
                    }

                    // Outlier rejection: nowy pomiar akceptowany tylko gdy
                    // jest w zakresie 1/3..3x aktualnej mediany (gdy mediana dostÄ™pna).
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
                // Oblicz medianÄ™ (lub surowy) do wheeltime
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
                // KoĹ‚o stanÄ™Ĺ‚o â€” reset mediany (wĹ‚Ä…cznie z zawartoĹ›ciÄ… bufora)
                g_speed_period_valid = 0;
                g_speed_period_buf[0] = g_speed_period_buf[1] = g_speed_period_buf[2] = 0;
                g_speed_last_accepted_us = 0;
            }
            // Przelicz: period_jednego_impulsu Ă— impulsy_na_obrĂłt = czas_obrotu_koĹ‚a
            if (wt_us > 0 && g_speed_pulses_per_rev > 1) {
                wt_us *= g_speed_pulses_per_rev;
            }
        } else {
            // P07 > 1: direct-drive hub, P07 = hall transitions per wheel revolution
            // P07 = 6 Ă— pole_pairs (np. 90 = 6Ă—15 par biegunĂłw)
            uint32_t hp = g_hall_period_us;  // volatile â†’ local copy
            uint32_t last = g_hall_last_change_us;
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            // Timeout: jeĹ›li >2s od ostatniego przejĹ›cia Halla â†’ silnik stoi
            if (hp > 0 && hp < 2000000 && last > 0 && (now_us - last) < 2000000) {
                wt_us = (uint32_t)hp * (uint32_t)p07;  // bez Ă—6! P07 juĹĽ zawiera 6Ă—pole_pairs
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

        // PrÄ™dkoĹ›Ä‡ koĹ‚a â€” oblicz tutaj, po aktualizacji wheeltime_ms
        g_bldc_state.wheel_speed_kmh = calculateWheelSpeedKmh();

        // ObsĹ‚uga protokoĹ‚u wyswietlacza
        s866_service(&g_display);
    }

    // ObsĹ‚uga komend USB Serial (zawsze aktywna)
    processSerialCommands();

    if (!g_wifi_active) {
        webConfigInit();
    }
    webConfigHandle();

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

    // Debug prÄ…dĂłw co 500ms
    if (g_debugCurrent && (millis() - g_lastDebugCurrentMs >= DEBUG_CURRENT_INTERVAL_MS)) {
        g_lastDebugCurrentMs = millis();
        float maxI = fmaxf(g_bldc_state.phase_current[0],
                           fmaxf(g_bldc_state.phase_current[1], g_bldc_state.phase_current[2]));
        float dutyPct = (float)g_bldc_state.duty_cycle * 100.0f / PWM_MAX_DUTY;
        // Idc â‰ Ifaz Ă— duty (przybliĹĽenie prÄ…du DC bus z zasilacza)
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

    // Debug Hall co 200ms â€” Ĺ›ledzenie komutacji
    if (g_debugHall && (millis() - g_lastDebugHallMs >= DEBUG_HALL_INTERVAL_MS)) {
        g_lastDebugHallMs = millis();
        const char* pathNames[] = {"OFF", "BLK", "SIN", "FOC", "B-S", "B12"};
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

    // â”€â”€ Event-driven debug: drukuj zdarzenia z ring buffera ISR â”€â”€
    if (g_debugCommutation) {
        static const char* modeTag[] = {"OFF", "BLK", "SIN", "FOC", "B12"};
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

    // Auto-tune fazy sinusoidalnej (maszyna stanĂłw)
    autoTuneStep();

    // PAS auto-tune (maszyna stanĂłw)
    pasAutoTuneStep();

    // SPEED calibration (maszyna stanĂłw)
    spdCalStep();
}

// ============================================================================
// Inicjalizacja GPIO
// ============================================================================

/**
 * @brief Konfiguracja wszystkich pinĂłw GPIO.
 *
 * Piny PWM (mostki) sÄ… ustawiane w bezpieczny stan (wszystkie tranzystory OFF)
 * PRZED przeĹ‚Ä…czeniem trybu na OUTPUT â€” zapobiega to impulsowi przy starcie.
 *
 * @warning GPIO12 (PIN_PWM_A_HIGH) jest pinem STRAP ESP32.
 * Pull-up na GPIO12 during boot przestawia VDD_SDIO na 1.8V â†’ brak uploadu do flash.
 *
 * @warning GPIO0 (PIN_EXT_1) jest pinem BOOT. LOW przy resecie = tryb programowania.
 * UĹĽywaÄ‡ ostroĹĽnie.
 */
void initGPIO() {
    // --- WyjĹ›cia PWM (sterowanie IR2103) ---
    // Najpierw ustawiamy bezpieczny stan, potem tryb OUTPUT
    
    // Faza A
    digitalWrite(PIN_PWM_A_HIGH, IR2103_HIN_OFF);   // High-side OFF
    digitalWrite(PIN_PWM_A_LOW, IR2103_LIN_OFF);     // Low-side OFF (LIN=HIGH bo odwrĂłcone)
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

    // --- WejĹ›cia analogowe (ADC) ---
    // GPIO 34, 35, 36 - tylko wejĹ›cie (input-only), nie wymagajÄ… pinMode
    // ale ustawiamy dla czytelnoĹ›ci
    pinMode(PIN_BATTERY_VOLTAGE, INPUT);
    pinMode(PIN_PHASE_B_CURRENT, INPUT);
    pinMode(PIN_PHASE_C_CURRENT, INPUT);

    // --- Czujnik temperatury FET (GPIO32/ADC1_CH4) ---
    // PIN_FET_TEMP â€” nie wymaga pinMode, input-only nie dotyczy, ale ustawiamy
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

    // --- PrÄ™dkoĹ›Ä‡ (wejĹ›cie czujnika zewnÄ™trznego â€” aktywne przy P07==1) ---
    pinMode(PIN_SPEED, INPUT_PULLUP);

    // --- UART Enable ---
    pinMode(PIN_UART_EN, OUTPUT);
    digitalWrite(PIN_UART_EN, LOW);

    // --- Rozszerzenia ---
    // DomyĹ›lnie jako wejĹ›cia
    // GPIO0 - uwaga: boot pin!
    pinMode(PIN_EXT_1, INPUT_PULLUP);
    pinMode(PIN_EXT_2, INPUT);
    pinMode(PIN_EXT_3, INPUT);
}

// ============================================================================
// Zmiana czÄ™stotliwoĹ›ci PWM w runtime
// ============================================================================

/**
 * @brief Zmienia czÄ™stotliwoĹ›Ä‡ PWM przez modyfikacjÄ™ prescalera timera MCPWM.
 *
 * Period=500 (PWM_MAX_DUTY) nie zmienia siÄ™ â€” duty 0-500 nadal odpowiada 0-100%.
 * Zmienia siÄ™ tylko prescaler timera: timer_clk = 160MHz / prescaler.
 * freq = timer_clk / (2 Ă— 500) = 160000 / prescaler
 * prescaler = round(160000 / freq_hz)
 *
 * Dead-time ticks skalowany do ~500ns: ticks = timer_clk * 500ns = 160M/pre * 500e-9
 *
 * @param freq_hz  Ĺ»Ä…dana czÄ™stotliwoĹ›Ä‡ [Hz], zakres 8000-32000
 * @return Faktyczna czÄ™stotliwoĹ›Ä‡ [Hz] po zaokrÄ…gleniu prescalera
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

    // WyĹ‚Ä…cz silnik na czas zmiany prescalera
    allMosfetsOff();

    // ZmieĹ„ prescaler i dead-time dla wszystkich 3 timerĂłw
    for (int t = 0; t < 3; t++) {
        mcpwm_ll_timer_set_clock_prescale(&MCPWM0, t, prescaler);
        // Period bez zmian (500) â€” PWM_MAX_DUTY zachowane
    }
    // Dead-time update (bypass mode â€” IR2103 robi own dead-time)
    // Nie potrzebujemy zmieniaÄ‡ DT bo IR2103 ma wbudowany

    g_pwm_freq_hz = actual_freq;
    return actual_freq;
}

// ============================================================================
// Inicjalizacja PWM (LEDC)
// ============================================================================

/**
 * @brief Konfiguracja 6 kanaĹ‚Ăłw LEDC dla sterowania mostkami IR2103.
 *
 * KaĹĽda z 3 faz (A, B, C) ma dwa kanaĹ‚y LEDC:
 * - HIGH: steruje wejĹ›ciem HIN (high-side) â€” duty=0 = OFF, duty=d = PWM
 * - LOW:  steruje wejĹ›ciem LIN (low-side) â€” LOGIKA ODWRĂ“CONA!
 *   - duty=PWM_MAX_DUTY â†’ LIN=HIGH â†’ low-side OFF (bezpieczny stan domyĹ›lny)
 *   - duty=0            â†’ LIN=LOW  â†’ low-side ON (przewodzi prÄ…d do GND)
 *
 * Parametry PWM (MCPWM center-aligned):
 * - CzÄ™stotliwoĹ›Ä‡: PWM_FREQUENCY = 20 kHz (UP_DOWN counter â†’ symmetric)
 * - RozdzielczoĹ›Ä‡ timera: 20 MHz â†’ period = 500 counts (0..500)
 * - Dead time: bypass (IR2103 ma wewnÄ™trzny ~520ns)
 * - Synchronizacja: Timer 1,2 zsynchronizowane z Timer 0 (TEZ)
 *
 * @note Po initPWM() wszystkie fazy sÄ… w stanie float (OFF).
 * allMosfetsOff() wywoĹ‚uje to samo, ale jest idempotentna.
 */
void initPWM() {
    // GPIO init â€” przypisanie pinĂłw do MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM_A_HIGH);  // Op0 gen_A â†’ HIN_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_PWM_A_LOW);   // Op0 gen_B â†’ LIN_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_PWM_B_HIGH);  // Op1 gen_A â†’ HIN_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_PWM_B_LOW);   // Op1 gen_B â†’ LIN_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_PWM_C_HIGH);  // Op2 gen_A â†’ HIN_C
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PIN_PWM_C_LOW);   // Op2 gen_B â†’ LIN_C

    // â”€â”€ Konfiguracja timerĂłw: UP_DOWN (center-aligned), 20 kHz â”€â”€
    // mcpwm_init() ustawia GPIO matrix, operator binding, counter mode itp.
    // CzÄ™stotliwoĹ›Ä‡ podana tutaj NIE jest dokĹ‚adna â€” prescalery i period
    // wymuszamy rÄ™cznie poniĹĽej, ĹĽeby uzyskaÄ‡ DOKĹADNIE 20 kHz z period=500.
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;     // orientacyjna, nadpisana poniĹĽej
    pwm_config.cmpr_a = 0.0f;               // duty A = 0%
    pwm_config.cmpr_b = 0.0f;               // duty B = 0%
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // active high
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;  // center-aligned!

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // â”€â”€ Wymuszenie prescalerĂłw + period dla DOKĹADNIE 20 kHz PWM â”€â”€
    // Timer clock = 160 MHz / group_pre / timer_pre = MCPWM_TIMER_RESOLUTION (20 MHz)
    // PWM freq (UP_DOWN) = timer_clk / (2 Ă— period) = 20M / (2Ă—500) = 20 kHz
    // BEZ TEGO: mcpwm_init() dobiera prescaler pod swojÄ… period (np. 200),
    //           a wymuszenie period=500 obniĹĽa freq do ~16 kHz (SĹYSZALNE!)
    mcpwm_ll_group_set_clock_prescale(&MCPWM0, 1);  // group_clk = 160 MHz
    for (int t = 0; t < 3; t++) {
        mcpwm_ll_timer_set_clock_prescale(&MCPWM0, t, 160000000UL / MCPWM_TIMER_RESOLUTION);
        mcpwm_ll_timer_set_peak(&MCPWM0, t, MCPWM_TIMER_PERIOD, true);
    }

    // â”€â”€ Synchronizacja timerĂłw: Timer 1,2 syncowane z Timer 0 (TEZ) â”€â”€
    // Timer 0: master â€” generuje sync na TEZ (Timer Equals Zero)
    mcpwm_sync_config_t sync_master = {};
    sync_master.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    sync_master.timer_val = 0;
    sync_master.count_direction = MCPWM_TIMER_DIRECTION_UP;

    // Timer 0 generuje sync przy TEZ
    mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
    // Timer 1,2 syncujÄ… siÄ™ z Timer 0
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_master);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_2, &sync_master);

    // â”€â”€ Dead time: bypass (IR2103 ma wewnÄ™trzny ~520ns dead time) â”€â”€
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_DEADTIME_BYPASS, 0, 0);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_DEADTIME_BYPASS, 0, 0);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_DEADTIME_BYPASS, 0, 0);

    // Startuj timery
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

    // â”€â”€ Konfiguracja generatorĂłw: action registers + force upmethod â”€â”€
    // mcpwm_init() ustawia gen_cntuforce_upmethod=32 (disable update).
    // Nasze ISR helpery zapisujÄ… gen_force.val bezpoĹ›rednio, wiÄ™c potrzebujemy
    // upmethod=0 (immediate). Zapisz generator action registers (MODE_0)
    // i ustaw force upmethod na immediate dla wszystkich 3 operatorĂłw.
    for (int op = 0; op < 3; op++) {
        MCPWM0.operators[op].generator[0].val = GEN_A_ACTION_MODE0;
        MCPWM0.operators[op].generator[1].val = GEN_B_ACTION_MODE0;
        // force upmethod = 0 (immediate), force modes = 0 (disabled/PWM)
        MCPWM0.operators[op].gen_force.val = 0;

        // â”€â”€ Compare shadow: update TYLKO na TEZ (nie TEZ+TEP!) â”€â”€
        // mcpwm_init() ustawia TEZ+TEP (stmp_cfg=0x33). W center-aligned PWM
        // to powoduje ASYMETRYCZNE impulsy: rampa UP uĹĽywa starych wartoĹ›ci
        // compare, a DOWN nowych (po TEP shadow transfer). Szum!
        // TEZ-only â†’ obie rampy uĹĽywajÄ… tej samej wartoĹ›ci â†’ symetryczny PWM.
        mcpwm_ll_operator_enable_update_compare_on_tez(&MCPWM0, op, 0, true);   // cmpr_a: TEZ âś“
        mcpwm_ll_operator_enable_update_compare_on_tep(&MCPWM0, op, 0, false);  // cmpr_a: !TEP
        mcpwm_ll_operator_enable_update_compare_on_tez(&MCPWM0, op, 1, true);   // cmpr_b: TEZ âś“
        mcpwm_ll_operator_enable_update_compare_on_tep(&MCPWM0, op, 1, false);  // cmpr_b: !TEP

        // Action register update: immediate (waĹĽne przy przejĹ›ciach BLOCKâ†”SIN/FOC)
        mcpwm_ll_operator_update_action_at_once(&MCPWM0, op);
    }

    // Inicjalny stan: wszystkie fazy OFF (float)
    allMosfetsOff();
}

// ============================================================================
// WyĹ‚Ä…czenie wszystkich MOSFETĂłw (stan bezpieczny)
// ============================================================================

/**
 * @brief PrzeĹ‚Ä…cza wszystkie tranzystory w stan OFF (stan bezpieczny / float).
 *
 * MCPWM: gen_A = forced LOW (HIN=LOW â†’ HS OFF),
 *         gen_B = forced HIGH (LIN=HIGH â†’ LS OFF, bo IR2103 LIN odwrĂłcony).
 *
 * WywoĹ‚ywana:
 * - po initPWM() podczas startu
 * - przy komendzie 'd' (disable)
 * - przy hamulcu w ISR
 * - przy bĹ‚Ä™dnym stanie Halla (0 lub 7)
 *
 * @note Bezpieczna do wywoĹ‚ania z loop() i z ISR.
 */
void IRAM_ATTR allMosfetsOff() {
    mcpwm_phase_off(0);  // Faza A
    mcpwm_phase_off(1);  // Faza B
    mcpwm_phase_off(2);  // Faza C
}

// ============================================================================
// Odczyt wejĹ›Ä‡ analogowych
// ============================================================================

/**
 * @brief Odczytuje wszystkie wejĹ›cia analogowe i przelicza na wartoĹ›ci fizyczne.
 *
 * Wykonywane obliczenia:
 * 1. ADC raw â†’ napiÄ™cie [V] (mnoĹĽnik 3.3/4095)
 * 2. VBAT: V_ADC Ă— kVbatDividerGain = V_ADC Ă— (R_top+R_bot)/R_bot
 * 3. PrÄ…dy: autokalibracja offsetu (EMA, Î±=0.02) gdy silnik off lub duty=0
 *    PrÄ…d [A] = (V_ADC - offset) Ă— kCurrentScale
 *    gdzie kCurrentScale = 1/(R_shunt Ă— INA_gain) = 1/(0.002 Ă— 50) = 10 A/V
 * 4. Przepustnica: raw ADC do g_bldc_state.throttle_raw (mapowanie w loop())
 * 5. Temperatura silnika: raw ADC (bez przeliczenia, czekamy na specyfikacjÄ™ czujnika)
 *
 * @note PrÄ…dy < 0 sÄ… clampowane do 0 (nie ma ujemnego prÄ…du przez shunty low-side).
 * @note Autokalibracja offsetu prÄ…du wymaga kilku sekund z wyĹ‚Ä…czonym silnikiem
 *       przy uruchomieniu firmware (filtr EMA stabilizuje siÄ™ po ~50 iteracjach).
 */
void readAnalogInputs() {
    // Odczyt napiÄ™cia baterii â€” ADC1_CH0 (GPIO36)
    // Guard: wyĹ‚Ä…cz ISR ADC na czas odczytu, bo ISR teĹĽ uĹĽywa ADC1.
    g_adc_isr_active = false;
    uint16_t batteryRaw = analogRead(PIN_BATTERY_VOLTAGE);
    g_adc_isr_active = true;

    // â”€â”€ Odczyt prÄ…dĂłw fazowych: zsynchronizowany z PWM (ISR TEZ) â”€â”€
    // ISR onCommutationTimer czyta ADC prÄ…du w dolinie center-aligned PWM
    // (counter=0, wszystkie low-side ON) â†’ odczyt stabilny, bez szpilek.
    // Gdy ISR nie dostarczyĹ‚ danych (np. przed uruchomieniem timera),
    // Odczyt ciÄ…gĹ‚ej EMA prÄ…du z ISR (fixed-point Q8 â†’ wartoĹ›Ä‡ realna).
    // Atomowy 32-bit read na ESP32 â€” bez spinlocka.
    uint16_t phaseA_raw, phaseB_raw, phaseC_raw;
    if (g_adc_ready_isr) {
        phaseA_raw = (uint16_t)(g_phase_adc_ema_q8[0] >> 8);
        phaseB_raw = (uint16_t)(g_phase_adc_ema_q8[1] >> 8);
        phaseC_raw = (uint16_t)(g_phase_adc_ema_q8[2] >> 8);
    } else {
        // Fallback: bezpoĹ›redni odczyt (przed init ISR lub ISR nieaktywny)
        g_adc_isr_active = false;
        phaseA_raw = analogRead(PIN_PHASE_A_CURRENT);
        phaseB_raw = analogRead(PIN_PHASE_B_CURRENT);
        phaseC_raw = analogRead(PIN_PHASE_C_CURRENT);
        g_adc_isr_active = true;
    }

    // Przepustnica: bufor koĹ‚owy N prĂłbek (1 na iteracjÄ™ loop) + mediana
    // GPIO33/ADC1_CH5 â€” podatny na szpilki EMI od PWM silnika.
    // Zamiast burst (N prĂłbek w ~100Âµs) â€” 1 prĂłbka na loop() (~0.5ms).
    // PrĂłbki rozĹ‚oĹĽone w czasie: szpilka EMI trwajÄ…ca <NĂ—0.5ms
    // zanieczyszcza tylko czÄ™Ĺ›Ä‡ bufora, mediana jÄ… odrzuca.
    {
        static uint16_t thr_ring[16] = {0};
        static uint8_t  thr_ring_idx = 0;
        static bool     thr_ring_init = false;

        controller_config_t& tcfg = config_get();
        uint8_t n_samples = tcfg.thr_samples;
        if (n_samples < 2) n_samples = 2;
        if (n_samples > 16) n_samples = 16;
        uint16_t thresh = tcfg.thr_outlier_thresh;

        // 1 prĂłbka ADC na wywoĹ‚anie (rozĹ‚oĹĽone w czasie, nie burst)
        uint16_t raw = analogRead(PIN_THROTTLE);
        thr_ring[thr_ring_idx] = raw;
        thr_ring_idx = (thr_ring_idx + 1) % n_samples;

        // Inicjalizacja: wypeĹ‚nij bufor pierwszÄ… prĂłbkÄ…
        if (!thr_ring_init) {
            for (uint8_t i = 0; i < 16; i++) thr_ring[i] = raw;
            thr_ring_init = true;
        }

        // Kopia do sortowania (nie modyfikujemy bufora koĹ‚owego)
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

        // Mediana â€” odporna na szpilki (do 50% bufora moĹĽe byÄ‡ zaĹ›miecone)
        uint16_t median = sorted[n_samples / 2];

        // Ĺšrednia z prĂłbek bliskich medianie (dodatkowe wygĹ‚adzenie)
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

    // Odczyt temperatury FET (ADC1_CH4/GPIO32) â€” PIN_FET_TEMP
    // Aktualnie PIN_FET_TEMP NIE jest czytany (czujnik niepodĹ‚Ä…czony).

    // Surowe wartoĹ›ci ADC
    const float batteryAdcV = batteryRaw * (3.3f / 4095.0f);
    const float phaseA_V = phaseA_raw * (3.3f / 4095.0f);
    const float phaseB_V = phaseB_raw * (3.3f / 4095.0f);
    const float phaseC_V = phaseC_raw * (3.3f / 4095.0f);

    // Autokalibracja zera prÄ…du gdy silnik nie pracuje
    if (g_bldc_state.mode == DRIVE_MODE_DISABLED || g_bldc_state.duty_cycle == 0) {
        g_currentOffsetV[0] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[0] + kCurrentOffsetAlpha * phaseA_V;
        g_currentOffsetV[1] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[1] + kCurrentOffsetAlpha * phaseB_V;
        g_currentOffsetV[2] = (1.0f - kCurrentOffsetAlpha) * g_currentOffsetV[2] + kCurrentOffsetAlpha * phaseC_V;
    }

    g_bldc_state.battery_voltage = batteryAdcV * kVbatDividerGain;

    float ia = (phaseA_V - g_currentOffsetV[0]) * kCurrentScale;
    float ib = (phaseB_V - g_currentOffsetV[1]) * kCurrentScale;
    float ic = (phaseC_V - g_currentOffsetV[2]) * kCurrentScale;

    // FOC: zapisz prÄ…dy przed klipowaniem (Clarke/Park potrzebuje wartoĹ›ci ze znakiem)
    g_foc_ia_signed = ia;
    g_foc_ib_signed = ib;
    g_foc_ic_signed = ic;

    // FOC: EMA filtr prÄ…dĂłw â€” wygĹ‚adza szum ADC.
    // ADC zsynchronizowany z PWM (ISR TEZ) â†’ odczyty stabilne, bez szpilek.
    // EMA wygĹ‚adza niewielki szum kwantyzacji SAR ADC.
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
// Odczyt czujnikĂłw Halla
// ============================================================================

/**
 * @brief Odczytuje 3 czujniki Halla i zapisuje 3-bitowy kod do g_bldc_state.hall_state.
 *
 * Format: hall_state = [C:B:A] gdzie bit0=HallA, bit1=HallB, bit2=HallC.
 * Czujniki sÄ… INPUT_PULLUP (aktywny LOW: logika odwrĂłcona przez hardware).
 * digitalRead() zwraca juĹĽ poprawnÄ… wartoĹ›Ä‡ logicznÄ… po pull-up.
 *
 * Poprawne stany: 1, 2, 3, 4, 5, 6 (6 pozycji elektrycznych rotora).
 * Stany 0 i 7 oznaczajÄ… bĹ‚Ä…d czujnikĂłw (zwarcie lub przerwa).
 *
 * @note W ISR Halle sÄ… czytane szybciej bezpoĹ›rednio z rejestru GPIO.in
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
// Odczyt wejĹ›Ä‡ cyfrowych
// ============================================================================

/**
 * @brief Odczytuje wejĹ›cia cyfrowe: hamulec i PAS.
 *
 * Hamulec: INPUT_PULLUP â€” aktywny sygnaĹ‚ = LOW (przycisk do GND).
 * Debounce: wymagaj BRAKE_DEBOUNCE_THRESHOLD kolejnych odczytĂłw LOW
 * zanim uzna hamulec za aktywny. Filtruje spike EMI z przeĹ‚Ä…czania
 * FETĂłw (szczegĂłlnie w SINUS/FOC, gdzie 6 FETĂłw przeĹ‚Ä…cza jednoczeĹ›nie).
 * Bez debounce: spike EMI â†’ brake_active=true â†’ g_duty_ramped=0 â†’
 * silnik gwaĹ‚townie zwalnia i rozpÄ™dza siÄ™ ponownie.
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
    // pas_active = pedaĹ‚owanie (potwierdzony PAS albo okno init)
    g_bldc_state.pas_active = g_pas_pedaling;
    g_bldc_state.pas_forward = g_pas_forward;  // volatile â†’ state
}

// ============================================================================
// WyĹ›wietlanie diagnostyki
// ============================================================================

/**
 * @brief Wypisuje pojedynczÄ… liniÄ™ statusu na Serial.
 *
 * Format: `MODE D:duty% V:Vbat Ia:X.XX Ib:X.XX Ic:X.XX H:CBA T:temp Thr:thr%(raw) [flagi]`
 *
 * PrzykĹ‚ad:
 * @code
 * BLK D:45% V:36.1 Ia:1.23 Ib:0.98 Ic:1.15 H:101 T:312 Thr:45%(1850)
 * @endcode
 *
 * Kolumny:
 * - MODE:    OFF/BLK/SIN/FOC (tryb sterowania)
 * - D:       duty cycle PWM [%]
 * - V:       napiÄ™cie baterii [V]
 * - Ia/Ib/Ic: prÄ…dy fazowe [A]
 * - H:       stan Halla [C:B:A] jako 3 bity
 * - T:       surowa wartoĹ›Ä‡ ADC temperatury silnika
 * - Thr:     przepustnica [%] i (RAW ADC)
 * - Opcjonalne flagi: BRK (hamulec), PAS, FAULT
 *
 * @note WywoĹ‚anie Serial.printf() moĹĽe zablokowaÄ‡ loop() na kilkanaĹ›cie ms.
 *       Komutacja jest w ISR i nie jest tym zakĹ‚Ăłcana.
 */
void printDiagnostics() {
    const char* modeNames[] = {"OFF", "BLK", "SIN", "FOC", "B12"};
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

    // PAS: linia diagnostyczna â€” zawsze wyĹ›wietlana
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

        // Kadencja [RPM] z okresu PAS i liczby magnesĂłw
        float cadence_rpm = 0.0f;
        uint8_t magnets = g_display.config.p13_pas_magnets;
        if (magnets == 0) magnets = 1;
        if (period > 0 && since_us < 2000000UL) {
            cadence_rpm = 60000000.0f / ((float)period * (float)magnets);
        }

        // Stan sĹ‚owny
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

        // JeĹ›li PAS aktywny â€” pokaĹĽ duty i V_target
        if (g_pas_pedaling) {
            int pasDutyPct = (int)((uint32_t)g_bldc_state.pas_duty * 100 / PWM_MAX_DUTY);
            uint8_t ra = g_display.rx.assist_level;
            uint8_t vm = g_display.config.p08_speed_limit;
            if (vm == 0) vm = 25;
            float vt = 6.0f + (float)ra * ((float)vm - 6.0f) / 15.0f;
            Serial.printf(" d:%d%% vt:%.1f sl:%.2f", pasDutyPct, vt, g_speed_limit_factor);
        }
    }

    // PrÄ™dkoĹ›Ä‡ koĹ‚a [km/h] + debug SPEED
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

    // Debug SINUS/FOC: parametry Ĺ›ledzenia kÄ…ta
    if (g_bldc_state.mode == DRIVE_MODE_SINUS || g_bldc_state.mode == DRIVE_MODE_FOC) {
        uint32_t ang = g_sine_angle_q16;
        int32_t ang_entry = (int32_t)(ang >> 16);
        int32_t hall_err_entry = g_dbg_last_hall_err >> 16;  // w wpisach tabeli (1=3.75Â°)
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

    // Informacje z wyĹ›wietlacza S866
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
 * Linie [SDBG] pokazujÄ… sekwencjÄ™:
 * Hall edges -> startup -> wejĹ›cie SINUS -> fallback -> ponowny startup.
 */
static void printSineDebug() {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    int32_t angle_entry = (int32_t)(g_sine_angle_q16 >> 16);
    int32_t angle_frac = (int32_t)(g_sine_angle_q16 & 0xFFFF);

    Serial.printf("[SDBG] mode:%s run:%u st:%u hall:%u hp:%luus since:%lums ",
        (g_bldc_state.mode == DRIVE_MODE_SINUS) ? "SIN" : ((g_bldc_state.mode == DRIVE_MODE_BLOCK) ? "BLK" : ((g_bldc_state.mode == DRIVE_MODE_BLOCK12) ? "B12" : "OTH")),
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

    // â”€â”€ Rozszerzona diagnostyka (running stats z okna ~200ms) â”€â”€
    int32_t err_min_ent = g_dbg_err_min_q16 >> 16;
    int32_t err_max_ent = g_dbg_err_max_q16 >> 16;
    Serial.printf("  err[%ld..%ld] spd[%lu..%lu] dt[%lu..%lu]us snp/cor:%lu/%lu\n",
        (long)err_min_ent, (long)err_max_ent,
        (unsigned long)g_dbg_speed_min, (unsigned long)g_dbg_speed_max,
        (unsigned long)g_dbg_dt_min, (unsigned long)g_dbg_dt_max,
        (unsigned long)g_dbg_snap_window, (unsigned long)g_dbg_corr_window);
    // KÄ…t obserwera vs oczekiwany kÄ…t (z ostatniego Hall edge w oknie)
    Serial.printf("  angH:%ld exp:%ld sec:%d dir:%d rev:%d\n",
        (long)(g_dbg_angle_at_hall >> 16),
        (long)(g_dbg_expected_at_hall >> 16),
        (int)g_sine_last_hall_idx,
        (int)g_sine_dir,
        (int)g_reverse_isr);

    // Reset running stats dla nastÄ™pnego okna
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
// Timer ISR - komutacja w przerwaniu (niezaleĹĽna od loop)
// ============================================================================

/**
 * @brief ISR timera sprzÄ™towego â€” wykonywana co 50 Âµs (20 kHz).
 *
 * To jest SERCE sterownika. WywoĹ‚ywana niezaleĹĽnie od loop().
 * Czyta stan z volatile zmiennych globalnych i ustawia kanaĹ‚y LEDC.
 *
 * ## Priorytety obsĹ‚ugi (kolejnoĹ›Ä‡ sprawdzania):
 * 1. Hamulec aktywny (g_brake_isr) â†’ natychmiast allMosfetsOff()
 * 2. Silnik wyĹ‚Ä…czony (!g_motor_enabled) â†’ allMosfetsOff()
 * 3. Odczyt Halli z rejestru GPIO.in (szybkie, bez przerwaĹ„)
 * 4. WywoĹ‚anie tabeli komutacji dla aktualnego kierunku
 *
 * ## Dlaczego IRAM_ATTR?
 * Kod ISR musi byÄ‡ w RAM, nie w flash. Bez IRAM_ATTR, jeĹ›li cache flash
 * jest zajÄ™ty (np. przez OTA lub SPIFFS), ISR moĹĽe wywoĹ‚aÄ‡ cache miss
 * i zawiesiÄ‡ siÄ™ na dziesiÄ…tki mikrosekund â†’ zakĹ‚Ăłcenia komutacji.
 *
 * ## Odczyt GPIO.in zamiast digitalRead()
 * `GPIO.in` to bezpoĹ›redni rejestr hardware GPIO0-31.
 * Bit N = stan GPIO N. Czytanie rejestru trwa ~5 ns vs ~1 Âµs dla digitalRead().
 *
 * @warning Nie wolno tu uĹĽywaÄ‡: malloc, Serial, delay, mutex, nor F() string.
 * @warning Funkcje MCPWM force/duty sÄ… ISR-safe (operujÄ… na rejestrach sprzÄ™towych).
 */

// â”€â”€ ISR-safe ADC1 read â€” bezpoĹ›redni dostÄ™p do rejestrĂłw SAR ADC â”€â”€
// analogRead() uĹĽywa mutex â†’ nie wolno w ISR.
// Ta funkcja omija sterownik i czyta SAR ADC1 bezpoĹ›rednio (~3Âµs).
// Wymaga wczeĹ›niejszej konfiguracji kanaĹ‚u (adc1_config_channel_atten w setup).
// Timeout: max ~5Âµs (1200 cykli @240MHz) â€” zapobiega blokowaniu ISR
// gdy ADC jest zajÄ™ty przez analogRead() z loop().
static uint16_t IRAM_ATTR adc1_read_isr(uint8_t channel) {
    // Upewnij siÄ™, ĹĽe SAR ADC1 jest w trybie RTC (software trigger)
    SENS.sar_meas_start1.meas1_start_force = 1;  // force start by software
    SENS.sar_read_ctrl.sar1_dig_force = 0;        // SAR1 controlled by RTC, not DIG
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    // Timeout: ~1200 iteracji â‰ 5Âµs @240MHz. Normalnie konwersja trwa ~2Âµs.
    uint32_t timeout = 1200;
    while (!SENS.sar_meas_start1.meas1_done_sar && --timeout) {}
    if (timeout == 0) { g_adc_timeout_count++; return 0; }  // ADC busy â€” zwrĂłÄ‡ 0
    return SENS.sar_meas_start1.meas1_data_sar;
}

void IRAM_ATTR onCommutationTimer(void *arg) {
    // Skasuj flagÄ™ przerwania TEZ Timer 0
    mcpwm_ll_intr_clear_timer_tez_status(&MCPWM0, 1 << 0);

    // â”€â”€ Odczyt ADC prÄ…du w dolinie PWM (TEZ) â”€â”€
    // TEZ = counter=0 w center-aligned: wszystkie low-side ON â†’ prÄ…d shuntu stabilny.
    // Czytaj NATYCHMIAST po wejĹ›ciu do ISR, zanim okno zero-vector siÄ™ skoĹ„czy.
    // Trzy odczyty ~9Âµs â€” mieĹ›ci siÄ™ w oknie zero-vector (â‰Ą6Âµs przy 75% duty).
    // Flaga g_adc_isr_active=false gdy loop() czyta ADC1 (battery) â†’ unikamy konfliktu HW.
    if (g_adc_isr_active) {
        uint16_t a = adc1_read_isr(ADC1_CHANNEL_3);  // Phase A (GPIO39)
        uint16_t b = adc1_read_isr(ADC1_CHANNEL_6);  // Phase B (GPIO34)
        uint16_t c = adc1_read_isr(ADC1_CHANNEL_7);  // Phase C (GPIO35)
        // Zachowaj ostatni raw do diagnostyki (idbg/cdbg)
        g_phase_adc_raw_isr[0] = a;
        g_phase_adc_raw_isr[1] = b;
        g_phase_adc_raw_isr[2] = c;
        // CiÄ…gĹ‚a EMA per kanaĹ‚ w fixed-point Q8 (wartoĹ›Ä‡ Ă— 256).
        // Spike > ADC_SPIKE_THRESHOLD â†’ pominiÄ™ty (EMA trzyma poprzedniÄ… wartoĹ›Ä‡).
        // ema += ((sample<<8) - ema) >> SHIFT  â€” bez mnoĹĽenia, ~3 cykle ARM.
        if (a < ADC_SPIKE_THRESHOLD) { int32_t d = (int32_t)((uint32_t)a << 8) - (int32_t)g_phase_adc_ema_q8[0]; g_phase_adc_ema_q8[0] = (uint32_t)((int32_t)g_phase_adc_ema_q8[0] + (d >> ADC_EMA_SHIFT)); }
        if (b < ADC_SPIKE_THRESHOLD) { int32_t d = (int32_t)((uint32_t)b << 8) - (int32_t)g_phase_adc_ema_q8[1]; g_phase_adc_ema_q8[1] = (uint32_t)((int32_t)g_phase_adc_ema_q8[1] + (d >> ADC_EMA_SHIFT)); }
        if (c < ADC_SPIKE_THRESHOLD) { int32_t d = (int32_t)((uint32_t)c << 8) - (int32_t)g_phase_adc_ema_q8[2]; g_phase_adc_ema_q8[2] = (uint32_t)((int32_t)g_phase_adc_ema_q8[2] + (d >> ADC_EMA_SHIFT)); }
        g_adc_ready_isr = true;
    }

    // Odczyt Halli ZAWSZE â€” pomiar prÄ™dkoĹ›ci nawet gdy silnik wyĹ‚Ä…czony
    uint8_t ha = (GPIO.in >> PIN_HALL_SENSOR_A) & 1;
    uint8_t hb = (GPIO.in >> PIN_HALL_SENSOR_B) & 1;
    uint8_t hc = (GPIO.in >> PIN_HALL_SENSOR_C) & 1;
    uint8_t hall = (hc << 2) | (hb << 1) | ha;

    // â”€â”€ Filtr Hall z wieloprĂłbkowym potwierdzeniem (anty-EMI) â”€â”€
    // 3 warstwy:
    //   1. Odrzucenie nieprawidĹ‚owych stanĂłw (0, 7)
    //   2. Multi-sample: nowy stan musi utrzymaÄ‡ siÄ™ HALL_CONFIRM_COUNT tickĂłw ISR
    //   3. Debounce czasowy: HALL_MIN_PERIOD_US od ostatniego potwierdzonego przejĹ›cia
    // Pod obciÄ…ĹĽeniem PWM generuje silne EMI sprzÄ™gajÄ…ce siÄ™ w linie Halla,
    // tworzÄ…c faĹ‚szywe przejĹ›cia. Glitch <150Âµs jest odrzucany.

    // Warstwa 1: odrzuÄ‡ nieprawidĹ‚owe stany (brak/zwarcie czujnika)
    if (hall == 0 || hall == 7) {
        g_dbg_hall_invalid++;
        g_hall_confirm_cnt = 0;  // reset potwierdzenia â€” przerwany ciÄ…g
    }
    else if (hall != g_hall_prev_isr) {
        // Warstwa 2: multi-sample confirmation
        if (hall == g_hall_candidate) {
            g_hall_confirm_cnt++;
        } else {
            // Kandydat siÄ™ zmieniĹ‚ przed potwierdzeniem â†’ prawdziwy glitch EMI
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
                g_startup_state < 2) {
                min_hall_period_us = HALL_MIN_PERIOD_STARTUP_US;
            }

            if (g_hall_last_change_us == 0 || dt_us >= min_hall_period_us) {
                g_dbg_hall_edges++;
                g_dbg_last_hall = hall;
                if (g_hall_last_change_us > 0) {
                    g_hall_period_us = dt_us;
                }
                g_hall_last_change_us = now_us;

                // Block crossfade: zapisz stary remapowany Hall PRZED aktualizacjÄ…
                if (g_mode_isr == DRIVE_MODE_BLOCK && g_hall_prev_isr >= 1 && g_hall_prev_isr <= 6) {
                    g_block_old_bh = g_reverse_isr ? g_hall_reverse_map[g_hall_prev_isr] : g_hall_prev_isr;
                    g_block_cross_cnt = BLOCK_CROSS_TICKS;
                }
                g_hall_prev_isr = hall;

                // â”€â”€ Sine/FOC mode: przetwarzanie przejĹ›cia Halla â”€â”€
                if (g_mode_isr == DRIVE_MODE_SINUS || g_mode_isr == DRIVE_MODE_FOC) {
                    int8_t new_idx = hallToSector(hall);
                    if (new_idx >= 0) {
                        int8_t old_idx = g_sine_last_hall_idx;
                        bool is_snap = false;

                        // Walidacja sekwencji: akceptuj tylko Â±1 sektor
                        // Przeskok >1 sektora = prawdopodobnie glitch EMI ktĂłry
                        // przeszedĹ‚ multi-sample (np. stabilne pole EMI pod duĹĽym obciÄ…ĹĽeniem).
                        // W takim przypadku: aktualizuj timestamp/idx ale NIE koryguj kÄ…ta/prÄ™dkoĹ›ci.
                        bool seq_valid = true;
                        if (old_idx >= 0) {
                            int8_t fwd = (old_idx + 1) % 6;
                            int8_t rev = (old_idx + 5) % 6;
                            if (new_idx == fwd) {
                                g_sine_dir = 1;   // prawidĹ‚owy kierunek
                            } else if (new_idx == rev) {
                                // Przy zablokowanym kole i maĹ‚ym duty szum Hall czÄ™sto wyglÄ…da jak
                                // naprzemienne Â±1 sektor. Blokujemy takie "cofanie" na starcie.
                                uint16_t duty_lock_th = (uint16_t)(PWM_MAX_DUTY * SINE_DIR_LOCK_DUTY_PCT / 100);
                                bool startup_lock =
                                    (g_startup_state < 2) &&
                                    (g_duty_isr <= duty_lock_th);
                                if (startup_lock) {
                                    g_dbg_hall_seq_reject++;
                                    seq_valid = false;
                                } else {
                                    g_sine_dir = -1;  // cofanie siÄ™
                                }
                            } else {
                                // Przeskok >1 sektora â€” podejrzany
                                g_dbg_hall_seq_reject++;
                                seq_valid = false;
                            }
                        }

                        // Stall detection timestamp â€” zawsze aktualizuj
                        g_sine_last_hall_ms = (uint32_t)(now_us / 1000);

                        // Running stats: Hall period min/max
                        if (dt_us < g_dbg_dt_min) g_dbg_dt_min = dt_us;
                        if (dt_us > g_dbg_dt_max) g_dbg_dt_max = dt_us;

                        if (seq_valid) {
                            // --- Aktualizacja prÄ™dkoĹ›ci per-sector ---
                            // Halle mogÄ… byÄ‡ nierĂłwno rozmieszczone (30-40%),
                            // globalna prÄ™dkoĹ›Ä‡ nie dziaĹ‚a â†’ zapamiÄ™taj czas
                            // kaĹĽdego sektora osobno.
                            if (dt_us >= min_hall_period_us && dt_us < 500000) {
                                uint32_t new_speed = 52428800UL / dt_us;
                                // old_idx = sektor WYCHODZÄ„CY (wĹ‚aĹ›nie zmierzony)
                                if (old_idx >= 0 && old_idx < 6) {
                                    uint32_t prev = g_sine_sector_speed[old_idx];
                                    if (prev == 0) {
                                        g_sine_sector_speed[old_idx] = new_speed;
                                    } else {
                                        // Lekki filtr: 3/4 new + 1/4 old
                                        g_sine_sector_speed[old_idx] = (new_speed * 3 + prev) >> 2;
                                    }
                                }
                                // PrÄ™dkoĹ›Ä‡ interpolacji = prÄ™dkoĹ›Ä‡ sektora WCHODZÄ„CEGO
                                if (new_idx >= 0 && new_idx < 6 && g_sine_sector_speed[new_idx] != 0) {
                                    g_sine_speed_q16 = g_sine_sector_speed[new_idx];
                                } else {
                                    // Fallback: uĹĽyj wĹ‚aĹ›nie zmierzonej
                                    g_sine_speed_q16 = new_speed;
                                }
                            }

                            // Korekcja kÄ…ta na przejĹ›ciu Halla
                            // Snap point = Ĺ›rodek sektora + offset.
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
                            g_dbg_angle_at_hall = current;  // kÄ…t PRZED korekcjÄ…
                            g_dbg_expected_at_hall = expected;  // oczekiwany kÄ…t wg tabeli

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
                                // Korekcja prÄ™dkoĹ›ci PLL.
                                // err>0 = kÄ…t za wolny (za maĹ‚y/za duĹĽy wg kierunku)
                                // Dla dir:+1 (kÄ…t roĹ›nie): err>0 â†’ zwiÄ™ksz prÄ™dkoĹ›Ä‡ (+)
                                // Dla dir:-1 (kÄ…t maleje): err<0 â†’ kÄ…t za wysoki = za wolny
                                //   â†’ potrzeba WIÄKSZEJ prÄ™dkoĹ›ci (wiÄ™cej odejmowania)
                                //   â†’ ale err<0 daĹ‚by ujemnÄ… korektÄ™ â†’ ODWRĂ“Ä† znak!
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

                        // â”€â”€ Push debug event: Hall change w trybie SINUS/FOC â”€â”€
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
                // â”€â”€ Push debug event: Hall change w trybie BLOCK â”€â”€
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

    // â”€â”€ Od tego momentu: uĹĽywaj PRZEFILTROWANEGO Halla do komutacji â”€â”€
    // Surowy `hall` z GPIO.in moĹĽe mieÄ‡ glitche EMI (transient na 1 tick ISR).
    // g_hall_prev_isr = potwierdzony stan (3 zgodne odczyty + debounce 200Âµs).
    // Komutacja BLOCK/SINUS/FOC musi uĹĽywaÄ‡ stabilnego stanu.
    g_dbg_hall_raw_isr = hall;  // diagnostyka: surowy vs filtrowany
    hall = g_hall_prev_isr;
    g_dbg_hall_filt_isr = hall;

    // Tryb testu MOSFET â€” ISR nie dotyka kanaĹ‚Ăłw LEDC, tylko mierzy Hall/prÄ™dkoĹ›Ä‡
    if (g_mosfet_test_active) {
        return;
    }

    if (g_brake_isr) {
        // Hamulec aktywny â€” regen braking jeĹ›li wĹ‚Ä…czony, inaczej coast
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

    // SINUS/FOC safety fallback: PRZED guardem d==0, ĹĽeby prÄ™dkoĹ›Ä‡
    // byĹ‚a zerowana nawet gdy duty=0 (motor coastuje po puszczeniu manetki).
    // Bez tego: spd zostaje zamroĹĽone na starej wartoĹ›ci â†’ przy ponownym
    // duty kÄ…t startuje z bĹ‚Ä™dnÄ… prÄ™dkoĹ›ciÄ… â†’ snap â†’ szarpniÄ™cie.
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
            // Stall â†’ powrĂłt do alignment startup
            g_sine_startup_count = 0;
            g_startup_state = 0;  // IDLE â†’ przy nastÄ™pnym d>0 zrobi alignment
            g_sine_last_hall_ms = now_ms;
        }
    }

    uint16_t d = g_duty_isr;

    // â”€â”€ Explicit freewheel: duty == 0 â†’ wolnobieg â”€â”€
    // BLOCK: natychmiast allMosfetsOff (bo d=0 + LS ON = hamowanie EM).
    // SINUS/FOC: kontynuuj do sinusCommutateISR/focCommutateISR â€”
    // angle advance musi dziaĹ‚aÄ‡ nawet przy d=0, inaczej po ponownym
    // wĹ‚Ä…czeniu duty kÄ…t jest stary â†’ duĹĽy err â†’ snap â†’ szarpniÄ™cie.
    // sinusCommutateISR/focCommutateISR same wyĹ‚Ä…czÄ… MOSFETy (amplitude < MIN).
    if (d == 0 && g_mode_isr != DRIVE_MODE_SINUS && g_mode_isr != DRIVE_MODE_FOC) {
        allMosfetsOff();
        return;
    }

    // Dispatch trybu sterowania
    switch (g_mode_isr) {
        case DRIVE_MODE_SINUS:
        case DRIVE_MODE_FOC:
        {
            // â”€â”€ Startup state machine: IDLE â†’ ALIGN â†’ RUN â”€â”€
            // 1. ALIGN: jeden krok BLOCK z maĹ‚ym duty â†’ rotor ustala pozycjÄ™
            // 2. RUN: normalna praca SINUS/FOC z monitorowaniem Halli
            uint32_t now_us = (uint32_t)esp_timer_get_time();

            // Reset do IDLE gdy duty=0 (manetka puszczona)
            if (d == 0) {
                if (g_startup_state != 0) {
                    g_startup_state = 0;
                    resetSineTracking(hall);
                }
            }

            // IDLE â†’ ALIGN: rozpocznij wyrĂłwnanie
            if (g_startup_state == 0 && d > 0) {
                g_startup_state = 1;  // ALIGN
                g_startup_align_start_us = now_us;
                g_startup_align_hall = hall;
                resetSineTracking(hall);
            }

            // ALIGN: wymuszony jeden krok BLOCK z kontrolowanym duty
            if (g_startup_state == 1) {
                uint32_t elapsed_us = now_us - g_startup_align_start_us;
                uint16_t align_duty = (uint16_t)((uint32_t)PWM_MAX_DUTY * STARTUP_ALIGN_DUTY_PCT / 100);
                // Duty narastajÄ…ce liniowo w pierwszej poĹ‚owie alignmentu
                uint32_t half_ms = STARTUP_ALIGN_MS / 2;
                uint32_t elapsed_ms = elapsed_us / 1000;
                if (elapsed_ms < half_ms) {
                    align_duty = (uint16_t)((uint32_t)align_duty * elapsed_ms / half_ms);
                    if (align_duty < 1) align_duty = 1;
                }
                // UĹĽyj BLOCK komutacji z bieĹĽÄ…cym Hallem
                uint8_t bh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;
                if (bh >= 1 && bh <= 6) {
                    const uint8_t *ps = g_block_phase_tbl[bh];
                    for (int ph = 0; ph < 3; ph++) {
                        if (ps[ph] == 2)      mcpwm_phase_pwm(ph, align_duty);
                        else if (ps[ph] == 1) mcpwm_phase_gnd(ph);
                        else                  mcpwm_phase_off(ph);
                    }
                }
                g_dbg_commut_path = 4;  // ALIGN
                // PrzejĹ›cie do RUN po upĹ‚ywie czasu ALIGN
                if (elapsed_us >= (uint32_t)STARTUP_ALIGN_MS * 1000) {
                    g_startup_state = 2;  // RUN
                    resetSineTracking(hall);
                }
                return;
            }

            // RUN: normalna praca SINUS/FOC
            // KÄ…t zarzÄ…dzany przez PLL (korekta na przejĹ›ciach Halla).
            // Nie robimy per-tick snap â€” alignment zapewnia poprawnÄ… pozycjÄ™ startowÄ….
            if (g_mode_isr == DRIVE_MODE_SINUS) {
                g_dbg_commut_path = 2;  // SINUS
                sinusCommutateISR(hall, d);
            } else {
                g_dbg_commut_path = 3;  // FOC
                focCommutateISR(hall, d);
            }
            return;
        }
        case DRIVE_MODE_BLOCK12:
        {
            g_dbg_commut_path = 5;  // BLOCK12
            uint8_t bh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;
            uint8_t mid_bh = bh;

            if (g_hall_last_change_us > 0 && g_hall_period_us >= (2 * HALL_MIN_PERIOD_US)) {
                uint32_t now_us = (uint32_t)esp_timer_get_time();
                uint32_t dt_us = now_us - g_hall_last_change_us;
                uint32_t half_us = g_hall_period_us >> 1;
                if (dt_us >= half_us && dt_us < (g_hall_period_us * 2U)) {
                    mid_bh = blockHallNextCw(bh);
                }
            }

            applyBlockState(mid_bh, d);
            return;
        }
        case DRIVE_MODE_BLOCK:
            g_dbg_commut_path = 1;  // BLOCK
            break;  // kontynuuj do komutacji blokowej poniĹĽej
        default:
            // Tryb DISABLED â†’ bezpieczny stan
            allMosfetsOff();
            return;
    }

    uint8_t bh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;
    applyBlockState(bh, d);
}

// ============================================================================
// Inicjalizacja timera komutacji
// ============================================================================

/**
 * @brief Konfiguruje i uruchamia ISR komutacji na przerwaniu MCPWM TEZ.
 *
 * Konfiguracja:
 * - MCPWM_UNIT_0, Timer 0 TEZ (Timer Equals Zero) interrupt
 * - CzÄ™stotliwoĹ›Ä‡ ISR = 20 kHz (= czÄ™stotliwoĹ›Ä‡ PWM, center-aligned)
 * - ISR wywoĹ‚ywana w dolinie PWM (counter=0) â€” wszystkie low-side ON
 *   â†’ optymalny moment na odczyt ADC prÄ…du (INA180A2 mierzy gdy LS przewodzi)
 *
 * @note allMosfetsOff() musi byÄ‡ wywoĹ‚ana PRZED initCommutationTimer().
 */
void initCommutationTimer() {
    // Rejestruj globalny ISR MCPWM
    mcpwm_isr_register(MCPWM_UNIT_0, onCommutationTimer, NULL,
                        ESP_INTR_FLAG_IRAM,
                        &g_mcpwm_isr_handle);
    // WĹ‚Ä…cz przerwanie TEZ dla Timer 0 (= dolina center-aligned PWM)
    mcpwm_ll_intr_enable_timer_tez(&MCPWM0, 0, true);
}

// ============================================================================
// Komutacja sinusoidalna â€” ISR (port z bldc_driver_v2 TIM1_UP_IRQHandler)
// ============================================================================

/**
 * @brief Interpolacja liniowa sinusa z tablicy 97-elementowej (Q16).
 *
 * Identyczna logika jak sine_interp_q16() z bldc_driver_v2/src/bldc.c.
 * Tablica ma 97 wpisĂłw (96+guard) â€” guard entry [96]=[0] eliminuje % 96.
 *
 * @param angle_q16  KÄ…t w wpisach tablicy (Q16), musi byÄ‡ 0..SINE_TABLE_Q16_FULL-1
 * @return WartoĹ›Ä‡ sinusa -1024..+1024
 */
static inline int32_t IRAM_ATTR sine_interp_q16(uint32_t angle_q16) {
    uint32_t idx  = angle_q16 >> 16;        // indeks caĹ‚kowity 0..95
    uint32_t frac = angle_q16 & 0xFFFF;     // czÄ™Ĺ›Ä‡ uĹ‚amkowa Q16
    int32_t s0 = (int32_t)g_sine_table[idx];
    int32_t s1 = (int32_t)g_sine_table[idx + 1];  // guard entry [96] = entry [0]
    return s0 + (((s1 - s0) * (int32_t)frac) >> 16);
}

/**
 * @brief Komutacja sinusoidalna â€” ciÄ…gĹ‚e Ĺ›ledzenie kÄ…ta.
 *
 * ## Algorytm (port z bldc_driver_v2 TIM1_UP_IRQHandler)
 *
 * 1. Advance angle: angle += speed_q16
 * 2. Stall freeze: brak przejĹ›Ä‡ Halla > 200ms â†’ zamroĹĽenie kÄ…ta
 * 3. Oblicz 3 kÄ…ty fazowe: C=base, A=base+32, B=base+64 (Ă—120Â°)
 * 4. Interpolacja sinusa z tabeli + obliczenie duty
 * 5. Center-aligned complementary: HIN=LIN=ten sam duty
 *
 * ## Mapowanie faz (identyczne z STM32)
 *   Phase C = sin(Î¸)           â€” faza referencyjna
 *   Phase A = sin(Î¸ + 120Â°)    â€” offset 32 wpisĂłw (96/3)
 *   Phase B = sin(Î¸ + 240Â°)    â€” offset 64 wpisĂłw (96Ă—2/3)
 *
 * ## PWM center-aligned complementary
 *   duty = PWM_MAX_DUTY/2 + (sine_val Ă— amplitude) >> 10
 *   HIN = LIN = duty â†’ IR2103 produkuje komplementarne switching z dead-time
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

    // â”€â”€ 1. Stall freeze: brak przejĹ›cia Halla > 200ms â†’ nie avansuj kÄ…ta â”€â”€
    // ZamraĹĽamy kÄ…t tylko na timeout Halla (rotor stanÄ…Ĺ‚), NIE na prĂłg prÄ™dkoĹ›ci.
    // Alignment zapewnia poprawnÄ… pozycjÄ™ startowÄ…, PLL koryguje na przejĹ›ciach Halla.
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    uint32_t real_speed = g_sine_speed_q16;
    bool stalled = (since_hall > SINE_STALL_FREEZE_MS);

    // â”€â”€ 2. Advance angle â”€â”€
    // Kierunek avansowania kÄ…ta: z detekcji Halli (g_sine_dir), nie config.
    if (!stalled) {
        uint32_t spd = real_speed;
        // Przy speed==0 (tuĹĽ po alignment) uĹĽyj minimalnej prÄ™dkoĹ›ci crawl
        if (spd == 0) spd = SINE_CRAWL_SPEED_Q16;
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

    // â”€â”€ Amplitude guard (PO angle advance!) â”€â”€
    if (amplitude < SINE_MIN_AMPLITUDE) {
        allMosfetsOff();
        return;
    }

    // â”€â”€ 3. Oblicz 3 kÄ…ty fazowe z offsetami â”€â”€
    //   Phase A = sin(Î¸)           â€” faza referencyjna
    //   Phase B = sin(Î¸ + 240Â°)    â€” offset 64 wpisĂłw
    //   Phase C = sin(Î¸ + 120Â°)    â€” offset 32 wpisĂłw
    uint32_t angle = g_sine_angle_q16;

    uint32_t angle_a = angle;  // A = reference
    uint32_t angle_b = angle + ((uint32_t)SINE_PHASE_B_OFFSET << 16);
    uint32_t angle_c = angle + ((uint32_t)SINE_PHASE_C_OFFSET << 16);

    // Wrap to valid range (subtraction, no modulo)
    if (angle_b >= SINE_TABLE_Q16_FULL) angle_b -= SINE_TABLE_Q16_FULL;
    if (angle_c >= SINE_TABLE_Q16_FULL) angle_c -= SINE_TABLE_Q16_FULL;

    // â”€â”€ 4. Interpolacja sinusa + obliczenie duty â”€â”€
    int32_t sin_a = sine_interp_q16(angle_a);  // -1024..+1024
    int32_t sin_b = sine_interp_q16(angle_b);
    int32_t sin_c = sine_interp_q16(angle_c);

    // amplitude: 0..PWM_MAX_DUTY, z ograniczeniem bezpieczeĹ„stwa
    int32_t amp = (int32_t)((amplitude > SINE_SAFE_MAX_DUTY) ? SINE_SAFE_MAX_DUTY : amplitude);
    g_dbg_last_amp = (uint16_t)amp;

    // Modulacja fazy: -amp..+amp
    int32_t ma = (sin_a * amp) >> 10;
    int32_t mb = (sin_b * amp) >> 10;
    int32_t mc = (sin_c * amp) >> 10;
    g_dbg_last_ma = (int16_t)ma;
    g_dbg_last_mb = (int16_t)mb;
    g_dbg_last_mc = (int16_t)mc;

    // â”€â”€ 5. Write LEDC â€” SVPWM (Space Vector PWM / min-max centering) â”€â”€
    // Zamiast staĹ‚ej bazy PWM_MAX_DUTY/2, przesuwamy wszystkie 3 modulacje razem tak,
    // ĹĽeby mieĹ›ciĹ‚y siÄ™ w zakresie 0..PWM_MAX_DUTY bez klipowania.
    // Offset = PWM_MAX_DUTY/2 - (max+min)/2 to zero-sequence (common-mode) skĹ‚adowa,
    // ktĂłra nie wpĹ‚ywa na napiÄ™cie linia-linia, ale zwiÄ™ksza zakres liniowy
    // o 15.5% (z Vbus*âš3/2 do Vbus) â€” identycznie jak SVPWM.
    //
    // Bez SVPWM: amp > PWM_MAX_DUTY/2 â†’ klipowanie â†’ brak wzrostu napiÄ™cia fundamentalnego
    // Z SVPWM:   amp do ~591 â†’ peĹ‚ny Vbus bez znieksztaĹ‚ceĹ„
    //
    // IR2103: HIN i LIN dostajÄ… TEN SAM duty â†’ komplementarne przeĹ‚Ä…czanie
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
// Komutacja FOC â€” ISR (inverse Park + inverse Clarke + SVPWM)
// ============================================================================

/**
 * @brief Komutacja FOC â€” ISR generuje SVPWM z Vd/Vq przygotowanych w loop().
 *
 * ## KRYTYCZNE: INTEGER-ONLY MATH
 * Timer ISR na ESP32 (level 3 interrupt) NIE zapisuje kontekst FPU.
 * UĹĽycie float w ISR korumpuje rejestry koprocesora â†’ crash (LoadProhibited).
 * CaĹ‚a arytmetyka uĹĽywa int32_t, identycznie jak sinusCommutateISR.
 *
 * ## Algorytm ISR (arytmetyka identyczna z sinusCommutateISR)
 * 1. Advance angle (Hall tracking + interpolacja Q16)
 * 2. Inverse Park: VÎ± = (VdÂ·cos - VqÂ·sin) >> 10, VÎ˛ = (VdÂ·sin + VqÂ·cos) >> 10
 * 3. Inverse Clarke: Va=VÎ±, Vb=(-VÎ±+âš3Â·VÎ˛)/2, Vc=(-VÎ±-âš3Â·VÎ˛)/2
 *    (âš3 â‰ 1774/1024 w Q10)
 * 4. SVPWM min-max centering
 * 5. MCPWM set duty
 *
 * @param hall      Aktualny stan Halla [C:B:A] 1-6
 * @param amplitude Duty z przepustnicy/rampy â€” uĹĽywane tylko jako guard (>SINE_MIN_AMPLITUDE)
 */
static void IRAM_ATTR focCommutateISR(uint8_t hall, uint16_t amplitude) {
    // Walidacja Halla
    if (hall == 0 || hall == 7) {
        allMosfetsOff();
        return;
    }

    // â”€â”€ 1. Stall freeze (identycznie jak SINUS) â”€â”€
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t since_hall = now_ms - g_sine_last_hall_ms;
    uint32_t real_speed = g_sine_speed_q16;
    bool stalled = (since_hall > SINE_STALL_FREEZE_MS);

    // â”€â”€ 2. Advance angle (identycznie jak SINUS) â”€â”€
    if (!stalled) {
        uint32_t spd = real_speed;
        if (spd == 0) spd = SINE_CRAWL_SPEED_Q16;
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

    // â”€â”€ Amplitude guard (PO angle advance!) â”€â”€
    if (amplitude < SINE_MIN_AMPLITUDE) {
        allMosfetsOff();
        return;
    }

    // â”€â”€ 3. Read Vd, Vq from loop() PI controller (int32_t, duty units) â”€â”€
    int32_t vd_i = g_foc_vd_i;
    int32_t vq_i = g_foc_vq_i;

    // â”€â”€ 4. Inverse Park transform (INTEGER-ONLY) â”€â”€
    // sin_val, cos_val: -1024..+1024 (Q10)
    //
    // UWAGA: znaki dopasowane do konwencji kÄ…ta sinusCommutateISR.
    // Standardowy inverse Park to: VÎ± = VdÂ·cos - VqÂ·sin, VÎ˛ = VdÂ·sin + VqÂ·cos
    // ale z naszÄ… tabelÄ… sinusa i mapowaniem faz A=0Â°, B=240Â°, C=120Â°
    // to daje wektor obracajÄ…cy siÄ™ w przeciwnym kierunku niĹĽ SINUS.
    // Weryfikacja numeryczna przy Î¸=0Â°:
    //   SINUS: A=512 B=468 C=556
    //   Poprawiony FOC: A=512 B=468 C=556 âś”
    //
    // VÎ± = (VdÂ·cos + VqÂ·sin) >> 10
    // VÎ˛ = (VdÂ·sin - VqÂ·cos) >> 10
    uint32_t angle = g_sine_angle_q16;
    // FOC: surowy Î¸ (bez offsetu 180Â°!) â€” musi byÄ‡ spĂłjny z forward Park w loop().
    int32_t sin_val = sine_interp_q16(angle);  // -1024..+1024
    uint32_t cos_angle = angle + (24UL << 16); // +90Â° (24/96 entries = 90Â°)
    if (cos_angle >= SINE_TABLE_Q16_FULL) cos_angle -= SINE_TABLE_Q16_FULL;
    int32_t cos_val = sine_interp_q16(cos_angle);

    int32_t v_alpha = (vd_i * cos_val + vq_i * sin_val) >> 10;
    int32_t v_beta  = (vd_i * sin_val - vq_i * cos_val) >> 10;

    // â”€â”€ 5. Inverse Clarke â†’ 3 napiÄ™cia fazowe (INTEGER-ONLY) â”€â”€
    // Va = VÎ±
    // Vb = (-VÎ± + âš3Â·VÎ˛) / 2    [âš3 â‰ 1774/1024 w Q10]
    // Vc = (-VÎ± - âš3Â·VÎ˛) / 2
    int32_t va = v_alpha;
    int32_t sqrt3_vbeta = (1774 * v_beta) >> 10;  // âš3Â·VÎ˛
    int32_t vb = (-v_alpha + sqrt3_vbeta) >> 1;
    int32_t vc = (-v_alpha - sqrt3_vbeta) >> 1;

    // â”€â”€ 6. SVPWM min-max centering (identycznie jak SINUS) â”€â”€
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

// Pomocnicze inline do ustawiania stanu fazy â€” wrapper na MCPWM helpers
/**
 * @brief Faza A: wyjĹ›cie PWM (high-side ON z modulacjÄ…, low-side OFF).
 * @param duty WypeĹ‚nienie PWM 0â€“PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseA_PWM(uint16_t duty) {
    mcpwm_phase_pwm(0, duty);
}
/** @brief Faza A: podĹ‚Ä…czona do GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseA_Low() {
    mcpwm_phase_gnd(0);
}
/** @brief Faza A: pĹ‚ywajÄ…ca (oba tranzystory OFF). */
static inline void IRAM_ATTR phaseA_Off() {
    mcpwm_phase_off(0);
}

/** @brief Faza B: PWM (high-side ON, low-side OFF). @param duty 0â€“PWM_MAX_DUTY. */
static inline void IRAM_ATTR phaseB_PWM(uint16_t duty) {
    mcpwm_phase_pwm(1, duty);
}
/** @brief Faza B: GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseB_Low() {
    mcpwm_phase_gnd(1);
}
/** @brief Faza B: pĹ‚ywajÄ…ca (oba OFF). */
static inline void IRAM_ATTR phaseB_Off() {
    mcpwm_phase_off(1);
}

/** @brief Faza C: PWM (high-side ON, low-side OFF). @param duty 0â€“PWM_MAX_DUTY. */
static inline void IRAM_ATTR phaseC_PWM(uint16_t duty) {
    mcpwm_phase_pwm(2, duty);
}
/** @brief Faza C: GND (high-side OFF, low-side ON). */
static inline void IRAM_ATTR phaseC_Low() {
    mcpwm_phase_gnd(2);
}
/** @brief Faza C: pĹ‚ywajÄ…ca (oba OFF). */
static inline void IRAM_ATTR phaseC_Off() {
    mcpwm_phase_off(2);
}

// ============================================================================
// Funkcje pomocnicze regen â€” Low-side PWM (HS OFF, LS modulowany)
// ============================================================================

/**
 * @brief Faza A: regen PWM (high-side OFF, low-side PWM).
 * @param duty SiĹ‚a hamowania 0â€“PWM_MAX_DUTY (0=brak zwarcia, MAX=peĹ‚ne zwarcie).
 * MCPWM: gen_A forced LOW (HS OFF), gen_B = active-low PWM (LIN=LOW â†’ LS ON).
 */
static inline void IRAM_ATTR phaseA_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(0, duty);
}

/**
 * @brief Faza B: regen PWM (high-side OFF, low-side PWM).
 * @param duty SiĹ‚a hamowania 0â€“PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseB_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(1, duty);
}

/**
 * @brief Faza C: regen PWM (high-side OFF, low-side PWM).
 * @param duty SiĹ‚a hamowania 0â€“PWM_MAX_DUTY.
 */
static inline void IRAM_ATTR phaseC_RegenPWM(uint16_t duty) {
    mcpwm_phase_regen(2, duty);
}

// ============================================================================
// ISR regen â€” komutacja regeneracyjna
// ============================================================================

/**
 * @brief Komutacja regeneracyjna w ISR (low-side boost chopper).
 *
 * Zasada dziaĹ‚ania:
 * - Faza, ktĂłra w motoring miaĹ‚a HS_PWM (ĹşrĂłdĹ‚o) â†’ teraz LS_PWM (regen)
 * - Faza, ktĂłra w motoring miaĹ‚a LS_ON (sink) â†’ LS_ON (bez zmian)
 * - Trzecia faza â†’ float (bez zmian)
 *
 * Cykl PWM regen:
 * 1. PWM ON (LS ON): uzwojenie zwarte przez GND, prÄ…d narasta (L Ĺ‚aduje siÄ™)
 * 2. PWM OFF (LS OFF): prÄ…d indukcyjny pĹ‚ynie przez body diodÄ™ HS â†’ Vbat (Ĺ‚aduje bateriÄ™)
 *
 * @param hall  Stan Halla [C:B:A] 1-6
 * @param regen_duty SiĹ‚a hamowania 0â€“PWM_MAX_DUTY
 *
 * @warning Wszystkie high-side FETy MUSZÄ„ byÄ‡ OFF! Shoot-through = uszkodzenie.
 * @warning Nigdy duty 100% â€” brak fazy OFF = brak transferu do baterii (tylko ciepĹ‚o).
 */
static void IRAM_ATTR regenCommutateISR(uint8_t hall, uint16_t regen_duty) {
    // OdwrĂłcenie kierunku: remap Hall
    uint8_t rh = g_reverse_isr ? g_hall_reverse_map[hall] : hall;

    switch (rh) {
        case 1:  // motoring: A+ B-  â†’  regen: A=LS_PWM, B=LS_ON, C=float
            phaseA_RegenPWM(regen_duty);
            phaseB_Low();
            phaseC_Off();
            break;
        case 3:  // motoring: A+ C-  â†’  regen: A=LS_PWM, B=float, C=LS_ON
            phaseA_RegenPWM(regen_duty);
            phaseB_Off();
            phaseC_Low();
            break;
        case 2:  // motoring: B+ C-  â†’  regen: A=float, B=LS_PWM, C=LS_ON
            phaseA_Off();
            phaseB_RegenPWM(regen_duty);
            phaseC_Low();
            break;
        case 6:  // motoring: B+ A-  â†’  regen: A=LS_ON, B=LS_PWM, C=float
            phaseA_Low();
            phaseB_RegenPWM(regen_duty);
            phaseC_Off();
            break;
        case 4:  // motoring: C+ A-  â†’  regen: A=LS_ON, B=float, C=LS_PWM
            phaseA_Low();
            phaseB_Off();
            phaseC_RegenPWM(regen_duty);
            break;
        case 5:  // motoring: C+ B-  â†’  regen: A=float, B=LS_ON, C=LS_PWM
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
 * @brief Komutacja blokowa 6-step (backup path â€” wywoĹ‚ywana z loop()).
 *
 * Ustawia stany faz A/B/C na podstawie stanu Halla.
 * Ta funkcja NIE jest uĹĽywana w normalnej pracy (ISR obsĹ‚uguje komutacjÄ™).
 * Zachowana jako fallback / do debugowania bez timera.
 *
 * @param hallState 3-bitowy stan Halla [C:B:A], wartoĹ›ci 1â€“6
 * @param duty      WypeĹ‚nienie PWM 0â€“PWM_MAX_DUTY
 *
 * @note JeĹ›li hallState == 0 lub 7 (bĹ‚Ä…d czujnikĂłw) â†’ allMosfetsOff() + fault=true.
 * @note UĹĽywa pomocniczych funkcji phaseX_PWM/Low/Off() (MCPWM wrappers).
 */
void blockCommutate(uint8_t hallState, uint16_t duty) {
    // Walidacja stanu Halla
    if (hallState == HALL_STATE_INVALID_0 || hallState == HALL_STATE_INVALID_7) {
        allMosfetsOff();
        g_bldc_state.fault = true;
        return;
    }

    if (g_bldc_state.mode == DRIVE_MODE_DISABLED) return;  // safety

    // OdwrĂłcenie kierunku: remap Hall
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
// ObsĹ‚uga komend Serial
// ============================================================================

/** @brief Wypisuje tabelÄ™ dostÄ™pnych komend Serial na konsole. */
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
    Serial.println("             Softwarowa zamiana faz â€” dziala we wszystkich trybach.");
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
    Serial.println("b12/b12on  Wlacz BLOCK12 (alias B12/m4)");
    Serial.println("b12off     Powrot do BLOCK (6-step)");
    Serial.println("b12stat    Krotki status BLOCK12: hall, bh, mid_bh, dt, period, half-step");
    Serial.println("b12seq     Pokaz sekwencje i warunki half-step");
    Serial.println("b12dbg     Pelny debug BLOCK12: polkrok, Hall period, timing, sekwencja");
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
 * @brief Wypisuje parametry konfiguracyjne wyĹ›wietlacza P01-P20.
 */
void printDisplayConfig() {
    Serial.println();
    if (!g_display.connected) {
        Serial.println("[S866] WyĹ›wietlacz nie podĹ‚Ä…czony â€” brak parametrĂłw");
        return;
    }
    const s866_config_t& c = g_display.config;
    Serial.println("========== PARAMETRY WYĹšWIETLACZA S866 ==========");
    Serial.printf("Assist level:  %d (raw: %d)\n", g_display.rx.assist_level / 3, g_display.rx.assist_level);
    Serial.println("--- Parametry lokalne wyĹ›wietlacza (nie w ramce) ---");
    Serial.printf("P01  JasnoĹ›Ä‡ podĹ›wietlenia:   [local]\n");
    Serial.printf("P02  Jednostki prÄ™dkoĹ›ci:     [local]\n");
    Serial.printf("P03  NapiÄ™cie systemu:        [local]\n");
    Serial.printf("P04  Auto-wyĹ‚Ä…czenie:         [local]\n");
    Serial.printf("P05  Poziomy wspomagania:     [local]\n");
    Serial.println("--- Parametry z ramki RX ---");
    Serial.printf("P06* Rozmiar koĹ‚a:            %d.%d\"\n",   c.p06_wheel_size_x10 / 10, c.p06_wheel_size_x10 % 10);
    Serial.printf("P07* Pole pairs / magnesy:    %d\n",        c.p07_speed_magnets);
    Serial.printf("P08* Limit prÄ™dkoĹ›ci:         %d km/h\n",   c.p08_speed_limit);
    Serial.printf("P09  Tryb startu:             %s\n",        c.p09_start_mode ? "po pedaĹ‚owaniu" : "od zera");
    Serial.printf("P10* Tryb jazdy:              %d (%s)\n",   c.p10_drive_mode,
        c.p10_drive_mode == 0 ? "PAS+gaz" : c.p10_drive_mode == 1 ? "tylko gaz" : "tylko PAS");
    Serial.printf("P11  CzuĹ‚oĹ›Ä‡ PAS:             %d\n",        c.p11_pas_sensitivity);
    Serial.printf("P12  IntensywnoĹ›Ä‡ startu PAS: %d\n",        c.p12_pas_start_strength);
    Serial.printf("P13* Magnesy PAS:             %d\n",        c.p13_pas_magnets);
    Serial.printf("P14* Limit prÄ…du:             %d A\n",      c.p14_current_limit_a);
    Serial.printf("P15  NapiÄ™cie odciÄ™cia:       %.1f V\n",    c.p15_undervoltage_x10 / 10.0f);
    Serial.println("--- Parametry lokalne wyĹ›wietlacza (nie w ramce) ---");
    Serial.printf("P16  Tryb komunikacji:        [local]\n");
    Serial.printf("P17* WiFi config:             %d (%s)\n",   c.p17_cruise_control, c.p17_cruise_control ? "WiFi ON" : "WiFi OFF");
    Serial.printf("P18  Tryb gazu:               [local]\n");
    Serial.printf("P19  Power Assist:            [local]\n");
    Serial.printf("P20  ProtokĂłĹ‚:                [local]\n");
    Serial.println("     * = parametr uĹĽywany w logice sterownika");
    Serial.println("=================================================");
    uint8_t p07 = g_display.config.p07_speed_magnets;
    if (p07 <= 1) {
        Serial.println("Tryb prÄ™dkoĹ›ci: czujnik SPEED (silnik przekĹ‚adniowy)");
    } else {
        Serial.printf("Tryb prÄ™dkoĹ›ci: Hall Ă— %d pole_pairs (direct-drive)\n", p07);
    }
    Serial.println();
}

// ============================================================================
// Test MOSFET â€” diagnostyka pojedynczych tranzystorĂłw
// ============================================================================

/**
 * @brief WyĹ›wietla pomoc trybu testowego MOSFET.
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
 * WyĹ‚Ä…cza silnik, aktywuje tryb testowy (ISR nie nadpisuje LEDC),
 * ustawia allMosfetsOff() a potem wĹ‚Ä…cza TYLKO wybrany tranzystor.
 *
 * Logika IR2103:
 *   HIGH-side ON: mcpwm_phase_pwm(op, g_mosfet_test_duty)
 *   LOW-side  ON: mcpwm_phase_regen(op, g_mosfet_test_duty)
 *                 â€” LIN=LOW przez X% (wejĹ›cie odwrĂłcone IR2103)
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

    // WyĹ‚Ä…cz silnik i wĹ‚Ä…cz tryb testowy
    g_bldc_state.mode = DRIVE_MODE_DISABLED;
    g_bldc_state.duty_cycle = 0;
    g_bldc_state.duty_target = 0;
    g_duty_ramped = 0;
    g_motor_enabled = false;
    g_mosfet_test_active = true;  // ISR nie bÄ™dzie nadpisywaÄ‡ LEDC
    g_mosfet_test_phase = phase;
    g_mosfet_test_side  = side;

    // Bezpieczny stan â€” wszystko OFF
    allMosfetsOff();

    // WybĂłr kanaĹ‚u i ustawienie PWM (MCPWM)
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
 * @brief Zmienia duty testowe i aktualizuje aktywny test (jeĹ›li trwa).
 *
 * @param pct Procent PWM (1-50)
 * @return Opis wyniku
 */
static String mosfetTestSetDuty(int pct) {
    if (pct < 1)  pct = 1;
    if (pct > 50) pct = 50;
    g_mosfet_test_duty = (uint16_t)((uint32_t)pct * PWM_MAX_DUTY / 100);

    // JeĹ›li test jest aktywny â€” natychmiast zaktualizuj PWM na bieĹĽÄ…cym tranzystorze
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
        snprintf(buf, sizeof(buf), "Test duty: %d%% â€” zaktualizowano Faza %c %s-side",
                 pct, phase, (side == 'H') ? "HIGH" : "LOW");
        return String(buf);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Test duty: %d%% (aktywuj komenda tXY)", pct);
    return String(buf);
}

/**
 * @brief Przetwarza wszystkie dostÄ™pne bajty z bufora Serial.
 *
 * Wszystkie komendy wymagajÄ… Enter. Znaki buforowane do '\n'/'\r',
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
// WspĂłlna obsĹ‚uga komend Serial
// ============================================================================

/**
 * @brief Wykonuje komendÄ™ â€” wspĂłlna logika.
 *
 * Komendy jednoznakowe + komendy konfiguracyjne cfg:param:value.
 *
 * @param cmd Komenda jako String
 * @return Opis wyniku
 */
static String executeCommand(const String& cmd) {
    if (cmd == "e" || cmd == "B") {
        g_mosfet_test_active = false;
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_BLOCK;
        g_bldc_state.fault = false;
        return "BLOCK ON";
    }
    if (cmd == "B12" || cmd == "m4" || cmd == "b12" || cmd == "b12on") {
        g_mosfet_test_active = false;
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_BLOCK12;
        g_bldc_state.fault = false;
        return "BLOCK12 ON";
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
        // NIE resetuj g_manual_duty_override â€” uĹĽytkownik moĹĽe chcieÄ‡
        // rÄ™cznie sterowaÄ‡ duty w FOC (man + d:60).
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
        // Reset EMA prÄ…dĂłw
        g_foc_ia_ema = 0.0f;
        g_foc_ib_ema = 0.0f;
        g_foc_ic_ema = 0.0f;
        g_foc_id_ema = 0.0f;
        g_foc_iq_ema = 0.0f;
        resetSineTracking(g_bldc_state.hall_state);
        return "FOC ON";
    }
    if (cmd == "d") {
        g_mosfet_test_active = false;  // WyĹ‚Ä…cz tryb testowy MOSFET
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
        return g_debugCurrent ? "Debug prÄ…dĂłw: ON (co 500ms)" : "Debug prÄ…dĂłw: OFF";
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
    // â”€â”€ FOC: strojenie i diagnostyka â”€â”€
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
        // StaĹ‚a 30 PWM przy niskim duty (np. 25=5%) dawaĹ‚a Â±120% oscylacjÄ™
        // â†’ silnik saturowaĹ‚ â†’ tiny amplituda â†’ kosmiczne Ku â†’ absurdalne Kp/Ki.
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
        return "[PI Auto-tune] START â€” relay feedback 5s. Silnik moze oscylowac.";
    }
    if (cmd == "man") {
        g_manual_duty_override = !g_manual_duty_override;
        return g_manual_duty_override ? "Manual duty: ON (manetka ignorowana)" : "Manual duty: OFF (manetka aktywna)";
    }
    if (cmd == "assist:auto") {
        g_web_assist_override = -1;
        return "Assist source: AUTO (display/standalone)";
    }
    if (cmd.startsWith("assist:")) {
        int val = cmd.substring(7).toInt();
        if (val >= 0 && val <= 15) {
            g_web_assist_override = (int8_t)val;
            return "Assist override RAW: " + String(val);
        }
        return "Zakres assist: 0-15 lub assist:auto";
    }
    // Strojenie SINE_HALL_PHASE_OFFSET w runtime (1 wpis = 3.75Â° elektr.)
    if (cmd == "so") {
        return "Sine offset: " + String((int)g_sine_hall_phase_offset) + " (" + String((float)g_sine_hall_phase_offset * 3.75f, 1) + "Â°)";
    }
    if (cmd == "so+") {
        int8_t o = g_sine_hall_phase_offset;
        if (o < 47) o += 2;
        g_sine_hall_phase_offset = o;
        config_get().sine_hall_offset = o;
        config_save();
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "Â°) (zapisano)";
    }
    if (cmd == "so-") {
        int8_t o = g_sine_hall_phase_offset;
        if (o > -47) o -= 2;
        g_sine_hall_phase_offset = o;
        config_get().sine_hall_offset = o;
        config_save();
        return "Sine offset: " + String((int)o) + " (" + String((float)o * 3.75f, 1) + "Â°) (zapisano)";
    }
    if (cmd.startsWith("so:")) {
        int val = cmd.substring(3).toInt();
        if (val < -48 || val > 48) return "Zakres: -48..+48 (1 wpis = 3.75Â°)";
        g_sine_hall_phase_offset = (int8_t)val;
        config_get().sine_hall_offset = (int8_t)val;
        config_save();
        return "Sine offset: " + String(val) + " (" + String((float)val * 3.75f, 1) + "Â°) (zapisano)";
    }
    // Auto-tune fazy sinusoidalnej
    if (cmd == "sat" || cmd.startsWith("sat:")) {
        if (g_atune_state != ATUNE_IDLE) {
            g_atune_state = ATUNE_IDLE;
            g_sine_hall_phase_offset = g_atune_saved_offset;
            g_manual_duty_override = false;
            return "[SAT] Anulowano. Offset przywrĂłcony: " + String((int)g_atune_saved_offset);
        }
        if (g_bldc_state.mode != DRIVE_MODE_SINUS && g_bldc_state.mode != DRIVE_MODE_FOC) {
            return "[SAT] Wymaga trybu SINUS lub FOC! UĹĽyj S/F lub m2/m3 aby wĹ‚Ä…czyÄ‡.";
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
                if (mn >= mx) return "[SAT] BĹ‚Ä…d: min >= max";
                g_atune_offset_min = (int8_t)mn;
                g_atune_offset_max = (int8_t)mx;
                g_atune_offset_step = (int8_t)st;
            }
        } else {
            // DomyĹ›lne: -24..+24 krok 2
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
        return "Test MOSFET: OFF â€” wszystkie tranzystory wyĹ‚Ä…czone";
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
        const char* modeNames[] = {"DISABLED", "BLOCK", "SINUS", "FOC", "BLOCK12"};
        const char* modeName = (cfg.drive_mode <= DRIVE_MODE_BLOCK12) ? modeNames[cfg.drive_mode] : "???";
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
        const char* rtMode = (g_bldc_state.mode <= DRIVE_MODE_BLOCK12) ? modeNames[g_bldc_state.mode] : "???";
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
            if (val >= DRIVE_MODE_BLOCK && val <= DRIVE_MODE_BLOCK12) {
                cfg.drive_mode = (uint8_t)val;
                config_save();
                const char* names[] = {"OFF", "BLOCK", "SINUS", "FOC", "BLOCK12"};
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
        // cfg:defaults â€” zaĹ‚aduj wartoĹ›ci domyĹ›lne do runtime (BEZ zapisu do EEPROM)
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
        // cfg:save â€” zsynchronizuj wartoĹ›ci runtime â†’ config i zapisz do EEPROM
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
            // display_required nie ma runtime var â€” zapisuje siÄ™ bezpoĹ›rednio w cfg
            config_save();
            return "[CFG] Aktualne wartosci runtime zapisane do EEPROM.";
        }
        // cfg:reload â€” wczytaj config z EEPROM i zastosuj do runtime
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
            // display_required nie wymaga sync â€” czytany bezpoĹ›rednio z config_get()
            // PrzeĹ‚Ä…cz tryb silnika jeĹ›li skonfigurowany tryb rozni siÄ™ od bieĹĽÄ…cego
            drive_mode_t target_mode = (drive_mode_t)c.drive_mode;
            if (target_mode >= DRIVE_MODE_BLOCK && target_mode <= DRIVE_MODE_BLOCK12
                && target_mode != g_bldc_state.mode) {
                g_bldc_state.mode  = target_mode;
                g_bldc_state.fault = false;
                g_foc_vd_i = 0; g_foc_vq_i = 0;
                g_foc_vd_dbg = 0.0f; g_foc_vq_dbg = 0.0f;
                g_foc_iq_target = 0.0f;
                g_foc_ia_ema = g_foc_ib_ema = g_foc_ic_ema = 0.0f;
                g_foc_last_loop_us = micros();
                resetSineTracking(g_bldc_state.hall_state);
                const char* mnames[] = {"DISABLED","BLOCK","SINUS","FOC","BLOCK12"};
                Serial.printf("[CFG] Tryb przeĹ‚Ä…czony na: %s\n", mnames[target_mode]);
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

    // Numeryczna wartoĹ›Ä‡ duty w %
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

    if (cmd == "b12off") {
        g_mosfet_test_active = false;
        g_manual_duty_override = false;
        g_bldc_state.mode = DRIVE_MODE_BLOCK;
        g_bldc_state.fault = false;
        return "BLOCK12 OFF -> BLOCK ON";
    }

    if (cmd == "b12seq") {
        Serial.println();
        Serial.println("===== BLOCK12 SEKWENCJA =====");
        Serial.printf("CW:  %d > %d > %d > %d > %d > %d\n",
                      g_block_hall_seq[0], g_block_hall_seq[1], g_block_hall_seq[2],
                      g_block_hall_seq[3], g_block_hall_seq[4], g_block_hall_seq[5]);
        Serial.println("HALF-STEP: pierwsza polowa sektora = bh, druga = blockHallNextCw(bh)");
        Serial.printf("Warunek half-step: hall_period >= %u us\n", (unsigned)(2u * HALL_MIN_PERIOD_US));
        Serial.println("=============================");
        Serial.println();
        return "";
    }

    if (cmd == "b12stat") {
        uint8_t hall = g_bldc_state.hall_state;
        uint8_t bh = (hall >= 1 && hall <= 6) ? (g_reverse_isr ? g_hall_reverse_map[hall] : hall) : 0;
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        uint32_t period = g_hall_period_us;
        uint32_t dt_us = (g_hall_last_change_us > 0) ? (now_us - g_hall_last_change_us) : 0;
        uint32_t half_us = period >> 1;
        bool in_half = (period >= (2u * HALL_MIN_PERIOD_US)) && (dt_us >= half_us) && (dt_us < (period * 2u));
        uint8_t mid_bh = (in_half && bh >= 1 && bh <= 6) ? blockHallNextCw(bh) : bh;
        return String("B12 hall=") + String(hall) +
               " bh=" + String(bh) +
               " mid=" + String(mid_bh) +
               " dt=" + String((unsigned long)dt_us) + "us" +
               " T=" + String((unsigned long)period) + "us" +
               (in_half ? " HALF" : " FIRST");
    }

    if (cmd == "b12dbg") {
        uint8_t hall = g_bldc_state.hall_state;
        uint8_t bh = (hall >= 1 && hall <= 6) ? (g_reverse_isr ? g_hall_reverse_map[hall] : hall) : 0;
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        uint32_t period = g_hall_period_us;
        uint32_t dt_us = (g_hall_last_change_us > 0) ? (now_us - g_hall_last_change_us) : 0;
        uint32_t half_us = period >> 1;
        bool in_half = (period >= (2u * HALL_MIN_PERIOD_US)) && (dt_us >= half_us) && (dt_us < (period * 2u));
        uint8_t mid_bh = (in_half && bh >= 1 && bh <= 6) ? blockHallNextCw(bh) : bh;
        float rpm_mech = (period > 0) ? (60000000.0f / (6.0f * (float)period)) : 0.0f;
        Serial.println();
        Serial.println("===== BLOCK12 DEBUG =====");
        Serial.printf("Tryb:         %s%s\n", driveModeName(g_bldc_state.mode),
                      (g_bldc_state.mode == DRIVE_MODE_BLOCK12) ? " (AKTYWNY)" : "");
        Serial.printf("Hall:         %d  bh=%d  mid_bh=%d\n", (int)hall, (int)bh, (int)mid_bh);
        Serial.printf("Polkrok:      %s\n", in_half ? ">> HALF-STEP aktywny (uzywa mid_bh)" : "   FIRST HALF (uzywa bh)");
        Serial.printf("Okres Halla:  %lu us  =>  %.1f RPM mech\n", (unsigned long)period, rpm_mech);
        Serial.printf("Czas od Hall: %lu us  (%.0f%% okresu)\n", (unsigned long)dt_us,
                      (period > 0) ? 100.0f * (float)dt_us / (float)period : 0.0f);
        Serial.printf("Prog polkroku:%lu us  (oczekuje: %lu us)\n", (unsigned long)half_us, (unsigned long)half_us);
        Serial.printf("Sekwencja CW: %d > %d > %d > %d > %d > %d\n",
                      g_block_hall_seq[0], g_block_hall_seq[1], g_block_hall_seq[2],
                      g_block_hall_seq[3], g_block_hall_seq[4], g_block_hall_seq[5]);
        Serial.printf("Odblokowanie: hall_period >= 2x HALL_MIN (%d us): %s\n",
                      HALL_MIN_PERIOD_US * 2,
                      (period >= (2u * HALL_MIN_PERIOD_US)) ? "TAK (polkrok aktywny)" : "NIE (za wolno/za malo danych)");
        Serial.println("=========================");
        Serial.println();
        return "";
    }

    return "Nieznana komenda: " + cmd;
}

// ============================================================================
// Serwer WWW konfiguracji przez WiFi
// WiFi AP: SSID="BLDC_Config", haslo="bldc1234", IP=192.168.4.1
// DostÄ™pny caĹ‚y czas â€” wszystkie uĹĽywane wejĹ›cia analogowe sÄ… na ADC1.
// ============================================================================

static const char BLDC_WIFI_SSID[] = "BLDC_Config";
static const char BLDC_WIFI_PASS[] = "bldc1234";

// Strona HTML w pamieci flash (PROGMEM)
static const char BLDC_WEB_HTML[] PROGMEM = R"bldc_html(<!DOCTYPE html><html lang="pl"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BLDC Control</title><style>
:root{--bg:#efe7d8;--panel:#fff9ee;--panel2:#f5ebd8;--border:#d1c1a4;--text:#2e2b24;--muted:#746f62;--ink:#17140f;--green:#2f8b4d;--red:#b43636;--orange:#b86a1f;--blue:#2f5fa8;--accent:#1f6c5d}
*{box-sizing:border-box;margin:0;padding:0}html{font-size:14px}body{background:radial-gradient(circle at 12% 0%,#fbf6ed 0,#f3e9d7 44%,#e9dcc4 100%);color:var(--text);font-family:"Trebuchet MS",Verdana,sans-serif;line-height:1.5;min-height:100vh}
header{padding:14px 18px;background:linear-gradient(120deg,#20463d,#2f5f53);border-bottom:2px solid #c88c58;display:flex;align-items:center;gap:12px}
header h1{font-size:1rem;font-weight:700;color:#fff6e9;letter-spacing:.02em}header .sub{font-size:.75rem;color:#e6dcca;margin-left:auto}
.wrap{display:grid;grid-template-columns:360px 1fr;gap:1px;background:var(--border);min-height:calc(100vh - 50px)}
@media(max-width:860px){.wrap{grid-template-columns:1fr}}
.col{background:var(--bg);padding:14px;overflow-y:auto}
.hero{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px;margin-bottom:12px}
.stat{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:12px;box-shadow:0 3px 14px rgba(67,52,31,.06)}.stat .k{font-size:.64rem;color:var(--muted);text-transform:uppercase;letter-spacing:.09em;margin-bottom:1px}
.stat .v{font-size:1.5rem;font-weight:700;color:var(--ink);line-height:1.15}.stat .s{font-size:.7rem;color:var(--muted);margin-top:2px}.stat.wide{grid-column:span 2}
.sec{background:var(--panel);border:1px solid var(--border);border-radius:10px;margin-bottom:10px;overflow:hidden;box-shadow:0 3px 14px rgba(67,52,31,.06)}
.sec-h{padding:8px 13px;background:linear-gradient(180deg,#f2e6d0,#e8d8bc);border-bottom:1px solid var(--border);font-size:.75rem;font-weight:700;color:#4e3b20;text-transform:uppercase;letter-spacing:.06em}
.sec-b{padding:11px 13px}
.bbar{display:flex;gap:5px;flex-wrap:wrap}
.btn{padding:6px 13px;border:1px solid #bca98a;border-radius:8px;background:#f7eddc;color:#2f2a22;font-size:.8rem;font-weight:700;cursor:pointer;transition:all .08s}
.btn:hover,.btn:active{filter:brightness(1.06);transform:translateY(-1px)}
.btn-g{background:#dff1df;border-color:#84ba8f;color:#1f6d3b}
.btn-r{background:#f6dcdc;border-color:#d09a9a;color:#8f2525}
.btn-o{background:#f8e6d2;border-color:#d9af82;color:#9a5818}
.btn-b{background:#dfe8f8;border-color:#9fb4dc;color:#234f92}
.abar{display:flex;gap:4px;flex-wrap:wrap;align-items:center}.abar .lbl{font-size:.74rem;color:var(--muted);margin-right:2px}
.abar .btn{min-width:40px;text-align:center}.abar .btn.sel{background:#0e4429;border-color:var(--green);color:var(--green);font-weight:700}
.status-grid{display:grid;grid-template-columns:1fr 1fr;gap:5px;font-size:.78rem}
.status-grid span{display:flex;align-items:center;gap:5px}
.dot{width:7px;height:7px;border-radius:50%;background:var(--muted);flex-shrink:0}
.dot.g{background:var(--green)}.dot.r{background:var(--red)}.dot.o{background:var(--orange)}
.diag{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:10px}
.dc{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:8px 9px}
.dc .dk{font-size:.64rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
.dc .dv{font-size:.95rem;font-weight:700;color:var(--ink);margin-top:1px}
.chart-wrap{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:10px;margin-bottom:10px;box-shadow:0 3px 14px rgba(67,52,31,.06)}
.chart-wrap canvas{display:block;width:100%;height:160px;border-radius:4px}
.cfg-row{display:grid;grid-template-columns:1fr 96px 62px;gap:5px;align-items:center;padding:4px 0;border-bottom:1px solid var(--border)}
.cfg-row:last-child{border-bottom:0}.cfg-row label{font-size:.8rem;color:var(--text)}
.cfg-row input,.cfg-row select{padding:5px 7px;background:#fffdf8;color:var(--text);border:1px solid var(--border);border-radius:6px;font-size:.8rem;width:100%}
.cfg-row .btn{padding:4px 9px;font-size:.75rem;width:100%;text-align:center}
.cmdl{display:flex;gap:5px;margin-top:10px}
.cmdl input{flex:1;padding:7px 10px;background:#fffdf8;color:var(--text);border:1px solid var(--border);border-radius:8px;font-size:.82rem}
#notif{padding:8px 14px;background:linear-gradient(180deg,#f8edd8,#eedfc4);border-top:1px solid var(--border);font-size:.75rem;color:#6d624f;min-height:30px}
</style></head><body>
<header>
<svg viewBox="0 0 20 20" width="20" height="20" fill="none"><rect width="20" height="20" rx="4" fill="#238636"/><path d="M5 11h4V6h2v5h4l-5 4-5-4z" fill="#fff"/></svg>
<h1>BLDC ESP32 Controller</h1>
<div class="sub">BLDC_Config&nbsp;/&nbsp;bldc1234&nbsp;&bull;&nbsp;192.168.4.1</div>
</header>
<div class="wrap">
<div class="col">
<div class="hero">
<div class="stat wide"><div class="k">Predkosc</div><div class="v" id="rt_speed">0.0 <span style="font-size:.85rem">km/h</span></div><div class="s">RPM: <span id="rt_rpm">0</span></div></div>
<div class="stat"><div class="k">Tryb</div><div class="v" style="font-size:1rem" id="rt_mode">-</div><div class="s" id="rt_fault" style="color:var(--green)">OK</div></div>
<div class="stat wide"><div class="k">Moc</div><div class="v" id="rt_power">0 <span style="font-size:.85rem">W</span></div><div class="s">Duty: <span id="rt_duty">0%</span></div></div>
<div class="stat"><div class="k">Bateria</div><div class="v" id="rt_batt">0.0 <span style="font-size:.85rem">V</span></div><div class="s">Thr: <span id="rt_thr">0</span></div></div>
<div class="stat"><div class="k">Assist</div><div class="v" style="font-size:1rem" id="rt_assist">AUTO</div><div class="s">PAS: <span id="rt_pas">-</span></div></div>
<div class="stat"><div class="k">Hall&nbsp;/&nbsp;Temp</div><div class="v" style="font-size:1rem" id="rt_hall">-</div><div class="s"><span id="rt_temp">-</span></div></div>
</div>
<div class="sec"><div class="sec-h">Sterowanie</div><div class="sec-b">
<div class="bbar" style="margin-bottom:7px"><button class="btn btn-r" onclick="sc('d')">&#9209; OFF</button>
<button class="btn" onclick="sc('B')">BLOCK</button>
<button class="btn btn-o" onclick="sc('B12')">BLOCK12</button>
<button class="btn btn-b" onclick="sc('S')">SINUS</button>
<button class="btn btn-b" onclick="sc('F')">FOC</button></div>
<div class="bbar"><button class="btn" onclick="sc('sat')">SAT tune</button>
<button class="btn" onclick="sc('fpitune')">FOC tune</button>
<button class="btn" onclick="sc('fvolt')">fvolt</button>
<button class="btn btn-o" onclick="sc('b12dbg')">B12 debug</button></div>
</div></div>
<div class="sec"><div class="sec-h">Wspomaganie</div><div class="sec-b">
<div class="abar"><span class="lbl">Level:</span>
<button id="ab_auto" class="btn" onclick="sa('auto')">AUTO</button>
<button id="ab_0" class="btn" onclick="sa(0)">0</button>
<button id="ab_3" class="btn" onclick="sa(3)">3</button>
<button id="ab_6" class="btn" onclick="sa(6)">6</button>
<button id="ab_9" class="btn" onclick="sa(9)">9</button>
<button id="ab_12" class="btn" onclick="sa(12)">12</button>
<button id="ab_15" class="btn" onclick="sa(15)">15</button>
</div></div></div>
<div class="sec"><div class="sec-h">Status systemu</div><div class="sec-b">
<div class="status-grid">
<span><span id="dot_disp" class="dot"></span>Display: <strong id="rt_display">-</strong></span>
<span><span id="dot_brake" class="dot"></span>Brake: <strong id="rt_brake">-</strong></span>
<span><span id="dot_regen" class="dot"></span>Regen: <strong id="rt_regen">0 W</strong></span>
<span><span id="dot_pas" class="dot"></span>Offset: <strong id="rt_offset">0</strong></span>
</div>
</div></div>
</div>
<div class="col">
<div class="diag">
<div class="dc"><div class="dk">I faza A</div><div class="dv" id="diag_ia">0.000 A</div></div>
<div class="dc"><div class="dk">I faza B</div><div class="dv" id="diag_ib">0.000 A</div></div>
<div class="dc"><div class="dk">I faza C</div><div class="dv" id="diag_ic">0.000 A</div></div>
<div class="dc"><div class="dk">I limit factor</div><div class="dv" id="diag_ilim">1.000</div></div>
<div class="dc"><div class="dk">FOC Id / Iq</div><div class="dv" id="diag_idiq">- / -</div></div>
<div class="dc"><div class="dk">FOC Vd / Vq</div><div class="dv" id="diag_vdvq">- / -</div></div>
</div>
<div class="chart-wrap">
<div style="font-size:.7rem;color:var(--muted);margin-bottom:5px">Prady fazowe realtime &mdash; <span style="color:#3fb950">&#9632; A &nbsp;</span><span style="color:#79c0ff">&#9632; B &nbsp;</span><span style="color:#d29922">&#9632; C</span></div>
<canvas id="currents" width="800" height="160"></canvas>
</div>
<div class="sec"><div class="sec-h">Konfiguracja &mdash; Silnik</div><div class="sec-b">
<div class="cfg-row"><label>Tryb boot</label><select id="cfg_mode"><option value="1">BLOCK</option><option value="2">SINUS</option><option value="3">FOC</option><option value="4">BLOCK12</option></select><button class="btn" onclick="av('cfg:mode:','cfg_mode')">Ustaw</button></div>
<div class="cfg-row"><label>Rampa [ms]</label><input id="cfg_ramp" type="number" min="0" max="10000" step="100"><button class="btn" onclick="av('cfg:ramp:','cfg_ramp')">Ustaw</button></div>
<div class="cfg-row"><label>Regen</label><select id="cfg_regen"><option value="0">OFF</option><option value="1">ON</option></select><button class="btn" onclick="av('cfg:regen:','cfg_regen')">Ustaw</button></div>
<div class="cfg-row"><label>Max step duty [%]</label><input id="cfg_step" type="number" min="0" max="100"><button class="btn" onclick="av('cfg:step:','cfg_step')">Ustaw</button></div>
<div class="cfg-row"><label>Kierunek</label><select id="cfg_rev"><option value="0">CW</option><option value="1">CCW</option></select><button class="btn" onclick="av('cfg:rev:','cfg_rev')">Ustaw</button></div>
<div class="cfg-row"><label>Offset Hall</label><select id="cfg_so"></select><button class="btn" onclick="av('so:','cfg_so')">Ustaw</button></div>
<div class="cfg-row"><label>Duty min [%]</label><input id="cfg_dutymin" type="number" min="0" max="50"><button class="btn" onclick="av('cfg:dutymin:','cfg_dutymin')">Ustaw</button></div>
</div></div>
<div class="sec"><div class="sec-h">Konfiguracja &mdash; System</div><div class="sec-b">
<div class="cfg-row"><label>Limit pradu [A]</label><input id="cfg_ilim" type="number" min="0" max="50"><button class="btn" onclick="av('cfg:ilim:','cfg_ilim')">Ustaw</button></div>
<div class="cfg-row"><label>Display wymagany</label><select id="cfg_disp"><option value="1">TAK</option><option value="0">NIE</option></select><button class="btn" onclick="av('cfg:dispreq:','cfg_disp')">Ustaw</button></div>
<div class="cfg-row"><label>Throttle samples</label><input id="cfg_thrn" type="number" min="2" max="16"><button class="btn" onclick="av('cfg:thrsamp:','cfg_thrn')">Ustaw</button></div>
<div class="cfg-row"><label>Throttle outlier</label><input id="cfg_thrd" type="number" min="10" max="2000" step="10"><button class="btn" onclick="av('cfg:thrdelta:','cfg_thrd')">Ustaw</button></div>
<div class="cfg-row"><label>Speed pulses/rev</label><input id="cfg_spdppr" type="number" min="1" max="20"><button class="btn" onclick="av('spdppr:','cfg_spdppr')">Ustaw</button></div>
<div class="cfg-row"><label>PWM freq [Hz]</label><input id="cfg_pwm" type="number" min="8000" max="32000" step="1000"><button class="btn" onclick="av('pwmfreq:','cfg_pwm')">Ustaw</button></div>
</div></div>
<div class="sec"><div class="sec-h">Konfiguracja &mdash; PAS</div><div class="sec-b">
<div class="cfg-row"><label>Start delay [ms]</label><input id="cfg_pas_s" type="number" min="0" max="10000" step="100"><button class="btn" onclick="av('passtart:','cfg_pas_s')">Ustaw</button></div>
<div class="cfg-row"><label>Stop delay [ms]</label><input id="cfg_pas_t" type="number" min="100" max="10000" step="100"><button class="btn" onclick="av('passtop:','cfg_pas_t')">Ustaw</button></div>
<div class="cfg-row"><label>Ramp [ms]</label><input id="cfg_pas_r" type="number" min="0" max="10000" step="100"><button class="btn" onclick="av('pasramp:','cfg_pas_r')">Ustaw</button></div>
<div class="cfg-row"><label>Invert</label><select id="cfg_pas_d"><option value="0">Normal</option><option value="1">Invert</option></select><button class="btn" onclick="pd()">Przelacz</button></div>
<div class="cfg-row"><label>Debounce [us]</label><input id="cfg_pas_db" type="number" min="500" max="10000" step="100"><button class="btn" onclick="av('pasdbnc:','cfg_pas_db')">Ustaw</button></div>
<div class="cfg-row"><label>Half-period [ms]</label><input id="cfg_pas_hp" type="number" min="1" max="200"><button class="btn" onclick="av('pashalf:','cfg_pas_hp')">Ustaw</button></div>
<div class="cfg-row"><label>Asymmetry [%]</label><input id="cfg_pas_as" type="number" min="1" max="50"><button class="btn" onclick="av('pasasym:','cfg_pas_as')">Ustaw</button></div>
<div class="cfg-row"><label>Slew rate</label><input id="cfg_pas_sl" type="number" min="1" max="100"><button class="btn" onclick="av('passlew:','cfg_pas_sl')">Ustaw</button></div>
<div class="cfg-row"><label>Holdoff [ms]</label><input id="cfg_pas_fh" type="number" min="50" max="2000" step="50"><button class="btn" onclick="av('pashold:','cfg_pas_fh')">Ustaw</button></div>
</div></div>
<div class="sec"><div class="sec-h">Konfiguracja &mdash; FOC</div><div class="sec-b">
<div class="cfg-row"><label>Kp q+d</label><input id="cfg_fkpq" type="number" min="0" max="100" step="0.01"><button class="btn" onclick="av('fkp:','cfg_fkpq')">Ustaw</button></div>
<div class="cfg-row"><label>Ki q+d</label><input id="cfg_fkiq" type="number" min="0" max="1000" step="0.1"><button class="btn" onclick="av('fki:','cfg_fkiq')">Ustaw</button></div>
<div class="cfg-row"><label>Kp d</label><input id="cfg_fkpd" type="number" min="0" max="100" step="0.01"><button class="btn" onclick="av('fkpd:','cfg_fkpd')">Ustaw</button></div>
<div class="cfg-row"><label>Ki d</label><input id="cfg_fkid" type="number" min="0" max="1000" step="0.1"><button class="btn" onclick="av('fkid:','cfg_fkid')">Ustaw</button></div>
</div></div>
<div class="sec"><div class="sec-h">EEPROM / NVS</div><div class="sec-b">
<div class="bbar"><button class="btn btn-g" onclick="sc('cfg:save')">&#128190; Zapisz EEPROM</button>
<button class="btn" onclick="sc('cfg:reload')">&#128194; Wczytaj</button>
<button class="btn btn-r" onclick="if(confirm('Reset do domyslnych?'))sc('cfg:defaults')">Domyslne</button>
<button class="btn" onclick="lA()">&#8635; Odswiez</button>
</div></div></div>
<div class="cmdl">
<input id="direct_cmd" type="text" placeholder="Komenda: cfg, b12dbg, foc, so:4, pasdbg, ilim:15 ...">
<button class="btn btn-g" onclick="sd()">&#9654; Wyslij</button>
</div>
</div>
</div>
<div id="notif">Ladowanie...</div>
<script>
const ge=id=>document.getElementById(id);let C={},T={};const hA=[],hB=[],hC=[],HM=180;
(()=>{const s=ge('cfg_so');for(let i=-48;i<=48;i+=2){const o=document.createElement('option');o.value=i;o.textContent=i+' ('+(i*3.75).toFixed(1)+'deg)';s.appendChild(o);}})();
function note(m,bad){const n=ge('notif');n.textContent=m;n.style.color=bad?'#b43636':'#6d624f';}
async function pc(cmd){const body=new URLSearchParams({cmd:String(cmd)}).toString();const r=await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded; charset=UTF-8'},body});return r.text();}
async function sc(cmd){try{const t=await pc(cmd);note((t||'OK')+' \u2190 '+cmd,false);await lA();}catch(e){note('Blad: '+e,true);}}
function av(pfx,id){return sc(pfx+ge(id).value);}
function sa(lvl){return sc(lvl==='auto'?'assist:auto':'assist:'+lvl);}
function pd(){const nv=String(C.pas_dir_invert||0)!==ge('cfg_pas_d').value;if(nv)sc('pasdir');else note('PAS dir bez zmian',false);}
function sd(){const i=ge('direct_cmd');const c=i.value.trim();if(!c)return;sc(c).then(()=>i.value='');}
function fmt(v,d=1){return Number(v||0).toFixed(d);}
function tx(id,val){ge(id).textContent=val;}
function ua(){document.querySelectorAll('.abar .btn').forEach(b=>b.classList.remove('sel'));const k='assist_override'in T&&T.assist_override>=0?String(T.assist_override):'auto';const eq=ge('ab_'+k);if(eq)eq.classList.add('sel');}
function drawChart(){const cv=ge('currents'),ctx=cv.getContext('2d'),w=cv.width,h=cv.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#fbf6ea';ctx.fillRect(0,0,w,h);const mx=Math.max(0.1,...hA.map(Math.abs),...hB.map(Math.abs),...hC.map(Math.abs));const sc2=(h-16)/(mx*2),z=h/2;ctx.strokeStyle='#dfcfb2';ctx.lineWidth=1;[.5,1].forEach(f=>{[z-mx*f*sc2,z+mx*f*sc2].forEach(y=>{ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();});});ctx.strokeStyle='#cdbb9c';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,z);ctx.lineTo(w,z);ctx.stroke();[[hA,'#2f8b4d'],[hB,'#2f5fa8'],[hC,'#b86a1f']].forEach(([arr,col])=>{if(arr.length<2)return;ctx.beginPath();ctx.strokeStyle=col;ctx.lineWidth=1.8;arr.forEach((v,i)=>{const x=i*(w-1)/Math.max(1,HM-1),y=z-v*sc2;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});ctx.stroke();});ctx.fillStyle='#7b6d56';ctx.font='10px Trebuchet MS';ctx.fillText(mx.toFixed(1)+'A peak',4,12);}
function push(a,b,c){hA.push(a);hB.push(b);hC.push(c);if(hA.length>HM){hA.shift();hB.shift();hC.shift();}drawChart();}
function rT(){const t=T;tx('rt_mode',t.mode_name||'-');tx('rt_speed',fmt(t.speed_kmh,1));tx('rt_rpm',Math.round(t.rpm||0));tx('rt_power',fmt(t.power_w,1)+' W');tx('rt_duty',(t.duty_pct||0)+'% ('+(t.duty_raw||0)+')');tx('rt_batt',fmt(t.battery_v,2)+' V');tx('rt_thr',t.throttle_raw||0);tx('rt_assist',(t.assist_override>=0?'MAN':'AUTO')+' '+(t.assist_raw||0));tx('rt_pas',(t.pas_rpm||0)+' rpm');tx('rt_hall',t.hall||'-');tx('rt_temp',fmt(t.fet_temp,1)+' C');tx('rt_display',t.display_connected?'ONLINE':'OFFLINE');tx('rt_brake',t.brake?'AKTYWNY':'off');tx('rt_regen',fmt(t.regen_w,1)+' W');tx('rt_offset',String(t.sine_offset||0));tx('diag_ia',fmt(t.phase_a,3)+' A');tx('diag_ib',fmt(t.phase_b,3)+' A');tx('diag_ic',fmt(t.phase_c,3)+' A');tx('diag_ilim',fmt(t.ilim_factor,3));tx('diag_idiq',fmt(t.foc_id,2)+' / '+fmt(t.foc_iq,2));tx('diag_vdvq',fmt(t.foc_vd,2)+' / '+fmt(t.foc_vq,2));const fe=ge('rt_fault');fe.textContent=t.fault?'FAULT':'OK';fe.style.color=t.fault?'#f85149':'#3fb950';ge('dot_disp').className='dot '+(t.display_connected?'g':'r');ge('dot_brake').className='dot '+(t.brake?'o':'');ge('dot_pas').className='dot '+(t.pas_active?'g':'');ge('dot_regen').className='dot '+(t.regen_w>0?'g':'');push(+(t.phase_a||0),+(t.phase_b||0),+(t.phase_c||0));ua();}
function fC(){ge('cfg_mode').value=C.drive_mode??1;ge('cfg_ramp').value=C.ramp_time_ms??1200;ge('cfg_regen').value=C.regen_enabled??0;ge('cfg_step').value=C.duty_max_step_pct??5;ge('cfg_rev').value=C.motor_reverse??0;ge('cfg_so').value=C.sine_hall_offset??0;ge('cfg_dutymin').value=C.duty_min_pct??0;ge('cfg_ilim').value=C.current_limit_a??0;ge('cfg_disp').value=C.display_required??1;ge('cfg_thrn').value=C.thr_samples??8;ge('cfg_thrd').value=C.thr_outlier_thresh??150;ge('cfg_spdppr').value=C.speed_pulses_per_rev??1;ge('cfg_pwm').value=C.pwm_freq_hz??20000;ge('cfg_pas_s').value=C.pas_start_delay_ms??2000;ge('cfg_pas_t').value=C.pas_stop_delay_ms??1000;ge('cfg_pas_r').value=C.pas_ramp_ms??1500;ge('cfg_pas_d').value=C.pas_dir_invert??0;ge('cfg_pas_db').value=C.pas_debounce_us??3000;ge('cfg_pas_hp').value=C.pas_min_halfperiod_ms??5;ge('cfg_pas_as').value=C.pas_dir_asymmetry_pct??5;ge('cfg_pas_sl').value=C.pas_slew_rate??30;ge('cfg_pas_fh').value=C.pas_fwd_holdoff_ms??300;ge('cfg_fkpq').value=Number(C.foc_kp_q??0).toFixed(3);ge('cfg_fkiq').value=Number(C.foc_ki_q??0).toFixed(3);ge('cfg_fkpd').value=Number(C.foc_kp_d??0).toFixed(3);ge('cfg_fkid').value=Number(C.foc_ki_d??0).toFixed(3);}
async function lC(){const r=await fetch('/api/config');C=await r.json();fC();}
async function lT(){const r=await fetch('/api/telemetry');T=await r.json();rT();}
async function lA(){try{await Promise.all([lC(),lT()]);note('OK '+new Date().toLocaleTimeString(),false);}catch(e){note('Blad: '+e,true);}}
setInterval(()=>lT().catch(e=>note('T:'+e,true)),500);
setInterval(()=>lC().catch(e=>note('C:'+e,true)),5000);
lA();
</script></body></html>)bldc_html";

// --- Handlery HTTP ---

static void webHandleRoot() {
    if (g_web_server) g_web_server->send_P(200, "text/html", BLDC_WEB_HTML);
}

static void webHandleApiConfig() {
    if (!g_web_server) return;
    controller_config_t& cfg = config_get();
    char buf[2048];
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
        "\"speed_pulses_per_rev\":%d,\"pwm_freq_hz\":%u,\"duty_min_pct\":%u,"
        "\"assist_override\":%d,\"queued_cmd\":\"%s\"}",
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
        (int)cfg.speed_pulses_per_rev,
        (unsigned)cfg.pwm_freq_hz,
        (unsigned)cfg.duty_min_pct,
        (int)g_web_assist_override,
        g_web_queued_cmd.c_str()
    );
    g_web_server->send(200, "application/json", buf);
}

static void webHandleApiTelemetry() {
    if (!g_web_server) return;
    char buf[1536];
    uint8_t assist_raw = getEffectiveAssistRaw();
    float phase_a = g_bldc_state.phase_current[0];
    float phase_b = g_bldc_state.phase_current[1];
    float phase_c = g_bldc_state.phase_current[2];
    float max_i = fmaxf(phase_a, fmaxf(phase_b, phase_c));
    float power = g_bldc_state.regen_active ? g_bldc_state.regen_power_watts : g_bldc_state.power_watts;
    snprintf(buf, sizeof(buf),
        "{\"mode\":%d,\"mode_name\":\"%s\",\"speed_kmh\":%.2f,\"rpm\":%lu,"
        "\"battery_v\":%.2f,\"power_w\":%.1f,\"regen_w\":%.1f,"
        "\"duty_pct\":%u,\"duty_raw\":%u,\"fault\":%s,\"brake\":%s,"
        "\"display_connected\":%s,\"assist_raw\":%u,\"assist_ui\":%u,\"assist_override\":%d,"
        "\"pas_rpm\":%u,\"pas_active\":%s,\"throttle_raw\":%u,\"hall\":%u,"
        "\"phase_a\":%.3f,\"phase_b\":%.3f,\"phase_c\":%.3f,\"max_i\":%.3f,"
        "\"fet_temp\":%.2f,\"ilim_factor\":%.3f,\"foc_vd\":%.2f,\"foc_vq\":%.2f,"
        "\"foc_id\":%.2f,\"foc_iq\":%.2f,\"fvolt\":%s,\"sine_offset\":%d}",
        (int)g_bldc_state.mode, driveModeName(g_bldc_state.mode), g_bldc_state.wheel_speed_kmh,
        (unsigned long)g_bldc_state.rpm, g_bldc_state.battery_voltage, power, g_bldc_state.regen_power_watts,
        (unsigned int)((uint32_t)g_bldc_state.duty_cycle * 100u / PWM_MAX_DUTY), (unsigned int)g_bldc_state.duty_cycle,
        g_bldc_state.fault ? "true" : "false", g_bldc_state.brake_active ? "true" : "false",
        g_display.connected ? "true" : "false", (unsigned)assist_raw, (unsigned)(assist_raw / 3), (int)g_web_assist_override,
        (unsigned)g_bldc_state.pas_cadence_rpm, g_bldc_state.pas_active ? "true" : "false", (unsigned)g_bldc_state.throttle_raw,
        (unsigned)g_bldc_state.hall_state, phase_a, phase_b, phase_c, max_i, g_bldc_state.fet_temperature,
        g_current_limit_factor, g_foc_vd_dbg, g_foc_vq_dbg, g_foc_id_meas, g_foc_iq_meas,
        g_foc_voltage_mode ? "true" : "false", (int)g_sine_hall_phase_offset
    );
    g_web_server->send(200, "application/json", buf);
}

static uint8_t hexNibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(10 + c - 'a');
    if (c >= 'A' && c <= 'F') return (uint8_t)(10 + c - 'A');
    return 0xFF;
}

static String urlDecodeForm(const String& s) {
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '+') {
            out += ' ';
            continue;
        }
        if (c == '%' && (i + 2) < s.length()) {
            uint8_t hi = hexNibble(s[i + 1]);
            uint8_t lo = hexNibble(s[i + 2]);
            if (hi != 0xFF && lo != 0xFF) {
                out += (char)((hi << 4) | lo);
                i += 2;
                continue;
            }
        }
        out += c;
    }
    return out;
}

static void webHandleApiCmd() {
    if (!g_web_server) return;

    String c;
    if (g_web_server->hasArg("cmd")) {
        c = g_web_server->arg("cmd");
    } else if (g_web_server->hasArg("plain")) {
        String body = g_web_server->arg("plain");
        body.trim();
        if (body.startsWith("cmd=")) {
            c = urlDecodeForm(body.substring(4));
        } else {
            c = body;
        }
    }

    if (c.length() == 0) {
        g_web_server->send(400, "text/plain", "Brak cmd");
        return;
    }

    c.trim();
    if (c.length() == 0) {
        g_web_server->send(400, "text/plain", "Pusta komenda");
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
    if (!g_web_server) return;
    // Captive portal: Android/iOS captive-portal probe -> redirect do strony glownej
    g_web_server->sendHeader("Location", "http://192.168.4.1/", true);
    g_web_server->send(302, "text/plain", "Redirect");
}

/**
 * @brief Uruchamia WiFi AP i serwer HTTP na porcie 80.
 * Nie zatrzymuje silnika â€” uĹĽywamy wyĹ‚Ä…cznie ADC1 dla wejĹ›Ä‡ krytycznych.
 */
static void webConfigInit() {
    if (g_wifi_active) return;

    WiFi.mode(WIFI_AP);
    // Statyczny IP/GW/MASK – stabilne DHCP dla klientów
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    WiFi.softAP(BLDC_WIFI_SSID, BLDC_WIFI_PASS);
    delay(100);  // czekaj aż AP się ustabilizuje
    Serial.printf("[WiFi] AP '%s' uruchomiony, IP: %s\n",
                  BLDC_WIFI_SSID, WiFi.softAPIP().toString().c_str());

    // DNS captive portal – wszystkie domeny -> 192.168.4.1
    // Dzięki temu telefon automatycznie przekieruje na panel
    g_dns_server = new DNSServer();
    g_dns_server->start(53, "*", WiFi.softAPIP());

    g_web_server = new WebServer(80);
    g_web_server->on("/",              HTTP_GET,  webHandleRoot);
    g_web_server->on("/api/config",    HTTP_GET,  webHandleApiConfig);
    g_web_server->on("/api/telemetry", HTTP_GET,  webHandleApiTelemetry);
    g_web_server->on("/api/cmd",       HTTP_GET,  webHandleApiCmd);
    g_web_server->on("/api/cmd",       HTTP_POST, webHandleApiCmd);
    g_web_server->on("/api/queue",     HTTP_POST, webHandleApiQueue);
    // Captive portal: Android/iOS connectivity check trafia tu -> redirect na /
    g_web_server->onNotFound(webHandleNotFound);
    g_web_server->begin();
    g_wifi_active = true;
    Serial.printf("[WiFi] HTTP port 80 + DNS captive portal.\n");
    Serial.printf("[WiFi] Polacz z '%s' (haslo: '%s'), otworz http://192.168.4.1\n",
                   BLDC_WIFI_SSID, BLDC_WIFI_PASS);
}

/**
 * @brief Zatrzymuje serwer HTTP i wylacza WiFi (ADC2 dostepne po powrocie).
 */
static void webConfigStop() {
    if (!g_wifi_active) return;
    if (g_dns_server) {
        g_dns_server->stop();
        delete g_dns_server;
        g_dns_server = nullptr;
    }
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
 * @brief Wywolywany w loop() gdy WiFi aktywne â€” obsluguje klientow HTTP.
 */
static void webConfigHandle() {
    if (g_dns_server) g_dns_server->processNextRequest();
    if (g_web_server) g_web_server->handleClient();
}






