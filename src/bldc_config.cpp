/**
 * @file bldc_config.cpp
 * @brief Implementacja persystencji konfiguracji w NVS (Preferences).
 *
 * Używa biblioteki Preferences (wbudowana w ESP32 Arduino) do zapisu/odczytu
 * struktury controller_config_t jako blobu binarnego w partycji NVS.
 *
 * Namespace: "bldc" — klucz: "cfg"
 */

#include "bldc_config.h"
#include <Preferences.h>

/// Globalny obiekt konfiguracji
static controller_config_t g_config;

/// Obiekt Preferences (NVS)
static Preferences g_prefs;

/// Namespace NVS
static const char* NVS_NAMESPACE = "bldc";
/// Klucz NVS dla binarnego blobu konfiguracji
static const char* NVS_KEY = "cfg";

/**
 * @brief Wypełnia konfigurację wartościami domyślnymi.
 *
 * Wywoływana gdy NVS jest puste lub wersja się nie zgadza.
 * Przy dodawaniu nowych parametrów — dodaj tu ich domyślne wartości.
 */
static void config_apply_defaults() {
    memset(&g_config, 0, sizeof(g_config));
    g_config.magic          = CONFIG_MAGIC;
    g_config.version        = CONFIG_VERSION;
    g_config.drive_mode     = (uint8_t)DRIVE_MODE_BLOCK;   // Po starcie: tryb BLOCK
    g_config.ramp_time_ms   = 1200;                         // 1200 ms rampa
    g_config.regen_enabled  = 0;                             // Regen domyślnie wyłączony
    g_config.pas_dir_invert = 0;                              // Kierunek PAS: normalny
    g_config.pas_start_delay_ms = 2000;                        // 2s ciągłego pedałowania do aktywacji
    g_config.pas_stop_delay_ms  = 1000;                        // 1s bez impulsów → stop
    g_config.pas_ramp_ms        = 1500;                        // 1.5s soft-start 0→100%
    g_config.duty_max_step_pct  = 5;                           // Max 5% duty change per loop call
    g_config.motor_reverse      = 0;                             // Kierunek: 0=CW (domyślny), 1=CCW
    g_config.sine_hall_offset   = 0;                             // Offset Hall→sine: 0 wpisów (0°)
    g_config.foc_kp_q           = 0.5f;                          // FOC Kp domyślne (PWM/A)
    g_config.foc_ki_q           = 5.0f;                          // FOC Ki domyślne (PWM/A/s)
    g_config.foc_kp_d           = 0.5f;                          // FOC Kp_d domyślne
    g_config.foc_ki_d           = 5.0f;                          // FOC Ki_d domyślne
    g_config.foc_voltage_mode   = 0;                              // FOC Voltage mode domyślnie OFF
    g_config.pas_debounce_us    = 3000;                            // Debounce PAS: 3ms (filtr szpilek EMI)
    g_config.display_required   = 1;                               // Silnik tylko z wyświetlaczem (domyślnie)
    g_config.thr_samples         = 8;                               // Throttle: 8 próbek burst
    g_config.thr_outlier_thresh  = 150;                             // Throttle: odrzucaj > 150 od mediany
    g_config.current_limit_a     = 15;                              // Limit prądu: 15A domyślnie (0=brak limitu)
    g_config.pas_min_halfperiod_ms = 5;                               // Min półokres PAS: 5 ms
    g_config.pas_dir_asymmetry_pct = 5;                               // Próg asymetrii kierunku PAS: 5%
    g_config.pas_slew_rate         = 30;                              // Slew rate PAS: 30 (~6% PWM/step)
    g_config.pas_fwd_holdoff_ms    = 300;                             // Holdoff reverse PAS: 300 ms
    g_config.speed_pulses_per_rev  = 1;                               // Impulsy SPEED na obrót: 1 (kalibracja: spdcal)
    g_config.pwm_freq_hz           = 20000;                           // Częstotliwość PWM: 20 kHz (domyślna)
    g_config.duty_min_pct          = 10;                              // Min duty: 10% (poniżej → 0)
    g_config.startup_boost_pct     = 50;                              // Boost na starcie: +50% duty przy 0 RPM
    g_config.startup_boost_rpm     = 80;                              // Boost zanika liniowo do 0 przy 80 RPM
    g_config.fb_wheel_size_x10    = 260;                              // Fallback P06: 26" koło
    g_config.fb_speed_magnets     = 1;                                // Fallback P07: 1 (zewn. czujnik SPEED)
    g_config.fb_speed_limit       = 25;                               // Fallback P08: 25 km/h
    g_config.fb_drive_mode        = 0;                                // Fallback P10: PAS+gaz
    g_config.fb_pas_magnets       = 12;                               // Fallback P13: 12 magnesów PAS
    g_config.assist_min_speed_kmh = 6;                                // Min prędkość przy assist=1: 6 km/h
}

void config_init() {
    g_prefs.begin(NVS_NAMESPACE, false);  // false = read-write

    size_t len = g_prefs.getBytesLength(NVS_KEY);

    if (len == sizeof(controller_config_t)) {
        // Blob istnieje i ma poprawny rozmiar — odczytaj
        g_prefs.getBytes(NVS_KEY, &g_config, sizeof(g_config));

        // Walidacja magic i version
        if (g_config.magic == CONFIG_MAGIC && g_config.version == CONFIG_VERSION) {
            Serial.printf("[CFG] Konfiguracja załadowana z NVS (v%d)\n", g_config.version);
            return;
        }
        // Nieprawidłowa wersja — reset do domyślnych
        Serial.printf("[CFG] Wersja NVS (%d) != oczekiwana (%d) — reset do domyślnych\n",
                       g_config.version, CONFIG_VERSION);
    } else {
        Serial.println("[CFG] Brak konfiguracji w NVS — tworzę domyślną");
    }

    // Pierwsz uruchomienie lub migracja — ustaw domyślne i zapisz
    config_apply_defaults();
    config_save();
}

void config_save() {
    // Upewnij się że magic/version są poprawne
    g_config.magic   = CONFIG_MAGIC;
    g_config.version = CONFIG_VERSION;

    size_t written = g_prefs.putBytes(NVS_KEY, &g_config, sizeof(g_config));
    if (written == sizeof(g_config)) {
        Serial.println("[CFG] Zapisano do NVS");
    } else {
        Serial.printf("[CFG] BŁĄD zapisu NVS! (zapisano %d/%d bajtów)\n",
                       written, sizeof(g_config));
    }
}

controller_config_t& config_get() {
    return g_config;
}
