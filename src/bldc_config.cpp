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
