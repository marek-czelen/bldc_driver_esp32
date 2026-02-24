/**
 * @file bldc_config.h
 * @brief Konfiguracja sterownika BLDC z persystencją w NVS (EEPROM).
 *
 * Struktura konfiguracyjna jest automatycznie zapisywana do NVS (Non-Volatile Storage)
 * ESP32 po każdej zmianie. Przy starcie dane są odczytywane z NVS, a jeśli nie istnieją
 * lub mają nieprawidłową wersję — inicjalizowane wartościami domyślnymi.
 *
 * ## Rozbudowa konfiguracji
 * Aby dodać nowy parametr:
 * 1. Dodaj pole do `controller_config_t` **przed** polem `_reserved[]`
 * 2. Zmniejsz `_reserved[]` o odpowiedni rozmiar
 * 3. Zwiększ `CONFIG_VERSION` o 1
 * 4. W `config_apply_defaults()` ustaw wartość domyślną nowego pola
 * 5. Gotowe — reszta (zapis, odczyt, auto-save) działa automatycznie
 *
 * ## Auto-save
 * Wywołaj `config_save()` po zmianie parametru. Zapis do NVS jest szybki (~1 ms),
 * ale nie należy robić tego w ISR ani w pętli o wysokiej częstotliwości.
 */

#ifndef BLDC_CONFIG_H
#define BLDC_CONFIG_H

#include <stdint.h>
#include "bldc_types.h"

/// Magic number identyfikujący poprawny blok konfiguracji w NVS
#define CONFIG_MAGIC    0x424C4401  // "BLD\x01"

/// Wersja struktury — inkrementuj przy każdej zmianie layoutu
#define CONFIG_VERSION  1

/**
 * @brief Struktura konfiguracji sterownika — persystowana w NVS.
 *
 * Rozmiar jest stały (64 bajty) dzięki polu `_reserved[]`.
 * Przy dodawaniu nowych pól zmniejsz `_reserved` i zwiększ `CONFIG_VERSION`.
 */
typedef struct __attribute__((packed)) {
    uint32_t        magic;              ///< CONFIG_MAGIC — walidacja poprawności
    uint16_t        version;            ///< CONFIG_VERSION — wersja struktury

    // === Parametry konfiguracyjne (dodawaj nowe TU, przed _reserved) ===

    uint8_t         drive_mode;         ///< Domyślny tryb sterowania (drive_mode_t) po starcie
    uint16_t        ramp_time_ms;       ///< Czas rampy rozpędzania 0→100% [ms]
    uint8_t         regen_enabled;      ///< Regeneracja ON/OFF (0/1)

    // === Rezerwa na przyszłe parametry ===
    uint8_t         _reserved[54];      ///< Padding do stałego rozmiaru (64 - użyte bajty)
} controller_config_t;

// Statyczne sprawdzenie rozmiaru (kompilator odmówi jeśli != 64)
static_assert(sizeof(controller_config_t) == 64, "controller_config_t must be 64 bytes");

/**
 * @brief Inicjalizuje system konfiguracji — odczytuje NVS lub tworzy domyślne.
 *
 * Wywołać raz w setup(), przed użyciem konfiguracji.
 * Jeśli NVS zawiera poprawne dane (magic + version) — ładuje je.
 * W przeciwnym razie — zapisuje domyślne wartości.
 */
void config_init();

/**
 * @brief Zapisuje aktualną konfigurację do NVS.
 *
 * Wywołać po każdej zmianie parametru konfiguracyjnego.
 * Bezpieczne do wywołania z loop() (szybkie, ~1 ms).
 * NIE wywoływać z ISR.
 */
void config_save();

/**
 * @brief Zwraca referencję do aktualnej konfiguracji.
 *
 * Można modyfikować pola bezpośrednio, ale po modyfikacji
 * należy wywołać config_save() aby zapisać zmiany.
 *
 * @return Referencja do globalnej struktury konfiguracji.
 */
controller_config_t& config_get();

#endif // BLDC_CONFIG_H
