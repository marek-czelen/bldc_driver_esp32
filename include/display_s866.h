/**
 * @file display_s866.h
 * @brief Obsługa wyświetlacza S866 — protokół nr 2
 *
 * Protokół komunikacyjny z wyświetlaczem e-bike S866 (tzw. "protocol 2").
 * Bazujący na implementacji EBiCS_Firmware (display_No_2.c/h) — GPL v3.
 *
 * ## Opis protokołu
 * Wyświetlacz jest masterem — wysyła ramkę 20 bajtów co ~100 ms.
 * Kontroler odpowiada ramką 14 bajtów z danymi telemetrycznymi.
 * Checksum: XOR wszystkich bajtów oprócz ostatniego (który jest checksumem).
 *
 * ## UART (Serial2 niezależny od USB)
 * - RX2: GPIO16 (PIN_EXT_3) — dane z wyświetlacza
 * - TX2: GPIO4 (PIN_EXT_2) — dane do wyświetlacza
 * - Baud: 9600
 * - UART_EN (GPIO17) steruje TXB0102 level shifter
 *
 * ## Ramka RX (wyświetlacz → kontroler, 20 bajtów)
 * | Bajt | Opis                                    |
 * |------|-----------------------------------------|
 * | 0-2  | Nagłówek                                |
 * | 3    | Throttle mode                           |
 * | 4    | Assist level (0-5, bity 0-3)            |
 * | 5    | Flagi: bit6=ZeroStart, bit5=Headlight, bit1=PushAssist |
 * | 6    | Gear ratio                              |
 * | 7-8  | Rozmiar koła ×10 cali (HiByte, LoByte)  |
 * | 9    | Start delay PAS                         |
 * | 10   | Boost power                             |
 * | 12   | Speed max limit                         |
 * | 13   | Current limit [A]                       |
 * | 14-15| Voltage min ×10 (HiByte, LoByte)        |
 * | 18   | bits0-3=PAS magnets, bit6=CruiseControl |
 * | 19   | Checksum (XOR bajtów 0-18)              |
 *
 * ## Ramka TX (kontroler → wyświetlacz, 14 bajtów)
 * | Bajt | Wartość / Opis                           |
 * |------|------------------------------------------|
 * | 0    | 0x02 (start)                             |
 * | 1    | 0x0E (długość = 14)                      |
 * | 2    | 0x01 (typ)                               |
 * | 3    | Error code                               |
 * | 4    | Brake active (bit 5)                     |
 * | 5    | 0x00                                     |
 * | 6-7  | Prąd ×10 [0.1A] (HiByte, LoByte)        |
 * | 8-9  | Czas obrotu koła [ms] (HiByte, LoByte)  |
 * | 10   | 0x00                                     |
 * | 11   | 0x00                                     |
 * | 12   | 0xFF                                     |
 * | 13   | Checksum (XOR bajtów 0-12)               |
 *
 * ## UART
 * Baud rate: 9600
 * UART_EN (GPIO17) steruje włączeniem konwertera poziomów TXB0102DCU.
 * Linia danych: GPIO1 (TX) / GPIO3 (RX) — współdzielone z UART0 (USB debug).
 * Gdy display mode aktywny, baud rate zmienia się z 115200 na 9600.
 */

#ifndef DISPLAY_S866_H
#define DISPLAY_S866_H

#include <Arduino.h>

// ============================================================================
// Stałe protokołu
// ============================================================================

#define S866_RX_FRAME_LEN   20      ///< Ramka od wyświetlacza
#define S866_TX_FRAME_LEN   14      ///< Ramka do wyświetlacza
#define S866_BAUD_RATE      9600    ///< Prędkość UART wyświetlacza
#define S866_TIMEOUT_MS     2000    ///< Timeout rozłączenia [ms]
#define S866_INTERBYTE_TIMEOUT_MS  50  ///< Timeout między bajtami w ramce [ms]

// ============================================================================
// Struktury danych
// ============================================================================

/**
 * @brief Parametry odebrane od wyświetlacza (ramka RX).
 */
typedef struct {
    uint8_t  assist_level;          ///< Poziom wspomagania 0-5 (raw z bajtu 4, bity 0-3)
    uint8_t  headlight;             ///< Światło przednie 0/1
    uint8_t  push_assist;           ///< Asystent pchania 0/1
    uint8_t  zero_start;            ///< Start od zera 0/1
    uint8_t  cruise_control;        ///< Tempomat 0/1
    uint8_t  throttle_mode;         ///< Tryb przepustnicy (bajt 3)
    uint8_t  gear_ratio;            ///< Przełożenie (bajt 6)
    uint8_t  start_delay_pas;       ///< Opóźnienie startu PAS (bajt 9)
    uint8_t  boost_power;           ///< Moc boost (bajt 10)
    uint8_t  speed_max_limit;       ///< Limit prędkości max (bajt 12)
    uint8_t  current_limit_a;       ///< Limit prądu [A] (bajt 13)
    uint16_t voltage_min_x10;       ///< Napięcie minimalne ×10 [V] (bajty 14-15)
    uint16_t wheel_size_inch_x10;   ///< Rozmiar koła ×10 cali (bajty 7-8)
    uint8_t  num_pas_magnets;       ///< Liczba magnesów PAS (bajt 18, bity 0-3)
} s866_rx_params_t;

/**
 * @brief Kopia parametrów konfiguracyjnych wyświetlacza P01-P20.
 *
 * Parametry P01-P20 to ustawienia z menu konfiguracyjnego wyświetlacza S866.
 * Kontroler przechowuje ich kopię, aby móc się nimi posługiwać
 * (np. rozmiar koła do przeliczenia prędkości, limit prądu itp.).
 * Aktualizowane po każdej poprawnie odebranej ramce RX.
 */
typedef struct {
    uint8_t  p01_backlight;         ///< P01: Jasność podświetlenia 1-3
    uint8_t  p02_units;             ///< P02: 0=km/h, 1=mph
    uint8_t  p03_voltage;           ///< P03: Napięcie systemu (24/36/48/52/60 V)
    uint8_t  p04_auto_off_min;      ///< P04: Czas auto-wyłączenia 0-60 min
    uint8_t  p05_assist_levels;     ///< P05: Liczba poziomów wspomagania (3 lub 5)
    uint16_t p06_wheel_size_x10;    ///< P06: Rozmiar koła ×10 [cale] (np. 260=26.0")
    uint8_t  p07_speed_magnets;     ///< P07: Liczba magnesów czujnika prędkości 1-100
    uint8_t  p08_speed_limit;       ///< P08: Limit prędkości (0=brak)
    uint8_t  p09_start_mode;        ///< P09: 0=start od zera, 1=start po pedałowaniu
    uint8_t  p10_drive_mode;        ///< P10: 0=PAS+gaz, 1=tylko gaz, 2=tylko PAS
    uint8_t  p11_pas_sensitivity;   ///< P11: Czułość PAS 1-24
    uint8_t  p12_pas_start_strength;///< P12: Intensywność startu PAS 0-5
    uint8_t  p13_pas_magnets;       ///< P13: Liczba magnesów PAS 1-100
    uint8_t  p14_current_limit_a;   ///< P14: Limit prądu sterownika [A]
    uint16_t p15_undervoltage_x10;  ///< P15: Napięcie odcięcia baterii ×10 [V]
    uint8_t  p16_comm_mode;         ///< P16: Tryb komunikacji / ODO reset
    uint8_t  p17_cruise_control;    ///< P17: Tempomat 0=off, 1=on
    uint8_t  p18_throttle_mode;     ///< P18: Definicja gazu
    uint8_t  p19_power_assist;      ///< P19: Power Assist Ratio
    uint8_t  p20_protocol;          ///< P20: Protokół komunikacji
} s866_config_t;

/**
 * @brief Parametry wysyłane do wyświetlacza (ramka TX).
 */
typedef struct {
    uint8_t  error;                 ///< Kod błędu (0 = OK)
    uint8_t  brake_active;          ///< Hamulec aktywny 0/1
    uint16_t current_x10;           ///< Prąd ×10 [0.1 A]
    uint16_t wheeltime_ms;          ///< Czas obrotu koła [ms] (0 = stoi)
} s866_tx_params_t;

/**
 * @brief Kontekst wyświetlacza S866 — cały stan komunikacji.
 */
typedef struct {
    uint8_t          rx_buf[S866_RX_FRAME_LEN];  ///< Bufor odbioru ramki
    uint8_t          rx_count;                    ///< Liczba odebranych bajtów
    s866_rx_params_t rx;                          ///< Sparsowane parametry RX
    s866_tx_params_t tx;                          ///< Parametry do wysłania TX
    s866_config_t    config;                      ///< Kopia parametrów P01-P20
    bool             connected;                   ///< Wyświetlacz podłączony?
    unsigned long    last_valid_ms;               ///< Czas ostatniej poprawnej ramki
    unsigned long    last_byte_ms;                ///< Czas ostatniego odebranego bajtu
} s866_display_t;

// ============================================================================
// Funkcje publiczne
// ============================================================================

/**
 * @brief Włącza tryb wyświetlacza: UART_EN HIGH, baud 9600.
 */
void s866_init();

/**
 * @brief Wyłącza tryb wyświetlacza: UART_EN LOW, baud 115200.
 */
void s866_deinit();

/**
 * @brief Obsługa komunikacji z wyświetlaczem — wywoływać w loop().
 *
 * Czyta bajty z Serial, parsuje ramki, wysyła odpowiedzi.
 * Ustawia ctx->connected na true/false w zależności od komunikacji.
 *
 * @param ctx Kontekst wyświetlacza
 */
void s866_service(s866_display_t* ctx);

#endif // DISPLAY_S866_H
