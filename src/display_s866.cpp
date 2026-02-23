/**
 * @file display_s866.cpp
 * @brief Implementacja protokołu wyświetlacza S866 ("protocol 2")
 *
 * Bazuje na implementacji EBiCS_Firmware (display_No_2.c) — GPL v3.
 * Zaadaptowane na ESP32 Arduino (bez DMA, polling Serial).
 *
 * ## Synchronizacja ramek
 * Używa sliding-window z checksumem XOR:
 * 1. Zbieramy bajty do bufora 20-bajtowego
 * 2. Po zapełnieniu sprawdzamy checksum (XOR bytes 0-18 == byte 19)
 * 3. Jeśli poprawny → parsujemy i odpowiadamy
 * 4. Jeśli nie → przesuwamy bufor o 1 bajt (memmove) i czekamy na kolejny
 * 5. Timeout międzybajtowy (50 ms) resetuje bufor → resync
 */

#include "display_s866.h"
#include "pinout.h"

// ============================================================================
// Funkcje wewnętrzne
// ============================================================================

/**
 * @brief Oblicza checksum XOR dla ramki protokołu S866.
 *
 * XOR wszystkich bajtów od 0 do (len-2). Wynik porównywany z bajtem (len-1).
 *
 * @param buf   Wskaźnik na bufor ramki
 * @param len   Całkowita długość ramki (włącznie z bajtem checksum)
 * @return      Obliczony checksum
 */
static uint8_t s866_calc_checksum(const uint8_t* buf, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/**
 * @brief Parsuje odebraną ramkę 20-bajtową z wyświetlacza.
 *
 * Odczytuje parametry z poszczególnych bajtów ramki i zapisuje do struktury rx.
 * Mapowanie bajtów zgodne z protokołem nr 2 (patrz display_s866.h).
 *
 * @param ctx   Kontekst wyświetlacza
 */
static void s866_parse_rx_frame(s866_display_t* ctx) {
    const uint8_t* f = ctx->rx_buf;

    ctx->rx.assist_level        = f[4] & 0x0F;       // 0-15 (zazwyczaj 0-5 na wyświetlaczu)
    ctx->rx.throttle_mode       = f[3];
    ctx->rx.headlight           = (f[5] >> 5) & 0x01;
    ctx->rx.push_assist         = (f[5] >> 1) & 0x01;
    ctx->rx.zero_start          = (f[5] >> 6) & 0x01;
    ctx->rx.gear_ratio          = f[6];
    ctx->rx.wheel_size_inch_x10 = ((uint16_t)f[7] << 8) | f[8];
    ctx->rx.start_delay_pas     = f[9];
    ctx->rx.boost_power         = f[10];
    ctx->rx.speed_max_limit     = f[12];
    ctx->rx.current_limit_a     = f[13];
    ctx->rx.voltage_min_x10     = ((uint16_t)f[14] << 8) | f[15];
    ctx->rx.num_pas_magnets     = f[18] & 0x0F;
    ctx->rx.cruise_control      = (f[18] >> 6) & 0x01;

    // Aktualizacja kopii parametrów konfiguracyjnych P01-P20
    // Mapowanie z pól ramki RX na parametry P
    // UWAGA: P01-P04, P16, P18-P20 nie są transmitowane w ramce —
    //        to ustawienia lokalne wyświetlacza.
    ctx->config.p05_assist_levels      = 5;  // protokół 2 zawsze 5 poziomów
    ctx->config.p06_wheel_size_x10     = ctx->rx.wheel_size_inch_x10;  // f[7-8]
    ctx->config.p07_speed_magnets      = ctx->rx.gear_ratio;           // f[6] = P07
    ctx->config.p08_speed_limit        = ctx->rx.speed_max_limit;      // f[12]
    ctx->config.p09_start_mode         = ctx->rx.zero_start;           // f[5] bit6
    ctx->config.p10_drive_mode         = ctx->rx.throttle_mode;        // f[3] = P10
    ctx->config.p11_pas_sensitivity    = ctx->rx.start_delay_pas;      // f[9]
    ctx->config.p12_pas_start_strength = ctx->rx.boost_power;          // f[10]
    ctx->config.p13_pas_magnets        = ctx->rx.num_pas_magnets;      // f[18] bits 0-3
    ctx->config.p14_current_limit_a    = ctx->rx.current_limit_a;      // f[13]
    ctx->config.p15_undervoltage_x10   = ctx->rx.voltage_min_x10;      // f[14-15]
    ctx->config.p17_cruise_control     = ctx->rx.cruise_control;       // f[18] bit6
}

/**
 * @brief Wysyła ramkę odpowiedzi (14 bajtów) do wyświetlacza.
 *
 * Format ramki TX:
 * {0x02, 0x0E, 0x01, error, brake<<5, 0x00, currentH, currentL,
 *  wheeltimeH, wheeltimeL, 0x00, 0x00, 0xFF, checksum}
 *
 * @param ctx   Kontekst wyświetlacza z wypełnionymi polami tx
 */
static void s866_send_response(s866_display_t* ctx) {
    uint8_t tx[S866_TX_FRAME_LEN] = {
        0x02,                                        // [0]  Start byte
        0x0E,                                        // [1]  Length = 14
        0x01,                                        // [2]  Type
        ctx->tx.error,                               // [3]  Error code
        (uint8_t)(ctx->tx.brake_active << 5),        // [4]  Brake flag (bit 5)
        0x00,                                        // [5]
        (uint8_t)(ctx->tx.current_x10 >> 8),         // [6]  Current high byte
        (uint8_t)(ctx->tx.current_x10 & 0xFF),       // [7]  Current low byte
        (uint8_t)(ctx->tx.wheeltime_ms >> 8),         // [8]  Wheeltime high byte
        (uint8_t)(ctx->tx.wheeltime_ms & 0xFF),       // [9]  Wheeltime low byte
        0x00,                                        // [10]
        0x00,                                        // [11]
        0xFF,                                        // [12]
        0x00                                         // [13] Checksum (obliczany poniżej)
    };
    tx[S866_TX_FRAME_LEN - 1] = s866_calc_checksum(tx, S866_TX_FRAME_LEN);

    Serial2.write(tx, S866_TX_FRAME_LEN);
}

// ============================================================================
// Funkcje publiczne
// ============================================================================

void s866_init() {
    // Włącz konwerter poziomów TXB0102DCU
    digitalWrite(PIN_UART_EN, HIGH);

    // Inicjalizuj Serial2 (GPIO4=TX2, GPIO16=RX2)
    Serial2.end();  // Upewnij się że wcześniej jest zamknięty
    delay(10);
    Serial2.begin(S866_BAUD_RATE, SERIAL_8N1, 16, 4);  // RX=GPIO16, TX=GPIO4
}

void s866_deinit() {
    // Wyłącz konwerter poziomów
    digitalWrite(PIN_UART_EN, LOW);

    // Zamknij Serial2
    Serial2.end();
    delay(10);
}

void s866_service(s866_display_t* ctx) {
    unsigned long now = millis();

    // --- Odbiór bajtów i synchronizacja ramki ---
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        ctx->last_byte_ms = now;

        // Dodaj bajt do bufora
        if (ctx->rx_count < S866_RX_FRAME_LEN) {
            ctx->rx_buf[ctx->rx_count++] = b;
        }

        // Gdy mamy pełną ramkę (20 bajtów) — sprawdź checksum
        if (ctx->rx_count >= S866_RX_FRAME_LEN) {
            uint8_t expected = s866_calc_checksum(ctx->rx_buf, S866_RX_FRAME_LEN);

            if (ctx->rx_buf[S866_RX_FRAME_LEN - 1] == expected) {
                // ✓ Poprawna ramka — parsuj i odpowiedz
                s866_parse_rx_frame(ctx);
                ctx->connected = true;
                ctx->last_valid_ms = now;

                // Wyślij odpowiedź (TX params muszą być zaktualizowane wcześniej w loop)
                s866_send_response(ctx);

                // Reset bufora na następną ramkę
                ctx->rx_count = 0;
            } else {
                // ✗ Zły checksum — przesuń bufor o 1 bajt (sliding window resync)
                memmove(ctx->rx_buf, ctx->rx_buf + 1, S866_RX_FRAME_LEN - 1);
                ctx->rx_count = S866_RX_FRAME_LEN - 1;
            }
        }
    }

    // --- Timeout międzybajtowy: niepełna ramka → reset bufora ---
    if (ctx->rx_count > 0 && (now - ctx->last_byte_ms > S866_INTERBYTE_TIMEOUT_MS)) {
        ctx->rx_count = 0;
    }

    // --- Timeout połączenia: brak poprawnych ramek → rozłączony ---
    if (ctx->connected && (now - ctx->last_valid_ms > S866_TIMEOUT_MS)) {
        ctx->connected = false;
    }
}
