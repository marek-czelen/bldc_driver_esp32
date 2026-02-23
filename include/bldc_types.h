/**
 * @file bldc_types.h
 * @brief Typy i struktury danych dla sterownika BLDC
 * 
 * Definicje metod sterowania silnikiem BLDC:
 * - BLOCK (Trapez/6-step commutation)
 * - SINUS (sinusoidalne sterowanie)
 * - FOC (Field Oriented Control)
 */

#ifndef BLDC_TYPES_H
#define BLDC_TYPES_H

#include <stdint.h>

/**
 * @brief Metody sterowania silnikiem BLDC
 */
typedef enum {
    DRIVE_MODE_DISABLED = 0,    ///< Silnik wyłączony - wszystkie MOSFETy OFF
    DRIVE_MODE_BLOCK,           ///< Komutacja blokowa (trapezoidalna / 6-step)
    DRIVE_MODE_SINUS,           ///< Sterowanie sinusoidalne
    DRIVE_MODE_FOC              ///< Field Oriented Control
} drive_mode_t;

/**
 * @brief Kierunek obrotu silnika
 */
typedef enum {
    DIRECTION_CW = 0,          ///< Zgodnie z ruchem wskazówek zegara
    DIRECTION_CCW               ///< Przeciwnie do ruchu wskazówek zegara
} motor_direction_t;

/**
 * @brief Stan czujników Halla (3-bitowy: CBA)
 * 
 * Czujniki Halla kodują pozycję rotora w 6 krokach (wartości 1-6).
 * Wartości 0 i 7 są nieprawidłowe (oznaczają błąd czujnika).
 */
typedef uint8_t hall_state_t;

#define HALL_STATE_INVALID_0    0   ///< Nieprawidłowy stan czujników
#define HALL_STATE_INVALID_7    7   ///< Nieprawidłowy stan czujników

/**
 * @brief Stan fazy mostka (dla jednej fazy)
 */
typedef enum {
    PHASE_OFF = 0,              ///< Faza wyłączona (oba MOSFETy OFF)
    PHASE_HIGH,                 ///< High-side ON, Low-side OFF
    PHASE_LOW,                  ///< High-side OFF, Low-side ON
    PHASE_PWM_HIGH,             ///< PWM na high-side, Low-side OFF
    PHASE_PWM_LOW               ///< High-side OFF, PWM na low-side
} phase_state_t;

/**
 * @brief Struktura stanu sterownika
 */
typedef struct {
    drive_mode_t    mode;           ///< Aktualny tryb sterowania
    motor_direction_t direction;    ///< Kierunek obrotu
    uint16_t        duty_cycle;     ///< Wypełnienie PWM (0 - PWM_MAX_DUTY)
    uint16_t        throttle_raw;   ///< Surowa wartość przepustnicy (ADC)
    float           battery_voltage;///< Napięcie baterii [V]
    float           phase_current[3]; ///< Prądy fazowe A, B, C [A]
    float           fet_temperature;  ///< Temperatura FET [°C]
    float           motor_temperature;///< Temperatura silnika [°C]
    hall_state_t    hall_state;     ///< Aktualny stan czujników Halla
    uint32_t        rpm;            ///< Obroty silnika [RPM] (mechaniczne koła)
    volatile uint32_t hall_period_us; ///< Czas między przejściami Halla [µs] (z ISR)
    uint16_t        wheeltime_ms;   ///< Czas obrotu koła [ms] (do wyświetlacza)
    float           power_watts;    ///< Aktualna moc pobierana z baterii [W]
    float           regen_power_watts; ///< Moc oddawana do baterii (regeneracja) [W]
    uint16_t        duty_target;    ///< Docelowe duty z przepustnicy (przed rampą)
    uint16_t        ramp_time_ms;   ///< Czas rampy rozpędzania 0→100% [ms]
    bool            brake_active;   ///< Hamulec aktywny
    bool            pas_active;     ///< PAS aktywny
    bool            regen_enabled;  ///< Tryb regeneracji włączony (komenda R)
    bool            regen_active;   ///< Regeneracja aktualnie aktywna (hamulec + warunki OK)
    bool            fault;          ///< Flaga błędu
} bldc_state_t;

#endif // BLDC_TYPES_H
