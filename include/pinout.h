/**
 * @file pinout.h
 * @brief Definicje pinów ESP32 dla sterownika BLDC
 * 
 * Sterowniki MOSFET: IR2103 (x3 - po jednym na fazę)
 * IR2103 - sterowanie:
 *   HIN: wejście nieodwracające (active HIGH) -> HIN=1 => HO=1 (high-side MOSFET ON)
 *   LIN: wejście odwracające  (active LOW)  -> LIN=0 => LO=1 (low-side MOSFET ON)
 * 
 * Wszystkie piny mają unikalne przypisania.
 */

#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// ============================================================================
// Pomiary napięcia i prądu (ADC)
// ============================================================================

/** @brief Pomiar napięcia baterii (ADC1_CH0) */
#define PIN_BATTERY_VOLTAGE     36  // GPIO36 (VP) - tylko wejście, ADC1_CH0

/** @brief Pomiar prądu fazy A (ADC1_CH3) */
#define PIN_PHASE_A_CURRENT     39  // GPIO39 (VN) - tylko wejście, ADC1_CH3

/** @brief Pomiar prądu fazy B (ADC1_CH6) */
#define PIN_PHASE_B_CURRENT     34  // GPIO34 - tylko wejście, ADC1_CH6

/** @brief Pomiar prądu fazy C (ADC1_CH7) */
#define PIN_PHASE_C_CURRENT     35  // GPIO35 - tylko wejście, ADC1_CH7

// ============================================================================
// Wyjścia PWM - Sterowanie mostkiem H (IR2103 x3)
// ============================================================================
// IR2103:
//   HIN (HIGH-side input): stan WYSOKI = MOSFET high-side WŁĄCZONY
//   LIN (LOW-side input):  stan NISKI  = MOSFET low-side WŁĄCZONY (wejście odwrócone!)
//
// Aby włączyć high-side MOSFET: ustaw pin PWM_x_HIGH = HIGH
// Aby wyłączyć high-side MOSFET: ustaw pin PWM_x_HIGH = LOW
// Aby włączyć low-side MOSFET: ustaw pin PWM_x_LOW = LOW  (LIN jest odwrócony!)
// Aby wyłączyć low-side MOSFET: ustaw pin PWM_x_LOW = HIGH (LIN jest odwrócony!)

/** @brief Faza A - sterowanie high-side MOSFET (IR2103 HIN) */
#define PIN_PWM_A_HIGH          12  // GPIO12

/** @brief Faza A - sterowanie low-side MOSFET (IR2103 LIN - odwrócony!) */
#define PIN_PWM_A_LOW           13  // GPIO13

/** @brief Faza B - sterowanie high-side MOSFET (IR2103 HIN) */
#define PIN_PWM_B_HIGH          25  // GPIO25

/** @brief Faza B - sterowanie low-side MOSFET (IR2103 LIN - odwrócony!) */
#define PIN_PWM_B_LOW           26  // GPIO26

/** @brief Faza C - sterowanie high-side MOSFET (IR2103 HIN) */
#define PIN_PWM_C_HIGH          27  // GPIO27

/** @brief Faza C - sterowanie low-side MOSFET (IR2103 LIN - odwrócony!) */
#define PIN_PWM_C_LOW           14  // GPIO14

// ============================================================================
// Stany bezpieczne dla IR2103
// ============================================================================
// Wszystkie MOSFETy wyłączone:
//   HIN = LOW  (high-side OFF)
//   LIN = HIGH (low-side OFF, bo wejście odwrócone)

#define IR2103_HIN_OFF          LOW     // High-side MOSFET wyłączony
#define IR2103_HIN_ON           HIGH    // High-side MOSFET włączony
#define IR2103_LIN_OFF          HIGH    // Low-side MOSFET wyłączony (odwrócone!)
#define IR2103_LIN_ON           LOW     // Low-side MOSFET włączony (odwrócone!)

// ============================================================================
// Czujniki temperatury (ADC)
// ============================================================================

/** @brief Czujnik temperatury tranzystorów FET (ADC1_CH4) */
#define PIN_FET_TEMP            32  // GPIO32 - ADC1_CH4

// ============================================================================
// Wejścia sterujące
// ============================================================================

/** @brief Wejście przepustnicy / gazu (ADC1_CH5) */
#define PIN_THROTTLE            33  // GPIO33 - ADC1_CH5

/** @brief Wejście czujnika PAS (Pedal Assist Sensor) */
#define PIN_PAS                 22  // GPIO22

/** @brief Wejście hamulca */
#define PIN_BRAKE               23  // GPIO23

// ============================================================================
// Czujniki Halla
// ============================================================================

/** @brief Czujnik Halla A */
#define PIN_HALL_SENSOR_A       4   // GPIO4

/** @brief Czujnik Halla B */
#define PIN_HALL_SENSOR_B       18  // GPIO18

/** @brief Czujnik Halla C */
#define PIN_HALL_SENSOR_C       19  // GPIO19

// ============================================================================
// Wejście czujnika prędkości
// ============================================================================

/** @brief Wejście czujnika prędkości (aktywne przy P07<=1, silnik przekładniowy) */
#define PIN_SPEED               21  // GPIO21

// ============================================================================
// UART
// ============================================================================

/** @brief UART RX (3.3V) */
#define PIN_UART_RX             16  // GPIO16

/** @brief UART TX (3.3V) */
#define PIN_UART_TX             17  // GPIO17

/** @brief UART Enable */
#define PIN_UART_EN             5   // GPIO5

// ============================================================================
// Wyprowadzenia rozszerzeń
// ============================================================================

/** @brief Rozszerzenie 1 */
#define PIN_EXT_1               0   // GPIO0 (UWAGA: boot pin)

/** @brief Rozszerzenie 2 */
#define PIN_EXT_2               2   // GPIO2

/** @brief Rozszerzenie 3 */
#define PIN_EXT_3               15  // GPIO15

// ============================================================================
// Konfiguracja PWM (MCPWM — center-aligned complementary)
// ============================================================================
//
// Architektura MCPWM:
//   MCPWM_UNIT_0 z 3 operatorami, UP_DOWN counter (center-aligned / symmetric).
//   Każdy operator steruje parą HIN/LIN jednej fazy (komplementarnie + dead time).
//   Wszystkie 3 timery zsynchronizowane → fazowe centrum w tym samym momencie.
//   ADC trigger: ISR na TEZ (Timer Equals Zero) = wszystkie low-side ON = okno pomiarowe.
//
// IR2103: HIN=active HIGH, LIN=active LOW
//   gen_A → HIN = normalny PWM (active high)
//   gen_B → LIN = komplementarny (odwrócony) z dead-time
//   Typ dead-time: MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
//     gen_A: PWM + rising edge delay (RED)
//     gen_B: ~PWM (complement) + falling edge delay (FED)
//
// Timer resolution: 20 MHz (prescaler = 8 z 160 MHz grupy)
// Period (UP_DOWN): 500 counts = 20 kHz (20M / 500 / 2 = 20k)
// Zakres duty: compare 0..500 (proporcjonalny do okresu, NIE 0..1023!)
// Dead time: ~500ns = 10 ticks @ 20MHz (RED=10, FED=10)

/** @brief Częstotliwość PWM dla sterowników MOSFET [Hz] */
#define PWM_FREQUENCY           20000

/** @brief Rozdzielczość timera MCPWM [Hz] — prescaler wewnętrzny */
#define MCPWM_TIMER_RESOLUTION  20000000

/** @brief Okres timera MCPWM (half-period for UP_DOWN = resolution / frequency / 2) */
#define MCPWM_TIMER_PERIOD      (MCPWM_TIMER_RESOLUTION / PWM_FREQUENCY / 2)


/** @brief Maksymalna wartość duty (compare value = half period) */
#define PWM_MAX_DUTY            MCPWM_TIMER_PERIOD

/** @brief Dead time rising edge delay [ticks timer resolution] (~500ns @ 20MHz) */
#define MCPWM_DEAD_TIME_RED     10

/** @brief Dead time falling edge delay [ticks timer resolution] (~500ns @ 20MHz) */
#define MCPWM_DEAD_TIME_FED     10

#endif // PINOUT_H
