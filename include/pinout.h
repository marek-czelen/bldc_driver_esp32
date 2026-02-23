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
#define PIN_PWM_A_HIGH          32  // GPIO32

/** @brief Faza A - sterowanie low-side MOSFET (IR2103 LIN - odwrócony!) */
#define PIN_PWM_A_LOW           33  // GPIO33

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

/**
 * @brief Czujnik temperatury tranzystorów FET (ADC2_CH5).
 * @warning GPIO12 (MTDI) jest pinem STRAP ESP32!
 * Pull-up na GPIO12 podczas boot-u przestawia VDD_SDIO na 1.8V zamiast 3.3V,
 * co uniemożliwia programowanie flas. Na PCB NIE stosować pull-up na GPIO12.
 * Stosować pull-down lub pozostawić floating.
 */
#define PIN_FET_TEMP            12  // GPIO12 (STRAP pin - nie stosuj pull-up!)

/** @brief Czujnik temperatury silnika */
#define PIN_MOTOR_TEMP          15  // GPIO15

// ============================================================================
// Wejścia sterujące
// ============================================================================

/** @brief Wejście przepustnicy / gazu (ADC2_CH2) */
#define PIN_THROTTLE            2   // GPIO2

/** @brief Wejście czujnika PAS (Pedal Assist Sensor) */
#define PIN_PAS                 22  // GPIO22

/** @brief Wejście hamulca */
#define PIN_BRAKE               23  // GPIO23

// ============================================================================
// Czujniki Halla
// ============================================================================

/** @brief Czujnik Halla A */
#define PIN_HALL_SENSOR_A       5   // GPIO5

/** @brief Czujnik Halla B */
#define PIN_HALL_SENSOR_B       18  // GPIO18

/** @brief Czujnik Halla C */
#define PIN_HALL_SENSOR_C       19  // GPIO19

// ============================================================================
// Wyjście prędkości
// ============================================================================

/** @brief Wyjście sygnału prędkości */
#define PIN_SPEED               21  // GPIO21

// ============================================================================
// UART
// ============================================================================

/** @brief UART RX (3.3V) */
#define PIN_UART_RX             3   // GPIO3 (RX0)

/** @brief UART TX (3.3V) */
#define PIN_UART_TX             1   // GPIO1 (TX0)

/** @brief UART Enable */
#define PIN_UART_EN             17  // GPIO17

// ============================================================================
// Wyprowadzenia rozszerzeń
// ============================================================================

/** @brief Rozszerzenie 1 */
#define PIN_EXT_1               0   // GPIO0 (UWAGA: boot pin)

/** @brief Rozszerzenie 2 */
#define PIN_EXT_2               4   // GPIO4

/** @brief Rozszerzenie 3 */
#define PIN_EXT_3               16  // GPIO16

// ============================================================================
// Konfiguracja PWM (LEDC)
// ============================================================================

/** @brief Częstotliwość PWM dla sterowników MOSFET [Hz] */
#define PWM_FREQUENCY           20000

/** @brief Rozdzielczość PWM [bity] */
#define PWM_RESOLUTION          10

/** @brief Maksymalna wartość PWM (2^PWM_RESOLUTION - 1) */
#define PWM_MAX_DUTY            ((1 << PWM_RESOLUTION) - 1)

/**
 * @defgroup ledc_channels Kanały LEDC
 * @brief Przypisanie kanałów LEDC do sterowania mostkami irR2103.
 *
 * ESP32 ma 16 kanałów LEDC (0-15). Każda faza wymaga 2 kanałów (HIGH + LOW).
 * Kanały 0-5 są używane dla faz A, B, C.
 *
 * Uwaga o LIN (LOW kanały):
 *   duty=0            → pin=LOW  → LIN=LOW  → low-side MOSFET ON (przewodzi)
 *   duty=PWM_MAX_DUTY → pin=HIGH → LIN=HIGH → low-side MOSFET OFF (bezpieczny stan)
 * @{
 */
#define PWM_CHANNEL_A_HIGH      0   ///< Kanał LEDC: Faza A high-side (HIN)
#define PWM_CHANNEL_A_LOW       1   ///< Kanał LEDC: Faza A low-side (LIN, logika odwrócona)
#define PWM_CHANNEL_B_HIGH      2   ///< Kanał LEDC: Faza B high-side (HIN)
#define PWM_CHANNEL_B_LOW       3   ///< Kanał LEDC: Faza B low-side (LIN, logika odwrócona)
#define PWM_CHANNEL_C_HIGH      4   ///< Kanał LEDC: Faza C high-side (HIN)
#define PWM_CHANNEL_C_LOW       5   ///< Kanał LEDC: Faza C low-side (LIN, logika odwrócona)
/** @} */

#endif // PINOUT_H
