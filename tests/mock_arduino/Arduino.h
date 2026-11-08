/**
 * @file Arduino.h
 * @brief Host-side mock definitions for Arduino core APIs and hardware registers.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Emulates Arduino timing functions, GPIO configuration, external interrupts,
 * analog-to-digital conversion (ADC), and math utilities for native Linux unit testing.
 */
#ifndef AIRYAK_MOCK_ARDUINO_H
#define AIRYAK_MOCK_ARDUINO_H

#include <stdint.h>

// --- GPIO & External Interrupt Constants ---
#define INPUT 0               /**< Digital pin input mode without internal pull-up. */
#define INPUT_PULLUP 2        /**< Digital pin input mode with internal pull-up resistor enabled. */
#define FALLING 0             /**< Interrupt trigger mode on high-to-low signal transition. */
#define NOT_AN_INTERRUPT -1   /**< Sentinel indicating a GPIO pin lacks hardware interrupt support. */

// --- PID Proportional Mode Constants ---
#define P_ON_M 0              /**< Proportional-on-Measurement mode flag. */
#define P_ON_E 1              /**< Proportional-on-Error mode flag. */

// --- Global Host Test State Variables (Defined in Arduino.cpp) ---
extern unsigned long mock_millis;          /**< Simulated system uptime in milliseconds. */
extern unsigned long mock_micros;          /**< Simulated system uptime in microseconds. */
extern void (*mock_isr_callback)(void);    /**< Captured ISR function pointer for manual test triggering. */
extern uint16_t mock_analog_value;         /**< Simulated 10-bit ADC readout value (0-1023). */

/**
 * @brief Advances the simulated millisecond and microsecond clocks.
 * @param[in] ms Duration to advance in milliseconds.
 */
inline void delay(unsigned long ms) {
  mock_millis += ms;
  mock_micros += ms * 1000UL;
}

/**
 * @brief Advances the simulated microsecond clock and synchronizes milliseconds.
 * @param[in] us Duration to advance in microseconds.
 */
inline void delayMicroseconds(unsigned long us) {
  mock_micros += us;
  mock_millis = mock_micros / 1000UL;
}

/**
 * @brief Retrieves the simulated system uptime in milliseconds.
 * @return unsigned long Current mock millisecond timestamp.
 */
inline unsigned long millis(void) {
  return mock_millis;
}

/**
 * @brief Retrieves the simulated system uptime in microseconds.
 * @return unsigned long Current mock microsecond timestamp.
 */
inline unsigned long micros(void) {
  return mock_micros;
}

/**
 * @brief Maps an ATmega328P digital pin number to its external interrupt vector.
 * @param[in] pin Digital GPIO pin index (supports pins 2 and 3).
 * @return int External interrupt vector (0 or 1), or NOT_AN_INTERRUPT if unsupported.
 */
inline int digitalPinToInterrupt(uint8_t pin) {
  return (pin == 2 || pin == 3) ? (pin - 2) : NOT_AN_INTERRUPT;
}

/**
 * @brief No-op stub for configuring GPIO pin direction and pull-up mode.
 * @param[in] pin Digital GPIO pin index.
 * @param[in] mode Pin mode constant (INPUT, INPUT_PULLUP, etc.).
 */
inline void pinMode(uint8_t pin, uint8_t mode) {
  (void)pin;
  (void)mode;
}

/**
 * @brief Registers an external interrupt service routine callback for test simulation.
 * @param[in] interruptNum External interrupt vector index (0 or 1).
 * @param[in] userFunc Function pointer to invoke when the interrupt fires.
 * @param[in] mode Edge trigger mode (e.g., FALLING).
 */
inline void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode) {
  (void)mode;
  if (interruptNum == 0 || interruptNum == 1) {
    mock_isr_callback = userFunc;
  }
}

/**
 * @brief Unregisters the active external interrupt callback.
 * @param[in] interruptNum External interrupt vector index (0 or 1).
 */
inline void detachInterrupt(int interruptNum) {
  if (interruptNum == 0 || interruptNum == 1) {
    mock_isr_callback = nullptr;
  }
}

/**
 * @brief Re-maps a number from one integer range to another.
 * @param[in] x Input value to map.
 * @param[in] in_min Lower bound of the input range.
 * @param[in] in_max Upper bound of the input range.
 * @param[in] out_min Lower bound of the target range.
 * @param[in] out_max Upper bound of the target range.
 * @return long Proportionally scaled value within [out_min, out_max].
 */
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Reads the simulated 10-bit ADC value for an analog input channel.
 * @param[in] pin Analog input pin index.
 * @return uint16_t Simulated ADC reading (0-1023).
 */
inline uint16_t analogRead(uint8_t pin) {
  (void)pin;
  return mock_analog_value;
}

/**
 * @brief Computes the absolute value of a numeric type without macro double-evaluation.
 * @tparam T Numeric value type.
 * @param[in] x Input value.
 * @return T Non-negative magnitude of x.
 */
template <typename T>
inline T abs(T x) {
  return (x < 0) ? -x : x;
}

#endif // AIRYAK_MOCK_ARDUINO_H