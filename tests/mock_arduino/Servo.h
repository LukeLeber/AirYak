/**
 * @file Servo.h
 * @brief Host-side mock definitions for the Arduino Servo library.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Emulates PWM servo pin attachment, angular positioning, and state inspection
 * for native Linux unit testing without hardware timers.
 */

#ifndef AIRYAK_MOCK_SERVO_H
#define AIRYAK_MOCK_SERVO_H

#include <stdint.h>

/**
 * @class Servo
 * @brief Lightweight mock implementation of the Arduino Servo actuator interface.
 */
class Servo {
  private:
    uint8_t _position;    /**< Last commanded target position in degrees (0-180). */
    bool _is_attached;    /**< Current PWM signal attachment state. */

  public:
    /**
     * @brief Constructs a mock Servo instance with Arduino's default unattached center baseline.
     */
    Servo(void) : _position(90), _is_attached(false) {}

    /**
     * @brief Sets the target angular position of the mock servo.
     * @param[in] value Target angle in degrees (typically 0-180).
     */
    void write(uint8_t value) {
      _position = value;
    }

    /**
     * @brief Attaches the mock servo to a digital GPIO pin.
     * @param[in] pin Digital GPIO pin index.
     */
    void attach(uint8_t pin) {
      (void)pin;
      _is_attached = true;
    }

    /**
     * @brief Detaches the mock servo from its GPIO pin to simulate power-saving signal isolation.
     */
    void detach(void) {
      _is_attached = false;
    }

    /**
     * @brief Retrieves the last commanded angular position.
     * @return uint8_t Current servo angle in degrees.
     */
    uint8_t read(void) const {
      return _position;
    }

    /**
     * @brief Checks whether the mock servo is currently attached to a control pin.
     * @return true If attached and actively driven.
     * @return false If detached.
     */
    bool attached(void) const {
      return _is_attached;
    }
};

#endif // AIRYAK_MOCK_SERVO_H