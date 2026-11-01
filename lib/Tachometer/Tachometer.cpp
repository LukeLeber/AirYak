/**
 * @file Tachometer.cpp
 * @brief Dynamic time-scale conversion calculations for engine RPM monitoring.
 */

#include <Tachometer.h>
#include <FlywheelMagnetInterrupt.h>
#include <Arduino.h>

/**
 * @class Tachometer::Impl
 * @brief Private data container managing the encapsulated sensor and calculations cache.
 */
class Tachometer::Impl {
  public:
    FlywheelMagnetInterrupt interrupt; /**< Private hardware sensor instance handle. */    
    uint32_t last_update;               /**< Absolute fixed-width timestamp anchor for sample sweeps. */
    uint16_t speed;                     /**< Cached operational value registry record. */

    Impl(uint8_t pin): interrupt(pin), last_update((uint32_t)millis()), speed(0) {}
    ~Impl() {}
};

Tachometer::Tachometer(uint8_t pin): impl(new Tachometer::Impl(pin)) {}

Tachometer::~Tachometer(void) {
  delete this->impl;
}

void Tachometer::resample(void) {
  uint32_t now = (uint32_t)millis();
  uint32_t elapsed = now - this->impl->last_update;

  // Enforce a strict minimum 1-second operational interval buffer to stabilize values
  if (elapsed >= 1000) {
    uint8_t recorded_pulses = this->impl->interrupt.getPulses();

    this->impl->speed = (uint16_t)(((uint32_t)recorded_pulses * 60000UL) / elapsed);
    
    this->impl->last_update = now;
  }
}

const uint16_t Tachometer::getSpeed(void) const {
  return this->impl->speed;
}