/**
 * @file Tachometer.cpp
 * @brief Dynamic time-scale conversion calculations for engine RPM monitoring.
 */

#include <Tachometer.h>
#include <FlywheelMagnetInterrupt.h>

/**
 * @class Tachometer::Impl
 * @brief Private data container managing the encapsulated sensor and calculations cache.
 */
class Tachometer::Impl {
  public:

    Impl(uint8_t pin): interrupt(pin), speed(0) {

    }

    void resample(void) {
      uint32_t period = this->interrupt.getPeriodMicros();
      
      if (period > 0) {
        // 60,000,000 microseconds per minute / period in microseconds
        uint16_t raw_rpm = (uint16_t)(60000000UL / period);
        
        // Optional light EMA smoothing (75% old, 25% new) to absorb minor crank speed variations
        if (this->speed == 0) {
          this->speed = raw_rpm;
        } else {
          this->speed = ((this->speed * 3) + raw_rpm) / 4;
        }
      } else {
        this->speed = 0;
      }
    }

    uint16_t getSpeed(void) const {
      return this->speed;
    }
    ~Impl() {}

  private:
    FlywheelMagnetInterrupt interrupt; /**< Private hardware sensor instance handle. */    
    uint16_t speed;                     /**< Cached operational value registry record. */
};

Tachometer::Tachometer(uint8_t pin): impl(new Tachometer::Impl(pin)) {}

void Tachometer::resample(void) {
  this->impl->resample();
}

uint16_t Tachometer::getSpeed(void) const {
  return this->impl->getSpeed();
}

Tachometer::~Tachometer(void) {
  delete this->impl;
}
