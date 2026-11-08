/**
 * @file FlywheelMagnetInterrupt.cpp
 * @brief Implementation details for the AirYak flywheel Hall Effect interrupt handler.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements low-level external interrupt registration, atomic 32-bit timestamp
 * isolation, and stall timeout detection using the Pointer to Implementation (PIMPL) idiom.
 */

#include <FlywheelMagnetInterrupt.h>
#include <Arduino.h>
#include <util/atomic.h>

/**
 * @class FlywheelMagnetInterrupt::Impl
 * @brief Private implementation handling low-level ISR routing and register locking.
 * 
 * Leverages an internal Resource Acquisition Is Initialization (RAII) structural framework
 * to control low-level peripheral mapping configurations and pin safety hooks.
 */
class FlywheelMagnetInterrupt::Impl {
  private:
    static Impl* instance;                  /**< Singleton implementation reference pointer for static ISR routing. */
    uint8_t pin;                            /**< Assigned physical micro-controller signal channel. */
    volatile uint32_t last_pulse_micros;    /**< Timestamp (in microseconds) of the most recent falling edge. */
    volatile uint32_t pulse_period_micros;  /**< Elapsed interval (in microseconds) between the last two falling edges. */

    /**
     * @brief Static Interrupt Service Routine (ISR) callback router.
     * 
     * Executed once per physical crankshaft revolution on the falling edge of the
     * sensor channel. Computes the delta since the prior pulse and updates the shared
     * volatile timing registers.
     */
    static void onPulse(void) {
      if (instance) {
        uint32_t now = micros();
        if (instance->last_pulse_micros > 0) {
          instance->pulse_period_micros = now - instance->last_pulse_micros;
        }
        instance->last_pulse_micros = now;
      }
    }

  public:

    /**
     * @brief Configures low-level pins and registers the asynchronous hardware hook.
     * 
     * Binds the singleton pointer prior to enabling interrupts, activates internal
     * pull-up resistors to stabilize signals against ignition EMI, and attaches the ISR.
     * 
     * @param[in] pin The target digital hardware GPIO pin.
     */
    Impl(uint8_t pin): pin(pin), last_pulse_micros(0), pulse_period_micros(0) {
      instance = this;
      auto interrupt = digitalPinToInterrupt(this->pin);
      if (interrupt != NOT_AN_INTERRUPT) {
        pinMode(pin, INPUT_PULLUP);
        attachInterrupt(interrupt, onPulse, FALLING);
      }
    }

    /**
     * @brief Atomically reads the measured rotational period and checks for engine stall.
     * 
     * Uses an atomic critical section to prevent 32-bit multi-byte read tearing on 8-bit
     * AVR hardware. If more than 500,000 microseconds (500 ms) have elapsed since the last
     * detected edge, returns 0 to indicate a stalled or stationary flywheel.
     * 
     * @return uint32_t Rotational period in microseconds, or 0 if stalled/uninitialized.
     */
    uint32_t getPeriodMicros(void) const {
      uint32_t period;
      uint32_t last;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        period = this->pulse_period_micros;
        last = this->last_pulse_micros;
      }
      // If no pulse for 500ms, engine has stalled (return 0)
      if (last == 0 || (micros() - last > 500000UL)) {
        return 0;
      }
      return period;
    }

    /**
     * @brief Releases hardware configurations and removes interrupt vectors safely.
     */
    ~Impl(void) {
      auto interrupt = digitalPinToInterrupt(this->pin);
      if (interrupt != NOT_AN_INTERRUPT) {
        detachInterrupt(interrupt);
      }
      instance = nullptr;
    }
};

// Allocate the static singleton initialization hook at global file scope.
FlywheelMagnetInterrupt::Impl* FlywheelMagnetInterrupt::Impl::instance = nullptr;

FlywheelMagnetInterrupt::FlywheelMagnetInterrupt(uint8_t pin): impl(new FlywheelMagnetInterrupt::Impl(pin)) {
}

uint32_t FlywheelMagnetInterrupt::getPeriodMicros(void) const {
  return this->impl->getPeriodMicros();
}

FlywheelMagnetInterrupt::~FlywheelMagnetInterrupt(void) {
  delete this->impl;
}