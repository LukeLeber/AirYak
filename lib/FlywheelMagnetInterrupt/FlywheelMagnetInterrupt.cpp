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
    static Impl* instance;          /**< Singleton implementation reference pointer for static ISR routing. */
    uint8_t pin;                    /**< Assigned physical micro-controller signal channel. */
    volatile uint8_t pulses = 0;    /**< Hardware event counter. Marked volatile due to asynchronous thread writes. */

    /**
     * @brief Static Interrupt Service Routine (ISR) callback router.
     * 
     * Executed exactly once per physical crankshaft revolution on the falling edge 
     * of the sensor channel. Forwards state mutation down to the scoped memory wrapper.
     */
    static void onPulse(void) {
      if (instance) {
        ++instance->pulses;
      }
    }

  public:

    /**
     * @brief Configures low-level pins and registers the asynchronous hardware hook.
     * 
     * Activates input pullup resistors to stabilize signals against high engine RF noise,
     * validates the interrupt mapping boundary, and safely links the handler.
     * 
     * @param[in] pin The target digital hardware GPIO pin.
     */
    Impl(uint8_t pin): pin(pin) {
      auto interrupt = digitalPinToInterrupt(this->pin);
      if (interrupt != NOT_AN_INTERRUPT) {
        pinMode(pin, INPUT_PULLUP);
        attachInterrupt(interrupt, onPulse, FALLING);
      }
      instance = this;
    }

    /**
     * @brief Releases hardware configurations and removes interrupt vectors safely.
     */
    ~Impl(void) {
      instance = nullptr;
      auto interrupt = digitalPinToInterrupt(this->pin);
      if (interrupt != NOT_AN_INTERRUPT) {
        detachInterrupt(interrupt);
      }
    }

    /**
     * @brief Atomically extracts and flushes the accumulated loop pulse count register.
     * 
     * Extracted logic utilizes critical block nesting to protect data lines from 
     * concurrent mutation splits while a multi-cycle assignment statement executes.
     * 
     * Math Proof for Data Type Sizing:
     * - Data register: 8-bit uint8_t (Max capability count = 255 pulses).
     * - Sampling loop frequency constraint: <= 1.0 second intervals.
     * - Max quantifiable velocity calculation: (255 pulses / 1 sec) * 60 sec/min = 15,300 RPM.
     * Given the physical structural yield limit of the Predator engine block sits well
     * below 10,000 RPM, an 8-bit tracking register provides safe overhead margins.
     * 
     * @return uint8_t Total accumulated edge pulses since the last sweep.
     */
    uint8_t getPulses(void) {
      uint8_t pulses;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // NOLINT
        pulses = this->pulses;
        this->pulses = 0;
      }
      return pulses;      
    }
};

// Allocate the static singleton initialization hook at global file scope.
FlywheelMagnetInterrupt::Impl* FlywheelMagnetInterrupt::Impl::instance = nullptr;

FlywheelMagnetInterrupt::FlywheelMagnetInterrupt(uint8_t pin): impl(new FlywheelMagnetInterrupt::Impl(pin)) {
  // Safe instantiation abstraction via the pointer layout boundary.
}

FlywheelMagnetInterrupt::~FlywheelMagnetInterrupt(void) {
  delete this->impl;
}

uint8_t FlywheelMagnetInterrupt::getPulses(void) {
  return this->impl->getPulses();
}