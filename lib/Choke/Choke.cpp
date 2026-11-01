/**
 * @file Choke.cpp
 * @brief Implementation details for the AirYak automated choke control state machine.
 * 
 * Implements a timed linear transition algorithm using integer mapping. The internal
 * implementation metrics are entirely hidden from the public API boundary via the 
 * Pointer to Implementation (PIMPL) idiom to decouple compile-time dependencies.
 */

#include <Arduino.h>
#include <Choke.h>
#include <EcoServo.h>

#define DEBUG 1

/**
 * @class Choke::Impl
 * @brief Private data container handling internal timer states and hardware tracking.
 * 
 * Separates data allocation boundaries from the public method structure. This ensures
 * changes to the internal structures do not break downstream compilation dependencies.
 * 
 * @todo Add a cylinder head temperature (CHT) sensor module integration to dynamically
 *       adjust the runtime warmup_duration limit based on actual engine thermal loads.
 */
class Choke::Impl {
  public:
    EcoServo output;                     /**< Reference to the actuated hardware channel. */
    const uint8_t choke_off_position;    /**< Target angular position for normal running. */
    const uint8_t choke_full_position;   /**< Target angular position for rich fueling. */
    const uint32_t warmup_duration;      /**< Allowed phase timeframe in milliseconds. */
    uint32_t engine_start_time;          /**< Timestamp anchor for the active cycle loop. */

    /**
     * @brief Instantiates the private structural context block.
     * 
     * Defaults the initial location tracking cache to choke_off_position to ensure
     * safe baseline operations before the initial update sweep.
     */
    Impl(uint8_t pin, uint8_t choke_off_position, uint8_t choke_full_position, uint32_t warmup_duration, uint16_t sweep_delay_millis): 
      output(pin, choke_off_position, sweep_delay_millis),
      choke_off_position(choke_off_position),
      choke_full_position(choke_full_position),
      warmup_duration(warmup_duration), 
      engine_start_time(0) {
      }
};

Choke::Choke(uint8_t pin, uint8_t choke_off_position, uint8_t choke_full_position, uint32_t warmup_duration, uint16_t sweep_delay_millis): 
  impl(new Choke::Impl(pin, choke_off_position, choke_full_position, warmup_duration, sweep_delay_millis)) {
  // Heap allocation instantiates private implementation bounds cleanly
}

Choke::~Choke(void) {
  delete this->impl; // Free allocated memory to protect against application-level leaks
}

void Choke::update(void) {
  // Pull current state baseline to prepare for conditional routing paths
  uint8_t new_choke_position = this->impl->output.getPosition();
  uint32_t now = millis();

  // Condition 1: Cold start/Retrigger initialization phase
  if (this->impl->engine_start_time == 0) {
    this->impl->engine_start_time = now;
    new_choke_position = this->impl->choke_full_position;
  }
  // Condition 2: Active execution loop
  else {
    // Unsigned 32-bit subtraction arithmetic naturally handles 49.7-day timer overflows
    uint32_t time_elapsed_since_start = now - this->impl->engine_start_time;

    // Linear progression phase
    if (time_elapsed_since_start < this->impl->warmup_duration) {
      new_choke_position = map(
        time_elapsed_since_start, 
        0, 
        this->impl->warmup_duration, 
        this->impl->choke_full_position, 
        this->impl->choke_off_position
      );
    }
    // Timeout cleanup catch phase
    else if (new_choke_position != this->impl->choke_off_position) {
      new_choke_position = this->impl->choke_off_position;
    }
  }

  // Defensive optimization: Suppress redundant servo signals to save system bus bandwidth
  // and minimize mechanical jitter or hum on the control wires.
  if (this->impl->output.getPosition() != new_choke_position) {
    this->impl->output.setPosition(new_choke_position);
  }
  this->impl->output.revalidateEcoMode();
}

void Choke::reset(void) {
  // Invalidating the start time anchor marks the state machine ready for a fresh cycle
  this->impl->engine_start_time = 0;
}

uint8_t Choke::getPosition(void) const {
  return this->impl->output.getPosition();
}
