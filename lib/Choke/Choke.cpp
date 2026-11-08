/**
 * @file Choke.cpp
 * @brief Implementation details for the AirYak automated choke control state machine.
 * 
 * Implements a timed linear transition algorithm using integer mapping. The internal
 * implementation metrics are entirely hidden from the public API boundary via the 
 * Pointer to Implementation (PIMPL) idiom to decouple compile-time dependencies.
 */
#include <Choke.h>
#include <Arduino.h>
#include <AirYakConfig.h>
#include <EcoServo.h>
#include <Tachometer.h>


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
    const Tachometer& tachometer;        /**< Reference to the tachometer subsystem. */
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
    Impl(const Tachometer& tachometer, uint8_t pin, uint8_t choke_off_position, uint8_t choke_full_position, uint32_t warmup_duration, uint16_t sweep_delay_millis): 
      tachometer(tachometer),
      output(pin, choke_off_position, sweep_delay_millis),
      choke_off_position(choke_off_position),
      choke_full_position(choke_full_position),
      warmup_duration(warmup_duration), 
      engine_start_time(0) {
        AIRYAK_DEBUG_METRIC("Choke initialized to", choke_off_position, "degrees");
      }

    /**
     * @brief Evaluates the current timeline state and updates the physical actuator position.
     * 
     * @note This function is non-blocking and must be invoked continuously inside the main application loop.
     *       It calculates intermediate steps using active time mapping and suppresses redundant servo writes.
     * 
     * @todo This algorithm is sub-optimal and assumes the engine will start
     *       within 10 seconds. Refactor the update method to start the timer
     *       when ignition is detected versus when cranking starts.
     */
    void update(void) {
      // Pull current state baseline to prepare for conditional routing paths
      uint8_t new_choke_position = this->output.getPosition();
      uint32_t now = millis();

      if(now != 0) {
        if (this->tachometer.getSpeed() > 0) {
          // Condition 1: Cold start/Retrigger initialization phase
          if (this->engine_start_time == 0) {
            this->engine_start_time = now;
            new_choke_position = this->choke_full_position;
          }
          // Condition 2: Active execution loop
          else {
            // Unsigned 32-bit subtraction arithmetic naturally handles 49.7-day timer overflows
            uint32_t time_elapsed_since_start = now - this->engine_start_time;

            // Linear progression phase
            if (time_elapsed_since_start < this->warmup_duration) {
              new_choke_position = map(
                time_elapsed_since_start, 
                0, 
                this->warmup_duration, 
                this->choke_full_position, 
                this->choke_off_position
              );
            }
            // Timeout cleanup catch phase
            else if (new_choke_position != this->choke_off_position) {
              new_choke_position = this->choke_off_position;
            }
          }
        }
        else if (this->output.getPosition() != this->choke_off_position) {
          AIRYAK_DEBUG_METRIC("Engine stalled; choke reset to", this->choke_off_position, "degrees");
          new_choke_position = this->choke_off_position;
          this->engine_start_time = 0;
        }

        // Defensive optimization: Suppress redundant servo signals to save system bus bandwidth
        // and minimize mechanical jitter or hum on the control wires.
        if (this->output.getPosition() != new_choke_position) {
          this->output.setPosition(new_choke_position);
          AIRYAK_DEBUG_METRIC("Choke moved to", new_choke_position, "degrees");
        }
        this->output.revalidateEcoMode();
      }
    }

    /**
     * @brief Restores the state machine back to its cold-start baseline conditions.
     * 
     * Call this whenever an engine start condition or cranking event is triggered.
     * The next time Choke::update() is called, it will immediately re-engage the full choke.
     */
    void reset(void) {
      this->engine_start_time = 0;
    }

    /**
     * @brief Retrieves the last commanded target position assigned to the actuator.
     * 
     * @return uint16_t The active position target value.
     */
    uint16_t getPosition(void) const {
      return this->output.getPosition();
    }
};

Choke::Choke(const Tachometer& tachometer, uint8_t pin, uint8_t choke_off_position, uint8_t choke_full_position, uint32_t warmup_duration, uint16_t sweep_delay_millis): 
  impl(new Choke::Impl(tachometer, pin, choke_off_position, choke_full_position, warmup_duration, sweep_delay_millis)) {

}

void Choke::update(void) {
  this->impl->update();
}

void Choke::reset(void) {
  this->impl->reset();
}

uint16_t Choke::getPosition(void) const {
  return this->impl->getPosition();
}

Choke::~Choke(void) {
  delete this->impl;
}