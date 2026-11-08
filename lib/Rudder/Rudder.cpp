/**
 * @file Rudder.cpp
 * @brief Moving-average FIR filtering and hysteresis stabilization implementation for the rudder.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements an 8-sample circular rolling-window moving average filter and output
 * hysteresis using the Pointer to Implementation (PIMPL) idiom to map pilot steering
 * inputs smoothly onto the airfoil servo without jitter or line buzz.
 */

#include <Rudder.h>
#include <AirYakConfig.h>
#include <Arduino.h>
#include <AnalogSensor.h>
#include <EcoServo.h>

/**
 * @class Rudder::Impl
 * @brief Private data allocation block keeping tracking histories hidden from the API interface boundary.
 */
class Rudder::Impl {

  public:
    /**
     * @brief Constructs a new rudder private implementation object.
     * 
     * Sets up the analog sensor input, eco servo output, and state machine properties.
     * 
     * @param[in] input_pin Physical analog ADC channel linked to the steering yoke potentiometer.
     * @param[in] output_pin Physical digital GPIO pin mapped to the steering servo.
     * @param[in] sweep_delay_millis The number of milliseconds that the output actuator takes to move.
     */
    Impl(uint8_t input_pin, uint8_t output_pin, uint16_t sweep_delay_millis): 
      input(input_pin, 0),
      output(output_pin, 90, sweep_delay_millis),
      next_sample(0),
      total_sample(720) {
        // Initialize the circular buffer to all center positions.
        for(uint8_t i = 0; i < 8; ++i) {
          this->samples[i] = 90;
        }
      AIRYAK_DEBUG_METRIC("Rudder is moving to position", 90, "degrees.");
    }

    /**
     * @brief Polls the input channel and updates the steering actuator.
     */
    void update(void) {
      // Cycle out the old sample.
      this->total_sample -= this->samples[this->next_sample];

      // Cycle in the new sample.
      this->samples[this->next_sample] = (uint8_t)(((uint32_t)this->input.read() * 180UL + 511UL) / 1023UL);
      
      // Recalculate the total samples.
      this->total_sample += this->samples[this->next_sample];

      // Move the sample index to the next position (or wrap around if at the end).
      this->next_sample = (this->next_sample + 1) % 8;

      // Calculate the new average of the samples to find the requested position.
      uint8_t new_position = (uint8_t)((this->total_sample + 4) / 8);
      
      // Only write a new position if there was a change.
      uint16_t current_position = this->output.getPosition();
      if (new_position != current_position) {

        // If not at one of the range extremes, apply hysteresis to prevent jitter.
        if (new_position == 0 || new_position == 180 || abs((int)new_position - (int)current_position) > 1) {
          this->output.setPosition(new_position);
          AIRYAK_DEBUG_METRIC("Rudder is moving to position", new_position, "degrees.");
        }
      }
      this->output.revalidateEcoMode();
    }

    /**
     * @brief Retrieves the last commanded target position assigned to the actuator.
     * 
     * @return uint8_t The active position target value.
     */
    uint8_t getPosition(void) const {
      return this->output.getPosition();
    }

  private:
    AnalogSensor input;           /**< Input potentiometer to read for desired rudder position. */
    EcoServo output;              /**< Actuator hardware for the rudder control. */
    uint8_t samples[8];           /**< Rolling circular memory array storage buffer. */
    uint8_t next_sample;          /**< Index position tracker highlighting where the oldest record sits. */
    uint16_t total_sample;        /**< Running cumulative sum registry backing the average calculation steps. */

};

Rudder::Rudder(uint8_t input_pin, uint8_t output_pin, uint16_t sweep_delay_millis):
  impl(new Rudder::Impl(input_pin, output_pin, sweep_delay_millis)) {

}

void Rudder::update() {
  this->impl->update();
}

uint8_t Rudder::getPosition(void) const {
  return this->impl->getPosition();
}

Rudder::~Rudder() {
  delete this->impl;
}
