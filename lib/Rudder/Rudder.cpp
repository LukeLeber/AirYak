/**
 * @file Rudder.cpp
 * @brief Moving-average filtering and hysteresis stabilization implementation for the rudder.
 */

#include <Rudder.h>
#include <AirYakConfig.h>
#include <Arduino.h>
#include <AnalogSensor.h>
#include <EcoServo.h>
#include <math.h>

#define DEBUG 1

/**
 * @class Rudder::Impl
 * @brief Private data allocation block keeping tracking histories hidden from the API interface boundary.
 */
class Rudder::Impl {
  public:
    AnalogSensor input;           /**< Input potentiometer to read for desired rudder position. */
    EcoServo output;              /**< Actuator hardware for the rudder control. */
    uint8_t samples[8];           /**< Rolling circular memory array storage buffer. */
    uint8_t next_sample;          /**< Index position tracker highlighting where the oldest record sits. */
    uint16_t total_sample;        /**< Running cumulative sum registry backing the average calculation steps. */
    uint8_t current_position;     /**< Cached historical location applied to the physical hardware channel. */

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
      input(input_pin),
      output(output_pin, 90, sweep_delay_millis),
      next_sample(0),
      total_sample(720),
      current_position(90) {
        // Initialize the circular buffer to all center positions.
        for(uint8_t i = 0; i < 8; ++i) {
          this->samples[i] = 90;
        }
      AIRYAK_DEBUG_METRIC('Rudder is moving to position', 90, 'degrees.');
    }
};

Rudder::Rudder(uint8_t input_pin, uint8_t output_pin, uint16_t sweep_delay_millis):
  impl(new Rudder::Impl(input_pin, output_pin, sweep_delay_millis)) {

}

Rudder::~Rudder() {
  delete this->impl;
}

void Rudder::update() {

  // Cycle out the old sample.
  this->impl->total_sample -= this->impl->samples[this->impl->next_sample];

  // Cycle in the new sample.
  this->impl->samples[this->impl->next_sample] = (uint8_t)round((float)this->impl->input.read() * 180.0f / 1023.0f);;

  // Recalculate the total samples.
  this->impl->total_sample += this->impl->samples[this->impl->next_sample];

  // Move the sample index to the next position (or wrap around if at the end).
  this->impl->next_sample = (this->impl->next_sample + 1) % 8;

  // Calculate the new average of the samples to find the requested position.
  uint8_t new_position = (uint8_t)round((float)this->impl->total_sample / 8.0f);

  // Only write a new position if there was a change.
  if (new_position != this->impl->current_position) {

    // If not at one of the range extremes, apply hysteresis to prevent jitter.
    if (new_position == 0 || new_position == 180 || abs((int)new_position - (int)this->impl->current_position) > 1) {
      this->impl->output.setPosition(new_position);
      this->impl->current_position = new_position;
      AIRYAK_DEBUG_METRIC('Rudder is moving to position', new_position, 'degrees.');
    }
  }
}
