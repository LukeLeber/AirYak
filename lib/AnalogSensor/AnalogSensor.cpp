/**
 * @file AnalogSensor.cpp
 * @brief Implementation details for the AirYak analog sensor.
 * 
 * Implements an analog sensor.
 */
#include <AnalogSensor.h>
#include <Arduino.h>

/**
 * @class AnalogSensor::Impl
 * @brief Private data container handling internal arduino implementation details.
 * 
 * Separates data allocation boundaries from the public method structure. This ensures
 * changes to the internal structures do not break downstream compilation dependencies.
 */
class AnalogSensor::Impl {
  public:
    uint8_t pin; /**< Assigned physical micro-controller input channel. */

    /**
     * @brief Instantiates the private structural context block.
     * 
     * Persists the input channel.
     */
    Impl(uint8_t pin): pin(pin) {
      pinMode(pin, INPUT);
    }
};

AnalogSensor::AnalogSensor(uint8_t pin): impl(new AnalogSensor::Impl(pin)) {

}

AnalogSensor::~AnalogSensor() {
  delete this->impl;
}

uint16_t AnalogSensor::read() const {

  // Toss out the first read.
  analogRead(this->impl->pin); 
  delayMicroseconds(10);

  // Collect 16 samples.
  uint32_t accumulator = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    accumulator += analogRead(this->impl->pin);
  }

  // Report the linear average of all samples.
  return static_cast<uint16_t>(accumulator >> 4);
}