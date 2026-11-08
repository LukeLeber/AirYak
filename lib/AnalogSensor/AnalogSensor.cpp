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
    uint8_t pin;                     /**< Assigned physical micro-controller input channel. */
    uint8_t sensitivity_factor;      /**< Assigned sensitivity factor for the sensor. */
    mutable uint16_t smoothed_value; /**< The current smoothed value for the sensor. */
    mutable bool is_initialized;     /**< An initialization flag for the sensor. */

    /**
     * @brief Instantiates the private structural context block.
     * 
     * Persists the input channel.
     * 
     * @param[in] pin The physical analog ADC input pin for the sensor.
     * @param[in] sensitivity_factor The EMA smoothing factor for the sensor.
     */
    Impl(uint8_t pin, uint8_t sensitivity_factor) : pin(pin), sensitivity_factor(sensitivity_factor), smoothed_value(0), is_initialized(false) {
      pinMode(pin, INPUT);
    }

    uint16_t read(void) const {
      uint16_t raw = analogRead(this->pin);
      if (!this->is_initialized) {
        this->smoothed_value = raw;
        this->is_initialized = true;
      } else {
        // Apply an exponential smoothing algorithm to cut down on jitter.
        // See https://en.wikipedia.org/wiki/Exponential_smoothing for more information.
        int32_t diff = static_cast<int32_t>(raw) - static_cast<int32_t>(this->smoothed_value);
        this->smoothed_value = static_cast<uint16_t>(this->smoothed_value + (diff >> this->sensitivity_factor));
      }
      return this->smoothed_value;
    }
};

AnalogSensor::AnalogSensor(uint8_t pin, uint8_t sensitivity_factor): impl(new AnalogSensor::Impl(pin, sensitivity_factor)) {

}

AnalogSensor::~AnalogSensor() {
  delete this->impl;
}

uint16_t AnalogSensor::read() const {
  return this->impl->read();
}