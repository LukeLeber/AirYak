/**
 * @file AnalogSensor.h
 * @brief Encapsulates a physical analog sensor.
 * @author Luke Leber
 * @version 1.0.0
 */
#ifndef AIRYAK_ANALOG_SENSOR_H
#define AIRYAK_ANALOG_SENSOR_H

#include <stdint.h>

/**
 * @class AnalogSensor
 * @brief Encapsulates a physical analog sensor.
 */
class AnalogSensor {

  public:

    /**
     * @brief Constructs a new analog sensor instance.
     * 
     * @param[in] pin The physical PWM input pin for the sensor.
     */
    AnalogSensor(uint8_t pin);

    /**
     * @brief Destroys an analog sensor instance.
     */
    ~AnalogSensor(void);

    /**
     * @brief Executes a sample request using a strict cross-platform type framework.
     * @return uint16_t Guaranteed 16-bit data container matching all ADC hardware spreads.
     */
    uint16_t read() const;

  private:
    class Impl;
    Impl* impl; /**< Pointer to the private implementation object (PIMPL pattern). */
};

#endif // AIRYAK_ANALOG_SENSOR_H