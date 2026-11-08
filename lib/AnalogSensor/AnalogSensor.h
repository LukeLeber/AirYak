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
 * 
 * This class applies an exponential smoothing algorithm to the raw values read
 * in from the underlying sensor. This helps to keep clean, gradual signals and
 * prevent noise and jitter. The higher the sensitivity factor, the more
 * smoothing is applied.
 */
class AnalogSensor {

  public:

    /**
     * @brief Constructs a new analog sensor instance.
     * 
     * @param[in] pin The physical analog ADC input pin for the sensor.
     * @param[in] sensitivity_factor The EMA smoothing factor for the sensor.
     */
    AnalogSensor(uint8_t pin, uint8_t sensitivity_factor = 2);

    // Prevent copies via copy constructor.
    AnalogSensor(const AnalogSensor&) = delete;

    // Prevent copies via assignment.
    AnalogSensor& operator=(const AnalogSensor&) = delete;

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