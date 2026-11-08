/**
 * @file Tachometer.h
 * @brief Rotational velocity metrics engine for the AirYak ECU subsystem.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Computes crankshaft revolutions per minute (RPM) by monitoring rotational
 * period intervals from a localized private hardware interrupt
 * line.
 */

#ifndef AIRYAK_TACHOMETER_H
#define AIRYAK_TACHOMETER_H

#include <stdint.h>

/**
 * @class Tachometer
 * @brief Manages sampling logic and time-scaling conversions for engine speed metrics.
 */
class Tachometer {
  public:
    /**
     * @brief Constructs a new Tachometer processor.
     * 
     * Internally provisions and links a private hardware interrupt channel 
     * on the designated hardware pin.
     * 
     * @param[in] pin The physical micro-controller digital GPIO pin mapped to the speed sensor.
     */
    Tachometer(uint8_t pin);

    /**
     * @brief Copy construction is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Tachometer(const Tachometer&) = delete;

    /**
     * @brief Copy assignment is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Tachometer& operator=(const Tachometer&) = delete;

    /**
     * @brief Evaluates time delta metrics and updates the current speed register.
     * 
     * @note This function is non-blocking and must be invoked continuously in the main loop.
     *       It calculates intermediate steps using dynamic elapsed time scale conversions.
     */
    void resample();

    /**
     * @brief Fetches the most recently processed crankshaft velocity calculation.
     * 
     * @return uint16_t The engine rotational speed value in Revolutions Per Minute (RPM).
     */
    uint16_t getSpeed() const;

    /**
     * @brief Destroys the context and cleans up internal data instances safely.
     */
    ~Tachometer(void);
    
  private:
    class Impl;
    Impl* impl; /**< Pointer to the private data container (PIMPL pattern). */
};

#endif // AIRYAK_TACHOMETER_H