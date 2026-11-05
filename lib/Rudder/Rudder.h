/**
 * @file Rudder.h
 * @brief Airfoil rudder steering subsystem for the AirYak.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Manages the airfoil steering subsystem by passing potentiometer voltages through a 
 * localized rolling-window finite impulse response (FIR) filter to isolate positioning commands.
 */

#ifndef AIRYAK_RUDDER_H
#define AIRYAK_RUDDER_H

#include <stdint.h>

/**
 * @class Rudder
 * @brief Interface mapping input tracking metrics directly onto physical steering actuators.
 */
class Rudder {
  public:
    /**
     * @brief Constructs a new Rudder flight management profile.
     * 
     * Hooks into the target input ADC line and attaches physical output servo control registers.
     * 
     * @param[in] input_pin Physical analog ADC channel linked to the steering yoke potentiometer.
     * @param[in] output_pin Physical digital GPIO pin mapped to the steering servo.
     * @param[in] sweep_delay_millis The number of milliseconds that the output actuator takes to move.
     */
    Rudder(uint8_t input_pin, uint8_t output_pin, uint16_t sweep_delay_millis);

    /**
     * @brief Destroys the Rudder instance and cleanly releases internal memory allocations.
     */
    ~Rudder();

    /**
     * @brief Polls the input channel and updates the steering actuator.
     */
    void update();
    
    uint8_t getPosition(void) const;

  private:
    class Impl;
    Impl* impl; /**< Pointer to the private data block container (PIMPL pattern). */
};

#endif // AIRYAK_RUDDER_H
