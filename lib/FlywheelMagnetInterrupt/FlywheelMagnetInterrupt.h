/**
 * @file FlywheelMagnetInterrupt.h
 * @brief Hall Effect sensor interface for flywheel rotational period measurement on the AirYak engine subsystem.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Hardware tracking module designed to measure the microsecond interval between consecutive
 * flywheel magnet passes. Provides an isolated interface boundary to decouple hardware
 * edge-triggered interrupts from core RPM calculation loops.
 */

#ifndef AIRYAK_FLYWHEEL_MAGNET_INTERRUPT_H
#define AIRYAK_FLYWHEEL_MAGNET_INTERRUPT_H

#include <stdint.h>

/**
 * @class FlywheelMagnetInterrupt
 * @brief Manages the edge-triggered interrupt inputs from a flywheel Hall Effect sensor.
 * 
 * @note The baseline Predator engine platform only features a single flywheel assembly layout. 
 *       This hardware tracking interface is strictly intended to be instantiated once per application runtime context.
 */
class FlywheelMagnetInterrupt {
  public:

    /**
     * @brief Constructs a new Flywheel Magnet Interrupt instance.
     * 
     * Hooks into low-level micro-controller hardware registers to establish an edge-triggered interrupt behavior.
     * 
     * @param[in] pin The physical micro-controller digital GPIO pin connected to the sensor's signal path.
     */
    FlywheelMagnetInterrupt(uint8_t pin);

    /**
     * @brief Copy construction is explicitly disabled due to physical hardware resource pinning constraints.
     */
    FlywheelMagnetInterrupt(const FlywheelMagnetInterrupt&) = delete;

    /**
     * @brief Copy assignment is explicitly disabled due to physical hardware resource pinning constraints.
     */
    FlywheelMagnetInterrupt& operator=(const FlywheelMagnetInterrupt&) = delete;

    /**
     * @brief Retrieves the measured rotational period between the two most recent flywheel pulses.
     * 
     * Atomically reads the microsecond interval captured by the ISR and validates that the
     * signal has not gone stale. If more than 500,000 microseconds (500 ms) have elapsed
     * since the last detected pulse, the engine is treated as stalled or stationary.
     * 
     * @note Uses atomic register isolation internally to prevent multi-byte read tearing
     *       while the asynchronous ISR updates 32-bit timing registers.
     * @return uint32_t The elapsed time in microseconds between the last two falling edges,
     *                  or 0 if fewer than two pulses have occurred or the engine has stalled.
     */
    uint32_t getPeriodMicros(void) const;

    /**
     * @brief Destroys the instance and releases low-level hardware interrupt binding hooks safely.
     */
    ~FlywheelMagnetInterrupt(void);

  private:

    class Impl;
    Impl* impl; /**< Pointer to the private implementation object containing hardware ISR mapping states. */

};

#endif // AIRYAK_FLYWHEEL_MAGNET_INTERRUPT_H