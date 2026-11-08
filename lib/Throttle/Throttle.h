/**
 * @file Throttle.h
 * @brief Drive-by-wire Electronic Throttle Body (ETB) governor system for the AirYak ECU.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements a Proportional-Integral-Derivative (PID) closed-loop governor engine
 * to manage throttle actuator positioning metrics, safeguard against overspeed states,
 * and handle transition pathways during active engine heat profiling.
 */

#ifndef AIRYAK_THROTTLE_H
#define AIRYAK_THROTTLE_H

#include <stdint.h>

class Tachometer;

/**
 * @class Throttle
 * @brief Closed-loop governor interface tracking physical engine velocities and actuating throttle positions.
 */
class Throttle {
  public:

    /**
     * @brief Constructs a new closed-loop Throttle controller governor profile.
     * 
     * Sets up the private implementation data boundaries, binds low-level registers, 
     * and seeds initial target governor variables.
     * 
     * @param[in] tachometer Reference to the operational Tachometer speed measurement subsystem framework.
     * @param[in] input_pin Physical analog GPIO pin index linked to the pilot's accelerator potentiometer.
     * @param[in] output_pin Physical digital GPIO pin index mapped to the target Throttle Servo actuator interface.
     * @param[in] Kp Proportional tuning coefficient gain.
     * @param[in] Ki Integral tuning coefficient gain.
     * @param[in] Kd Derivative tuning coefficient gain.
     * @param[in] starting_throttle_position Baseline angular position index deployed during early starter cranking sweeps.
     * @param[in] dangerous_engine_speed Overspeed trip threshold boundary (RPM) to immediately invoke critical safety cuts.
     * @param[in] engine_idle_speed Targeted rotational velocity target (RPM) during baseline running conditions.
     * @param[in] max_engine_speed Maximum governed physical operational boundary target (RPM).
     * @param[in] throttle_servo_min_position Absolute mechanical protection limit (degrees) preventing actuator binding at zero.
     * @param[in] throttle_servo_max_position Absolute mechanical protection limit (degrees) preventing linkage stress at Wide Open Throttle.
     * @param[in] sweep_delay_millis The maximum number of milliseconds for the throttle plate to move from one extreme to the other.
     */
    Throttle(const Tachometer& tachometer, 
             uint8_t input_pin, 
             uint8_t output_pin, 
             double Kp, 
             double Ki, 
             double Kd, 
             uint8_t starting_throttle_position, 
             uint16_t dangerous_engine_speed, 
             uint16_t engine_idle_speed, 
             uint16_t max_engine_speed, 
             uint8_t throttle_servo_min_position, 
             uint8_t throttle_servo_max_position,
             uint16_t sweep_delay_millis
      );

    /**
     * @brief Copy construction is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Throttle(const Throttle&) = delete;

    /**
     * @brief Copy assignment is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Throttle& operator=(const Throttle&) = delete;

    /**
     * @brief Re-evaluates current tracking signals and drives the servo positioning step.
     * 
     * Calculates the error matrix between the driver's intent and target governor tracking bounds. 
     * Executes loop calculations, filters noise, and monitors for safety shutdowns.
     * 
     * @note This function is non-blocking and must be invoked continuously in the main thread.
     */
    void update(void);

    /**
     * @brief Instantly resets internal tracking terms (such as the integral error accumulator).
     * 
     * Call this during engine startup or stall recoveries to prevent integral windup issues.
     */
    void reset(void);

    /**
     * @brief Fetches the current target positioning layout computed by the controller core.
     * 
     * @return uint8_t The output actuation signal position currently applied to the target Servo hardware channel.
     */
    uint8_t getPosition(void) const;

    /**
     * @brief Destroys the Throttle context instance and safely clears internal allocations.
     */
    ~Throttle(void);
    
  private:
    class Impl;
    Impl* impl; /**< Pointer to the private implementation object (PIMPL pattern). */
};

#endif // AIRYAK_THROTTLE_H