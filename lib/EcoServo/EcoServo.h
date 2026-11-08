/**
 * @file EcoServo.h
 * @brief Power-optimized structural actuator module implementing transient attachment windows for the AirYak.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Mitigates current starvation, line sag, and mechanical hum in high-torque digital 
 * marine servos (e.g., MG90S) by dynamically isolating the control signal between commands.
 */

#ifndef AIRYAK_ECO_SERVO_H
#define AIRYAK_ECO_SERVO_H

#include <stdint.h>

/**
 * @class EcoServo
 * @brief Actuator interface managing on-demand physical signal attachment and power conservation.
 */
class EcoServo {
  public:
    /**
     * @brief Constructs a new power-optimized EcoServo controller profile.
     * 
     * Binds the tracking state to a target digital pin, initializes the home location, 
     * and configures the default temporal cushion required for mechanical travel.
     * 
     * @param[in] pin Physical digital GPIO channel mapped to the servo signal wire.
     * @param[in] position Initial target position value in angular degrees or pulse duration units.
     * @param[in] sweep_delay_millis Temporal tracking buffer (in milliseconds) allowed for structural transit.
     */
    EcoServo(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis);

    /**
     * @brief Copy construction is explicitly disabled due to physical hardware resource pinning constraints.
     */
    EcoServo(const EcoServo&) = delete;
    
    /**
     * @brief Copy assignment is explicitly disabled due to physical hardware resource pinning constraints.
     */
    EcoServo& operator=(const EcoServo&) = delete;

    /**
     * @brief Retrieves the last commanded target position assigned to the actuator.
     * 
     * @return uint16_t The active position target value.
     */
    uint16_t getPosition(void) const;

    /**
     * @brief Dispatches a new positioning target and immediately re-arms the transient power window.
     * 
     * If the new target introduces an angular delta, this method attaches the underlying 
     * hardware registers, streams the command, and restarts the internal non-blocking countdown timer.
     * 
     * @param[in] position The new target destination. Redundant identical commands are discarded.
     */
    void setPosition(uint16_t position);

    /**
     * @brief Assesses whether the actuator has safely transitioned into an isolated power-saving state.
     * 
     * @return true The physical signal line is currently silenced (detached), drawing zero steady-state current.
     * @return false The physical signal line is actively driven (attached) and executing a transit sweep.
     */
    bool isEcoMode(void) const;

    /**
     * @brief Evaluates active transit timers and automatically isolates the hardware pin upon sweep completion.
     * 
     * @note This function is non-blocking and must be invoked continuously in the main execution thread.
     *       It calculates temporal deltas against the current clock and breaks the physical connection 
     *       the moment the calculated mechanical sweep window expires to silence linkage hum and prevent sags.
     */
    void revalidateEcoMode(void);

    /**
     * @brief Destroys the EcoServo instance and cleanly releases internal memory allocations.
     */
    ~EcoServo(void);

  private:
    class Impl;
    Impl* impl; /**< Pointer to the private data block container (PIMPL pattern). */
};

#endif // AIRYAK_ECO_SERVO_H