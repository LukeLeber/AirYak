/**
 * @file Choke.h
 * @brief Automated electronic choke control mechanism for the AirYak engine subsystem.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Tracks engine startup sequences and handles progressive positional transition
 * profiles for fuel enrichment actuators (Servos) across high-rollover time vectors.
 */

#ifndef AIRYAK_CHOKE_H
#define AIRYAK_CHOKE_H

#include <stdint.h>

// Forward declare tachometer to reduce coupling.
class Tachometer;

/**
 * @class Choke
 * @brief Manages the state machine and linear positioning profile of an engine choke valve.
 * 
 * The class transitions an enrichment valve from an engaged position to an unengaged
 * position over a fixed timeframe. It uses non-blocking math and handles internal clock rollovers.
 */
class Choke {
  public:

    /**
     * @brief Constructs a new Choke controller instance.
     * 
     * Initializes the state tracking cache to the unengaged (off) state as a safety fallback.
     * 
     * @param[in] tachometer Reference to the tachometer subsystem.
     * @param[in] pin The physical PWM output pin for the choke servo.
     * @param[in] choke_off_position The angular position (in degrees) when the choke is fully open/disengaged.
     * @param[in] choke_full_position The angular position (in degrees) when the choke is fully closed/engaged.
     * @param[in] warmup_duration The overall runtime interval for the choke lifecycle in milliseconds.
     * @param[in] sweep_delay_millis The maximum number of milliseconds for the choke plate to move from one extreme to the other.
     */
    Choke(const Tachometer& tachometer, uint8_t pin, uint8_t choke_off_position, uint8_t choke_full_position, uint32_t warmup_duration, uint16_t sweep_delay_millis);

    /**
     * @brief Copy construction is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Choke(const Choke&) = delete;

    /**
     * @brief Copy assignment is explicitly disabled due to physical hardware resource pinning constraints.
     */
    Choke& operator=(const Choke&) = delete;

    /**
     * @brief Restores the state machine back to its cold-start baseline conditions.
     * 
     * Call this whenever an engine start condition or cranking event is triggered.
     * The next time Choke::update() is called, it will immediately re-engage the full choke.
     */
    void reset(void);

    /**
     * @brief Evaluates the current timeline state and updates the physical actuator position.
     * 
     * @note This function is non-blocking and must be invoked continuously inside the main application loop.
     *       It calculates intermediate steps using active time mapping and suppresses redundant servo writes.
     */
    void update(void);

    /**
     * @brief Retrieves the last commanded target position assigned to the actuator.
     * 
     * @return uint16_t The active position target value.
     */
    uint16_t getPosition(void) const;
    
    /**
     * @brief Destroys the Choke instance, safely freeing internal implementation memory.
     */
    ~Choke(void);

  private:
    class Impl;
    Impl* impl; /**< Pointer to the private implementation object (PIMPL pattern). */
};

#endif // AIRYAK_CHOKE_H