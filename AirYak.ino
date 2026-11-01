/**
 * @file AirYak.ino
 * @brief Primary Engine Control Unit (ECU) main coordination firmware loop.
 * 
 * Hardware Platform: Custom ATmega328P Standalone Circuitry Configuration
 * Clock Core: 16.0 MHz External Oscillating Quartz Crystal Array
 * 
 * System Loop Features:
 * - Asynchronous Hall Effect Crankshaft Pulses Tracking Layer
 * - Time-Scaled Linear Thermal Choke Profiling Governor
 * - Closed-Loop Proportional-Integral-Derivative (PID) Electronic Governor
 * - Airfoil Rudder Steering Control
 */

#include <Arduino.h>
#include <AirYakConfig.h>
#include <Choke.h>
#include <Tachometer.h>
#include <Throttle.h>
#include <Rudder.h>

#define DEBUG 1

// --- Governor Calibration Profiles ---
#define STARTING_THROTTLE_POSITION    10   /**< Initial mechanical throttle opening angle during cranking (degrees). */
#define IDLE_ENGINE_SPEED             1500 /**< Targeted baseline controlled minimum idling limit (RPM). */
#define MAX_ENGINE_SPEED              3600 /**< Maximum allowed governed target speed via driver demand (RPM). */
#define DANGEROUS_ENGINE_SPEED        4000 /**< Absolute overspeed boundary line threshold passed to the governor core (RPM). */

// --- Physical ATMega328P Pin Allocation Matrix ---
#define THROTTLE_ANALOG_IN_PIN        A2   /**< ADC input linked to driver's throttle demand potentiometer. */
#define RUDDER_ANALOG_IN_PIN          A0   /**< ADC input linked to driver's rudder demand potentiometer. */
#define TACHOMETER_DIGITAL_IN_PIN     3    /**< Hardwired external interrupt channel 1 (INT1). */
#define THROTTLE_PWM_OUT_PIN          9    /**< Dedicated timer counter PWM actuator output for throttle servo. */
#define CHOKE_PWM_OUT_PIN             11   /**< Dedicated timer counter PWM actuator output for choke servo. */
#define RUDDER_PWM_OUT_PIN            13   /**< Dedicated timer counter PWM actuator output for rudder servo. */

// --- Output Actuator Software Tuning ---
#define CHOKE_SWEEP_DELAY_MILLIS      150 /**< The number of milliseconds that it takes for the choke actuator to travel 90 degrees. */
#define THROTTLE_SWEEP_DELAY_MILLIS   150 /**< The number of milliseconds that it takes for the throttle actuator to travel 90 degrees. */
#define RUDDER_SWEEP_DELAY_MILLIS     300 /**< The number of milliseconds that it takes for the rudder actuator to travel 180 degrees. */

// --- Mechanical Actuator Target Bounds ---
#define THROTTLE_SERVO_MIN_POSITION   0    /**< Absolute lower limit stopping point constraint to prevent linkage binding. */
#define THROTTLE_SERVO_MAX_POSITION   90   /**< Absolute Wide Open Throttle mechanical link constraint upper bound. */
#define CHOKE_SERVO_MIN_POSITION      0    /**< Mechanical baseline target for a fully open choke butterfly valve. */
#define CHOKE_SERVO_MAX_POSITION      90   /**< Mechanical baseline target for a fully closed choke butterfly valve. */
#define RUDDER_SERVO_MIN_POSITION     0    /**< Mechanical baseline target for a rudder all the way port. */
#define RUDDER_SERVO_MAX_POSITION     180  /**< Mechanical baseline target for a rudder all the way starboard. */

// --- Timing Metrics Settings ---
#define WARMUP_DURATION               10000UL /**< Overall target timed cold-start profile window (10.0 Seconds). */

// --- Algorithmic Controller Coefficients ---
#define PROPORTIONAL_GAIN             2.0  /**< Proportional (Kp) tuning constant parameter step multiplier. */
#define INTEGRAL_GAIN                 5.0  /**< Integral (Ki) tuning constant parameter error accumulator loop window. */
#define DERIVATIVE_GAIN               1.0  /**< Derivative (Kd) tuning constant parameter tracking dampener. */

// --- Subsystem Instance Interfaces ---
Tachometer* tachometer;
Choke* choke;
Throttle* throttle;
Rudder* rudder;

/**
 * @brief Initializes peripheral profiles, anchors hardware lines, and links variables.
 */
void setup() {
  AIRYAK_DEBUG_START();
  tachometer = new Tachometer(TACHOMETER_DIGITAL_IN_PIN);
  choke = new Choke(
    CHOKE_PWM_OUT_PIN, 
    CHOKE_SERVO_MAX_POSITION, 
    CHOKE_SERVO_MIN_POSITION, 
    WARMUP_DURATION, 
    CHOKE_SWEEP_DELAY_MILLIS
  );
  throttle = new Throttle(
    *tachometer, 
    THROTTLE_ANALOG_IN_PIN, 
    THROTTLE_PWM_OUT_PIN, 
    PROPORTIONAL_GAIN, 
    INTEGRAL_GAIN, 
    DERIVATIVE_GAIN, 
    STARTING_THROTTLE_POSITION, 
    DANGEROUS_ENGINE_SPEED, 
    WARMUP_DURATION, 
    IDLE_ENGINE_SPEED, 
    MAX_ENGINE_SPEED, 
    THROTTLE_SERVO_MIN_POSITION, 
    THROTTLE_SERVO_MAX_POSITION,
    THROTTLE_SWEEP_DELAY_MILLIS
  );
  rudder = new Rudder(RUDDER_ANALOG_IN_PIN, RUDDER_PWM_OUT_PIN, RUDDER_SWEEP_DELAY_MILLIS);
}

/**
 * @brief Continuous execution main thread processing logic.
 */
void loop() {
  // Update current rotational metrics from the crank sensor
  tachometer->resample();

  // Lifecycle Coordination State Check: If engine stalls, flush state timers instantly
  if (tachometer->getSpeed() == 0) {
    choke->reset();
    throttle->reset();
  }

  // Adjust output servo states.
  choke->update();
  throttle->update();
  rudder->update();
}
