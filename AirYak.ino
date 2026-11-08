/**
 * @file AirYak.ino
 * @brief Primary Engine Control Unit (ECU) main coordination firmware loop.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Hardware Platform: Custom ATmega328P Standalone Circuitry Configuration
 * Clock Core: 16.0 MHz External Oscillating Quartz Crystal Array
 * 
 * System Loop Features:
 * - Asynchronous Hall Effect Crankshaft Period Tracking Layer
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


// --- Governor Calibration Profiles ---
#define STARTING_THROTTLE_POSITION    5   /**< Initial mechanical throttle opening angle during cranking (degrees). */
#define IDLE_ENGINE_SPEED             1200 /**< Targeted baseline controlled minimum idling limit (RPM). */
#define MAX_ENGINE_SPEED              3600 /**< Maximum allowed governed target speed via driver demand (RPM). */
#define DANGEROUS_ENGINE_SPEED        4000 /**< Absolute overspeed boundary line threshold passed to the governor core (RPM). */

// --- Physical ATMega328P Pin Allocation Matrix ---
#define THROTTLE_ANALOG_IN_PIN        A2   /**< ADC input linked to driver's throttle demand potentiometer. */
#define RUDDER_ANALOG_IN_PIN          A0   /**< ADC input linked to driver's rudder demand potentiometer. */
#define TACHOMETER_DIGITAL_IN_PIN     3    /**< Hardwired external interrupt channel 1 (INT1). */
#define THROTTLE_PWM_OUT_PIN          9    /**< Dedicated timer counter PWM actuator output for throttle servo. */
#define CHOKE_PWM_OUT_PIN             11   /**< Dedicated timer counter PWM actuator output for choke servo. */
#define RUDDER_PWM_OUT_PIN            7   /**< Dedicated timer counter PWM actuator output for rudder servo. */

// --- Output Actuator Software Tuning ---
#define CHOKE_SWEEP_DELAY_MILLIS      150 /**< The number of milliseconds that it takes for the choke actuator to travel 90 degrees. */
#define THROTTLE_SWEEP_DELAY_MILLIS   150 /**< The number of milliseconds that it takes for the throttle actuator to travel 90 degrees. */
#define RUDDER_SWEEP_DELAY_MILLIS     300 /**< The number of milliseconds that it takes for the rudder actuator to travel 180 degrees. */

// --- Mechanical Actuator Target Bounds ---
#define THROTTLE_SERVO_MIN_POSITION   0    /**< Absolute lower limit stopping point constraint to prevent linkage binding. */
#define THROTTLE_SERVO_MAX_POSITION   90   /**< Absolute Wide Open Throttle mechanical link constraint upper bound. */
#define CHOKE_SERVO_MIN_POSITION      0    /**< Mechanical baseline target for a fully closed choke butterfly valve. */
#define CHOKE_SERVO_MAX_POSITION      90   /**< Mechanical baseline target for a fully open choke butterfly valve. */
#define RUDDER_SERVO_MIN_POSITION     0    /**< Mechanical baseline target for a rudder all the way port. */
#define RUDDER_SERVO_MAX_POSITION     180  /**< Mechanical baseline target for a rudder all the way starboard. */

// --- Timing Metrics Settings ---
#define WARMUP_DURATION               10000UL /**< Overall target timed cold-start profile window (10.0 Seconds). */

// --- Algorithmic Controller Coefficients ---
#define PROPORTIONAL_GAIN             0.004  /**< Gentle proportional response */
#define INTEGRAL_GAIN                 0.01   /**< Slow, smooth integral trim */
#define DERIVATIVE_GAIN               0.0    /**< MUST be 0.0; single-cylinder pulses cause Kd spikes */

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
    *tachometer,
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
  static bool engine_was_running = false;
  static unsigned long last_time_speed_seen = 0;
  
  tachometer->resample();
  unsigned long now = millis();

  // Print pacing gate (0.25 seconds)
  static unsigned long last_print = 0;
  if (now - last_print >= 250UL) {
    AIRYAK_DEBUG_METRIC("TACH READOUT:", tachometer->getSpeed(), "RPM");
    last_print = now;
  }

  uint16_t current_rpm = tachometer->getSpeed();

  if (current_rpm >= 1000) {
    engine_was_running = true;
    last_time_speed_seen = now; // Lock timestamp anchor whenever active speed is verified
  }

  // Robust Stall Watchdog: Only reset state profiles if the engine has been 
  // completely dead for a continuous 500ms block window
  if (current_rpm == 0 && engine_was_running) {
    if (now - last_time_speed_seen >= 500UL) { 
      choke->reset();
      throttle->reset();
      engine_was_running = false;
      AIRYAK_DEBUG_PRINTLN("--- TRUE ENGINE STALL DECLARED; SYSTEM RESET ---");
    }
  }

  choke->update();
  throttle->update();
  rudder->update();
}
