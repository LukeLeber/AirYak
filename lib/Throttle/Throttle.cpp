/**
 * @file Throttle.cpp
 * @brief Proportional-Integral-Derivative (PID) closed-loop engine governor implementation.
 * 
 * Handles transition boundaries across distinct operational modes:
 * 1. Cranking/Static Mode: Engine off, locks valve at optimal mechanical starting baseline.
 * 2. Warmup Mode: Engine running, holds current position across active timeout timelines.
 * 3. Governed Runtime Mode: Adjusts target RPM using deadband-filtered inputs.
 * 4. Emergency Overspeed Cutout: Closes the valve completely if safety metrics are breached.
 */

#include <Arduino.h>
#include <EcoServo.h>
#include <Throttle.h>
#include <AnalogSensor.h>
#include <math.h>
#include <PID_v1.h>    // Standard official library interface for embedded target compilation

/**
 * @class Throttle::Impl
 * @brief Private data allocation block shielding governor system variables via the PIMPL pattern.
 */
class Throttle::Impl {
  public:
    double throttle_position;            /**< Calculated output tracking variable driven by governor core. */
    double target_speed;                 /**< Regulated target operational velocity in RPM. */
    double current_speed;                /**< Actively monitored rotational speed readout cache in RPM. */
    uint8_t starting_throttle_position;  /**< Hardcoded initial startup throttle valve angle. */
    uint16_t dangerous_engine_speed;     /**< Absolute hardware protection limit boundary (RPM). */
    uint32_t warmup_duration;            /**< Overall initialization sequence timeframe in milliseconds. */
    uint16_t engine_idle_speed;          /**< Lower mapping boundary value for governed pilot requests. */
    uint16_t max_engine_speed;           /**< Upper mapping boundary value for governed pilot requests. */
    uint32_t start_time;                 /**< Relative clock snapshot tracking when active runtime execution began. */
    uint8_t throttle_servo_min_position; /**< Angular position of the minimum throttle angle. */
    uint16_t sweep_delay_millis;
    PID pid;                             /**< Core automated feedback calculation engine instance. */

    const Tachometer& tachometer;        /**< Injected reference link targeting the velocity subsystem. */
    AnalogSensor input;                  /**< Potentiometer hardware signal converter monitor hook. */
    EcoServo output;                        /**< Controlled physical servo channel actuator reference layout. */

    Impl(const Tachometer& tachometer, uint8_t input_pin, uint8_t output_pin, double Kp, double Ki, double Kd, uint8_t starting_throttle_position, uint16_t dangerous_engine_speed, uint32_t warmup_duration, uint16_t engine_idle_speed, uint16_t max_engine_speed, uint8_t throttle_servo_min_position, uint8_t throttle_servo_max_position, uint16_t sweep_delay_millis):
      throttle_position(starting_throttle_position),
      target_speed(engine_idle_speed),
      current_speed(0.0),
      starting_throttle_position(starting_throttle_position),
      dangerous_engine_speed(dangerous_engine_speed),
      warmup_duration(warmup_duration),
      engine_idle_speed(engine_idle_speed),
      max_engine_speed(max_engine_speed),
      start_time(0),
      throttle_servo_min_position(throttle_servo_min_position),
      sweep_delay_millis(sweep_delay_millis),
      tachometer(tachometer),
      input(AnalogSensor(input_pin)),
      output(output_pin, starting_throttle_position, sweep_delay_millis),
      pid(&this->current_speed, &this->throttle_position, &this->target_speed, Kp, Ki, Kd, DIRECT) {
        this->pid.SetOutputLimits(throttle_servo_min_position, throttle_servo_max_position);
    }
};

Throttle::Throttle(const Tachometer& tachometer, uint8_t input_pin, uint8_t output_pin, double Kp, double Ki, double Kd, uint8_t starting_throttle_position, uint16_t dangerous_engine_speed, uint32_t warmup_duration, uint16_t engine_idle_speed, uint16_t max_engine_speed, uint8_t throttle_servo_min_position, uint8_t throttle_servo_max_position, uint16_t sweep_delay_millis): 
  impl(new Throttle::Impl(tachometer, input_pin, output_pin, Kp, Ki, Kd, starting_throttle_position, dangerous_engine_speed, warmup_duration, engine_idle_speed, max_engine_speed, throttle_servo_min_position, throttle_servo_max_position, sweep_delay_millis)) {}

Throttle::~Throttle(void) {
  delete this->impl;
}

void Throttle::update(void) {

  // Synchronize current loop speed registry cache with the external tachometer source
  this->impl->current_speed = (double)this->impl->tachometer.getSpeed();

  // Mode Paradigm 1: Engine is completely stationary or stalled
  if (this->impl->current_speed == 0) {
    // Ensure the PID controller doesn't try to take over during cranking
    if (this->impl->pid.GetMode() != MANUAL) {
      this->impl->pid.SetMode(MANUAL);
    }

    // Move the throttle position to the ideal angle if it's not already there.
    if (this->impl->throttle_position != this->impl->starting_throttle_position) {
      this->impl->throttle_position = this->impl->starting_throttle_position;
      this->impl->output.setPosition((uint8_t)this->impl->throttle_position);
    }
  }
  // Mode Paradigm 2: Engine operational within standard safe velocity metrics
  else if (this->impl->current_speed < this->impl->dangerous_engine_speed) {

    // Seed timing reference anchor precisely when combustion transitions out of stall
    if (this->impl->start_time == 0) {
      this->impl->start_time = millis();
    }

    unsigned long time_since_warmup = millis() - this->impl->start_time;

    // Operation Phase 2B: Warmup duration cleared - activate normal governed throttle actions
    if (time_since_warmup > this->impl->warmup_duration) {

      // Make sure the PID controller takes over.
      if (this->impl->pid.GetMode() != AUTOMATIC) {
        this->impl->pid.SetMode(AUTOMATIC);
      }

      // Map pilot demand inputs cleanly to speed references. 
      // Accounts for raw fixed-width uint16_t metrics scaling ranges (0-1023).
      double new_target_speed = map(
        (long)this->impl->input.read(),
        0, 1023,
        (long)this->impl->engine_idle_speed,
        (long)this->impl->max_engine_speed
      );

      #ifdef DEBUG
        Serial.print("Throttle position: ");
        Serial.println(new_target_speed);
      #endif

      // Deadband Input Filter: Rejects micro-voltage signal noise at rest to target 10 RPM increments
      if (abs(new_target_speed - this->impl->target_speed) > 9) {
        this->impl->target_speed = new_target_speed;
        #ifdef DEBUG
          Serial.print("New target speed: ");
          Serial.print(new_target_speed);
          Serial.println("RPM");
        #endif
      }

      double old_throttle_position = this->impl->throttle_position;
      this->impl->pid.Compute();

      // Deadband Output Filter: Stabilizes the physical servo against control line chatter
      if (abs(this->impl->throttle_position - old_throttle_position) > 1) {
        this->impl->output.setPosition((uint8_t)this->impl->throttle_position);
        #ifdef DEBUG
          Serial.print("Throttle is moving to ");
          Serial.println(this->impl->throttle_position);
        #endif
      }
    }
    // Operation Phase 2A: Warmup cycle active. 
    // Intentionally no-op to let the throttle hold position without active servo corrections.
  }
  // Mode Paradigm 3: Overspeed condition breached. Force safety override cutout path immediately.
  else {
    // Ensure the PID controller doesn't try to take over.
    if (this->impl->pid.GetMode() != MANUAL) {
      this->impl->pid.SetMode(MANUAL);
    }

    // Completely close the throttle to minimize fuel intake.
    if (this->impl->throttle_position != this->impl->throttle_servo_min_position) {
      this->impl->throttle_position = this->impl->throttle_servo_min_position;
      this->impl->output.setPosition(this->impl->throttle_position);
      #ifdef DEBUG
        Serial.println("Throttle is moving to closed position");
      #endif
    }
  }
}

void Throttle::reset(void) {
  // Clear the internal warmup timing anchor to allow clean cycle restarts
  this->impl->start_time = 0;
}

uint8_t Throttle::getPosition(void) const {
  return (uint8_t)this->impl->throttle_position;
}