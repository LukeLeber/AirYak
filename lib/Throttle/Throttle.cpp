/**
 * @file Throttle.cpp
 * @brief Proportional-Integral-Derivative (PID) closed-loop engine governor implementation.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements closed-loop engine RPM regulation using the Pointer to Implementation
 * (PIMPL) idiom. Handles transition boundaries across three operational modes:
 * 1. Cranking/Stall Mode (< 1000 RPM): Locks the throttle at the mechanical starting baseline.
 * 2. Governed Runtime Mode: Regulates target RPM using deadband-filtered pilot inputs,
 *    slew-rate clamping, and output hysteresis.
 * 3. Emergency Overspeed Cutout: Closes the valve completely if safety limits are breached.
 */

#include <Throttle.h>
#include <Arduino.h>
#include <AirYakConfig.h>
#include <EcoServo.h>
#include <Tachometer.h>
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
    uint16_t engine_idle_speed;          /**< Lower mapping boundary value for governed pilot requests. */
    uint16_t max_engine_speed;           /**< Upper mapping boundary value for governed pilot requests. */
    uint8_t throttle_servo_min_position; /**< Angular position of the minimum throttle angle. */
    PID pid;                             /**< Core automated feedback calculation engine instance. */
    float last_sent_position;
    const Tachometer& tachometer;        /**< Injected reference link targeting the velocity subsystem. */
    AnalogSensor input;                  /**< Potentiometer hardware signal converter monitor hook. */
    EcoServo output;                        /**< Controlled physical servo channel actuator reference layout. */

    Impl(const Tachometer& tachometer, uint8_t input_pin, uint8_t output_pin, double Kp, double Ki, double Kd, uint8_t starting_throttle_position, uint16_t dangerous_engine_speed, uint16_t engine_idle_speed, uint16_t max_engine_speed, uint8_t throttle_servo_min_position, uint8_t throttle_servo_max_position, uint16_t sweep_delay_millis):
      throttle_position(starting_throttle_position),
      target_speed(engine_idle_speed),
      current_speed(0.0),
      starting_throttle_position(starting_throttle_position),
      dangerous_engine_speed(dangerous_engine_speed),
      engine_idle_speed(engine_idle_speed),
      max_engine_speed(max_engine_speed),
      throttle_servo_min_position(throttle_servo_min_position),
      last_sent_position(-10.0f),
      tachometer(tachometer),
      input(input_pin, 0),
      output(output_pin, starting_throttle_position, sweep_delay_millis),
      pid(&this->current_speed, &this->throttle_position, &this->target_speed, Kp, Ki, Kd, P_ON_E, DIRECT) {
        this->pid.SetOutputLimits(throttle_servo_min_position, throttle_servo_max_position);
        this->pid.SetSampleTime(50);
        AIRYAK_DEBUG_METRIC("Throttle initialized to", starting_throttle_position, "degrees");
    }

    void update(void) {
      // Synchronize current loop speed registry cache with the external tachometer source
      this->current_speed = (double)this->tachometer.getSpeed();

      // Mode Paradigm 1: Engine is completely stationary or stalled
      if (this->current_speed < 1000) {
        // Ensure the PID controller doesn't try to take over during cranking
        if (this->pid.GetMode() != MANUAL) {
          this->pid.SetMode(MANUAL);
          AIRYAK_DEBUG_PRINTLN("Throttle transitioned to manual PID controller");
        }

        // Move the throttle position to the ideal angle if it's not already there.
        if (this->throttle_position != this->starting_throttle_position) {
          this->throttle_position = this->starting_throttle_position;
          this->output.setPosition((uint8_t)this->throttle_position);
          this->last_sent_position = (float)this->throttle_position;
          AIRYAK_DEBUG_METRIC("Engine stalled; moving throttle to", this->starting_throttle_position, "degrees");
        }
      }
      // Mode Paradigm 2: Engine operational within standard safe velocity metrics
      else if (this->current_speed < this->dangerous_engine_speed) {

        // Make sure the PID controller takes over.
        if (this->pid.GetMode() != AUTOMATIC) {
          this->pid.SetMode(AUTOMATIC);
          AIRYAK_DEBUG_PRINTLN("Throttle transitioned to automatic PID controller");
        }

        // Map pilot demand inputs cleanly to speed references. 
        // Accounts for raw fixed-width uint16_t metrics scaling ranges (0-1023).
        double new_target_speed = map(
          (long)this->input.read(),
          0, 1023,
          (long)this->engine_idle_speed,
          (long)this->max_engine_speed
        );

        // HACK: the throttle joystick is physically damaged and another won't be here for a few days.
        // new_target_speed = 1200;

        // Input Deadband Filter: Rejects micro-voltage signal noise
        if (abs(new_target_speed - this->target_speed) > 90) {
          this->target_speed = new_target_speed;
          AIRYAK_DEBUG_METRIC("Throttle responding to new target speed of", new_target_speed, "RPM");
        }

        double prev_pid_output = this->throttle_position;

        if (abs(this->current_speed - this->target_speed) > 100.0) {
          this->pid.Compute();
          // Clamp max change to 1.0 degree per 50ms sample to stop multi-degree spikes
          double delta = this->throttle_position - prev_pid_output;
          if (delta > 1.0) this->throttle_position = prev_pid_output + 1.0;
          else if (delta < -1.0) this->throttle_position = prev_pid_output - 1.0;
        }

        if (abs((float)this->throttle_position - last_sent_position) >= 1.5f) {
          uint8_t target_servo_angle = (uint8_t)round(this->throttle_position);
          this->output.setPosition(target_servo_angle);
          last_sent_position = (float)target_servo_angle;
          AIRYAK_DEBUG_METRIC("Throttle plate moving to", target_servo_angle, "degrees");
        }

      }
      // Mode Paradigm 3: Overspeed condition breached. Force safety override cutout path immediately.
      else {
        // Ensure the PID controller doesn't try to take over.
        if (this->pid.GetMode() != MANUAL) {
          this->pid.SetMode(MANUAL);
        }

        // Completely close the throttle to minimize fuel intake.
        if (this->throttle_position != this->throttle_servo_min_position) {
          this->throttle_position = this->throttle_servo_min_position;
          this->output.setPosition(this->throttle_position);
          this->last_sent_position = (float)this->throttle_position;
          AIRYAK_DEBUG_METRIC("DANGEROUS SPEED DETECTED! Throttle plate moved to", this->throttle_position, "degrees");
        }
      }
    }

    uint8_t getPosition(void) const {
      return (uint8_t)this->throttle_position;
    }

    void reset(void) {
      this->pid.SetMode(MANUAL);
      this->throttle_position = this->starting_throttle_position;
      this->last_sent_position = (float)this->starting_throttle_position;
      this->output.setPosition(this->starting_throttle_position);
    }
};

Throttle::Throttle(const Tachometer& tachometer, uint8_t input_pin, uint8_t output_pin, double Kp, double Ki, double Kd, uint8_t starting_throttle_position, uint16_t dangerous_engine_speed, uint16_t engine_idle_speed, uint16_t max_engine_speed, uint8_t throttle_servo_min_position, uint8_t throttle_servo_max_position, uint16_t sweep_delay_millis): 
  impl(new Throttle::Impl(tachometer, input_pin, output_pin, Kp, Ki, Kd, starting_throttle_position, dangerous_engine_speed, engine_idle_speed, max_engine_speed, throttle_servo_min_position, throttle_servo_max_position, sweep_delay_millis)) {}

void Throttle::update(void) {
  this->impl->update();
}

uint8_t Throttle::getPosition(void) const {
  return this->impl->getPosition();
}

void Throttle::reset(void) {
  this->impl->reset();
}

Throttle::~Throttle(void) {
  delete this->impl;
}