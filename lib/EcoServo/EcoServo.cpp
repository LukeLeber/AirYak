/**
 * @file EcoServo.cpp
 * @brief Implementation details for the AirYak power-optimized EcoServo actuator controller.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements transient PWM attachment windows and non-blocking idle timeout detaching
 * using the Pointer to Implementation (PIMPL) idiom to conserve power and prevent line sag.
 */

#include <Arduino.h>
#include <AirYakConfig.h>
#include <Servo.h>
#include <EcoServo.h>

class EcoServo::Impl {
  public:
    uint8_t pin;
    Servo output;

    uint16_t position;
    uint32_t last_command;
    uint16_t sweep_delay_millis;

    bool is_attached;

    Impl(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis): pin(pin), position(position), sweep_delay_millis(sweep_delay_millis) {
      this->output.attach(pin);
      this->output.write(position);
      this->is_attached = true;
      this->last_command = (uint32_t)millis();
      AIRYAK_DEBUG_METRIC("EcoServo initialized with", sweep_delay_millis, "milliseconds idle time before power off.");
    }
};

EcoServo::EcoServo(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis) : impl(new EcoServo::Impl(pin, position, sweep_delay_millis)) {

}

void EcoServo::setPosition(uint16_t position) {
  if (this->impl->position != position) {
    if (!this->impl->is_attached) {
      this->impl->output.attach(this->impl->pin);
      this->impl->is_attached = true;
      AIRYAK_DEBUG_PRINTLN("EcoServo reattached");
    }
    this->impl->position = position;
    this->impl->output.write(this->impl->position);
    this->impl->last_command = static_cast<uint32_t>(millis());
  }
}

void EcoServo::revalidateEcoMode(void) {
  if (this->impl->is_attached && ((uint32_t)millis() - this->impl->last_command) >= this->impl->sweep_delay_millis) {
    this->impl->output.detach();
    this->impl->is_attached = false;
    AIRYAK_DEBUG_PRINTLN("EcoServo detached");
  }
}

uint16_t EcoServo::getPosition(void) const {
  return this->impl->position;
}

bool EcoServo::isEcoMode(void) const {
  return !this->impl->is_attached;
}

EcoServo::~EcoServo(void) {
  delete this->impl;
}