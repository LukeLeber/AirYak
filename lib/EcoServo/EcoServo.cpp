/**
 * @file EcoServo.cpp
 * @brief Implementation details for the AirYak power-optimized EcoServo actuator controller.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Implements transient PWM attachment windows and non-blocking idle timeout detaching
 * using the Pointer to Implementation (PIMPL) idiom to conserve power and prevent line sag.
 */

#include <EcoServo.h>
#include <Arduino.h>
#include <AirYakConfig.h>
#include <Servo.h>

class EcoServo::Impl {
  public:

    Impl(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis): pin(pin), position(position), sweep_delay_millis(sweep_delay_millis) {
      this->output.attach(pin);
      this->output.write(position);
      this->is_attached = true;
      this->last_command = (uint32_t)millis();
      AIRYAK_DEBUG_METRIC("EcoServo initialized with", sweep_delay_millis, "milliseconds idle time before power off.");
    }

    void setPosition(uint16_t position) {
      if (this->position != position) {
        if (!this->is_attached) {
          this->output.attach(this->pin);
          this->is_attached = true;
          AIRYAK_DEBUG_PRINTLN("EcoServo reattached");
        }
        this->position = position;
        this->output.write(this->position);
        this->last_command = static_cast<uint32_t>(millis());
      }
    }

    void revalidateEcoMode(void) {
      if (this->is_attached && ((uint32_t)millis() - this->last_command) >= this->sweep_delay_millis) {
        this->output.detach();
        this->is_attached = false;
        AIRYAK_DEBUG_PRINTLN("EcoServo detached");
      }
    }

    uint16_t getPosition(void) const {
      return this->position;
    }

    bool isEcoMode(void) const {
      return !this->is_attached;
    }

  private:
    uint8_t pin;
    Servo output;
    uint16_t position;
    uint32_t last_command;
    uint16_t sweep_delay_millis;
    bool is_attached;
};

EcoServo::EcoServo(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis) : impl(new EcoServo::Impl(pin, position, sweep_delay_millis)) {

}

void EcoServo::setPosition(uint16_t position) {
  this->impl->setPosition(position);
}

void EcoServo::revalidateEcoMode(void) {
  this->impl->revalidateEcoMode();
}

uint16_t EcoServo::getPosition(void) const {
  return this->impl->getPosition();
}

bool EcoServo::isEcoMode(void) const {
  return this->impl->isEcoMode();
}

EcoServo::~EcoServo(void) {
  delete this->impl;
}