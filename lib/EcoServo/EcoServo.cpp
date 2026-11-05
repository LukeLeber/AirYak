#include <Arduino.h>
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
      #ifdef DEBUG
        Serial.print("Initialized pin ");
        Serial.print(pin);
        Serial.print(" with written value ");
        Serial.println(position);
        Serial.print("EcoServo will wait ");
        Serial.print(sweep_delay_millis);
        Serial.println(" milliseconds before switching off");
      #endif
    }
};

EcoServo::EcoServo(uint8_t pin, uint16_t position, uint16_t sweep_delay_millis) : impl(new EcoServo::Impl(pin, position, sweep_delay_millis)) {

}

void EcoServo::setPosition(uint16_t position) {
  if (this->impl->position != position) {
    if (!this->impl->is_attached) {
      this->impl->output.attach(this->impl->pin);
      this->impl->is_attached = true;
      #ifdef DEBUG
        Serial.println("Turned on");
      #endif
    }
    this->impl->position = position;
    this->impl->output.write(this->impl->position);
    this->impl->last_command = static_cast<uint32_t>(millis());
    #ifdef DEBUG
    Serial.print("Wrote command ");
    Serial.print(position);
    Serial.print(" to EcoServo running on pin ");
    Serial.println(this->impl->pin);
    #endif
  }
}

void EcoServo::revalidateEcoMode(void) {
  if (this->impl->is_attached && ((uint32_t)millis() - this->impl->last_command) >= this->impl->sweep_delay_millis) {
    this->impl->output.detach();
    this->impl->is_attached = false;
    #ifdef DEBUG
      Serial.println("Turned off");
    #endif
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