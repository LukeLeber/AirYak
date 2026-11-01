#ifndef MOCK_SERVO_H
#define MOCK_SERVO_H

#include <stdint.h>

class Servo {
  private:
    uint8_t _position = 82;
    bool _is_attached = false;
  public:
    void write(uint8_t value) { _position = value; }
    
    void attach(uint8_t pin) { _is_attached = true; } 
    void detach() { _is_attached = false; }
    
    uint8_t read() const { return _position; }
    bool attached() const { return _is_attached; } // Expose for mock validation
};

#endif