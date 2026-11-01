#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>

// Mock hardware settings
#define INPUT 0
#define NOT_AN_INTERRUPT -1
#define FALLING 0
#define INPUT_PULLUP 2

extern unsigned long mock_millis;
extern void (*mock_isr_callback)(void); // Expose the registered function pointer
extern uint16_t mock_analog_value;

inline void delay(unsigned long ms) { mock_millis += ms; }
inline void delayMicroseconds(unsigned long us) { }
inline unsigned long millis() { return mock_millis; }
inline int digitalPinToInterrupt(uint8_t pin) { return (pin == 2) ? 0 : NOT_AN_INTERRUPT; }
inline void pinMode(uint8_t pin, uint8_t mode) {}

inline void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode) {
    if (interruptNum == 0) {
        mock_isr_callback = userFunc; // Save the registered callback address
    }
}

inline void detachInterrupt(int interruptNum) {
    if (interruptNum == 0) {
        mock_isr_callback = nullptr;
    }
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline uint16_t analogRead(uint8_t pin) { return mock_analog_value; }

#endif