#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>

// Mock hardware settings
#define INPUT 0
#define NOT_AN_INTERRUPT -1
#define FALLING 0
#define INPUT_PULLUP 2

#define P_ON_M 0
#define P_ON_E 1

extern unsigned long mock_millis;
extern unsigned long mock_micros;
extern void (*mock_isr_callback)(void);
extern uint16_t mock_analog_value;

inline void delay(unsigned long ms) {
    mock_millis += ms;
    mock_micros += ms * 1000UL;
}

inline void delayMicroseconds(unsigned long us) {
    mock_micros += us;
    mock_millis = mock_micros / 1000UL;
}

inline unsigned long millis() { return mock_millis; }
inline unsigned long micros() { return mock_micros; }

inline int digitalPinToInterrupt(uint8_t pin) { return (pin == 2 || pin == 3) ? (pin - 2) : NOT_AN_INTERRUPT; }

inline void pinMode(uint8_t pin, uint8_t mode) {}

inline void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode) {
    if (interruptNum == 0 || interruptNum == 1) {
        mock_isr_callback = userFunc;
    }
}

inline void detachInterrupt(int interruptNum) {
    if (interruptNum == 0 || interruptNum == 1) {
        mock_isr_callback = nullptr;
    }
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline uint16_t analogRead(uint8_t pin) { return mock_analog_value; }

template <typename T>
inline T abs(T x) {
    return (x < 0) ? -x : x;
}

#endif