// tests/mock_arduino/Arduino.cpp
#include "Arduino.h"

// Define the global mock storage variables once for the linker
unsigned long mock_millis = 0;
void (*mock_isr_callback)(void) = nullptr;
uint16_t mock_analog_value = 0;