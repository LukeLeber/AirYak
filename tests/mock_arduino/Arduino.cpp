#include "Arduino.h"

unsigned long mock_millis = 0;
unsigned long mock_micros = 0;
void (*mock_isr_callback)(void) = nullptr;
uint16_t mock_analog_value = 0;