#ifndef MOCK_UTIL_ATOMIC_H
#define MOCK_UTIL_ATOMIC_H

// Simulate the AVR architecture macros cleanly on Linux host environments
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(type) for(int _init = 1; _init; _init = 0)

#endif