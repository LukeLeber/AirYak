/**
 * @file atomic.h
 * @brief Host-side mock definitions for AVR <util/atomic.h> critical section macros.
 * @author Luke Leber
 * @version 1.0.0
 */
#ifndef AIRYAK_MOCK_UTIL_ATOMIC_H
#define AIRYAK_MOCK_UTIL_ATOMIC_H

// This file emulates several AVR architecture macros on Linux host
// envirionments. On AVR chips, the ATOMIC_RESTORESTATE and ATOMIC_BLOCK macros
// work together to provide highly performant thread safety.

#define ATOMIC_RESTORESTATE
#define ATOMIC_BLOCK(type) if (true)

#endif // AIRYAK_MOCK_UTIL_ATOMIC_H
