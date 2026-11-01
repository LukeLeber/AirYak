/**
 * @file AirYakConfig.h
 * @brief Global project configuration.
 * @author Luke Leber
 * @version 1.0.0
 */
#ifndef AIRYAK_CONFIG_H
#define AIRYAK_CONFIG_H

#define AIRYAK_DEBUG 1

#ifdef AIRYAK_DEBUG
  #define AIRYAK_DEBUG_START()  (void)(Serial.begin(9600))
  #define AIRYAK_DEBUG_PRINT(...)   (void)(Serial.print(__VA_ARGS__))
  #define AIRYAK_DEBUG_PRINTLN(...) (void)(Serial.println(__VA_ARGS__))
  #define AIRYAK_DEBUG_METRIC(label, val, unit) \
    do { \
      Serial.print(label); \
      Serial.print(val); \
      Serial.println(unit); \
    } while(0)
#else
  #define AIRYAK_DEBUG_START() ((void)0);
  #define AIRYAK_DEBUG_PRINT(...)   ((void)0)
  #define AIRYAK_DEBUG_PRINTLN(...) ((void)0)
  #define AIRYAK_DEBUG_METRIC(label, val, unit) ((void)0)
#endif

#endif // AIRYAK_CONFIG_H