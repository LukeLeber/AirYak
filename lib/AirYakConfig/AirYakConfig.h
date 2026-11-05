/**
 * @file AirYakConfig.h
 * @brief Global project configuration.
 * @author Luke Leber
 * @version 1.0.0
 */
#ifndef AIRYAK_CONFIG_H
#define AIRYAK_CONFIG_H

// #define AIRYAK_DEBUG 1

#ifdef AIRYAK_DEBUG
  /**
   * @brief Initializes the hardware serial interface for debug logging.
   * @details Sets the baud rate to 115200 bps to ensure high-speed, non-blocking telemetry.
   */
  #define AIRYAK_DEBUG_START()      (void)(Serial.begin(115200))
  #define AIRYAK_DEBUG_PRINT(...)   (void)(Serial.print(__VA_ARGS__))
  #define AIRYAK_DEBUG_PRINTLN(...) (void)(Serial.println(__VA_ARGS__))
  #define AIRYAK_DEBUG_METRIC(label, val, unit) \
    do { \
      Serial.print(F(label)); \
      Serial.print(val); \
      Serial.println(F(unit)); \
    } while(0)
#else

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_START()                  ((void)0)
  
  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_PRINT(...)               ((void)0)

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_PRINTLN(...)             ((void)0)

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_METRIC(label, val, unit) ((void)0)
#endif

#endif // AIRYAK_CONFIG_H