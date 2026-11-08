/**
 * @file AirYakConfig.h
 * @brief Global project configuration.
 * @author Luke Leber
 * @version 1.0.0
 */
#ifndef AIRYAK_CONFIG_H
#define AIRYAK_CONFIG_H

// There are several debugging helper macros that are critical to properly
// tuning the ECU. These macros are used throughout the AirYak project to
// capture critical events in the tachometer, choke, throttle, and rudder
// subsystems. By using the logging output, any performance issues should be
// able to be diagnosed.
//
// If the AIRYAK_DEBUG build flag is not provided, the debugging macros are
// no-ops that are completely optimized away by the compiler.
#ifdef AIRYAK_DEBUG

  #include <Arduino.h>

  /**
   * @brief Initializes the hardware serial interface for debug logging.
   * @details Sets the baud rate to 115200 bps to ensure high-speed, non-blocking telemetry.
   */
  #define AIRYAK_DEBUG_START() \
    do { \
      Serial.begin(115200); \
      delay(100); \
      while(Serial.available() > 0) { Serial.read(); } \
    } while(0)

  /**
   * @brief Prints a message or value to the serial debug stream without a newline.
   * @param[in] ... Variadic argument representing the item(s) to be printed.
   */
  #define AIRYAK_DEBUG_PRINT(...) (void)(Serial.print(__VA_ARGS__))

  /**
   * @brief Prints a message or value to the serial debug stream followed by a newline.
   * @param[in] ... Variadic argument representing the item(s) to be printed.
   */
  #define AIRYAK_DEBUG_PRINTLN(...) (void)(Serial.println(__VA_ARGS__))

  /**
   * @brief Outputs a formatted metric with a flash-memory string label and unit to minimize SRAM usage.
   * @param[in] label Constant string literal label describing the metric.
   * @param[in] val The dynamic data value or sensor variable to monitor.
   * @param[in] unit Constant string literal representing the unit of measurement.
   */
  #define AIRYAK_DEBUG_METRIC(label, val, unit) \
    do { \
      Serial.print(F(label " ")); \
      Serial.print(val); \
      Serial.println(F(" " unit)); \
    } while(0)

#else

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_START() ((void)0)
  
  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_PRINT(...) ((void)0)

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_PRINTLN(...) ((void)0)

  /**
   * @brief No-op fallback when debugging is disabled.
   */
  #define AIRYAK_DEBUG_METRIC(label, val, unit) ((void)0)

#endif

#endif // AIRYAK_CONFIG_H