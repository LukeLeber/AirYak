/**
 * @file PID_v1.h
 * @brief Host-side mock definitions for the Brett Beauregard Arduino PID library.
 * @author Luke Leber
 * @version 1.0.0
 * 
 * Emulates PID controller mode transitions, output clamping, and deterministic
 * negative-feedback stepping for native Linux unit testing without hardware timers.
 */

#ifndef AIRYAK_MOCK_PID_V1_H
#define AIRYAK_MOCK_PID_V1_H

// --- PID Controller Operating Modes ---
#define MANUAL 0        /**< Controller disabled; output is driven manually by caller. */
#define AUTOMATIC 1     /**< Controller active; Compute() updates output toward setpoint. */

// --- PID Action Directions ---
#define DIRECT 0        /**< Positive error produces a positive output adjustment. */
#define REVERSE 1       /**< Positive error produces a negative output adjustment. */

// --- PID Proportional Calculation Modes ---
#ifndef P_ON_M
#define P_ON_M 0        /**< Proportional-on-Measurement mode flag. */
#endif
#ifndef P_ON_E
#define P_ON_E 1        /**< Proportional-on-Error mode flag. */
#endif

/**
 * @class PID
 * @brief Deterministic mock implementation of the PID_v1 closed-loop controller.
 */
class PID {
  private:
    int _mode;            /**< Active operating mode (MANUAL or AUTOMATIC). */
    double* _input;       /**< Pointer to the process variable (measured RPM). */
    double* _output;      /**< Pointer to the control variable (throttle angle). */
    double* _setpoint;    /**< Pointer to the target setpoint (desired RPM). */
    double _min_limit;    /**< Lower output saturation bound. */
    double _max_limit;    /**< Upper output saturation bound. */

  public:
    /**
     * @brief Constructs a mock PID controller with explicit proportional calculation mode.
     * @param[in] Input Pointer to the measured process variable.
     * @param[in,out] Output Pointer to the computed control output variable.
     * @param[in] Setpoint Pointer to the target setpoint variable.
     * @param[in] Kp Proportional gain (unused in deterministic mock).
     * @param[in] Ki Integral gain (unused in deterministic mock).
     * @param[in] Kd Derivative gain (unused in deterministic mock).
     * @param[in] POn Proportional calculation mode (P_ON_E or P_ON_M).
     * @param[in] Direction Control action direction (DIRECT or REVERSE).
     */
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int Direction) :
      _mode(MANUAL),
      _input(Input),
      _output(Output),
      _setpoint(Setpoint),
      _min_limit(0.0),
      _max_limit(255.0) {
        (void)Kp;
        (void)Ki;
        (void)Kd;
        (void)POn;
        (void)Direction;
    }

    /**
     * @brief Constructs a mock PID controller defaulting to Proportional-on-Error (P_ON_E).
     */
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int Direction) :
      PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, Direction) {}

    /**
     * @brief No-op stub for configuring the PID evaluation period in milliseconds.
     * @param[in] NewSampleTime Sample interval in milliseconds.
     */
    void SetSampleTime(int NewSampleTime) {
      (void)NewSampleTime;
    }

    /**
     * @brief Configures the minimum and maximum saturation bounds for the control output.
     * @param[in] min Lower output clamp bound.
     * @param[in] max Upper output clamp bound.
     */
    void SetOutputLimits(double min, double max) {
      _min_limit = min;
      _max_limit = max;
    }

    /**
     * @brief Switches the controller between MANUAL and AUTOMATIC operating modes.
     * @param[in] mode Target operating mode (MANUAL or AUTOMATIC).
     */
    void SetMode(int mode) {
      _mode = mode;
    }

    /**
     * @brief Retrieves the current operating mode of the controller.
     * @return int Current mode (MANUAL or AUTOMATIC).
     */
    int GetMode(void) const {
      return _mode;
    }

    /**
     * @brief Executes a deterministic feedback step when in AUTOMATIC mode.
     * 
     * Steps the output by +2.0 when input is below setpoint, or -2.0 when input
     * exceeds setpoint, then clamps the result to [_min_limit, _max_limit].
     * 
     * @return true If the controller is in AUTOMATIC mode and updated the output.
     * @return false If the controller is in MANUAL mode (no calculation performed).
     */
    bool Compute(void) {
      if (_mode == MANUAL) {
        return false;
      }

      if (*_input < *_setpoint) {
        *_output += 2.0;
      } else if (*_input > *_setpoint) {
        *_output -= 2.0;
      }

      if (*_output > _max_limit) {
        *_output = _max_limit;
      } else if (*_output < _min_limit) {
        *_output = _min_limit;
      }

      return true;
    }
};

#endif // AIRYAK_MOCK_PID_V1_H