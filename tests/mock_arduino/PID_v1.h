/**
 * @file PID_v1.h
 * @brief Intercepted mock file for the Br3tt PID engine library.
 */

#ifndef PID_V1_H
#define PID_V1_H

#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0

class PID {
  private:
    int _mode;
    double* _input;
    double* _output;
    double* _setpoint;
    double _min_limit;
    double _max_limit;

  public:
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int Direction) :
      _mode(MANUAL),
      _input(Input),
      _output(Output),
      _setpoint(Setpoint),
      _min_limit(0.0),
      _max_limit(255.0) {}

    void SetOutputLimits(double min, double max) {
        _min_limit = min;
        _max_limit = max;
    }

    void SetMode(int mode) {
        _mode = mode;
    }

    int GetMode() const {
        return _mode;
    }

    bool Compute() {
        if (_mode == MANUAL) {
            return false;
        }

        // Simulate basic linear negative feedback loop logic for unit tests
        if (*_input < *_setpoint) {
            *_output += 2.0; 
        } else if (*_input > *_setpoint) {
            *_output -= 2.0; 
        }

        if (*_output > _max_limit) *_output = _max_limit;
        if (*_output < _min_limit) *_output = _min_limit;

        return true;
    }
};

#endif // PID_V1_H