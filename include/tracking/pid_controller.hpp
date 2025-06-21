#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
public:
    PIDController(double Kp, double Ki, double Kd, double max_output, double min_output);

    double compute(double setpoint, double current_value, double dt);

private:
    double Kp_, Ki_, Kd_;
    double prev_error_;
    double integral_;
    double max_output_;
    double min_output_;
};

#endif // PID_CONTROLLER_HPP
